#define _GNU_SOURCE

/*
 * Intel KMS Color Demo v4.2
 *
 * Target environment:
 *   - Ubuntu 24.04
 *   - kernel 6.6 / 6.12 LTS
 *   - Intel iGPU / i915 (or any DRM/KMS driver that exposes the same
 *     standard CRTC color-management properties)
 *
 * This program demonstrates the standard upstream DRM/KMS CRTC color pipeline:
 *   - DEGAMMA_LUT
 *   - CTM
 *   - GAMMA_LUT
 *   - SHARPNESS_STRENGTH (optional, only if the driver exposes it)
 *
 * High-level behavior:
 *   1. Find an active CRTC.
 *   2. Optionally save the original scanout state (framebuffer + mode) if we
 *      are going to replace it with our own full-screen test frame.
 *   3. Save the original CRTC color state.
 *   4. Run a sequence of color transforms, staying on each step for N seconds.
 *   5. Restore the original color state.
 *   6. Optionally restore the original scanout state.
 *
 * Operational note:
 *   - v4.2 defaults to COLOR-ONLY mode, i.e. it does NOT draw or present the
 *     full-screen demo framebuffer unless --frame is explicitly requested.
 *   - Scanout restore is guarded by default: before restoring framebuffer/mode,
 *     the program checks that the current scanout state still matches the demo
 *     state that this program wrote. Use --force to bypass that guard.
 *
 * IMPORTANT NOTE ABOUT RESTORING DRM BLOB PROPERTIES
 * --------------------------------------------------
 * The current CRTC property value for DEGAMMA_LUT / CTM / GAMMA_LUT is NOT the
 * raw LUT or matrix payload itself. It is a kernel property-blob ID.
 *
 * Reusing a foreign blob ID directly for restore is fragile because:
 *   - we do not own the original blob,
 *   - we did not copy its payload,
 *   - another KMS client may have created it.
 *
 * Therefore this program, at startup, reads the original blob payload using
 * drmModeGetPropertyBlob() and then clones it into a NEW backup blob owned by
 * this process. Later, restore_original_color() restores from our own backup
 * blob IDs, not from foreign ones.
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm/drm.h>
#include <drm/drm_mode.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* DRM/KMS property IDs that we care about on the target CRTC. */
struct crtc_prop_ids {
    uint32_t degamma_lut;
    uint32_t degamma_lut_size;
    uint32_t ctm;
    uint32_t gamma_lut;
    uint32_t gamma_lut_size;
    uint32_t sharpness_strength;
};

/*
 * Original color state saved at startup.
 *
 * For blob properties, these are backup blob IDs created by THIS process after
 * cloning the original payloads. These backup blobs are safe to use later in a
 * restore path because we own them.
 */
struct saved_state {
    uint32_t restore_degamma_blob;
    uint32_t restore_ctm_blob;
    uint32_t restore_gamma_blob;
    uint64_t sharpness_strength;
    bool valid;
};

/*
 * Original scanout state for the selected CRTC.
 *
 * This represents the original framebuffer + mode routing of the display. It
 * is only relevant if the program is asked to present its own full-screen demo
 * framebuffer using --frame.
 */
struct saved_crtc_state {
    uint32_t crtc_id;
    uint32_t buffer_id;
    int x;
    int y;
    drmModeModeInfo mode;
    bool mode_valid;
    uint32_t connector_id;
    bool valid;
};

/* Simple dumb framebuffer used for the optional full-screen demo image. */
struct dumb_fb {
    uint32_t fb_id;
    uint32_t handle;
    uint32_t width;
    uint32_t height;
    uint32_t pitch;
    uint64_t size;
    void *map;
};

/*
 * Demo runtime context.
 */
struct demo_ctx {
    int fd;
    char device[256];
    uint32_t crtc_id;
    uint32_t connector_id;
    int duration_sec;
    int loops;
    bool verbose;
    bool paint_frame;          /* false by default in v4.2 */
    bool force_restore_crtc;   /* force scanout restore even if the guard fails */

    struct crtc_prop_ids prop_ids;
    uint32_t degamma_size;
    uint32_t gamma_size;
    bool has_degamma;
    bool has_ctm;
    bool has_gamma;
    bool has_sharpness;

    struct saved_state saved;
    struct saved_crtc_state saved_crtc;
    struct dumb_fb demo_fb;
};

/*
 * A single demo step.
 *
 * Each step is expressed in terms of conceptual color operations; at runtime
 * those are translated into actual DRM/KMS objects:
 *   - optional DEGAMMA_LUT blob
 *   - optional CTM blob
 *   - optional GAMMA_LUT blob
 *   - optional SHARPNESS_STRENGTH scalar value
 */
struct demo_step {
    const char *name;
    bool use_degamma;
    bool use_ctm;
    bool use_gamma;
    bool use_sharpness;
    double degamma_gamma;
    double gamma_gamma;
    double brightness;
    double contrast;
    double gains[3];
    double ctm[9];
    uint64_t sharpness_strength;
};

static volatile sig_atomic_t g_stop = 0;
static struct demo_ctx *g_ctx = NULL;

static void on_signal(int signo)
{
    (void)signo;
    g_stop = 1;
}

static void log_errno(const char *msg)
{
    fprintf(stderr, "%s: %s\n", msg, strerror(errno));
}

static double clamp01(double x)
{
    if (x < 0.0)
        return 0.0;
    if (x > 1.0)
        return 1.0;
    return x;
}

/*
 * Convert a floating-point CTM element into the signed-magnitude S31.32 fixed-
 * point representation used by DRM CTM blobs.
 *
 * DRM stores each matrix element as:
 *   - 1 sign bit
 *   - 31 integer bits
 *   - 32 fractional bits
 *
 * Conceptually:
 *   fixed = round(abs(v) * 2^32)
 *   if v < 0: set the sign bit
 */
static uint64_t s3132_from_double(double v)
{
    bool neg = v < 0.0;
    double abs_v = fabs(v);

    if (abs_v > 2147483647.0)
        abs_v = 2147483647.0;

    uint64_t mag = (uint64_t) llround(abs_v * 4294967296.0);
    return neg ? (1ULL << 63) | mag : mag;
}

static void sleep_interruptible(int sec)
{
    struct timespec ts = { .tv_sec = sec, .tv_nsec = 0 };
    while (!g_stop && nanosleep(&ts, &ts) == -1 && errno == EINTR) {
    }
}

/* 3x3 identity matrix. */
static void identity3(double m[9])
{
    memset(m, 0, sizeof(double) * 9);
    m[0] = m[4] = m[8] = 1.0;
}

/* Standard 3x3 matrix multiplication: out = a * b. */
static void multiply3(const double a[9], const double b[9], double out[9])
{
    double tmp[9] = {0};

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            for (int k = 0; k < 3; ++k)
                tmp[r * 3 + c] += a[r * 3 + k] * b[k * 3 + c];
        }
    }

    memcpy(out, tmp, sizeof(tmp));
}

/*
 * Build a saturation matrix from a scalar saturation parameter s.
 *
 * Formula:
 *   out = L + s * (I - L)
 * where:
 *   - I is the 3x3 identity matrix
 *   - L is the luminance projection matrix built from Rec.709 weights
 *     (lr, lg, lb)
 *
 * Interpretation:
 *   - s = 1.0  -> identity (no saturation change)
 *   - s = 0.0  -> full grayscale / desaturation
 *   - s > 1.0  -> saturation boost
 *
 * Rec.709 luminance weights are used because they are a practical and common
 * approximation for RGB luminance contribution:
 *   Y = 0.2126 * R + 0.7152 * G + 0.0722 * B
 */
static void saturation_matrix(double s, double out[9])
{
    const double lr = 0.2126;
    const double lg = 0.7152;
    const double lb = 0.0722;

    out[0] = lr * (1.0 - s) + s;
    out[1] = lg * (1.0 - s);
    out[2] = lb * (1.0 - s);

    out[3] = lr * (1.0 - s);
    out[4] = lg * (1.0 - s) + s;
    out[5] = lb * (1.0 - s);

    out[6] = lr * (1.0 - s);
    out[7] = lg * (1.0 - s);
    out[8] = lb * (1.0 - s) + s;
}

/*
 * Build a grayscale conversion matrix.
 *
 * This is the s = 0 special case of the saturation matrix above:
 * every output channel becomes the same luminance estimate
 *   Y = lr * R + lg * G + lb * B
 */
static void grayscale_matrix(double out[9])
{
    const double lr = 0.2126;
    const double lg = 0.7152;
    const double lb = 0.0722;

    out[0] = out[3] = out[6] = lr;
    out[1] = out[4] = out[7] = lg;
    out[2] = out[5] = out[8] = lb;
}

/*
 * Build an approximate hue-rotation matrix.
 *
 * This is a practical RGB-space artistic transform commonly used in CSS/SVG-
 * style implementations. It is useful for visual demos but is NOT intended to
 * be a strict color-science hue rotation in a uniform perceptual space.
 *
 * The matrix is derived from a rotation around the luminance axis using a
 * closed-form approximation in RGB space. The angle is in degrees.
 */
static void hue_rotate_matrix(double deg, double out[9])
{
    double rad = deg * M_PI / 180.0;
    double c = cos(rad);
    double s = sin(rad);

    out[0] = 0.213 + 0.787 * c - 0.213 * s;
    out[1] = 0.715 - 0.715 * c - 0.715 * s;
    out[2] = 0.072 - 0.072 * c + 0.928 * s;

    out[3] = 0.213 - 0.213 * c + 0.143 * s;
    out[4] = 0.715 + 0.285 * c + 0.140 * s;
    out[5] = 0.072 - 0.072 * c - 0.283 * s;

    out[6] = 0.213 - 0.213 * c - 0.787 * s;
    out[7] = 0.715 - 0.715 * c + 0.715 * s;
    out[8] = 0.072 + 0.928 * c + 0.072 * s;
}

/*
 * Build a simple diagonal RGB gain matrix.
 *
 * This is intentionally simple and practical for demo purposes. It scales each
 * output primary independently:
 *   R' = r_gain * R
 *   G' = g_gain * G
 *   B' = b_gain * B
 *
 * This approximates "warm" / "cool" visual modes but is not a physically
 * correct black-body or chromatic-adaptation transform.
 */
static void temperature_matrix(double r_gain, double g_gain, double b_gain, double out[9])
{
    memset(out, 0, sizeof(double) * 9);
    out[0] = r_gain;
    out[4] = g_gain;
    out[8] = b_gain;
}

static uint32_t pack_xrgb8888(uint8_t r, uint8_t g, uint8_t b)
{
    return 0xff000000u | ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

/* Convert HSV to RGB for the optional colorful demo pattern. */
static void hsv_to_rgb(double h, double s, double v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    double c = v * s;
    double x = c * (1.0 - fabs(fmod(h / 60.0, 2.0) - 1.0));
    double m = v - c;
    double rr = 0.0, gg = 0.0, bb = 0.0;

    if (h < 60.0) {
        rr = c; gg = x; bb = 0.0;
    } else if (h < 120.0) {
        rr = x; gg = c; bb = 0.0;
    } else if (h < 180.0) {
        rr = 0.0; gg = c; bb = x;
    } else if (h < 240.0) {
        rr = 0.0; gg = x; bb = c;
    } else if (h < 300.0) {
        rr = x; gg = 0.0; bb = c;
    } else {
        rr = c; gg = 0.0; bb = x;
    }

    *r = (uint8_t) llround((rr + m) * 255.0);
    *g = (uint8_t) llround((gg + m) * 255.0);
    *b = (uint8_t) llround((bb + m) * 255.0);
}

/*
 * Paint a colorful full-screen demo pattern.
 *
 * This is only used when --frame is explicitly requested. The goal is not
 * photometric accuracy; the pattern is designed to make brightness / contrast /
 * saturation / hue / sharpness changes easy to observe.
 */
static void fill_demo_pattern(struct dumb_fb *fb)
{
    uint32_t *pixels = (uint32_t *) fb->map;
    int w = (int) fb->width;
    int h = (int) fb->height;

    if (!pixels)
        return;

    for (int y = 0; y < h; ++y) {
        double yf = (double)y / (double)(h > 1 ? h - 1 : 1);

        for (int x = 0; x < w; ++x) {
            double xf = (double)x / (double)(w > 1 ? w - 1 : 1);
            uint8_t r = 0, g = 0, b = 0;

            if (y < h / 3) {
                double hue = xf * 360.0;
                hsv_to_rgb(hue, 0.95, 0.95, &r, &g, &b);
                double fade = 0.70 + 0.30 * (1.0 - yf * 3.0);
                r = (uint8_t) llround(r * fade);
                g = (uint8_t) llround(g * fade);
                b = (uint8_t) llround(b * fade);
            } else if (y < (2 * h) / 3) {
                double local_y = (double)(y - h / 3) /
                                 (double)(((h / 3) > 1) ? ((h / 3) - 1) : 1);

                if (x < w / 2) {
                    double gx = xf * 2.0;
                    r = (uint8_t) llround(255.0 * gx);
                    g = (uint8_t) llround(255.0 * (1.0 - fabs(gx - 0.5) * 2.0));
                    b = (uint8_t) llround(255.0 * (1.0 - gx));
                } else {
                    uint8_t gray = (uint8_t) llround(
                        ((double)(x - w / 2) /
                         (double)(((w / 2) > 1) ? ((w / 2) - 1) : 1)) * 255.0);
                    r = g = b = gray;
                }

                double vignette = 0.85 + 0.15 * sin(local_y * M_PI);
                r = (uint8_t) llround(r * vignette);
                g = (uint8_t) llround(g * vignette);
                b = (uint8_t) llround(b * vignette);
            } else {
                int tile = 32;
                bool checker = ((x / tile) ^ (y / tile)) & 1;

                if (checker) {
                    r = 40; g = 40; b = 40;
                } else {
                    r = 220; g = 220; b = 220;
                }

                int cx = w / 2;
                int cy = (5 * h) / 6;
                int dx = x - cx;
                int dy = y - cy;
                double dist = sqrt((double)(dx * dx + dy * dy));
                double radius = (double)(h / 7);

                if (dist < radius) {
                    double hue = fmod((atan2((double)dy, (double)dx) * 180.0 / M_PI) + 360.0,
                                      360.0);
                    hsv_to_rgb(hue, dist / radius, 1.0, &r, &g, &b);
                }
            }

            pixels[y * (fb->pitch / 4) + x] = pack_xrgb8888(r, g, b);
        }
    }

    int bar_h = h / 12;
    static const uint32_t bars[8] = {
        0xffffffffu, 0xffffff00u, 0xff00ffffu, 0xff00ff00u,
        0xffff00ffu, 0xffff0000u, 0xff0000ffu, 0xff000000u
    };

    for (int y = h - bar_h; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int idx = (x * 8) / (w > 0 ? w : 1);
            if (idx < 0)
                idx = 0;
            if (idx > 7)
                idx = 7;
            pixels[y * (fb->pitch / 4) + x] = bars[idx];
        }
    }
}

static int create_demo_fb(int fd, uint32_t width, uint32_t height, struct dumb_fb *fb)
{
    struct drm_mode_create_dumb create = {0};
    struct drm_mode_map_dumb map = {0};

    create.width = width;
    create.height = height;
    create.bpp = 32;

    if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) != 0)
        return -1;

    fb->width = width;
    fb->height = height;
    fb->pitch = create.pitch;
    fb->size = create.size;
    fb->handle = create.handle;

    if (drmModeAddFB(fd, width, height, 24, 32, fb->pitch, fb->handle, &fb->fb_id) != 0)
        return -1;

    map.handle = fb->handle;
    if (ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &map) != 0)
        return -1;

    fb->map = mmap(NULL, fb->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, map.offset);
    if (fb->map == MAP_FAILED) {
        fb->map = NULL;
        return -1;
    }

    memset(fb->map, 0, fb->size);
    fill_demo_pattern(fb);
    return 0;
}

static void destroy_demo_fb(int fd, struct dumb_fb *fb)
{
    if (!fb)
        return;

    if (fb->map)
        munmap(fb->map, fb->size);
    fb->map = NULL;

    if (fb->fb_id)
        drmModeRmFB(fd, fb->fb_id);

    if (fb->handle) {
        struct drm_mode_destroy_dumb destroy = {0};
        destroy.handle = fb->handle;
        ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
    }

    memset(fb, 0, sizeof(*fb));
}

/*
 * Build a DRM LUT blob for DEGAMMA_LUT or GAMMA_LUT.
 *
 * The transformation applied per normalized sample x in [0, 1] is:
 *
 *   1. Contrast + brightness around mid-gray:
 *        y0 = clamp( (x - 0.5) * contrast + 0.5 + brightness )
 *
 *      - contrast = 1.0 keeps slope unchanged
 *      - contrast > 1.0 steepens the curve around 0.5
 *      - brightness adds a constant offset in normalized space
 *
 *   2. Gamma shaping:
 *        y1 = pow(y0, gamma_exp)
 *
 *      - gamma_exp < 1.0 brightens shadows / midtones
 *      - gamma_exp > 1.0 darkens shadows / midtones
 *
 *   3. Per-channel gain:
 *        R = clamp(y1 * gains[0])
 *        G = clamp(y1 * gains[1])
 *        B = clamp(y1 * gains[2])
 *
 * Finally, each channel is quantized to 16-bit DRM LUT entries.
 */
static int create_lut_blob(int fd, uint32_t size, double gamma_exp,
                           double brightness, double contrast,
                           const double gains[3], uint32_t *blob_id)
{
    struct drm_color_lut *lut;

    if (!size) {
        *blob_id = 0;
        return 0;
    }

    lut = calloc(size, sizeof(*lut));
    if (!lut) {
        errno = ENOMEM;
        return -1;
    }

    for (uint32_t i = 0; i < size; ++i) {
        double x = (size == 1) ? 0.0 : (double)i / (double)(size - 1);
        double y = clamp01((x - 0.5) * contrast + 0.5 + brightness);

        if (gamma_exp > 0.0 && fabs(gamma_exp - 1.0) > 1e-9)
            y = pow(y, gamma_exp);

        double r = clamp01(y * gains[0]);
        double g = clamp01(y * gains[1]);
        double b = clamp01(y * gains[2]);

        lut[i].red = (uint16_t) llround(r * 65535.0);
        lut[i].green = (uint16_t) llround(g * 65535.0);
        lut[i].blue = (uint16_t) llround(b * 65535.0);
        lut[i].reserved = 0;
    }

    int ret = drmModeCreatePropertyBlob(fd, lut, size * sizeof(*lut), blob_id);
    free(lut);
    return ret == 0 ? 0 : -1;
}

static int create_ctm_blob(int fd, const double m[9], uint32_t *blob_id)
{
    struct drm_color_ctm ctm;

    for (int i = 0; i < 9; ++i)
        ctm.matrix[i] = s3132_from_double(m[i]);

    return drmModeCreatePropertyBlob(fd, &ctm, sizeof(ctm), blob_id) == 0 ? 0 : -1;
}

static int clone_property_blob_or_zero(int fd, uint64_t source_blob_id, uint32_t *backup_blob_id)
{
    drmModePropertyBlobRes *blob;

    if (!backup_blob_id) {
        errno = EINVAL;
        return -1;
    }

    *backup_blob_id = 0;

    if (source_blob_id == 0)
        return 0;

    blob = drmModeGetPropertyBlob(fd, (uint32_t) source_blob_id);
    if (!blob)
        return -1;

    int ret = drmModeCreatePropertyBlob(fd, blob->data, blob->length, backup_blob_id);
    drmModeFreePropertyBlob(blob);
    return ret == 0 ? 0 : -1;
}

static int blobs_equal_or_zero(int fd, uint64_t a_id, uint32_t b_id)
{
    drmModePropertyBlobRes *a = NULL;
    drmModePropertyBlobRes *b = NULL;
    int equal = 0;

    if (a_id == 0 && b_id == 0)
        return 1;
    if ((a_id == 0) != (b_id == 0))
        return 0;

    a = drmModeGetPropertyBlob(fd, (uint32_t) a_id);
    b = drmModeGetPropertyBlob(fd, b_id);
    if (!a || !b)
        goto out;

    if (a->length == b->length && memcmp(a->data, b->data, a->length) == 0)
        equal = 1;

out:
    if (a)
        drmModeFreePropertyBlob(a);
    if (b)
        drmModeFreePropertyBlob(b);
    return equal;
}

static void destroy_saved_color_backups(struct demo_ctx *ctx)
{
    if (!ctx)
        return;

    if (ctx->saved.restore_degamma_blob) {
        drmModeDestroyPropertyBlob(ctx->fd, ctx->saved.restore_degamma_blob);
        ctx->saved.restore_degamma_blob = 0;
    }

    if (ctx->saved.restore_ctm_blob) {
        drmModeDestroyPropertyBlob(ctx->fd, ctx->saved.restore_ctm_blob);
        ctx->saved.restore_ctm_blob = 0;
    }

    if (ctx->saved.restore_gamma_blob) {
        drmModeDestroyPropertyBlob(ctx->fd, ctx->saved.restore_gamma_blob);
        ctx->saved.restore_gamma_blob = 0;
    }
}

static int find_prop_on_object(int fd, uint32_t object_id, uint32_t object_type,
                               const char *name, uint32_t *prop_id, uint64_t *value)
{
    drmModeObjectProperties *props;

    props = drmModeObjectGetProperties(fd, object_id, object_type);
    if (!props)
        return -1;

    int ret = -1;

    for (uint32_t i = 0; i < props->count_props; ++i) {
        drmModePropertyRes *p = drmModeGetProperty(fd, props->props[i]);
        if (!p)
            continue;

        if (strcmp(p->name, name) == 0) {
            *prop_id = p->prop_id;
            if (value)
                *value = props->prop_values[i];
            ret = 0;
            drmModeFreeProperty(p);
            break;
        }

        drmModeFreeProperty(p);
    }

    drmModeFreeObjectProperties(props);
    return ret;
}

static int load_crtc_properties(struct demo_ctx *ctx)
{
    uint64_t value = 0;

    memset(&ctx->prop_ids, 0, sizeof(ctx->prop_ids));
    memset(&ctx->saved, 0, sizeof(ctx->saved));
    ctx->degamma_size = 0;
    ctx->gamma_size = 0;

    if (find_prop_on_object(ctx->fd, ctx->crtc_id, DRM_MODE_OBJECT_CRTC,
                            "DEGAMMA_LUT", &ctx->prop_ids.degamma_lut, &value) == 0) {
        ctx->has_degamma = true;
        if (clone_property_blob_or_zero(ctx->fd, value, &ctx->saved.restore_degamma_blob) != 0) {
            log_errno("Failed to clone the original DEGAMMA_LUT blob");
            return -1;
        }
    }

    if (find_prop_on_object(ctx->fd, ctx->crtc_id, DRM_MODE_OBJECT_CRTC,
                            "DEGAMMA_LUT_SIZE", &ctx->prop_ids.degamma_lut_size, &value) == 0)
        ctx->degamma_size = (uint32_t) value;

    if (find_prop_on_object(ctx->fd, ctx->crtc_id, DRM_MODE_OBJECT_CRTC,
                            "CTM", &ctx->prop_ids.ctm, &value) == 0) {
        ctx->has_ctm = true;
        if (clone_property_blob_or_zero(ctx->fd, value, &ctx->saved.restore_ctm_blob) != 0) {
            log_errno("Failed to clone the original CTM blob");
            return -1;
        }
    }

    if (find_prop_on_object(ctx->fd, ctx->crtc_id, DRM_MODE_OBJECT_CRTC,
                            "GAMMA_LUT", &ctx->prop_ids.gamma_lut, &value) == 0) {
        ctx->has_gamma = true;
        if (clone_property_blob_or_zero(ctx->fd, value, &ctx->saved.restore_gamma_blob) != 0) {
            log_errno("Failed to clone the original GAMMA_LUT blob");
            return -1;
        }
    }

    if (find_prop_on_object(ctx->fd, ctx->crtc_id, DRM_MODE_OBJECT_CRTC,
                            "GAMMA_LUT_SIZE", &ctx->prop_ids.gamma_lut_size, &value) == 0)
        ctx->gamma_size = (uint32_t) value;

    if (find_prop_on_object(ctx->fd, ctx->crtc_id, DRM_MODE_OBJECT_CRTC,
                            "SHARPNESS_STRENGTH", &ctx->prop_ids.sharpness_strength, &value) == 0) {
        ctx->has_sharpness = true;
        ctx->saved.sharpness_strength = value;
    }

    ctx->saved.valid = ctx->has_degamma || ctx->has_ctm || ctx->has_gamma || ctx->has_sharpness;
    return ctx->saved.valid ? 0 : -1;
}

static int atomic_set_properties(struct demo_ctx *ctx,
                                 uint32_t degamma_blob,
                                 uint32_t ctm_blob,
                                 uint32_t gamma_blob,
                                 uint64_t sharpness_strength,
                                 bool test_only)
{
    drmModeAtomicReq *req = drmModeAtomicAlloc();
    if (!req) {
        errno = ENOMEM;
        return -1;
    }

    if (ctx->has_degamma && ctx->prop_ids.degamma_lut) {
        if (drmModeAtomicAddProperty(req, ctx->crtc_id, ctx->prop_ids.degamma_lut, degamma_blob) <= 0)
            goto fail;
    }

    if (ctx->has_ctm && ctx->prop_ids.ctm) {
        if (drmModeAtomicAddProperty(req, ctx->crtc_id, ctx->prop_ids.ctm, ctm_blob) <= 0)
            goto fail;
    }

    if (ctx->has_gamma && ctx->prop_ids.gamma_lut) {
        if (drmModeAtomicAddProperty(req, ctx->crtc_id, ctx->prop_ids.gamma_lut, gamma_blob) <= 0)
            goto fail;
    }

    if (ctx->has_sharpness && ctx->prop_ids.sharpness_strength) {
        if (drmModeAtomicAddProperty(req, ctx->crtc_id,
                                     ctx->prop_ids.sharpness_strength,
                                     sharpness_strength) <= 0)
            goto fail;
    }

    uint32_t flags = test_only ? DRM_MODE_ATOMIC_TEST_ONLY : 0;
    if (drmModeAtomicCommit(ctx->fd, req, flags, NULL) != 0)
        goto fail;

    drmModeAtomicFree(req);
    return 0;

fail:
    drmModeAtomicFree(req);
    return -1;
}

static int apply_step(struct demo_ctx *ctx, const struct demo_step *step)
{
    uint32_t degamma_blob = 0;
    uint32_t ctm_blob = 0;
    uint32_t gamma_blob = 0;
    const double unit[3] = {1.0, 1.0, 1.0};
    int ret = 0;

    if (step->use_degamma && ctx->has_degamma && ctx->degamma_size) {
        if (create_lut_blob(ctx->fd, ctx->degamma_size,
                            step->degamma_gamma,
                            0.0, 1.0,
                            unit,
                            &degamma_blob) != 0) {
            log_errno("Failed to create DEGAMMA_LUT blob");
            return -1;
        }
    }

    if (step->use_ctm && ctx->has_ctm) {
        if (create_ctm_blob(ctx->fd, step->ctm, &ctm_blob) != 0) {
            log_errno("Failed to create CTM blob");
            ret = -1;
            goto out;
        }
    }

    if (step->use_gamma && ctx->has_gamma && ctx->gamma_size) {
        if (create_lut_blob(ctx->fd, ctx->gamma_size,
                            step->gamma_gamma,
                            step->brightness,
                            step->contrast,
                            step->gains,
                            &gamma_blob) != 0) {
            log_errno("Failed to create GAMMA_LUT blob");
            ret = -1;
            goto out;
        }
    }

    if (atomic_set_properties(ctx,
                              degamma_blob,
                              ctm_blob,
                              gamma_blob,
                              step->use_sharpness ? step->sharpness_strength : 0,
                              true) != 0) {
        fprintf(stderr, "Atomic TEST_ONLY failed for step: %s\n", step->name);
        ret = -1;
        goto out;
    }

    if (atomic_set_properties(ctx,
                              degamma_blob,
                              ctm_blob,
                              gamma_blob,
                              step->use_sharpness ? step->sharpness_strength : 0,
                              false) != 0) {
        fprintf(stderr, "Atomic commit failed for step: %s\n", step->name);
        ret = -1;
        goto out;
    }

    if (ctx->verbose) {
        printf("Applied step: %s", step->name);
        if (step->use_sharpness && ctx->has_sharpness)
            printf(" (sharpness=%llu)", (unsigned long long) step->sharpness_strength);
        printf("\n");
        fflush(stdout);
    }

out:
    if (degamma_blob)
        drmModeDestroyPropertyBlob(ctx->fd, degamma_blob);
    if (ctm_blob)
        drmModeDestroyPropertyBlob(ctx->fd, ctm_blob);
    if (gamma_blob)
        drmModeDestroyPropertyBlob(ctx->fd, gamma_blob);
    return ret;
}

static int restore_original_color(struct demo_ctx *ctx)
{
    if (!ctx || !ctx->saved.valid)
        return 0;

    if (ctx->verbose) {
        printf("Restoring original CRTC color state...\n");
        fflush(stdout);
    }

    if (atomic_set_properties(ctx,
                              ctx->saved.restore_degamma_blob,
                              ctx->saved.restore_ctm_blob,
                              ctx->saved.restore_gamma_blob,
                              ctx->saved.sharpness_strength,
                              true) != 0) {
        fprintf(stderr, "Atomic TEST_ONLY failed while restoring the original color state\n");
        return -1;
    }

    if (atomic_set_properties(ctx,
                              ctx->saved.restore_degamma_blob,
                              ctx->saved.restore_ctm_blob,
                              ctx->saved.restore_gamma_blob,
                              ctx->saved.sharpness_strength,
                              false) != 0) {
        fprintf(stderr, "Atomic commit failed while restoring the original color state\n");
        return -1;
    }

    return 0;
}

static int save_current_crtc(struct demo_ctx *ctx)
{
    drmModeCrtc *crtc = drmModeGetCrtc(ctx->fd, ctx->crtc_id);
    if (!crtc)
        return -1;

    ctx->saved_crtc.crtc_id = crtc->crtc_id;
    ctx->saved_crtc.buffer_id = crtc->buffer_id;
    ctx->saved_crtc.x = crtc->x;
    ctx->saved_crtc.y = crtc->y;
    ctx->saved_crtc.mode_valid = crtc->mode_valid;

    if (crtc->mode_valid)
        ctx->saved_crtc.mode = crtc->mode;

    ctx->saved_crtc.connector_id = ctx->connector_id;
    ctx->saved_crtc.valid = true;

    drmModeFreeCrtc(crtc);
    return 0;
}

static int present_demo_frame(struct demo_ctx *ctx)
{
    if (!ctx->paint_frame)
        return 0;

    if (!ctx->saved_crtc.valid || !ctx->saved_crtc.mode_valid) {
        fprintf(stderr, "Current CRTC mode is not valid; cannot present the demo framebuffer\n");
        return -1;
    }

    if (create_demo_fb(ctx->fd,
                       ctx->saved_crtc.mode.hdisplay,
                       ctx->saved_crtc.mode.vdisplay,
                       &ctx->demo_fb) != 0) {
        log_errno("Failed to create the demo framebuffer");
        return -1;
    }

    if (drmModeSetCrtc(ctx->fd,
                       ctx->crtc_id,
                       ctx->demo_fb.fb_id,
                       0, 0,
                       &ctx->connector_id,
                       1,
                       &ctx->saved_crtc.mode) != 0) {
        log_errno("Failed to present the demo framebuffer");
        destroy_demo_fb(ctx->fd, &ctx->demo_fb);
        return -1;
    }

    if (ctx->verbose) {
        printf("Presented a full-screen demo frame (%ux%u).\n",
               ctx->demo_fb.width,
               ctx->demo_fb.height);
        fflush(stdout);
    }

    return 0;
}

static int current_scanout_matches_demo(struct demo_ctx *ctx)
{
    drmModeCrtc *crtc;
    int match = 0;

    if (!ctx->paint_frame)
        return 1;

    if (!ctx->demo_fb.fb_id)
        return 0;

    crtc = drmModeGetCrtc(ctx->fd, ctx->crtc_id);
    if (!crtc)
        return 0;

    if (crtc->buffer_id == ctx->demo_fb.fb_id &&
        crtc->mode_valid == ctx->saved_crtc.mode_valid &&
        crtc->x == 0 && crtc->y == 0) {
        if (!ctx->saved_crtc.mode_valid ||
            memcmp(&crtc->mode, &ctx->saved_crtc.mode, sizeof(drmModeModeInfo)) == 0)
            match = 1;
    }

    drmModeFreeCrtc(crtc);
    return match;
}

/*
 * Restore the original framebuffer + mode on the selected CRTC.
 *
 * Default behavior in v4.2:
 *   - If --frame was not used, scanout restore is skipped entirely.
 *   - If --frame was used, the program checks that the current scanout still
 *     matches the demo scanout written by this program. This is the default,
 *     safer behavior.
 *   - Use --force to bypass that guard and restore unconditionally.
 */
static int restore_original_crtc(struct demo_ctx *ctx)
{
    if (!ctx)
        return 0;

    if (!ctx->paint_frame)
        return 0;

    if (!ctx->saved_crtc.valid)
        return 0;

    if (!ctx->force_restore_crtc && !current_scanout_matches_demo(ctx)) {
        if (ctx->verbose) {
            printf("Skipping original scanout restore because the current scanout state no longer matches the demo state. Use --force to override.\n");
            fflush(stdout);
        }
        return 0;
    }

    if (ctx->saved_crtc.mode_valid) {
        if (drmModeSetCrtc(ctx->fd,
                           ctx->saved_crtc.crtc_id,
                           ctx->saved_crtc.buffer_id,
                           ctx->saved_crtc.x,
                           ctx->saved_crtc.y,
                           &ctx->saved_crtc.connector_id,
                           1,
                           &ctx->saved_crtc.mode) != 0) {
            log_errno("Failed to restore the original CRTC framebuffer/mode");
            return -1;
        }
    }

    if (ctx->verbose) {
        printf("Restored the original CRTC framebuffer/mode.\n");
        fflush(stdout);
    }

    destroy_demo_fb(ctx->fd, &ctx->demo_fb);
    return 0;
}

static void cleanup_all(void)
{
    if (!g_ctx)
        return;

    (void) restore_original_color(g_ctx);
    (void) restore_original_crtc(g_ctx);
    destroy_saved_color_backups(g_ctx);
}

static int pick_active_crtc(struct demo_ctx *ctx)
{
    drmModeRes *res = drmModeGetResources(ctx->fd);
    if (!res)
        return -1;

    if (ctx->crtc_id) {
        drmModeCrtc *crtc = drmModeGetCrtc(ctx->fd, ctx->crtc_id);
        if (crtc) {
            drmModeFreeCrtc(crtc);
            drmModeFreeResources(res);
            return 0;
        }
        drmModeFreeResources(res);
        errno = ENOENT;
        return -1;
    }

    for (int i = 0; i < res->count_connectors; ++i) {
        drmModeConnector *conn = drmModeGetConnector(ctx->fd, res->connectors[i]);
        if (!conn)
            continue;

        if (conn->connection != DRM_MODE_CONNECTED || conn->count_modes == 0) {
            drmModeFreeConnector(conn);
            continue;
        }

        drmModeEncoder *enc = NULL;
        if (conn->encoder_id)
            enc = drmModeGetEncoder(ctx->fd, conn->encoder_id);

        if (!enc) {
            for (int j = 0; j < conn->count_encoders; ++j) {
                enc = drmModeGetEncoder(ctx->fd, conn->encoders[j]);
                if (enc)
                    break;
            }
        }

        if (enc && enc->crtc_id) {
            drmModeCrtc *crtc = drmModeGetCrtc(ctx->fd, enc->crtc_id);
            if (crtc && crtc->mode_valid) {
                ctx->crtc_id = enc->crtc_id;
                ctx->connector_id = conn->connector_id;
                drmModeFreeCrtc(crtc);
                drmModeFreeEncoder(enc);
                drmModeFreeConnector(conn);
                drmModeFreeResources(res);
                return 0;
            }
            if (crtc)
                drmModeFreeCrtc(crtc);
        }

        if (enc)
            drmModeFreeEncoder(enc);
        drmModeFreeConnector(conn);
    }

    for (int i = 0; i < res->count_crtcs; ++i) {
        drmModeCrtc *crtc = drmModeGetCrtc(ctx->fd, res->crtcs[i]);
        if (!crtc)
            continue;

        if (crtc->mode_valid) {
            ctx->crtc_id = crtc->crtc_id;
            drmModeFreeCrtc(crtc);
            drmModeFreeResources(res);
            return 0;
        }

        drmModeFreeCrtc(crtc);
    }

    drmModeFreeResources(res);
    errno = ENOENT;
    return -1;
}

static int open_drm_device(const char *path)
{
    int fd = open(path, O_RDWR | O_CLOEXEC);
    if (fd < 0)
        return -1;

    if (drmSetClientCap(fd, DRM_CLIENT_CAP_ATOMIC, 1) != 0) {
        close(fd);
        errno = ENOTSUP;
        return -1;
    }

    (void) drmSetClientCap(fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);
    return fd;
}

static int build_steps(struct demo_ctx *ctx, struct demo_step **out_steps, size_t *out_count)
{
    static struct demo_step steps[12];
    size_t n = 0;
    double m[9], m2[9], m3[9];

    memset(steps, 0, sizeof(steps));

    identity3(steps[n].ctm);
    steps[n].name = "Step 0 - Identity / passthrough";
    steps[n].use_gamma = ctx->has_gamma;
    steps[n].gamma_gamma = 1.0;
    steps[n].brightness = 0.0;
    steps[n].contrast = 1.0;
    steps[n].gains[0] = steps[n].gains[1] = steps[n].gains[2] = 1.0;
    n++;

    if (ctx->has_gamma) {
        steps[n].name = "Step 1 - Digital brightness boost (+0.08)";
        steps[n].use_gamma = true;
        steps[n].gamma_gamma = 1.0;
        steps[n].brightness = 0.08;
        steps[n].contrast = 1.0;
        steps[n].gains[0] = steps[n].gains[1] = steps[n].gains[2] = 1.0;
        n++;

        steps[n].name = "Step 2 - Contrast boost (1.25x)";
        steps[n].use_gamma = true;
        steps[n].gamma_gamma = 1.0;
        steps[n].brightness = 0.0;
        steps[n].contrast = 1.25;
        steps[n].gains[0] = steps[n].gains[1] = steps[n].gains[2] = 1.0;
        n++;

        steps[n].name = "Step 3 - Gamma brighten (0.80)";
        steps[n].use_gamma = true;
        steps[n].gamma_gamma = 0.80;
        steps[n].brightness = 0.0;
        steps[n].contrast = 1.0;
        steps[n].gains[0] = steps[n].gains[1] = steps[n].gains[2] = 1.0;
        n++;

        steps[n].name = "Step 4 - Gamma darken (1.25)";
        steps[n].use_gamma = true;
        steps[n].gamma_gamma = 1.25;
        steps[n].brightness = 0.0;
        steps[n].contrast = 1.0;
        steps[n].gains[0] = steps[n].gains[1] = steps[n].gains[2] = 1.0;
        n++;
    }

    if (ctx->has_ctm) {
        saturation_matrix(1.6, steps[n].ctm);
        steps[n].name = "Step 5 - Saturation boost (1.6x)";
        steps[n].use_ctm = true;
        steps[n].use_gamma = ctx->has_gamma;
        steps[n].gamma_gamma = 1.0;
        steps[n].brightness = 0.0;
        steps[n].contrast = 1.0;
        steps[n].gains[0] = steps[n].gains[1] = steps[n].gains[2] = 1.0;
        n++;

        grayscale_matrix(steps[n].ctm);
        steps[n].name = "Step 6 - Grayscale / desaturation";
        steps[n].use_ctm = true;
        steps[n].use_gamma = ctx->has_gamma;
        steps[n].gamma_gamma = 1.0;
        steps[n].brightness = 0.0;
        steps[n].contrast = 1.0;
        steps[n].gains[0] = steps[n].gains[1] = steps[n].gains[2] = 1.0;
        n++;

        hue_rotate_matrix(35.0, steps[n].ctm);
        steps[n].name = "Step 7 - Hue rotate (+35 degrees)";
        steps[n].use_ctm = true;
        steps[n].use_gamma = ctx->has_gamma;
        steps[n].gamma_gamma = 1.0;
        steps[n].brightness = 0.0;
        steps[n].contrast = 1.0;
        steps[n].gains[0] = steps[n].gains[1] = steps[n].gains[2] = 1.0;
        n++;

        temperature_matrix(1.08, 1.00, 0.90, steps[n].ctm);
        steps[n].name = "Step 8 - Warm color mode";
        steps[n].use_ctm = true;
        steps[n].use_gamma = ctx->has_gamma;
        steps[n].gamma_gamma = 1.0;
        steps[n].brightness = 0.0;
        steps[n].contrast = 1.0;
        steps[n].gains[0] = steps[n].gains[1] = steps[n].gains[2] = 1.0;
        n++;

        saturation_matrix(1.25, m);
        temperature_matrix(1.05, 1.00, 0.95, m2);
        multiply3(m2, m, m3);
        memcpy(steps[n].ctm, m3, sizeof(m3));
        steps[n].name = "Step 9 - Vivid mode";
        steps[n].use_ctm = true;
        steps[n].use_gamma = ctx->has_gamma;
        steps[n].gamma_gamma = 0.92;
        steps[n].brightness = 0.01;
        steps[n].contrast = 1.08;
        steps[n].gains[0] = steps[n].gains[1] = steps[n].gains[2] = 1.0;
        n++;
    }

    if (ctx->has_sharpness) {
        identity3(steps[n].ctm);
        steps[n].name = "Step 10 - Sharpness medium (128)";
        steps[n].use_gamma = ctx->has_gamma;
        steps[n].gamma_gamma = 1.0;
        steps[n].brightness = 0.0;
        steps[n].contrast = 1.0;
        steps[n].gains[0] = steps[n].gains[1] = steps[n].gains[2] = 1.0;
        steps[n].use_sharpness = true;
        steps[n].sharpness_strength = 128;
        n++;

        identity3(steps[n].ctm);
        steps[n].name = "Step 11 - Sharpness high (220)";
        steps[n].use_gamma = ctx->has_gamma;
        steps[n].gamma_gamma = 1.0;
        steps[n].brightness = 0.0;
        steps[n].contrast = 1.0;
        steps[n].gains[0] = steps[n].gains[1] = steps[n].gains[2] = 1.0;
        steps[n].use_sharpness = true;
        steps[n].sharpness_strength = 220;
        n++;
    }

    if (n == 0)
        return -1;

    *out_steps = steps;
    *out_count = n;
    return 0;
}

static void print_usage(const char *prog)
{
    printf(
        "Usage: %s [options]\n\n"
        "Intel i915 / DRM-KMS color demo for Ubuntu 24.04 with kernel 6.6 or 6.12 LTS.\n"
        "The program uses standard upstream CRTC color-management properties\n"
        "(DEGAMMA_LUT / CTM / GAMMA_LUT) and, if present on newer kernels,\n"
        "the SHARPNESS_STRENGTH property.\n\n"
        "Options:\n"
        "  -d, --device <path>    DRM device node, default: /dev/dri/card0\n"
        "  -c, --crtc <id>       Force a CRTC id, default: auto-pick an active CRTC\n"
        "  -t, --time <sec>      Seconds per step, default: 5\n"
        "  -n, --loops <n>       Number of loops, default: 1, 0 = infinite\n"
        "  --frame               Present the built-in full-screen demo image\n"
        "  --no-frame            Explicitly disable the full-screen demo image (default)\n"
        "  --force               Force framebuffer/mode restore even if the guarded scanout check fails\n"
        "  -v, --verbose         Verbose logs\n"
        "  -h, --help            Show this help\n",
        prog);
}

static int parse_args(int argc, char **argv, struct demo_ctx *ctx)
{
    static const struct option long_opts[] = {
        {"device", required_argument, 0, 'd'},
        {"crtc", required_argument, 0, 'c'},
        {"time", required_argument, 0, 't'},
        {"loops", required_argument, 0, 'n'},
        {"frame", no_argument, 0, 1000},
        {"no-frame", no_argument, 0, 1001},
        {"force", no_argument, 0, 1002},
        {"verbose", no_argument, 0, 'v'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };

    strcpy(ctx->device, "/dev/dri/card0");
    ctx->duration_sec = 5;
    ctx->loops = 1;
    ctx->verbose = false;
    ctx->paint_frame = false;        /* v4.2 default: do not draw/present */
    ctx->force_restore_crtc = false; /* v4.2 default: guarded scanout restore */

    int opt;
    while ((opt = getopt_long(argc, argv, "d:c:t:n:vh", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'd':
            snprintf(ctx->device, sizeof(ctx->device), "%s", optarg);
            break;
        case 'c':
            ctx->crtc_id = (uint32_t) strtoul(optarg, NULL, 0);
            break;
        case 't':
            ctx->duration_sec = atoi(optarg);
            if (ctx->duration_sec <= 0)
                ctx->duration_sec = 5;
            break;
        case 'n':
            ctx->loops = atoi(optarg);
            if (ctx->loops < 0)
                ctx->loops = 1;
            break;
        case 'v':
            ctx->verbose = true;
            break;
        case 'h':
            print_usage(argv[0]);
            return 1;
        case 1000:
            ctx->paint_frame = true;
            break;
        case 1001:
            ctx->paint_frame = false;
            break;
        case 1002:
            ctx->force_restore_crtc = true;
            break;
        default:
            return -1;
        }
    }

    return 0;
}

int main(int argc, char **argv)
{
    struct demo_ctx ctx;
    memset(&ctx, 0, sizeof(ctx));

    int prc = parse_args(argc, argv, &ctx);
    if (prc != 0)
        return prc > 0 ? 0 : 1;

    ctx.fd = open_drm_device(ctx.device);
    if (ctx.fd < 0) {
        log_errno("Failed to open the DRM device");
        return 1;
    }

    if (pick_active_crtc(&ctx) != 0) {
        log_errno("Failed to find an active CRTC");
        close(ctx.fd);
        return 1;
    }

    if (ctx.paint_frame) {
        if (save_current_crtc(&ctx) != 0) {
            log_errno("Failed to save the current CRTC state");
            close(ctx.fd);
            return 1;
        }
    }

    if (load_crtc_properties(&ctx) != 0) {
        fprintf(stderr, "No standard CRTC color-management property was exposed on the selected CRTC.\n");
        close(ctx.fd);
        return 1;
    }

    if (ctx.verbose) {
        printf("Device: %s\n", ctx.device);
        printf("CRTC: %u\n", ctx.crtc_id);
        printf("Connector: %u\n", ctx.connector_id);
        printf("Capabilities: degamma=%s(size=%u), ctm=%s, gamma=%s(size=%u), sharpness=%s\n",
               ctx.has_degamma ? "yes" : "no", ctx.degamma_size,
               ctx.has_ctm ? "yes" : "no",
               ctx.has_gamma ? "yes" : "no", ctx.gamma_size,
               ctx.has_sharpness ? "yes" : "no");
        printf("Saved restore blobs: degamma=%u, ctm=%u, gamma=%u, sharpness=%llu\n",
               ctx.saved.restore_degamma_blob,
               ctx.saved.restore_ctm_blob,
               ctx.saved.restore_gamma_blob,
               (unsigned long long) ctx.saved.sharpness_strength);
        printf("Frame presentation: %s\n", ctx.paint_frame ? "enabled (--frame)" : "disabled (default color-only mode)");
        printf("CRTC restore guard: %s\n", ctx.force_restore_crtc ? "disabled (--force)" : "enabled (default guarded restore)");
        fflush(stdout);
    }

    if (present_demo_frame(&ctx) != 0) {
        destroy_saved_color_backups(&ctx);
        close(ctx.fd);
        return 1;
    }

    struct demo_step *steps = NULL;
    size_t step_count = 0;
    if (build_steps(&ctx, &steps, &step_count) != 0) {
        fprintf(stderr, "Failed to build the demo sequence.\n");
        (void) restore_original_crtc(&ctx);
        destroy_saved_color_backups(&ctx);
        close(ctx.fd);
        return 1;
    }

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    g_ctx = &ctx;
    atexit(cleanup_all);

    if (ctx.verbose) {
        printf("Starting the demo: %zu steps, %d second(s) each, loops=%d\n",
               step_count, ctx.duration_sec, ctx.loops);
        fflush(stdout);
    }

    int loop = 0;
    while (!g_stop && (ctx.loops == 0 || loop < ctx.loops)) {
        for (size_t i = 0; i < step_count && !g_stop; ++i) {
            printf("%s\n", steps[i].name);
            fflush(stdout);

            if (apply_step(&ctx, &steps[i]) != 0) {
                fprintf(stderr, "Stopping the demo because a step failed.\n");
                g_stop = 1;
                break;
            }

            sleep_interruptible(ctx.duration_sec);
        }
        loop++;
    }

    (void) restore_original_color(&ctx);
    (void) restore_original_crtc(&ctx);
    destroy_saved_color_backups(&ctx);

    close(ctx.fd);
    g_ctx = NULL;
    return 0;
}

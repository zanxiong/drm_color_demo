# Intel KMS Color Demo

This package contains a **production-oriented demo utility** built on top of
**libdrm + DRM/KMS** for Intel i915.

It demonstrates the **upstream-standard CRTC color-management pipeline**
available on Ubuntu 24.04 with **kernel 6.6 LTS / 6.12 LTS**:

- `DEGAMMA_LUT`
- `CTM`
- `GAMMA_LUT`

In addition, the code is **future-proofed** for the newer
`SHARPNESS_STRENGTH` CRTC property. If that property is present on a newer
kernel/platform, the demo automatically adds sharpness steps. On **Ubuntu 24.04
with kernel 6.6 / 6.12 on Alder Lake / Raptor Lake**, this property is normally
**not present**, so the sharpness steps are skipped automatically.

---

## Common behavior

The tool:

1. Opens a DRM primary node (`/dev/dri/card0` by default).
2. Selects an active CRTC automatically unless one is forced with `--crtc`.
3. Reads the original CRTC color state.
4. Runs a sequence of color transforms using DRM/KMS properties.
5. Restores the original color state when finished.
6. Optionally presents its own full-screen demo image and restores the
   original framebuffer/mode afterward.

The program is designed to be useful both for:

- **factory / validation / bring-up** style testing on a text console
- **careful manual experiments** on a system where you control the display
  pipeline ownership

---

## Default operating mode in v4.2

By default, v4.2 runs in **color-only mode**:

- it **does not** draw or present the full-screen demo framebuffer
- it **does** apply and later restore the color pipeline state

If you want the built-in full-screen colorful test image, enable it explicitly
with:

```bash
--frame
```

This means the default behavior is more conservative than earlier revisions and
is better suited to setups where you want to test color transforms against the
current screen content.

---

## Guarded CRTC restore (default)

When `--frame` is used, the program temporarily replaces the active scanout
framebuffer/mode with its own demo framebuffer.

By default, v4.2 performs a **guarded** framebuffer/mode restore:

- before restoring the original scanout state, it checks whether the current
  scanout still matches the demo state written by this program
- if the current scanout no longer matches, restore is skipped

This behavior is intended to avoid rolling back someone else's newer changes in
more complex environments.

To bypass the guard and restore the original framebuffer/mode unconditionally,
use:

```bash
--force
```

---

## Important operational note

If you use `--frame`, the tool temporarily replaces the active CRTC framebuffer
with its own full-screen demo frame. For that reason it is **strongly
recommended** to run it on a **text VT/TTY** or on a dedicated production test
environment without a running desktop compositor on the same output.

Typical safe workflow:

```bash
sudo systemctl isolate multi-user.target
sudo ./intel_kms_color_demo -v --frame
sudo systemctl isolate graphical.target
```

If you run this while GNOME / KDE / Mutter / KWin / Weston is actively driving
the same display pipeline, the result depends on how that compositor reacts to
the CRTC handover.

---

## Demo sequence

The program auto-detects the properties exposed by the selected CRTC and builds
the sequence dynamically.

Typical steps on Ubuntu 24.04 + kernel 6.6 / 6.12:

1. Identity / passthrough
2. Digital brightness boost
3. Contrast boost
4. Gamma brighten
5. Gamma darken
6. Saturation boost
7. Grayscale / desaturation
8. Hue rotation
9. Warm color mode
10. Vivid mode

If `SHARPNESS_STRENGTH` is present on a newer kernel/platform, it also adds:

11. Sharpness medium
12. Sharpness high

---

## Color transform implementation notes

### 1. LUT-based transforms

For `DEGAMMA_LUT` and `GAMMA_LUT`, each normalized sample `x` in `[0, 1]`
undergoes the following computation:

1. Contrast + brightness around mid-gray:

```text
y0 = clamp((x - 0.5) * contrast + 0.5 + brightness)
```

2. Gamma shaping:

```text
y1 = pow(y0, gamma_exp)
```

3. Per-channel gain:

```text
R = clamp(y1 * gains[0])
G = clamp(y1 * gains[1])
B = clamp(y1 * gains[2])
```

The final values are quantized into 16-bit DRM LUT entries.

### 2. CTM-based transforms

The 3x3 CTM is used for:

- saturation adjustment
- grayscale conversion
- hue rotation (approximate RGB-space artistic matrix)
- simple warm/cool style RGB gain matrices
- combined vivid mode (matrix composition)

### 3. Fixed-point representation

DRM CTM elements are encoded in **signed-magnitude S31.32** fixed-point format.
The code converts floating-point matrix elements into that representation before
creating the CTM blob.

---

## Restore of original color state

The original `DEGAMMA_LUT`, `CTM`, and `GAMMA_LUT` property values are not used
as raw values directly, because those property values are only **blob IDs**.

Instead, at startup the program:

1. reads the original blob ID from the CRTC property value
2. fetches the blob payload with `drmModeGetPropertyBlob()`
3. creates a new backup blob containing the same payload
4. stores the backup blob ID
5. restores from that backup blob later

This makes `restore_original_color()` safe and self-contained.

---

## Dependencies

Install on Ubuntu 24.04:

```bash
sudo apt update
sudo apt install -y build-essential pkg-config libdrm-dev
```

---

## Build

```bash
make
```

---

## Run

### Default run (color-only mode)

```bash
sudo ./intel_kms_color_demo -v
```

### Explicitly present the built-in full-screen demo image

```bash
sudo ./intel_kms_color_demo --frame -v
```

### 5 seconds per step (default)

```bash
sudo ./intel_kms_color_demo -t 5 -v
```

### Infinite loop

```bash
sudo ./intel_kms_color_demo -n 0 -v
```

### Force a device node

```bash
sudo ./intel_kms_color_demo -d /dev/dri/card0 -v
```

### Force a CRTC id

```bash
sudo ./intel_kms_color_demo -c 74 -v
```

### Force framebuffer/mode restore even if the guarded scanout check fails

```bash
sudo ./intel_kms_color_demo --frame --force -v
```

### Explicitly disable frame presentation (same as default)

```bash
sudo ./intel_kms_color_demo --no-frame -v
```

---

## Expected verbose output example

```text
Device: /dev/dri/card0
CRTC: 74
Connector: 95
Capabilities: degamma=yes(size=33), ctm=yes, gamma=yes(size=1024), sharpness=no
Saved restore blobs: degamma=101, ctm=102, gamma=103, sharpness=0
Frame presentation: disabled (default color-only mode)
CRTC restore guard: enabled (default guarded restore)
Starting the demo: 10 steps, 5 second(s) each, loops=1
Step 0 - Identity / passthrough
Step 1 - Digital brightness boost (+0.08)
...
Restoring original CRTC color state...
```

---

## Production integration suggestions

### 1. Use a dedicated test target
For factory / line validation, run the demo in a dedicated boot target or a
 dedicated production image.

### 2. Fix the object selection on stable hardware
If the platform is fixed, pass a known `cardX` and `CRTC_ID` from your test
harness.

### 3. Keep the test image deterministic when using `--frame`
The utility renders a colorful full-screen frame (HSV gradient + grayscale ramp
+ checkerboard + color bars). If needed, you can replace `fill_demo_pattern()`
with a house-style validation image.

### 4. Log the exposed capabilities
For production, it is often useful to log the exposed properties and their
sizes per machine. The current program already prints the key capabilities in
verbose mode.

---

## Files

- `intel_kms_color_demo.c` — main source
- `Makefile` — build file
- `README.md` — documentation

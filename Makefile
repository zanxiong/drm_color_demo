CC ?= gcc
CFLAGS ?= -O2 -g -Wall -Wextra -Wpedantic -std=c11
PKG_CFLAGS := $(shell pkg-config --cflags libdrm)
PKG_LIBS := $(shell pkg-config --libs libdrm)

TARGET := intel_kms_color_demo
SRC := intel_kms_color_demo.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(PKG_CFLAGS) -o $@ $< $(PKG_LIBS) -lm

clean:
	rm -f $(TARGET)

.PHONY: all clean

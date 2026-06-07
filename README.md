# lan80xx-tools

Userspace bring-up utilities for Microchip LAN80XX-family PHYs
(LAN8023, LAN8044, ...) talking directly over a Linux `spidev` node:
no MEPA, no MESA, no kernel PHY driver in the way.

Currently ships one tool:

- `lan8023_probe`: read and decode the chip's identity, fuse,
  strap, MCU boot, and POST1 BIST registers. First sanity check when
  bringing up a board: tells you whether the SPI bus is alive, which
  LAN80XX SKU answered, and whether straps/fuses look sane.

## Build

```sh
meson setup build
meson compile -C build
```

```sh
meson setup build-static -Dstatic=true
meson compile -C build-static
```

## Run

```sh
lan8023_probe /dev/spidev0.0
lan8023_probe /dev/spidev0.0 -c 1 -s 1000000 -p 2
lan8023_probe /var/run/spid.sock        # shared access via lan80xx-spid proxy

```

`-c` selects the channel (0..3), `-s` the SCLK frequency in Hz,
`-p` the number of dummy turn-around bytes between address and data
phases (needed at high SCLK; 0 is fine at < 1 MHz).

## License

BSD-3-Clause. Copyright 2026 Free Mobile — Vincent Jardin.

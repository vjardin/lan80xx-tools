# lan80xx-tools

Userspace bring-up utilities for Microchip LAN80XX-family PHYs
(LAN8023, LAN8044, ...) talking directly over a Linux `spidev` node:
no MEPA, no MESA, no kernel PHY driver in the way.

Ships two tools:

- `lan8023_probe`: read and decode the chip's identity, fuse,
  strap, MCU boot, and POST1 BIST registers. First sanity check when
  bringing up a board: tells you whether the SPI bus is alive, which
  LAN80XX SKU answered, and whether straps/fuses look sane.
- `lan80xx_bringup`: replay a recorded register sequence to actually
  bring a PHY up — no MEPA, MESA, or kernel PHY framework at runtime.
  Sequences are recorded once from a known-good MEPA bring-up (see
  `scripts/events2seq.py`).

Both tools take either a raw `spidev` node (exclusive access) or a
`lan80xx-spid` proxy Unix socket (shared access); the transport is
auto-selected from the device argument.

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

```sh
lan80xx_bringup /dev/spidev0.0 bringup.seq -s 5000000 -v
lan80xx_bringup /var/run/spid.sock bringup.seq        # via lan80xx-spid proxy
```

`-s` sets the SCLK frequency in Hz, `-p` the padding (raw spidev
only), `-v` echoes each opcode as it replays.

## License

BSD-3-Clause. Copyright 2026 Free Mobile — Vincent Jardin.

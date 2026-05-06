# SPDX-License-Identifier: BSD-3-Clause
# Copyright 2026 Free Mobile — Vincent Jardin

project('lan80xx-tools', 'c',
  version: '0.1.0',
  license: 'BSD-3-Clause',
  meson_version: '>= 1.1.0',
  default_options: [
    'c_std=c17',
    'warning_level=3',
    'buildtype=release',
  ],
)

cc = meson.get_compiler('c')

if not cc.has_header('linux/spi/spidev.h')
  error('linux/spi/spidev.h not found: install linux-libc-dev')
endif

link_args = []
if get_option('static')
  link_args += '-static'
endif

executable('lan8023_probe',
  'src/lan8023_probe.c',
  link_args: link_args,
  install: true,
)

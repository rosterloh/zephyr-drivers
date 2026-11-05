# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(uzlib)
zephyr_include_directories(.)
zephyr_library_sources(adler32.c crc32.c tinflate.c tinfzlib.c)

# OpenDisplay receiver

`CONFIG_OPENDISPLAY` makes a Zephyr device an [OpenDisplay](https://opendisplay.org)
display: Home Assistant's `opendisplay` integration, or
[py-opendisplay](https://pypi.org/project/py-opendisplay/) (tested against 7.18.1), finds it,
reads its config and pushes images to the `zephyr,display` device. Protocol 2.2. The code is
written from the published protocol description; nothing is taken from the GPL reference
firmware.

API: `include/zephyr/opendisplay/opendisplay.h`. Sample: `samples/opendisplay`.

## Transports

| Kconfig | What |
| --- | --- |
| `CONFIG_OPENDISPLAY_BLE` | GATT service and characteristic 0x2446 (write + notify), manufacturer data 0x2446 in the advertisement with the 4-bit loop counter, battery and temperature from `opendisplay_set_status()`. One central at a time. Start with `opendisplay_ble_start()` after `bt_enable()`. |
| `CONFIG_OPENDISPLAY_LAN` | TCP `[len:2 LE][frame]` on WifiConfig `server_port` (default 2446). One client at a time: a new one evicts the old. Idle timeout 30 s. |
| `CONFIG_OPENDISPLAY_LAN_MDNS` | `_opendisplay._tcp` with TXT `mac`, `tls`, `fw`, `cm`, `pv`. Needs `MDNS_RESPONDER`, `DNS_SD`. |

The host-written config packets that the device cannot derive (WiFi, security, extended data)
persist through settings when `CONFIG_SETTINGS` is on. Everything else in READ_CONFIG comes
from the display's capabilities and Kconfig (`OPENDISPLAY_IC_TYPE`, `_MANUFACTURER_ID`,
`_BOARD_TYPE`, `_POWER_MODE`, `_FW_VERSION_*`).

## Images

| Kconfig | Transmission mode |
| --- | --- |
| always | Direct write (0x70-0x72): uncompressed 1 bpp rows, streamed into the display 8 rows at a time; no framebuffer. |
| `CONFIG_OPENDISPLAY_ZLIB` | Streaming zlib for direct write and PIPE, window up to 512 bytes (what hosts send). Uses the vendored uzlib (`lib/uzlib`, Zlib license). About 3 KB RAM. |
| `CONFIG_OPENDISPLAY_PIPE` | PIPE_WRITE (0x80-0x82): sequenced frames with a cumulative ACK every few. Off by default: hosts prefer it once advertised, and it has not been bench-tested. |

Only the bits that are built in are advertised in `transmission_modes`.

Supported display formats:

- `PIXEL_FORMAT_MONO01` and `MONO10`, horizontal or vertically tiled
  (`SCREEN_INFO_MONO_VTILED`), either bit order. Vertically tiled panels need a height that is a
  multiple of 8. E-paper (`SCREEN_INFO_EPD`) is blanked during the upload and refreshed by
  `display_blanking_off()` at END.
- Formats with 8 or more bits per pixel (RGB565, RGB888, ARGB8888, L8 and so on): each host
  pixel is written as all-zero or all-one bytes, so TFTs show the image in black and white.

The device always reports the monochrome color scheme. py-opendisplay 7.18.1 refuses scheme
100 (RGB565), and Zephyr's e-paper drivers are monochrome.

## Encryption

`CONFIG_OPENDISPLAY_SECURITY` follows the host-written SecurityConfig. When it is enabled with
a non-zero key:

- BLE: hosts must pass AUTHENTICATE (0x50, AES-CMAC challenge-response with a proof from the
  device) before any command but FIRMWARE_VERSION. Frames are then AES-128-CCM wrapped. The
  session ends on disconnect, on key change or after the configured timeout.
- LAN: only TLS 1.2 PSK on `server_port + 1` (identity `opendisplay`, PSK derived from the key);
  mDNS shows `tls=1`. Needs `NET_SOCKETS_SOCKOPT_TLS` and a PSK ciphersuite such as
  `MBEDTLS_CIPHERSUITE_TLS_PSK_WITH_AES_128_GCM_SHA256`.

Uses PSA crypto. `psa_crypto_init()` at boot needs a main stack of about 4 KB.

## Out of scope

LED, buzzer, NFC, touch and sensor commands; DFU, deep sleep and power off; colour planes
(BWR, BWRY, Spectra) and grey levels; partial refresh and etags; the ZIP (non-streaming)
transmission mode. Those commands get a NACK.

## Footprint

reTerminal E1001 (`reterminal_e1001/esp32s3/procpu`, 800x480 e-paper), sample with BLE and
zlib, from `rom_report` / `ram_report`: 6.9 KB flash (4.7 KB OpenDisplay, 2.2 KB uzlib) and
9.9 KB RAM, BLE thread stack and receive queue included. The image as a whole is 310 KB of
flash, almost all of it the Bluetooth stack and the Espressif HAL.

## Tests

`tests/subsys/opendisplay/` on native_sim and qemu_cortex_m3: config TLV against a
py-opendisplay serialized config, direct write into a capture display in every layout,
zlib and PIPE, LAN over loopback (framing, eviction, idle timeout, port moves, mDNS, TLS-PSK),
and encryption against vectors generated with py-opendisplay. BLE builds only.

`samples/opendisplay` (scenario `sample.opendisplay.lan`) uploads an image with py-opendisplay
over the host's sockets on native_sim. It is skipped when py-opendisplay is not installed:

```sh
pip install py-opendisplay==7.18.1
west twister -T samples/opendisplay -p native_sim
```

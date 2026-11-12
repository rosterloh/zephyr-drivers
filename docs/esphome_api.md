# ESPHome native API server

Lets Home Assistant discover a Zephyr device, read its sensors, control its
switches and buttons and show its camera through the ESPHome integration, without ESPHome's YAML or
code generation. The server speaks ESPHome's native API: TCP port 6053,
protobuf messages, optional Noise encryption, mDNS discovery. Home Assistant
sees an ordinary ESPHome device.

Sources: `subsys/net/lib/esphome_api/`, header
`include/zephyr/net/esphome_api.h`, sample `samples/net/esphome_api/`, tests
`tests/subsys/net/esphome_api/`.

## Quick start

```c
#include <zephyr/net/esphome_api.h>

static void relay_set(struct esphome_entity *e, bool on)
{
	gpio_pin_set_dt(&relay_gpio, on);
	esphome_api_publish_bool(e, on); /* confirm, or HA keeps the old state */
}

ESPHOME_SENSOR_DEFINE(temperature, .name = "Temperature", .unit = "°C",
		      .device_class = "temperature",
		      .state_class = ESPHOME_STATE_CLASS_MEASUREMENT, .accuracy_decimals = 1);
ESPHOME_SWITCH_DEFINE(relay, .name = "Relay", .on_switch = relay_set);

/* anywhere, from any thread (not an ISR) */
esphome_api_publish_float(&temperature, 21.5f);
```

```
CONFIG_NETWORKING=y
CONFIG_NET_SOCKETS=y
CONFIG_NET_TCP=y
CONFIG_ESPHOME_API=y
# discovery
CONFIG_NET_HOSTNAME_ENABLE=y
CONFIG_NET_HOSTNAME="my-device"
CONFIG_MDNS_RESPONDER=y
CONFIG_DNS_SD=y
```

The server thread starts at boot and listens once the interface is up. In
Home Assistant the device shows up under *Settings → Devices & services*
as a discovered ESPHome device; or add it by host name or IP.

The west manifest needs the `nanopb` module, and `protoc` must be installed at
build time (`pip install grpcio-tools`). Noise also needs `mbedtls` and
`tf-psa-crypto`.

## Entities

Each `ESPHOME_*_DEFINE(id, ...)` places a `struct esphome_entity` in an
iterable section; the server lists them in link order. `id` becomes the
`object_id`, which Home Assistant keys the entity on (the wire key is the
FNV-1 hash of it, as ESPHome computes it), so keep it stable across firmware
versions. Further fields are designated initializers:

| Field | Applies to | Meaning |
|---|---|---|
| `.name` | all | Display name; `object_id` when unset |
| `.icon` | all | e.g. `"mdi:thermometer"` |
| `.device_class` | all | Home Assistant device class |
| `.entity_category` | all | `ESPHOME_ENTITY_CATEGORY_CONFIG` / `_DIAGNOSTIC` |
| `.unit`, `.accuracy_decimals`, `.state_class` | sensor | Unit, shown decimals, `ESPHOME_STATE_CLASS_*` |
| `.on_switch` | switch | Called with the requested state |
| `.on_press` | button | Called on a press |
| `.get_image`, `.release_image` | camera | Lend a JPEG to send; told when it was sent |
| `.user_data` | all | Free for the application |

| Macro | Publish with |
|---|---|
| `ESPHOME_SENSOR_DEFINE` | `esphome_api_publish_float()` |
| `ESPHOME_BINARY_SENSOR_DEFINE` | `esphome_api_publish_bool()` |
| `ESPHOME_SWITCH_DEFINE` | `esphome_api_publish_bool()` (to confirm a command, or report a local change) |
| `ESPHOME_BUTTON_DEFINE` | — |
| `ESPHOME_CAMERA_DEFINE` | — (images on request) |

A camera image is sent when Home Assistant asks for one, single or stream:
the server calls `.get_image`, sends the JPEG in 768-byte chunks to that
client and calls `.release_image`. A stream request also gets one image; Home
Assistant asks again for the next, so a stream runs at the round-trip rate.

Publishing stores the state and sends it to every subscribed client; a client
that subscribes later gets the stored states first. Entities never published
are reported as "unknown". Callbacks run on the server thread with its lock
held; publishing from inside one is allowed, blocking in one stalls the
server.

## Kconfig

| Option | Default | Notes |
|---|---|---|
| `CONFIG_ESPHOME_API_PORT` | 6053 | |
| `CONFIG_ESPHOME_API_MAX_CLIENTS` | 2 | Home Assistant uses one; the ESPHome dashboard log viewer another |
| `CONFIG_ESPHOME_API_IDLE_TIMEOUT` | 90 s | HA pings every 20 s |
| `CONFIG_ESPHOME_API_RX_BUF_SIZE` | 256 | Largest accepted request frame |
| `CONFIG_ESPHOME_API_DEVICE_NAME` | `"zephyr"` | Only without `NET_HOSTNAME_ENABLE`; otherwise the hostname is the name |
| `CONFIG_ESPHOME_API_FRIENDLY_NAME` | `""` | Shown in HA; also the mDNS `friendly_name` |
| `CONFIG_ESPHOME_API_MANUFACTURER` | `"Zephyr"` | Model is `CONFIG_BOARD` |
| `CONFIG_ESPHOME_API_MDNS` | y | Needs `MDNS_RESPONDER` + `DNS_SD` |
| `CONFIG_ESPHOME_API_NOISE` | n | See below |
| `CONFIG_ESPHOME_API_NOISE_PSK` | `""` | Development key, base64 |
| `CONFIG_ESPHOME_API_STACK_SIZE` | 2048 / 4096 with Noise | Peak with Noise ~1.6 KB on Cortex-M3 |

With IPv4 and IPv6 both enabled the server listens on one IPv6 socket that also
takes IPv4 clients (`NET_IPV4_MAPPING_TO_IPV6`, implied). Turning that mapping
off leaves IPv4 only.

mDNS advertises `_esphomelib._tcp` named after the hostname, with the TXT keys
Home Assistant reads: `friendly_name`, `version`, `mac` (required for
discovery), `board`, `network`, and `api_encryption` with Noise.

## Encryption and keys

`CONFIG_ESPHOME_API_NOISE=y` encrypts every connection with
`Noise_NNpsk0_25519_ChaChaPoly_SHA256`, ESPHome's `api: encryption:`, on PSA
crypto. Plaintext clients are refused (aioesphomeapi reports "requires
encryption"), as is everyone until a key is set.

The key is 32 random bytes; Home Assistant asks for it base64-encoded, e.g.
from `openssl rand -base64 32`. Provision it with:

```c
uint8_t key[32]; /* from your provisioning path: shell, BLE, factory data, ... */
esphome_api_set_psk(key);
```

With `CONFIG_SETTINGS` the key is saved under `esphome_api/psk` and loaded by
the application's `settings_load()` at boot. `CONFIG_ESPHOME_API_NOISE_PSK`
compiles a key into the image and is for development only. Home Assistant
setting the key over the API (`NoiseEncryptionSetKeyRequest`) is not
implemented.

Cost on Cortex-M3: about 21 KB flash and 1.9 KB RAM for PSA crypto, 2.5 KB
flash for the server side.

## Protocol version

`subsys/net/lib/esphome_api/proto/` holds `api.proto` pinned to aioesphomeapi
v46.25.1 (API 1.19); `api.options` chooses which messages nanopb generates.
Bumping it is described in `proto/README.md`.

Not implemented: other entity types (lights, covers, numbers, …), logs over
the API, Home Assistant services/actions, Bluetooth proxy, voice.

## Testing

- `tests/subsys/net/esphome_api`: ztest over loopback for framing, entities,
  commands, mDNS records, IPv6, and Noise known-answer vectors generated by
  `gen_noise_vectors.py` from the Python `noise` library aioesphomeapi uses.
- `samples/net/esphome_api`: twister pytest on native_sim drives the sample
  with aioesphomeapi (plaintext and Noise: entities, states, commands,
  reconnect, wrong key). Needs `pip install aioesphomeapi==46.25.1`.

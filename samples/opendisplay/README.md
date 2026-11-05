# OpenDisplay receiver sample

Lets Home Assistant's `opendisplay` integration, or py-opendisplay, push images to the board's
display over BLE and, with `overlay-wifi.conf`, over Wi-Fi. See
[docs/opendisplay.md](../../docs/opendisplay.md).

Boards:

- `reterminal_e1001/esp32s3/procpu`: Seeed reTerminal E1001, 7.5" 800x480 monochrome e-paper
  (UC8179).
- `m5stack_core2/esp32/procpu`: M5Stack Core2, 320x240 ILI9342C TFT, shown in black and white.
- `native_sim`: LAN only over the host's sockets, with a dummy display (`CONF_FILE=native_sim.conf`).

```sh
west blobs fetch hal_espressif
west build -b reterminal_e1001/esp32s3/procpu samples/opendisplay
# BLE and Wi-Fi; set the SSID and password in overlay-wifi.conf first
west build -b reterminal_e1001/esp32s3/procpu samples/opendisplay -- -DEXTRA_CONF_FILE=overlay-wifi.conf
```

Then add the device in Home Assistant (it is discovered over Bluetooth), or:

```python
from opendisplay import OpenDisplayDevice
async with OpenDisplayDevice(mac_address="...") as dev:   # or host="<ip>"
    await dev.upload_image(image)
```

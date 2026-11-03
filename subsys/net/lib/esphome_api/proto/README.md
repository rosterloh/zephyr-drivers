# ESPHome API protocol definitions

`api.proto` and `api_options.proto` are unmodified copies from
[aioesphomeapi](https://github.com/esphome/aioesphomeapi) (MIT, Copyright (c) 2018 Otto Winter).

Pinned: tag `v46.25.1`, commit `57ed5becfb2698ae579d6b514d4f61b520d3aa1e`, API version 1.19.

`api.options` selects the messages nanopb generates and bounds their strings.

To bump:

1. Copy both files from the new tag and update the pin above.
2. Update `ESPHOME_API_VERSION_MINOR` in `include/zephyr/net/esphome_api.h` to the server version ESPHome reports
   (`resp.api_version_minor` in `esphome/components/api/api_connection.cpp`).
3. Check each `msgid` in `api.options` still matches the message's `option (id)` and each `max_size` is
   ESPHome's `(max_data_length)` + 1.

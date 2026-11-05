#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Print the WRITE_CONFIG vector used by src/main.c, serialized by py-opendisplay.

pip install py-opendisplay==7.18.1
"""

from opendisplay.models.config import DataExtended, WifiConfig
from opendisplay.protocol.config_parser import parse_tlv_config
from opendisplay.protocol.config_serializer import serialize_config

hw = (
    bytes([0, 0x01]) + bytes(22)
    + bytes([0, 0x02]) + bytes(22)
    + bytes([0, 0x04, 2]) + bytes(29)
    + bytes([0, 0x20]) + bytes(46)
)
cfg = parse_tlv_config(hw, version=1)
cfg.version = 1
cfg.wifi_config = WifiConfig.from_strings(ssid="zephyr", password="secret", server_port=2446)
cfg.data_extended = DataExtended.from_strings(friendly_name="Kitchen")
blob = serialize_config(cfg)

print(f"/* py-opendisplay serialize_config(), {len(blob)} bytes */")
for i in range(0, len(blob), 12):
    print("\t" + " ".join(f"0x{b:02x}," for b in blob[i : i + 12]))

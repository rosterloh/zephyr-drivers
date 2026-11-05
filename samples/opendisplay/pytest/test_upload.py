# SPDX-License-Identifier: Apache-2.0
"""Upload an image with py-opendisplay over the LAN transport (native_sim, NSOS).

Needs py-opendisplay (pip install py-opendisplay==7.18.1); skipped without it.
"""

import asyncio

import pytest
from twister_harness import DeviceAdapter

opendisplay = pytest.importorskip("opendisplay")
from PIL import Image, ImageDraw  # noqa: E402  (pulled in by py-opendisplay)

PORT = 2446


async def upload() -> None:
    async with opendisplay.OpenDisplayDevice(host="127.0.0.1", port=PORT) as dev:
        assert (dev.width, dev.height) == (296, 128)
        img = Image.new("RGB", (dev.width, dev.height), "white")
        ImageDraw.Draw(img).rectangle((10, 10, 100, 60), fill="black")
        await dev.upload_image(img)


def test_upload(dut: DeviceAdapter):
    dut.readlines_until(regex=f"listening on port {PORT}", timeout=10)
    asyncio.run(upload())
    dut.readlines_until(regex="refresh done: 0", timeout=10)

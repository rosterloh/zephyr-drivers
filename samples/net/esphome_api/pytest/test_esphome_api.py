# SPDX-License-Identifier: Apache-2.0
"""Drive the sample with aioesphomeapi, the client library Home Assistant uses."""

import asyncio
import re
from pathlib import Path

import pytest
from aioesphomeapi import (
    APIClient,
    ButtonInfo,
    InvalidEncryptionKeyAPIError,
    RequiresEncryptionAPIError,
    SensorInfo,
    SensorState,
    SwitchInfo,
    SwitchState,
)
from twister_harness import DeviceAdapter

HOST = "127.0.0.1"


BUILD_KEY = object()


def kconfig(dut: DeviceAdapter) -> dict[str, str]:
    text = (Path(dut.device_config.build_dir) / "zephyr" / ".config").read_text()
    return {k: q or v for k, q, v in re.findall(r'^(CONFIG_\w+)=(?:"(.*)"|(.*))$', text, re.M)}


@pytest.fixture
def cfg(dut: DeviceAdapter) -> dict[str, str]:
    dut.readlines_until(regex="esphome_api: listening on port", timeout=10)
    return kconfig(dut)


def client(cfg: dict[str, str], psk=BUILD_KEY) -> APIClient:
    if psk is BUILD_KEY:
        psk = cfg.get("CONFIG_ESPHOME_API_NOISE_PSK")
    return APIClient(HOST, int(cfg["CONFIG_ESPHOME_API_PORT"]), None, noise_psk=psk, client_info="twister")


async def session(cfg: dict[str, str], dut: DeviceAdapter) -> None:
    cli = client(cfg)
    await cli.connect(login=False)
    info = await cli.device_info()
    assert info.name == cfg["CONFIG_NET_HOSTNAME"]
    assert info.friendly_name == cfg["CONFIG_ESPHOME_API_FRIENDLY_NAME"]

    entities, _ = await cli.list_entities_services()
    by_id = {e.object_id: e for e in entities}
    assert isinstance(by_id["temperature"], SensorInfo)
    assert by_id["temperature"].unit_of_measurement == "°C"
    assert isinstance(by_id["relay"], SwitchInfo)
    assert isinstance(by_id["reset"], ButtonInfo)

    states: list = []
    cli.subscribe_states(states.append)
    await asyncio.sleep(2.5)
    temps = [s.state for s in states if isinstance(s, SensorState)]
    assert len(temps) >= 2 and all(20.0 <= t <= 24.5 for t in temps)

    cli.switch_command(by_id["relay"].key, True)
    cli.button_command(by_id["reset"].key)
    await asyncio.sleep(0.5)
    assert any(isinstance(s, SwitchState) and s.state for s in states)
    await cli.disconnect()

    dut.readlines_until(regex="relay on", timeout=2)
    dut.readlines_until(regex="reset pressed", timeout=2)


def test_session_and_reconnect(dut: DeviceAdapter, cfg: dict[str, str]):
    asyncio.run(session(cfg, dut))
    # Home Assistant reconnects after restarts; the server must take a new session.
    asyncio.run(session(cfg, dut))


def test_wrong_key(cfg: dict[str, str]):
    if "CONFIG_ESPHOME_API_NOISE" not in cfg:
        pytest.skip("plaintext build")

    async def run():
        with pytest.raises(InvalidEncryptionKeyAPIError):
            await client(cfg, "A" * 43 + "=").connect(login=False)
        with pytest.raises(RequiresEncryptionAPIError):
            await client(cfg, None).connect(login=False)

    asyncio.run(run())

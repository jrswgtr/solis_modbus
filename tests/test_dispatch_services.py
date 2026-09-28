"""Remote Dispatch services (44100 block) — live-verified write sequences."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest
import voluptuous as vol
from homeassistant.core import HomeAssistant
from homeassistant.exceptions import ServiceValidationError
from pytest_homeassistant_custom_component.common import MockConfigEntry

from custom_components.solis_modbus import (
    SCHEME_DISPATCH,
    SCHEME_DISPATCH_SCHEDULE,
    _dispatch_function_value,
    _dispatch_system_limits,
    _s32_words,
)
from custom_components.solis_modbus.const import DOMAIN
from custom_components.solis_modbus.data.enums import InverterType
from custom_components.solis_modbus.runtime import SolisRuntimeData


def test_s32_words():
    assert _s32_words(-600) == [0xFFFF, 0xFDA8]  # import 6 kW (x10 W)
    assert _s32_words(500) == [0x0000, 0x01F4]
    assert _s32_words(0) == [0, 0]


def test_function_value_pairs():
    assert _dispatch_function_value(1) == 0x0055
    assert _dispatch_function_value(2) == 0x1555
    assert _dispatch_function_value(3) == 0x5555
    assert _dispatch_function_value(1, pv_shutdown=True) == 0x0056
    assert _dispatch_function_value(3, allow_grid_charge=False, disable_discharge=True) == 0x5965
    assert _dispatch_function_value(3, battery_reserve=True, pv_limit=True) == 0x9655


DISPATCH_BOOLEAN_FIELDS = ("pv_shutdown", "allow_grid_charge", "disable_discharge", "battery_reserve", "pv_limit")


@pytest.mark.parametrize(
    ("value", "expected"),
    [(False, False), ("false", False), ("off", False), ("0", False), (0, False), (True, True), ("true", True), ("on", True), ("1", True), (1, True)],
)
@pytest.mark.parametrize("schema, required", [(SCHEME_DISPATCH, {"mode": "battery_hold"}), (SCHEME_DISPATCH_SCHEDULE, {"period": 1, "enabled": True})])
@pytest.mark.parametrize("field", DISPATCH_BOOLEAN_FIELDS)
def test_dispatch_boolean_schema_uses_home_assistant_values(schema, required, field, value, expected):
    assert schema({**required, field: value})[field] is expected


@pytest.mark.parametrize(
    ("value", "expected"),
    [(False, False), ("false", False), ("off", False), ("0", False), (0, False), (True, True), ("true", True), ("on", True), ("1", True), (1, True)],
)
def test_dispatch_schedule_enabled_uses_home_assistant_values(value, expected):
    assert SCHEME_DISPATCH_SCHEDULE({"period": 1, "enabled": value})["enabled"] is expected


@pytest.mark.parametrize("schema, required", [(SCHEME_DISPATCH, {"mode": "battery_hold"}), (SCHEME_DISPATCH_SCHEDULE, {"period": 1, "enabled": True})])
@pytest.mark.parametrize("field", DISPATCH_BOOLEAN_FIELDS)
@pytest.mark.parametrize("value", ["maybe", "2", None])
def test_dispatch_boolean_schema_rejects_invalid_values(schema, required, field, value):
    with pytest.raises(vol.Invalid):
        schema({**required, field: value})


@pytest.mark.parametrize("value", ["maybe", "2", None])
def test_dispatch_schedule_enabled_rejects_invalid_values(value):
    with pytest.raises(vol.Invalid):
        SCHEME_DISPATCH_SCHEDULE({"period": 1, "enabled": value})


@pytest.mark.parametrize("value", [False, "false", "off", "0", 0])
def test_false_text_never_enables_dispatch_function_bits(value):
    data = SCHEME_DISPATCH({"mode": "battery_hold", **dict.fromkeys(DISPATCH_BOOLEAN_FIELDS, value)})
    assert (
        _dispatch_function_value(
            3,
            pv_shutdown=data["pv_shutdown"],
            allow_grid_charge=data["allow_grid_charge"],
            disable_discharge=data["disable_discharge"],
            battery_reserve=data["battery_reserve"],
            pv_limit=data["pv_limit"],
        )
        == 0x5565
    )


def test_system_limits():
    assert _dispatch_system_limits(None, None) == (0, 0xFFFF, 0xFFFF)
    assert _dispatch_system_limits(24000, 24000) == (3, 240, 240)
    assert _dispatch_system_limits(0, None) == (1, 0, 0xFFFF)
    assert _dispatch_system_limits(None, 100) == (2, 0xFFFF, 1)
    assert _dispatch_system_limits(50, 150) == (3, 1, 2)  # 100 W/LSB, half-up
    assert _dispatch_system_limits(49, 24999) == (3, 0, 250)
    # zero is a valid cap on both sides
    assert _dispatch_system_limits(0, 0) == (3, 0, 0)
    # exact LSB multiples
    assert _dispatch_system_limits(100, 200) == (3, 1, 2)
    assert _dispatch_system_limits(149, 151) == (3, 1, 2)
    # 2.5 LSB banker's-rounds down; half-up goes up
    assert _dispatch_system_limits(250, None) == (1, 3, 0xFFFF)
    # service schema ceiling
    assert _dispatch_system_limits(240000, 240000) == (3, 2400, 2400)
    # HA may pass watts as strings
    assert _dispatch_system_limits("6000", "0") == (3, 60, 0)


@pytest.fixture
def controller():
    c = MagicMock()
    c.host = "1.2.3.4"
    c.device_id = 1
    c.inverter_config.type = InverterType.HYBRID
    c.async_read_input_register = AsyncMock(return_value=[0xAA55, 3])
    c.async_read_holding_register = AsyncMock(return_value=[0x5555, 0, 100, 40, 10000])
    c.async_write_holding_register = AsyncMock()
    c.async_write_holding_registers = AsyncMock()
    return c


async def setup_services(hass, controller):
    from custom_components.solis_modbus import async_setup

    entry = MockConfigEntry(domain=DOMAIN, data={})
    entry.add_to_hass(hass)
    entry.runtime_data = SolisRuntimeData(controller=controller)
    await async_setup(hass, {})
    return entry


def single_writes(controller):
    return [c.args for c in controller.async_write_holding_register.await_args_list]


@pytest.mark.asyncio
async def test_dispatch_grid_import_sequence(hass: HomeAssistant, controller):
    """The exact live-verified order: failsafe, master on, power pair, function, mode LAST."""
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", return_value=None):
        await hass.services.async_call(
            DOMAIN,
            "solis_dispatch",
            {"mode": "grid_import", "power_watts": 6000, "pv_shutdown": True, "failsafe_minutes": 30},
            blocking=True,
        )
    # Two atomic FC16 chunks: global (44100-44104) then realtime (44105-44112)
    blocks = [c.args for c in controller.async_write_holding_registers.await_args_list]
    assert blocks == [
        (44100, [1, 30, 0, 0xFFFF, 0xFFFF]),
        (44105, [3, 0xFFFF, 0xFDA8, 0x5556, 0, 100, 0, 0]),
    ]
    assert single_writes(controller) == []


@pytest.mark.asyncio
async def test_dispatch_battery_charge_positive_sign(hass: HomeAssistant, controller):
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 3]):
        await hass.services.async_call(DOMAIN, "solis_dispatch", {"mode": "battery_charge", "power_watts": 3000}, blocking=True)
    blocks = [c.args for c in controller.async_write_holding_registers.await_args_list]
    # battery_charge => mode 2, positive power 3000 W -> raw 300
    assert blocks[0] == (44100, [1, 30, 0, 0xFFFF, 0xFFFF])
    assert blocks[1] == (44105, [2, 0, 300, 0x5555, 0, 100, 0, 0])


@pytest.mark.asyncio
async def test_dispatch_uses_explicit_defaults_without_profile_read(hass: HomeAssistant, controller):
    controller.async_read_holding_register.side_effect = AssertionError("dispatch must not read the existing profile")
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 3]):
        await hass.services.async_call(DOMAIN, "solis_dispatch", {"mode": "battery_hold"}, blocking=True)
    assert controller.async_write_holding_registers.await_args_list[1].args == (44105, [1, 0, 0, 0x5555, 0, 100, 0, 0])
    controller.async_read_holding_register.assert_not_awaited()


@pytest.mark.asyncio
async def test_dispatch_sets_reserve_and_scaled_pv_limit_explicitly(hass: HomeAssistant, controller):
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 3]):
        await hass.services.async_call(
            DOMAIN,
            "solis_dispatch",
            {
                "mode": "self_consumption",
                "battery_reserve": True,
                "battery_reserve_soc": 40,
                "pv_limit": True,
                "pv_limit_percentage": 12.34,
            },
            blocking=True,
        )
    assert controller.async_write_holding_registers.await_args_list[1].args == (44105, [5, 0, 0, 0x9655, 0, 100, 40, 1234])


@pytest.mark.asyncio
async def test_dispatch_battery_charge_with_import_export_limits(hass: HomeAssistant, controller):
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 3]):
        await hass.services.async_call(
            DOMAIN,
            "solis_dispatch",
            {"mode": "battery_charge", "power_watts": 3000, "import_limit_watts": 6000, "export_limit_watts": 0},
            blocking=True,
        )
    blocks = [c.args for c in controller.async_write_holding_registers.await_args_list]
    # 44102 BIT00|BIT01, 44103 = 6000 W / 100, 44104 = 0 W
    assert blocks[0] == (44100, [1, 30, 0b11, 60, 0])


@pytest.mark.asyncio
async def test_dispatch_rejected_without_capability(hass: HomeAssistant, controller):
    controller.async_read_input_register = AsyncMock(return_value=[0, 3])
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", return_value=None):
        with pytest.raises(ServiceValidationError):
            await hass.services.async_call(DOMAIN, "solis_dispatch", {"mode": "grid_import", "power_watts": 1000}, blocking=True)
    controller.async_write_holding_register.assert_not_awaited()


@pytest.mark.asyncio
async def test_dispatch_stop_verified_revert(hass: HomeAssistant, controller):
    controller.async_read_input_register.side_effect = AssertionError("stop must not read the dispatch profile")
    controller.async_read_holding_register.side_effect = AssertionError("stop must not read the function word")
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", return_value=None):
        await hass.services.async_call(DOMAIN, "solis_dispatch_stop", {}, blocking=True)
    assert single_writes(controller) == [(44105, 1), (44108, 1), (44100, 0)]
    controller.async_read_input_register.assert_not_awaited()
    controller.async_read_holding_register.assert_not_awaited()


@pytest.mark.asyncio
async def test_dispatch_schedule_period_block(hass: HomeAssistant, controller):
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 3]):
        await hass.services.async_call(
            DOMAIN,
            "solis_dispatch_schedule",
            {
                "period": 2,
                "enabled": True,
                "start_time": "08:30",
                "end_time": "16:00",
                "mode": "grid_import",
                "power_watts": 6000,
                "pv_shutdown": True,
                "soc_max": 100,
            },
            blocking=True,
        )
    # period 2 base = 44116 + 14 = 44130
    controller.async_write_holding_registers.assert_awaited_once_with(44130, [1, (8 << 8) | 30, 16 << 8, 3, 0xFFFF, 0xFDA8, 0x5556, 0, 100, 0, 0])
    controller.async_read_holding_register.assert_not_awaited()
    # enabled -> long failsafe + master on
    assert (44101, 1440) in single_writes(controller)
    assert (44100, 1) in single_writes(controller)


@pytest.mark.asyncio
async def test_dispatch_schedule_disable_leaves_master_alone(hass: HomeAssistant, controller):
    controller.async_read_input_register.side_effect = AssertionError("schedule disable must not read the dispatch profile")
    controller.async_read_holding_register.side_effect = AssertionError("schedule disable must not preserve the period block")
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", return_value=None):
        await hass.services.async_call(DOMAIN, "solis_dispatch_schedule", {"period": 1, "enabled": False}, blocking=True)
    assert single_writes(controller) == [(44116, 0)]
    controller.async_write_holding_registers.assert_not_awaited()
    controller.async_read_input_register.assert_not_awaited()
    controller.async_read_holding_register.assert_not_awaited()


@pytest.mark.asyncio
async def test_dispatch_v04_activation_fails_closed(hass: HomeAssistant, controller):
    controller.async_read_input_register.return_value = [0xAA55, 4]
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 4]):
        with pytest.raises(ServiceValidationError):
            await hass.services.async_call(DOMAIN, "solis_dispatch", {"mode": "battery_hold"}, blocking=True)
    controller.async_write_holding_register.assert_not_awaited()
    controller.async_write_holding_registers.assert_not_awaited()


@pytest.mark.asyncio
async def test_dispatch_v03_self_use_discharge_block_and_release(hass: HomeAssistant, controller):
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 3]):
        await hass.services.async_call(
            DOMAIN,
            "solis_dispatch",
            {"mode": "self_consumption", "allow_grid_charge": False, "disable_discharge": True},
            blocking=True,
        )
    assert controller.async_write_holding_registers.await_args_list[1].args == (44105, [5, 0, 0, 0x5965, 0, 100, 0, 0])

    controller.reset_mock()
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 3]):
        await hass.services.async_call(
            DOMAIN,
            "solis_dispatch",
            {"mode": "self_consumption", "allow_grid_charge": False, "disable_discharge": False},
            blocking=True,
        )
    assert controller.async_write_holding_registers.await_args_list[1].args[1][3] == 0x5565


@pytest.mark.asyncio
async def test_dispatch_v01_compatibility_and_version_validation(hass: HomeAssistant, controller):
    controller.async_read_input_register.return_value = [0xAA55, 1]
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 1]):
        await hass.services.async_call(DOMAIN, "solis_dispatch", {"mode": "battery_hold"}, blocking=True)
    assert controller.async_write_holding_registers.await_args_list[1].args == (44105, [1, 0, 0, 0x55, 0, 100, 0, 0])

    controller.reset_mock()
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 1]):
        with pytest.raises(ServiceValidationError):
            await hass.services.async_call(DOMAIN, "solis_dispatch", {"mode": "self_consumption"}, blocking=True)
    controller.async_write_holding_registers.assert_not_awaited()

    controller.reset_mock()
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 1]):
        with pytest.raises(ServiceValidationError):
            await hass.services.async_call(DOMAIN, "solis_dispatch", {"mode": "battery_hold", "battery_reserve_soc": 40}, blocking=True)
    controller.async_write_holding_registers.assert_not_awaited()


@pytest.mark.asyncio
async def test_dispatch_v02_rejects_pv_limiting(hass: HomeAssistant, controller):
    controller.async_read_input_register.return_value = [0xAA55, 2]
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 2]):
        with pytest.raises(ServiceValidationError):
            await hass.services.async_call(DOMAIN, "solis_dispatch", {"mode": "battery_hold", "pv_limit": True}, blocking=True)
    controller.async_write_holding_registers.assert_not_awaited()


@pytest.mark.asyncio
async def test_dispatch_uses_live_version_when_once_cache_is_stale(hass: HomeAssistant, controller):
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 1]):
        await hass.services.async_call(
            DOMAIN,
            "solis_dispatch",
            {"mode": "self_consumption", "disable_discharge": True},
            blocking=True,
        )

    controller.async_read_input_register.assert_awaited_once_with(34502, 2)
    assert controller.async_write_holding_registers.await_args_list[1].args == (44105, [5, 0, 0, 0x5955, 0, 100, 0, 0])


@pytest.mark.asyncio
async def test_dispatch_rejects_invalid_soc_window(hass: HomeAssistant, controller):
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 3]):
        with pytest.raises(ServiceValidationError):
            await hass.services.async_call(
                DOMAIN,
                "solis_dispatch",
                {"mode": "battery_hold", "soc_min": 100, "soc_max": 100},
                blocking=True,
            )
    controller.async_write_holding_registers.assert_not_awaited()

    controller.reset_mock()
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 3]):
        with pytest.raises(ServiceValidationError):
            await hass.services.async_call(
                DOMAIN,
                "solis_dispatch",
                {"mode": "battery_hold", "soc_max": 80, "battery_reserve_soc": 81},
                blocking=True,
            )
    controller.async_write_holding_registers.assert_not_awaited()


@pytest.mark.asyncio
async def test_dispatch_schedule_sets_reserve_and_scaled_pv_limit(hass: HomeAssistant, controller):
    await setup_services(hass, controller)
    with patch("custom_components.solis_modbus.helpers.cache_get", side_effect=[0xAA55, 3]):
        await hass.services.async_call(
            DOMAIN,
            "solis_dispatch_schedule",
            {
                "period": 1,
                "enabled": True,
                "battery_reserve": True,
                "battery_reserve_soc": 55,
                "pv_limit": True,
                "pv_limit_percentage": 87.65,
            },
            blocking=True,
        )
    block = controller.async_write_holding_registers.await_args.args
    assert block[1][6:] == [0x9655, 0, 100, 55, 8765]

"""The Modbus Integration."""

import asyncio
import logging
from datetime import datetime

import voluptuous as vol
from homeassistant.components.persistent_notification import async_create as pn_create
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import Platform
from homeassistant.core import HomeAssistant, ServiceCall, SupportsResponse
from homeassistant.exceptions import ConfigEntryError, HomeAssistantError, ServiceValidationError
from homeassistant.helpers import config_validation as cv
from homeassistant.helpers.device_registry import DeviceEntry

from .const import (
    CONF_BAUDRATE,
    CONF_BYTESIZE,
    CONF_CONNECTION_TYPE,
    CONF_INVERTER_SERIAL,
    CONF_PARITY,
    CONF_POLL_PROFILE,
    CONF_SERIAL_PORT,
    CONF_SLAVE,
    CONF_STOPBITS,
    CONF_SUPPRESS_NIGHT_OFFLINE_ISSUE,
    CONN_TYPE_SERIAL,
    CONN_TYPE_TCP,
    DEFAULT_BAUDRATE,
    DEFAULT_BYTESIZE,
    DEFAULT_PARITY,
    DEFAULT_STOPBITS,
    DOMAIN,
    MODBUS_ILLEGAL_DATA_ADDRESS,
    POLL_PROFILE_ESSENTIAL,
    POLL_PROFILE_EXTREME,
    POLL_PROFILE_FULL,
)
from .data.solis_config import SOLIS_INVERTERS, InverterConfig, InverterType, inverter_options_from_config
from .data_retrieval import DataRetrieval
from .helpers import (
    combine_u32,
    combine_u32_le,
    derived_sensor_is_supported,
    extreme_includes_battery,
    get_controller,
    get_poll_profile,
    group_in_poll_profile,
    iter_controllers,
    iter_platform_entities,
    registers_declared_by,
    set_controller,
    split_s32,
    unique_id_generator,
)
from .modbus_controller import ModbusController
from .sensors.solis_base_sensor import SolisBaseSensor, SolisSensorGroup

_LOGGER = logging.getLogger(__name__)

PLATFORMS = [Platform.NUMBER, Platform.SWITCH, Platform.TIME, Platform.SELECT]

SCHEME_HOLDING_REGISTER = vol.Schema(
    {
        vol.Required("address"): vol.Coerce(int),
        vol.Required("value"): vol.Coerce(int),
        vol.Optional("host"): vol.Coerce(str),
        # services.yaml documents `slave` and the handler reads it, so it has to
        # be declared here -- voluptuous rejects undeclared keys, which made any
        # call passing a slave fail validation before it reached the handler.
        # Deliberately no default: an omitted slave still means "all controllers".
        vol.Optional("slave"): vol.Coerce(int),
    }
)
SCHEME_TIME_SET = vol.Schema({vol.Required("entity_id"): vol.Coerce(str), vol.Required("time"): vol.Coerce(str)})
SCHEME_READ_REGISTER = vol.Schema(
    {
        vol.Required("address"): vol.Coerce(int),
        vol.Optional("count", default=1): vol.All(vol.Coerce(int), vol.Range(min=1, max=50)),
        vol.Optional("register_type", default="input"): vol.In(["input", "holding"]),
        vol.Optional("host"): vol.Coerce(str),
        vol.Optional("slave", default=1): vol.Coerce(int),
    }
)
SCHEME_FORCE_CHARGE = vol.Schema(
    {
        vol.Optional("power_watts"): vol.All(vol.Coerce(int), vol.Range(min=0, max=60000)),
        vol.Optional("duration_minutes"): vol.All(vol.Coerce(int), vol.Range(min=1, max=30)),
        vol.Optional("host"): vol.Coerce(str),
        vol.Optional("slave", default=1): vol.Coerce(int),
    }
)
SCHEME_STOP_FORCE = vol.Schema(
    {
        vol.Optional("host"): vol.Coerce(str),
        vol.Optional("slave", default=1): vol.Coerce(int),
    }
)

# RC (remote control) force charge/discharge registers — the #352 latch combo:
# Solis firmware requires 43135 to be enabled BEFORE the setpoints and timeout
# are written, otherwise they do not stick.
RC_FORCE_MODE_REG = 43135  # 0 = none, 1 = force charge, 2 = force discharge
RC_CHARGE_POWER_REG = 43136  # raw = watts / 10
RC_DISCHARGE_POWER_REG = 43129  # raw = watts / 10
RC_TIMEOUT_REG = 43282  # minutes (1-30); command self-reverts after this
RC_POWER_MULTIPLIER = 10

# Remote Dispatch (protocol Ver3.4, 44100-44199; capability gate: input 34502
# reads 0xAA55). RAM-only registers with an inverter-side failsafe (44101):
# if the controller goes silent the inverter reverts on its own. Live-verified
# write order: failsafe -> master on -> power/function/SOC -> control mode LAST
# (the function field is re-initialized by the inverter unless dispatch is on).
DISPATCH_CAPABILITY_REG = 34502
DISPATCH_VERSION_REG = 34503
DISPATCH_CAPABLE_MAGIC = 0xAA55
DISPATCH_MASTER_REG = 44100
DISPATCH_FAILSAFE_REG = 44101
DISPATCH_MODE_REG = 44105
DISPATCH_POWER_REG = 44106  # S32 pair 44106/44107, x10 W
DISPATCH_FUNCTION_REG = 44108
DISPATCH_SOC_LOW_REG = 44109
DISPATCH_SOC_HIGH_REG = 44110
DISPATCH_SCHEDULE_BASE = 44116  # 6 periods x 14 registers
DISPATCH_SCHEDULE_STRIDE = 14
DISPATCH_LIMIT_UNIT = 100
DISPATCH_LIMIT_DEFAULT = 0xFFFF

# mode -> (44105 value, sign applied to power_watts); modes 3/4: +export/-import
DISPATCH_MODES = {
    "battery_hold": (1, 0),
    "battery_charge": (2, 1),
    "battery_discharge": (2, -1),
    "grid_import": (3, -1),
    "grid_export": (3, 1),
    "grid_port_import": (4, -1),
    "grid_port_export": (4, 1),
    "self_consumption": (5, 0),
    "feed_in_priority": (6, 0),
}

SCHEME_DISPATCH = vol.Schema(
    {
        vol.Required("mode"): vol.In(sorted(DISPATCH_MODES)),
        vol.Optional("power_watts", default=0): vol.All(vol.Coerce(int), vol.Range(min=0, max=240000)),
        vol.Optional("pv_shutdown"): cv.boolean,
        vol.Optional("allow_grid_charge"): cv.boolean,
        vol.Optional("disable_discharge"): cv.boolean,
        vol.Optional("battery_reserve"): cv.boolean,
        vol.Optional("battery_reserve_soc"): vol.All(vol.Coerce(int), vol.Range(min=0, max=100)),
        vol.Optional("pv_limit"): cv.boolean,
        vol.Optional("pv_limit_percentage"): vol.All(vol.Coerce(float), vol.Range(min=0, max=100)),
        vol.Optional("soc_min"): vol.All(vol.Coerce(int), vol.Range(min=0, max=100)),
        vol.Optional("soc_max"): vol.All(vol.Coerce(int), vol.Range(min=0, max=100)),
        vol.Optional("import_limit_watts"): vol.All(vol.Coerce(int), vol.Range(min=0, max=240000)),
        vol.Optional("export_limit_watts"): vol.All(vol.Coerce(int), vol.Range(min=0, max=240000)),
        vol.Optional("failsafe_minutes", default=30): vol.All(vol.Coerce(int), vol.Range(min=1, max=1440)),
        vol.Optional("host"): vol.Coerce(str),
        vol.Optional("slave", default=1): vol.Coerce(int),
    }
)
SCHEME_DISPATCH_SCHEDULE = vol.Schema(
    {
        vol.Required("period"): vol.All(vol.Coerce(int), vol.Range(min=1, max=6)),
        vol.Required("enabled"): cv.boolean,
        vol.Optional("start_time", default="00:00"): vol.Coerce(str),
        vol.Optional("end_time", default="00:00"): vol.Coerce(str),
        vol.Optional("mode", default="battery_hold"): vol.In(sorted(DISPATCH_MODES)),
        vol.Optional("power_watts", default=0): vol.All(vol.Coerce(int), vol.Range(min=0, max=240000)),
        vol.Optional("pv_shutdown"): cv.boolean,
        vol.Optional("allow_grid_charge"): cv.boolean,
        vol.Optional("disable_discharge"): cv.boolean,
        vol.Optional("battery_reserve"): cv.boolean,
        vol.Optional("battery_reserve_soc"): vol.All(vol.Coerce(int), vol.Range(min=0, max=100)),
        vol.Optional("pv_limit"): cv.boolean,
        vol.Optional("pv_limit_percentage"): vol.All(vol.Coerce(float), vol.Range(min=0, max=100)),
        vol.Optional("soc_min", default=0): vol.All(vol.Coerce(int), vol.Range(min=0, max=100)),
        vol.Optional("soc_max", default=100): vol.All(vol.Coerce(int), vol.Range(min=0, max=100)),
        vol.Optional("failsafe_minutes", default=1440): vol.All(vol.Coerce(int), vol.Range(min=1, max=1440)),
        vol.Optional("host"): vol.Coerce(str),
        vol.Optional("slave", default=1): vol.Coerce(int),
    }
)


def _dispatch_function_value(
    version: int,
    *,
    pv_shutdown: bool | None = None,
    allow_grid_charge: bool | None = None,
    battery_reserve: bool | None = None,
    disable_discharge: bool | None = None,
    pv_limit: bool | None = None,
) -> int:
    """Build a version-specific 44108/44122 function word.

    Every field supported by the reported protocol version is written as an
    explicit valid 2-bit value. Solis defines 00 and 11 as invalid; relying on
    those values to mean "unchanged" makes the result depend on stale inverter
    state and writes reserved fields on older protocol versions.
    """

    fields: list[tuple[bool | None, bool, int, int]] = [
        (pv_shutdown, False, 2, 1),
        (None, False, 2, 1),  # digital-output control
        (allow_grid_charge, True, 1, 2),
        (None, False, 2, 1),  # off-grid battery standby
    ]
    if version >= 2:
        fields.extend(
            [
                (battery_reserve, False, 2, 1),
                (disable_discharge, False, 2, 1),
                (None, False, 2, 1),  # demand control
            ]
        )
    if version >= 3:
        fields.append((pv_limit, False, 2, 1))

    result = 0
    for index, (value, default, true_code, false_code) in enumerate(fields):
        selected = default if value is None else value
        code = true_code if selected else false_code
        result |= code << (index * 2)
    return result


def _s32_words(value: int) -> list[int]:
    raw = value & 0xFFFFFFFF
    return [(raw >> 16) & 0xFFFF, raw & 0xFFFF]


def _dispatch_system_limits(import_limit_watts, export_limit_watts) -> tuple[int, int, int]:
    """Build 44102/44103/44104. Omitted = switch off + 0xFFFF (inverter default)."""

    def raw(watts) -> int:
        return (int(watts) + DISPATCH_LIMIT_UNIT // 2) // DISPATCH_LIMIT_UNIT

    switches = 0
    import_raw = DISPATCH_LIMIT_DEFAULT
    export_raw = DISPATCH_LIMIT_DEFAULT
    if import_limit_watts is not None:
        switches |= 0b01
        import_raw = raw(import_limit_watts)
    if export_limit_watts is not None:
        switches |= 0b10
        export_raw = raw(export_limit_watts)
    return switches, import_raw, export_raw


async def async_remove_config_entry_device(hass: HomeAssistant, config_entry: ConfigEntry, device_entry: DeviceEntry) -> bool:
    """Remove a config entry from a device."""
    return True


async def async_setup(hass: HomeAssistant, entry: ConfigEntry):
    """Set up the Modbus integration."""

    def service_write_holding_register(call: ServiceCall):
        address = call.data.get("address")
        value = call.data.get("value")
        host = call.data.get("host")
        slave = call.data.get("slave")

        if host:
            controller = get_controller(hass, host, slave if slave is not None else 1)
            if controller is None:
                raise ServiceValidationError(f"No Solis inverter configured for host {host} (slave {slave if slave is not None else 1})")
            hass.create_task(controller.async_write_holding_register(int(address), int(value)))
        else:
            # Without a host we write to every controller, unless a slave was
            # given explicitly -- in which case only matching devices are written.
            targets = [controller for controller in iter_controllers(hass) if slave is None or getattr(controller, "device_id", 1) == slave]
            if not targets:
                raise ServiceValidationError(f"No Solis inverter configured with slave {slave}")
            for controller in targets:
                hass.create_task(controller.async_write_holding_register(int(address), int(value)))

    async def service_set_time(call: ServiceCall) -> None:
        """Service to update a Solis time entity."""
        entity_id = call.data.get("entity_id")
        time_str = call.data.get("time")

        if not entity_id or not time_str:
            _LOGGER.error("Missing entity_id or time parameter in service call")
            return

        try:
            # Try to parse time in HH:MM:SS format first, then fallback to HH:MM
            try:
                new_time = datetime.strptime(time_str, "%H:%M:%S").time()
            except ValueError:
                new_time = datetime.strptime(time_str, "%H:%M").time()
        except Exception as e:
            _LOGGER.error("❌ Failed to parse time string '%s': %s", time_str, e)
            return

        # Look through the registered time entities (per-entry runtime data) for a match
        for entity in iter_platform_entities(call.hass, "time"):
            if entity.entity_id == entity_id:
                await entity.async_set_value(new_time)
                _LOGGER.debug("Set time for %s to %s", entity_id, new_time)
                return

        raise ServiceValidationError(f"Entity {entity_id} is not a solis_modbus time entity")

    def _resolve_controller(call: ServiceCall):
        """Resolve the target controller from optional host/slave service fields."""
        host = call.data.get("host")
        slave = call.data.get("slave", 1)
        if host:
            controller = get_controller(hass, host, slave)
            if controller is None:
                raise ServiceValidationError(f"No Solis inverter found for host {host} (slave {slave})")
            return controller
        controllers = list(iter_controllers(hass))
        if not controllers:
            raise ServiceValidationError("No Solis inverter is configured")
        if len(controllers) > 1:
            raise ServiceValidationError("Multiple Solis inverters configured — specify the 'host' field")
        return controllers[0]

    async def service_read_register(call: ServiceCall) -> dict:
        """Read arbitrary registers and return the values (register discovery / debugging)."""
        address = int(call.data["address"])
        count = int(call.data.get("count", 1))
        register_type = call.data.get("register_type", "input")
        controller = _resolve_controller(call)

        # Use the detailed variants so a Modbus exception code survives: an
        # inverter rejecting the address is the caller's mistake (usually asking
        # for a holding register as "input"), not a failure of the integration,
        # and should not surface as an unhandled 500 (#447).
        if register_type == "holding":
            values, exception_code = await controller.async_read_holding_registers_with_exception(address, count)
        else:
            values, exception_code = await controller.async_read_input_registers_with_exception(address, count)

        if values is None:
            if exception_code == MODBUS_ILLEGAL_DATA_ADDRESS:
                other = "input" if register_type == "holding" else "holding"
                raise ServiceValidationError(
                    f"The inverter has no {register_type} register at {address} (count {count}). "
                    f'If you are probing a documented address, try register_type: "{other}".'
                )
            raise HomeAssistantError(f"Read of {register_type} register {address} (count {count}) failed — see logs")

        response = {
            "address": address,
            "count": count,
            "register_type": register_type,
            "values": list(values),
            "hex": [f"0x{v:04X}" for v in values],
        }
        if count == 2:
            # Convenience decodes for 32-bit probing
            response["u32_be"] = combine_u32(list(values))
            response["s32_be"] = split_s32(list(values))
            response["u32_le"] = combine_u32_le(list(values))
        return response

    def _require_hybrid(controller):
        if controller.inverter_config.type not in (InverterType.HYBRID, InverterType.ENERGY):
            raise ServiceValidationError("Force charge/discharge is only supported on hybrid/energy-storage inverters")

    async def _force_battery(call: ServiceCall, mode: int, power_register: int) -> None:
        """Write the #352 RC combo: enable 43135 first, then setpoint + timeout."""
        controller = _resolve_controller(call)
        _require_hybrid(controller)

        await controller.async_write_holding_register(RC_FORCE_MODE_REG, mode)

        power_watts = call.data.get("power_watts")
        if power_watts is not None:
            max_watts = getattr(controller.inverter_config, "wattage_chosen", 60000) or 60000
            watts = min(int(power_watts), int(max_watts))
            await controller.async_write_holding_register(power_register, round(watts / RC_POWER_MULTIPLIER))

        duration = call.data.get("duration_minutes")
        if duration is not None:
            await controller.async_write_holding_register(RC_TIMEOUT_REG, int(duration))

    async def service_force_battery_charge(call: ServiceCall) -> None:
        await _force_battery(call, 1, RC_CHARGE_POWER_REG)

    async def service_force_battery_discharge(call: ServiceCall) -> None:
        await _force_battery(call, 2, RC_DISCHARGE_POWER_REG)

    async def service_stop_force_charge_discharge(call: ServiceCall) -> None:
        controller = _resolve_controller(call)
        _require_hybrid(controller)
        await controller.async_write_holding_register(RC_FORCE_MODE_REG, 0)

    async def _dispatch_version(controller) -> int:
        values = await controller.async_read_input_register(DISPATCH_CAPABILITY_REG, 2)
        capability = values[0] if values else None
        version = values[1] if values and len(values) > 1 else None
        if capability != DISPATCH_CAPABLE_MAGIC:
            raise ServiceValidationError(f"This inverter does not support Remote Dispatch (register 34502 reads {capability}, expected 0xAA55)")
        if version not in (1, 2, 3):
            raise ServiceValidationError(f"Unsupported Remote Dispatch Function Version in register 34503: {version} (expected 1, 2, or 3)")
        return version

    def _validate_dispatch_options(call: ServiceCall, version: int) -> None:
        if DISPATCH_MODES[call.data.get("mode", "battery_hold")][0] >= 5 and version < 2:
            raise ServiceValidationError("Self-Use and Feed-in Priority require Remote Dispatch Function Version V02 or newer")
        if version < 2 and any(call.data.get(key) is not None for key in ("battery_reserve", "battery_reserve_soc", "disable_discharge")):
            raise ServiceValidationError("Battery reserve and disable discharge require Remote Dispatch Function Version V02 or newer")
        if version < 3 and any(call.data.get(key) is not None for key in ("pv_limit", "pv_limit_percentage")):
            raise ServiceValidationError("PV limiting requires Remote Dispatch Function Version V03 or newer")
        soc_min = call.data.get("soc_min", 0)
        soc_max = call.data.get("soc_max", 100)
        if soc_min is not None and soc_max is not None and int(soc_min) >= int(soc_max):
            raise ServiceValidationError("Remote Dispatch SOC upper limit must be greater than the lower limit")
        reserve_soc = call.data.get("battery_reserve_soc")
        if reserve_soc is not None and soc_max is not None and int(reserve_soc) > int(soc_max):
            raise ServiceValidationError("Battery reserve SOC must not exceed the Remote Dispatch SOC upper limit")

    def _function_value(call: ServiceCall, version: int) -> int:
        return _dispatch_function_value(
            version,
            pv_shutdown=call.data.get("pv_shutdown"),
            allow_grid_charge=call.data.get("allow_grid_charge"),
            battery_reserve=call.data.get("battery_reserve"),
            disable_discharge=call.data.get("disable_discharge"),
            pv_limit=call.data.get("pv_limit"),
        )

    async def service_dispatch(call: ServiceCall) -> None:
        """Real-time Remote Dispatch: goal-seeking grid/battery control with failsafe."""
        controller = _resolve_controller(call)
        _require_hybrid(controller)
        version = await _dispatch_version(controller)
        _validate_dispatch_options(call, version)

        mode_value, sign = DISPATCH_MODES[call.data["mode"]]
        power_raw = sign * round(int(call.data.get("power_watts", 0)) / 10)
        function_value = _function_value(call, version)
        reserve_soc = int(call.data.get("battery_reserve_soc") or 0)
        pv_limit_raw = round(float(call.data.get("pv_limit_percentage") or 0) * 100)
        soc_low = int(call.data["soc_min"]) if call.data.get("soc_min") is not None else 0
        soc_high = int(call.data["soc_max"]) if call.data.get("soc_max") is not None else 100
        switches, import_raw, export_raw = _dispatch_system_limits(call.data.get("import_limit_watts"), call.data.get("export_limit_watts"))

        # The dispatch block must be written as contiguous chunks (Ver3.4 doc):
        # scattered single-register writes get silently dropped/re-initialized,
        # especially under write-queue contention. Two atomic FC16 blocks:
        #   global 44100-44104  = master, failsafe, limit switch, import/export caps
        #   realtime 44105-44112 = mode, power(S32), function, SOC window
        # Global first so dispatch is active before the realtime block lands
        # (the function field is re-initialized unless the master is already on).
        await controller.async_write_holding_registers(
            DISPATCH_MASTER_REG,
            [1, int(call.data.get("failsafe_minutes", 30)), switches, import_raw, export_raw],
        )
        await controller.async_write_holding_registers(
            DISPATCH_MODE_REG,
            [mode_value, *_s32_words(power_raw), function_value, soc_low, soc_high, reserve_soc, pv_limit_raw],
        )

    async def service_dispatch_stop(call: ServiceCall) -> None:
        """Release Remote Dispatch (live-verified revert sequence)."""
        controller = _resolve_controller(call)
        _require_hybrid(controller)

        await controller.async_write_holding_register(DISPATCH_MODE_REG, 1)
        await controller.async_write_holding_register(DISPATCH_FUNCTION_REG, 1)
        await controller.async_write_holding_register(DISPATCH_MASTER_REG, 0)

    async def service_dispatch_schedule(call: ServiceCall) -> None:
        """Program one of the six inverter-resident scheduled dispatch periods.

        The schedule executes on the inverter itself — it keeps running even if
        Home Assistant dies (until the failsafe interval expires unrefreshed).
        """
        controller = _resolve_controller(call)
        _require_hybrid(controller)

        base = DISPATCH_SCHEDULE_BASE + (int(call.data["period"]) - 1) * DISPATCH_SCHEDULE_STRIDE
        if not call.data["enabled"]:
            await controller.async_write_holding_register(base, 0)
            return

        version = await _dispatch_version(controller)
        _validate_dispatch_options(call, version)

        def packed_time(value: str) -> int:
            parsed = datetime.strptime(value, "%H:%M")
            return (parsed.hour << 8) | parsed.minute

        try:
            start_packed = packed_time(call.data.get("start_time", "00:00"))
            end_packed = packed_time(call.data.get("end_time", "00:00"))
        except ValueError as err:
            raise ServiceValidationError(f"Invalid time (expected HH:MM): {err}") from err

        mode_value, sign = DISPATCH_MODES[call.data.get("mode", "battery_hold")]
        power_raw = sign * round(int(call.data.get("power_watts", 0)) / 10)

        function_value = _function_value(call, version)
        reserve_soc = int(call.data.get("battery_reserve_soc") or 0)
        pv_limit_raw = round(float(call.data.get("pv_limit_percentage") or 0) * 100)
        block = [
            1,
            start_packed,
            end_packed,
            mode_value,
            *_s32_words(power_raw),
            function_value,
            int(call.data.get("soc_min", 0)),
            int(call.data.get("soc_max", 100)),
            reserve_soc,
            pv_limit_raw,
        ]
        await controller.async_write_holding_registers(base, block)
        await controller.async_write_holding_register(DISPATCH_FAILSAFE_REG, int(call.data.get("failsafe_minutes", 1440)))
        await controller.async_write_holding_register(DISPATCH_MASTER_REG, 1)

    hass.services.async_register(DOMAIN, "solis_write_holding_register", service_write_holding_register, schema=SCHEME_HOLDING_REGISTER)
    hass.services.async_register(DOMAIN, "solis_write_time", service_set_time, schema=SCHEME_TIME_SET)
    hass.services.async_register(DOMAIN, "solis_read_register", service_read_register, schema=SCHEME_READ_REGISTER, supports_response=SupportsResponse.ONLY)
    hass.services.async_register(DOMAIN, "solis_force_battery_charge", service_force_battery_charge, schema=SCHEME_FORCE_CHARGE)
    hass.services.async_register(DOMAIN, "solis_force_battery_discharge", service_force_battery_discharge, schema=SCHEME_FORCE_CHARGE)
    hass.services.async_register(DOMAIN, "solis_stop_force_charge_discharge", service_stop_force_charge_discharge, schema=SCHEME_STOP_FORCE)
    hass.services.async_register(DOMAIN, "solis_dispatch", service_dispatch, schema=SCHEME_DISPATCH)
    hass.services.async_register(DOMAIN, "solis_dispatch_stop", service_dispatch_stop, schema=SCHEME_STOP_FORCE)
    hass.services.async_register(DOMAIN, "solis_dispatch_schedule", service_dispatch_schedule, schema=SCHEME_DISPATCH_SCHEDULE)

    return True


async def _async_reload_on_update(hass: HomeAssistant, entry: ConfigEntry) -> None:
    """Reload the entry when its options change.

    Without this, edits made in the options flow (poll intervals, model,
    feature toggles, essential-only) are saved to the entry but never applied
    until Home Assistant is restarted.
    """
    await hass.config_entries.async_reload(entry.entry_id)


async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry):
    """Set up Modbus from a config entry."""

    # Merge data and options (options take priority)
    config = {**entry.data, **entry.options}
    slave = config.get(CONF_SLAVE, 1)
    inverter_serial = config.get(CONF_INVERTER_SERIAL)

    # --- MISSING VALIDATION BLOCK RESTORED ---
    if not inverter_serial:
        pn_create(
            hass,
            "Solis Modbus: Inverter Serial is missing. Please reconfigure the integration.",
            title="Solis Modbus Configuration Issue",
            notification_id="solis_modbus_missing_serial",
        )
        raise ConfigEntryError("Inverter Serial is missing")
    # -----------------------------------------

    # Deferred Migration Check
    if entry.unique_id != inverter_serial:
        _LOGGER.warning("Executing deferred migration to Serial Number IDs")
        await async_migrate_to_serial_ids(hass, entry)

    # Determine connection type (default to TCP for backwards compatibility with old configs)
    connection_type = config.get(CONF_CONNECTION_TYPE, CONN_TYPE_TCP if "host" in config else CONN_TYPE_SERIAL)

    # Get connection-specific parameters
    host = config.get("host")
    port = config.get("port", 502)

    # ... (Rest of your function remains the same) ...
    if connection_type == CONN_TYPE_TCP:
        connection_id = f"{host}:{port}"
    else:  # Serial
        serial_port = config.get(CONF_SERIAL_PORT, "/dev/ttyUSB0")
        connection_id = serial_port

    _LOGGER.debug(config)

    # Shared cross-entry storage (register cache); per-entry state lives on
    # entry.runtime_data (see runtime.SolisRuntimeData).
    hass.data.setdefault(DOMAIN, {})

    # Stagger additional config entries on the same Modbus link so two inverters do not hammer the logger at once.
    existing_same_link = sum(1 for c in iter_controllers(hass) if getattr(c, "connection_id", None) == connection_id)
    if existing_same_link:
        delay_s = min(1.5 * existing_same_link, 5.0)
        _LOGGER.debug(
            "Staggering startup: %s existing controller(s) on %s, waiting %.1fs",
            existing_same_link,
            connection_id,
            delay_s,
        )
        await asyncio.sleep(delay_s)

    _LOGGER.info(f"Loaded Solis Modbus Integration ({connection_type}) with Model: {config.get('model')}")

    # ... (Config extraction ...) ...
    poll_interval_fast = config.get("poll_interval_fast", 5)
    poll_interval_normal = config.get("poll_interval_normal", 15)
    poll_interval_slow = config.get("poll_interval_slow", 30)
    inverter_model = config.get("model")
    identification = config.get("identification", None)

    if inverter_model is None:
        old_type = config.get("type", "hybrid")
        inverter_model = "S6-EH3P" if old_type == "hybrid" else ("WAVESHARE" if old_type == "hybrid-waveshare" else "S6-GR1P")

    inverter_template: InverterConfig | None = next((inv for inv in SOLIS_INVERTERS if inv.model == inverter_model), None)

    # defaulting
    if inverter_template is None:
        pn_create(
            hass,
            "Your Solis Modbus configuration is invalid. Please reconfigure the integration.",
            title="Solis Modbus Configuration Issue",
            notification_id="solis_modbus_invalid_config",
        )
        raise ConfigEntryError

    user_options = inverter_options_from_config(config, inverter_template)
    inverter_config = inverter_template.clone_with_options(user_options, config.get("connection", "S2_WL_ST"))

    # Load correct sensor data based on inverter type
    if inverter_config.type in [InverterType.STRING, InverterType.GRID]:
        from .sensor_data.string_sensors import string_sensors as sensors
        from .sensor_data.string_sensors import string_sensors_derived as sensors_derived
    else:
        from .sensor_data.hybrid_sensors import hybrid_sensors as sensors
        from .sensor_data.hybrid_sensors import hybrid_sensors_derived as sensors_derived

    # Create the Modbus controller and assign sensor groups
    controller_params = {
        "hass": hass,
        "device_id": slave,
        "identification": identification,
        "fast_poll": poll_interval_fast,
        "normal_poll": poll_interval_normal,
        "slow_poll": poll_interval_slow,
        "inverter_config": inverter_config,
        "connection_type": connection_type,
        "serial_number": inverter_serial,
    }

    if connection_type == CONN_TYPE_TCP:
        controller_params["host"] = host
        controller_params["port"] = port
    else:  # Serial
        controller_params["serial_port"] = config.get(CONF_SERIAL_PORT, "/dev/ttyUSB0")
        controller_params["baudrate"] = config.get(CONF_BAUDRATE, DEFAULT_BAUDRATE)
        controller_params["bytesize"] = config.get(CONF_BYTESIZE, DEFAULT_BYTESIZE)
        controller_params["parity"] = config.get(CONF_PARITY, DEFAULT_PARITY)
        controller_params["stopbits"] = config.get(CONF_STOPBITS, DEFAULT_STOPBITS)

    controller = ModbusController(**controller_params)

    # From here the controller holds a ref on the shared Modbus client; release it
    # if setup fails so a failed entry doesn't pin the connection open (HA won't
    # call async_unload_entry when async_setup_entry raises).
    try:
        controller._sensor_groups = []
        poll_profile = get_poll_profile(entry)
        include_battery = extreme_includes_battery(entry)

        # A profile that matches nothing would set the entry up with no sensors at
        # all, which reads as a broken integration. Extreme currently only maps the
        # hybrid groups, so fall back rather than silently produce an empty entry.
        if poll_profile == POLL_PROFILE_EXTREME and not any(group.get("extreme") for group in sensors):
            _LOGGER.warning(
                "Extreme poll profile is not mapped for this inverter type yet; falling back to essential-only polling",
            )
            poll_profile = POLL_PROFILE_ESSENTIAL

        skipped_by_profile = 0
        selected_groups = []
        for group in sensors:
            feature_requirement = group.get("feature_requirement", [])
            if feature_requirement and not any(feature in inverter_config.features for feature in feature_requirement):
                group_name = group.get("name", group.get("register_start", "Unnamed"))
                _LOGGER.warning(f"Skipping sensor group '{group_name}' due to missing required features: {feature_requirement}")
                continue

            if not group_in_poll_profile(group, poll_profile, include_battery):
                skipped_by_profile += 1
                continue

            selected_groups.append(group)
            controller._sensor_groups.append(SolisSensorGroup(hass=hass, definition=group, controller=controller, identification=identification))

        if poll_profile != POLL_PROFILE_FULL:
            _LOGGER.info(
                "Poll profile '%s' active: %d sensor group(s) skipped, %d remaining (reduces datalogger load)",
                poll_profile,
                skipped_by_profile,
                len(controller._sensor_groups),
            )

        # Derived sensors are computed from registers other groups poll, so a
        # reduced profile has to filter them too — otherwise e.g. Power Factor
        # (33079-33082) survives into extreme mode and never receives a value.
        polled_registers = registers_declared_by(selected_groups)
        known_registers = registers_declared_by(sensors)
        supported_derived = [entity for entity in sensors_derived if derived_sensor_is_supported(entity, polled_registers, known_registers)]

        if poll_profile != POLL_PROFILE_FULL and len(supported_derived) != len(sensors_derived):
            _LOGGER.info(
                "Poll profile '%s': %d derived sensor(s) skipped, their source registers are not polled",
                poll_profile,
                len(sensors_derived) - len(supported_derived),
            )

        controller._derived_sensors = [
            SolisBaseSensor(
                hass=hass,
                name=entity.get("name"),
                controller=controller,
                registrars=[int(r) for r in entity.get("register", [])],
                write_register=entity.get("write_register", None),
                state_class=entity.get("state_class", None),
                device_class=entity.get("device_class", None),
                unit_of_measurement=entity.get("unit_of_measurement", None),
                multiplier=entity.get("multiplier", 1),
                editable=entity.get("editable", False),
                hidden=entity.get("hidden", False),
                category=entity.get("category", None),
                unique_id=unique_id_generator(controller, entity.get("unique", "reserve")),
            )
            for entity in supported_derived
        ]

        set_controller(hass, controller, entry)

        _LOGGER.debug(f"Config entry setup for {connection_type} connection: {connection_id}, slave {slave}")

        # Set up all platforms in one call (concurrent) — matches the unload side,
        # which already unloads [Platform.SENSOR, *PLATFORMS] together.
        await hass.config_entries.async_forward_entry_setups(entry, [Platform.SENSOR, *PLATFORMS])

        entry.runtime_data.data_retrieval = DataRetrieval(
            hass,
            controller,
            entry.entry_id,
            suppress_night_issue=bool(config.get(CONF_SUPPRESS_NIGHT_OFFLINE_ISSUE, False)),
        )
    except Exception:
        controller.close_connection()
        entry.runtime_data = None
        raise

    # Apply option changes automatically (see _async_reload_on_update).
    entry.async_on_unload(entry.add_update_listener(_async_reload_on_update))

    return True


async def async_migrate_to_serial_ids(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Migrate entities and config entry to use Serial Number."""
    import homeassistant.helpers.device_registry as dr
    import homeassistant.helpers.entity_registry as er

    config = {**entry.data, **entry.options}
    inverter_serial = config.get(CONF_INVERTER_SERIAL)
    host = config.get("host")
    identification = config.get("identification")

    # Double-check we have what we need
    if not inverter_serial:
        return False

    _LOGGER.info("Starting migration of entities to Serial: %s", inverter_serial)

    # =========================================================================
    # NEW: Device Registry Migration (Prevention of Duplicate/Ghost Devices)
    # =========================================================================
    dev_reg = dr.async_get(hass)

    # Get all devices associated with this config entry
    devices = dev_reg.devices.get_devices_for_config_entry_id(entry.entry_id)

    for device in devices:
        # Check if this device is NOT using the new serial yet
        # (This identifies the old IP-based device entry)
        if not any(idf[1] == inverter_serial for idf in device.identifiers):
            try:
                _LOGGER.info("Migrating Device identifiers for %s to %s", device.name, inverter_serial)
                dev_reg.async_update_device(device.id, new_identifiers={(DOMAIN, inverter_serial)})
            except ValueError:
                # This happens if a device with the new serial ALREADY exists.
                # In that case, we can't merge them automatically, so we skip
                # and let the old one become a ghost (user can delete it).
                _LOGGER.warning("Could not migrate device identifiers: Target serial %s already exists", inverter_serial)
    # =========================================================================

    # Setup the mock controller for ID generation
    from types import SimpleNamespace

    controller = SimpleNamespace()
    controller.device_serial_number = inverter_serial
    controller.identification = identification
    controller.host = host

    # Get the Entity Registry
    ent_reg = er.async_get(hass)

    def safe_migrate_entity(platform, old_uid, new_uid):
        """Safely migrates an entity ID, handling collisions."""
        old_entity_id = ent_reg.async_get_entity_id(platform, DOMAIN, old_uid)
        if not old_entity_id:
            return

        new_entity_id = ent_reg.async_get_entity_id(platform, DOMAIN, new_uid)
        if new_entity_id:
            if new_entity_id == old_entity_id:
                return
            _LOGGER.warning("Migration collision: Removing %s to migrate %s", new_entity_id, old_entity_id)
            ent_reg.async_remove(new_entity_id)

        try:
            _LOGGER.info("Migrating %s -> %s", old_uid, new_uid)
            ent_reg.async_update_entity(old_entity_id, new_unique_id=new_uid)
        except ValueError as e:
            _LOGGER.error("Migration failed for %s: %s", old_entity_id, e)

    # --- SENSOR LOGIC (Same as before) ---
    inverter_model = config.get("model")
    if inverter_model is None:
        old_type = config.get("type", "hybrid")
        inverter_model = "S6-EH3P" if old_type == "hybrid" else ("WAVESHARE" if old_type == "hybrid-waveshare" else "S6-GR1P")

    inverter_template: InverterConfig | None = next((inv for inv in SOLIS_INVERTERS if inv.model == inverter_model), None)

    if inverter_template:
        user_options = inverter_options_from_config(config, inverter_template)
        inverter_config = inverter_template.clone_with_options(user_options, config.get("connection", "S2_WL_ST"))
        if inverter_config.type in [InverterType.STRING, InverterType.GRID]:
            from .sensor_data.string_sensors import string_sensors, string_sensors_derived

            sensors = string_sensors
            sensors_derived = string_sensors_derived
        else:
            from .sensor_data.hybrid_sensors import hybrid_sensors, hybrid_sensors_derived

            sensors = hybrid_sensors
            sensors_derived = hybrid_sensors_derived

        from .helpers import unique_id_generator
        from .sensor_data.time_sensors import get_time_sensors

        def get_old_id(uid, ctrl):
            if ctrl.identification:
                return f"{DOMAIN}_{ctrl.identification}_{uid}"
            return f"{DOMAIN}_{ctrl.host}_{uid}"

        # A. Standard Sensors
        for group in sensors:
            feature_requirement = group.get("feature_requirement", [])
            if feature_requirement and not any(feature in inverter_config.features for feature in feature_requirement):
                continue
            for entity in group.get("entities", []):
                if entity.get("type") == "reserve":
                    continue
                uid_key = entity.get("unique", "reserve")
                new_uid = unique_id_generator(controller, uid_key)
                old_uid = get_old_id(uid_key, controller)
                if new_uid != old_uid:
                    safe_migrate_entity(Platform.SENSOR, old_uid, new_uid)

        # B. Derived Sensors
        for entity in sensors_derived:
            uid_key = entity.get("unique", "reserve")
            new_uid = unique_id_generator(controller, uid_key)
            if identification:
                old_uid_derived = f"{DOMAIN}_{identification}_{uid_key}"
            else:
                old_uid_derived = f"{DOMAIN}_{uid_key}"
            if new_uid != old_uid_derived:
                safe_migrate_entity(Platform.SENSOR, old_uid_derived, new_uid)

        # C. Time Entities
        for entity in get_time_sensors(inverter_config):
            uid_key = entity.get("unique", "reserve")
            new_uid = unique_id_generator(controller, uid_key)
            old_uid = get_old_id(uid_key, controller)
            if new_uid != old_uid:
                safe_migrate_entity(Platform.TIME, old_uid, new_uid)

    # --- CONFIG ENTRY LOGIC ---
    if entry.unique_id != inverter_serial:
        _LOGGER.info("Migrating Config Entry ID from %s to %s", entry.unique_id, inverter_serial)
        hass.config_entries.async_update_entry(entry, unique_id=inverter_serial)

    return True


def _broken_dict_unique_id_matches(unique_id: str, unique_key: str) -> bool:
    """True when unique_id is a dict-stringified form for this unique key (#452)."""
    return f"'unique': '{unique_key}'" in unique_id


def _entity_sort_key(entry):
    """Oldest first; UIDs without data_type sort before those that embed it."""
    created = getattr(entry, "created_at", None)
    uid = entry.unique_id or ""
    has_data_type = "'data_type'" in uid
    return (created is None, created, has_data_type, entry.entity_id)


def _iter_sensor_unique_keys(sensors) -> list[str]:
    keys: list[str] = []
    for group in sensors:
        for entity in group.get("entities", []):
            if entity.get("type") == "reserve":
                continue
            uid_key = entity.get("unique")
            if uid_key:
                keys.append(uid_key)
    return keys


async def async_migrate_dict_unique_ids(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Rewrite dict-stringified unique_ids to stable keys; restore original entity_ids.

    Issue #452: SolisSensorGroup passed the whole entity dict into unique_id_generator,
    so definition changes (e.g. data_type=U32 in v4.2.0) created duplicate entities.
    For duplicates we remove the oldest registry row (frees the original entity_id,
    recorder history for that id remains) then rename the newest onto that entity_id
    with the stable unique_id so HA's rename migration carries post-upgrade history.
    """
    import homeassistant.helpers.entity_registry as er

    config = {**entry.data, **entry.options}
    inverter_serial = config.get(CONF_INVERTER_SERIAL)
    host = config.get("host")
    identification = config.get("identification")

    from types import SimpleNamespace

    controller = SimpleNamespace()
    controller.device_serial_number = inverter_serial
    controller.identification = identification
    controller.host = host

    inverter_model = config.get("model")
    if inverter_model is None:
        old_type = config.get("type", "hybrid")
        inverter_model = "S6-EH3P" if old_type == "hybrid" else ("WAVESHARE" if old_type == "hybrid-waveshare" else "S6-GR1P")

    inverter_template: InverterConfig | None = next((inv for inv in SOLIS_INVERTERS if inv.model == inverter_model), None)
    if inverter_template is None:
        _LOGGER.warning("Dict unique_id migration skipped: unknown model %s", inverter_model)
        return True

    user_options = inverter_options_from_config(config, inverter_template)
    inverter_config = inverter_template.clone_with_options(user_options, config.get("connection", "S2_WL_ST"))
    if inverter_config.type in [InverterType.STRING, InverterType.GRID]:
        from .sensor_data.string_sensors import string_sensors as sensors
    else:
        from .sensor_data.hybrid_sensors import hybrid_sensors as sensors

    unique_keys = _iter_sensor_unique_keys(sensors)
    ent_reg = er.async_get(hass)
    platforms = (Platform.SENSOR, Platform.NUMBER)

    # Index this config entry's entities once per platform.
    by_platform: dict[str, list] = {p: [] for p in platforms}
    for ent in er.async_entries_for_config_entry(ent_reg, entry.entry_id):
        if ent.domain in by_platform:
            by_platform[ent.domain].append(ent)

    for platform in platforms:
        entries = by_platform[platform]
        for unique_key in unique_keys:
            correct_uid = unique_id_generator(controller, unique_key)
            matches = [e for e in entries if e.unique_id == correct_uid or _broken_dict_unique_id_matches(e.unique_id or "", unique_key)]
            if not matches:
                continue

            # Drop stale refs after removals within this loop
            matches = [e for e in matches if ent_reg.async_get(e.entity_id) is not None]
            if not matches:
                continue

            if len(matches) == 1:
                sole = matches[0]
                if sole.unique_id != correct_uid:
                    try:
                        _LOGGER.info(
                            "Migrating dict unique_id for %s -> %s",
                            sole.entity_id,
                            correct_uid,
                        )
                        ent_reg.async_update_entity(sole.entity_id, new_unique_id=correct_uid)
                    except ValueError as err:
                        _LOGGER.error("Dict unique_id migration failed for %s: %s", sole.entity_id, err)
                continue

            matches_sorted = sorted(matches, key=_entity_sort_key)
            oldest = matches_sorted[0]
            newest = matches_sorted[-1]
            original_entity_id = oldest.entity_id
            newest_entity_id = newest.entity_id

            # Remove oldest and any middle ghosts to free original entity_id.
            for ghost in matches_sorted[:-1]:
                _LOGGER.info(
                    "Removing duplicate/orphan entity %s (unique_id=%s) to restore %s",
                    ghost.entity_id,
                    ghost.unique_id,
                    original_entity_id,
                )
                ent_reg.async_remove(ghost.entity_id)

            try:
                _LOGGER.info(
                    "Restoring %s onto original entity_id %s with stable unique_id",
                    newest_entity_id,
                    original_entity_id,
                )
                update_kwargs = {"new_unique_id": correct_uid}
                if newest_entity_id != original_entity_id:
                    update_kwargs["new_entity_id"] = original_entity_id
                ent_reg.async_update_entity(newest_entity_id, **update_kwargs)
            except ValueError as err:
                _LOGGER.error(
                    "Failed to restore %s -> %s: %s",
                    newest_entity_id,
                    original_entity_id,
                    err,
                )

    return True


async def async_migrate_entry(hass: HomeAssistant, config_entry: ConfigEntry) -> bool:
    _LOGGER.debug("Migrating from version %s", config_entry.version)

    if config_entry.version <= 2:
        # Check if we have the serial right now
        if config_entry.data.get(CONF_INVERTER_SERIAL):
            await async_migrate_to_serial_ids(hass, config_entry)
        else:
            _LOGGER.info("Migration deferred: Waiting for Serial Number reconfigure.")
            return False

        hass.config_entries.async_update_entry(config_entry, version=3)

    if config_entry.version == 3:
        await async_migrate_dict_unique_ids(hass, config_entry)
        hass.config_entries.async_update_entry(config_entry, version=4)

    if config_entry.version == 4:
        _migrate_essential_only_to_poll_profile(hass, config_entry)
        hass.config_entries.async_update_entry(config_entry, version=5)

    _LOGGER.info("Migration to version %s successful", config_entry.version)
    return True


def _migrate_essential_only_to_poll_profile(hass: HomeAssistant, config_entry: ConfigEntry) -> None:
    """Fold the `essential_only` boolean into the `poll_profile` select (#457).

    Both data and options are rewritten: setup merges {**data, **options}, so a
    leftover `essential_only` in options would shadow a migrated data key and
    quietly keep the old behaviour.
    """
    merged = {**config_entry.data, **config_entry.options}
    existing = merged.get(CONF_POLL_PROFILE)
    had_legacy_key = "essential_only" in config_entry.data or "essential_only" in config_entry.options

    if existing is not None and not had_legacy_key:
        return

    if existing is not None:
        # Already on a profile, but a stale boolean is still present. Keep the
        # chosen profile and drop the dead key rather than letting it linger.
        profile = existing
    else:
        profile = POLL_PROFILE_ESSENTIAL if merged.get("essential_only", False) else POLL_PROFILE_FULL

    data = {k: v for k, v in config_entry.data.items() if k != "essential_only"}
    options = {k: v for k, v in config_entry.options.items() if k != "essential_only"}
    data[CONF_POLL_PROFILE] = profile

    hass.config_entries.async_update_entry(config_entry, data=data, options=options)
    _LOGGER.info("Migrated essential_only=%s to poll_profile='%s'", merged.get("essential_only", False), profile)


async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry):
    """Unload a Modbus config entry."""
    _LOGGER.debug("init async_unload_entry")

    # Unload platforms associated with this integration. SENSOR is forwarded
    # in async_setup_entry alongside PLATFORMS, so it must be unloaded here too —
    # otherwise a reload leaves the sensor platform set up and the next setup
    # fails with "config entry for solis_modbus.sensor has already been setup!".
    unload_ok = all(await asyncio.gather(*(hass.config_entries.async_forward_entry_unload(entry, platform) for platform in [Platform.SENSOR, *PLATFORMS])))

    # Clean up resources (per-entry state lives on entry.runtime_data, which HA
    # clears after unload; the shared register cache in hass.data survives on purpose)
    if unload_ok:
        runtime = getattr(entry, "runtime_data", None)
        if runtime is not None:
            if runtime.data_retrieval is not None:
                await runtime.data_retrieval.async_stop()
            _LOGGER.debug("Closing Modbus connection for entry %s", entry.entry_id)
            runtime.controller.close_connection()

    return unload_ok

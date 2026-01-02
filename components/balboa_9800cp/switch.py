import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

from . import (
    CONF_BALBOA_ID,
    Balboa9800CP,
    balboa_ns,
)

CONF_PUMP1 = "pump1"
CONF_PUMP2 = "pump2"
CONF_BLOWER = "blower"

BalboaToggleSwitch = balboa_ns.class_("BalboaToggleSwitch", switch.Switch, cg.Component)

DEPENDENCIES = ["balboa_9800cp"]

# ESPHome 2025.11 deprecates *_SCHEMA constants; use switch.switch_schema when available.
try:
    _ENTITY_SCHEMA = switch.switch_schema(BalboaToggleSwitch)
except AttributeError:
    # Fallback for older ESPHome
    _ENTITY_SCHEMA = switch.SWITCH_SCHEMA.extend({cv.GenerateID(): cv.declare_id(BalboaToggleSwitch)})

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CP),
        cv.Optional(CONF_PUMP1): _ENTITY_SCHEMA,
        cv.Optional(CONF_PUMP2): _ENTITY_SCHEMA,
        cv.Optional(CONF_BLOWER): _ENTITY_SCHEMA,
    }
)

async def _build_one(parent, key, conf, setter_name: str, kind: int):
    # Create/register switch entity
    if hasattr(switch, "new_switch"):
        var = await switch.new_switch(conf)
    else:
        var = cg.new_Pvariable(conf[CONF_ID])
        await switch.register_switch(var, conf)

    await cg.register_component(var, conf)

    # Configure entity in C++
    cg.add(getattr(var, "set_kind")(kind))
    cg.add(getattr(parent, setter_name)(var))

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    # kind values must match your C++ enum CommandKind:
    # 5=PUMP1, 6=PUMP2, 7=BLOWER (as implemented in the updated C++)
    if CONF_PUMP1 in config:
        await _build_one(parent, CONF_PUMP1, config[CONF_PUMP1], "set_pump1_switch", 5)
    if CONF_PUMP2 in config:
        await _build_one(parent, CONF_PUMP2, config[CONF_PUMP2], "set_pump2_switch", 6)
    if CONF_BLOWER in config:
        await _build_one(parent, CONF_BLOWER, config[CONF_BLOWER], "set_blower_switch", 7)

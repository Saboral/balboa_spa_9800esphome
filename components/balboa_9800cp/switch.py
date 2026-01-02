import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

from . import CONF_BALBOA_ID, Balboa9800CP, balboa_ns

CONF_PUMP1 = "pump1"
CONF_PUMP2 = "pump2"
CONF_BLOWER = "blower"

# Must match the C++ class name and inheritance
BalboaToggleSwitch = balboa_ns.class_("BalboaToggleSwitch", switch.Switch, cg.Component)

def _schema_for_switch():
    # ESPHome newer versions use switch.switch_schema(); older used SWITCH_SCHEMA
    if hasattr(switch, "switch_schema"):
        return switch.switch_schema(BalboaToggleSwitch)
    # fallback (very old)
    return switch.SWITCH_SCHEMA.extend({cv.GenerateID(): cv.declare_id(BalboaToggleSwitch)})

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CP),
        cv.Optional(CONF_PUMP1): _schema_for_switch(),
        cv.Optional(CONF_PUMP2): _schema_for_switch(),
        cv.Optional(CONF_BLOWER): _schema_for_switch(),
    }
)

async def _new_switch(conf):
    if hasattr(switch, "new_switch"):
        return await switch.new_switch(conf)
    # older ESPHome
    return await cg.new_Pvariable(conf[CONF_ID])

async def _build_one(parent, conf, setter_name: str, kind: int):
    var = await _new_switch(conf)
    await cg.register_component(var, conf)
    cg.add(var.set_kind(kind))
    cg.add(getattr(parent, setter_name)(var))

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    # kind values must match C++ enum CommandKind in the updated .cpp/.h:
    # 5=PUMP1, 6=PUMP2, 7=BLOWER
    if CONF_PUMP1 in config:
        await _build_one(parent, config[CONF_PUMP1], "set_pump1_switch", 5)
    if CONF_PUMP2 in config:
        await _build_one(parent, config[CONF_PUMP2], "set_pump2_switch", 6)
    if CONF_BLOWER in config:
        await _build_one(parent, config[CONF_BLOWER], "set_blower_switch", 7)

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

from . import Balboa9800CP, balboa_ns

CONF_BALBOA_ID = "balboa_id"

# Switch keys
CONF_PUMP1 = "pump1"
CONF_PUMP2 = "pump2"
CONF_BLOWER = "blower"

BalboaToggleSwitch = balboa_ns.class_("BalboaToggleSwitch", switch.Switch)

# Command IDs must match your C++ enum in balboa_9800cp_updated.h
CMD_PUMP1 = 5
CMD_PUMP2 = 6
CMD_BLOWER = 7

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CP),
        cv.Optional(CONF_PUMP1): switch.SWITCH_SCHEMA.extend({cv.GenerateID(): cv.declare_id(BalboaToggleSwitch)}),
        cv.Optional(CONF_PUMP2): switch.SWITCH_SCHEMA.extend({cv.GenerateID(): cv.declare_id(BalboaToggleSwitch)}),
        cv.Optional(CONF_BLOWER): switch.SWITCH_SCHEMA.extend({cv.GenerateID(): cv.declare_id(BalboaToggleSwitch)}),
    }
)


async def _build_one(parent, cfg, cmd_id):
    var = cg.new_Pvariable(cfg[CONF_ID])
    await switch.register_switch(var, cfg)
    cg.add(var.set_parent(parent))
    cg.add(var.set_command(cmd_id))
    return var


async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    if CONF_PUMP1 in config:
        var = await _build_one(parent, config[CONF_PUMP1], CMD_PUMP1)
        cg.add(parent.set_pump1_switch(var))

    if CONF_PUMP2 in config:
        var = await _build_one(parent, config[CONF_PUMP2], CMD_PUMP2)
        cg.add(parent.set_pump2_switch(var))

    if CONF_BLOWER in config:
        var = await _build_one(parent, config[CONF_BLOWER], CMD_BLOWER)
        cg.add(parent.set_blower_switch(var))

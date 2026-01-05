import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch

from . import Balboa9800CPComponent, balboa_ns

CONF_BALBOA_ID = "balboa_id"

BalboaToggleSwitch = balboa_ns.class_("BalboaToggleSwitch", switch.Switch)

CONF_BLOWER = "blower"
CONF_PUMP1 = "pump1"
CONF_PUMP2 = "pump2"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CPComponent),
        cv.Optional(CONF_BLOWER): switch.switch_schema(BalboaToggleSwitch),
        cv.Optional(CONF_PUMP1): switch.switch_schema(BalboaToggleSwitch),
        cv.Optional(CONF_PUMP2): switch.switch_schema(BalboaToggleSwitch),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    if CONF_BLOWER in config:
        sw = await switch.new_switch(config[CONF_BLOWER])
        cg.add(sw.set_parent(parent))
        cg.add(sw.set_kind(1))  # blower
        cg.add(parent.set_blower_switch(sw))

    if CONF_PUMP1 in config:
        sw = await switch.new_switch(config[CONF_PUMP1])
        cg.add(sw.set_parent(parent))
        cg.add(sw.set_kind(2))  # pump1
        cg.add(parent.set_pump1_switch(sw))

    if CONF_PUMP2 in config:
        sw = await switch.new_switch(config[CONF_PUMP2])
        cg.add(sw.set_parent(parent))
        cg.add(sw.set_kind(3))  # pump2
        cg.add(parent.set_pump2_switch(sw))

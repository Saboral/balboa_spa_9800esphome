import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_ID

from . import (
    CONF_BALBOA_ID,
    Balboa9800CP,
    balboa_ns,
)

CONF_SETPOINT = "setpoint"

BalboaSetpointNumber = balboa_ns.class_("BalboaSetpointNumber", number.Number, cg.Component)

DEPENDENCIES = ["balboa_9800cp"]

# ESPHome 2025.11 deprecates *_SCHEMA constants; use number.number_schema when available.
try:
    _ENTITY_SCHEMA = number.number_schema(BalboaSetpointNumber)
except AttributeError:
    _ENTITY_SCHEMA = number.NUMBER_SCHEMA.extend({cv.GenerateID(): cv.declare_id(BalboaSetpointNumber)})

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CP),
        cv.Optional(CONF_SETPOINT): _ENTITY_SCHEMA.extend(
            {
                cv.Optional("min_value", default=80): cv.int_range(min=80, max=104),
                cv.Optional("max_value", default=104): cv.int_range(min=80, max=104),
                cv.Optional("step", default=1): cv.int_range(min=1, max=1),
            }
        ),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    if CONF_SETPOINT not in config:
        return

    conf = config[CONF_SETPOINT]
    min_v = conf.get("min_value", 80)
    max_v = conf.get("max_value", 104)
    step = conf.get("step", 1)

    if hasattr(number, "new_number"):
        var = await number.new_number(conf, min_value=min_v, max_value=max_v, step=step)
    else:
        var = cg.new_Pvariable(conf[CONF_ID])
        await number.register_number(var, conf, min_value=min_v, max_value=max_v, step=step)

    await cg.register_component(var, conf)

    cg.add(parent.set_setpoint_number(var))

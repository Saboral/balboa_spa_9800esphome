import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_ID

from . import CONF_BALBOA_ID, Balboa9800CP, balboa_ns

CONF_SETPOINT = "setpoint"

BalboaSetpointNumber = balboa_ns.class_("BalboaSetpointNumber", number.Number, cg.Component)

def _schema_for_number():
    if hasattr(number, "number_schema"):
        return number.number_schema(BalboaSetpointNumber)
    return number.NUMBER_SCHEMA.extend({cv.GenerateID(): cv.declare_id(BalboaSetpointNumber)})

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CP),
        cv.Optional(CONF_SETPOINT): _schema_for_number(),
    }
)

async def _new_number(conf):
    if hasattr(number, "new_number"):
        return await number.new_number(conf, min_value=80, max_value=104, step=1)
    return await cg.new_Pvariable(conf[CONF_ID])

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    if CONF_SETPOINT not in config:
        return

    conf = config[CONF_SETPOINT]
    var = await _new_number(conf)
    await cg.register_component(var, conf)

    # Register the number entity (signature differs across ESPHome versions)
    if hasattr(number, "register_number"):
        try:
            await number.register_number(var, conf, min_value=80, max_value=104, step=1)
        except TypeError:
            await number.register_number(var, conf)

    cg.add(parent.set_setpoint_number(var))

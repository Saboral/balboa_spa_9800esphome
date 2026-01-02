import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_ID

from . import Balboa9800CP, balboa_ns

CONF_BALBOA_ID = "balboa_id"
CONF_SETPOINT = "setpoint"

BalboaSetpointNumber = balboa_ns.class_("BalboaSetpointNumber", number.Number)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CP),
        cv.Required(CONF_SETPOINT): number.NUMBER_SCHEMA.extend(
            {cv.GenerateID(): cv.declare_id(BalboaSetpointNumber)}
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    cfg = config[CONF_SETPOINT]
    var = cg.new_Pvariable(cfg[CONF_ID])
    await number.register_number(var, cfg, min_value=80, max_value=104, step=1)
    cg.add(var.set_parent(parent))
    cg.add(parent.set_setpoint_number(var))

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button

from . import balboa_ns, Balboa9800CPComponent

CONF_BALBOA_ID = "balboa_id"
CONF_TEMP_UP = "temp_up"
CONF_TEMP_DOWN = "temp_down"

BalboaTempButton = balboa_ns.class_("BalboaTempButton", button.Button)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CPComponent),
        cv.Optional(CONF_TEMP_UP): button.button_schema(BalboaTempButton),
        cv.Optional(CONF_TEMP_DOWN): button.button_schema(BalboaTempButton),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    if CONF_TEMP_UP in config:
        b = await button.new_button(config[CONF_TEMP_UP])
        cg.add(b.set_parent(parent))
        cg.add(b.set_direction(1))  # up
    if CONF_TEMP_DOWN in config:
        b = await button.new_button(config[CONF_TEMP_DOWN])
        cg.add(b.set_parent(parent))
        cg.add(b.set_direction(2))  # down

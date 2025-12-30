import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button

from . import Balboa9800CP

CONF_BALBOA_ID = "balboa_id"
CONF_TEMP_UP = "temp_up"
CONF_TEMP_DOWN = "temp_down"
CONF_MODE = "mode"

balboa_ns = cg.esphome_ns.namespace("balboa_9800cp")
BalboaButton = balboa_ns.class_("BalboaButton", button.Button)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CP),
        cv.Optional(CONF_TEMP_UP): button.button_schema(BalboaButton),
        cv.Optional(CONF_TEMP_DOWN): button.button_schema(BalboaButton),
        cv.Optional(CONF_MODE): button.button_schema(BalboaButton),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    if CONF_TEMP_UP in config:
        b = await button.new_button(config[CONF_TEMP_UP])
        cg.add(b.set_command(1))
        cg.add(b.set_parent(parent))

    if CONF_TEMP_DOWN in config:
        b = await button.new_button(config[CONF_TEMP_DOWN])
        cg.add(b.set_command(2))
        cg.add(b.set_parent(parent))

    if CONF_MODE in config:
        b = await button.new_button(config[CONF_MODE])
        cg.add(b.set_command(3))
        cg.add(b.set_parent(parent))

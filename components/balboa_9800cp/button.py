import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID

balboa_ns = cg.esphome_ns.namespace("balboa_9800cp")
Balboa9800CP = balboa_ns.class_("Balboa9800CP", cg.Component)

BalboaButton = balboa_ns.class_("BalboaButton", button.Button)

CONF_TEMP_UP = "temp_up"
CONF_TEMP_DOWN = "temp_down"
CONF_MODE = "mode"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(Balboa9800CP),

        cv.Optional(CONF_TEMP_UP): button.button_schema(BalboaButton),
        cv.Optional(CONF_TEMP_DOWN): button.button_schema(BalboaButton),
        cv.Optional(CONF_MODE): button.button_schema(BalboaButton),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])

    if CONF_TEMP_UP in config:
        b = await button.new_button(config[CONF_TEMP_UP])
        cg.add(b.set_command(1))
        cg.add(parent.register_button(b))

    if CONF_TEMP_DOWN in config:
        b = await button.new_button(config[CONF_TEMP_DOWN])
        cg.add(b.set_command(2))
        cg.add(parent.register_button(b))

    if CONF_MODE in config:
        b = await button.new_button(config[CONF_MODE])
        cg.add(b.set_command(3))
        cg.add(parent.register_button(b))

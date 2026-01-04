import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID

from . import Balboa9800CP, CONF_BALBOA_ID

CONF_TEMP_UP = "temp_up"
CONF_TEMP_DOWN = "temp_down"
CONF_MODE = "mode"

balboa_ns = cg.esphome_ns.namespace("balboa_9800cp")
BalboaButton = balboa_ns.class_("BalboaButton", button.Button)

def _btn_schema():
    return button.button_schema(BalboaButton)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CP),
        cv.Optional(CONF_TEMP_UP): _btn_schema(),
        cv.Optional(CONF_TEMP_DOWN): _btn_schema(),
        cv.Optional(CONF_MODE): _btn_schema(),
    }
)

async def _add_button(parent, cfg, cmd):
    b = cg.new_Pvariable(cfg[CONF_ID])
    await button.register_button(b, cfg)
    cg.add(b.set_parent(parent))
    cg.add(b.set_cmd(cmd))
    return b

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    # Command IDs must match your C++ mapping
    CMD_TEMP_UP = 2
    CMD_TEMP_DOWN = 3
    CMD_MODE = 1

    if CONF_TEMP_UP in config:
        await _add_button(parent, config[CONF_TEMP_UP], CMD_TEMP_UP)

    if CONF_TEMP_DOWN in config:
        await _add_button(parent, config[CONF_TEMP_DOWN], CMD_TEMP_DOWN)

    if CONF_MODE in config:
        await _add_button(parent, config[CONF_MODE], CMD_MODE)

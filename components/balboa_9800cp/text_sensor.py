import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import Balboa9800CP

CONF_BALBOA_ID = "balboa_id"
CONF_DISPLAY = "display"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CP),
        cv.Required(CONF_DISPLAY): text_sensor.text_sensor_schema(),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])
    s = await text_sensor.new_text_sensor(config[CONF_DISPLAY])
    cg.add(parent.set_display_text_sensor(s))

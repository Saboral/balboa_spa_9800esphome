import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import Balboa9800CPComponent

CONF_BALBOA_ID = "balboa_id"
CONF_LCD_DISPLAY = "lcd_display"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CPComponent),
        cv.Optional(CONF_LCD_DISPLAY): text_sensor.text_sensor_schema(),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    if CONF_LCD_DISPLAY in config:
        ts = await text_sensor.new_text_sensor(config[CONF_LCD_DISPLAY])
        cg.add(parent.set_lcd_display_text_sensor(ts))

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

from . import Balboa9800CP, CONF_BALBOA_ID

CONF_WATER_TEMP = "water_temp"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CP),
        cv.Required(CONF_WATER_TEMP): sensor.sensor_schema(
            unit_of_measurement="°F",
            accuracy_decimals=0,
        ),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])
    s = await sensor.new_sensor(config[CONF_WATER_TEMP])
    cg.add(parent.set_water_temp_sensor(s))

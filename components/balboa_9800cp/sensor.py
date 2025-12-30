import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import DEVICE_CLASS_TEMPERATURE, STATE_CLASS_MEASUREMENT, UNIT_FAHRENHEIT

from . import Balboa9800CP

CONF_BALBOA_ID = "balboa_id"
CONF_WATER_TEMP_F = "water_temp_f"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CP),
        cv.Required(CONF_WATER_TEMP_F): sensor.sensor_schema(
            unit_of_measurement=UNIT_FAHRENHEIT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])
    s = await sensor.new_sensor(config[CONF_WATER_TEMP_F])
    cg.add(parent.set_water_temp_sensor(s))

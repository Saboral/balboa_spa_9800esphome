import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import UNIT_DEGREE_FAHRENHEIT, DEVICE_CLASS_TEMPERATURE, STATE_CLASS_MEASUREMENT

from . import DOMAIN, Balboa9800CPComponent

CONF_BALBOA_ID = "balboa_id"
CONF_WATER_TEMPERATURE = "water_temperature"
CONF_SET_TEMPERATURE = "set_temperature"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CPComponent),
        cv.Optional(CONF_WATER_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREE_FAHRENHEIT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SET_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREE_FAHRENHEIT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    if CONF_WATER_TEMPERATURE in config:
        s = await sensor.new_sensor(config[CONF_WATER_TEMPERATURE])
        cg.add(parent.set_water_temperature_sensor(s))

    if CONF_SET_TEMPERATURE in config:
        s = await sensor.new_sensor(config[CONF_SET_TEMPERATURE])
        cg.add(parent.set_set_temperature_sensor(s))

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import Balboa9800CP

CONF_BALBOA_ID = "balboa_id"
CONF_HEATING = "heating"
CONF_STANDARD_MODE = "standard_mode"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CP),
        cv.Optional(CONF_HEATING): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_STANDARD_MODE): binary_sensor.binary_sensor_schema(),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    if CONF_HEATING in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_HEATING])
        cg.add(parent.set_heating_sensor(s))

    if CONF_STANDARD_MODE in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_STANDARD_MODE])
        cg.add(parent.set_standard_mode_sensor(s))

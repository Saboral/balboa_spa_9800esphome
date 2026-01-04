import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID

from . import Balboa9800CP, CONF_BALBOA_ID

CONF_INVERTED = "inverted"
CONF_SET_HEAT = "set_heat"
CONF_MODE_STANDARD = "mode_standard"
CONF_HEATING = "heating"
CONF_TEMP_UP_DISPLAY = "temp_up_display"
CONF_TEMP_DOWN_DISPLAY = "temp_down_display"
CONF_BLOWER = "blower"
CONF_PUMP = "pump"
CONF_JETS = "jets"
CONF_LIGHT = "light"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CP),
        cv.Optional(CONF_INVERTED): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_SET_HEAT): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_MODE_STANDARD): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_HEATING): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_TEMP_UP_DISPLAY): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_TEMP_DOWN_DISPLAY): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_BLOWER): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_PUMP): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_JETS): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_LIGHT): binary_sensor.binary_sensor_schema(),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    if CONF_INVERTED in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_INVERTED])
        cg.add(parent.set_inverted_sensor(s))

    if CONF_SET_HEAT in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_SET_HEAT])
        cg.add(parent.set_set_heat_sensor(s))

    if CONF_MODE_STANDARD in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_MODE_STANDARD])
        cg.add(parent.set_mode_standard_sensor(s))

    if CONF_HEATING in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_HEATING])
        cg.add(parent.set_heating_sensor(s))

    if CONF_TEMP_UP_DISPLAY in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_TEMP_UP_DISPLAY])
        cg.add(parent.set_temp_up_display_sensor(s))

    if CONF_TEMP_DOWN_DISPLAY in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_TEMP_DOWN_DISPLAY])
        cg.add(parent.set_temp_down_display_sensor(s))

    if CONF_BLOWER in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_BLOWER])
        cg.add(parent.set_blower_sensor(s))

    if CONF_PUMP in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_PUMP])
        cg.add(parent.set_pump_sensor(s))

    if CONF_JETS in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_JETS])
        cg.add(parent.set_jets_sensor(s))

    if CONF_LIGHT in config:
        s = await binary_sensor.new_binary_sensor(config[CONF_LIGHT])
        cg.add(parent.set_light_sensor(s))

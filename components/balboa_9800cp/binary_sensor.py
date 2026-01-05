import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import Balboa9800CPComponent

CONF_BALBOA_ID = "balboa_id"

# Match your header variable names under // Status tracking
BIN_KEYS = {
    "display_button": "set_display_button_binary",
    "display_bit29": "set_display_bit29_binary",
    "display_bit30": "set_display_bit30_binary",
    "display_standard_mode": "set_display_standard_mode_binary",
    "display_bit32": "set_display_bit32_binary",
    "display_bit33": "set_display_bit33_binary",
    "display_bit34": "set_display_bit34_binary",
    "display_heater": "set_display_heater_binary",
    "display_pump1": "set_display_pump1_binary",
    "display_pump2": "set_display_pump2_binary",
    "display_air_blower": "set_display_air_blower_binary",
    "display_data_buffer_overflow": "set_display_overflow_binary",
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CPComponent),
        **{cv.Optional(k): binary_sensor.binary_sensor_schema() for k in BIN_KEYS.keys()},
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    for key, setter in BIN_KEYS.items():
        if key in config:
            bs = await binary_sensor.new_binary_sensor(config[key])
            cg.add(getattr(parent, setter)(bs))

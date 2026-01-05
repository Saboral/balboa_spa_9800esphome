import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import UNIT_DEGREE_FAHRENHEIT, DEVICE_CLASS_TEMPERATURE

from . import balboa_ns, Balboa9800CPComponent

CONF_BALBOA_ID = "balboa_id"
CONF_TARGET_TEMPERATURE = "target_temperature"

BalboaTargetTempNumber = balboa_ns.class_("BalboaTargetTempNumber", number.Number)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BALBOA_ID): cv.use_id(Balboa9800CPComponent),
        cv.Optional(CONF_TARGET_TEMPERATURE): number.number_schema(
            BalboaTargetTempNumber,
            unit_of_measurement=UNIT_DEGREE_FAHRENHEIT,
            device_class=DEVICE_CLASS_TEMPERATURE,
        ).extend(
            {
                cv.Optional("min_value", default=80): cv.float_,
                cv.Optional("max_value", default=104): cv.float_,
                cv.Optional("step", default=1): cv.float_,
            }
        ),
    }
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_BALBOA_ID])

    if CONF_TARGET_TEMPERATURE in config:
        n = await number.new_number(
            config[CONF_TARGET_TEMPERATURE],
            min_value=config[CONF_TARGET_TEMPERATURE]["min_value"],
            max_value=config[CONF_TARGET_TEMPERATURE]["max_value"],
            step=config[CONF_TARGET_TEMPERATURE]["step"],
        )
        cg.add(n.set_parent(parent))
        cg.add(parent.set_target_temp_number(n))

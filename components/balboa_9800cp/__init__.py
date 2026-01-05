import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins

DOMAIN = "balboa_9800cp"
balboa_ns = cg.global_ns.namespace("balboa_9800cp")  # <-- IMPORTANT

Balboa9800CPComponent = balboa_ns.class_("Balboa9800CPComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Balboa9800CPComponent),
        cv.Required("clock_pin"): pins.internal_gpio_input_pin_schema,
        cv.Required("read_pin"): pins.internal_gpio_input_pin_schema,
        cv.Required("write_pin"): pins.internal_gpio_output_pin_schema,
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[cv.GenerateID()])
    await cg.register_component(var, config)

    clock = await cg.gpio_pin_expression(config["clock_pin"])
    read = await cg.gpio_pin_expression(config["read_pin"])
    write = await cg.gpio_pin_expression(config["write_pin"])

    cg.add(var.set_clock_pin(clock))
    cg.add(var.set_read_pin(read))
    cg.add(var.set_write_pin(write))

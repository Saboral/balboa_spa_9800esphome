import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import (
    CONF_ID,
    CONF_NUMBER,
)

# NOTE: this external component exposes:
#  - a core component: balboa_9800cp:
#  - button platform: button.balboa_9800cp (existing in your repo)
#  - binary_sensor platform: binary_sensor.balboa_9800cp (existing in your repo)
#  - NEW: switch platform: switch.balboa_9800cp
#  - NEW: number platform: number.balboa_9800cp

CODEOWNERS = ["@Saboral"]

balboa_ns = cg.esphome_ns.namespace("balboa_9800cp")
Balboa9800CP = balboa_ns.class_("Balboa9800CP", cg.Component)

# These are helper entities implemented in C++ (balboa_9800cp_updated.h/.cpp)
BalboaToggleSwitch = balboa_ns.class_("BalboaToggleSwitch", cg.Component)  # declared for type reference only
BalboaSetpointNumber = balboa_ns.class_("BalboaSetpointNumber", cg.Component)  # declared for type reference only

CONF_CLK_PIN = "clk_pin"
CONF_DATA_PIN = "data_pin"
CONF_CTRL_IN_PIN = "ctrl_in_pin"
CONF_CTRL_OUT_PIN = "ctrl_out_pin"
CONF_GAP_US = "gap_us"
CONF_PRESS_FRAMES = "press_frames"

# ISR-safe raw pin numbers are pulled from pin schemas via CONF_NUMBER.
_PIN_SCHEMA_IN = pins.internal_gpio_input_pin_schema
_PIN_SCHEMA_OUT = pins.internal_gpio_output_pin_schema

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Balboa9800CP),
        cv.Required(CONF_CLK_PIN): _PIN_SCHEMA_IN,
        cv.Required(CONF_DATA_PIN): _PIN_SCHEMA_IN,
        cv.Required(CONF_CTRL_IN_PIN): _PIN_SCHEMA_IN,
        cv.Required(CONF_CTRL_OUT_PIN): _PIN_SCHEMA_OUT,
        cv.Optional(CONF_GAP_US, default=3000): cv.positive_int,
        cv.Optional(CONF_PRESS_FRAMES, default=3): cv.int_range(min=1, max=10),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    clk_pin = await cg.gpio_pin_expression(config[CONF_CLK_PIN])
    data_pin = await cg.gpio_pin_expression(config[CONF_DATA_PIN])
    ctrl_in_pin = await cg.gpio_pin_expression(config[CONF_CTRL_IN_PIN])
    ctrl_out_pin = await cg.gpio_pin_expression(config[CONF_CTRL_OUT_PIN])

    cg.add(var.set_pins(clk_pin, data_pin, ctrl_in_pin, ctrl_out_pin))

    # Pass raw GPIO numbers for ISR-safe gpio_get_level()
    cg.add(
        var.set_gpio_numbers(
            config[CONF_CLK_PIN][CONF_NUMBER],
            config[CONF_DATA_PIN][CONF_NUMBER],
            config[CONF_CTRL_IN_PIN][CONF_NUMBER],
            config[CONF_CTRL_OUT_PIN][CONF_NUMBER],
        )
    )

    cg.add(var.set_gap_us(config[CONF_GAP_US]))
    cg.add(var.set_press_frames(config[CONF_PRESS_FRAMES]))

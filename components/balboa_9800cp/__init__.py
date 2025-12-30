import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ID, CONF_NUMBER

CODEOWNERS = ["@Saboral"]

balboa_ns = cg.esphome_ns.namespace("balboa_9800cp")
Balboa9800CP = balboa_ns.class_("Balboa9800CP", cg.Component)

CONF_CLK_PIN = "clk_pin"
CONF_DATA_PIN = "data_pin"
CONF_CTRL_IN_PIN = "ctrl_in_pin"
CONF_CTRL_OUT_PIN = "ctrl_out_pin"
CONF_GAP_US = "gap_us"
CONF_PRESS_FRAMES = "press_frames"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Balboa9800CP),
            cv.Required(CONF_CLK_PIN): pins.gpio_input_pin_schema,
            cv.Required(CONF_DATA_PIN): pins.gpio_input_pin_schema,
            cv.Required(CONF_CTRL_IN_PIN): pins.gpio_input_pin_schema,
            cv.Required(CONF_CTRL_OUT_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_GAP_US, default=8000): cv.int_range(min=1000, max=50000),
            cv.Optional(CONF_PRESS_FRAMES, default=6): cv.int_range(min=1, max=30),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    clk = await cg.gpio_pin_expression(config[CONF_CLK_PIN])
    data = await cg.gpio_pin_expression(config[CONF_DATA_PIN])
    cin = await cg.gpio_pin_expression(config[CONF_CTRL_IN_PIN])
    cout = await cg.gpio_pin_expression(config[CONF_CTRL_OUT_PIN])

    cg.add(var.set_pins(clk, data, cin, cout))

    # Pass raw GPIO numbers for interrupt + open-drain injection (ESP32/Arduino attachInterrupt)
    cg.add(var.set_gpio_numbers(config[CONF_CLK_PIN][CONF_NUMBER],
                                config[CONF_CTRL_OUT_PIN][CONF_NUMBER]))

    cg.add(var.set_gap_us(config[CONF_GAP_US]))
    cg.add(var.set_press_frames(config[CONF_PRESS_FRAMES]))

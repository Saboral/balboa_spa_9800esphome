#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace balboa_9800cp {

class Balboa9800CP;

class BalboaButton : public button::Button {
 public:
  void set_command(uint8_t cmd) { this->cmd_ = cmd; }
  void set_parent(Balboa9800CP *p) { this->parent_ = p; }

 protected:
  void press_action() override;

 private:
  uint8_t cmd_{0};
  Balboa9800CP *parent_{nullptr};
};

class Balboa9800CP : public Component {
 public:
  // Wiring (ESPHome pin wrappers)
  void set_pins(GPIOPin *clk, GPIOPin *data, GPIOPin *ctrl_in, GPIOPin *ctrl_out);

  // Raw GPIO numbers (from YAML) — used for ISR-safe gpio_get_level() and ctrl_out driving
  void set_gpio_numbers(int clk_gpio, int data_gpio, int ctrl_in_gpio, int ctrl_out_gpio) {
    this->clk_gpio_ = clk_gpio;
    this->data_gpio_ = data_gpio;
    this->ctrl_in_gpio_ = ctrl_in_gpio;
    this->ctrl_out_gpio_ = ctrl_out_gpio;
  }

  void set_gap_us(uint32_t gap) { this->gap_us_ = gap; }
  void set_press_frames(uint8_t n) { this->press_frames_ = n; }

  // Entities (optional; can be wired later)
  void set_water_temp_sensor(sensor::Sensor *s) { this->water_temp_ = s; }
  void set_display_text_sensor(text_sensor::TextSensor *s) { this->display_text_ = s; }

  void setup() override;
  void loop() override;
  void dump_config() override;

  void queue_command(uint8_t cmd);

 protected:
  static void isr_router_();
  void on_clock_edge_();  // IRAM_ATTR on definition in .cpp
  void process_frame_();  // not used in Step-1 raw logger build, kept for later

  // ESPHome pin wrappers (not used inside ISR)
  GPIOPin *clk_{nullptr};
  GPIOPin *data_{nullptr};
  GPIOPin *ctrl_in_{nullptr};
  GPIOPin *ctrl_out_{nullptr};

  // Raw GPIO numbers (used inside ISR)
  int clk_gpio_{-1};
  int data_gpio_{-1};
  int ctrl_in_gpio_{-1};
  int ctrl_out_gpio_{-1};

  // Capture state
  volatile uint8_t bits_[76]{0};
  volatile uint32_t last_edge_us_{0};
  volatile int bit_index_{0};
  volatile bool frame_ready_{false};

  uint32_t gap_us_{8000};
  uint8_t press_frames_{6};

  // Injection state
  volatile uint8_t pending_cmd_{0};  // 1=Up, 2=Down, 3=Mode
  volatile uint8_t frames_left_{0};
  volatile bool release_frame_{false};

  // ISR instance for trampoline
  static Balboa9800CP *instance_;

  // Step 1 diagnostic
  volatile uint32_t isr_edge_count_{0};

  // Optional entities (unused in this Step-1 raw logger cpp)
  sensor::Sensor *water_temp_{nullptr};
  text_sensor::TextSensor *display_text_{nullptr};
};

}  // namespace balboa_9800cp
}  // namespace esphome

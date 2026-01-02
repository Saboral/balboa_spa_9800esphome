#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
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

class BalboaToggleSwitch : public switch_::Switch {
 public:
  void set_command(uint8_t cmd) { this->cmd_ = cmd; }
  void set_parent(Balboa9800CP *p) { this->parent_ = p; }

 protected:
  void write_state(bool state) override;

 private:
  uint8_t cmd_{0};
  Balboa9800CP *parent_{nullptr};
};

class BalboaSetpointNumber : public number::Number {
 public:
  void set_parent(Balboa9800CP *p) { this->parent_ = p; }

 protected:
  void control(float value) override;

 private:
  Balboa9800CP *parent_{nullptr};
};


class Balboa9800CP : public Component {
 public:
  void set_pins(GPIOPin *clk, GPIOPin *data, GPIOPin *ctrl_in, GPIOPin *ctrl_out);

  // Raw GPIO numbers (from YAML)
  void set_gpio_numbers(int clk_gpio, int data_gpio, int ctrl_in_gpio, int ctrl_out_gpio) {
    this->clk_gpio_ = clk_gpio;
    this->data_gpio_ = data_gpio;         // Display data (Pin 5)
    this->ctrl_in_gpio_ = ctrl_in_gpio;   // Control data (Pin 3)
    this->ctrl_out_gpio_ = ctrl_out_gpio;
  }

  void set_gap_us(uint32_t gap) { this->gap_us_ = gap; }
  void set_press_frames(uint8_t n) { this->press_frames_ = n; }

  // Entities (kept so your generated main.cpp compiles)
  void set_water_temp_sensor(sensor::Sensor *s) { this->water_temp_ = s; }
  void set_display_text_sensor(text_sensor::TextSensor *s) { this->display_text_ = s; }

  void set_inverted_sensor(binary_sensor::BinarySensor *s) { this->inverted_ = s; }
  void set_set_heat_sensor(binary_sensor::BinarySensor *s) { this->set_heat_ = s; }
  void set_mode_standard_sensor(binary_sensor::BinarySensor *s) { this->mode_standard_ = s; }
  void set_heating_sensor(binary_sensor::BinarySensor *s) { this->heating_ = s; }
  void set_temp_up_display_sensor(binary_sensor::BinarySensor *s) { this->temp_up_display_ = s; }
  void set_temp_down_display_sensor(binary_sensor::BinarySensor *s) { this->temp_down_display_ = s; }
  void set_blower_sensor(binary_sensor::BinarySensor *s) { this->blower_ = s; }
  void set_pump_sensor(binary_sensor::BinarySensor *s) { this->pump_ = s; }
  void set_jets_sensor(binary_sensor::BinarySensor *s) { this->jets_ = s; }
  void set_light_sensor(binary_sensor::BinarySensor *s) { this->light_ = s; }

  // NEW: Switches (control + state from bits 36/37/38)
  void set_pump1_switch(switch_::Switch *s) { this->pump1_switch_ = s; }
  void set_pump2_switch(switch_::Switch *s) { this->pump2_switch_ = s; }
  void set_blower_switch(switch_::Switch *s) { this->blower_switch_ = s; }

  // NEW: Setpoint number (80-104F, 1F steps). control() triggers sync/adjust sequence.
  void set_setpoint_number(number::Number *n) { this->setpoint_number_ = n; }

  void setup() override;
  void loop() override;
  void dump_config() override;

  void queue_command(uint8_t cmd);

  // NEW: Setpoint control request (invoked by BalboaSetpointNumber)
  void request_setpoint_(int target_f);

 protected:
  static void isr_router_();
  void on_clock_edge_();   // IRAM_ATTR on definition in .cpp
  void process_frame_();   // unused in this logger build

  GPIOPin *clk_{nullptr};
  GPIOPin *data_{nullptr};      // Display data pin wrapper
  GPIOPin *ctrl_in_{nullptr};   // Control data pin wrapper
  GPIOPin *ctrl_out_{nullptr};

  int clk_gpio_{-1};
  int data_gpio_{-1};        // Display data (Pin 5)
  int ctrl_in_gpio_{-1};     // Control data (Pin 3)
  int ctrl_out_gpio_{-1};

  // ---- NEW: Dual capture buffers ----
  volatile uint8_t disp_bits_[76]{0};   // sampled from data_gpio_ (Display Data / Pin 5)
  volatile uint8_t ctrl_bits_[76]{0};   // sampled from ctrl_in_gpio_ (Control Data / Pin 3)

  // Timing/capture state
  volatile uint32_t last_edge_us_{0};
  volatile int bit_index_{0};
  volatile bool frame_ready_{false};

  uint32_t gap_us_{8000};
  uint8_t press_frames_{6};

  // Injection state (not used in dual-stream logger build, kept for later)
  volatile uint8_t frames_left_{0};
  volatile bool release_frame_{false};

  static Balboa9800CP *instance_;

  // Debug
  volatile uint32_t isr_edge_count_{0};

  // Entity pointers (kept for later decode/publish)
  sensor::Sensor *water_temp_{nullptr};
  text_sensor::TextSensor *display_text_{nullptr};

  binary_sensor::BinarySensor *inverted_{nullptr};
  binary_sensor::BinarySensor *set_heat_{nullptr};
  binary_sensor::BinarySensor *mode_standard_{nullptr};
  binary_sensor::BinarySensor *heating_{nullptr};
  binary_sensor::BinarySensor *temp_up_display_{nullptr};
  binary_sensor::BinarySensor *temp_down_display_{nullptr};
  binary_sensor::BinarySensor *blower_{nullptr};
  binary_sensor::BinarySensor *pump_{nullptr};
  binary_sensor::BinarySensor *jets_{nullptr};
  binary_sensor::BinarySensor *light_{nullptr};

  // ---- NEW: control entities ----
  switch_::Switch *pump1_switch_{nullptr};
  switch_::Switch *pump2_switch_{nullptr};
  switch_::Switch *blower_switch_{nullptr};
  number::Number *setpoint_number_{nullptr};

  // ---- NEW: command injection state ----
  volatile uint8_t pending_cmd_{0};
  volatile uint8_t pending_frames_left_{0};
  uint32_t last_cmd_ms_{0};
  uint32_t last_temp_cmd_ms_{0};

  // ---- NEW: setpoint sync/adjust state ----
  bool setpoint_sync_active_{false};
  bool awaiting_setpoint_read_{false};
  int target_setpoint_f_{0};
  int current_setpoint_f_{0};
  bool current_setpoint_valid_{false};
  int pending_adjust_presses_{0};
  uint32_t next_adjust_ms_{0};
};

}  // namespace balboa_9800cp
}  // namespace esphome
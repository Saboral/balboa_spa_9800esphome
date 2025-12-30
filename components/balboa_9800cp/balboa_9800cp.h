#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"

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
  void set_pins(GPIOPin *clk, GPIOPin *data, GPIOPin *ctrl_in, GPIOPin *ctrl_out);
  void set_gap_us(uint32_t gap) { this->gap_us_ = gap; }
  void set_press_frames(uint8_t n) { this->press_frames_ = n; }

  void set_water_temp_sensor(sensor::Sensor *s) { this->water_temp_ = s; }
  void set_heating_sensor(binary_sensor::BinarySensor *s) { this->heating_ = s; }
  void set_standard_mode_sensor(binary_sensor::BinarySensor *s) { this->standard_mode_ = s; }

  void setup() override;
  void loop() override;

  void queue_command(uint8_t cmd);

 protected:
  static void IRAM_ATTR isr_router_();
  void IRAM_ATTR on_clock_edge_();

  void process_frame_();

  // Mirrors decoder.js :contentReference[oaicite:1]{index=1}
  char decode_digit_(uint8_t seg, bool inverted) const;
  void decode_display_(char out[5], bool &inverted) const;
  int convert_temp_(const char *disp) const;

  int get_bit1_(int bit_1_index) const;  // 1..76 -> 0/1

  GPIOPin *clk_{nullptr};
  GPIOPin *data_{nullptr};
  GPIOPin *ctrl_in_{nullptr};
  GPIOPin *ctrl_out_{nullptr};

  // 76 bits captured from DATA line (board->topside)
  volatile uint8_t bits_[76]{0};

  volatile uint32_t last_edge_us_{0};
  volatile int bit_index_{0};
  volatile bool frame_ready_{false};

  uint32_t gap_us_{8000};
  uint8_t press_frames_{6};

  // Controls injection state
  volatile uint8_t pending_cmd_{0};         // 0=none, 1=up, 2=down, 3=mode
  volatile uint8_t frames_left_{0};
  volatile bool release_frame_{false};

  static Balboa9800CP *instance_;

  // Entities
  sensor::Sensor *water_temp_{nullptr};
  binary_sensor::BinarySensor *heating_{nullptr};
  binary_sensor::BinarySensor *standard_mode_{nullptr};

  int last_temp_f_{-999};
};

}  // namespace balboa_9800cp
}  // namespace esphome

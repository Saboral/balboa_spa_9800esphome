#pragma once
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/button/button.h"
#include <vector>

namespace esphome {
namespace balboa_9800cp {

class BalboaButton : public button::Button {
 public:
  void set_command(uint8_t cmd) { this->cmd_ = cmd; }
  uint8_t get_command() const { return this->cmd_; }

 protected:
  void press_action() override;

 private:
  uint8_t cmd_{0};
};

class Balboa9800CP : public Component {
 public:
  void set_pins(GPIOPin *clk, GPIOPin *data, GPIOPin *ctrl_in, GPIOPin *ctrl_out);
  void set_frame_bits(int bits) { this->frame_bits_ = bits; }
  void set_gap_us(uint32_t gap) { this->gap_us_ = gap; }
  void set_press_frames(uint8_t n) { this->press_frames_ = n; }

  void set_water_temp_sensor(sensor::Sensor *s) { this->water_temp_ = s; }

  void register_button(BalboaButton *b) { this->buttons_.push_back(b); b->set_parent(this); }

  void setup() override;
  void loop() override;

  // Called by button instances
  void queue_command(uint8_t cmd);

  // For buttons to back-reference
  void set_parent_for_button(BalboaButton *b) {}

 protected:
  friend class BalboaButton;

  // ISR handler
  static void IRAM_ATTR isr_router_();
  void IRAM_ATTR on_clock_edge_();

  // decoding (outside ISR)
  void process_frame_();
  int decode_temp_from_segments_();  // implement once you map segments

  // Pins
  GPIOPin *clk_{nullptr};
  GPIOPin *data_{nullptr};
  GPIOPin *ctrl_in_{nullptr};
  GPIOPin *ctrl_out_{nullptr};

  // Config
  int frame_bits_{76};
  uint32_t gap_us_{8000};
  uint8_t press_frames_{6};

  // Runtime state (ISR-shared)
  volatile uint32_t last_edge_us_{0};
  volatile int bit_index_{0};

  static Balboa9800CP *instance_;  // single instance for ISR routing

  // Frame buffers (use bytes for speed)
  uint8_t data_bits_[128]{0};
  uint8_t ctrl_bits_in_[128]{0};

  // Frame ready flag
  volatile bool frame_ready_{false};

  // Command injection state
  volatile uint8_t pending_cmd_{0};        // 0 = none
  volatile uint8_t pending_frames_left_{0};

  // Entities
  sensor::Sensor *water_temp_{nullptr};
  std::vector<BalboaButton *> buttons_;

  // last published temp to avoid spam
  int last_temp_{-999};
};

}  // namespace balboa_9800cp
}  // namespace esphome

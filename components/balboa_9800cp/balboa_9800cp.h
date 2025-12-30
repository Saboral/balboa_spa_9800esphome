#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/sensor/sensor.h"
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
  void set_frame_bits(int bits) { this->frame_bits_ = bits; }
  void set_gap_us(uint32_t gap) { this->gap_us_ = gap; }
  void set_press_frames(uint8_t n) { this->press_frames_ = n; }

  void set_water_temp_sensor(sensor::Sensor *s) { this->water_temp_ = s; }

  void setup() override;
  void loop() override;

  // Called by buttons
  void queue_command(uint8_t cmd);

 protected:
  // ISR routing
  static void IRAM_ATTR isr_router_();
  void IRAM_ATTR on_clock_edge_();

  // Frame processing (outside ISR)
  void process_frame_();
  int decode_temp_from_segments_();

  // Pins
  GPIOPin *clk_{nullptr};
  GPIOPin *data_{nullptr};
  GPIOPin *ctrl_in_{nullptr};
  GPIOPin *ctrl_out_{nullptr};

  // Config
  int frame_bits_{76};
  uint32_t gap_us_{8000};
  uint8_t press_frames_{6};

  // ISR-shared state
  volatile uint32_t last_edge_us_{0};
  volatile int bit_index_{0};
  volatile bool frame_ready_{false};

  // Captured bits
  uint8_t data_bits_[128]{0};
  uint8_t ctrl_bits_in_[128]{0};

  // Command injection state
  volatile uint8_t pending_cmd_{0};           // 0 = none
  volatile uint8_t pending_frames_left_{0};   // frames remaining to hold button pattern
  volatile bool release_frame_{false};        // one steady (0000) release frame

  // Single instance for ISR
  static Balboa9800CP *instance_;

  // Entities
  sensor::Sensor *water_temp_{nullptr};
  int last_temp_{-999};
};

}  // namespace balboa_9800cp
}  // namespace esphome

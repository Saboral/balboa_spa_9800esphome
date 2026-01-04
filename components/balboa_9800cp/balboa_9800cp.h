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

/**
 * Simple "button" entity used by your existing codegen.
 * It calls parent_->queue_command(cmd_) when pressed.
 */
class BalboaButton : public button::Button {
 public:
  void set_parent(Balboa9800CP *parent) { parent_ = parent; }
  void set_cmd(uint8_t cmd) { cmd_ = cmd; }

 protected:
  void press_action() override;

  Balboa9800CP *parent_{nullptr};
  uint8_t cmd_{0};
};

class Balboa9800CP : public Component {
 public:
  // --- Wiring ---
  void set_pins(GPIOPin *clk, GPIOPin *data, GPIOPin *ctrl_in, GPIOPin *ctrl_out);

  // --- Existing entities (your project already uses these setters in codegen) ---
  void set_water_temp_sensor(sensor::Sensor *s) { water_temp_ = s; }
  void set_display_text(text_sensor::TextSensor *t) { display_text_ = t; }
  void set_binary_sensors(binary_sensor::BinarySensor *inverted,
                          binary_sensor::BinarySensor *set_heat,
                          binary_sensor::BinarySensor *mode_standard,
                          binary_sensor::BinarySensor *heating,
                          binary_sensor::BinarySensor *temp_up_display,
                          binary_sensor::BinarySensor *temp_down_display,
                          binary_sensor::BinarySensor *blower,
                          binary_sensor::BinarySensor *pump,
                          binary_sensor::BinarySensor *jets,
                          binary_sensor::BinarySensor *light);

  // Command entrypoint used by BalboaButton (and your switches/numbers if present)
  void queue_command(uint8_t cmd);

  void setup() override;
  void loop() override;
  void dump_config() override {}

  // Singleton used by ISR router
  static Balboa9800CP *instance_;

 protected:
  // ===== Command injector (GS510SZ / Balboa_GS_Interface style) =====
  static constexpr uint32_t INTER_PRESS_DELAY_MS = 500;
  static constexpr uint8_t CMD_Q_SIZE = 16;

  uint8_t cmd_q_[CMD_Q_SIZE]{};
  volatile uint8_t q_head_{0};
  volatile uint8_t q_tail_{0};

  volatile bool press_active_{false};
  volatile uint8_t active_cmd_{0};
  volatile uint8_t active_press_frames_left_{0};
  volatile bool press_ended_flag_{false};
  uint32_t last_press_end_ms_{0};

  bool q_empty_() const { return q_head_ == q_tail_; }
  bool q_full_() const { return (uint8_t) ((q_tail_ + 1) % CMD_Q_SIZE) == q_head_; }
  bool q_push_(uint8_t cmd);
  bool q_pop_(uint8_t &cmd);

  void start_press_(uint8_t cmd);
  void service_queue_();

  // Maps active_cmd_ to the required (b39,b40,b41) bits.
  // Returns true=drive HIGH, false=drive LOW during that bit time.
  bool command_bit_level_(uint8_t cmd, int bit_index) const;

  // ===== ISR capture =====
  static void IRAM_ATTR isr_router_();
  void IRAM_ATTR on_clock_edge_();

  // Capture config/state
  GPIOPin *clk_{nullptr};
  GPIOPin *data_{nullptr};
  GPIOPin *ctrl_in_{nullptr};
  GPIOPin *ctrl_out_{nullptr};

  int clk_gpio_{-1};
  int data_gpio_{-1};
  int ctrl_in_gpio_{-1};
  int ctrl_out_gpio_{-1};

  volatile uint8_t disp_bits_[42]{};
  volatile uint8_t ctrl_bits_[42]{};
  volatile int bit_index_{0};
  volatile bool frame_ready_{false};

  volatile uint32_t last_edge_us_{0};
  uint32_t gap_us_{1500};

  volatile uint32_t isr_edge_count_{0};

  // existing tunables already present in your cpp
  uint8_t press_frames_{2};

  // Entities
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

  // processing (kept as in your original file)
  void process_frame_();
};

}  // namespace balboa_9800cp
}  // namespace esphome

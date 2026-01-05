#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"

#include "balboa_9800cp.h"
#include "platform_entities.h"

namespace balboa_9800cp {

class Balboa9800CPComponent : public esphome::public Component {
 public:
  void set_clock_pin(GPIOPin *pin) { this->clock_pin_ = pin; }
  void set_read_pin(GPIOPin *pin) { this->read_pin_ = pin; }
  void set_write_pin(GPIOPin *pin) { this->write_pin_ = pin; }

  // Status entities
  void set_water_temperature_sensor(sensor::Sensor *s) { this->water_temperature_sensor_ = s; }
  void set_set_temperature_sensor(sensor::Sensor *s) { this->set_temperature_sensor_ = s; }
  void set_lcd_display_text_sensor(text_sensor::TextSensor *s) { this->lcd_display_text_sensor_ = s; }

  void set_display_button_binary(binary_sensor::BinarySensor *s) { this->display_button_binary_ = s; }
  void set_display_bit29_binary(binary_sensor::BinarySensor *s) { this->display_bit29_binary_ = s; }
  void set_display_bit30_binary(binary_sensor::BinarySensor *s) { this->display_bit30_binary_ = s; }
  void set_display_standard_mode_binary(binary_sensor::BinarySensor *s) { this->display_standard_mode_binary_ = s; }
  void set_display_bit32_binary(binary_sensor::BinarySensor *s) { this->display_bit32_binary_ = s; }
  void set_display_bit33_binary(binary_sensor::BinarySensor *s) { this->display_bit33_binary_ = s; }
  void set_display_bit34_binary(binary_sensor::BinarySensor *s) { this->display_bit34_binary_ = s; }
  void set_display_heater_binary(binary_sensor::BinarySensor *s) { this->display_heater_binary_ = s; }
  void set_display_pump1_binary(binary_sensor::BinarySensor *s) { this->display_pump1_binary_ = s; }
  void set_display_pump2_binary(binary_sensor::BinarySensor *s) { this->display_pump2_binary_ = s; }
  void set_display_air_blower_binary(binary_sensor::BinarySensor *s) { this->display_air_blower_binary_ = s; }
  void set_display_overflow_binary(binary_sensor::BinarySensor *s) { this->display_overflow_binary_ = s; }

  // Control entities
  void set_blower_switch(switch_::Switch *sw) { this->blower_switch_ = sw; }
  void set_pump1_switch(switch_::Switch *sw) { this->pump1_switch_ = sw; }
  void set_pump2_switch(switch_::Switch *sw) { this->pump2_switch_ = sw; }
  void set_target_temp_number(number::Number *num) { this->target_temp_number_ = num; }

  // Actions called by platforms
  void press_temp_up();
  void press_temp_down();

  void request_blower_state(bool on);
  void request_pump1_state(bool on);
  void request_pump2_state(bool on);

  void set_target_temperature(float temp_f);

  // Used by switches to decide if a press is needed
  bool is_blower_on() const { return this->iface_ != nullptr && this->iface_->displayAirBlower; }
  bool is_pump1_on() const { return this->iface_ != nullptr && this->iface_->displayPump1; }
  bool is_pump2_on() const { return this->iface_ != nullptr && this->iface_->displayPump2; }

  void setup() override;
  void loop() override;

 protected:
  GPIOPin *clock_pin_{nullptr};
  GPIOPin *read_pin_{nullptr};
  GPIOPin *write_pin_{nullptr};

  BalboaInterface *iface_{nullptr};

  // status entity pointers
  sensor::Sensor *water_temperature_sensor_{nullptr};
  sensor::Sensor *set_temperature_sensor_{nullptr};
  text_sensor::TextSensor *lcd_display_text_sensor_{nullptr};

  binary_sensor::BinarySensor *display_button_binary_{nullptr};
  binary_sensor::BinarySensor *display_bit29_binary_{nullptr};
  binary_sensor::BinarySensor *display_bit30_binary_{nullptr};
  binary_sensor::BinarySensor *display_standard_mode_binary_{nullptr};
  binary_sensor::BinarySensor *display_bit32_binary_{nullptr};
  binary_sensor::BinarySensor *display_bit33_binary_{nullptr};
  binary_sensor::BinarySensor *display_bit34_binary_{nullptr};
  binary_sensor::BinarySensor *display_heater_binary_{nullptr};
  binary_sensor::BinarySensor *display_pump1_binary_{nullptr};
  binary_sensor::BinarySensor *display_pump2_binary_{nullptr};
  binary_sensor::BinarySensor *display_air_blower_binary_{nullptr};
  binary_sensor::BinarySensor *display_overflow_binary_{nullptr};

  // control entity pointers (so we can keep them in sync from display data)
  switch_::Switch *blower_switch_{nullptr};
  switch_::Switch *pump1_switch_{nullptr};
  switch_::Switch *pump2_switch_{nullptr};
  number::Number *target_temp_number_{nullptr};

  // last-published cache
  float last_water_temp_{NAN};
  float last_set_temp_{NAN};
  std::string last_lcd_;

  bool last_display_button_{false};
  bool last_bit29_{false};
  bool last_bit30_{false};
  bool last_standard_{false};
  bool last_bit32_{false};
  bool last_bit33_{false};
  bool last_bit34_{false};
  bool last_heater_{false};
  bool last_pump1_{false};
  bool last_pump2_{false};
  bool last_blower_{false};
  bool last_overflow_{false};

  uint32_t last_publish_ms_{0};

  void publish_if_changed_();
};

}  // namespace balboa_9800cp

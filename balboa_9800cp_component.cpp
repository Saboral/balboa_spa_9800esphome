#include "balboa_9800cp_component.h"
#include "esphome/core/log.h"

namespace balboa_9800cp {

static const char *const TAG = "balboa_9800cp";

void Balboa9800CPComponent::setup() {
  if (this->clock_pin_ == nullptr || this->read_pin_ == nullptr || this->write_pin_ == nullptr) {
    ESP_LOGE(TAG, "Pins not set");
    return;
  }

  // Ensure pins are initialized the ESPHome way
  this->clock_pin_->setup();
  this->read_pin_->setup();
  this->write_pin_->setup();

  const uint8_t clock = this->clock_pin_->get_pin();
  const uint8_t read = this->read_pin_->get_pin();
  const uint8_t write = this->write_pin_->get_pin();

  this->iface_ = new BalboaInterface(clock, read, write);
  this->iface_->begin();

  // Initialize cache with "unknown" so first publish happens quickly
  this->last_publish_ms_ = 0;
}

void Balboa9800CPComponent::loop() {
  if (this->iface_ == nullptr) return;

  // Run the underlying interface loop as often as possible.
  bool ok = this->iface_->loop();
  if (!ok) return;

  // Publish at most ~10Hz even if loop() returns true frequently
  const uint32_t now = millis();
  if (now - this->last_publish_ms_ < 100) return;
  this->last_publish_ms_ = now;

  this->publish_if_changed_();
}

void Balboa9800CPComponent::publish_if_changed_() {
  // Sensors
  if (this->water_temperature_sensor_ != nullptr) {
    float v = this->iface_->waterTemperature;
    if (isnan(this->last_water_temp_) || v != this->last_water_temp_) {
      this->last_water_temp_ = v;
      this->water_temperature_sensor_->publish_state(v);
    }
  }

  if (this->set_temperature_sensor_ != nullptr) {
    float v = this->iface_->setTemperature;
    if (isnan(this->last_set_temp_) || v != this->last_set_temp_) {
      this->last_set_temp_ = v;
      this->set_temperature_sensor_->publish_state(v);

      // Keep the Number entity in sync with display setpoint
      if (this->target_temp_number_ != nullptr) {
        this->target_temp_number_->publish_state(v);
      }
    }
  }

  if (this->lcd_display_text_sensor_ != nullptr) {
    std::string lcd = this->iface_->LCD_display.c_str();
    if (lcd != this->last_lcd_) {
      this->last_lcd_ = lcd;
      this->lcd_display_text_sensor_->publish_state(lcd);
    }
  }

  // Binary sensors
  auto pub_bin = [](binary_sensor::BinarySensor *s, bool &last, bool cur) {
    if (s == nullptr) return;
    if (cur != last) {
      last = cur;
      s->publish_state(cur);
    }
  };

  pub_bin(this->display_button_binary_, this->last_display_button_, this->iface_->displayButton);
  pub_bin(this->display_bit29_binary_, this->last_bit29_, this->iface_->displayBit29);
  pub_bin(this->display_bit30_binary_, this->last_bit30_, this->iface_->displayBit30);
  pub_bin(this->display_standard_mode_binary_, this->last_standard_, this->iface_->displayStandardMode);
  pub_bin(this->display_bit32_binary_, this->last_bit32_, this->iface_->displayBit32);
  pub_bin(this->display_bit33_binary_, this->last_bit33_, this->iface_->displayBit33);
  pub_bin(this->display_bit34_binary_, this->last_bit34_, this->iface_->displayBit34);
  pub_bin(this->display_heater_binary_, this->last_heater_, this->iface_->displayHeater);
  pub_bin(this->display_pump1_binary_, this->last_pump1_, this->iface_->displayPump1);
  pub_bin(this->display_pump2_binary_, this->last_pump2_, this->iface_->displayPump2);
  pub_bin(this->display_air_blower_binary_, this->last_blower_, this->iface_->displayAirBlower);
  pub_bin(this->display_overflow_binary_, this->last_overflow_, BalboaInterface::displayDataBufferOverflow);

  // Keep HA switches in sync with display states (authoritative)
  if (this->blower_switch_ != nullptr) this->blower_switch_->publish_state(this->iface_->displayAirBlower);
  if (this->pump1_switch_ != nullptr) this->pump1_switch_->publish_state(this->iface_->displayPump1);
  if (this->pump2_switch_ != nullptr) this->pump2_switch_->publish_state(this->iface_->displayPump2);
}

// -------- Controls (write flags) --------

void Balboa9800CPComponent::press_temp_up() {
  BalboaInterface::writeDisplayData = true;
  BalboaInterface::writeTempUp = true;
}

void Balboa9800CPComponent::press_temp_down() {
  BalboaInterface::writeDisplayData = true;
  BalboaInterface::writeTempDown = true;
}

void Balboa9800CPComponent::request_blower_state(bool on) {
  if (this->iface_ == nullptr) return;
  const bool cur = this->iface_->displayAirBlower;
  if (cur == on) return;  // already in desired state
  BalboaInterface::writeDisplayData = true;
  BalboaInterface::writeBlower = true;  // press-to-toggle
}

void Balboa9800CPComponent::request_pump1_state(bool on) {
  if (this->iface_ == nullptr) return;
  const bool cur = this->iface_->displayPump1;
  if (cur == on) return;
  BalboaInterface::writeDisplayData = true;
  BalboaInterface::writePump1 = true;  // press-to-toggle
}

void Balboa9800CPComponent::request_pump2_state(bool on) {
  if (this->iface_ == nullptr) return;
  const bool cur = this->iface_->displayPump2;
  if (cur == on) return;
  BalboaInterface::writeDisplayData = true;
  BalboaInterface::writePump2 = true;  // press-to-toggle
}

void Balboa9800CPComponent::set_target_temperature(float temp_f) {
  if (this->iface_ == nullptr) return;
  // Underlying logic calculates how many temp-up/down "presses" are needed
  this->iface_->updateTemperature(temp_f);
}

}  // namespace balboa_9800cp


#ifndef Balboa_9800CP_Interface_h
#define Balboa_9800CP_Interface_h

#include <Arduino.h>


const byte displayDataBufferSize       		= 40;		// Size of display data buffer
const byte displayDataBits             		= 38;       // 0-38 bits length of display data within a cycle
const byte buttonDataBits              		= 2;        // 0-2 bits length of button data within a cycle
const byte totalDataBits               		= 41;       // 0-41 total number of pulses within a cycle
const unsigned int durationNewCycle    		= 5000;		// How many microsecounds to detect new cycle if no interrupt occurs 
const unsigned long buttonPressTimerMillis  = 500;  	// Timer in milliseconds between update temperature button presses 


class BalboaInterface {

  public:
	
	BalboaInterface(byte setClockPin, byte setReadPin, byte setWritePin);
	
	// Interface control
	void begin();						           	// Initializes the stream output to Serial by default
    bool loop();                                    // Returns true if valid data is available
    void stop();                                    // Disables the clock hardware interrupt 
    void resetStatus();                             // Resets the state of all status components as changed for sketches to get the current status	
	void updateTemperature(float Temperature);		// Function to set the water temperature 	

	// Status tracking
	float waterTemperature;               // Water temperature
	float setTemperature;                 // The wanted set temperature   
	String LCD_display;								    // The text shown on display 
	bool displayButton;        						// Temp up/down button pressed
	bool displayBit29;        						// Still unknown functionality, if at all used!
	bool displayBit30;        						// Still unknown functionality, if at all used!
	bool displayStandardMode;        			// Standard Mode activated or not
	bool displayBit32;        						// Still unknown functionality, if at all used!
	bool displayBit33;        						// Still unknown functionality, if at all used!  
	bool displayBit34;        						// Still unknown functionality, if at all used!
	bool displayHeater;        						// Heater running or not
	bool displayPump1;        						// Pump 1 running or not 
	bool displayPump2;        						// Pump 2 running or not
	bool displayAirBlower;        			  // Hot Blower running or not
// Backwards-compat alias (some code uses displayBlower)
#ifndef displayBlower
#define displayBlower displayAirBlower
#endif

	static bool displayDataBufferOverflow;
	
	// Write button data to control unit  
	static bool writeDisplayData;            		// If something should be written to button data line  
	static bool writeMode;
	static bool writeTempUp;
	static bool writeTempDown;
	static bool writeBlower;
	static bool writePump1;
	static bool writePump2;		
	static bool writePump3;

  private:
	
	static void clockPinInterrupt();
	void decodeDisplayData();
	String lockup_LCD_character(int LCD_character);
	int LCD_segment_1;
	int LCD_segment_2;
	int LCD_segment_3;
	int LCD_segment_4;
	String LCD_display_1;
	String LCD_display_2;
	String LCD_display_3;
	String LCD_display_4;  
	static byte displayDataBuffer[displayDataBufferSize]; 	// Array of display data measurements 
	static unsigned long clockInterruptTime;
	static int clockBitCounter;               		 		// Counter of pulses within a cycle
	static byte dataIndex; 									
	static bool displayDataBufferReady;            			// Is buffer available to be decoded
	static byte clockPin;
    static byte displayPin;
    static byte buttonPin;

	int updateTempDirection;
	int updateTempButtonPresses;
	unsigned long buttonPressTimerPrevMillis; 
};

// ===================== ESPHome Wrapper Types =====================
// These wrappers expose BalboaInterface status/control to ESPHome.
// They do not change BalboaInterface logic.

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"

namespace balboa_9800cp {

class Balboa9800CPComponent : public esphome::Component {
 public:
  void set_clock_pin(esphome::GPIOPin *pin) { this->clock_pin_ = pin; }
  void set_read_pin(esphome::GPIOPin *pin) { this->read_pin_ = pin; }
  void set_write_pin(esphome::GPIOPin *pin) { this->write_pin_ = pin; }

  // Status entities
  void set_water_temperature_sensor(esphome::sensor::Sensor *s) { this->water_temperature_sensor_ = s; }
  void set_set_temperature_sensor(esphome::sensor::Sensor *s) { this->set_temperature_sensor_ = s; }
  void set_lcd_display_text_sensor(esphome::text_sensor::TextSensor *s) { this->lcd_display_text_sensor_ = s; }

  void set_display_button_binary(esphome::binary_sensor::BinarySensor *s) { this->display_button_binary_ = s; }
  void set_display_bit29_binary(esphome::binary_sensor::BinarySensor *s) { this->display_bit29_binary_ = s; }
  void set_display_bit30_binary(esphome::binary_sensor::BinarySensor *s) { this->display_bit30_binary_ = s; }
  void set_display_standard_mode_binary(esphome::binary_sensor::BinarySensor *s) { this->display_standard_mode_binary_ = s; }
  void set_display_bit32_binary(esphome::binary_sensor::BinarySensor *s) { this->display_bit32_binary_ = s; }
  void set_display_bit33_binary(esphome::binary_sensor::BinarySensor *s) { this->display_bit33_binary_ = s; }
  void set_display_bit34_binary(esphome::binary_sensor::BinarySensor *s) { this->display_bit34_binary_ = s; }
  void set_display_heater_binary(esphome::binary_sensor::BinarySensor *s) { this->display_heater_binary_ = s; }
  void set_display_pump1_binary(esphome::binary_sensor::BinarySensor *s) { this->display_pump1_binary_ = s; }
  void set_display_pump2_binary(esphome::binary_sensor::BinarySensor *s) { this->display_pump2_binary_ = s; }
  void set_display_air_blower_binary(esphome::binary_sensor::BinarySensor *s) { this->display_air_blower_binary_ = s; }
  void set_display_overflow_binary(esphome::binary_sensor::BinarySensor *s) { this->display_overflow_binary_ = s; }

  // Controls (optional, for sync)
  void set_blower_switch(esphome::switch_::Switch *sw) { this->blower_switch_ = sw; }
  void set_pump1_switch(esphome::switch_::Switch *sw) { this->pump1_switch_ = sw; }
  void set_pump2_switch(esphome::switch_::Switch *sw) { this->pump2_switch_ = sw; }
  void set_target_temp_number(esphome::number::Number *num) { this->target_temp_number_ = num; }

  void setup() override {
    if (this->clock_pin_ == nullptr || this->read_pin_ == nullptr || this->write_pin_ == nullptr) {
      ESP_LOGE("balboa_9800cp", "Pins not set");
      return;
    }
    this->clock_pin_->setup();
    this->read_pin_->setup();
    this->write_pin_->setup();

    const uint8_t clock = this->clock_pin_->get_pin();
    const uint8_t read = this->read_pin_->get_pin();
    const uint8_t write = this->write_pin_->get_pin();

    this->iface_ = new BalboaInterface(clock, read, write);
    this->iface_->begin();
  }

  void loop() override {
    if (this->iface_ == nullptr) return;
    if (!this->iface_->loop()) return;

    if (this->water_temperature_sensor_ != nullptr) this->water_temperature_sensor_->publish_state(this->iface_->waterTemperature);

    if (this->set_temperature_sensor_ != nullptr) {
      this->set_temperature_sensor_->publish_state(this->iface_->setTemperature);
      if (this->target_temp_number_ != nullptr) this->target_temp_number_->publish_state(this->iface_->setTemperature);
    }

    if (this->lcd_display_text_sensor_ != nullptr) this->lcd_display_text_sensor_->publish_state(this->iface_->LCD_display.c_str());

    if (this->display_button_binary_ != nullptr) this->display_button_binary_->publish_state(this->iface_->displayButton);
    if (this->display_bit29_binary_ != nullptr) this->display_bit29_binary_->publish_state(this->iface_->displayBit29);
    if (this->display_bit30_binary_ != nullptr) this->display_bit30_binary_->publish_state(this->iface_->displayBit30);
    if (this->display_standard_mode_binary_ != nullptr) this->display_standard_mode_binary_->publish_state(this->iface_->displayStandardMode);
    if (this->display_bit32_binary_ != nullptr) this->display_bit32_binary_->publish_state(this->iface_->displayBit32);
    if (this->display_bit33_binary_ != nullptr) this->display_bit33_binary_->publish_state(this->iface_->displayBit33);
    if (this->display_bit34_binary_ != nullptr) this->display_bit34_binary_->publish_state(this->iface_->displayBit34);
    if (this->display_heater_binary_ != nullptr) this->display_heater_binary_->publish_state(this->iface_->displayHeater);
    if (this->display_pump1_binary_ != nullptr) this->display_pump1_binary_->publish_state(this->iface_->displayPump1);
    if (this->display_pump2_binary_ != nullptr) this->display_pump2_binary_->publish_state(this->iface_->displayPump2);
    if (this->display_air_blower_binary_ != nullptr) this->display_air_blower_binary_->publish_state(this->iface_->displayAirBlower);
    if (this->display_overflow_binary_ != nullptr) this->display_overflow_binary_->publish_state(BalboaInterface::displayDataBufferOverflow);

    // HA switch state should reflect display (authoritative)
    if (this->blower_switch_ != nullptr) this->blower_switch_->publish_state(this->iface_->displayAirBlower);
    if (this->pump1_switch_ != nullptr) this->pump1_switch_->publish_state(this->iface_->displayPump1);
    if (this->pump2_switch_ != nullptr) this->pump2_switch_->publish_state(this->iface_->displayPump2);
  }

  // Controls
  void press_temp_up() {
    BalboaInterface::writeDisplayData = true;
    BalboaInterface::writeTempUp = true;
  }

  void press_temp_down() {
    BalboaInterface::writeDisplayData = true;
    BalboaInterface::writeTempDown = true;
  }

  void request_blower_state(bool on) {
    if (this->iface_ == nullptr) return;
    if (this->iface_->displayAirBlower == on) return;
    BalboaInterface::writeDisplayData = true;
    BalboaInterface::writeBlower = true;
  }

  void request_pump1_state(bool on) {
    if (this->iface_ == nullptr) return;
    if (this->iface_->displayPump1 == on) return;
    BalboaInterface::writeDisplayData = true;
    BalboaInterface::writePump1 = true;
  }

  void request_pump2_state(bool on) {
    if (this->iface_ == nullptr) return;
    if (this->iface_->displayPump2 == on) return;
    BalboaInterface::writeDisplayData = true;
    BalboaInterface::writePump2 = true;
  }

  void set_target_temperature(float temp_f) {
    if (this->iface_ == nullptr) return;
    this->iface_->updateTemperature(temp_f);
  }

 protected:
  esphome::GPIOPin *clock_pin_{nullptr};
  esphome::GPIOPin *read_pin_{nullptr};
  esphome::GPIOPin *write_pin_{nullptr};

  BalboaInterface *iface_{nullptr};

  esphome::sensor::Sensor *water_temperature_sensor_{nullptr};
  esphome::sensor::Sensor *set_temperature_sensor_{nullptr};
  esphome::text_sensor::TextSensor *lcd_display_text_sensor_{nullptr};

  esphome::binary_sensor::BinarySensor *display_button_binary_{nullptr};
  esphome::binary_sensor::BinarySensor *display_bit29_binary_{nullptr};
  esphome::binary_sensor::BinarySensor *display_bit30_binary_{nullptr};
  esphome::binary_sensor::BinarySensor *display_standard_mode_binary_{nullptr};
  esphome::binary_sensor::BinarySensor *display_bit32_binary_{nullptr};
  esphome::binary_sensor::BinarySensor *display_bit33_binary_{nullptr};
  esphome::binary_sensor::BinarySensor *display_bit34_binary_{nullptr};
  esphome::binary_sensor::BinarySensor *display_heater_binary_{nullptr};
  esphome::binary_sensor::BinarySensor *display_pump1_binary_{nullptr};
  esphome::binary_sensor::BinarySensor *display_pump2_binary_{nullptr};
  esphome::binary_sensor::BinarySensor *display_air_blower_binary_{nullptr};
  esphome::binary_sensor::BinarySensor *display_overflow_binary_{nullptr};

  esphome::switch_::Switch *blower_switch_{nullptr};
  esphome::switch_::Switch *pump1_switch_{nullptr};
  esphome::switch_::Switch *pump2_switch_{nullptr};
  esphome::number::Number *target_temp_number_{nullptr};
};

// Switch wrapper (press-to-toggle; state is confirmed from display data)
class BalboaToggleSwitch : public esphome::switch_::Switch {
 public:
  void set_parent(Balboa9800CPComponent *p) { parent_ = p; }
  void set_kind(uint8_t k) { kind_ = k; }  // 1=blower,2=pump1,3=pump2

 protected:
  void write_state(bool state) override {
    if (!parent_) return;
    if (kind_ == 1) parent_->request_blower_state(state);
    else if (kind_ == 2) parent_->request_pump1_state(state);
    else if (kind_ == 3) parent_->request_pump2_state(state);
  }

  Balboa9800CPComponent *parent_{nullptr};
  uint8_t kind_{0};
};

// Button wrapper
class BalboaTempButton : public esphome::button::Button {
 public:
  void set_parent(Balboa9800CPComponent *p) { parent_ = p; }
  void set_direction(uint8_t d) { dir_ = d; }  // 1=up,2=down

 protected:
  void press_action() override {
    if (!parent_) return;
    if (dir_ == 1) parent_->press_temp_up();
    else if (dir_ == 2) parent_->press_temp_down();
  }

  Balboa9800CPComponent *parent_{nullptr};
  uint8_t dir_{0};
};

// Number wrapper
class BalboaTargetTempNumber : public esphome::number::Number {
 public:
  void set_parent(Balboa9800CPComponent *p) { parent_ = p; }

 protected:
  void control(float value) override {
    if (!parent_) return;
    parent_->set_target_temperature(value);
  }

  Balboa9800CPComponent *parent_{nullptr};
};

}  // namespace balboa_9800cp


#endif  // Balboa_9800CP_Interface_h
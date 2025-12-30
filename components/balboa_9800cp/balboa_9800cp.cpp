#include "balboa_9800cp.h"
#include "esphome/core/log.h"
#include <Arduino.h>

namespace esphome {
namespace balboa_9800cp {

static const char *const TAG = "balboa_9800cp";

Balboa9800CP *Balboa9800CP::instance_ = nullptr;

void BalboaButton::press_action() {
  if (this->parent_ != nullptr) {
    this->parent_->queue_command(this->cmd_);
  }
}

void Balboa9800CP::set_pins(GPIOPin *clk, GPIOPin *data, GPIOPin *ctrl_in, GPIOPin *ctrl_out) {
  this->clk_ = clk;
  this->data_ = data;
  this->ctrl_in_ = ctrl_in;
  this->ctrl_out_ = ctrl_out;
}

void Balboa9800CP::setup() {
  instance_ = this;

  this->clk_->setup();
  this->data_->setup();
  this->ctrl_in_->setup();
  this->ctrl_out_->setup();

  // Default pass-through state: drive low or high depending on your interface hardware.
  // If you're using an external open-drain transistor, your GPIO should control the transistor gate.
  this->ctrl_out_->digital_write(false);

  // ESP8266 interrupt attach uses raw Arduino pin number.
  auto clk_pin = this->clk_->get_pin();
  ::pinMode(clk_pin, INPUT);

  ::attachInterrupt(digitalPinToInterrupt(clk_pin), Balboa9800CP::isr_router_, RISING);

  ESP_LOGW(TAG, "Started (frame_bits=%d gap_us=%u press_frames=%u)",
           this->frame_bits_, this->gap_us_, this->press_frames_);
}

void IRAM_ATTR Balboa9800CP::isr_router_() {
  if (instance_ != nullptr) instance_->on_clock_edge_();
}

void IRAM_ATTR Balboa9800CP::on_clock_edge_() {
  const uint32_t now = micros();
  const uint32_t dt = now - this->last_edge_us_;
  this->last_edge_us_ = now;

  // Frame boundary detection by long gap (doc ~12ms gap; threshold is configurable)
  if (dt > this->gap_us_) {
    this->bit_index_ = 0;
  }

  const int i = this->bit_index_;
  if (i >= this->frame_bits_) return;

  // Sample incoming bus signals
  const uint8_t data_bit = this->data_->digital_read() ? 1 : 0;
  const uint8_t ctrl_in_bit = this->ctrl_in_->digital_read() ? 1 : 0;

  this->data_bits_[i] = data_bit;
  this->ctrl_bits_in_[i] = ctrl_in_bit;

  // Default pass-through output equals incoming controls bit
  uint8_t out = ctrl_in_bit;

  // If we're pressing or releasing, override last 4 bits only
  if (this->pending_frames_left_ > 0 || this->release_frame_) {
    const int last4_start = this->frame_bits_ - 4;
    if (i >= last4_start) {
      uint8_t pattern = 0b0000;

      if (this->pending_frames_left_ > 0) {
        // Verified codes from your capture:
        // Steady 0000, Up 1110, Down 1111, Mode 1000
        switch (this->pending_cmd_) {
          case 1: pattern = 0b1110; break;  // TEMP_UP
          case 2: pattern = 0b1111; break;  // TEMP_DOWN
          case 3: pattern = 0b1000; break;  // MODE
          default: pattern = 0b0000; break;
        }
      } else {
        // Release frame forces steady state code
        pattern = 0b0000;
      }

      const int bitpos = i - last4_start;         // 0..3
      out = (pattern >> (3 - bitpos)) & 0x1;      // MSB first
    }
  }

  // Drive output. (If using open-drain hardware, ensure this matches your transistor logic.)
  this->ctrl_out_->digital_write(out);

  // Advance bit index and handle end-of-frame
  this->bit_index_++;

  if (this->bit_index_ >= this->frame_bits_) {
    this->frame_ready_ = true;

    // Button press duration bookkeeping
    if (this->pending_frames_left_ > 0) {
      this->pending_frames_left_--;
      if (this->pending_frames_left_ == 0) {
        // After press frames, emit a single "release" frame at steady code (0000)
        this->release_frame_ = true;
      }
    } else if (this->release_frame_) {
      // After one release frame, clear command
      this->release_frame_ = false;
      this->pending_cmd_ = 0;
    }
  }
}

void Balboa9800CP::queue_command(uint8_t cmd) {
  // Simple behavior: ignore if already in the middle of a press/release
  if (this->pending_frames_left_ > 0 || this->release_frame_) return;

  this->pending_cmd_ = cmd;
  this->pending_frames_left_ = this->press_frames_;
  this->release_frame_ = false;
}

void Balboa9800CP::loop() {
  if (!this->frame_ready_) return;
  this->frame_ready_ = false;
  this->process_frame_();
}

void Balboa9800CP::process_frame_() {
  // TODO: Implement decoding of the first 28 bits (4 x 7-seg digits) once mapped.
  // For now, publish nothing unless decode returns a valid value.
  int temp = this->decode_temp_from_segments_();
  if (this->water_temp_ != nullptr && temp > -50 && temp < 150) {
    if (temp != this->last_temp_) {
      this->water_temp_->publish_state(temp);
      this->last_temp_ = temp;
    }
  }
}

int Balboa9800CP::decode_temp_from_segments_() {
  // Placeholder until you map 7-seg bit order to digits.
  // When you're ready, you’ll:
  //  - group bits into 4 groups of 7
  //  - translate segments -> digit
  //  - interpret which digits represent water temp vs setpoint vs time/etc.
  return -999;
}

}  // namespace balboa_9800cp
}  // namespace esphome

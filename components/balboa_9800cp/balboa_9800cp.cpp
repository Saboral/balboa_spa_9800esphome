#include "balboa_9800cp.h"
#include "esphome/core/log.h"

namespace esphome {
namespace balboa_9800cp {

static const char *const TAG = "balboa_9800cp";

Balboa9800CP *Balboa9800CP::instance_ = nullptr;

void BalboaButton::press_action() {
  if (this->parent_ != nullptr) {
    auto *p = reinterpret_cast<Balboa9800CP *>(this->parent_);
    p->queue_command(this->cmd_);
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
  this->ctrl_out_->digital_write(false);  // default low or tri-state via external transistor/open-drain

  // Attach interrupt on clock rising edge (or falling—pick the edge that samples stable data)
  // NOTE: ESPHome GPIOPin doesn't attach interrupts directly; use Arduino attachInterrupt on raw pin number.
  auto pin = this->clk_->get_pin();
  ::pinMode(pin, INPUT);

  // data/ctrl pins already set up via GPIOPin; ensure fast read:
  ::attachInterrupt(digitalPinToInterrupt(pin), Balboa9800CP::isr_router_, RISING);

  ESP_LOGW(TAG, "Balboa9800CP bus started (frame_bits=%d gap_us=%u)", this->frame_bits_, this->gap_us_);
}

void IRAM_ATTR Balboa9800CP::isr_router_() {
  if (instance_ != nullptr) instance_->on_clock_edge_();
}

void IRAM_ATTR Balboa9800CP::on_clock_edge_() {
  const uint32_t now = micros();
  const uint32_t dt = now - this->last_edge_us_;
  this->last_edge_us_ = now;

  // Detect frame boundary by long gap (doc ~12ms) :contentReference[oaicite:1]{index=1}
  if (dt > this->gap_us_) {
    this->bit_index_ = 0;
  }

  const int i = this->bit_index_;
  if (i < this->frame_bits_) {
    // Sample inputs
    this->data_bits_[i] = this->data_->digital_read() ? 1 : 0;
    this->ctrl_bits_in_[i] = this->ctrl_in_->digital_read() ? 1 : 0;

    // Default pass-through
    uint8_t out = this->ctrl_bits_in_[i];

    // Optional override for last 4 bits (button code) :contentReference[oaicite:2]{index=2}
    if (this->pending_frames_left_ > 0) {
      const int last4_start = this->frame_bits_ - 4;
      if (i >= last4_start) {
        // Map command -> 4-bit pattern (you’ll set these to match the doc’s observed codes)
        // Placeholder patterns (replace with your verified codes)
        uint8_t pattern = 0b0000;
        switch (this->pending_cmd_) {
          case 1: pattern = 0b1000; break; // TEMP_UP example placeholder
          case 2: pattern = 0b0100; break; // TEMP_DOWN placeholder (doc showed "down" last 4 high-ish)
          case 3: pattern = 0b0010; break; // MODE placeholder
          default: pattern = 0b0000; break;
        }
        const int bitpos = i - last4_start;  // 0..3
        out = (pattern >> (3 - bitpos)) & 0x1;
      }
    }

    // Drive output (prefer open-drain external transistor; this assumes ctrl_out drives “low true”)
    this->ctrl_out_->digital_write(out);

    this->bit_index_++;

    if (this->bit_index_ >= this->frame_bits_) {
      // End of frame
      this->frame_ready_ = true;

      if (this->pending_frames_left_ > 0) {
        this->pending_frames_left_--;
        if (this->pending_frames_left_ == 0) {
          this->pending_cmd_ = 0;
        }
      }
    }
  }
}

void Balboa9800CP::queue_command(uint8_t cmd) {
  // Start “press” for N frames; keep very simple to avoid concurrency issues
  this->pending_cmd_ = cmd;
  this->pending_frames_left_ = this->press_frames_;
}

void Balboa9800CP::loop() {
  if (!this->frame_ready_) return;
  this->frame_ready_ = false;
  this->process_frame_();
}

void Balboa9800CP::process_frame_() {
  // TODO: Implement segment decode once you confirm mapping.
  // The doc notes the first 28 bits correspond to 4x7-seg digits. :contentReference[oaicite:3]{index=3}

  int temp = this->decode_temp_from_segments_();
  if (this->water_temp_ != nullptr && temp > -100 && temp < 150) {
    if (temp != this->last_temp_) {
      this->water_temp_->publish_state(temp);
      this->last_temp_ = temp;
    }
  }
}

int Balboa9800CP::decode_temp_from_segments_() {
  // Placeholder: return invalid until mapping is implemented
  return -999;
}

}  // namespace balboa_9800cp
}  // namespace esphome

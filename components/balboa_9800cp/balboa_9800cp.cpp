#include "balboa_9800cp.h"
#include "esphome/core/log.h"

#include <Arduino.h>
#include <ctype.h>
#include <string.h>

namespace esphome {
namespace balboa_9800cp {

Balboa9800CP *Balboa9800CP::instance_ = nullptr;

}  // namespace balboa_9800cp
}  // namespace esphome

static const char *const TAG = "balboa_9800cp";

void Balboa9800CP::dump_config() {
  ESP_LOGCONFIG(TAG, "Balboa 9800CP:");
  ESP_LOGCONFIG(TAG, "  clk_gpio: %d", this->clk_gpio_);
  ESP_LOGCONFIG(TAG, "  ctrl_out_gpio: %d", this->ctrl_out_gpio_);
  ESP_LOGCONFIG(TAG, "  gap_us: %u", (unsigned) this->gap_us_);
  ESP_LOGCONFIG(TAG, "  press_frames: %u", (unsigned) this->press_frames_);
}


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
  ESP_LOGW(TAG, "BALBOA COMPONENT setup() reached");
  instance_ = this;

  this->clk_->setup();
  this->data_->setup();
  this->ctrl_in_->setup();
  this->ctrl_out_->setup();

  if (this->clk_gpio_ < 0 || this->ctrl_out_gpio_ < 0) {
    ESP_LOGE(TAG, "GPIO numbers not set (clk_gpio=%d ctrl_out_gpio=%d). Check __init__.py set_gpio_numbers().",
             this->clk_gpio_, this->ctrl_out_gpio_);
    return;
  }

  // Ensure clock pin is input and attach interrupt on raw GPIO
  pinMode(this->clk_gpio_, INPUT);
  attachInterrupt(digitalPinToInterrupt(this->clk_gpio_), Balboa9800CP::isr_router_, RISING);

  // SAFE resistor-only injection: start ctrl_out as high-Z (INPUT)
  pinMode(this->ctrl_out_gpio_, INPUT);

  ESP_LOGI(TAG, "Started (gap_us=%u press_frames=%u) clk_gpio=%d data_gpio=<GPIOPin> ctrl_in_gpio=<GPIOPin> ctrl_out_gpio=%d",
           (unsigned) this->gap_us_, (unsigned) this->press_frames_,
           this->clk_gpio_, this->ctrl_out_gpio_);
}

void IRAM_ATTR Balboa9800CP::isr_router_() {
  if (instance_ != nullptr) instance_->on_clock_edge_();
}

void IRAM_ATTR Balboa9800CP::on_clock_edge_() {
  const uint32_t now = micros();
  const uint32_t dt = now - this->last_edge_us_;
  this->last_edge_us_ = now;

  // Frame boundary
  if (dt > this->gap_us_) this->bit_index_ = 0;

  const int i = this->bit_index_;
  if (i >= 76) return;

  // Capture DATA bit (board -> topside)
  this->bits_[i] = this->data_->digital_read() ? 1 : 0;

  // Default CTRL pass-through
  uint8_t out = this->ctrl_in_->digital_read() ? 1 : 0;

  // Inject CTRL last 4 bits (i=72..75) with verified button codes:
  // steady 0000, Up 1110, Down 1111, Mode 1000
  if ((this->frames_left_ > 0 || this->release_frame_) && i >= 72) {
    uint8_t pattern = 0b0000;

    if (this->frames_left_ > 0) {
      switch (this->pending_cmd_) {
        case 1: pattern = 0b1110; break;  // Up
        case 2: pattern = 0b1111; break;  // Down
        case 3: pattern = 0b1000; break;  // Mode
        default: pattern = 0b0000; break;
      }
    } else {
      pattern = 0b0000;  // release frame
    }

    const int bitpos = i - 72;              // 0..3
    out = (pattern >> (3 - bitpos)) & 0x1;  // MSB first
  }

  // SAFE open-drain/high-Z behavior (resistor-only injection):
  // out=0 -> OUTPUT LOW (pull down)
  // out=1 -> INPUT (float), allowing spa pull-up / pass-through to represent HIGH
  if (out == 0) {
    pinMode(this->ctrl_out_gpio_, OUTPUT);
    digitalWrite(this->ctrl_out_gpio_, LOW);
  } else {
    pinMode(this->ctrl_out_gpio_, INPUT);
  }

  this->bit_index_++;

  if (this->bit_index_ >= 76) {
    this->frame_ready_ = true;

    // Press bookkeeping: hold N frames, then one release frame
    if (this->frames_left_ > 0) {
      this->frames_left_--;
      if (this->frames_left_ == 0) this->release_frame_ = true;
    } else if (this->release_frame_) {
      this->release_frame_ = false;
      this->pending_cmd_ = 0;
    }
  }
}

void Balboa9800CP::queue_command(uint8_t cmd) {
  if (this->frames_left_ > 0 || this->release_frame_) return;
  this->pending_cmd_ = cmd;
  this->frames_left_ = this->press_frames_;
}

/* -------- decoder mapping helpers -------- */
int Balboa9800CP::get_bit1_(int bit_1_index) const {
  if (bit_1_index < 1 || bit_1_index > 76) return 0;
  return this->bits_[bit_1_index - 1] ? 1 : 0;
}

char Balboa9800CP::decode_digit_(uint8_t seg, bool inverted) const {
  if (!inverted) {
    switch (seg) {
      case 0b0000000: return ' ';
      case 0b1111110: return '0';
      case 0b0110000: return '1';
      case 0b1101101: return '2';
      case 0b1111001: return '3';
      case 0b0110011: return '4';
      case 0b1011011: return '5';
      case 0b1011111: return '6';
      case 0b1110000: return '7';
      case 0b1111111: return '8';
      case 0b1110011: return '9';
      case 0b1000111: return 'F';
      case 0b1001110: return 'C';
      case 0b0001110: return 'L';
      case 0b0010101: return 'n';
      case 0b0110111: return 'H';
      case 0b1001111: return 'E';
      default: return '?';
    }
  } else {
    switch (seg) {
      case 0b0000000: return ' ';
      case 0b1111110: return '0';
      case 0b0000110: return '1';
      case 0b1101101: return '2';
      case 0b1001111: return '3';
      case 0b0010111: return '4';
      case 0b1011011: return '5';
      case 0b1111011: return '6';
      case 0b0001110: return '7';
      case 0b1111111: return '8';
      case 0b0011111: return '9';
      case 0b0111001: return 'F';
      case 0b1111000: return 'C';
      case 0b1110000: return 'L';
      case 0b0100011: return 'n';
      case 0b0110111: return 'H';
      case 0b1111001: return 'E';
      default: return '?';
    }
  }
}

void Balboa9800CP::decode_display_(char out[5], bool &inverted) const {
  inverted = (this->get_bit1_(29) == 1);

  uint8_t digit[4] = {0, 0, 0, 0};
  const int base = 2;  // bits 2..29 (1-based) -> 28 bits -> 4x7

  for (int d = 0; d < 4; d++) {
    uint8_t v = 0;
    for (int k = 0; k < 7; k++) {
      v <<= 1;
      v |= (uint8_t) this->get_bit1_(base + d * 7 + k);
    }
    digit[d] = v;
  }

  char normal[4] = {
      this->decode_digit_(digit[3], false),
      this->decode_digit_(digit[2], false),
      this->decode_digit_(digit[1], false),
      this->decode_digit_(digit[0], false),
  };

  char inv[4] = {
      this->decode_digit_(digit[0], true),
      this->decode_digit_(digit[1], true),
      this->decode_digit_(digit[2], true),
      this->decode_digit_(digit[3], true),
  };

  bool normal_has_unknown = false;
  for (int i = 0; i < 4; i++) {
    if (normal[i] == '?') {
      normal_has_unknown = true;
      break;
    }
  }

  const char *chosen = normal_has_unknown ? inv : normal;
  for (int i = 0; i < 4; i++) out[i] = chosen[i];
  out[4] = '\0';
}

int Balboa9800CP::convert_temp_(const char *disp) const {
  int val = 0;
  int digits = 0;

  for (int i = 0; i < 3 && disp[i]; i++) {
    if (isdigit((unsigned char) disp[i])) {
      val = val * 10 + (disp[i] - '0');
      digits++;
    }
  }
  if (digits == 0) return 60;
  return val;
}

void Balboa9800CP::process_frame_() {
  char disp[5];
  bool inv_flag = false;
  this->decode_display_(disp, inv_flag);

  const int temp_f = this->convert_temp_(disp);

  const bool b_inverted = (this->get_bit1_(29) == 1);
  const bool b_set_heat = (this->get_bit1_(41) == 1);
  const bool b_mode_std = (this->get_bit1_(60) == 1);   // 0=Economy, 1=Standard
  const bool b_heating  = (this->get_bit1_(40) == 1);
  const bool b_temp_up  = (this->get_bit1_(30) == 1);
  const bool b_temp_dn  = (this->get_bit1_(39) == 1);
  const bool b_blower   = (this->get_bit1_(43) == 1);
  const bool b_pump     = (this->get_bit1_(49) == 1);
  const bool b_jets     = (this->get_bit1_(50) == 1);
  const bool b_light    = (this->get_bit1_(48) == 1);

  // ✅ DEBUG line: shows what the decoder is seeing every processed frame
  ESP_LOGD(TAG,
           "disp='%s' temp_f=%d inv=%d set_heat=%d mode_std=%d heating=%d up=%d down=%d blower=%d pump=%d jets=%d light=%d",
           disp, temp_f,
           b_inverted, b_set_heat, b_mode_std, b_heating, b_temp_up, b_temp_dn,
           b_blower, b_pump, b_jets, b_light);

  uint16_t flags = 0;
  flags |= (uint16_t) (b_inverted ? 1 : 0) << 0;
  flags |= (uint16_t) (b_set_heat ? 1 : 0) << 1;
  flags |= (uint16_t) (b_mode_std ? 1 : 0) << 2;
  flags |= (uint16_t) (b_heating  ? 1 : 0) << 3;
  flags |= (uint16_t) (b_temp_up  ? 1 : 0) << 4;
  flags |= (uint16_t) (b_temp_dn  ? 1 : 0) << 5;
  flags |= (uint16_t) (b_blower   ? 1 : 0) << 6;
  flags |= (uint16_t) (b_pump     ? 1 : 0) << 7;
  flags |= (uint16_t) (b_jets     ? 1 : 0) << 8;
  flags |= (uint16_t) (b_light    ? 1 : 0) << 9;

  // Publish display text if changed
  if (this->display_text_ != nullptr) {
    if (strncmp(this->last_display_, disp, 4) != 0) {
      this->display_text_->publish_state(disp);
      strncpy(this->last_display_, disp, 4);
      this->last_display_[4] = '\0';
    }
  }

  // Publish temperature if changed
  if (this->water_temp_ != nullptr && temp_f != this->last_temp_f_) {
    this->water_temp_->publish_state((float) temp_f);
    this->last_temp_f_ = temp_f;
  }

  // Publish binary flags only when any flag changes
  if (!this->last_flags_valid_ || flags != this->last_flags_) {
    this->last_flags_valid_ = true;
    this->last_flags_ = flags;

    if (this->inverted_ != nullptr) this->inverted_->publish_state(b_inverted);
    if (this->set_heat_ != nullptr) this->set_heat_->publish_state(b_set_heat);
    if (this->mode_standard_ != nullptr) this->mode_standard_->publish_state(b_mode_std);
    if (this->heating_ != nullptr) this->heating_->publish_state(b_heating);
    if (this->temp_up_display_ != nullptr) this->temp_up_display_->publish_state(b_temp_up);
    if (this->temp_down_display_ != nullptr) this->temp_down_display_->publish_state(b_temp_dn);
    if (this->blower_ != nullptr) this->blower_->publish_state(b_blower);
    if (this->pump_ != nullptr) this->pump_->publish_state(b_pump);
    if (this->jets_ != nullptr) this->jets_->publish_state(b_jets);
    if (this->light_ != nullptr) this->light_->publish_state(b_light);
  }
}

void Balboa9800CP::loop() {
  if (!this->frame_ready_) return;
  this->frame_ready_ = false;
  this->process_frame_();
}

}  // namespace balboa_9800cp
}  // namespace esphome

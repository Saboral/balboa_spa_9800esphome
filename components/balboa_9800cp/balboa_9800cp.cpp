#include "balboa_9800cp.h"
#include "esphome/core/log.h"
#include <Arduino.h>
#include <ctype.h>

namespace esphome {
namespace balboa_9800cp {

static const char *TAG = "balboa_9800cp";
Balboa9800CP *Balboa9800CP::instance_ = nullptr;

void BalboaButton::press_action() {
  if (this->parent_ != nullptr) this->parent_->queue_command(this->cmd_);
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
  this->ctrl_out_->digital_write(false);

  auto clk_pin = this->clk_->get_pin();
  pinMode(clk_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(clk_pin), Balboa9800CP::isr_router_, RISING);

  ESP_LOGI(TAG, "Started (ESP32) gap_us=%u press_frames=%u", (unsigned) this->gap_us_, this->press_frames_);
}

void IRAM_ATTR Balboa9800CP::isr_router_() {
  if (instance_ != nullptr) instance_->on_clock_edge_();
}

void IRAM_ATTR Balboa9800CP::on_clock_edge_() {
  const uint32_t now = micros();
  const uint32_t dt = now - this->last_edge_us_;
  this->last_edge_us_ = now;

  // frame boundary after long gap (doc ~12ms; threshold configurable)
  if (dt > this->gap_us_) this->bit_index_ = 0;

  const int i = this->bit_index_;
  if (i >= 76) return;

  // capture DATA bit (board->topside)
  this->bits_[i] = this->data_->digital_read() ? 1 : 0;

  // default CTRL pass-through
  uint8_t out = this->ctrl_in_->digital_read() ? 1 : 0;

  // Inject button presses by overriding last 4 bits of CTRL frame (same as earlier)
  // Your verified codes:
  // steady 0000, up 1110, down 1111, mode 1000
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
      pattern = 0b0000; // release frame
    }

    // i=72..75 maps to pattern bit 3..0 (MSB first)
    const int bitpos = i - 72;            // 0..3
    out = (pattern >> (3 - bitpos)) & 0x1;
  }

  this->ctrl_out_->digital_write(out);

  this->bit_index_++;

  if (this->bit_index_ >= 76) {
    this->frame_ready_ = true;

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

/* ---------- decoder.js port (exact mapping) ---------- */

int Balboa9800CP::get_bit1_(int bit_1_index) const {
  // decoder.js uses 1-based indexing: array[bit-1]
  if (bit_1_index < 1 || bit_1_index > 76) return 0;
  return this->bits_[bit_1_index - 1] ? 1 : 0;
}

char Balboa9800CP::decode_digit_(uint8_t seg, bool inverted) const {
  // Exact digit maps from decoder.js :contentReference[oaicite:2]{index=2}
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
  // decoder.js:
  // displayBits = bits.substring(1,29) -> ignores first bit, takes next 28 bits (bits 2..29 1-based)
  // inverted flag stored separately at bit 29 (1-based) which is bits_[28] (0-based)
  inverted = (this->get_bit1_(29) == 1);

  // Build 4x 7-bit digits from bits 2..28 (1-based), i.e., array index 1..27 (0-based)
  // We'll assemble digit segments in displayBits order: digit1 first, digit4 last,
  // then choose orientation exactly like decoder.js.
  uint8_t digit[4] = {0, 0, 0, 0};

  // digit0 uses bits 2..8 (1-based), digit1 bits 9..15, digit2 bits 16..22, digit3 bits 23..29? (but displayBits stops at 28)
  // decoder.js uses substring(1,29) and then slices [0..7), [7..14), [14..21), [21..28)
  // That corresponds to 28 bits total (positions 2..29 exclusive?), but operationally 4*7=28 bits after ignoring the first.
  // So we read bits 2..29 (1-based) for these 28 bits:
  int base_bit = 2;
  for (int d = 0; d < 4; d++) {
    uint8_t v = 0;
    for (int k = 0; k < 7; k++) {
      v <<= 1;
      v |= (uint8_t) this->get_bit1_(base_bit + d * 7 + k);
    }
    digit[d] = v;
  }

  // decoder.js ordering:
  // display = [ map(bits[21..28]), map(bits[14..21]), map(bits[7..14]), map(bits[0..7]) ]
  // inverted = [ invMap(bits[0..7]), invMap(bits[7..14]), invMap(bits[14..21]), invMap(bits[21..28]) ]
  // With our digit array in forward order, digit[0]=bits[0..7], digit[3]=bits[21..28]
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

  // decoder.js chooses whichever orientation DOESN'T contain '?'.
  bool normal_has_q = false;
  for (int i = 0; i < 4; i++) if (normal[i] == '?') normal_has_q = true;

  const char *chosen = normal_has_q ? inv : normal;
  for (int i = 0; i < 4; i++) out[i] = chosen[i];
  out[4] = '\0';
}

int Balboa9800CP::convert_temp_(const char *disp) const {
  // decoder.js: take first 3 chars, parseInt; if NaN => 60
  // We'll emulate behavior: parse first 3 chars that might include spaces/letters; if no digits => default 60.
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
  bool inverted = false;
  decode_display_(disp, inverted);

  const int temp_f = convert_temp_(disp);

  // Publish temp (°F)
  if (this->water_temp_ != nullptr && temp_f != this->last_temp_f_) {
    this->water_temp_->publish_state((float) temp_f);
    this->last_temp_f_ = temp_f;
  }

  // Publish flags matching decoder.js bit mapping :contentReference[oaicite:3]{index=3}
  if (this->heating_ != nullptr) {
    this->heating_->publish_state(this->get_bit1_(40) == 1);
  }
  if (this->standard_mode_ != nullptr) {
    // bit 60: 0=Economy, 1=Standard
    this->standard_mode_->publish_state(this->get_bit1_(60) == 1);
  }

  // Optional: uncomment for debugging
  // ESP_LOGD(TAG, "disp='%s' inv=%d temp=%d heat=%d mode=%d",
  //          disp, inverted ? 1 : 0, temp_f, get_bit1_(40), get_bit1_(60));
}

void Balboa9800CP::loop() {
  if (!this->frame_ready_) return;
  this->frame_ready_ = false;
  this->process_frame_();
}

}  // namespace balboa_9800cp
}  // namespace esphome

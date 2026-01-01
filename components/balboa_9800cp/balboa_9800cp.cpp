#include "balboa_9800cp.h"
#include "esphome/core/log.h"

#include <Arduino.h>
#include "driver/gpio.h"
#include <ctype.h>
#include <string>

namespace esphome {
namespace balboa_9800cp {

static const char *const TAG = "balboa_9800cp";

Balboa9800CP *Balboa9800CP::instance_ = nullptr;
static portMUX_TYPE balboa_mux = portMUX_INITIALIZER_UNLOCKED;

// ---- Protocol constants ----
static constexpr int FRAME_BITS = 42;   // 6 chunks * 7 bits
static constexpr int CHUNK_BITS = 7;

// Cache last completed frame (avoids ISR/main loop races)
static uint8_t last_disp_[FRAME_BITS];
static uint8_t last_ctrl_[FRAME_BITS];
static bool last_valid_ = false;

// Track last STAT/EQ for diff logging
static bool have_prev_stat_eq_ = false;
static uint8_t prev_stat_ = 0;
static uint8_t prev_eq_ = 0;

// Track last published temp so HA keeps it even when display shows non-temp messages
static bool have_last_temp_ = false;
static float last_temp_f_ = NAN;

// 7-seg reference table (standard “gfedcba” style patterns masked to 7 bits).
// We match on low 7 bits only, returning ASCII chars.
static const uint8_t SevenSegmentASCII[96] = {
  0x00, /* space */ 0x86, /* ! */ 0x22, /* " */ 0x7E, /* # */ 0x6D, /* $ */ 0xD2, /* % */ 0x46, /* & */ 0x20, /* ' */
  0x29, /* ( */     0x0B, /* ) */ 0x21, /* * */ 0x70, /* + */ 0x10, /* , */ 0x40, /* - */ 0x80, /* . */ 0x52, /* / */
  0x3F, /* 0 */     0x06, /* 1 */ 0x5B, /* 2 */ 0x4F, /* 3 */ 0x66, /* 4 */ 0x6D, /* 5 */ 0x7D, /* 6 */ 0x07, /* 7 */
  0x7F, /* 8 */     0x6F, /* 9 */ 0x09, /* : */ 0x0D, /* ; */ 0x61, /* < */ 0x48, /* = */ 0x43, /* > */ 0xD3, /* ? */
  0x5F, /* @ */     0x77, /* A */ 0x7C, /* B */ 0x39, /* C */ 0x5E, /* D */ 0x79, /* E */ 0x71, /* F */ 0x3D, /* G */
  0x76, /* H */     0x30, /* I */ 0x1E, /* J */ 0x75, /* K */ 0x38, /* L */ 0x15, /* M */ 0x37, /* N */ 0x3F, /* O */
  0x73, /* P */     0x6B, /* Q */ 0x33, /* R */ 0x6D, /* S */ 0x78, /* T */ 0x3E, /* U */ 0x3E, /* V */ 0x2A, /* W */
  0x76, /* X */     0x6E, /* Y */ 0x5B, /* Z */ 0x39, /* [ */ 0x64, /* \ */ 0x0F, /* ] */ 0x23, /* ^ */ 0x08, /* _ */
  0x02, /* ` */     0x5F, /* a */ 0x7C, /* b */ 0x58, /* c */ 0x5E, /* d */ 0x7B, /* e */ 0x71, /* f */ 0x6F, /* g */
  0x74, /* h */     0x10, /* i */ 0x0C, /* j */ 0x75, /* k */ 0x30, /* l */ 0x14, /* m */ 0x54, /* n */ 0x5C, /* o */
  0x73, /* p */     0x67, /* q */ 0x50, /* r */ 0x6D, /* s */ 0x78, /* t */ 0x1C, /* u */ 0x1C, /* v */ 0x14, /* w */
  0x76, /* x */     0x6E, /* y */ 0x5B, /* z */ 0x46, /* { */ 0x30, /* | */ 0x70, /* } */ 0x01, /* ~ */ 0x00  /* del */
};

static char seg7_to_char(uint8_t pat7) {
  const uint8_t p = pat7 & 0x7F;
  for (int i = 0; i < 96; i++) {
    if ((SevenSegmentASCII[i] & 0x7F) == p) {
      return static_cast<char>(i + 32);
    }
  }
  return '?';
}

// Pack 7 bits MSB-first from bit array: bits[start+0] becomes bit6 ... bits[start+6] becomes bit0
static uint8_t pack7_msb(const uint8_t *bits, int start) {
  uint8_t v = 0;
  for (int k = 0; k < 7; k++) {
    v <<= 1;
    v |= (bits[start + k] ? 1 : 0);
  }
  return v & 0x7F;
}

static std::string bits7(uint8_t v) {
  std::string s;
  s.reserve(7);
  for (int i = 6; i >= 0; i--) s.push_back((v & (1 << i)) ? '1' : '0');
  return s;
}

static bool looks_like_set_mode(const std::string &s) {
  bool has_s = false, has_e = false, has_t = false;
  for (char c : s) {
    c = (char) toupper((int) c);
    if (c == 'S') has_s = true;
    if (c == 'E') has_e = true;
    if (c == 'T') has_t = true;
  }
  return has_s && has_e && has_t;
}

static bool parse_reasonable_temp_f(const std::string &s, int &out_temp) {
  int digits[4];
  int n = 0;
  for (char c : s) {
    if (c >= '0' && c <= '9' && n < 4) digits[n++] = c - '0';
  }
  if (n < 2) return false;

  int val = 0;
  if (n == 2) val = digits[0] * 10 + digits[1];
  else if (n == 3) val = digits[0] * 100 + digits[1] * 10 + digits[2];
  else val = digits[0] * 1000 + digits[1] * 100 + digits[2] * 10 + digits[3];

  // Typical spa water temps in F.
  if (val < 50 || val > 120) return false;
  out_temp = val;
  return true;
}

// ---------------- Buttons ----------------
void BalboaButton::press_action() {
  if (this->parent_ != nullptr)
    this->parent_->queue_command(this->cmd_);
}

// ---------------- Component ----------------
void Balboa9800CP::set_pins(GPIOPin *clk, GPIOPin *data, GPIOPin *ctrl_in, GPIOPin *ctrl_out) {
  this->clk_ = clk;
  this->data_ = data;
  this->ctrl_in_ = ctrl_in;
  this->ctrl_out_ = ctrl_out;
}

void Balboa9800CP::dump_config() {
  ESP_LOGCONFIG(TAG, "Balboa 9800CP (corrected capture + diff logger):");
  ESP_LOGCONFIG(TAG, "  clk_gpio: %d", this->clk_gpio_);
  ESP_LOGCONFIG(TAG, "  display_data_gpio: %d", this->data_gpio_);
  ESP_LOGCONFIG(TAG, "  ctrl_in_gpio: %d", this->ctrl_in_gpio_);
  ESP_LOGCONFIG(TAG, "  ctrl_out_gpio: %d", this->ctrl_out_gpio_);
  ESP_LOGCONFIG(TAG, "  gap_us: %u", (unsigned) this->gap_us_);
  ESP_LOGCONFIG(TAG, "  press_frames: %u", (unsigned) this->press_frames_);
}

void Balboa9800CP::setup() {
  ESP_LOGI(TAG, "setup() reached");
  instance_ = this;

  if (this->clk_) this->clk_->setup();
  if (this->data_) this->data_->setup();
  if (this->ctrl_in_) this->ctrl_in_->setup();
  if (this->ctrl_out_) this->ctrl_out_->setup();

  if (this->clk_gpio_ < 0 || this->data_gpio_ < 0 || this->ctrl_in_gpio_ < 0 || this->ctrl_out_gpio_ < 0) {
    ESP_LOGE(TAG, "GPIO numbers not set. clk=%d disp=%d ctrl=%d out=%d",
             this->clk_gpio_, this->data_gpio_, this->ctrl_in_gpio_, this->ctrl_out_gpio_);
    return;
  }

  gpio_num_t clk  = (gpio_num_t) this->clk_gpio_;
  gpio_num_t disp = (gpio_num_t) this->data_gpio_;
  gpio_num_t ctrl = (gpio_num_t) this->ctrl_in_gpio_;
  gpio_num_t out  = (gpio_num_t) this->ctrl_out_gpio_;

  gpio_reset_pin(clk);
  gpio_set_direction(clk, GPIO_MODE_INPUT);

  // ✅ CRITICAL FIX: rising edge only (one sample per clock pulse)
  gpio_set_intr_type(clk, GPIO_INTR_POSEDGE);

  gpio_reset_pin(disp);
  gpio_set_direction(disp, GPIO_MODE_INPUT);

  gpio_reset_pin(ctrl);
  gpio_set_direction(ctrl, GPIO_MODE_INPUT);

  // Keep ctrl_out high-Z for now (button injection later)
  gpio_reset_pin(out);
  gpio_set_direction(out, GPIO_MODE_INPUT);

  static bool installed = false;
  if (!installed) {
    esp_err_t e = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "gpio_install_isr_service failed: %d", (int) e);
    } else {
      installed = true;
    }
  }

  esp_err_t e2 = gpio_isr_handler_add(clk, (gpio_isr_t) &Balboa9800CP::isr_router_, nullptr);
  if (e2 != ESP_OK) {
    ESP_LOGE(TAG, "gpio_isr_handler_add failed: %d", (int) e2);
    return;
  }

  this->bit_index_ = 0;
  this->frame_ready_ = false;
  this->last_edge_us_ = micros();
  this->isr_edge_count_ = 0;

  last_valid_ = false;
  have_prev_stat_eq_ = false;
  have_last_temp_ = false;
  last_temp_f_ = NAN;

  ESP_LOGI(TAG, "ISR attached (POSEDGE). clk=%d disp=%d ctrl_in=%d ctrl_out=%d",
           this->clk_gpio_, this->data_gpio_, this->ctrl_in_gpio_, this->ctrl_out_gpio_);
}

void IRAM_ATTR Balboa9800CP::isr_router_() {
  if (Balboa9800CP::instance_ != nullptr)
    Balboa9800CP::instance_->on_clock_edge_();
}

void IRAM_ATTR Balboa9800CP::on_clock_edge_() {
  this->isr_edge_count_++;

  const uint32_t now = micros();
  const uint32_t dt = now - this->last_edge_us_;
  this->last_edge_us_ = now;

  // Resync on long idle gap: start a new frame.
  if (dt > this->gap_us_) {
    this->bit_index_ = 0;
    this->frame_ready_ = false;  // ✅ important so capture resumes immediately
  }

  // ✅ Frame-freeze guard: don’t overwrite until we see a resync gap
  if (this->frame_ready_) return;

  const int i = this->bit_index_;
  if (i < 0 || i >= FRAME_BITS) return;

  // Sample both lines on the same rising edge
  this->disp_bits_[i] = gpio_get_level((gpio_num_t) this->data_gpio_) ? 1 : 0;
  this->ctrl_bits_[i] = gpio_get_level((gpio_num_t) this->ctrl_in_gpio_) ? 1 : 0;

  this->bit_index_++;

  if (this->bit_index_ >= FRAME_BITS) {
    portENTER_CRITICAL_ISR(&balboa_mux);
    this->frame_ready_ = true;
    portEXIT_CRITICAL_ISR(&balboa_mux);
  }
}

void Balboa9800CP::queue_command(uint8_t cmd) {
  (void) cmd;
  // Injection still disabled in this step.
}

void Balboa9800CP::loop() {
  const uint32_t now = millis();

  // 1 Hz edge counter
  static uint32_t last_edges_ms = 0;
  if (now - last_edges_ms >= 1000) {
    last_edges_ms = now;
    const uint32_t edges = this->isr_edge_count_;
    this->isr_edge_count_ = 0;
    ESP_LOGD(TAG, "clk edges/sec=%u bit_index=%d frame_ready=%d",
             (unsigned) edges, this->bit_index_, (int) this->frame_ready_);
  }

  // Copy out a completed frame if available
  bool got_frame = false;
  portENTER_CRITICAL(&balboa_mux);
  if (this->frame_ready_) {
    for (int i = 0; i < FRAME_BITS; i++) {
      last_disp_[i] = this->disp_bits_[i];
      last_ctrl_[i] = this->ctrl_bits_[i];
    }
    last_valid_ = true;
    this->frame_ready_ = false;  // allow next frame after resync gap
    got_frame = true;
  }
  portEXIT_CRITICAL(&balboa_mux);

  if (!last_valid_) return;

  // Process at ~5 Hz
  static uint32_t last_process_ms = 0;
  if (now - last_process_ms < 200) return;
  last_process_ms = now;

  // Decode chunks
  const uint8_t d1   = pack7_msb(last_disp_, 0);
  const uint8_t d2   = pack7_msb(last_disp_, 7);
  const uint8_t d3   = pack7_msb(last_disp_, 14);
  const uint8_t d4   = pack7_msb(last_disp_, 21);
  const uint8_t stat = pack7_msb(last_disp_, 28);
  const uint8_t eq   = pack7_msb(last_disp_, 35);

  // Decode display (4 chars)
  std::string disp;
  disp.reserve(4);
  disp.push_back(seg7_to_char(d1));
  disp.push_back(seg7_to_char(d2));
  disp.push_back(seg7_to_char(d3));
  disp.push_back(seg7_to_char(d4));

  // Publish display
  if (this->display_text_ != nullptr) {
    this->display_text_->publish_state(disp);
  }

  // Publish temp only when numeric-looking; otherwise keep last
  if (this->water_temp_ != nullptr) {
    int temp_f = 0;
    if (parse_reasonable_temp_f(disp, temp_f)) {
      last_temp_f_ = (float) temp_f;
      have_last_temp_ = true;
      this->water_temp_->publish_state(last_temp_f_);
    } else {
      // keep last known temp; do not clear in HA
      if (have_last_temp_) {
        // optional: you can republish occasionally if you want, but not necessary
      }
    }
  }

  // “Set heat” heuristic (display shows SET/SEt/etc)
  const bool set_heat = looks_like_set_mode(disp);

  // For now: do NOT hard-map EQ/STAT to pumps/etc until we see real changes.
  // Instead, publish “best effort” and focus on diff logging to learn mapping.
  // You can still wire a few obvious items later after we correlate flips.
  if (this->set_heat_ != nullptr) this->set_heat_->publish_state(set_heat);

  // --- Diff logging: print STAT/EQ changes and which bits flipped ---
  if (got_frame) {
    if (!have_prev_stat_eq_) {
      have_prev_stat_eq_ = true;
      prev_stat_ = stat;
      prev_eq_ = eq;
      ESP_LOGI(TAG, "Initial STAT=0x%02X (%s)  EQ=0x%02X (%s)  Display=\"%s\"",
               stat, bits7(stat).c_str(), eq, bits7(eq).c_str(), disp.c_str());
    } else {
      const uint8_t dstat = (uint8_t) (prev_stat_ ^ stat);
      const uint8_t deq   = (uint8_t) (prev_eq_ ^ eq);

      if (dstat != 0 || deq != 0) {
        ESP_LOGI(TAG, "CHANGE Display=\"%s\"  STAT 0x%02X->0x%02X (xor 0x%02X)  EQ 0x%02X->0x%02X (xor 0x%02X)",
                 disp.c_str(), prev_stat_, stat, dstat, prev_eq_, eq, deq);

        if (dstat != 0) {
          ESP_LOGI(TAG, "  STAT bits old=%s new=%s",
                   bits7(prev_stat_).c_str(), bits7(stat).c_str());
        }
        if (deq != 0) {
          ESP_LOGI(TAG, "  EQ   bits old=%s new=%s",
                   bits7(prev_eq_).c_str(), bits7(eq).c_str());
        }

        prev_stat_ = stat;
        prev_eq_ = eq;
      }
    }
  }

  // Optional: still print full raw frame once per second for sanity
  static uint32_t last_dbg_ms = 0;
  if (now - last_dbg_ms >= 1000) {
    last_dbg_ms = now;

    char disp_bits_str[FRAME_BITS + 1];
    char ctrl_bits_str[FRAME_BITS + 1];
    for (int i = 0; i < FRAME_BITS; i++) {
      disp_bits_str[i] = last_disp_[i] ? '1' : '0';
      ctrl_bits_str[i] = last_ctrl_[i] ? '1' : '0';
    }
    disp_bits_str[FRAME_BITS] = '\0';
    ctrl_bits_str[FRAME_BITS] = '\0';

    ESP_LOGI(TAG, "DISP(pin5) bits[%d]: %s", FRAME_BITS, disp_bits_str);
    ESP_LOGI(TAG, "CTRL(pin3) bits[%d]: %s", FRAME_BITS, ctrl_bits_str);
    ESP_LOGI(TAG, "Chunks: D1=0x%02X D2=0x%02X D3=0x%02X D4=0x%02X STAT=0x%02X EQ=0x%02X  Display=\"%s\"",
             d1, d2, d3, d4, stat, eq, disp.c_str());
  }
}

void Balboa9800CP::process_frame_() {
  // Not used (processing happens in loop() using cached frame)
}

}  // namespace balboa_9800cp
}  // namespace esphome

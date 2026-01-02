#include "balboa_9800cp.h"
#include "esphome/core/log.h"

#include <Arduino.h>
#include "driver/gpio.h"
#include <ctype.h>
#include <string>
#include <cmath>

namespace esphome {
namespace balboa_9800cp {

static const char *const TAG = "balboa_9800cp";

Balboa9800CP *Balboa9800CP::instance_ = nullptr;
static portMUX_TYPE balboa_mux = portMUX_INITIALIZER_UNLOCKED;


// ---- Control command encoding (Balboa GS 3-bit command @ bit indices 39-41) ----
static constexpr uint8_t CMD_NONE      = 0;
static constexpr uint8_t CMD_MODE      = 1;  // 001
static constexpr uint8_t CMD_TEMP_UP   = 2;  // 100
static constexpr uint8_t CMD_TEMP_DOWN = 3;  // 101
static constexpr uint8_t CMD_LIGHT     = 4;  // 011
static constexpr uint8_t CMD_PUMP1     = 5;  // 110
static constexpr uint8_t CMD_PUMP2     = 6;  // 010
static constexpr uint8_t CMD_BLOWER    = 7;  // 111

static inline uint8_t cmd_bits_3(uint8_t cmd) {
  switch (cmd) {
    case CMD_MODE:      return 0b001;
    case CMD_TEMP_UP:   return 0b100;
    case CMD_TEMP_DOWN: return 0b101;
    case CMD_LIGHT:     return 0b011;
    case CMD_PUMP1:     return 0b110;
    case CMD_PUMP2:     return 0b010;
    case CMD_BLOWER:    return 0b111;
    default:            return 0b000;
  }
}

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

// -----------------------------------------------------------------------
// Intermittent sampling control (reduce ISR/CPU load)
// Capture one frame, then disable clock interrupts until next interval.
// -----------------------------------------------------------------------
static constexpr uint32_t SAMPLE_INTERVAL_MS = 2000;  // adjust (e.g., 2000–5000ms)
static constexpr uint32_t CAPTURE_TIMEOUT_MS  = 200;  // safety timeout for one frame
static bool capture_enabled = false;

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

// ---- Decoder from decoder.txt (7-bit LCD character patterns) ----
// NOTE: decoder.txt returns strings; here we return a single char.
// Any multi-char/ambiguous cases in decoder.txt were commented out there; we keep it the same.
static char seg7_to_char(uint8_t LCD_character) {
  switch (LCD_character & 0x7F) {
    case 0b0000000: return ' ';  // " "
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

    case 0b1110111: return 'A';
    // case 0b0011111: return 'B';
    case 0b1001110: return 'C';
    // case 0b0111101: return 'D';
    case 0b1001111: return 'E';
    // case 0b1000111: return 'F';
    case 0b1011110: return 'G';
    case 0b0110111: return 'H';
    // case 0b0000110: return 'I';
    case 0b0111100: return 'J';
    // case 0b1010111: return 'K';
    case 0b0001110: return 'L';
    case 0b1010100: return 'M';
    case 0b1110110: return 'N';
    // case 0b1111110: return 'O';
    // case 0b1100111: return 'P';
    case 0b1101011: return 'Q';
    case 0b1100110: return 'R';
    // case 0b1011011: return 'S';
    // case 0b0001111: return 'T';
    case 0b0111110: return 'U';
    // case 0b0111110: return 'V';
    case 0b0101010: return 'W';
    // case 0b0110111: return 'X';
    // case 0b0111011: return 'Y';
    // case 0b1101101: return 'Z';

    case 0b1111101: return 'a';
    case 0b0011111: return 'b';
    case 0b0001101: return 'c';
    case 0b0111101: return 'd';
    case 0b1101111: return 'e';
    case 0b1000111: return 'f';

    // case 0b1111011: return 'g';
    case 0b0010111: return 'h';
    case 0b0000100: return 'i';
    case 0b0000001: return 'j';
    case 0b1010111: return 'k';
    case 0b0000110: return 'l';
    case 0b0010100: return 'm';
    case 0b0010101: return 'n';
    case 0b0011101: return 'o';
    case 0b1100111: return 'p';
    // case 0b1110011: return 'q';
    case 0b0000101: return 'r';
    // case 0b1011011: return 's';
    case 0b0001111: return 't';
    case 0b0011100: return 'u';
    // case 0b0011100: return 'v';
    // case 0b0010100: return 'w';
    // case 0b0110111: return 'x';
    case 0b0111011: return 'y';

    default: return '-'; // Error condition
  }
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
  // Extract digits from the 4-char display.
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


// ---------------- Switches ----------------
void BalboaToggleSwitch::write_state(bool state) {
  // The spa buttons toggle; we only press if requested state differs from actual state.
  if (this->parent_ == nullptr) return;

  // Use last-known published state from ESPHome base class.
  // If we don't have feedback yet, still send a press and let feedback correct us.
  if (state != this->state) {
    this->parent_->queue_command(this->cmd_);
  }
  // Optimistic publish; next frame will correct if needed.
  this->publish_state(state);
}

// ---------------- Setpoint number ----------------
void BalboaSetpointNumber::control(float value) {
  if (this->parent_ == nullptr) return;

  // Clamp and round to 1°F steps (80–104°F).
  int target = (int) lroundf(value);
  if (target < 80) target = 80;
  if (target > 104) target = 104;

  this->parent_->request_setpoint_(target);
}

// ---------------- Component ----------------
void Balboa9800CP::set_pins(GPIOPin *clk, GPIOPin *data, GPIOPin *ctrl_in, GPIOPin *ctrl_out) {
  this->clk_ = clk;
  this->data_ = data;
  this->ctrl_in_ = ctrl_in;
  this->ctrl_out_ = ctrl_out;
}

void Balboa9800CP::dump_config() {
  ESP_LOGCONFIG(TAG, "Balboa 9800CP (decoder.txt seg7 + bit-mapped equipment):");
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

  // ✅ CRITICAL: rising edge only (one sample per clock pulse)
  gpio_set_intr_type(clk, GPIO_INTR_POSEDGE);

  gpio_reset_pin(disp);
  gpio_set_direction(disp, GPIO_MODE_INPUT);

  gpio_reset_pin(ctrl);
  gpio_set_direction(ctrl, GPIO_MODE_INPUT);

  // ctrl_out drives 3-bit command during bit indices 39-41
  // Use open-drain so '1' releases the line (pulled up externally / via internal pullup).
  gpio_reset_pin(out);
  gpio_set_pull_mode(out, GPIO_PULLUP_ONLY);
  gpio_set_direction(out, GPIO_MODE_OUTPUT_OD);
  gpio_set_level(out, 1);
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

  // Start with clock interrupt disabled; loop() will enable during sampling windows
  gpio_intr_disable(clk);
  capture_enabled = false;
}

void IRAM_ATTR Balboa9800CP::isr_router_() {
  if (Balboa9800CP::instance_ != nullptr)
    Balboa9800CP::instance_->on_clock_edge_();
}

void IRAM_ATTR Balboa9800CP::on_clock_edge_() {
  if (!capture_enabled) return;
  this->isr_edge_count_++;

  const uint32_t now = micros();
  const uint32_t dt = now - this->last_edge_us_;
  this->last_edge_us_ = now;

  // Resync on long idle gap: start a new frame
  if (dt > this->gap_us_) {
    this->bit_index_ = 0;
    this->frame_ready_ = false;  // ensure capture resumes
  }

  // Frame-freeze guard: don’t overwrite until next resync gap
  if (this->frame_ready_) return;

  const int i = this->bit_index_;
  if (i < 0 || i >= FRAME_BITS) return;

  // Command injection: drive ctrl_out only at bit indices 39-41 (b39,b40,b41).
  // IMPORTANT: Set this early in the ISR to maximize setup time before the spa samples.
  // Line is open-drain; '1' releases (idle/high), '0' pulls low.
  if (this->ctrl_out_gpio_ >= 0) {
    int out_level = 1;  // idle = released/high
    if (this->press_active_ && i >= 39 && i <= 41) {
      const uint8_t bits = cmd_bits_3(this->active_cmd_);
      // Map i=39->MSB (bit2), i=40->bit1, i=41->LSB (bit0)
      const uint8_t shift = (uint8_t) (41 - i);
      out_level = (bits >> shift) & 0x01;
    }
    gpio_set_level((gpio_num_t) this->ctrl_out_gpio_, out_level);
  }

  // Sample both lines on same rising edge
  this->disp_bits_[i] = gpio_get_level((gpio_num_t) this->data_gpio_) ? 1 : 0;
  this->ctrl_bits_[i] = gpio_get_level((gpio_num_t) this->ctrl_in_gpio_) ? 1 : 0;

  this->bit_index_++;

  if (this->bit_index_ >= FRAME_BITS) {
    portENTER_CRITICAL_ISR(&balboa_mux);
    this->frame_ready_ = true;
    if (this->press_active_) {
      if (this->active_frames_left_ > 0) {
        this->active_frames_left_--;
      }
      if (this->active_frames_left_ == 0) {
        this->press_active_ = false;
        this->active_cmd_ = CMD_NONE;
        this->press_finished_ = true;
      }
    }
    portEXIT_CRITICAL_ISR(&balboa_mux);
  }
}


void Balboa9800CP::request_setpoint_(int target_f) {
  // Begin a sync/adjust cycle using the "first press enters setpoint menu" behavior.
  if (target_f < 80) target_f = 80;
  if (target_f > 104) target_f = 104;

  this->target_setpoint_f_ = target_f;
  this->setpoint_sync_active_ = true;
  this->awaiting_setpoint_read_ = true;
  this->pending_adjust_presses_ = 0;
  this->next_adjust_ms_ = 0;

  // Enter setpoint menu without changing value (first press).
  this->queue_command(CMD_TEMP_UP);

  // Don't immediately change the number entity; we'll validate by reading the displayed setpoint.
}

void Balboa9800CP::loop() {
  const uint32_t now = millis();

  // Service queued commands (enforces inter-press delay)
  this->service_command_queue_();

  // -----------------------------------------------------------------------
  // Intermittent sampling scheduler:
  // Enable clock interrupt briefly to capture one frame, then disable.
  // -----------------------------------------------------------------------
  static uint32_t next_sample_ms = 0;
  static uint32_t capture_start_ms = 0;

  if (!capture_enabled && now >= next_sample_ms) {
    capture_enabled = true;
    capture_start_ms = now;

    // Reset capture state for a fresh frame
    portENTER_CRITICAL(&balboa_mux);
    this->bit_index_ = 0;
    this->frame_ready_ = false;
    portEXIT_CRITICAL(&balboa_mux);

    gpio_intr_enable((gpio_num_t) this->clk_gpio_);
  }

  if (capture_enabled) {
    bool done = false;
    portENTER_CRITICAL(&balboa_mux);
    done = this->frame_ready_;
    portEXIT_CRITICAL(&balboa_mux);

    const bool command_active =
        (this->press_active_) || (!this->queue_empty_()) ||
        this->setpoint_sync_active_ || this->awaiting_setpoint_read_ || (this->pending_adjust_presses_ != 0);

    if ((done || (now - capture_start_ms) > CAPTURE_TIMEOUT_MS) && !command_active) {
      gpio_intr_disable((gpio_num_t) this->clk_gpio_);
      capture_enabled = false;
      next_sample_ms = now + SAMPLE_INTERVAL_MS;
    }
  }

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
    this->frame_ready_ = false;
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

  // Decode display (4 chars) -- keep your established order
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

  // Publish temp only when numeric; otherwise keep last
  // Temperature handling:
  // - Normal case: publish water temperature to HA.
  // - Immediately after temp up/down activity, the topside briefly shows SETPOINT.
  //   We capture that separately and do NOT publish it as water temperature.
  const uint32_t setpoint_window_ms = 6000;  // conservative window for setpoint display after a temp button press

  int parsed_temp_f = 0;
  const bool have_temp = parse_reasonable_temp_f(disp, parsed_temp_f);

  const bool in_setpoint_window =
      (this->last_temp_cmd_ms_ != 0U) && ((now - this->last_temp_cmd_ms_) <= setpoint_window_ms);

  if (have_temp && in_setpoint_window) {
    // Treat display as SETPOINT (momentary set-temp menu).
    this->current_setpoint_f_ = parsed_temp_f;
    this->current_setpoint_valid_ = (parsed_temp_f >= 80 && parsed_temp_f <= 104);

    if (this->setpoint_number_ != nullptr && this->current_setpoint_valid_) {
      this->setpoint_number_->publish_state((float) this->current_setpoint_f_);
    }
  } else if (this->water_temp_ != nullptr) {
    if (have_temp) {
      last_temp_f_ = (float) parsed_temp_f;
      have_last_temp_ = true;
      this->water_temp_->publish_state(last_temp_f_);
    } else {
      // keep last known temp in HA
    }
  }


  // -----------------------------------------------------------------------
  // Setpoint sync/adjust state machine:
  // 1) request_setpoint_() presses TEMP_UP once to enter setpoint menu (no change).
  // 2) We read the momentary setpoint display and compute delta.
  // 3) We issue the needed TEMP_UP/DOWN presses to reach target.
  // -----------------------------------------------------------------------
  if (this->setpoint_sync_active_) {
    // Once we have a valid setpoint readout, compute remaining presses.
    if (this->awaiting_setpoint_read_ && this->current_setpoint_valid_) {
      this->awaiting_setpoint_read_ = false;

      const int delta = this->target_setpoint_f_ - this->current_setpoint_f_;
      this->pending_adjust_presses_ = delta;

      // Small delay before starting adjustments so the menu is stable.
      this->next_adjust_ms_ = now + 50;
      ESP_LOGI(TAG, "Setpoint sync: current=%dF target=%dF delta=%d",
               this->current_setpoint_f_, this->target_setpoint_f_, delta);
    }

    // Issue step presses at a controlled rate.
    if (!this->awaiting_setpoint_read_ && this->pending_adjust_presses_ != 0 && now >= this->next_adjust_ms_) {
      if (this->pending_adjust_presses_ > 0) {
        this->queue_command(CMD_TEMP_UP);
        this->pending_adjust_presses_--;
      } else {
        this->queue_command(CMD_TEMP_DOWN);
        this->pending_adjust_presses_++;
      }
      this->next_adjust_ms_ = now + 50;
    }

    // Done when no remaining presses and we're not waiting on read.
    if (!this->awaiting_setpoint_read_ && this->pending_adjust_presses_ == 0) {
      // We'll leave setpoint_number_ at last observed value; the topside may keep showing setpoint briefly.
      this->setpoint_sync_active_ = false;
      ESP_LOGI(TAG, "Setpoint sync complete (target %dF).", this->target_setpoint_f_);
    }
  }

  // "Set heat" heuristic
  const bool set_heat = looks_like_set_mode(disp);
  if (this->set_heat_ != nullptr) this->set_heat_->publish_state(set_heat);

  // Mapped equipment bits (raw DISP frame indices)
  // 35=Heater, 36=Pump1, 37=Pump2, 38=Air Blower
  const bool heater_on = last_disp_[35] != 0;
  const bool pump1_on  = last_disp_[36] != 0;
  const bool pump2_on  = last_disp_[37] != 0;
  const bool blower_on = last_disp_[38] != 0;

  if (this->heating_ != nullptr) this->heating_->publish_state(heater_on);
  if (this->pump_ != nullptr) this->pump_->publish_state(pump1_on);
  if (this->jets_ != nullptr) this->jets_->publish_state(pump2_on);
  if (this->blower_ != nullptr) this->blower_->publish_state(blower_on);

  // NEW: Switch entities mirror the same state bits (36/37/38)
  if (this->pump1_switch_ != nullptr) this->pump1_switch_->publish_state(pump1_on);
  if (this->pump2_switch_ != nullptr) this->pump2_switch_->publish_state(pump2_on);
  if (this->blower_switch_ != nullptr) this->blower_switch_->publish_state(blower_on);
  if (this->light_ != nullptr) this->light_->publish_state(false);

  // Keep other binary sensors publishing “unknown/false” until we map them:
  if (this->mode_standard_ != nullptr) this->mode_standard_->publish_state(false);
  if (this->temp_up_display_ != nullptr) this->temp_up_display_->publish_state(false);
  if (this->temp_down_display_ != nullptr) this->temp_down_display_->publish_state(false);
  if (this->inverted_ != nullptr) this->inverted_->publish_state(false);

  // Diff logging: STAT/EQ changes and which bits flipped
  if (got_frame) {
    if (!have_prev_stat_eq_) {
      have_prev_stat_eq_ = true;
      prev_stat_ = stat;
      prev_eq_ = eq;
      ESP_LOGI(TAG, "Initial STAT=%s  EQ=%s  Display=\"%s\" Pump1=%d",
               bits7(stat).c_str(), bits7(eq).c_str(), disp.c_str(), (int) pump1_on);
    } else {
      const uint8_t dstat = (uint8_t) (prev_stat_ ^ stat);
      const uint8_t deq   = (uint8_t) (prev_eq_ ^ eq);

      if (dstat != 0 || deq != 0) {
        ESP_LOGI(TAG, "CHANGE Display=\"%s\"  STAT %s->%s (xor %s)  EQ %s->%s (xor %s) Pump1=%d",
                 disp.c_str(),
                 bits7(prev_stat_).c_str(), bits7(stat).c_str(), bits7(dstat).c_str(),
                 bits7(prev_eq_).c_str(),   bits7(eq).c_str(),   bits7(deq).c_str(),
                 (int) pump1_on);

        prev_stat_ = stat;
        prev_eq_ = eq;
      }
    }
  }

  // Optional: raw frame once/sec (unchanged from your file)
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

    ESP_LOGI(TAG, "Chunks: D1=%s D2=%s D3=%s D4=%s STAT=%s EQ=%s  Display=\"%s\" Pump1=%d",
             bits7(d1).c_str(), bits7(d2).c_str(), bits7(d3).c_str(), bits7(d4).c_str(),
             bits7(stat).c_str(), bits7(eq).c_str(), disp.c_str(), (int) pump1_on);
  }
}


// ---------------- Command queue + timing ----------------
bool Balboa9800CP::queue_empty_() const {
  return this->cmd_q_head_ == this->cmd_q_tail_;
}

bool Balboa9800CP::queue_full_() const {
  return (uint8_t) ((this->cmd_q_tail_ + 1U) % CMD_QUEUE_SIZE) == this->cmd_q_head_;
}

bool Balboa9800CP::queue_push_(uint8_t cmd) {
  if (this->queue_full_()) return false;
  this->cmd_queue_[this->cmd_q_tail_] = cmd;
  this->cmd_q_tail_ = (uint8_t) ((this->cmd_q_tail_ + 1U) % CMD_QUEUE_SIZE);
  return true;
}

bool Balboa9800CP::queue_pop_(uint8_t &cmd) {
  if (this->queue_empty_()) return false;
  cmd = this->cmd_queue_[this->cmd_q_head_];
  this->cmd_q_head_ = (uint8_t) ((this->cmd_q_head_ + 1U) % CMD_QUEUE_SIZE);
  return true;
}

void Balboa9800CP::enqueue_command(uint8_t cmd) {
  if (cmd == CMD_NONE) return;

  if (!this->queue_push_(cmd)) {
    ESP_LOGW(TAG, "Command queue full; dropping cmd=%u", cmd);
    return;
  }

  const uint32_t now_ms = millis();
  if (cmd == CMD_TEMP_UP || cmd == CMD_TEMP_DOWN) {
    this->last_temp_cmd_ms_ = now_ms;
  }

  // Ensure sampling is enabled so ISR runs and can inject commands.
  if (!capture_enabled && this->clk_gpio_ >= 0) {
    capture_enabled = true;
    gpio_intr_enable((gpio_num_t) this->clk_gpio_);
  }
}

void Balboa9800CP::start_press_(uint8_t cmd) {
  this->active_cmd_ = cmd;
  this->active_frames_left_ = this->press_frames_;
  this->press_active_ = true;
}

void Balboa9800CP::service_command_queue_() {
  // If ISR marked a press finished, timestamp it here (never call millis() in ISR).
  if (this->press_finished_) {
    this->press_finished_ = false;
    this->last_press_end_ms_ = millis();
  }

  if (this->press_active_) return;

  if (this->last_press_end_ms_ != 0U) {
    const uint32_t since = millis() - this->last_press_end_ms_;
    if (since < INTER_PRESS_DELAY_MS) return;
  }

  uint8_t cmd;
  if (!this->queue_pop_(cmd)) return;

  this->start_press_(cmd);
}

void Balboa9800CP::process_frame_() {
  // Not used
}

}  // namespace balboa_9800cp
}  // namespace esphome
//comment to test for OTA upload
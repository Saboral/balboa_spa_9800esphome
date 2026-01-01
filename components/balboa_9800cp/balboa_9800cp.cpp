#include "balboa_9800cp.h"
#include "esphome/core/log.h"

#include <Arduino.h>
#include "driver/gpio.h"

namespace esphome {
namespace balboa_9800cp {

static const char *const TAG = "balboa_9800cp";

Balboa9800CP *Balboa9800CP::instance_ = nullptr;
static portMUX_TYPE balboa_mux = portMUX_INITIALIZER_UNLOCKED;

// Protocol observed: 42 bits = 6 chunks × 7 bits (LSB-first)
static constexpr int FRAME_BITS = 42;
static constexpr int CHUNK_BITS = 7;

// Cache last completed frame (so loop never races ISR)
static uint8_t last_disp_[76];
static uint8_t last_ctrl_[76];
static bool last_valid_ = false;

// ---- helpers ----
static inline uint8_t bits_to_u7_lsb_first(const uint8_t *bits, int start_bit) {
  uint8_t v = 0;
  for (int k = 0; k < CHUNK_BITS; k++) {
    if (bits[start_bit + k]) v |= (1U << k);
  }
  return v & 0x7F;
}

// Strict 7-seg mapping for *this wiring/protocol* (based on your captured chunks)
// Unknown patterns return '?'
static inline char seg7_to_char(uint8_t s) {
  switch (s) {
    // Digits seen and/or derived (many are bit-reverse of common 7seg, except '1' proved as 0x39 in your logs)
    case 0x7E: return '0';
    case 0x39: return '1';   // proven by your logs (D1=0x39 corresponds to leading '1')
    case 0x6D: return '2';   // proven by your logs (D2=0x6D corresponds to '2' in 102F)
    case 0x79: return '3';   // best-effort derived; will show '?' if wrong
    case 0x33: return '4';   // best-effort derived
    case 0x5B: return '5';   // best-effort derived (NOTE: NOT 0x6D)
    case 0x5F: return '6';   // best-effort derived
    case 0x70: return '7';   // best-effort derived
    case 0x7F: return '8';   // best-effort derived (same both ways for many wirings)
    case 0x7B: return '9';   // best-effort derived

    // Units / letters (as observed: D4=0x06 is 'F' in your logs)
    case 0x06: return 'F';

    // Common extras (best-effort)
    case 0x00: return ' ';
    case 0x40: return '-';

    default: return '?';
  }
}

static inline bool parse_temp_f_from_display(const char *disp, float *out_f) {
  // Expect formats like "102F", " 98F", etc.
  // We’ll extract up to 3 digits before trailing 'F'.
  if (!disp) return false;

  int len = (int) strlen(disp);
  if (len < 2) return false;
  if (disp[len - 1] != 'F') return false;

  int value = 0;
  int digits = 0;
  for (int i = 0; i < len - 1; i++) {
    if (disp[i] >= '0' && disp[i] <= '9') {
      value = value * 10 + (disp[i] - '0');
      digits++;
      if (digits >= 3) break;
    }
  }
  if (digits == 0) return false;
  *out_f = (float) value;
  return true;
}

// ---- Button ----
void BalboaButton::press_action() {
  if (this->parent_ != nullptr) this->parent_->queue_command(this->cmd_);
}

// ---- Component ----
void Balboa9800CP::set_pins(GPIOPin *clk, GPIOPin *data, GPIOPin *ctrl_in, GPIOPin *ctrl_out) {
  this->clk_ = clk;
  this->data_ = data;
  this->ctrl_in_ = ctrl_in;
  this->ctrl_out_ = ctrl_out;
}

void Balboa9800CP::dump_config() {
  ESP_LOGCONFIG(TAG, "Balboa 9800CP (strict seg7 + Pump bits):");
  ESP_LOGCONFIG(TAG, "  clk_gpio: %d", this->clk_gpio_);
  ESP_LOGCONFIG(TAG, "  display_data_gpio (Pin5): %d", this->data_gpio_);
  ESP_LOGCONFIG(TAG, "  control_gpio (Pin3): %d", this->ctrl_in_gpio_);
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

  gpio_num_t clk = (gpio_num_t) this->clk_gpio_;
  gpio_num_t disp = (gpio_num_t) this->data_gpio_;
  gpio_num_t ctrl = (gpio_num_t) this->ctrl_in_gpio_;
  gpio_num_t out = (gpio_num_t) this->ctrl_out_gpio_;

  gpio_reset_pin(clk);
  gpio_set_direction(clk, GPIO_MODE_INPUT);
  gpio_set_intr_type(clk, GPIO_INTR_ANYEDGE);

  gpio_reset_pin(disp);
  gpio_set_direction(disp, GPIO_MODE_INPUT);

  gpio_reset_pin(ctrl);
  gpio_set_direction(ctrl, GPIO_MODE_INPUT);

  gpio_reset_pin(out);
  gpio_set_direction(out, GPIO_MODE_INPUT);  // keep high-Z for now (no injection yet)

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

  ESP_LOGI(TAG, "ISR attached. clk=%d disp(pin5)=%d ctrl(pin3)=%d out=%d",
           this->clk_gpio_, this->data_gpio_, this->ctrl_in_gpio_, this->ctrl_out_gpio_);
}

void IRAM_ATTR Balboa9800CP::isr_router_() {
  if (Balboa9800CP::instance_ != nullptr) Balboa9800CP::instance_->on_clock_edge_();
}

void IRAM_ATTR Balboa9800CP::on_clock_edge_() {
  this->isr_edge_count_++;

  const uint32_t now = micros();
  const uint32_t dt = now - this->last_edge_us_;
  this->last_edge_us_ = now;

  if (dt > this->gap_us_) this->bit_index_ = 0;

  const int i = this->bit_index_;
  if (i < 0 || i >= 76) return;

  // Sample BOTH lines on the same clock edge:
  this->disp_bits_[i] = gpio_get_level((gpio_num_t) this->data_gpio_) ? 1 : 0;
  this->ctrl_bits_[i] = gpio_get_level((gpio_num_t) this->ctrl_in_gpio_) ? 1 : 0;

  this->bit_index_++;

  // We only need the first 42 bits for 6×7 decoding
  if (this->bit_index_ >= FRAME_BITS) {
    portENTER_CRITICAL_ISR(&balboa_mux);
    this->frame_ready_ = true;
    portEXIT_CRITICAL_ISR(&balboa_mux);
    this->bit_index_ = 0;  // restart cleanly for next frame
  }
}

void Balboa9800CP::queue_command(uint8_t cmd) {
  (void) cmd;
  // Injection not enabled yet in this phase.
}

void Balboa9800CP::process_frame_() {
  // not used
}

void Balboa9800CP::loop() {
  const uint32_t now = millis();

  // 1 Hz ISR activity debug
  static uint32_t last_edges_ms = 0;
  if (now - last_edges_ms >= 1000) {
    last_edges_ms = now;
    const uint32_t edges = this->isr_edge_count_;
    this->isr_edge_count_ = 0;
    ESP_LOGD(TAG, "clk edges/sec=%u bit_index=%d frame_ready=%d",
             (unsigned) edges, this->bit_index_, this->frame_ready_ ? 1 : 0);
  }

  // Cache latest completed frame
  bool got = false;
  portENTER_CRITICAL(&balboa_mux);
  if (this->frame_ready_) {
    for (int i = 0; i < FRAME_BITS; i++) {
      last_disp_[i] = this->disp_bits_[i];
      last_ctrl_[i] = this->ctrl_bits_[i];
    }
    last_valid_ = true;
    this->frame_ready_ = false;
    got = true;
  }
  portEXIT_CRITICAL(&balboa_mux);

  if (!last_valid_ || !got) return;

  // Build 6 chunks (LSB-first 7-bit each) from display line
  const uint8_t d1 = bits_to_u7_lsb_first(last_disp_, 0);
  const uint8_t d2 = bits_to_u7_lsb_first(last_disp_, 7);
  const uint8_t d3 = bits_to_u7_lsb_first(last_disp_, 14);
  const uint8_t d4 = bits_to_u7_lsb_first(last_disp_, 21);
  const uint8_t stat = bits_to_u7_lsb_first(last_disp_, 28);
  const uint8_t eq = bits_to_u7_lsb_first(last_disp_, 35);

  // Display char order that matches your observed logs: D1, D3, D2, D4
  char disp[5];
  disp[0] = seg7_to_char(d1);
  disp[1] = seg7_to_char(d3);
  disp[2] = seg7_to_char(d2);
  disp[3] = seg7_to_char(d4);
  disp[4] = '\0';

  // Bits proven from your logs:
  const bool pump1 = (eq & 0x20) != 0;  // Pump1
  const bool pump2 = (eq & 0x10) != 0;  // Pump2
  const bool light = (eq & 0x08) != 0;  // Light

  // Best-effort: setpoint indicator observed as STAT bit 0x40 toggling during set temp display
  const bool showing_setpoint = (stat & 0x40) != 0;

  // Publish entities
  if (this->display_text_ != nullptr) this->display_text_->publish_state(std::string(disp));

  // Water temp: only publish when we're NOT showing setpoint (so we treat that as "current temp")
  float tf = 0;
  if (!showing_setpoint && this->water_temp_ != nullptr && parse_temp_f_from_display(disp, &tf)) {
    this->water_temp_->publish_state(tf);
  }

  if (this->set_heat_ != nullptr) this->set_heat_->publish_state(showing_setpoint);

  // Map your YAML sensors to what we now know:
  if (this->pump_ != nullptr) this->pump_->publish_state(pump1);     // "Spa Pump" = Pump1
  if (this->jets_ != nullptr) this->jets_->publish_state(pump2);     // "Spa Jets" = Pump2
  if (this->light_ != nullptr) this->light_->publish_state(light);   // Light

  // Leave these as placeholders for now (until we positively identify their bits)
  if (this->blower_ != nullptr) this->blower_->publish_state((eq & 0x04) != 0);   // guess
  if (this->heating_ != nullptr) this->heating_->publish_state((eq & 0x01) != 0); // guess
  if (this->mode_standard_ != nullptr) this->mode_standard_->publish_state(false);
  if (this->temp_up_display_ != nullptr) this->temp_up_display_->publish_state(false);
  if (this->temp_down_display_ != nullptr) this->temp_down_display_->publish_state(false);
  if (this->inverted_ != nullptr) this->inverted_->publish_state(false);

  // Change logging (very similar to what you’re seeing)
  static char last_disp_str[5] = "----";
  static uint8_t last_stat = 0xFF;
  static uint8_t last_eq = 0xFF;

  if (strncmp(last_disp_str, disp, 4) != 0 || last_stat != stat || last_eq != eq) {
    ESP_LOGI(TAG, "CHANGE Display=\"%s\"  STAT 0x%02X->0x%02X (xor 0x%02X)  EQ 0x%02X->0x%02X (xor 0x%02X) Pump1=%d",
             disp,
             last_stat, stat, (uint8_t)(last_stat ^ stat),
             last_eq, eq, (uint8_t)(last_eq ^ eq),
             pump1 ? 1 : 0);

    last_disp_str[0] = disp[0];
    last_disp_str[1] = disp[1];
    last_disp_str[2] = disp[2];
    last_disp_str[3] = disp[3];
    last_disp_str[4] = '\0';
    last_stat = stat;
    last_eq = eq;
  }

  // Periodic detailed chunk log (1 Hz)
  static uint32_t last_chunk_ms = 0;
  if (now - last_chunk_ms >= 1000) {
    last_chunk_ms = now;

    // Print the 42 captured bits as a sanity check
    char bits42[FRAME_BITS + 1];
    for (int i = 0; i < FRAME_BITS; i++) bits42[i] = last_disp_[i] ? '1' : '0';
    bits42[FRAME_BITS] = '\0';

    ESP_LOGI(TAG, "DISP(pin5) bits[%d]: %s", FRAME_BITS, bits42);
    ESP_LOGI(TAG, "Chunks: D1=0x%02X D2=0x%02X D3=0x%02X D4=0x%02X STAT=0x%02X EQ=0x%02X  Display=\"%s\" Pump1=%d",
             d1, d2, d3, d4, stat, eq, disp, pump1 ? 1 : 0);
  }
}

}  // namespace balboa_9800cp
}  // namespace esphome

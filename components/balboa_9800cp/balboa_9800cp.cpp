#include "balboa_9800cp.h"
#include "esphome/core/log.h"

#include <Arduino.h>
#include "driver/gpio.h"

namespace esphome {
namespace balboa_9800cp {

static const char *const TAG = "balboa_9800cp";

Balboa9800CP *Balboa9800CP::instance_ = nullptr;
static portMUX_TYPE balboa_mux = portMUX_INITIALIZER_UNLOCKED;

// ---- Protocol constants (this step) ----
static constexpr int FRAME_BITS = 42;        // 6 chunks * 7 bits
static constexpr int CHUNK_BITS = 7;
static constexpr int CHUNK_COUNT = 6;

// Cache last completed frames (so printing never loses the race)
static uint8_t last_disp_[76];
static uint8_t last_ctrl_[76];
static bool last_valid_ = false;

void BalboaButton::press_action() {
  if (this->parent_ != nullptr)
    this->parent_->queue_command(this->cmd_);
}

void Balboa9800CP::set_pins(GPIOPin *clk, GPIOPin *data, GPIOPin *ctrl_in, GPIOPin *ctrl_out) {
  this->clk_ = clk;
  this->data_ = data;
  this->ctrl_in_ = ctrl_in;
  this->ctrl_out_ = ctrl_out;
}

void Balboa9800CP::dump_config() {
  ESP_LOGCONFIG(TAG, "Balboa 9800CP (Fixed 42-bit frame logger):");
  ESP_LOGCONFIG(TAG, "  clk_gpio: %d", this->clk_gpio_);
  ESP_LOGCONFIG(TAG, "  display_data_gpio (Pin5): %d", this->data_gpio_);
  ESP_LOGCONFIG(TAG, "  control_gpio (Pin3): %d", this->ctrl_in_gpio_);
  ESP_LOGCONFIG(TAG, "  ctrl_out_gpio: %d", this->ctrl_out_gpio_);
  ESP_LOGCONFIG(TAG, "  gap_us (resync only): %u", (unsigned) this->gap_us_);
  ESP_LOGCONFIG(TAG, "  frame_bits: %d (6x7)", FRAME_BITS);
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
  // IMPORTANT: rising edge only (one sample per clock pulse)
  gpio_set_intr_type(clk, GPIO_INTR_POSEDGE);

  gpio_reset_pin(disp);
  gpio_set_direction(disp, GPIO_MODE_INPUT);

  gpio_reset_pin(ctrl);
  gpio_set_direction(ctrl, GPIO_MODE_INPUT);

  // keep high-Z for now (injection disabled at this step)
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

  // Reset capture state
  this->bit_index_ = 0;
  this->frame_ready_ = false;
  this->last_edge_us_ = micros();
  this->isr_edge_count_ = 0;

  // clear buffers
  for (int i = 0; i < 76; i++) {
    this->disp_bits_[i] = 0;
    this->ctrl_bits_[i] = 0;
    last_disp_[i] = 0;
    last_ctrl_[i] = 0;
  }
  last_valid_ = false;

  ESP_LOGI(TAG, "ISR attached (POSEDGE). clk=%d disp(pin5)=%d ctrl(pin3)=%d out=%d",
           this->clk_gpio_, this->data_gpio_, this->ctrl_in_gpio_, this->ctrl_out_gpio_);
}

void IRAM_ATTR Balboa9800CP::isr_router_() {
  if (Balboa9800CP::instance_ != nullptr)
    Balboa9800CP::instance_->on_clock_edge_();
}

// Helper: pack 7 bits (LSB-first) into a byte
static inline uint8_t pack7(const uint8_t *bits, int start) {
  uint8_t v = 0;
  for (int k = 0; k < 7; k++) {
    v |= (bits[start + k] ? 1 : 0) << k;
  }
  return v & 0x7F;
}

void IRAM_ATTR Balboa9800CP::on_clock_edge_() {
  this->isr_edge_count_++;

  const uint32_t now = micros();
  const uint32_t dt = now - this->last_edge_us_;
  this->last_edge_us_ = now;

  // Safety resync: if we see a long idle gap, start a new frame
  if (dt > this->gap_us_) {
    this->bit_index_ = 0;
  }

  // If we already captured a full frame and main loop hasn't consumed it yet,
  // ignore edges until a resync gap resets bit_index_.
  if (this->frame_ready_) return;

  const int i = this->bit_index_;

  if (i < 0 || i >= FRAME_BITS) {
    return;
  }

  // Sample BOTH lines on the same rising clock edge:
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
  // Injection disabled in this step (logger + decode stabilization)
}

void Balboa9800CP::loop() {
  const uint32_t now = millis();

  // 1 Hz ISR activity
  static uint32_t last_edges_ms = 0;
  if (now - last_edges_ms >= 1000) {
    last_edges_ms = now;
    const uint32_t edges = this->isr_edge_count_;
    this->isr_edge_count_ = 0;
    ESP_LOGD(TAG, "clk edges/sec=%u bit_index=%d frame_ready=%d",
             (unsigned) edges, this->bit_index_, this->frame_ready_ ? 1 : 0);
  }

  // Cache latest completed frame (copy only first FRAME_BITS; zero rest)
  portENTER_CRITICAL(&balboa_mux);
  if (this->frame_ready_) {
    for (int i = 0; i < 76; i++) {
      if (i < FRAME_BITS) {
        last_disp_[i] = this->disp_bits_[i];
        last_ctrl_[i] = this->ctrl_bits_[i];
      } else {
        last_disp_[i] = 0;
        last_ctrl_[i] = 0;
      }
    }
    last_valid_ = true;
    this->frame_ready_ = false;
  }
  portEXIT_CRITICAL(&balboa_mux);

  // Print once/sec
  static uint32_t last_print_ms = 0;
  if (!last_valid_ || (now - last_print_ms) < 1000) return;
  last_print_ms = now;

  // Raw bit strings (first 42 bits only)
  char disp_raw[FRAME_BITS + 1];
  char ctrl_raw[FRAME_BITS + 1];
  for (int i = 0; i < FRAME_BITS; i++) {
    disp_raw[i] = last_disp_[i] ? '1' : '0';
    ctrl_raw[i] = last_ctrl_[i] ? '1' : '0';
  }
  disp_raw[FRAME_BITS] = '\0';
  ctrl_raw[FRAME_BITS] = '\0';

  // Chunk decode (6x7)
  uint8_t d[CHUNK_COUNT];
  uint8_t c[CHUNK_COUNT];
  for (int ch = 0; ch < CHUNK_COUNT; ch++) {
    d[ch] = pack7(last_disp_, ch * CHUNK_BITS);
    c[ch] = pack7(last_ctrl_, ch * CHUNK_BITS);
  }

  ESP_LOGI(TAG, "DISP(pin5) bits[%d]: %s", FRAME_BITS, disp_raw);
  ESP_LOGI(TAG, "CTRL(pin3) bits[%d]: %s", FRAME_BITS, ctrl_raw);

  ESP_LOGI(TAG, "DISP chunks: D1=0x%02X D2=0x%02X D3=0x%02X D4=0x%02X STAT=0x%02X EQ=0x%02X",
           d[0], d[1], d[2], d[3], d[4], d[5]);

  ESP_LOGI(TAG, "CTRL chunks: C1=0x%02X C2=0x%02X C3=0x%02X C4=0x%02X C5=0x%02X C6=0x%02X",
           c[0], c[1], c[2], c[3], c[4], c[5]);
}

void Balboa9800CP::process_frame_() {
  // Not used in this logger build
}

}  // namespace balboa_9800cp
}  // namespace esphome

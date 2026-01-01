#include "balboa_9800cp.h"
#include "esphome/core/log.h"

#include <Arduino.h>
#include "driver/gpio.h"

namespace esphome {
namespace balboa_9800cp {

static const char *const TAG = "balboa_9800cp";

Balboa9800CP *Balboa9800CP::instance_ = nullptr;
static portMUX_TYPE balboa_mux = portMUX_INITIALIZER_UNLOCKED;

// ثابت: 42 bits = 6 chunks × 7 bits
static constexpr int FRAME_BITS = 42;
static constexpr int CHUNK_BITS = 7;

static uint8_t last_disp_[FRAME_BITS];
static uint8_t last_ctrl_[FRAME_BITS];
static bool last_valid_ = false;

// For change detection
static bool have_prev_ = false;
static uint8_t prev_stat_ = 0;
static uint8_t prev_eq_ = 0;

// Pack 7 bits LSB-first (the original stable logger style)
// If you want MSB-first again later, we can flip this *after* stability is confirmed.
static inline uint8_t pack7_lsb(const uint8_t *bits, int start) {
  uint8_t v = 0;
  for (int k = 0; k < CHUNK_BITS; k++) {
    if (bits[start + k]) v |= (1U << k);
  }
  return v & 0x7F;
}

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
  ESP_LOGCONFIG(TAG, "Balboa 9800CP (ROLLBACK stable capture):");
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

  // ✅ IMPORTANT: rising edge only
  gpio_set_intr_type(clk, GPIO_INTR_POSEDGE);

  gpio_reset_pin(disp);
  gpio_set_direction(disp, GPIO_MODE_INPUT);

  gpio_reset_pin(ctrl);
  gpio_set_direction(ctrl, GPIO_MODE_INPUT);

  // keep high-Z (no injection)
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
  have_prev_ = false;

  ESP_LOGI(TAG, "ISR attached (POSEDGE). clk=%d disp=%d ctrl_in=%d out=%d",
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

  // Gap-based resync: start a new frame
  if (dt > this->gap_us_) {
    this->bit_index_ = 0;
    this->frame_ready_ = false;
  }

  // Freeze until main loop consumes the frame
  if (this->frame_ready_) return;

  const int i = this->bit_index_;
  if (i < 0 || i >= FRAME_BITS) return;

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
  // injection disabled in rollback build
}

void Balboa9800CP::loop() {
  const uint32_t now = millis();

  // Edge counter once per second
  static uint32_t last_edges_ms = 0;
  if (now - last_edges_ms >= 1000) {
    last_edges_ms = now;
    const uint32_t edges = this->isr_edge_count_;
    this->isr_edge_count_ = 0;
    ESP_LOGD(TAG, "clk edges/sec=%u bit_index=%d frame_ready=%d",
             (unsigned) edges, this->bit_index_, (int) this->frame_ready_);
  }

  // Pull completed frame if available
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

  if (!last_valid_ || !got_frame) return;

  // Decode chunks (LSB-first stable baseline)
  const uint8_t d1   = pack7_lsb(last_disp_, 0);
  const uint8_t d2   = pack7_lsb(last_disp_, 7);
  const uint8_t d3   = pack7_lsb(last_disp_, 14);
  const uint8_t d4   = pack7_lsb(last_disp_, 21);
  const uint8_t stat = pack7_lsb(last_disp_, 28);
  const uint8_t eq   = pack7_lsb(last_disp_, 35);

  // Log *only* changes in STAT/EQ (this is what we need for mapping)
  if (!have_prev_) {
    have_prev_ = true;
    prev_stat_ = stat;
    prev_eq_ = eq;
    ESP_LOGI(TAG, "Initial: D1=0x%02X D2=0x%02X D3=0x%02X D4=0x%02X STAT=0x%02X EQ=0x%02X",
             d1, d2, d3, d4, stat, eq);
  } else {
    const uint8_t dstat = (uint8_t)(prev_stat_ ^ stat);
    const uint8_t deq   = (uint8_t)(prev_eq_ ^ eq);
    if (dstat || deq) {
      ESP_LOGI(TAG, "CHANGE: STAT 0x%02X->0x%02X xor 0x%02X | EQ 0x%02X->0x%02X xor 0x%02X",
               prev_stat_, stat, dstat, prev_eq_, eq, deq);
      prev_stat_ = stat;
      prev_eq_ = eq;
    }
  }

  // One compact raw snapshot per second (not spam)
  static uint32_t last_dump_ms = 0;
  if (now - last_dump_ms >= 1000) {
    last_dump_ms = now;

    char disp_bits_str[FRAME_BITS + 1];
    for (int i = 0; i < FRAME_BITS; i++)
      disp_bits_str[i] = last_disp_[i] ? '1' : '0';
    disp_bits_str[FRAME_BITS] = '\0';

    ESP_LOGI(TAG, "DISP bits[%d]: %s", FRAME_BITS, disp_bits_str);
    ESP_LOGI(TAG, "Chunks: D1=0x%02X D2=0x%02X D3=0x%02X D4=0x%02X STAT=0x%02X EQ=0x%02X",
             d1, d2, d3, d4, stat, eq);
  }
}

void Balboa9800CP::process_frame_() {
  // not used in rollback build
}

}  // namespace balboa_9800cp
}  // namespace esphome

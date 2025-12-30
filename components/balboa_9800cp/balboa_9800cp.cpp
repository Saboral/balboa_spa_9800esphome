#include "balboa_9800cp.h"
#include "esphome/core/log.h"

#include <Arduino.h>
#include "driver/gpio.h"

namespace esphome {
namespace balboa_9800cp {

static const char *const TAG = "balboa_9800cp";

Balboa9800CP *Balboa9800CP::instance_ = nullptr;
static portMUX_TYPE balboa_mux = portMUX_INITIALIZER_UNLOCKED;

void BalboaButton::press_action() {
  if (this->parent_ != nullptr) this->parent_->queue_command(this->cmd_);
}

void Balboa9800CP::set_pins(GPIOPin *clk, GPIOPin *data, GPIOPin *ctrl_in, GPIOPin *ctrl_out) {
  this->clk_ = clk;
  this->data_ = data;
  this->ctrl_in_ = ctrl_in;
  this->ctrl_out_ = ctrl_out;
}

void Balboa9800CP::dump_config() {
  ESP_LOGCONFIG(TAG, "Balboa 9800CP (Step 1 RAW logger):");
  ESP_LOGCONFIG(TAG, "  clk_gpio: %d", this->clk_gpio_);
  ESP_LOGCONFIG(TAG, "  data_gpio: %d", this->data_gpio_);
  ESP_LOGCONFIG(TAG, "  ctrl_in_gpio: %d", this->ctrl_in_gpio_);
  ESP_LOGCONFIG(TAG, "  ctrl_out_gpio: %d", this->ctrl_out_gpio_);
  ESP_LOGCONFIG(TAG, "  gap_us: %u", (unsigned) this->gap_us_);
  ESP_LOGCONFIG(TAG, "  press_frames: %u", (unsigned) this->press_frames_);
}

void Balboa9800CP::setup() {
  ESP_LOGI(TAG, "setup() reached");
  instance_ = this;

  // Safe outside ISR: allow ESPHome to configure pins too
  if (this->clk_) this->clk_->setup();
  if (this->data_) this->data_->setup();
  if (this->ctrl_in_) this->ctrl_in_->setup();
  if (this->ctrl_out_) this->ctrl_out_->setup();

  // Require raw GPIO numbers for ISR-safe reads/writes
  if (this->clk_gpio_ < 0 || this->data_gpio_ < 0 || this->ctrl_in_gpio_ < 0 || this->ctrl_out_gpio_ < 0) {
    ESP_LOGE(TAG, "GPIO numbers not set. clk=%d data=%d ctrl_in=%d ctrl_out=%d",
             this->clk_gpio_, this->data_gpio_, this->ctrl_in_gpio_, this->ctrl_out_gpio_);
    return;
  }

  // Configure pins via ESP-IDF
  gpio_num_t clk = (gpio_num_t) this->clk_gpio_;
  gpio_num_t data = (gpio_num_t) this->data_gpio_;
  gpio_num_t ctrl_in = (gpio_num_t) this->ctrl_in_gpio_;
  gpio_num_t ctrl_out = (gpio_num_t) this->ctrl_out_gpio_;

  gpio_reset_pin(clk);
  gpio_set_direction(clk, GPIO_MODE_INPUT);
  gpio_set_intr_type(clk, GPIO_INTR_ANYEDGE);

  gpio_reset_pin(data);
  gpio_set_direction(data, GPIO_MODE_INPUT);

  gpio_reset_pin(ctrl_in);
  gpio_set_direction(ctrl_in, GPIO_MODE_INPUT);

  // ctrl_out starts high-Z (safe resistor-only injection)
  gpio_reset_pin(ctrl_out);
  gpio_set_direction(ctrl_out, GPIO_MODE_INPUT);

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

  ESP_LOGI(TAG, "ISR attached. clk=%d data=%d ctrl_in=%d ctrl_out=%d",
           this->clk_gpio_, this->data_gpio_, this->ctrl_in_gpio_, this->ctrl_out_gpio_);
}

void IRAM_ATTR Balboa9800CP::isr_router_() {
  if (Balboa9800CP::instance_ != nullptr) {
    Balboa9800CP::instance_->on_clock_edge_();
  }
}

void IRAM_ATTR Balboa9800CP::on_clock_edge_() {
  this->isr_edge_count_++;

  const uint32_t now = micros();
  const uint32_t dt = now - this->last_edge_us_;
  this->last_edge_us_ = now;

  // Frame boundary detection
  if (dt > this->gap_us_) this->bit_index_ = 0;

  const int i = this->bit_index_;
  if (i < 0 || i >= 76) return;

  // Capture DATA bit (ISR-safe)
  const uint8_t data_bit = (uint8_t) gpio_get_level((gpio_num_t) this->data_gpio_);
  this->bits_[i] = data_bit ? 1 : 0;

  // Default CTRL pass-through
  uint8_t out = (uint8_t) gpio_get_level((gpio_num_t) this->ctrl_in_gpio_);

  // Injection (same codes as your previous implementation)
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
      pattern = 0b0000;
    }

    const int bitpos = i - 72;              // 0..3
    out = (pattern >> (3 - bitpos)) & 0x1;  // MSB first
  }

  // Open-drain/high-Z behavior on ctrl_out
  gpio_num_t ctrl_out = (gpio_num_t) this->ctrl_out_gpio_;
  if (out == 0) {
    gpio_set_direction(ctrl_out, GPIO_MODE_OUTPUT);
    gpio_set_level(ctrl_out, 0);
  } else {
    gpio_set_direction(ctrl_out, GPIO_MODE_INPUT);
  }

  this->bit_index_++;

  if (this->bit_index_ >= 76) {
    portENTER_CRITICAL_ISR(&balboa_mux);
    this->frame_ready_ = true;
    portEXIT_CRITICAL_ISR(&balboa_mux);

    // Press bookkeeping
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

void Balboa9800CP::loop() {
  // 1 Hz: show interrupt activity
  static uint32_t last_ms = 0;
  const uint32_t now = millis();
  if (now - last_ms >= 1000) {
    last_ms = now;
    const uint32_t edges = this->isr_edge_count_;
    this->isr_edge_count_ = 0;
    ESP_LOGD(TAG, "clk edges/sec=%u bit_index=%d", (unsigned) edges, this->bit_index_);
  }

  // Consume completed frames and print RAW bits
  bool ready = false;
  portENTER_CRITICAL(&balboa_mux);
  ready = this->frame_ready_;
  if (ready) this->frame_ready_ = false;
  portEXIT_CRITICAL(&balboa_mux);

  if (!ready) return;

  uint8_t local_bits[76];
  portENTER_CRITICAL(&balboa_mux);
  for (int i = 0; i < 76; i++) local_bits[i] = this->bits_[i];
  portEXIT_CRITICAL(&balboa_mux);

  char raw[77];
  for (int i = 0; i < 76; i++) raw[i] = local_bits[i] ? '1' : '0';
  raw[76] = '\0';

  ESP_LOGD(TAG, "RAW frame: %s", raw);
}

// Not used in Step-1 RAW logger build, kept for later
void Balboa9800CP::process_frame_() {}

}  // namespace balboa_9800cp
}  // namespace esphome

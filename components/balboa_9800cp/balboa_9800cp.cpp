#include "balboa_9800cp.h"
#include "esphome/core/log.h"

#include <Arduino.h>
#include "driver/gpio.h"

namespace esphome {
namespace balboa_9800cp {

static const char *const TAG = "balboa_9800cp";

// ---- static instance pointer (fixes your undefined reference) ----
Balboa9800CP *Balboa9800CP::instance_ = nullptr;

// ---- ISR safety helpers ----
static portMUX_TYPE balboa_mux = portMUX_INITIALIZER_UNLOCKED;

void BalboaButton::press_action() {
  if (this->parent_ != nullptr) {
    this->parent_->queue_command(this->cmd_);
  }
}

void Balboa9800CP::dump_config() {
  ESP_LOGCONFIG(TAG, "Balboa 9800CP:");
  ESP_LOGCONFIG(TAG, "  clk_gpio: %d", this->clk_gpio_);
  ESP_LOGCONFIG(TAG, "  ctrl_out_gpio: %d", this->ctrl_out_gpio_);
  ESP_LOGCONFIG(TAG, "  gap_us: %u", (unsigned) this->gap_us_);
  ESP_LOGCONFIG(TAG, "  press_frames: %u", (unsigned) this->press_frames_);
}

void Balboa9800CP::set_pins(GPIOPin *clk, GPIOPin *data, GPIOPin *ctrl_in, GPIOPin *ctrl_out) {
  this->clk_ = clk;
  this->data_ = data;
  this->ctrl_in_ = ctrl_in;
  this->ctrl_out_ = ctrl_out;
}

void Balboa9800CP::setup() {
  ESP_LOGW(TAG, "setup() reached");
  instance_ = this;

  // Normal ESPHome pin setup (OK outside ISR)
  if (this->clk_) this->clk_->setup();
  if (this->data_) this->data_->setup();
  if (this->ctrl_in_) this->ctrl_in_->setup();
  if (this->ctrl_out_) this->ctrl_out_->setup();

  if (this->clk_gpio_ < 0 || this->ctrl_out_gpio_ < 0) {
    ESP_LOGE(TAG,
             "GPIO numbers not set (clk_gpio=%d ctrl_out_gpio=%d). Check __init__.py set_gpio_numbers().",
             this->clk_gpio_, this->ctrl_out_gpio_);
    return;
  }

  // --- Configure CLK interrupt using ESP-IDF (more reliable than attachInterrupt in some ESPHome builds) ---
  gpio_num_t clk_gpio = (gpio_num_t) this->clk_gpio_;
  gpio_reset_pin(clk_gpio);
  gpio_set_direction(clk_gpio, GPIO_MODE_INPUT);
  gpio_set_intr_type(clk_gpio, GPIO_INTR_POSEDGE);  // try GPIO_INTR_ANYEDGE if needed

  // ctrl_out starts HIGH-Z (INPUT) for safe resistor-only injection
  gpio_num_t ctrl_out_gpio = (gpio_num_t) this->ctrl_out_gpio_;
  gpio_reset_pin(ctrl_out_gpio);
  gpio_set_direction(ctrl_out_gpio, GPIO_MODE_INPUT);

  // Install ISR service once
  static bool isr_service_installed = false;
  if (!isr_service_installed) {
    esp_err_t e = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "gpio_install_isr_service failed: %d", (int) e);
    } else {
      isr_service_installed = true;
    }
  }

  esp_err_t e2 = gpio_isr_handler_add(clk_gpio, (gpio_isr_t) &Balboa9800CP::isr_router_, nullptr);
  if (e2 != ESP_OK) {
    ESP_LOGE(TAG, "gpio_isr_handler_add failed: %d", (int) e2);
    return;
  }

  ESP_LOGI(TAG,
           "ISR attached. gap_us=%u press_frames=%u clk_gpio=%d data_pin=%d ctrl_in_pin=%d ctrl_out_gpio=%d",
           (unsigned) this->gap_us_, (unsigned) this->press_frames_,
           this->clk_gpio_,
           this->data_ ? this->data_->get_pin() : -1,
           this->ctrl_in_ ? this->ctrl_in_->get_pin() : -1,
           this->ctrl_out_gpio_);
}

void IRAM_ATTR Balboa9800CP::isr_router_() {
  if (Balboa9800CP::instance_ != nullptr) {
    Balboa9800CP::instance_->on_clock_edge_();
  }
}

void IRAM_ATTR Balboa9800CP::on_clock_edge_() {
  // Step 1 proof-of-life: count edges
  this->isr_edge_count_++;

  const uint32_t now = micros();
  const uint32_t dt = now - this->last_edge_us_;
  this->last_edge_us_ = now;

  // Frame boundary
  if (dt > this->gap_us_) {
    this->bit_index_ = 0;
  }

  int i = this->bit_index_;
  if (i < 0 || i >= 76) return;

  // IMPORTANT: use gpio_get_level() inside ISR (avoid GPIOPin calls in ISR)
  int data_gpio = this->data_ ? this->data_->get_pin() : -1;
  int ctrl_in_gpio = this->ctrl_in_ ? this->ctrl_in_->get_pin() : -1;

  uint8_t data_bit = 0;
  if (data_gpio >= 0) data_bit = (uint8_t) gpio_get_level((gpio_num_t) data_gpio);

  // capture bit
  this->bits_[i] = data_bit ? 1 : 0;

  // pass-through ctrl default
  uint8_t out = 1;
  if (ctrl_in_gpio >= 0) out = (uint8_t) gpio_get_level((gpio_num_t) ctrl_in_gpio);

  // Injection (same behavior as your original)
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

    int bitpos = i - 72;                    // 0..3
    out = (pattern >> (3 - bitpos)) & 0x1;  // MSB first
  }

  // SAFE open-drain/high-Z behavior:
  gpio_num_t ctrl_out_gpio = (gpio_num_t) this->ctrl_out_gpio_;
  if (out == 0) {
    gpio_set_direction(ctrl_out_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(ctrl_out_gpio, 0);
  } else {
    gpio_set_direction(ctrl_out_gpio, GPIO_MODE_INPUT);  // high-Z
  }

  this->bit_index_++;

  if (this->bit_index_ >= 76) {
    // mark frame ready
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
  // Step 1: prove interrupts are happening (once per second)
  static uint32_t last_log_ms = 0;
  uint32_t now = millis();
  if (now - last_log_ms >= 1000) {
    last_log_ms = now;
    uint32_t edges = this->isr_edge_count_;
    this->isr_edge_count_ = 0;
    ESP_LOGD(TAG, "clk edges/sec=%u", (unsigned) edges);
  }

  // RAW frame logging
  bool ready = false;
  portENTER_CRITICAL(&balboa_mux);
  ready = this->frame_ready_;
  if (ready) this->frame_ready_ = false;
  portEXIT_CRITICAL(&balboa_mux);

  if (!ready) return;

  // copy bits locally (avoid races)
  uint8_t local_bits[76];
  portENTER_CRITICAL(&balboa_mux);
  for (int i = 0; i < 76; i++) local_bits[i] = this->bits_[i];
  portEXIT_CRITICAL(&balboa_mux);

  // Print RAW bits as 76-char string
  char raw[77];
  for (int i = 0; i < 76; i++) raw[i] = local_bits[i] ? '1' : '0';
  raw[76] = '\0';

  ESP_LOGD(TAG, "RAW(%u us gap): %s", (unsigned) this->gap_us_, raw);

  // (Decoder can be re-enabled later once RAW proves we have frames)
}

}  // namespace balboa_9800cp
}  // namespace esphome

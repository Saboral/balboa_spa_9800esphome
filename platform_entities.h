// ---- Switch platform ----
class BalboaToggleSwitch : public switch_::Switch {
 public:
  void set_parent(Balboa9800CPComponent *p) { parent_ = p; }
  void set_kind(uint8_t k) { kind_ = k; }  // 1=blower,2=pump1,3=pump2

 protected:
  void write_state(bool state) override {
    if (parent_ == nullptr) return;
    if (kind_ == 1) parent_->request_blower_state(state);
    else if (kind_ == 2) parent_->request_pump1_state(state);
    else if (kind_ == 3) parent_->request_pump2_state(state);
    // Do not optimistically publish; parent will publish from DisplayData.
  }

  Balboa9800CPComponent *parent_{nullptr};
  uint8_t kind_{0};
};

// ---- Button platform ----
class BalboaTempButton : public button::Button {
 public:
  void set_parent(Balboa9800CPComponent *p) { parent_ = p; }
  void set_direction(uint8_t d) { dir_ = d; }  // 1=up,2=down

 protected:
  void press_action() override {
    if (parent_ == nullptr) return;
    if (dir_ == 1) parent_->press_temp_up();
    else if (dir_ == 2) parent_->press_temp_down();
  }

  Balboa9800CPComponent *parent_{nullptr};
  uint8_t dir_{0};
};

// ---- Number platform ----
class BalboaTargetTempNumber : public number::Number {
 public:
  void set_parent(Balboa9800CPComponent *p) { parent_ = p; }

 protected:
  void control(float value) override {
    if (parent_ == nullptr) return;
    parent_->set_target_temperature(value);
    // Do not publish immediately; parent syncs from DisplayData setTemperature.
  }

  Balboa9800CPComponent *parent_{nullptr};
};

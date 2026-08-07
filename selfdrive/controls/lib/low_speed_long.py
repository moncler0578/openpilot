LOW_SPEED_LONG_MIN_SPEED = 2.0 / 3.6   # 2 km/h
STOCK_SCC_LEADLESS_MIN_SPEED_KPH = 30.0
LOW_SPEED_LONG_MAX_SPEED = STOCK_SCC_LEADLESS_MIN_SPEED_KPH / 3.6
LOW_SPEED_LONG_REQUEST_TIME = 1.0
CRUISE_SPEED_MIN_DEFAULT = 30
CRUISE_SPEED_MIN_LOWER = 5
CRUISE_SPEED_MIN_UPPER = 30
AUTO_RESUME_REQUEST_TIME = 1.0
AUTO_RESUME_MAX_STEERING_ANGLE_DEG = 20.0
AUTO_RESUME_STRONG_GAS = 0.6
# carrot-wip uses a 0.4 second gas-tap window in cruise.py.
AUTO_RESUME_QUICK_GAS_TIME = 0.4
AUTO_RESUME_MAX_SPEED_KPH = 161.0


def read_cruise_speed_min(params):
  raw = params.get("CruiseSpeedMin", encoding="utf8")
  try:
    value = int(raw) if raw else CRUISE_SPEED_MIN_DEFAULT
  except (TypeError, ValueError):
    value = CRUISE_SPEED_MIN_DEFAULT
  return max(CRUISE_SPEED_MIN_LOWER, min(CRUISE_SPEED_MIN_UPPER, value))


def suppress_low_speed_scc_alerts(values, force_long):
  """Clear transient stock SCC cluster prompts only during explicit engagement."""
  if force_long:
    values["SCCInfoDisplay"] = 0
    values["DriverAlertDisplay"] = 0
  return values


class LowSpeedLongEngage:
  """Temporarily allow SCC commands after an explicit low-speed SET/RES press."""

  def __init__(self):
    self.remaining = 0.0

  def reset(self):
    self.remaining = 0.0

  def update(self, available, request_pressed, brake_pressed, v_ego, has_lead, dt):
    if not available or brake_pressed:
      self.reset()
      return False

    low_speed_request = (LOW_SPEED_LONG_MIN_SPEED <= v_ego < LOW_SPEED_LONG_MAX_SPEED)
    if request_pressed and (has_lead or low_speed_request):
      self.remaining = LOW_SPEED_LONG_REQUEST_TIME
    else:
      self.remaining = max(0.0, self.remaining - dt)

    return self.remaining > 0.0


class AutoResumeController:
  """aPilot-style resume request generator adapted for Hyundai SCC control."""

  def __init__(self):
    self.remaining = 0.0
    self.target_speed_kph = 0.0
    self.prev_gas_pressed = False
    self.prev_brake_pressed = False
    self.gas_pressed_time = 0.0
    self.max_gas = 0.0

  def reset(self):
    self.remaining = 0.0
    self.target_speed_kph = 0.0

  @staticmethod
  def _target_speed(current_speed_kph, previous_speed_kph, cruise_speed_min,
                    speed_mode, has_lead, strong_gas):
    current = min(max(current_speed_kph, cruise_speed_min), AUTO_RESUME_MAX_SPEED_KPH)
    previous = min(max(previous_speed_kph, cruise_speed_min), AUTO_RESUME_MAX_SPEED_KPH)
    if strong_gas or speed_mode == 1 or (speed_mode == 2 and has_lead):
      return previous
    return current

  def update(self, available, cruise_enabled, gas_mode, gas_resume_speed_kph,
             gas_cancel_speed_kph,
             speed_mode, brake_release_enabled, brake_resume_speed_kph,
             brake_lead_distance, cruise_speed_min, gas_pressed, gas,
             brake_pressed, v_ego, steering_angle_deg, left_blinker,
             right_blinker, traffic_state, has_lead, lead_distance,
             previous_speed_kph, safety_guard, dt):
    gas_released = self.prev_gas_pressed and not gas_pressed
    brake_released = self.prev_brake_pressed and not brake_pressed
    previous_gas_time = self.gas_pressed_time
    previous_max_gas = self.max_gas

    if gas_pressed:
      self.gas_pressed_time += dt
      self.max_gas = max(self.max_gas, gas)

    v_ego_kph = v_ego * 3.6
    strong_gas = gas >= AUTO_RESUME_STRONG_GAS or previous_max_gas >= AUTO_RESUME_STRONG_GAS
    steering_ok = abs(steering_angle_deg) < AUTO_RESUME_MAX_STEERING_ANGLE_DEG or strong_gas
    red_signal = traffic_state % 10 == 1
    close_lead = has_lead and 0 < lead_distance < max(2.0, v_ego * 0.8)
    guarded_out = safety_guard and (left_blinker or right_blinker or close_lead)
    can_resume = (available and not cruise_enabled and previous_speed_kph > 0 and
                  not brake_pressed and steering_ok and not red_signal and not guarded_out)

    gas_trigger = False
    if can_resume and gas_mode > 0:
      gas_trigger = gas_pressed and (v_ego_kph >= gas_resume_speed_kph or strong_gas)
      quick_release = (gas_mode > 1 and gas_released and previous_max_gas > 0.03 and
                       previous_gas_time < AUTO_RESUME_QUICK_GAS_TIME and
                       v_ego_kph >= gas_cancel_speed_kph)
      gas_trigger = gas_trigger or quick_release

    brake_trigger = False
    if can_resume and brake_release_enabled and brake_released:
      lead_ok = has_lead and lead_distance >= brake_lead_distance
      speed_ok = v_ego_kph >= brake_resume_speed_kph
      brake_trigger = lead_ok or speed_ok

    if gas_trigger or brake_trigger:
      self.target_speed_kph = self._target_speed(
        v_ego_kph, previous_speed_kph, cruise_speed_min,
        speed_mode, has_lead, strong_gas)
      self.remaining = AUTO_RESUME_REQUEST_TIME
    elif not available or cruise_enabled or brake_pressed:
      self.reset()
    else:
      self.remaining = max(0.0, self.remaining - dt)

    self.prev_gas_pressed = gas_pressed
    self.prev_brake_pressed = brake_pressed
    if not gas_pressed:
      self.gas_pressed_time = 0.0
      self.max_gas = 0.0

    return self.remaining > 0.0, self.target_speed_kph

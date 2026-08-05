LOW_SPEED_LONG_MIN_SPEED = 2.0 / 3.6   # 2 km/h
LOW_SPEED_LONG_MAX_SPEED = 30.0 / 3.6  # stock leadless threshold
LOW_SPEED_LONG_REQUEST_TIME = 1.0
CRUISE_SPEED_MIN_DEFAULT = 30
CRUISE_SPEED_MIN_LOWER = 5
CRUISE_SPEED_MIN_UPPER = 30


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

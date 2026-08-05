import math


JERK_START_DEFAULT_RAW = 10
JERK_START_MIN_RAW = 5
JERK_START_MAX_RAW = 30
JERK_START_SCALE = 0.1
JERK_LIMIT = 5.0
JERK_HOLD_TIME = 1.5
JERK_RAMP_END_TIME = 2.5


def read_jerk_start_limit(params):
  raw = params.get("JerkStartLimit", encoding="utf8")
  try:
    value = int(raw) if raw else JERK_START_DEFAULT_RAW
  except (TypeError, ValueError):
    value = JERK_START_DEFAULT_RAW
  value = max(JERK_START_MIN_RAW, min(JERK_START_MAX_RAW, value))
  return value * JERK_START_SCALE


class LongitudinalJerkController:
  """Convert the planner jerk into bounded Hyundai SCC14 jerk limits."""

  def __init__(self, start_limit=JERK_START_DEFAULT_RAW * JERK_START_SCALE):
    self.start_limit = 0.0
    self.elapsed = 0.0
    self.set_start_limit(start_limit)

  def set_start_limit(self, start_limit):
    if not math.isfinite(start_limit):
      start_limit = JERK_START_DEFAULT_RAW * JERK_START_SCALE
    self.start_limit = max(JERK_START_MIN_RAW * JERK_START_SCALE,
                           min(JERK_START_MAX_RAW * JERK_START_SCALE, start_limit))

  def reset(self):
    self.elapsed = 0.0

  def _current_limit(self):
    if self.elapsed <= JERK_HOLD_TIME + 1e-9:
      return self.start_limit
    if self.elapsed >= JERK_RAMP_END_TIME:
      return JERK_LIMIT
    ratio = (self.elapsed - JERK_HOLD_TIME) / (JERK_RAMP_END_TIME - JERK_HOLD_TIME)
    return self.start_limit + ratio * (JERK_LIMIT - self.start_limit)

  def update(self, active, stopping, soft_hold, planned_jerk, dt):
    if not active:
      self.reset()
      return JERK_LIMIT, JERK_LIMIT

    if stopping or soft_hold:
      self.reset()
      # Preserve the branch's proven stop/hold behavior.
      return 0.5, 10.0

    elapsed_step = dt if math.isfinite(dt) else 0.0
    self.elapsed += max(0.0, elapsed_step)
    jerk_limit = self._current_limit()
    jerk = planned_jerk if math.isfinite(planned_jerk) else 0.0

    upper = min(max(0.5, jerk * 2.0), jerk_limit)
    lower = min(max(1.0, -jerk * 2.0), jerk_limit)
    return upper, lower

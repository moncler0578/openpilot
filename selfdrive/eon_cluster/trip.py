class TripTracker(object):
  def __init__(self):
    self.started = False
    self.last_t = None
    self.duration_s = 0.0
    self.moving_time_s = 0.0
    self.distance_m = 0.0
    self.max_speed_kph = 0.0
    self.engaged_time_s = 0.0
    self.max_accel = 0.0
    self.max_decel = 0.0
    self.hard_accel_count = 0
    self.hard_brake_count = 0
    self._hard_accel_active = False
    self._hard_brake_active = False

  def update(self, started, speed_kph, now, enabled=False, accel=0.0):
    if started and not self.started:
      self.duration_s = 0.0
      self.moving_time_s = 0.0
      self.distance_m = 0.0
      self.max_speed_kph = 0.0
      self.engaged_time_s = 0.0
      self.max_accel = 0.0
      self.max_decel = 0.0
      self.hard_accel_count = 0
      self.hard_brake_count = 0
      self._hard_accel_active = False
      self._hard_brake_active = False
      self.last_t = now
    if started and self.last_t is not None:
      dt = max(0.0, min(1.0, now - self.last_t))
      speed_kph = max(0.0, float(speed_kph))
      self.duration_s += dt
      self.distance_m += speed_kph / 3.6 * dt
      if speed_kph > 1.0:
        self.moving_time_s += dt
      self.max_speed_kph = max(self.max_speed_kph, speed_kph)
      if enabled:
        self.engaged_time_s += dt
      try:
        accel = float(accel)
      except (TypeError, ValueError):
        accel = 0.0
      self.max_accel = max(self.max_accel, accel)
      self.max_decel = min(self.max_decel, accel)
      hard_accel = speed_kph > 5.0 and accel >= 2.0
      hard_brake = speed_kph > 5.0 and accel <= -2.5
      if hard_accel and not self._hard_accel_active:
        self.hard_accel_count += 1
      if hard_brake and not self._hard_brake_active:
        self.hard_brake_count += 1
      self._hard_accel_active = hard_accel
      self._hard_brake_active = hard_brake
    self.started = bool(started)
    self.last_t = now if started else None

  def snapshot(self):
    average = self.distance_m / self.moving_time_s * 3.6 if self.moving_time_s > 0.0 else 0.0
    return {
      "duration_s": self.duration_s,
      "distance_m": self.distance_m,
      "average_speed_kph": average,
      "max_speed_kph": self.max_speed_kph,
      "engaged_time_s": self.engaged_time_s,
      "max_accel": self.max_accel,
      "max_decel": self.max_decel,
      "hard_accel_count": self.hard_accel_count,
      "hard_brake_count": self.hard_brake_count,
    }

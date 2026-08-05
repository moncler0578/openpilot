class TripTracker(object):
  def __init__(self):
    self.started = False
    self.last_t = None
    self.duration_s = 0.0
    self.moving_time_s = 0.0
    self.distance_m = 0.0
    self.max_speed_kph = 0.0

  def update(self, started, speed_kph, now):
    if started and not self.started:
      self.duration_s = 0.0
      self.moving_time_s = 0.0
      self.distance_m = 0.0
      self.max_speed_kph = 0.0
      self.last_t = now
    if started and self.last_t is not None:
      dt = max(0.0, min(1.0, now - self.last_t))
      speed_kph = max(0.0, float(speed_kph))
      self.duration_s += dt
      self.distance_m += speed_kph / 3.6 * dt
      if speed_kph > 1.0:
        self.moving_time_s += dt
      self.max_speed_kph = max(self.max_speed_kph, speed_kph)
    self.started = bool(started)
    self.last_t = now if started else None

  def snapshot(self):
    average = self.distance_m / self.moving_time_s * 3.6 if self.moving_time_s > 0.0 else 0.0
    return {
      "duration_s": self.duration_s,
      "distance_m": self.distance_m,
      "average_speed_kph": average,
      "max_speed_kph": self.max_speed_kph,
    }

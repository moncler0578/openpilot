SOFT_HOLD_ENTRY_SPEED = 0.1  # m/s
SOFT_HOLD_ENTRY_TIME = 0.7   # seconds


class SoftHoldController:
  """Latch a driver-requested standstill until gas or RES/+ is pressed."""

  def __init__(self):
    self.active = False
    self.entry_time = 0.0

  def reset(self):
    self.active = False
    self.entry_time = 0.0

  def update(self, available, brake_pressed, gas_pressed, v_ego, resume_pressed, dt):
    if not available:
      self.reset()
      return False

    if self.active:
      self.entry_time = 0.0
      if gas_pressed or resume_pressed:
        self.active = False
      return self.active

    if gas_pressed or resume_pressed:
      self.entry_time = 0.0
      return False

    if brake_pressed and v_ego < SOFT_HOLD_ENTRY_SPEED:
      self.entry_time += dt
      if self.entry_time + 1e-6 >= SOFT_HOLD_ENTRY_TIME:
        self.active = True
        self.entry_time = 0.0
    else:
      self.entry_time = 0.0

    return self.active

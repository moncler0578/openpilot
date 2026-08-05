from selfdrive.controls.lib.longitudinal_jerk import JERK_HOLD_TIME, JERK_RAMP_END_TIME, \
  LongitudinalJerkController, read_jerk_start_limit


DT_CTRL = 0.01


class FakeParams:
  def __init__(self, value):
    self.value = value

  def get(self, key, encoding=None):
    assert key == "JerkStartLimit"
    return self.value


def test_jerk_start_limit_default_and_clamp():
  assert read_jerk_start_limit(FakeParams(None)) == 1.0
  assert read_jerk_start_limit(FakeParams("1")) == 0.5
  assert read_jerk_start_limit(FakeParams("20")) == 2.0
  assert read_jerk_start_limit(FakeParams("50")) == 3.0


def test_start_limit_is_held_for_first_1_5_seconds():
  controller = LongitudinalJerkController(1.0)
  for _ in range(int(JERK_HOLD_TIME / DT_CTRL)):
    upper, lower = controller.update(True, False, False, 3.0, DT_CTRL)
  assert upper == 1.0
  assert lower == 1.0


def test_limit_ramps_to_five_after_2_5_seconds():
  controller = LongitudinalJerkController(1.0)
  for _ in range(int(JERK_RAMP_END_TIME / DT_CTRL) + 1):
    upper, lower = controller.update(True, False, False, 3.0, DT_CTRL)
  assert upper == 5.0
  assert lower == 1.0


def test_negative_planner_jerk_controls_deceleration_limit():
  controller = LongitudinalJerkController(3.0)
  for _ in range(int(JERK_RAMP_END_TIME / DT_CTRL) + 1):
    upper, lower = controller.update(True, False, False, -2.0, DT_CTRL)
  assert upper == 0.5
  assert lower == 4.0


def test_stop_and_soft_hold_preserve_braking_limits_and_reset_ramp():
  controller = LongitudinalJerkController(1.0)
  controller.update(True, False, False, 3.0, 2.0)
  assert controller.update(True, True, False, -3.0, DT_CTRL) == (0.5, 10.0)
  assert controller.update(True, False, False, 3.0, DT_CTRL) == (1.0, 1.0)
  assert controller.update(True, False, True, -3.0, DT_CTRL) == (0.5, 10.0)


def test_inactive_and_non_finite_values_are_safe():
  controller = LongitudinalJerkController(float("nan"))
  assert controller.update(False, False, False, float("nan"), DT_CTRL) == (5.0, 5.0)
  assert controller.update(True, False, False, float("nan"), DT_CTRL) == (0.5, 1.0)

from selfdrive.controls.lib.low_speed_long import AUTO_RESUME_REQUEST_TIME, LOW_SPEED_LONG_REQUEST_TIME, \
  AutoResumeController, LowSpeedLongEngage, read_cruise_speed_min, suppress_low_speed_scc_alerts


DT_CTRL = 0.01


def update_auto_resume(controller, **overrides):
  args = dict(
    available=True,
    cruise_enabled=False,
    gas_mode=1,
    gas_resume_speed_kph=30,
    gas_cancel_speed_kph=30,
    speed_mode=0,
    brake_release_enabled=False,
    brake_resume_speed_kph=30,
    brake_lead_distance=10,
    cruise_speed_min=5,
    gas_pressed=False,
    gas=0.0,
    brake_pressed=False,
    v_ego=10 / 3.6,
    steering_angle_deg=0.0,
    left_blinker=False,
    right_blinker=False,
    traffic_state=0,
    has_lead=False,
    lead_distance=0.0,
    previous_speed_kph=30.0,
    safety_guard=True,
    dt=DT_CTRL,
  )
  args.update(overrides)
  return controller.update(**args)


class FakeParams:
  def __init__(self, value):
    self.value = value

  def get(self, key, encoding=None):
    assert key == "CruiseSpeedMin"
    return self.value


def test_cruise_speed_min_defaults_to_30_kph():
  assert read_cruise_speed_min(FakeParams(None)) == 30


def test_cruise_speed_min_accepts_configured_value():
  assert read_cruise_speed_min(FakeParams("12")) == 12


def test_cruise_speed_min_is_safely_clamped():
  assert read_cruise_speed_min(FakeParams("1")) == 5
  assert read_cruise_speed_min(FakeParams("50")) == 30


def test_forced_low_speed_request_clears_transient_cluster_prompts():
  values = {"SCCInfoDisplay": 3, "DriverAlertDisplay": 2, "ACCFailInfo": 1}
  suppress_low_speed_scc_alerts(values, True)
  assert values == {"SCCInfoDisplay": 0, "DriverAlertDisplay": 0, "ACCFailInfo": 1}


def test_normal_scc_status_is_preserved_outside_request_window():
  values = {"SCCInfoDisplay": 4, "DriverAlertDisplay": 1}
  suppress_low_speed_scc_alerts(values, False)
  assert values == {"SCCInfoDisplay": 4, "DriverAlertDisplay": 1}


def test_driver_set_allows_leadless_low_speed_request():
  engage = LowSpeedLongEngage()
  assert engage.update(True, True, False, 10.0 / 3.6, False, DT_CTRL)


def test_low_speed_without_button_does_not_request():
  engage = LowSpeedLongEngage()
  assert not engage.update(True, False, False, 10.0 / 3.6, False, DT_CTRL)


def test_request_stays_active_long_enough_for_scc_commands():
  engage = LowSpeedLongEngage()
  engage.update(True, True, False, 10.0 / 3.6, False, DT_CTRL)
  for _ in range(int(LOW_SPEED_LONG_REQUEST_TIME / DT_CTRL) - 1):
    assert engage.update(True, False, False, 10.0 / 3.6, False, DT_CTRL)
  assert not engage.update(True, False, False, 10.0 / 3.6, False, DT_CTRL)


def test_leadless_standstill_request_is_blocked():
  engage = LowSpeedLongEngage()
  assert not engage.update(True, True, False, 0.0, False, DT_CTRL)


def test_leadless_creep_below_two_kph_is_blocked():
  engage = LowSpeedLongEngage()
  assert not engage.update(True, True, False, 1.0 / 3.6, False, DT_CTRL)


def test_lead_allows_standstill_request():
  engage = LowSpeedLongEngage()
  assert engage.update(True, True, False, 0.0, True, DT_CTRL)


def test_brake_cancels_pending_request():
  engage = LowSpeedLongEngage()
  assert engage.update(True, True, False, 10.0 / 3.6, False, DT_CTRL)
  assert not engage.update(True, False, True, 10.0 / 3.6, False, DT_CTRL)


def test_main_or_long_control_off_cancels_request():
  engage = LowSpeedLongEngage()
  assert engage.update(True, True, False, 10.0 / 3.6, False, DT_CTRL)
  assert not engage.update(False, False, False, 10.0 / 3.6, False, DT_CTRL)


def test_high_speed_path_is_left_to_stock_scc():
  engage = LowSpeedLongEngage()
  assert not engage.update(True, True, False, 35.0 / 3.6, False, DT_CTRL)


def test_gas_auto_resume_uses_configured_minimum_speed():
  resume = AutoResumeController()
  requested, target = update_auto_resume(
    resume, gas_pressed=True, gas=0.2, gas_resume_speed_kph=5,
    cruise_speed_min=12, v_ego=8 / 3.6)
  assert requested
  assert target == 12


def test_gas_auto_resume_can_restore_previous_speed():
  resume = AutoResumeController()
  requested, target = update_auto_resume(
    resume, gas_pressed=True, gas=0.2, gas_resume_speed_kph=5,
    speed_mode=1, previous_speed_kph=42)
  assert requested
  assert target == 42


def test_auto_resume_rejects_invalid_previous_speed():
  resume = AutoResumeController()
  requested, target = update_auto_resume(
    resume, gas_pressed=True, gas=0.7, previous_speed_kph=255)
  assert requested
  assert target == 161


def test_strong_gas_can_resume_above_steering_limit_and_restores_previous_speed():
  resume = AutoResumeController()
  requested, target = update_auto_resume(
    resume, gas_pressed=True, gas=0.7, steering_angle_deg=30,
    previous_speed_kph=40)
  assert requested
  assert target == 40


def test_red_signal_blocks_gas_auto_resume():
  resume = AutoResumeController()
  requested, _ = update_auto_resume(
    resume, gas_pressed=True, gas=0.7, traffic_state=1)
  assert not requested


def test_blinker_and_close_lead_guard_block_auto_resume():
  resume = AutoResumeController()
  requested, _ = update_auto_resume(
    resume, gas_pressed=True, gas=0.7, left_blinker=True)
  assert not requested
  requested, _ = update_auto_resume(
    resume, gas_pressed=True, gas=0.7, has_lead=True,
    lead_distance=2.5, v_ego=50 / 3.6)
  assert not requested


def test_brake_release_auto_resume_requires_configured_lead_distance():
  resume = AutoResumeController()
  update_auto_resume(resume, gas_mode=0, brake_release_enabled=True,
                     brake_pressed=True, has_lead=True, lead_distance=12)
  requested, target = update_auto_resume(
    resume, gas_mode=0, brake_release_enabled=True,
    brake_pressed=False, has_lead=True, lead_distance=12,
    v_ego=0.0, cruise_speed_min=10)
  assert requested
  assert target == 10


def test_auto_resume_request_expires():
  resume = AutoResumeController()
  requested, _ = update_auto_resume(
    resume, gas_pressed=True, gas=0.2, gas_resume_speed_kph=5)
  assert requested
  for _ in range(int(AUTO_RESUME_REQUEST_TIME / DT_CTRL)):
    requested, _ = update_auto_resume(resume, gas_mode=0)
  assert not requested


def test_quick_gas_release_uses_carrot_point_four_second_window():
  resume = AutoResumeController()
  for _ in range(39):
    update_auto_resume(resume, gas_mode=2, gas_pressed=True, gas=0.2,
                       v_ego=35 / 3.6)
  requested, _ = update_auto_resume(resume, gas_mode=2, gas_pressed=False,
                                    v_ego=35 / 3.6)
  assert requested


def test_quick_gas_release_below_cancel_speed_is_blocked():
  resume = AutoResumeController()
  update_auto_resume(resume, gas_mode=2, gas_pressed=True, gas=0.2,
                     gas_cancel_speed_kph=30, v_ego=20 / 3.6)
  requested, _ = update_auto_resume(resume, gas_mode=2, gas_pressed=False,
                                    gas_cancel_speed_kph=30, v_ego=20 / 3.6)
  assert not requested

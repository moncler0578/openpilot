from selfdrive.controls.lib.low_speed_long import LOW_SPEED_LONG_REQUEST_TIME, LowSpeedLongEngage, \
  read_cruise_speed_min, suppress_low_speed_scc_alerts


DT_CTRL = 0.01


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

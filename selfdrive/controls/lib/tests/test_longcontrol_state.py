from types import SimpleNamespace

from selfdrive.controls.lib.longcontrol import LongCtrlState, long_control_state_trans


def make_cp(openpilot_longitudinal=True):
  return SimpleNamespace(
    enableGasInterceptor=False,
    openpilotLongitudinalControl=openpilot_longitudinal,
    vEgoStopping=0.3,
    vEgoStarting=0.2,
  )


def transition(cp, state=LongCtrlState.stopping, v_target=0.0,
               v_target_1sec=0.0, brake_pressed=False, cruise_standstill=False,
               soft_hold=False, starting_state=False):
  next_state, _ = long_control_state_trans(
    cp, True, state, 0.0, v_target, v_target_1sec,
    brake_pressed, cruise_standstill, soft_hold, 0.0, starting_state,
  )
  return next_state


def test_small_planner_fluctuation_stays_stopped():
  assert transition(make_cp(), v_target=0.0, v_target_1sec=0.19) == LongCtrlState.stopping


def test_apilot_c2_launches_when_carstate_suppresses_stock_standstill():
  cp = make_cp()
  state = transition(cp, v_target=0.0, v_target_1sec=0.21)
  assert state == LongCtrlState.pid
  assert transition(cp, state=state, v_target=0.0, v_target_1sec=0.21) == LongCtrlState.pid


def test_start_accel_setting_uses_starting_state():
  cp = make_cp()
  state = transition(cp, v_target=0.0, v_target_1sec=0.21, starting_state=True)
  assert state == LongCtrlState.starting
  assert transition(cp, state=state, v_target=0.0, v_target_1sec=0.21,
                    starting_state=True) == LongCtrlState.starting


def test_stock_acc_still_waits_for_stock_standstill_to_clear():
  assert transition(make_cp(False), v_target=0.0, v_target_1sec=0.21,
                    cruise_standstill=True) == LongCtrlState.stopping


def test_brake_prevents_stale_standstill_override():
  assert transition(make_cp(), v_target=0.0, v_target_1sec=0.21,
                    brake_pressed=True) == LongCtrlState.stopping


def test_soft_hold_still_overrides_planner_launch():
  assert transition(make_cp(), v_target=0.0, v_target_1sec=0.21,
                    soft_hold=True) == LongCtrlState.stopping

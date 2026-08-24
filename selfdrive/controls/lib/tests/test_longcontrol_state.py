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


def latched_transition(cp, standstill_latched=True, standstill_release=False, **kwargs):
  defaults = dict(state=LongCtrlState.stopping, v_target=0.0, v_target_1sec=0.0,
                  brake_pressed=False, cruise_standstill=False, soft_hold=False,
                  starting_state=False)
  defaults.update(kwargs)
  next_state, _ = long_control_state_trans(
    cp, True, defaults["state"], 0.0, defaults["v_target"], defaults["v_target_1sec"],
    defaults["brake_pressed"], defaults["cruise_standstill"], defaults["soft_hold"],
    0.0, defaults["starting_state"], standstill_latched, standstill_release)
  return next_state


def test_latched_standstill_ignores_a_planner_start_request():
  # 정차 래치 중에는 플래너가 출발을 요구해도 stopping 을 유지한다.
  assert latched_transition(make_cp(), v_target_1sec=1.0) == LongCtrlState.stopping


def test_confirmed_release_leaves_stopping():
  cp = make_cp()
  assert latched_transition(cp, standstill_latched=False, standstill_release=True,
                            v_target_1sec=1.0) == LongCtrlState.pid
  assert latched_transition(cp, standstill_latched=False, standstill_release=True,
                            v_target_1sec=1.0, starting_state=True) == LongCtrlState.starting


def test_latch_does_not_block_pid_while_still_moving():
  # 아직 서지 않았으면 래치가 걸리지 않으므로 평소대로 동작한다.
  assert latched_transition(make_cp(), standstill_latched=False,
                            state=LongCtrlState.pid, v_target=5.0,
                            v_target_1sec=6.0) == LongCtrlState.pid

"""NOO 회전 상태기계(turnState) 테스트.

DesireHelper.__init__ 은 Params 를 읽으므로 __new__ 로 인스턴스만 만들고
회전 관련 필드만 채워서 _advance_noo_turn 을 직접 돌린다.
"""
from common.realtime import DT_MDL
from selfdrive.controls.lib.desire_helper import DesireHelper


def machine():
  helper = DesireHelper.__new__(DesireHelper)
  helper.turn_state = 0
  helper.turn_state_timer = 0.0
  helper.turn_direction_latched = 0
  helper.turn_ll_prob = 1.0
  return helper


def frames(seconds):
  return range(int(seconds / DT_MDL))


def test_turn_is_held_while_the_model_has_not_picked_it_up_yet():
  # 큰 교차로: 거리창은 1초 만에 닫히지만 실제 선회는 그 뒤에 시작된다.
  helper = machine()
  for _ in frames(1.0):
    assert helper._advance_noo_turn(1, 0.0) == 1
  for _ in frames(2.0):
    assert helper._advance_noo_turn(0, 0.0) == 1
  assert helper.turn_state == 1


def test_model_pickup_moves_to_state_two_and_probability_ends_the_turn():
  helper = machine()
  helper._advance_noo_turn(1, 0.0)
  assert helper._advance_noo_turn(0, 0.5) == 1
  assert helper.turn_state == 2
  for _ in frames(3.0):
    assert helper._advance_noo_turn(0, 0.6) == 1
  # 확률이 떨어지면 0.5 s 페이드 뒤 해제.
  assert helper._advance_noo_turn(0, 0.0) == 1
  for _ in frames(0.6):
    helper._advance_noo_turn(0, 0.0)
  assert helper.turn_state == 0
  assert helper.turn_direction_latched == 0


def test_state_one_times_out_when_the_model_never_turns():
  helper = machine()
  helper._advance_noo_turn(1, 0.0)
  for _ in frames(DesireHelper.NOO_TURN_ARM_TIMEOUT - 0.2):
    assert helper._advance_noo_turn(0, 0.0) == 1
  for _ in frames(0.5):
    helper._advance_noo_turn(0, 0.0)
  assert helper.turn_state == 0


def test_overall_time_limit_releases_a_stuck_turn():
  helper = machine()
  helper._advance_noo_turn(1, 0.0)
  helper._advance_noo_turn(0, 0.5)
  for _ in frames(DesireHelper.NOO_TURN_MAX_TIME + 0.5):
    helper._advance_noo_turn(0, 0.9)
  assert helper.turn_state == 0


def test_a_new_request_restarts_the_machine_after_release():
  helper = machine()
  helper._advance_noo_turn(1, 0.0)
  helper._reset_noo_turn()
  assert helper._advance_noo_turn(-1, 0.0) == -1
  assert helper.turn_state == 1
  assert helper.turn_direction_latched == -1

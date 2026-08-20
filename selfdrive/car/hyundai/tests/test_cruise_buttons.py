from selfdrive.car.hyundai.cruise_buttons import (button_pressed_in_samples, button_transitions,
                                                  collect_button_samples, main_button_transitions)
from selfdrive.car.hyundai.values import Buttons


def test_collects_all_can_samples():
  samples = collect_button_samples(Buttons.NONE, Buttons.NONE,
                                   [Buttons.RES_ACCEL, Buttons.NONE])
  assert samples == [Buttons.RES_ACCEL, Buttons.NONE]


def test_latest_value_fallback():
  assert collect_button_samples(Buttons.NONE, Buttons.SET_DECEL, []) == [Buttons.SET_DECEL]
  assert collect_button_samples(Buttons.SET_DECEL, Buttons.SET_DECEL, []) == []


def test_quick_tap_in_one_parser_update():
  transitions = button_transitions(Buttons.NONE, [Buttons.RES_ACCEL, Buttons.NONE])
  assert transitions == [(Buttons.RES_ACCEL, True), (Buttons.RES_ACCEL, False)]


def test_mid_press_change_releases_old_button_first():
  transitions = button_transitions(Buttons.RES_ACCEL, [Buttons.SET_DECEL])
  assert transitions == [(Buttons.RES_ACCEL, False), (Buttons.SET_DECEL, True)]


def test_repeated_can_values_do_not_duplicate_events():
  transitions = button_transitions(
    Buttons.NONE,
    [Buttons.NONE, Buttons.RES_ACCEL, Buttons.RES_ACCEL, Buttons.NONE, Buttons.NONE],
  )
  assert transitions == [(Buttons.RES_ACCEL, True), (Buttons.RES_ACCEL, False)]


def test_main_button_keeps_every_edge():
  assert main_button_transitions(False, [0, 1, 1, 0]) == [True, False]


def test_quick_physical_tap_blocks_synthetic_button_transport():
  assert button_pressed_in_samples(Buttons.NONE, [Buttons.RES_ACCEL, Buttons.NONE])
  assert not button_pressed_in_samples(Buttons.NONE, [Buttons.NONE])

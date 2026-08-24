"""MixRadarInfo 조건부 비전 가속도 혼합 테스트."""
from math import isclose

from selfdrive.controls.lib.radar_helpers import blend_radar_vision_accel


def blend(radar_accel, vision_accel, model_prob=0.9, mix_radar_info=True,
          track_frames=10, v_rel=-1.0):
  return blend_radar_vision_accel(radar_accel, vision_accel, model_prob,
                                  mix_radar_info, track_frames, v_rel)


def close(actual, expected):
  return isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-9)


def test_new_track_keeps_radar_for_stationary_lead():
  accel, mixed = blend(0.05, -2.5, track_frames=2, v_rel=-8.0)
  assert not mixed
  assert close(accel, 0.05)


def test_radar_lag_uses_bounded_vision_braking_after_track_is_stable():
  accel, mixed = blend(0.05, -2.5, track_frames=10, v_rel=-2.0)
  assert mixed
  assert close(accel, -0.15)


def test_stronger_vision_in_same_direction_is_weighted_not_replaced():
  brake_accel, brake_mixed = blend(-1.0, -3.0)
  launch_accel, launch_mixed = blend(0.5, 1.5, v_rel=1.0)
  assert brake_mixed and launch_mixed
  assert close(brake_accel, -1.2)
  assert close(launch_accel, 0.8)


def test_opposite_sign_is_rejected():
  accel, mixed = blend(-1.5, 2.0)
  assert not mixed and close(accel, -1.5)
  accel, mixed = blend(1.0, -2.5)
  assert not mixed and close(accel, 1.0)


def test_stronger_radar_keeps_the_radar_value():
  accel, mixed = blend(-3.0, -1.0)
  assert not mixed and close(accel, -3.0)


def test_undecided_radar_requires_relative_speed_confirmation():
  accel, mixed = blend(0.05, -2.5, v_rel=0.0)
  assert not mixed and close(accel, 0.05)

  accel, mixed = blend(0.05, 1.5, v_rel=0.5)
  assert mixed and close(accel, 0.35)


def test_low_model_probability_or_disabled_never_mixes():
  accel, mixed = blend(0.05, -2.5, model_prob=0.4)
  assert not mixed and close(accel, 0.05)
  accel, mixed = blend(0.05, -2.5, mix_radar_info=False)
  assert not mixed and close(accel, 0.05)

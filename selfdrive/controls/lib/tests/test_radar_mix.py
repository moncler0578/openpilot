"""MixRadarInfo 의 비전 가속도 혼합 조건 테스트.

radar_helpers 는 cereal 을 끌고 오므로, 판정식만 같은 규칙으로 검증한다.
실제 코드는 get_RadarState2() 안의 use_vision_mix 이며 상수는 공유한다.
"""
from selfdrive.controls.lib.radar_helpers import RADAR_ACCEL_UNDECIDED


def use_vision_mix(radar_accel, vision_accel, model_prob=0.9, mix_radar_info=True):
  same_direction = (radar_accel * vision_accel > 0.0
                    or abs(radar_accel) < RADAR_ACCEL_UNDECIDED)
  return (mix_radar_info and model_prob > 0.5
          and same_direction and abs(radar_accel) < abs(vision_accel))


def test_radar_lag_lets_vision_lead_the_braking():
  # SCC11 은 가속도 필드가 없어 aLeadK 가 늦게 따라온다. 이 구간이 이득.
  assert use_vision_mix(0.05, -2.5)


def test_stronger_vision_in_the_same_direction_is_used():
  assert use_vision_mix(-1.0, -3.0)
  assert use_vision_mix(0.5, 1.5)


def test_opposite_sign_is_rejected():
  # 레이더가 감속을 보고 있는데 비전이 한 프레임 튀어 양수가 되어도
  # 제동을 풀지 않는다.
  assert not use_vision_mix(-1.5, 2.0)
  assert not use_vision_mix(1.0, -2.5)


def test_stronger_radar_keeps_the_radar_value():
  assert not use_vision_mix(-3.0, -1.0)


def test_low_model_probability_or_disabled_never_mixes():
  assert not use_vision_mix(0.05, -2.5, model_prob=0.4)
  assert not use_vision_mix(0.05, -2.5, mix_radar_info=False)

"""조향 라이브 튜닝값 (carrot 방식 Params).

nTune common.json 을 대체한다. Params 파일 읽기는 비싸므로 키별 호출 횟수를
캐시하고 100Hz 제어 경로에서도 1초에 한 번만 갱신한다.
"""
from common.params import Params

_REFRESH_CALLS = 100

_params = Params()
_cache = {}
_counters = {}


def _get(key, default):
  count = _counters.get(key, 0)
  if count % _REFRESH_CALLS == 0 or key not in _cache:
    try:
      v = _params.get(key, encoding="utf8")
      _cache[key] = float(v) if v not in (None, "") else default
    except (TypeError, ValueError):
      _cache[key] = default
  _counters[key] = count + 1
  return _cache[key]


def steer_actuator_delay():
  """조향 반응 지연 보상(초)."""
  return _get("SteerActuatorDelay", 10.0) * 0.01


def use_live_steer_ratio():
  """liveParameters 학습 조향비 사용 여부."""
  return _get("UseLiveSteerRatio", 1.0) > 0.5


def custom_steer_ratio():
  """고정 조향비."""
  return _get("CustomSteerRatio", 1650.0) * 0.01


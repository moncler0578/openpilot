"""S9-only remote HUD wrapper.

Keeps the existing low-overhead remote_hud transport and exposes the S9 HUD
Params to the Android renderer for live tuning.
"""

import time

from common.params import Params
from selfdrive.eon_cluster import remote_hud as base


_params = Params()
_original_packet = base._packet
_param_cache = {}
PARAM_CACHE_S = 1.0


def _bounded_int(key, default, minimum, maximum):
  now = time.monotonic()
  cached = _param_cache.get(key)
  if cached is not None and now - cached[0] < PARAM_CACHE_S:
    value = cached[1]
  else:
    try:
      raw = _params.get(key)
      value = int(raw) if raw is not None else default
    except (TypeError, ValueError):
      value = default
    _param_cache[key] = (now, value)
  return max(minimum, min(maximum, value))


def _flip_points_y(points):
  if not points:
    return points
  return [[p[0], -p[1]] for p in points]


def _apply_path_flip(packet):
  # 2026-08-18: 실차 확인 결과 EonClusterHudMirror(화면 전체 좌우반전)는
  # 꺼져 있는데도 차선/경로 리본이 실제 도로와 반대로 보이는 문제가
  # 있었다. 텍스트/아이콘은 정상이라 전체 미러는 원인이 아니고, 차선과
  # 리본이 서로는 어긋나지 않고 같이 반대로 보인다는 것도 확인했다.
  # BSD/텍스트/아이콘은 건드리지 않고 path/lanes/edges/pathOffset/lead.y
  # 만 좌우 부호를 뒤집는, APK 재설치 없이(파이썬만 재시작) 테스트할 수
  # 있는 진단용 토글이다. 기본 꺼짐.
  if not _bounded_int("EonClusterHudPathFlip", 0, 0, 1):
    return packet
  packet["path"] = _flip_points_y(packet.get("path"))
  for line in packet.get("lanes") or []:
    if isinstance(line, dict):
      line["p"] = _flip_points_y(line.get("p"))
  for line in packet.get("edges") or []:
    if isinstance(line, dict):
      line["p"] = _flip_points_y(line.get("p"))
  packet["pathOffset"] = -float(packet.get("pathOffset", 0.0) or 0.0)
  for key in ("lead", "lead2"):
    lead = packet.get(key)
    if isinstance(lead, dict) and "y" in lead:
      lead["y"] = -float(lead["y"] or 0.0)
  return packet


def _packet(sm, *args, **kwargs):
  # base._packet 의 인자가 늘어나도(noo_enabled → +path_offset 등) 그대로
  # 흘려보낸다. 고정 인자로 받으면 base 쪽 시그니처가 바뀔 때마다
  # TypeError 로 패킷이 아예 안 나가고 폰에는 "EON 연결 끊김" 만 뜬다.
  packet = _original_packet(sm, *args, **kwargs)
  packet = _apply_path_flip(packet)

  # Preserve 0 for FPS (pause) and brightness (auto), matching the UI.
  packet["hudFps"] = _bounded_int("EonClusterHudFps", 7, 0, 15)
  packet["hudMapFps"] = _bounded_int("EonClusterHudMapFps", 3, 2, 5)
  packet["hudBrightness"] = _bounded_int("EonClusterHudBrightness", 65, 0, 100)
  packet["hudJpegQuality"] = _bounded_int("EonClusterHudJpegQuality", 55, 20, 95)
  packet["hudScreenMode"] = _bounded_int("EonClusterHudScreenMode", 1, 1, 3)
  packet["hudTheme"] = _bounded_int("EonClusterHudTheme", 0, 0, 2)
  packet["hudOrientation"] = _bounded_int("EonClusterHudOrientation", 0, 0, 2)
  packet["hudMirror"] = _bounded_int("EonClusterHudMirror", 0, 0, 1)
  packet["hudLanguage"] = _bounded_int("EonClusterHudLanguage", 0, 0, 1)
  packet["hudRadarInfo"] = _bounded_int("EonClusterHudRadarInfo", 4, 0, 4)
  packet["hudBuildings"] = _bounded_int("EonClusterHudBuildings", 1, 0, 1)
  # 노면 높낮이 배율(%). 100=원본, 0=평지. 모델 z 의 부호가 기기마다 다를 수
  # 있어 음수까지 열어둔다 — 오르막이 아래로 꺼져 보이면 -100 으로 뒤집는다.
  packet["hudRoadZ"] = _bounded_int("EonClusterHudRoadZ", 100, -300, 300)
  # 주행 중 차량 pitch 를 수평선에 반영하는 정도(%). 0=끄기(정적 캘리브만).
  packet["hudPitchDyn"] = _bounded_int("EonClusterHudPitchDyn", 60, 0, 200)
  packet["hudOutputMode"] = _bounded_int("EonClusterHudOutputMode", 1, 1, 3)
  packet["hudLayoutMode"] = _bounded_int("EonClusterHudLayoutMode", 1, 1, 2)
  packet["hudOutputTarget"] = _bounded_int("EonClusterHudOutputTarget", 3, 1, 3)
  packet["hudBsdStyle"] = _bounded_int("EonClusterHudBsdStyle", 2, 1, 3)
  packet["hudCarStyle"] = _bounded_int("EonClusterHudCarStyle", 1, 1, 2)
  packet["hudRoadSigns"] = _bounded_int("EonClusterHudRoadSigns", 3, 0, 3)
  # 0(기본): 앱 내장 화살표 그림 사용
  # 1: 티맵 tbt_current_compact 사용 — 다만 이 스트림은 아이콘이 아니라
  #    "화살표+거리+도로명"이 한 장에 그려진 미니 배너라 아이콘 자리에 넣으면
  #    검은 박스나 배너 중복으로 보인다. 실물을 확인한 뒤에만 켤 것.
  packet["hudTmapIcon"] = _bounded_int("EonClusterHudTmapIcon", 0, 0, 1)
  # 0: 끔 / 1: 분기 실사 이미지 / 2: 실사 + 하단 도착정보 바
  packet["hudJunction"] = _bounded_int("EonClusterHudJunction", 2, 0, 2)
  return packet


base._packet = _packet

# manager(selfdrive/manager/process.py) 는 importlib 로 모듈을 불러
# mod.main() 을 호출한다. __name__ 이 "__main__" 이 아니므로 아래
# 블록만으로는 절대 실행되지 않는다.
main = base.main


if __name__ == "__main__":
  base.main()

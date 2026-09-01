"""Low-overhead HUD telemetry publisher for a separate Android renderer.

This process is the remote-HUD path for EON. It deliberately sends only
compact scene data and already-compressed TMAP assets: no framebuffer copies,
map decoding, JPEG rendering, or USB display traffic happens on the EON.
"""

import base64
import json
import math
import os
import signal
import socket
import struct
import time

import cereal.messaging as messaging
from common.params import Params


from selfdrive.modeld.constants import T_IDXS
from selfdrive.eon_cluster.scene import (camera_lane_position, final_lateral_path,
                                         reconcile_lane_position)

PORT = 7210
MAP_PORT = 7211
MAP_FILE = "/dev/shm/carrot_navi_map.jpg"
TBT_CURRENT_FILE = "/dev/shm/carrot_navi_tbt_current_full.png"
TBT_COMPACT_FILE = "/dev/shm/carrot_navi_tbt_current_compact.png"
CROSSROAD_FILE = "/dev/shm/carrot_navi_crossroad.png"
TBT_NEXT_FILE = "/dev/shm/carrot_navi_tbt_next.png"
LANE_BOTTOM_FILE = "/dev/shm/carrot_navi_lane_bottom.png"
NAVI_STATE = "/dev/shm/carrot_navi_route.json"
MAP_MAX_BYTES = 2 * 1024 * 1024
OVERLAY_MAX_BYTES = 512 * 1024
MAP_KEEPALIVE_S = 1.0
NAVI_MAX_AGE_MS = 35000
NAVI_GUIDANCE_MAX_AGE_MS = 3000
NAVI_STREAM_MAX_AGE_MS = 3000
NAVI_FUTURE_TOLERANCE_MS = 5000
NAVI_POSITION_PREDICT_MAX_S = 0.5
MAP_IDLE_JPEG = base64.b64decode(
  "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDABALDA4MChAODQ4SERATGCgaGBYWGDEjJR0oOjM9PDkzODdASFxOQERXRTc4UG1RV19iZ2hnPk1xeXBkeFxlZ2P/2wBDARESEhgVGC8aGi9jQjhCY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2NjY2P/wAARCAACAAIDASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDz+iiigD//2Q==")
MAX_TELEMETRY_FPS = 10
PAUSED_TELEMETRY_FPS = 2
PARAM_ENABLED = "EonClusterHud"
PARAM_CONNECTED = "EonClusterHudConnected"
PARAM_HEARTBEAT = "EonClusterHudHeartbeat"
PARAM_FPS = "EonClusterHudFps"
PARAM_MAP_FPS = "EonClusterHudMapFps"
HEARTBEAT_PERIOD_S = 2.0
PARAM_NOO_ENABLED = "NavigationOnOpenpilot"
_NAVI_CACHE = {"signature": None, "state": {}, "scene_sig": None, "scene": None, "parsed_at": 0.0}

# 날씨 조회용 마지막 좌표. 3초 신선도(NAVI_STREAM_MAX_AGE_MS)를 적용하지 않는다.
# 날씨는 15분 주기라 몇 분 지난 좌표여도 무의미한 차이다. 정밀 GPS 는 여전히
# 전송하지 않으며, 소수점 2자리(약 1.1km)로 뭉개서 내보낸다.
_WX_POS = {"lat": None, "lon": None}


def _remember_weather_position(state):
  """TMAP vehicle 스트림에서 좌표만 뽑아 캐시한다. 나이 판정은 하지 않는다."""
  vehicle = (state or {}).get("vehicle") or {}
  try:
    lat = float(vehicle.get("lat"))
    lon = float(vehicle.get("lon"))
  except (TypeError, ValueError):
    return
  if not (math.isfinite(lat) and math.isfinite(lon)):
    return
  if lat == 0.0 and lon == 0.0:
    return
  if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
    return
  _WX_POS["lat"] = lat
  _WX_POS["lon"] = lon


def _wx_pos():
  """날씨용 저정밀 좌표. 없으면 None."""
  lat, lon = _WX_POS["lat"], _WX_POS["lon"]
  if lat is None or lon is None:
    return None
  return [round(lat, 2), round(lon, 2)]
_NAVI_POSE_FILTER = {"heading": None, "lat": None, "lon": None, "seen": 0.0}
# 티맵 상태 파일은 최대 20 Hz 로 다시 쓰이지만, 여기서 필요한 건 안내 거리와
# 앞길 곡선뿐이라 5 Hz 로 충분하다. 경로 폴리라인이 길면(장거리 목적지) JSON
# 파싱이 EON 에서 프레임당 15 ms 까지 나오므로, 파싱만 이 주기로 제한한다.
# 남은거리·신선도 같은 값은 캐시된 상태로 매 프레임 다시 계산되므로 표시는
# 그대로 10 Hz 로 갱신된다.
NAVI_PARSE_INTERVAL_S = 0.20

# One-time S9 APK support for runtime layout tuning.  After the compatible APK
# is installed, ordinary HUD position/size/color tweaks only require changing
# this dictionary on EON; the values ride along with the existing 10 Hz JSON.
# Per-element positioning uses <name>Dx / <name>Dy / <name>Scale.
REMOTE_LAYOUT = {
  # 색은 여기서 강제하지 않는다. 넣으면 앱의 다크/라이트 테마를 덮어써서
  # hudTheme 설정이 주행씬에 반영되지 않는다. 특정 색을 고정하고 싶을 때만
  # driveBg / roadTop / roadBottom / pathColor 를 다시 넣을 것.
  "lightsDx": 0, "lightsDy": 0, "lightsScale": 1.0,
  "prndDx": 0, "prndDy": 0, "prndScale": 1.0,
  "speedDx": 0, "speedDy": 0, "speedScale": 1.0,
  "wheelDx": 0, "wheelDy": 0, "wheelScale": 1.0,
  "setDx": 0, "setDy": 0, "setScale": 1.0,
  "cameraDx": 0, "cameraDy": 0, "cameraScale": 1.0,
  "leadDx": 0, "leadDy": 0, "leadScale": 1.0,
  "tpmsDx": 0, "tpmsDy": 0, "tpmsScale": 1.0,
  # NOO 안내는 주행 패널 중앙으로 옮겼다(구 atc* 키는 사라짐).
  "nooDx": 0, "nooDy": 0, "nooScale": 1.0,
  "junctionDx": 0, "junctionDy": 0, "junctionScale": 1.0,
  "etaDx": 0, "etaDy": 0, "etaScale": 1.0,
  "systemDx": 0, "systemDy": 0, "systemScale": 1.0,
  # 아래 값들은 주행패널이 765 폭이던 시절에 맞춘 것이라, 5:4:1 레이아웃
  # (주행 952) 에서는 앱 기본값을 덮어써 요소를 왼쪽에 붙여 놓았다.
  # tbt1Dx / tbt2Dx 가 서로 달라 TBT 두 줄의 왼쪽도 어긋나 있었다.
  # 이제 앱 기본값(modeX 938 / etaRight 832 / TBT 오프셋 없음)을 그대로 쓴다.
  "modeX": 938, "modeY": 116, "modeSize": 29,
  "etaRight": 832, "etaY": 116, "etaTimeSize": 27, "etaLabelSize": 14, "etaGap": 8,
  "tbt1Dx": 0, "tbt1Dy": 0, "tbt1Scale": 1.0,
  "tbt2Dx": 0, "tbt2Dy": 0, "tbt2Scale": 1.0,
  "laneDx": 0, "laneDy": 0, "laneScale": 1.0,
  "rpmDx": 0, "rpmDy": 0, "rpmScale": 1.0,
  "rpmRedline": 6500,   # DH 3.8 기준. 차종 바꾸면 여기만 고치면 됨
}


class MapFrameServer(object):
  """Forward native compressed TMAP map/guidance assets without decoding."""

  ASSETS = (
    (b"MAP1", MAP_FILE, MAP_MAX_BYTES, MAP_IDLE_JPEG),
    (b"TBT1", TBT_CURRENT_FILE, OVERLAY_MAX_BYTES, b""),
    (b"TBT2", TBT_NEXT_FILE, OVERLAY_MAX_BYTES, b""),
    (b"TBT3", TBT_COMPACT_FILE, OVERLAY_MAX_BYTES, b""),
    (b"XRD1", CROSSROAD_FILE, OVERLAY_MAX_BYTES, b""),
    (b"LANE", LANE_BOTTOM_FILE, OVERLAY_MAX_BYTES, b""),
  )

  def __init__(self):
    self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.listener.bind(("0.0.0.0", MAP_PORT))
    self.listener.listen(1)
    self.listener.setblocking(False)
    self.client = None
    self.signatures = {}
    self.cached = {tag: fallback for tag, _, _, fallback in self.ASSETS}
    self.pending = set(tag for tag, _, _, _ in self.ASSETS)
    self.last_send = 0.0
    self.poll_interval = 1.0 / 5.0
    self.next_poll = 0.0

  def set_poll_fps(self, fps):
    self.poll_interval = 1.0 / max(1.0, float(fps))

  def _drop_client(self):
    if self.client is not None:
      try:
        self.client.close()
      except Exception:
        pass
    self.client = None
    self.last_send = 0.0
    self.pending = set(tag for tag, _, _, _ in self.ASSETS)

  @staticmethod
  def _valid_image(data):
    if not data:
      return False
    if data.startswith(b"\xff\xd8"):
      return data.endswith(b"\xff\xd9")
    return data.startswith(b"\x89PNG\r\n\x1a\n")

  def _refresh_asset(self, tag, path, maximum, fallback):
    try:
      stat = os.stat(path)
      signature = (getattr(stat, "st_mtime_ns", int(stat.st_mtime * 1e9)), stat.st_size)
    except (IOError, OSError):
      if self.signatures.get(tag) is not None or self.cached.get(tag, fallback) != fallback:
        self.signatures[tag] = None
        self.cached[tag] = fallback
        self.pending.add(tag)
      return

    if signature == self.signatures.get(tag):
      return
    if stat.st_size <= 4 or stat.st_size > maximum:
      return
    try:
      with open(path, "rb") as image_file:
        data = image_file.read()
    except (IOError, OSError):
      return
    if not self._valid_image(data):
      return
    self.signatures[tag] = signature
    self.cached[tag] = data
    self.pending.add(tag)

  def _send_asset(self, tag):
    payload = self.cached.get(tag, b"")
    self.client.sendall(tag + struct.pack(">I", len(payload)) + payload)

  def poll(self):
    now = time.monotonic()
    if now < self.next_poll:
      return
    # Advance from the previous deadline instead of from `now`. This avoids
    # quantizing a 3 Hz map stream down to 2.3-2.5 Hz when telemetry runs at
    # 7 or 10 Hz. If the process was stalled, skip the missed polls rather
    # than doing a burst of file I/O.
    self.next_poll += self.poll_interval
    if self.next_poll <= now:
      self.next_poll = now + self.poll_interval

    if self.client is None:
      try:
        self.client, _ = self.listener.accept()
        self.client.settimeout(0.5)
        self.last_send = 0.0
        self.pending = set(tag for tag, _, _, _ in self.ASSETS)
      except BlockingIOError:
        return

    for asset in self.ASSETS:
      self._refresh_asset(*asset)

    if now - self.last_send >= MAP_KEEPALIVE_S:
      self.pending.add(b"MAP1")

    if not self.pending:
      return
    try:
      for tag, _, _, _ in self.ASSETS:
        if tag in self.pending:
          self._send_asset(tag)
      self.pending.clear()
      self.last_send = now
    except socket.error:
      self._drop_client()

  def close(self):
    self._drop_client()
    self.listener.close()

  def set_inactive(self):
    self._drop_client()


def _field(obj, name, default=0):
  try:
    return getattr(obj, name)
  except Exception:
    return default


def _finite(value, default=0.0):
  try:
    value = float(value)
    return value if math.isfinite(value) else default
  except (TypeError, ValueError):
    return default


def _param_bool(params, key, default=False):
  """Read a boolean Params key without coupling telemetry to a UI renderer."""
  try:
    raw = params.get(key)
  except (TypeError, ValueError):
    return default
  if raw is None:
    return default
  if isinstance(raw, bytes):
    raw = raw.decode("utf-8", "replace")
  return str(raw).strip() not in ("", "0", "False", "false")


def _param_int(params, key, default=0, minimum=0, maximum=999):
  try:
    raw = params.get(key)
    value = int(raw) if raw is not None else default
  except (TypeError, ValueError):
    value = default
  return max(minimum, min(maximum, value))


def _alert(controls_state):
  """controlsState에서 Android HUD에 필요한 openpilot 이벤트 알림을 추출한다."""
  text1 = str(_field(controls_state, "alertText1", "") or "")
  text2 = str(_field(controls_state, "alertText2", "") or "")
  if not (text1 or text2):
    return None
  size = str(_field(controls_state, "alertSize", ""))
  if "none" in size.lower():
    return None

  # Keep camera/bump icons visible, but suppress only their "decelerating"
  # event box while cruise is disengaged.  Other safety alerts remain visible.
  alert_type = str(_field(controls_state, "alertType", "") or "")
  road_event_types = ("slowingDownSpeed/", "slowingDownSpeedSound/",
                      "speedBump/", "speedBumpSound/")
  road_event_text = text1 in ("과속카메라 감지 : 감속중", "과속방지턱 감지 : 감속중")
  if (not bool(_field(controls_state, "enabled", False)) and
      (alert_type.startswith(road_event_types) or road_event_text)):
    return None
  return {
    "text1": text1[:64],
    "text2": text2[:64],
    "status": str(_field(controls_state, "alertStatus", "")),
    "size": size,
  }


def _path_offset(params):
  """lateral_planner 가 최종 path_xyz[:,1] 에 더하는 오프셋 (m).
  앱도 같은 값을 경로에 더해야 실제 주행선과 화면이 맞는다.

  lateral_planner.update():
      self.path_xyz[:, 1] += self.offset_total
  offset_total = OffsetTotal (m, 사용자 수동)"""
  try:
    total = float(params.get("OffsetTotal", encoding="utf8") or 0.0)
  except (TypeError, ValueError):
    total = 0.0
  total = max(-1.0, min(1.0, total))

  return round(total, 3)


def _engine_rpm(car_state):
  """EMS11 'N' 엔진 회전수. 신호가 없거나 EV 면 0 이 올라오므로 -1 로 바꿔
  앱이 게이지를 아예 숨기게 한다."""
  rpm = _finite(_field(car_state, "engineRpm", 0.0))
  if rpm <= 0.0 or rpm > 12000.0:
    return -1
  return int(round(rpm))


def _calib_pitch(live_calibration):
  """liveCalibration.rpyCalib 의 pitch(rad). 앱 카메라 수평선 보정에 쓴다."""
  rpy = list(_field(live_calibration, "rpyCalib", []) or [])
  if len(rpy) < 2:
    return 0.0
  pitch = _finite(rpy[1], 0.0)
  return round(max(-0.15, min(0.15, pitch)), 4)


def _remote_output_enabled(params):
  return params.get_bool(PARAM_ENABLED)


def _publish_connected(params, state, value):
  if state[0] is not value:
    try:
      params.put_bool(PARAM_CONNECTED, value)
      state[0] = value
    except Exception as exc:
      print("remote HUD connected flag failed: %s" % exc, flush=True)


def _publish_heartbeat(params, state):
  # 2026-08-19: 하트비트를 "폰이 ACK 를 보냈는가(connected)" 와 분리했다.
  # 예전에는 connected 가 True 일 때만 찍어서, 와이파이가 잠깐 흔들리거나
  # ACK 가 늦으면 EON UI 가 내비/NOO 패널을 다시 그렸다(= 외부 HUD 를 쓰는데도
  # 이온에 지도·NOO 박스가 뜨는 증상). 이제는 원격 출력이 켜져 있고 이 프로세스가
  # 살아 있으면 2초마다 찍는다. 프로세스가 죽거나 EonClusterHud 를 끄면
  # 10초 뒤 EON 이 다시 그린다.
  now = time.time()
  if now - state[1] < HEARTBEAT_PERIOD_S:
    return
  try:
    params.put(PARAM_HEARTBEAT, str(int(now)))
    state[1] = now
  except Exception as exc:
    print("remote HUD heartbeat failed: %s" % exc, flush=True)


def _stop_point(long_plan):
  """신호/E2E 정지까지 남은 거리(m). 없으면 None.

  별도 메시지 필드를 만들지 않고 이미 구독 중인 longitudinalPlan 의 속도
  궤적을 적분한다. 속도가 0 에 수렴하는 지점이 곧 정지 지점이다.
  trafficState/onStop 으로 게이트해서 앞차 추종 정차에는 선을 그리지 않는다.
  """
  traffic = int(_finite(_field(long_plan, "trafficState", 0)))
  if traffic <= 0 and not bool(_field(long_plan, "onStop", False)):
    return None
  speeds = list(_field(long_plan, "speeds", []) or [])
  if len(speeds) < 2:
    return None
  # T_IDXS 와 같은 비균등 시간축. 여기서는 인접 구간을 사다리꼴로 적분한다.
  dist = 0.0
  for i in range(1, min(len(speeds), len(T_IDXS))):
    v0 = _finite(speeds[i - 1])
    v1 = _finite(speeds[i])
    if v1 < 0.3:
      return round(max(0.0, dist), 1)
    dist += (v0 + v1) * 0.5 * (T_IDXS[i] - T_IDXS[i - 1])
  return None


def _first(seq, default=0.0):
  try:
    for value in seq:
      return value
  except TypeError:
    pass
  return default


def _line_points(position, limit=33, with_z=False):
  xs = list(_field(position, "x", []) or [])
  ys = list(_field(position, "y", []) or [])
  count = min(len(xs), len(ys), limit)
  if count < 2:
    return []
  if with_z:
    # 2026-08-20: 노면 높낮이. 앱의 ModelWorldGL.project() 가 이 z 로 도로면을
    # 올리고 내린다(오르막/내리막/둔덕). 경로 하나만 보내면 되는 이유는,
    # 같은 거리에서는 차선·도로경계·노면이 모두 같은 높이이기 때문이다.
    # 차선까지 z 를 실으면 패킷만 커지고 그림은 같다.
    zs = list(_field(position, "z", []) or [])
    if len(zs) >= count:
      return [[round(_finite(xs[i]), 2), round(_finite(ys[i]), 2),
               round(_finite(zs[i]), 2)] for i in range(count)]
  # 예전에는 count // 12 로 솎아 17점만 보냈다. 급커브에서 보간이 실제
  # 곡률을 못 따라가므로 33점을 전부 보낸다. 경로+차선4+경계2 가 두 배가 돼도
  # 패킷은 1.9KB → 3~4KB 수준이고 EON 부하(1~3%)는 그대로다.
  return [[round(_finite(xs[i]), 2), round(_finite(ys[i]), 2)] for i in range(count)]


def _model_lines(model, name, confidence_name, confidence_default,
                 invert_confidence=False, preserve_slots=False):
  lines = list(_field(model, name, []) or [])
  confidences = list(_field(model, confidence_name, []) or [])
  result = []
  slot_count = min(4, max(len(lines), len(confidences)))
  for index in range(slot_count):
    line = lines[index] if index < len(lines) else None
    points = _line_points(line)
    confidence = confidences[index] if index < len(confidences) else confidence_default
    if invert_confidence:
      confidence = 1.0 - _finite(confidence, 1.0)
    confidence = round(max(0.0, min(1.0,
                                   _finite(confidence, confidence_default))), 2)
    if len(points) < 2:
      # laneLines indices have fixed meaning (0 outer-left, 1/2 ego,
      # 3 outer-right).  Never let one malformed line shift the remaining
      # slots and turn a single-lane road into a phantom adjacent lane.
      if preserve_slots:
        result.append({"p": [], "c": 0.0})
      continue
    result.append({"p": points, "c": confidence})
  return result


def _limit_lane_visibility(lines, lane_position):
  """Keep modelV2's fixed four-line layout but hide impossible adjacent lanes.

  Indices 1/2 are the ego-lane boundaries.  Index 0 exists only when there is
  a lane left of ego, and index 3 only when there is a lane right of ego.
  Keeping four entries preserves the Android decoder's stable index mapping.
  """
  if not isinstance(lane_position, dict) or len(lines) < 4:
    return lines
  try:
    lane_count = int(lane_position.get("n", 0))
    current_lane = int(lane_position.get("cur", 0))
  except (TypeError, ValueError):
    return lines
  if lane_count < 1 or current_lane < 1 or current_lane > lane_count:
    return lines

  visible = {1, 2}
  if current_lane > 1:
    visible.add(0)
  if current_lane < lane_count:
    visible.add(3)
  for index, line in enumerate(lines):
    if index not in visible and isinstance(line, dict):
      line["c"] = 0.0
  return lines


def _lead(radar_state, name):
  lead = _field(radar_state, name, None)
  if not bool(_field(lead, "status", False)):
    return None
  return {
    "d": round(max(0.0, _finite(_field(lead, "dRel", 0.0))), 1),
    "y": round(_finite(_field(lead, "yRel", 0.0)), 2),
    "v": round(_finite(_field(lead, "vRel", 0.0)) * 3.6, 1),
    # 앞차 가속도(m/s^2, 칼만필터). 음수가 크면 앞차가 실제로 감속 중이라는
    # 뜻이라 앱이 후미등을 켠다. vRel 만으로는 "내가 더 빠른 것"과 구분이
    # 안 돼서 오르막 추월 등에서 오검출이 난다.
    "a": round(_finite(_field(lead, "aLeadK", 0.0)), 2),
  }


def _gear(car_state):
  value = str(_field(car_state, "gearShifter", "") or "").split(".")[-1].lower()
  label = {"park": "P", "reverse": "R", "neutral": "N", "drive": "D",
           "sport": "S", "low": "L", "brake": "B"}.get(value)
  if label:
    return label
  step = int(_finite(_field(car_state, "gearStep", 0)))
  return str(step) if step > 0 else "--"


def _apply_speed(car_control):
  """NOO·곡선·카메라 감속으로 실제 적용 중인 상한과 그 원인.

  EON 화면(drawCarrotHud)과 같은 규칙: 설정속도와 0.5 km/h 넘게 차이날 때만
  의미가 있다. (속도 kph, 원인 문자열) 을 돌려준다.
  """
  smoother = _field(car_control, "sccSmoother", None)
  if smoother is None:
    return 0, ""
  apply_max = _finite(_field(smoother, "applyMaxSpeed", 0.0))
  cruise_max = _finite(_field(smoother, "cruiseMaxSpeed", 0.0))
  source = str(_field(smoother, "applySource", "") or "")
  if apply_max <= 0 or cruise_max <= 0 or abs(apply_max - cruise_max) <= 0.5:
    return 0, ""
  return max(0, int(round(apply_max))), source[:8]


def _set_speed(controls_state, car_control):
  smoother = _field(car_control, "sccSmoother", None)
  value = _field(smoother, "cruiseMaxSpeed", None)
  if value is None:
    value = _field(controls_state, "vCruiseCluster", _field(controls_state, "vCruise", 0.0))
  return max(0, int(round(_finite(value))))


def _navi_scene(state):
  """티맵 lane_current + route.polyline 을 HUD 3D씬용으로 가공한다.

  파일 서명 단위(_NAVI_CACHE)로 캐시하므로 폴리라인 최근접점 탐색이 10Hz 마다
  돌지 않는다(파일 자체가 보통 1Hz 갱신).

  결과: {"lane": {"n","cur","turns","avail","dist"}, "cat": roadcate,
        "curve": [[x, y], ...]}  (전방 x m, 좌 +y m — 주행씬 좌표계와 동일)
  """
  scene = {}

  lane = state.get("lane_current") or {}
  try:
    n = int(lane.get("count", 0) or 0)
    cur = int(lane.get("current_lane", 0) or 0)
  except (TypeError, ValueError):
    n, cur = 0, 0
  # A single-lane TMAP count is essential on bollard/median roads: without it
  # the camera-only road edge can be rounded into a phantom lane on the left.
  if 1 <= n <= 8 and 1 <= cur <= n:
    def _ints(key):
      raw = lane.get(key) or []
      out = []
      for i in range(n):
        try:
          out.append(int(raw[i]))
        except (TypeError, ValueError, IndexError):
          out.append(0)
      return out
    try:
      lane_dist = max(0, int(round(float(lane.get("distance_m", 0) or 0))))
    except (TypeError, ValueError):
      lane_dist = 0
    scene["lane"] = {"n": n, "cur": cur, "turns": _ints("turn_info"),
                     "avail": _ints("available"), "dist": lane_dist}
  try:
    cat = int(lane.get("road_category", -1))
  except (TypeError, ValueError):
    cat = -1
  if cat >= 0:
    scene["cat"] = cat

  # route.polyline → 자차 로컬좌표. 위치/방위는 티맵 vehicle 스트림(EON GPS 불요).
  vehicle = state.get("vehicle") or {}
  route = state.get("route") or {}
  poly = route.get("polyline") or []
  try:
    lat0 = float(vehicle.get("lat"))
    lon0 = float(vehicle.get("lon"))
    heading = math.radians(float(vehicle.get("heading_deg")))
  except (TypeError, ValueError):
    lat0 = None
  if lat0 is not None:
    # TMAP 경로를 차량 좌표계로 변환하기 위한 위치/방위.
    scene["pos"] = [round(lat0, 6), round(lon0, 6), round(math.degrees(heading), 1)]
  if lat0 is not None and len(poly) >= 2:
    m_lat = 111320.0
    m_lon = 111320.0 * math.cos(math.radians(lat0))
    sin_h, cos_h = math.sin(heading), math.cos(heading)
    # 최근접점부터 시작해 전방 380m 까지, 12m 이상 간격으로 최대 24점.
    best_i, best_d = 0, float("inf")
    pts = []
    for i, pt in enumerate(poly):
      try:
        e = (float(pt.get("lon")) - lon0) * m_lon
        nn = (float(pt.get("lat")) - lat0) * m_lat
      except (TypeError, ValueError, AttributeError):
        pts.append(None)
        continue
      x = e * sin_h + nn * cos_h          # 전방 +
      # HUD world uses left-positive lateral coordinates.  Keep the
      # TMAP display polyline in the same handedness; NOO/control paths are
      # generated independently and remain untouched.
      y = nn * sin_h - e * cos_h           # 주행씬 좌 +
      pts.append((x, y))
      d = x * x + y * y
      if d < best_d:
        best_d, best_i = d, i
    curve = []
    last_x = -1e9
    for pt in pts[best_i:]:
      if pt is None:
        continue
      x, y = pt
      if x < 0.0 or x <= last_x + 12.0:
        continue
      if x > 380.0:
        break
      curve.append([round(x, 1), round(y, 1)])
      last_x = x
      if len(curve) >= 24:
        break
    if len(curve) >= 2:
      scene["curve"] = curve

  return scene or None


def _read_navi_summary():
  try:
    stat = os.stat(NAVI_STATE)
    signature = (getattr(stat, "st_mtime_ns", int(stat.st_mtime * 1e9)), stat.st_size)
  except (IOError, OSError):
    _NAVI_CACHE["signature"] = None
    _NAVI_CACHE["state"] = {}
    _NAVI_CACHE["parsed_at"] = 0.0
    return {}

  now = time.monotonic()
  stale_enough = now - _NAVI_CACHE["parsed_at"] >= NAVI_PARSE_INTERVAL_S
  if signature != _NAVI_CACHE["signature"] and stale_enough:
    try:
      with open(NAVI_STATE, "r") as state_file:
        state = json.load(state_file)
    except (IOError, ValueError):
      return _NAVI_CACHE["state"]
    _NAVI_CACHE["signature"] = signature
    _NAVI_CACHE["state"] = state
    _NAVI_CACHE["parsed_at"] = now
  else:
    state = _NAVI_CACHE["state"]

  now_ms = int(time.time() * 1000)
  # 날씨 좌표는 navi 전체가 만료돼도 유지한다(목적지 미설정 상태 포함).
  _remember_weather_position(state)
  updated_at = int(state.get("updated_at_ms", 0) or 0)
  if updated_at <= 0 or abs(now_ms - updated_at) > NAVI_MAX_AGE_MS:
    return {}

  route = state.get("route") or {}
  guide = state.get("guidance_current") or {}
  vehicle = state.get("vehicle") or {}
  stream_times = state.get("stream_updated_at_ms") or {}
  guidance_at = int(stream_times.get("guidance_current", updated_at) or 0)
  vehicle_at = int(stream_times.get("vehicle", 0) or 0)
  route_at = int(stream_times.get("route", 0) or 0)
  lane_at = int(stream_times.get("lane_current", 0) or 0)

  def _stream_live(timestamp_ms, maximum_age_ms=NAVI_STREAM_MAX_AGE_MS):
    age_ms = now_ms - timestamp_ms
    return timestamp_ms > 0 and -NAVI_FUTURE_TOLERANCE_MS <= age_ms <= maximum_age_ms

  guidance_live = -NAVI_FUTURE_TOLERANCE_MS <= now_ms - guidance_at <= NAVI_GUIDANCE_MAX_AGE_MS
  vehicle_live = _stream_live(vehicle_at)
  route_live = _stream_live(route_at)
  lane_live = _stream_live(lane_at)
  status = state.get("navigation_status") or {}
  active = True
  if isinstance(status, dict):
    for key in ("active", "is_active", "isActive", "navigating", "is_navigating", "isNavigating",
                "route_active", "routeActive"):
      if key in status and not bool(status.get(key)):
        active = False
    status_text = str(status.get("state", status.get("status", "")) or "").lower()
    if status_text in ("idle", "inactive", "off", "stopped", "ended", "none"):
      active = False
  try:
    remain_distance = float(route.get("remain_distance_m", 0) or 0)
  except (TypeError, ValueError):
    remain_distance = 0.0
  active = active and (remain_distance > 0 or bool(guide))

  # Keep the navigation scene cached so route intent remains stable across
  # short guidance-state transitions.
  if _NAVI_CACHE["scene_sig"] != _NAVI_CACHE["signature"]:
    _NAVI_CACHE["scene_sig"] = _NAVI_CACHE["signature"]
    try:
      _NAVI_CACHE["scene"] = _navi_scene(state)
    except Exception:
      _NAVI_CACHE["scene"] = None
  scene = dict(_NAVI_CACHE["scene"] or {})
  if not vehicle_live:
    scene.pop("pos", None)
    scene.pop("curve", None)
  else:
    scene["posAgeMs"] = max(0, min(NAVI_STREAM_MAX_AGE_MS, now_ms - vehicle_at))
  if not route_live:
    scene.pop("curve", None)
  if not lane_live:
    scene.pop("lane", None)
    scene.pop("cat", None)

  if not active:
    inactive = {"active": False}
    if scene:
      inactive["scene"] = scene
    return inactive

  try:
    turn_type = int(guide.get("turn_type", 0) or 0)
    turn_distance = max(0, int(round(float(guide.get("distance_m", 0) or 0))))
    remain_time = max(0, int(route.get("remain_time_sec", 0) or 0))
  except (TypeError, ValueError):
    turn_type, turn_distance, remain_time = 0, 0, 0
  title = str(guide.get("main_text") or guide.get("road_name") or vehicle.get("road_name") or "")

  # 다음 회전 (폰 HUD TBT 2행). 없으면 키 자체를 넣지 않는다.
  next_guide = state.get("guidance_next") or {}
  next_summary = None
  if next_guide:
    try:
      next_distance = int(round(float(next_guide.get("distance_m", 0) or 0)))
    except (TypeError, ValueError):
      next_distance = -1
    if next_distance >= 0:
      try:
        next_type = int(next_guide.get("turn_type", 0) or 0)
      except (TypeError, ValueError):
        next_type = 0
      next_title = str(next_guide.get("main_text") or next_guide.get("road_name") or "")
      next_summary = {"turnType": next_type, "turnDist": next_distance,
                      "title": next_title[:48]}

  summary = {
    "active": True,
    "guidanceLive": bool(guidance_live),
    "turnType": turn_type,
    "turnDist": turn_distance,
    "remainTime": remain_time,
    "remainDist": max(0, int(round(remain_distance))),
    "title": title[:48],
  }
  if next_summary is not None:
    summary["next"] = next_summary
  if scene:
    summary["scene"] = scene
  return summary


def _compensate_navi_pose(navi, v_ego):
  """Keep stopped heading stable and project a fresh TMAP fix to packet time."""
  if not isinstance(navi, dict):
    return
  scene = navi.get("scene")
  if not isinstance(scene, dict):
    return
  pos = scene.get("pos")
  if not isinstance(pos, list) or len(pos) < 3:
    return
  try:
    lat = float(pos[0])
    lon = float(pos[1])
    heading_deg = float(pos[2])
    age_s = max(0.0, min(NAVI_POSITION_PREDICT_MAX_S,
                         float(scene.get("posAgeMs", 0) or 0) * 0.001))
    speed = max(0.0, min(70.0, float(v_ego)))
  except (TypeError, ValueError):
    return
  if not all(math.isfinite(value) for value in (lat, lon, heading_deg, age_s, speed)):
    scene.pop("pos", None)
    scene.pop("curve", None)
    return
  if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
    scene.pop("pos", None)
    scene.pop("curve", None)
    return

  now = time.monotonic()
  raw_heading_deg = heading_deg
  previous_heading = _NAVI_POSE_FILTER["heading"]
  previous_lat = _NAVI_POSE_FILTER["lat"]
  previous_lon = _NAVI_POSE_FILTER["lon"]
  nearby = False
  if previous_lat is not None and previous_lon is not None:
    north = (lat - previous_lat) * 111320.0
    east = (lon - previous_lon) * 111320.0 * math.cos(math.radians(lat))
    nearby = north * north + east * east < 15.0 * 15.0
  if speed < 0.5 and previous_heading is not None and nearby and now - _NAVI_POSE_FILTER["seen"] < 5.0:
    heading_deg = previous_heading
  else:
    _NAVI_POSE_FILTER["heading"] = heading_deg
  _NAVI_POSE_FILTER["lat"] = lat
  _NAVI_POSE_FILTER["lon"] = lon
  _NAVI_POSE_FILTER["seen"] = now

  distance = speed * age_s
  heading = math.radians(heading_deg)
  lat += math.cos(heading) * distance / 111320.0
  lon_scale = 111320.0 * max(0.1, math.cos(math.radians(lat)))
  lon += math.sin(heading) * distance / lon_scale

  # curve was built in the raw-heading frame at the unprojected position.
  # Rotate it into the stabilized frame and move the origin forward by the
  # same latency distance so the TMAP trace and model world share one frame.
  curve = scene.get("curve")
  if isinstance(curve, list):
    delta = math.radians(heading_deg - raw_heading_deg)
    cos_delta, sin_delta = math.cos(delta), math.sin(delta)
    adjusted = []
    for point in curve:
      if not isinstance(point, list) or len(point) < 2:
        continue
      try:
        x = float(point[0])
        y = float(point[1])
      except (TypeError, ValueError):
        continue
      adjusted.append([round(x * cos_delta - y * sin_delta - distance, 1),
                       round(x * sin_delta + y * cos_delta, 1)])
    if len(adjusted) >= 2:
      scene["curve"] = adjusted
    else:
      scene.pop("curve", None)

  scene["pos"] = [round(lat, 6), round(lon, 6), round(heading_deg, 1)]
  scene["posAgeMs"] = 0


def _packet(sm, noo_enabled, path_offset=0.0):
  car = sm["carState"]
  controls = sm["controlsState"]
  road = sm["roadLimitSpeed"]
  device = sm["deviceState"]
  plan = sm["longitudinalPlan"]
  accels = list(_field(plan, "accels", []) or [])
  cam_type = int(_finite(_field(road, "camType", 0)))
  cam_speed = int(_finite(_field(road, "camLimitSpeed", 0)))
  cam_dist = int(_finite(_field(road, "camLimitSpeedLeftDist", 0)))
  # camType 22 = 과속방지턱. 아래 분기에서 구간단속 값으로 덮이기 전에 따로 빼둔다
  # (EON onroad.cc drawSpeedLimit 의 bump_detected 와 같은 판정).
  bump_dist = cam_dist if (cam_type == 22 and cam_dist > 0) else 0
  camera_section = False
  if cam_type == 22 or cam_speed <= 0 or cam_dist <= 0:
    cam_speed = int(_finite(_field(road, "sectionLimitSpeed", 0)))
    cam_dist = int(_finite(_field(road, "sectionLeftDist", 0)))
    camera_section = cam_speed > 0 and cam_dist > 0
  cpu = list(_field(device, "cpuUsagePercent", []) or [])
  temps = list(_field(device, "cpuTempC", []) or [])
  cpu_avg = (sum(float(v) for v in cpu) / len(cpu)) if cpu else 0.0
  temp_avg = (sum(float(v) for v in temps) / len(temps)) if temps else 0.0
  engine_temp = _finite(_field(car, "engineOilTempC", -1000.0), -1000.0)
  coolant_temp = _finite(_field(car, "engineCoolantTempC", -1000.0), -1000.0)
  engine_temp = engine_temp if -50.0 <= engine_temp <= 200.0 else None
  coolant_temp = coolant_temp if -50.0 <= coolant_temp <= 200.0 else None
  # 차량 CAN 이 안 붙어 있으면 0.0 이 그대로 올라와 실제 0도와 구분되지 않는다.
  if not sm.alive.get("carState", False):
    engine_temp = None
    coolant_temp = None
  gap = int(_finite(_field(controls, "longCruiseGap", 0)))
  if not 1 <= gap <= 4:
    gap = int(_finite(_field(car, "cruiseGap", 0)))
  mode = int(_finite(_field(controls, "myDrivingMode", 3), 3))
  if not 1 <= mode <= 4:
    mode = 3
  tpms = _field(car, "tpms", None)
  parking_sensors = _field(car, "parkingSensors", None)
  navi = _read_navi_summary()
  _compensate_navi_pose(navi, _finite(_field(car, "vEgo", 0.0)))
  # Preserve the compensated TMAP pose for the phone-local, display-only map
  # context before removing it from the diagnostic/navigation object.
  map_pose = None
  navi_scene = navi.get("scene") if isinstance(navi, dict) else None
  if isinstance(navi_scene, dict):
    raw_map_pose = navi_scene.get("pos")
    if isinstance(raw_map_pose, list) and len(raw_map_pose) >= 3:
      try:
        candidate = [float(raw_map_pose[0]), float(raw_map_pose[1]),
                     float(raw_map_pose[2])]
        if (all(math.isfinite(value) for value in candidate) and
            -85.0 <= candidate[0] <= 85.0 and -180.0 <= candidate[1] <= 180.0):
          map_pose = [round(candidate[0], 6), round(candidate[1], 6),
                      round(candidate[2], 1)]
      except (TypeError, ValueError):
        pass
    navi_scene.pop("pos", None)
    navi_scene.pop("posAgeMs", None)
  raw_lane_position = camera_lane_position(sm["modelV2"])
  route_lane_count = 0
  try:
    route_lane_count = int(navi.get("scene", {}).get("lane", {}).get("n", 0))
  except (AttributeError, TypeError, ValueError):
    route_lane_count = 0
  reconciled_lane_position = reconcile_lane_position(raw_lane_position, route_lane_count)
  lane_position = reconciled_lane_position or raw_lane_position

  # Keep the nested object for the diagnostic panel, and also publish the flat
  # keys consumed by the installed driving-scene renderer.  When NOO is inactive
  # (for example while stopped), use the same conservative HUD-only camera /
  # TMAP reconciliation so a shoulder or median cannot shift the displayed car
  # from lane 1 to lane 2.
  noo_camera_count = int(_finite(_field(sm["lateralPlan"], "nooCameraLaneCount", 0)))
  noo_route_count = int(_finite(_field(sm["lateralPlan"], "nooRouteLaneCount", 0)))
  noo_current_lane = int(_finite(_field(sm["lateralPlan"], "nooCurrentLane", 0)))
  noo_target_lane = int(_finite(_field(sm["lateralPlan"], "nooTargetLane", 0)))
  noo_lane_direction = int(_finite(_field(sm["lateralPlan"], "nooLaneChangeDirection", 0)))
  planner_lane_valid = (noo_route_count == route_lane_count and
                        1 <= noo_current_lane <= route_lane_count and
                        1 <= noo_target_lane <= route_lane_count)
  hud_camera_count = noo_camera_count or int((raw_lane_position or {}).get("n", 0))
  if planner_lane_valid:
    hud_route_count = noo_route_count
    hud_current_lane = noo_current_lane
    hud_target_lane = noo_target_lane
  else:
    hud_route_count = route_lane_count
    hud_current_lane = int((lane_position or {}).get("cur", 0))
    hud_target_lane = hud_current_lane

  apply_speed, apply_source = _apply_speed(sm["carControl"])
  hud_path = final_lateral_path(sm["lateralPlan"], sm["modelV2"], T_IDXS)
  path_final = len(hud_path) >= 2
  if not path_final:
    hud_path = _line_points(_field(sm["modelV2"], "position", None), with_z=True)
  hud_lanes = _model_lines(sm["modelV2"], "laneLines", "laneLineProbs", 0.0,
                           preserve_slots=True)
  hud_lanes = _limit_lane_visibility(hud_lanes, lane_position)
  hud_edges = _model_lines(sm["modelV2"], "roadEdges", "roadEdgeStds", 1.0, True)
  # Keep camera-observed lane lines and road edges in their original modelV2
  # coordinates.  The MPC ribbon is a separate control prediction and must
  # never drag the perceived road sideways on the HUD.
  return {
    "v": 6,
    "t": int(time.time() * 1000),
    "mapPose": map_pose,
    "layout": REMOTE_LAYOUT,
    "speed": int(round(_finite(_field(car, "vEgoCluster", _field(car, "vEgo", 0.0))) * 3.6)),
    "set": _set_speed(controls, sm["carControl"]),
    "applySpeed": apply_speed,
    "applySource": apply_source,
    "enabled": bool(_field(controls, "enabled", False)),
    "gear": _gear(car),
    "gap": gap if 1 <= gap <= 4 else 0,
    "drivingMode": mode,
    "limit": max(0, int(_finite(_field(road, "roadLimitSpeed", 0)))),
    "camera": max(0, cam_speed),
    "cameraDist": max(0, cam_dist),
    "cameraSection": bool(camera_section),
    "bumpDist": bump_dist,
    "leftBsd": bool(_field(car, "leftBlindspot", False)),
    "rightBsd": bool(_field(car, "rightBlindspot", False)),
    "steer": round(_finite(_field(car, "steeringAngleDeg", 0.0)), 1),
    "accel": round(_finite(accels[0] if accels else 0.0), 2),
    "desiredDistance": round(max(0.0, min(150.0,
        _finite(_field(sm["longitudinalPlan"], "desiredDistance", 0.0)))), 1),
    "cpu": int(round(cpu_avg)),
    "temp": round(temp_avg, 1),
    "system": {
      "cpu": round(cpu_avg, 1),
      "temp": round(temp_avg, 1),
      "engineTemp": engine_temp,
      "coolantTemp": coolant_temp,
      "cores": [round(float(v), 1) for v in cpu[:8]],
    },
    "leftBlinker": bool(_field(car, "leftBlinker", False)),
    "rightBlinker": bool(_field(car, "rightBlinker", False)),
    "lowBeam": bool(_field(car, "lowBeam", False)),
    "highBeam": bool(_field(car, "highBeam", False)),
    "frontFog": bool(_field(car, "frontFogLight", False)),
    "wiperMode": max(0, min(5, int(_finite(_field(car, "wiperMode", 0))))),
    "seatbeltUnlatched": bool(_field(car, "seatbeltUnlatched", False)),
    "doors": {
      "fl": bool(_field(car, "frontLeftDoorOpen", False)),
      "fr": bool(_field(car, "frontRightDoorOpen", False)),
      "rl": bool(_field(car, "rearLeftDoorOpen", False)),
      "rr": bool(_field(car, "rearRightDoorOpen", False)),
      "trunk": bool(_field(car, "trunkOpen", False)),
      "hood": bool(_field(car, "hoodOpen", False)),
    },
    "windows": {
      "fl": bool(_field(car, "frontLeftWindowOpen", False)),
      "fr": bool(_field(car, "frontRightWindowOpen", False)),
      "rl": bool(_field(car, "rearLeftWindowOpen", False)),
      "rr": bool(_field(car, "rearRightWindowOpen", False)),
    },
    "parkingBrake": bool(_field(car, "parkingBrake", False)),
    "steerFaultTemporary": bool(_field(car, "steerFaultTemporary", False)),
    "steerFaultPermanent": bool(_field(car, "steerFaultPermanent", False)),
    "stockFcw": bool(_field(car, "stockFcw", False)),
    "stockAeb": bool(_field(car, "stockAeb", False)),
    "aebSystemFault": bool(_field(car, "aebSystemFault", False)),
    "blindSpotSystemFault": bool(_field(car, "blindSpotSystemFault", False)),
    "lowFuelWarning": bool(_field(car, "lowFuelWarning", False)),
    "parkingSensors": {
      "valid": bool(_field(parking_sensors, "valid", False)),
      "fl": int(_finite(_field(parking_sensors, "frontLeft", 0))),
      "fc": int(_finite(_field(parking_sensors, "frontCenter", 0))),
      "fr": int(_finite(_field(parking_sensors, "frontRight", 0))),
      "rl": int(_finite(_field(parking_sensors, "rearLeft", 0))),
      "rc": int(_finite(_field(parking_sensors, "rearCenter", 0))),
      "rr": int(_finite(_field(parking_sensors, "rearRight", 0))),
    },
    "outsideTemp": round(_finite(_field(car, "outsideTempC", -1000.0), -1000.0), 1),
    "distanceToEmpty": round(_finite(_field(car, "distanceToEmptyKm", -1.0)), 1),
    "rpm": _engine_rpm(car),
    "tpms": {
      "fl": _finite(_field(tpms, "fl", -1.0), -1.0),
      "fr": _finite(_field(tpms, "fr", -1.0), -1.0),
      "rl": _finite(_field(tpms, "rl", -1.0), -1.0),
      "rr": _finite(_field(tpms, "rr", -1.0), -1.0),
    },
    # Legacy JSON keys are retained so the installed S9 APK remains wire
    # compatible. Their values now come exclusively from NOO.
    "atcMode": 1 if noo_enabled else 0,
    "atcBlend": round(_finite(_field(sm["lateralPlan"], "nooMapBlend", 0.0)), 3),
    "atcDirection": int(_finite(_field(sm["lateralPlan"], "nooTurnDirection", 0))),
    # New key alongside the legacy atcMode above. Old APKs ignore it.
    "nooMode": 1 if noo_enabled else 0,
    # The driving-scene renderer reads these flat wire keys. They must remain available alongside
    # the nested diagnostic object below for already-installed APKs.
    "nooCameraLaneCount": hud_camera_count,
    "nooRouteLaneCount": hud_route_count,
    "nooCurrentLane": hud_current_lane,
    "nooTargetLane": hud_target_lane,
    # Lane-change diagnostics. cam/map are the camera and TMAP lane counts; a
    # permanent mismatch there is why a lane change never starts.
    "noo": {
      "cam": noo_camera_count,
      "map": noo_route_count,
      "cur": noo_current_lane,
      "tgt": noo_target_lane,
      "dir": noo_lane_direction,
    },
    # The optimized MPC state follows a reference that already contains
    # OffsetTotal. Keep the old offset only when falling back to the raw model
    # path so old and new APKs both avoid adding it twice.
    "pathOffset": 0.0 if path_final else float(path_offset),
    "pathFinal": path_final,
    # 현재 차량 자세 pitch(rad). liveCalibration 의 정적 보정과 달리 주행 중
    # 가감속·요철로 실시간 변한다. 앱은 여기에 게인을 곱해 수평선을 움직인다.
    "pitch": round(_finite(_first(_field(_field(sm["modelV2"], "orientation", None), "y", []))), 4),
    "calibPitch": _calib_pitch(sm["liveCalibration"]),
    # 정지선까지 거리(m). None 이면 앱이 안 그린다.
    "stopDist": _stop_point(sm["longitudinalPlan"]),
    # 모델이 추정한 자기 차로 폭(m). 앱의 폴백 도로폭 계산에 쓴다.
    "laneWidth": round(_finite(_field(sm["lateralPlan"], "laneWidth", 0.0)), 2),
    # 카메라 roadEdges/laneLines 로 추정한 도로 내 자차 위치. 화면 배치에만
    # 사용하며 조향 제어에는 절대 되먹이지 않는다.
    "lanePosition": lane_position,
    "alert": _alert(controls),
    "navi": navi,
    # 날씨 조회 전용 저정밀 좌표(소수점 2자리). navi.scene.pos 는 위에서 제거되고
    # TMAP 안내가 꺼져 있으면 navi 자체가 비므로, 별도 최상위 키로 내보낸다.
    "wxPos": _wx_pos(),
    "path": hud_path,
    "lanes": hud_lanes,
    "edges": hud_edges,
    "lead": _lead(sm["radarState"], "leadOne"),
    "lead2": _lead(sm["radarState"], "leadTwo"),
  }


def main():
  params = Params()
  running = [True]
  signal.signal(signal.SIGINT, lambda *_: running.__setitem__(0, False))
  signal.signal(signal.SIGTERM, lambda *_: running.__setitem__(0, False))
  sm = messaging.SubMaster(["carState", "carControl", "controlsState", "deviceState",
                            "modelV2", "radarState", "longitudinalPlan", "roadLimitSpeed",
                            "liveCalibration", "lateralPlan"])
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
  sock.setblocking(False)
  last_ack = 0.0
  connected = False
  published = [None, 0.0]
  map_server = MapFrameServer()
  noo_enabled = _param_bool(params, PARAM_NOO_ENABLED)
  path_offset = _path_offset(params)
  configured_fps = _param_int(params, PARAM_FPS, 7, 0, 15)
  telemetry_fps = PAUSED_TELEMETRY_FPS if configured_fps == 0 else min(MAX_TELEMETRY_FPS, configured_fps)
  map_server.set_poll_fps(_param_int(params, PARAM_MAP_FPS, 3, 2, 5))
  next_param_read = 0.0
  while running[0]:
    started = time.monotonic()
    if not _remote_output_enabled(params):
      connected = False
      last_ack = 0.0
      _publish_connected(params, published, False)
      map_server.set_inactive()
      time.sleep(0.25)
      continue
    _publish_heartbeat(params, published)
    if started >= next_param_read:
      noo_enabled = _param_bool(params, PARAM_NOO_ENABLED)
      path_offset = _path_offset(params)
      configured_fps = _param_int(params, PARAM_FPS, 7, 0, 15)
      telemetry_fps = PAUSED_TELEMETRY_FPS if configured_fps == 0 else min(MAX_TELEMETRY_FPS, configured_fps)
      map_server.set_poll_fps(_param_int(params, PARAM_MAP_FPS, 3, 2, 5))
      next_param_read = started + 1.0
    sm.update(0)
    try:
      sock.sendto(json.dumps(_packet(sm, noo_enabled, path_offset), separators=(",", ":"), ensure_ascii=False).encode("utf-8"),
                  ("255.255.255.255", PORT))
      try:
        while True:
          reply, _ = sock.recvfrom(64)
          if reply == b"HUD1":
            last_ack = time.monotonic()
      except (BlockingIOError, socket.error):
        pass
      connected = time.monotonic() - last_ack < 2.0
      _publish_connected(params, published, connected)
      map_server.poll()
    except Exception as exc:
      connected = False
      _publish_connected(params, published, False)
      print("remote HUD send failed: %s" % exc, flush=True)
    time.sleep(max(0.0, 1.0 / telemetry_fps - (time.monotonic() - started)))
  _publish_connected(params, published, False)
  map_server.close()
  sock.close()


if __name__ == "__main__":
  main()

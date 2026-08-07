import json
import math
import time


STATE_FILE = "/dev/shm/carrot_navi_route.json"
STALE_TIMEOUT = 3.0

TURN_LEFT = {12, 16}
TURN_RIGHT = {13, 19}
FORK_LEFT = {7, 17, 44, 75, 76, 102, 105, 112, 115, 118}
FORK_RIGHT = {6, 43, 73, 74, 101, 104, 111, 114, 117, 123, 124}
ROTARY = set(range(131, 143))
UTURN = {14}


def _number(value, default=-1.0):
  try:
    return float(value)
  except (TypeError, ValueError):
    return default


def _first(data, names, default=None):
  if not isinstance(data, dict):
    return default
  for name in names:
    if name in data and data[name] is not None:
      return data[name]
  return default


class CarrotNaviAtc:
  """Read CarrotNavi guidance without adding a cereal dependency."""

  def __init__(self, state_file=STATE_FILE):
    self.state_file = state_file
    self.last_read = 0.0
    self.state = self.empty_state()

  @staticmethod
  def empty_state():
    return {"fresh": False, "kind": "none", "direction": 0,
            "distance": -1.0, "turn_type": -1, "text": "", "next": None}

  @classmethod
  def guidance_state(cls, guidance, fresh):
    turn_type = int(_number(_first(guidance, (
      "turn_type", "turnType", "nTBTTurnType", "tbt_turn_type")), -1))
    distance = _number(_first(guidance, (
      "distance_m", "distance", "turn_distance", "nTBTDist", "tbt_dist")), -1.0)
    text = str(_first(guidance, (
      "main_text", "text", "road_name", "szTBTMainText"), "") or "")
    kind, direction = cls.classify(turn_type, text)
    return {"fresh": fresh and kind != "none" and distance >= 0.0,
            "kind": kind, "direction": direction, "distance": distance,
            "turn_type": turn_type, "text": text, "next": None}

  def update(self):
    now = time.monotonic()
    if now - self.last_read < 0.20:
      return self.state
    self.last_read = now
    try:
      with open(self.state_file, "r") as f:
        root = json.load(f)
      stream_times = root.get("stream_updated_at_ms") or {}
      guidance_updated_at = stream_times.get("guidance_current", root.get("updated_at_ms"))
      age = time.time() - _number(guidance_updated_at, 0.0) / 1000.0
      if age < -5.0 or age > STALE_TIMEOUT:
        self.state = self.empty_state()
        return self.state
      self.state = self.guidance_state(root.get("guidance_current") or {}, True)

      # c3-style look-ahead: longitudinal control may prepare for the maneuver
      # after the current one, but steering continues to use current guidance only.
      next_updated_at = stream_times.get("guidance_next", guidance_updated_at)
      next_age = time.time() - _number(next_updated_at, 0.0) / 1000.0
      next_fresh = -5.0 <= next_age <= STALE_TIMEOUT
      next_state = self.guidance_state(root.get("guidance_next") or {}, next_fresh)
      self.state["next"] = next_state if next_state["fresh"] else None
    except (IOError, OSError, ValueError, TypeError):
      self.state = self.empty_state()
    return self.state

  @staticmethod
  def classify(turn_type, text=""):
    if turn_type in TURN_LEFT:
      return "turn", -1
    if turn_type in TURN_RIGHT:
      return "turn", 1
    if turn_type in FORK_LEFT:
      return "fork", -1
    if turn_type in FORK_RIGHT:
      return "fork", 1
    if turn_type in UTURN:
      return "uturn", -1
    if turn_type in ROTARY:
      return "rotary", 0
    lower = text.lower()
    if "유턴" in lower or "u-turn" in lower or "uturn" in lower:
      return "uturn", -1
    if any(word in lower for word in ("좌회전", "왼쪽", "left")):
      return ("fork" if any(word in lower for word in ("분기", "진출", "fork")) else "turn"), -1
    if any(word in lower for word in ("우회전", "오른쪽", "right")):
      return ("fork" if any(word in lower for word in ("분기", "진출", "fork")) else "turn"), 1
    return "none", 0

  @staticmethod
  def steering_request(state, v_ego):
    if not state["fresh"] or state["kind"] not in ("turn", "uturn"):
      return 0
    trigger_distance = max(35.0, min(70.0, v_ego * 3.0))
    if 3.0 <= state["distance"] <= trigger_distance and v_ego <= 60.0 / 3.6:
      return state["direction"]
    return 0

  @staticmethod
  def speed_limit_kph(state, target_kph=30.0, end_time=6.0, decel=1.2):
    if not state["fresh"] or state["kind"] not in ("turn", "uturn", "rotary"):
      return None
    distance = state["distance"]
    if distance < 0.0 or distance > 350.0:
      return None
    target_kph = max(30.0, min(60.0, float(target_kph)))
    end_time = max(2.0, min(12.0, float(end_time)))
    target_mps = target_kph / 3.6
    braking_distance = max(0.0, distance - target_mps * end_time)
    return min(250.0, math.sqrt(target_mps ** 2 + 2.0 * decel * braking_distance) * 3.6)

  @classmethod
  def speed_limits_kph(cls, state, target_kph=30.0, end_time=6.0, decel=1.2):
    """Return current and next-maneuver limits without allowing next guidance to steer."""
    current = cls.speed_limit_kph(state, target_kph, end_time, decel)
    next_state = state.get("next") if isinstance(state, dict) else None
    following = cls.speed_limit_kph(next_state, target_kph, end_time, decel) \
      if isinstance(next_state, dict) else None
    return current, following


class AtcForkLaneChangeController:
  """One-shot, right-exit-only lane-change gate for CarrotNavi forks."""

  MIN_DISTANCE = 20.0
  CONFIRM_FRAMES = 10  # 0.5 s at model rate

  def __init__(self):
    self.reset()

  def reset(self):
    self.event_key = None
    self.last_distance = -1.0
    self.armed_at_last_lane = False
    self.canceled = False
    self.completed = False
    self.lane_open_count = 0
    self.lane_closed_count = 0

  @staticmethod
  def _event_key(state):
    return state.get("turn_type", -1), state.get("direction", 0)

  def update(self, state, v_ego, right_lane_open, driver_cancel=False,
             lane_change_started=False, lane_change_finished=False):
    is_right_fork = (state.get("fresh", False) and state.get("kind") == "fork" and
                     state.get("direction") == 1)
    distance = float(state.get("distance", -1.0))
    if not is_right_fork or distance < self.MIN_DISTANCE:
      self.reset()
      return 0

    event_key = self._event_key(state)
    new_event = (event_key != self.event_key or
                 (self.last_distance >= 0.0 and distance > self.last_distance + 50.0))
    if new_event:
      self.reset()
      self.event_key = event_key

    self.last_distance = distance
    self.lane_open_count = self.lane_open_count + 1 if right_lane_open else 0
    self.lane_closed_count = self.lane_closed_count + 1 if not right_lane_open else 0
    if driver_cancel:
      self.canceled = True
    if lane_change_finished:
      self.completed = True

    action_distance = min(350.0, max(160.0, v_ego * 12.0))
    # Observe the current last lane only inside the actual ATC action range,
    # before allowing an exit lane that appears later to trigger a change.
    if (distance <= action_distance and self.lane_closed_count >= self.CONFIRM_FRAMES and
        not lane_change_started):
      self.armed_at_last_lane = True

    if (self.canceled or self.completed or not self.armed_at_last_lane or
        self.lane_open_count < self.CONFIRM_FRAMES or distance > action_distance):
      return 0
    return 1


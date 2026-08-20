import json
import math
import time


STATE_FILE = "/dev/shm/carrot_navi_route.json"
STALE_TIMEOUT = 3.0
MAP_CURVE_UPDATE_INTERVAL = 0.20
ROUTE_COARSE_SAMPLE_LIMIT = 256
ROUTE_WINDOW_POINT_LIMIT = 512
ROUTE_LOOKAHEAD_M = 400.0

TURN_LEFT = {12, 16, 1000}
TURN_RIGHT = {13, 19, 1001}
FORK_LEFT = {7, 17, 44, 75, 76, 102, 105, 112, 115, 118, 1002, 1006}
FORK_RIGHT = {6, 43, 73, 74, 101, 104, 111, 114, 117, 123, 124, 1003, 1007}
ROTARY = set(range(131, 143))
UTURN = {14}

# carrot-wip route-curvature table. Sharp curves are finally clamped by
# AutoCurveSpeedLowerLimit before being applied.
MAP_CURVE_BP = [0.0, 1./800., 1./670., 1./560., 1./440., 1./360., 1./265.,
                1./190., 1./135., 1./85., 1./55., 1./30., 1./25.]
MAP_CURVE_SPEED_KPH = [300.0, 150.0, 120.0, 110.0, 100.0, 90.0, 80.0,
                       70.0, 60.0, 50.0, 40.0, 15.0, 5.0]
EARTH_RADIUS_M = 6371000.0


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
    self.last_map_curve_calc = -MAP_CURVE_UPDATE_INTERVAL
    self.map_curve_speed_cache = None
    self.last_route_curvature_calc = -MAP_CURVE_UPDATE_INTERVAL
    self.route_curvature_cache = None

  @staticmethod
  def empty_state():
    return {"fresh": False, "kind": "none", "direction": 0,
            "distance": -1.0, "turn_type": -1, "text": "", "next": None,
            "route_fresh": False, "route": None, "vehicle": None,
            "speed_fresh": False, "speed": None, "off_route": False,
            "road_limit_kph": 0.0}

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
      guidance_fresh = -5.0 <= age <= STALE_TIMEOUT

      status = root.get("navigation_status") or {}
      off_route = bool(_first(status, ("off_route", "offRoute"), False))
      # Match carrot-wip control semantics: guidance_active is informational.
      # A present guidance item remains usable unless navigation reports that
      # the vehicle is actually off route.
      guidance_blocked = off_route

      self.state = self.guidance_state(
        root.get("guidance_current") or {}, guidance_fresh and not guidance_blocked)
      self.state["off_route"] = off_route
      route_updated_at = stream_times.get("route", guidance_updated_at)
      vehicle_updated_at = stream_times.get("vehicle", guidance_updated_at)
      route_age = time.time() - _number(route_updated_at, 0.0) / 1000.0
      vehicle_age = time.time() - _number(vehicle_updated_at, 0.0) / 1000.0
      self.state["route_fresh"] = (-5.0 <= route_age <= STALE_TIMEOUT and
                                   -5.0 <= vehicle_age <= STALE_TIMEOUT and
                                   not guidance_blocked)
      self.state["route"] = root.get("route")
      self.state["vehicle"] = root.get("vehicle")
      speed_state = root.get("speed") or {}
      speed_updated_at = stream_times.get("speed", root.get("updated_at_ms"))
      speed_age = time.time() - _number(speed_updated_at, 0.0) / 1000.0
      self.state["speed_fresh"] = (-5.0 <= speed_age <= STALE_TIMEOUT and
                                   isinstance(speed_state, dict) and
                                   not off_route)
      self.state["speed"] = speed_state if self.state["speed_fresh"] else None
      self.state["road_limit_kph"] = _number(_first(
        speed_state, ("road_limit_kph", "limit_speed", "roadLimitKph",
                      "section_speed_limit_kph", "sectionSpeedLimitKph")), 0.0)

      # c3-style look-ahead: longitudinal control may prepare for the maneuver
      # after the current one, but steering continues to use current guidance only.
      next_updated_at = stream_times.get("guidance_next", guidance_updated_at)
      next_age = time.time() - _number(next_updated_at, 0.0) / 1000.0
      next_fresh = -5.0 <= next_age <= STALE_TIMEOUT and not guidance_blocked
      next_state = self.guidance_state(root.get("guidance_next") or {}, next_fresh)
      self.state["next"] = next_state if next_state["fresh"] else None
    except (IOError, OSError, ValueError, TypeError):
      self.state = self.empty_state()
    return self.state

  @staticmethod
  def speed_events(state):
    """Return fresh 7714 camera/bump and section inputs for CruiseHelper."""
    result = {"camera": None, "section": None}
    if not isinstance(state, dict) or not state.get("speed_fresh", False) or state.get("off_route", False):
      return result

    speed = state.get("speed")
    if not isinstance(speed, dict):
      return result

    section = speed.get("section")
    if isinstance(section, dict):
      section_active = bool(_first(section, ("active", "section_active", "sectionActive"), False))
      section_suspended = bool(_first(section, ("suspended", "section_suspended", "sectionSuspended"), False))
      section_off_route = bool(_first(section, ("off_route", "offRoute"), False))
      section_limit = _number(_first(section, (
        "speed_limit_kph", "limit_kph", "section_speed_limit_kph", "sectionSpeedLimitKph")), 0.0)
      section_distance = _number(_first(section, (
        "remaining_distance_m", "distance_m", "section_remaining_distance_m", "sectionRemainingDistanceM")), -1.0)
      if section_active and not section_suspended and not section_off_route and section_limit > 0.0 and section_distance > 0.0:
        result["section"] = {"distance": section_distance, "limit": section_limit}

    primary = speed.get("sdi")
    if not isinstance(primary, dict):
      primary = speed
    sdi_type = int(_number(_first(primary, ("type", "sdi_type", "sdiType")), -1))
    sdi_distance = _number(_first(primary, ("distance_m", "sdi_distance_m", "sdiDistanceM")), -1.0)
    sdi_limit = _number(_first(primary, ("speed_limit_kph", "limit_kph", "sdi_speed_limit_kph", "sdiSpeedLimitKph")), 0.0)
    block_type = int(_number(_first(primary, ("block_type", "sdi_block_type", "sdiBlockType")), -1))
    block_distance = _number(_first(primary, ("block_distance_m", "sdi_block_distance_m", "sdiBlockDistanceM")), -1.0)
    block_limit = _number(_first(primary, ("block_speed_kph", "sdi_block_speed_kph", "sdiBlockSpeedKph")), 0.0)

    if result["section"] is None and block_type in (2, 3) and block_distance > 0.0:
      limit = block_limit if block_limit > 0.0 else sdi_limit
      if limit > 0.0:
        result["section"] = {"distance": block_distance, "limit": limit}

    # An explicit/legacy block section owns the speed event, matching
    # carrot-wip's section-first projection instead of applying it twice as a
    # camera and a section.
    if result["section"] is None and sdi_type >= 0 and sdi_distance > 0.0 and \
       (sdi_limit > 0.0 or sdi_type == 22):
      result["camera"] = {"type": sdi_type, "distance": sdi_distance, "limit": sdi_limit}
    elif result["section"] is None:
      secondary = speed.get("sdi_secondary")
      if isinstance(secondary, dict):
        secondary_type = int(_number(_first(secondary, ("type", "sdi_type", "sdiType")), -1))
        secondary_distance = _number(_first(secondary, ("distance_m", "sdi_distance_m", "sdiDistanceM")), -1.0)
        secondary_limit = _number(_first(secondary, ("speed_limit_kph", "limit_kph", "sdi_speed_limit_kph", "sdiSpeedLimitKph")), 0.0)
        if secondary_type == 22 and secondary_distance > 0.0:
          result["camera"] = {"type": secondary_type, "distance": secondary_distance,
                              "limit": secondary_limit}
    return result

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
  def _lat_lon(point):
    if not isinstance(point, dict):
      return None
    lat = _number(_first(point, ("lat", "latitude", "y")), 0.0)
    lon = _number(_first(point, ("lon", "lng", "longitude", "x")), 0.0)
    if abs(lat) < 1e-6 or abs(lon) < 1e-6:
      return None
    return lat, lon

  @classmethod
  def _local_route_points(cls, state, max_distance=ROUTE_LOOKAHEAD_M):
    """Return only the route window around and ahead of the vehicle.

    Long TMAP routes can contain thousands of points. A bounded coarse search
    locates the vehicle, then at most ROUTE_WINDOW_POINT_LIMIT nearby points
    are converted to metres and clipped to the requested look-ahead distance.
    """
    route = state.get("route") if isinstance(state, dict) else None
    vehicle = cls._lat_lon(state.get("vehicle")) if isinstance(state, dict) else None
    if not isinstance(route, dict) or vehicle is None:
      return None
    raw_points = route.get("polyline")
    if not isinstance(raw_points, list) or len(raw_points) < 5:
      return None

    lat0, lon0 = vehicle
    cos_lat = max(0.1, math.cos(math.radians(lat0)))
    stride = max(1, int(math.ceil(len(raw_points) / float(ROUTE_COARSE_SAMPLE_LIMIT))))
    coarse_indices = list(range(0, len(raw_points), stride))
    if coarse_indices[-1] != len(raw_points) - 1:
      coarse_indices.append(len(raw_points) - 1)

    coarse_nearest = None
    for index in coarse_indices:
      point = cls._lat_lon(raw_points[index])
      if point is None:
        continue
      lat, lon = point
      x = math.radians(lon - lon0) * EARTH_RADIUS_M * cos_lat
      y = math.radians(lat - lat0) * EARTH_RADIUS_M
      distance_sq = x * x + y * y
      if coarse_nearest is None or distance_sq < coarse_nearest[0]:
        coarse_nearest = (distance_sq, index)
    if coarse_nearest is None or coarse_nearest[0] > 100.0 ** 2:
      return None

    # Include two coarse strides behind the candidate so the exact nearest
    # segment is still present even when the polyline is densely sampled.
    coarse_index = coarse_nearest[1]
    start = max(0, coarse_index - 2 * stride - 2)
    end = min(len(raw_points), coarse_index + ROUTE_WINDOW_POINT_LIMIT)
    points = []
    for raw in raw_points[start:end]:
      point = cls._lat_lon(raw)
      if point is not None:
        lat, lon = point
        points.append((math.radians(lon - lon0) * EARTH_RADIUS_M * cos_lat,
                       math.radians(lat - lat0) * EARTH_RADIUS_M))
    if len(points) < 5:
      return None

    best = None
    for i in range(len(points) - 1):
      p0, p1 = points[i], points[i + 1]
      dx, dy = p1[0] - p0[0], p1[1] - p0[1]
      length_sq = dx * dx + dy * dy
      if length_sq < 0.25:
        continue
      ratio = max(0.0, min(1.0, -(p0[0] * dx + p0[1] * dy) / length_sq))
      projected = (p0[0] + ratio * dx, p0[1] + ratio * dy)
      distance_sq = projected[0] ** 2 + projected[1] ** 2
      if best is None or distance_sq < best[0]:
        best = (distance_sq, i, projected)
    if best is None or best[0] > 25.0 ** 2:
      return None

    _, nearest_segment, projected = best
    route_points = [projected]
    cumulative = [0.0]
    max_distance = max(25.0, min(ROUTE_LOOKAHEAD_M, float(max_distance)))
    for point in points[nearest_segment + 1:]:
      gap = math.hypot(point[0] - route_points[-1][0], point[1] - route_points[-1][1])
      if gap < 0.5:
        continue
      route_points.append(point)
      cumulative.append(cumulative[-1] + gap)
      if cumulative[-1] >= max_distance:
        break
    return (route_points, cumulative) if len(route_points) >= 5 else None

  @staticmethod
  def _interp(value, breakpoints, values):
    if value <= breakpoints[0]:
      return values[0]
    for i in range(1, len(breakpoints)):
      if value <= breakpoints[i]:
        span = breakpoints[i] - breakpoints[i - 1]
        ratio = (value - breakpoints[i - 1]) / span if span > 0.0 else 0.0
        return values[i - 1] + ratio * (values[i] - values[i - 1])
    return values[-1]

  @classmethod
  def map_curve_speed_kph(cls, state, v_ego_kph, speed_factor=0.9,
                          lower_limit_kph=30.0, decel=1.2):
    """Calculate carrot-wip-style general-road curve speed from Tmap polyline."""
    if not isinstance(state, dict) or not state.get("route_fresh", False):
      return None
    local_route = cls._local_route_points(state, 350.0)
    if local_route is None:
      return None
    points, cumulative = local_route
    if cumulative[-1] < 80.0:
      return None

    samples = []
    segment = 1
    distance = 0.0
    while distance <= min(cumulative[-1], 300.0):
      while segment < len(cumulative) and cumulative[segment] < distance:
        segment += 1
      if segment >= len(cumulative):
        break
      d0, d1 = cumulative[segment - 1], cumulative[segment]
      ratio = (distance - d0) / (d1 - d0) if d1 > d0 else 0.0
      p0, p1 = points[segment - 1], points[segment]
      samples.append((p0[0] + ratio * (p1[0] - p0[0]),
                      p0[1] + ratio * (p1[1] - p0[1])))
      distance += 10.0
    if len(samples) < 9:
      return None

    speeds = []
    sample_gap = 4
    road_limit = max(0.0, float(state.get("road_limit_kph", 0.0)))
    for i in range(len(samples) - sample_gap * 2):
      p1, p2, p3 = samples[i], samples[i + sample_gap], samples[i + sample_gap * 2]
      v1 = (p2[0] - p1[0], p2[1] - p1[1])
      v2 = (p3[0] - p2[0], p3[1] - p2[1])
      len1, len2 = math.hypot(*v1), math.hypot(*v2)
      curvature = 0.0 if len1 * len2 == 0.0 else \
                  (v1[0] * v2[1] - v1[1] * v2[0]) / (len1 * len2 * len1)
      speed = cls._interp(abs(curvature), MAP_CURVE_BP, MAP_CURVE_SPEED_KPH)
      if abs(curvature) < 0.02 and road_limit > 0.0:
        speed = max(speed, road_limit)
      speeds.append(speed)
    if not speeds:
      return None

    decel = max(0.1, min(3.0, float(decel)))
    accel_kph = decel * 3.6
    output = [0.0] * len(speeds)
    output[-1] = speeds[-1]
    wait_time = 0.0
    for i in range(len(speeds) - 2, -1, -1):
      target = speeds[i]
      next_speed = output[i + 1]
      if target < next_speed:
        wait_time = -(max(0.0, float(v_ego_kph) - target) / accel_kph)
      interval = 10.0 / (next_speed / 3.6) if next_speed > 0.0 else 0.0
      apply_time = min(interval, max(0.0, interval + wait_time))
      output[i] = min(target, next_speed + accel_kph * apply_time)
      wait_time += min(2.0, interval)

    return max(float(lower_limit_kph), output[0] * float(speed_factor))

  def cached_map_curve_speed_kph(self, state, v_ego_kph, speed_factor=0.9,
                                 lower_limit_kph=30.0, decel=1.2, now=None):
    """Run the route-polyline calculation at most 5 Hz and reuse its result."""
    now = time.monotonic() if now is None else float(now)
    if now - self.last_map_curve_calc < MAP_CURVE_UPDATE_INTERVAL:
      return self.map_curve_speed_cache

    self.last_map_curve_calc = now
    self.map_curve_speed_cache = self.map_curve_speed_kph(
      state, v_ego_kph, speed_factor, lower_limit_kph, decel)
    return self.map_curve_speed_cache

  @classmethod
  def route_curvature_profile(cls, state, sample_distances, max_curvature=0.06):
    """Return a signed TMAP curvature profile ahead of the vehicle.

    The absolute map position is deliberately discarded. TMAP's centre-line
    can be several metres away from the camera path, so lateral control uses
    only the route shape and keeps the model path as its positional anchor.
    """
    if not isinstance(state, dict) or not state.get("route_fresh", False) or \
       not state.get("fresh", False) or state.get("kind") not in ("turn", "uturn"):
      return None
    direction = int(_number(state.get("direction"), 0))
    if direction not in (-1, 1):
      return None
    requested = [max(0.0, float(distance)) for distance in sample_distances]
    if not requested:
      return None
    local_route = cls._local_route_points(state, max(80.0, max(requested) + 20.0))
    if local_route is None:
      return None
    route_points, cumulative = local_route
    if cumulative[-1] < min(25.0, max(requested) + 5.0):
      return None

    def point_at(distance):
      distance = max(0.0, min(float(distance), cumulative[-1]))
      segment = 1
      while segment < len(cumulative) and cumulative[segment] < distance:
        segment += 1
      if segment >= len(cumulative):
        return route_points[-1]
      d0, d1 = cumulative[segment - 1], cumulative[segment]
      ratio = (distance - d0) / (d1 - d0) if d1 > d0 else 0.0
      p0, p1 = route_points[segment - 1], route_points[segment]
      return (p0[0] + ratio * (p1[0] - p0[0]),
              p0[1] + ratio * (p1[1] - p0[1]))

    expected_sign = 1.0 if direction < 0 else -1.0
    max_curvature = max(0.005, min(0.08, float(max_curvature)))
    half_window = 6.0
    profile = []
    for distance in requested:
      before = point_at(max(0.0, distance - half_window))
      centre = point_at(distance)
      after = point_at(min(cumulative[-1], distance + half_window))
      a = math.hypot(centre[0] - before[0], centre[1] - before[1])
      b = math.hypot(after[0] - centre[0], after[1] - centre[1])
      c = math.hypot(after[0] - before[0], after[1] - before[1])
      cross = ((centre[0] - before[0]) * (after[1] - before[1]) -
               (centre[1] - before[1]) * (after[0] - before[0]))
      curvature = 0.0 if a * b * c < 1e-3 else 2.0 * cross / (a * b * c)
      # A route kink in the opposite direction must never make ATC steer
      # across the instructed turn. Small map noise is ignored as well.
      if curvature * expected_sign <= 0.0 or abs(curvature) < 0.002:
        curvature = 0.0
      profile.append(max(-max_curvature, min(max_curvature, curvature)))

    # One-pass smoothing removes polyline vertices while retaining the turn's
    # location. Require meaningful curvature before enabling map assistance.
    if len(profile) >= 3:
      profile = ([profile[0]] +
                 [0.25 * profile[i - 1] + 0.5 * profile[i] + 0.25 * profile[i + 1]
                  for i in range(1, len(profile) - 1)] +
                 [profile[-1]])
    return profile if max(abs(value) for value in profile) >= 0.004 else None

  @staticmethod
  def integrate_curvature_profile(curvatures, distances, max_heading=math.radians(85.0)):
    """Integrate route curvature into a vehicle-relative path and heading."""
    if len(curvatures) != len(distances) or not curvatures:
      return None
    y_values = [0.0]
    headings = [0.0]
    for i in range(1, len(curvatures)):
      ds = max(0.0, float(distances[i]) - float(distances[i - 1]))
      curvature = 0.5 * (float(curvatures[i - 1]) + float(curvatures[i]))
      mid_heading = headings[-1] + 0.5 * curvature * ds
      y_values.append(y_values[-1] + math.sin(mid_heading) * ds)
      headings.append(max(-max_heading, min(max_heading, headings[-1] + curvature * ds)))
    return y_values, headings

  def cached_route_curvature_profile(self, state, sample_distances, max_curvature=0.06, now=None):
    """Limit route geometry processing to the same 5 Hz as navigation input."""
    now = time.monotonic() if now is None else float(now)
    if now - self.last_route_curvature_calc < MAP_CURVE_UPDATE_INTERVAL:
      return self.route_curvature_cache
    self.last_route_curvature_calc = now
    self.route_curvature_cache = self.route_curvature_profile(
      state, sample_distances, max_curvature=max_curvature)
    return self.route_curvature_cache

  @staticmethod
  def steering_request(state, v_ego):
    if not state["fresh"] or state["kind"] not in ("turn", "uturn"):
      return 0
    trigger_distance = max(45.0, min(60.0, v_ego * 4.0))
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

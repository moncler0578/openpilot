import json
import math
import tempfile
import time

from selfdrive.controls.lib.carrot_navi_atc import CarrotNaviAtc, AtcForkLaneChangeController


def state_with_speed(speed, off_route=False):
  return {"speed_fresh": True, "speed": speed, "off_route": off_route}


def test_7714_primary_camera_and_bump_are_projected():
  camera = CarrotNaviAtc.speed_events(state_with_speed({
    "sdi": {"type": 1, "distance_m": 420, "speed_limit_kph": 60},
  }))
  assert camera["camera"] == {"type": 1, "distance": 420.0, "limit": 60.0}

  bump = CarrotNaviAtc.speed_events(state_with_speed({
    "sdi": {"type": 22, "distance_m": 93},
  }))
  assert bump["camera"] == {"type": 22, "distance": 93.0, "limit": 0.0}


def test_7714_explicit_and_block_sections_are_projected():
  explicit = CarrotNaviAtc.speed_events(state_with_speed({
    "section": {"active": True, "speed_limit_kph": 80, "remaining_distance_m": 2345},
  }))
  assert explicit["section"] == {"distance": 2345.0, "limit": 80.0}

  block = CarrotNaviAtc.speed_events(state_with_speed({
    "sdi": {"type": 1, "speed_limit_kph": 60, "block_type": 2,
            "block_speed_kph": 50, "block_distance_m": 390},
  }))
  assert block["section"] == {"distance": 390.0, "limit": 50.0}
  assert block["camera"] is None


def test_7714_secondary_bump_is_used_only_without_primary_camera():
  secondary = CarrotNaviAtc.speed_events(state_with_speed({
    "sdi_secondary": {"type": 22, "distance_m": 80},
  }))
  assert secondary["camera"] == {"type": 22, "distance": 80.0, "limit": 0.0}


def test_off_route_suppresses_all_7714_speed_events():
  blocked = CarrotNaviAtc.speed_events(state_with_speed({
    "sdi": {"type": 1, "distance_m": 100, "speed_limit_kph": 30},
    "section": {"active": True, "speed_limit_kph": 80, "remaining_distance_m": 1000},
  }, off_route=True))
  assert blocked == {"camera": None, "section": None}


def test_update_off_route_blocks_turn_route_and_speed_control():
  now_ms = int(time.time() * 1000)
  root = {
    "stream_updated_at_ms": {
      "guidance_current": now_ms, "guidance_next": now_ms,
      "route": now_ms, "vehicle": now_ms, "speed": now_ms,
      "navigation_status": now_ms,
    },
    "guidance_current": {"turn_type": 12, "distance_m": 80},
    "guidance_next": {"turn_type": 13, "distance_m": 400},
    "route": {"polyline": []},
    "vehicle": {"lat": 37.5, "lon": 127.1},
    "speed": {"sdi": {"type": 1, "distance_m": 100, "speed_limit_kph": 30}},
    "navigation_status": {"guidance_active": True, "off_route": True},
  }
  with tempfile.NamedTemporaryFile(mode="w+") as state_file:
    json.dump(root, state_file)
    state_file.flush()
    state = CarrotNaviAtc(state_file.name).update()

  assert state["off_route"]
  assert not state["fresh"]
  assert state["next"] is None
  assert not state["route_fresh"]
  assert not state["speed_fresh"]


def test_guidance_inactive_does_not_block_present_guidance_or_speed_events():
  now_ms = int(time.time() * 1000)
  root = {
    "stream_updated_at_ms": {
      "guidance_current": now_ms, "guidance_next": now_ms,
      "route": now_ms, "vehicle": now_ms, "speed": now_ms,
      "navigation_status": now_ms,
    },
    "guidance_current": {"turn_type": 12, "distance_m": 80},
    "guidance_next": {"turn_type": 13, "distance_m": 400},
    "route": {"polyline": []},
    "vehicle": {"lat": 37.5, "lon": 127.1},
    "speed": {"sdi": {"type": 1, "distance_m": 100, "speed_limit_kph": 30}},
    "navigation_status": {"guidance_active": False, "off_route": False},
  }
  with tempfile.NamedTemporaryFile(mode="w+") as state_file:
    json.dump(root, state_file)
    state_file.flush()
    state = CarrotNaviAtc(state_file.name).update()

  assert not state["off_route"]
  assert state["fresh"]
  assert state["next"] is not None
  assert state["route_fresh"]
  assert state["speed_fresh"]
  assert CarrotNaviAtc.speed_events(state)["camera"] == {
    "type": 1, "distance": 100.0, "limit": 30.0,
  }


def test_active_route_ignores_transient_guidance_inactive_status():
  now_ms = int(time.time() * 1000)
  root = {
    "stream_updated_at_ms": {
      "guidance_current": now_ms, "guidance_next": now_ms,
      "route": now_ms, "vehicle": now_ms,
    },
    "guidance_current": {"turn_type": 12, "distance_m": 35},
    "guidance_next": {"turn_type": 13, "distance_m": 300},
    "route": {"remain_distance_m": 1200, "remain_time_sec": 180, "polyline": []},
    "vehicle": {"lat": 37.5, "lon": 127.1},
    "navigation_status": {"guidance_active": False, "off_route": False},
  }
  with tempfile.NamedTemporaryFile(mode="w+") as state_file:
    json.dump(root, state_file)
    state_file.flush()
    state = CarrotNaviAtc(state_file.name).update()

  assert state["fresh"]
  assert state["next"] is not None
  assert state["route_fresh"]
  assert CarrotNaviAtc.steering_request(state, 30.0 / 3.6) == -1


def test_extended_carrot_turn_types_are_classified():
  assert CarrotNaviAtc.classify(1000) == ("turn", -1)
  assert CarrotNaviAtc.classify(1001) == ("turn", 1)
  assert CarrotNaviAtc.classify(1002) == ("fork", -1)
  assert CarrotNaviAtc.classify(1003) == ("fork", 1)
  assert CarrotNaviAtc.classify(1006) == ("fork", -1)
  assert CarrotNaviAtc.classify(1007) == ("fork", 1)


def test_turn_steering_request_keeps_speed_distance_and_freshness_gates():
  active = {"fresh": True, "kind": "turn", "direction": 1, "distance": 45.0}
  assert CarrotNaviAtc.steering_request(active, 30.0 / 3.6) == 1
  assert CarrotNaviAtc.steering_request(dict(active, distance=45.1), 30.0 / 3.6) == 0
  assert CarrotNaviAtc.steering_request(dict(active, distance=55.0), 50.0 / 3.6) == 1
  assert CarrotNaviAtc.steering_request(dict(active, distance=60.0), 60.0 / 3.6) == 1
  assert CarrotNaviAtc.steering_request(dict(active, distance=60.1), 60.0 / 3.6) == 0
  assert CarrotNaviAtc.steering_request(dict(active, distance=2.0), 30.0 / 3.6) == 0
  assert CarrotNaviAtc.steering_request(active, 61.0 / 3.6) == 0
  assert CarrotNaviAtc.steering_request(dict(active, fresh=False), 30.0 / 3.6) == 0


def test_next_maneuver_speed_limit_is_independent_from_steering():
  current = {"fresh": True, "kind": "fork", "direction": 1,
             "distance": 120.0, "turn_type": 6, "text": "exit"}
  following = {"fresh": True, "kind": "turn", "direction": 1,
               "distance": 300.0, "turn_type": 13, "text": "right"}
  current["next"] = following

  current_limit, next_limit = CarrotNaviAtc.speed_limits_kph(current, 30.0, 6.0)
  assert current_limit is None
  assert next_limit is not None
  assert CarrotNaviAtc.steering_request(current, 10.0) == 0


def test_stale_next_maneuver_has_no_speed_limit():
  current = {"fresh": True, "kind": "turn", "direction": -1,
             "distance": 100.0, "turn_type": 12, "text": "left", "next": None}
  current_limit, next_limit = CarrotNaviAtc.speed_limits_kph(current)
  assert current_limit is not None
  assert next_limit is None


def test_map_curve_speed_calculation_is_cached_at_5hz():
  class CountingNavi(CarrotNaviAtc):
    def __init__(self):
      super().__init__()
      self.calls = 0

    def map_curve_speed_kph(self, *_args, **_kwargs):
      self.calls += 1
      return 40.0 + self.calls

  navi = CountingNavi()

  assert navi.cached_map_curve_speed_kph({}, 60.0, now=0.0) == 41.0
  assert navi.cached_map_curve_speed_kph({}, 60.0, now=0.10) == 41.0
  assert navi.calls == 1

  assert navi.cached_map_curve_speed_kph({}, 60.0, now=0.21) == 42.0
  assert navi.calls == 2


def route_turn_state(left=True):
  lat0, lon0 = 37.5, 127.1
  cos_lat = math.cos(math.radians(lat0))
  local_points = [(float(x), 0.0) for x in range(0, 21, 4)]
  direction = -1 if left else 1
  for degree in range(-90, 1, 10):
    angle = math.radians(degree)
    x = 20.0 + 20.0 * math.cos(angle)
    y = 20.0 + 20.0 * math.sin(angle)
    local_points.append((x, y if left else -y))
  polyline = [{
    "lat": lat0 + math.degrees(y / 6371000.0),
    "lon": lon0 + math.degrees(x / (6371000.0 * cos_lat)),
  } for x, y in local_points]
  return {
    "fresh": True, "route_fresh": True, "kind": "turn",
    "direction": direction, "distance": 20.0,
    "vehicle": {"lat": lat0, "lon": lon0},
    "route": {"polyline": polyline},
  }


def test_route_curvature_profile_matches_guidance_direction():
  distances = [float(x) for x in range(0, 61, 2)]
  left = CarrotNaviAtc.route_curvature_profile(route_turn_state(True), distances)
  right = CarrotNaviAtc.route_curvature_profile(route_turn_state(False), distances)

  assert left is not None and max(left) > 0.02 and min(left) >= 0.0
  assert right is not None and min(right) < -0.02 and max(right) <= 0.0


def test_route_curvature_rejects_stale_or_direction_mismatched_route():
  distances = [float(x) for x in range(0, 61, 2)]
  stale = route_turn_state(True)
  stale["route_fresh"] = False
  assert CarrotNaviAtc.route_curvature_profile(stale, distances) is None

  mismatched = route_turn_state(True)
  mismatched["direction"] = 1
  assert CarrotNaviAtc.route_curvature_profile(mismatched, distances) is None


def test_integrated_route_curvature_builds_smooth_relative_path():
  distances = [float(x) for x in range(0, 51, 5)]
  integrated = CarrotNaviAtc.integrate_curvature_profile([0.02] * len(distances), distances)
  assert integrated is not None
  y_values, headings = integrated
  assert y_values[0] == 0.0 and headings[0] == 0.0
  assert all(b >= a for a, b in zip(y_values, y_values[1:]))
  assert all(b >= a for a, b in zip(headings, headings[1:]))
  assert headings[-1] <= math.radians(85.0)


def test_route_curvature_profile_is_cached_at_5hz():
  class CountingNavi(CarrotNaviAtc):
    def __init__(self):
      super().__init__()
      self.calls = 0

    def route_curvature_profile(self, *_args, **_kwargs):
      self.calls += 1
      return [float(self.calls)]

  navi = CountingNavi()
  assert navi.cached_route_curvature_profile({}, [0.0], now=0.0) == [1.0]
  assert navi.cached_route_curvature_profile({}, [0.0], now=0.1) == [1.0]
  assert navi.calls == 1
  assert navi.cached_route_curvature_profile({}, [0.0], now=0.21) == [2.0]
  assert navi.calls == 2


def test_long_route_window_finds_vehicle_far_from_route_start():
  lat0, lon0 = 37.5, 127.1
  cos_lat = math.cos(math.radians(lat0))
  vehicle_x = 6000.0
  local_points = [(float(x), 0.0) for x in range(0, 6021, 2)]
  for degree in range(-90, 1, 5):
    angle = math.radians(degree)
    local_points.append((6020.0 + 20.0 * math.cos(angle),
                         20.0 + 20.0 * math.sin(angle)))
  local_points.extend((6040.0, float(y)) for y in range(22, 402, 2))

  def geo_point(x, y):
    return {
      "lat": lat0 + math.degrees(y / 6371000.0),
      "lon": lon0 + math.degrees(x / (6371000.0 * cos_lat)),
    }

  state = {
    "fresh": True, "route_fresh": True, "kind": "turn",
    "direction": -1, "distance": 20.0,
    "vehicle": geo_point(vehicle_x, 0.0),
    "route": {"polyline": [geo_point(x, y) for x, y in local_points]},
  }
  distances = [float(x) for x in range(0, 81, 2)]
  profile = CarrotNaviAtc.route_curvature_profile(state, distances, 0.03)
  assert profile is not None and max(profile) > 0.02 and min(profile) >= 0.0


def fork_state(distance=300.0, direction=1, turn_type=6, text="right exit"):
  return {"fresh": True, "kind": "fork", "direction": direction,
          "distance": distance, "turn_type": turn_type, "text": text}


def confirm(controller, state, v_ego, right_lane_open, **kwargs):
  result = 0
  for _ in range(controller.CONFIRM_FRAMES):
    result = controller.update(state, v_ego, right_lane_open, **kwargs)
  return result


def test_right_exit_arms_closed_then_requests_when_lane_opens():
  controller = AtcForkLaneChangeController()
  assert confirm(controller, fork_state(340), 30.0, right_lane_open=False) == 0
  assert confirm(controller, fork_state(320), 30.0, right_lane_open=True) == 1


def test_does_not_start_when_exit_lane_was_already_open():
  controller = AtcForkLaneChangeController()
  assert confirm(controller, fork_state(320), 30.0, right_lane_open=True) == 0


def test_left_fork_is_not_automatic():
  controller = AtcForkLaneChangeController()
  assert confirm(controller, fork_state(direction=-1), 30.0, right_lane_open=False) == 0
  assert confirm(controller, fork_state(250, direction=-1), 30.0, right_lane_open=True) == 0


def test_driver_cancel_latches_for_current_exit():
  controller = AtcForkLaneChangeController()
  confirm(controller, fork_state(340), 30.0, right_lane_open=False)
  assert confirm(controller, fork_state(320), 30.0, right_lane_open=True, driver_cancel=True) == 0
  assert controller.update(fork_state(280), 30.0, right_lane_open=True) == 0


def test_completed_change_does_not_repeat():
  controller = AtcForkLaneChangeController()
  confirm(controller, fork_state(340), 30.0, right_lane_open=False)
  assert confirm(controller, fork_state(320), 30.0, right_lane_open=True) == 1
  assert controller.update(fork_state(300), 30.0, right_lane_open=True,
                           lane_change_finished=True) == 0
  assert controller.update(fork_state(250), 30.0, right_lane_open=True) == 0


def test_bsd_style_block_can_retry_while_event_is_armed():
  controller = AtcForkLaneChangeController()
  confirm(controller, fork_state(340), 30.0, right_lane_open=False)
  # The controller keeps requesting while the regular lane-change FSM waits
  # for BSD to clear.
  assert confirm(controller, fork_state(320), 30.0, right_lane_open=True) == 1
  assert controller.update(fork_state(300), 30.0, right_lane_open=True) == 1


def test_new_exit_resets_completed_latch():
  controller = AtcForkLaneChangeController()
  confirm(controller, fork_state(340), 30.0, right_lane_open=False)
  controller.update(fork_state(320), 30.0, right_lane_open=True, lane_change_finished=True)
  assert confirm(controller, fork_state(340, turn_type=43), 30.0, right_lane_open=False) == 0
  assert confirm(controller, fork_state(300, turn_type=43), 30.0, right_lane_open=True) == 1


def test_single_frame_lane_open_noise_does_not_request():
  controller = AtcForkLaneChangeController()
  confirm(controller, fork_state(340), 30.0, right_lane_open=False)
  assert controller.update(fork_state(320), 30.0, right_lane_open=True) == 0
  assert controller.update(fork_state(319), 30.0, right_lane_open=False) == 0


def test_does_not_arm_before_speed_based_action_range():
  controller = AtcForkLaneChangeController()
  # At 100 km/h (27.8 m/s), the action range is about 333 m.
  confirm(controller, fork_state(400), 27.8, right_lane_open=False)
  assert confirm(controller, fork_state(320), 27.8, right_lane_open=True) == 0


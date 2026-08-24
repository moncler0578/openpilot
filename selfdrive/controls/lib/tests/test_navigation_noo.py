import json
import math
import tempfile
import time
from types import SimpleNamespace

from selfdrive.controls.lib.navigation_route import NavigationRouteData
from selfdrive.controls.lib.navigation_noo import NavigationLaneChangeController


def state_with_speed(speed, off_route=False):
  return {"speed_fresh": True, "speed": speed, "off_route": off_route}


def test_7714_primary_camera_and_bump_are_projected():
  camera = NavigationRouteData.speed_events(state_with_speed({
    "sdi": {"type": 1, "distance_m": 420, "speed_limit_kph": 60},
  }))
  assert camera["camera"] == {"type": 1, "distance": 420.0, "limit": 60.0}

  bump = NavigationRouteData.speed_events(state_with_speed({
    "sdi": {"type": 22, "distance_m": 93},
  }))
  assert bump["camera"] == {"type": 22, "distance": 93.0, "limit": 0.0}


def test_7714_explicit_and_block_sections_are_projected():
  explicit = NavigationRouteData.speed_events(state_with_speed({
    "section": {"active": True, "speed_limit_kph": 80, "remaining_distance_m": 2345},
  }))
  assert explicit["section"] == {"distance": 2345.0, "limit": 80.0}

  block = NavigationRouteData.speed_events(state_with_speed({
    "sdi": {"type": 1, "speed_limit_kph": 60, "block_type": 2,
            "block_speed_kph": 50, "block_distance_m": 390},
  }))
  assert block["section"] == {"distance": 390.0, "limit": 50.0}
  assert block["camera"] is None


def test_7714_secondary_bump_is_used_only_without_primary_camera():
  secondary = NavigationRouteData.speed_events(state_with_speed({
    "sdi_secondary": {"type": 22, "distance_m": 80},
  }))
  assert secondary["camera"] == {"type": 22, "distance": 80.0, "limit": 0.0}


def test_off_route_suppresses_all_7714_speed_events():
  blocked = NavigationRouteData.speed_events(state_with_speed({
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
    state = NavigationRouteData(state_file.name).update()

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
    state = NavigationRouteData(state_file.name).update()

  assert not state["off_route"]
  assert state["fresh"]
  assert state["next"] is not None
  assert state["route_fresh"]
  assert state["speed_fresh"]
  assert NavigationRouteData.speed_events(state)["camera"] == {
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
    state = NavigationRouteData(state_file.name).update()

  assert state["fresh"]
  assert state["next"] is not None
  assert state["route_fresh"]
  assert NavigationRouteData.steering_request(state, 30.0 / 3.6) == -1


def test_extended_carrot_turn_types_are_classified():
  assert NavigationRouteData.classify(1000) == ("turn", -1)
  assert NavigationRouteData.classify(1001) == ("turn", 1)
  assert NavigationRouteData.classify(1002) == ("fork", -1)
  assert NavigationRouteData.classify(1003) == ("fork", 1)
  assert NavigationRouteData.classify(1006) == ("fork", -1)
  assert NavigationRouteData.classify(1007) == ("fork", 1)


def test_turn_steering_request_keeps_speed_distance_and_freshness_gates():
  active = {"fresh": True, "kind": "turn", "direction": 1, "distance": 45.0}
  assert NavigationRouteData.steering_request(active, 30.0 / 3.6) == 1
  assert NavigationRouteData.steering_request(dict(active, distance=45.1), 30.0 / 3.6) == 0
  assert NavigationRouteData.steering_request(dict(active, distance=55.0), 50.0 / 3.6) == 1
  assert NavigationRouteData.steering_request(dict(active, distance=60.0), 60.0 / 3.6) == 1
  assert NavigationRouteData.steering_request(dict(active, distance=60.1), 60.0 / 3.6) == 0
  assert NavigationRouteData.steering_request(dict(active, distance=2.0), 30.0 / 3.6) == 0
  assert NavigationRouteData.steering_request(active, 61.0 / 3.6) == 0
  assert NavigationRouteData.steering_request(dict(active, fresh=False), 30.0 / 3.6) == 0


def test_next_maneuver_speed_limit_is_independent_from_steering():
  current = {"fresh": True, "kind": "fork", "direction": 1,
             "distance": 120.0, "turn_type": 6, "text": "exit"}
  following = {"fresh": True, "kind": "turn", "direction": 1,
               "distance": 300.0, "turn_type": 13, "text": "right"}
  current["next"] = following

  current_limit, next_limit = NavigationRouteData.speed_limits_kph(current, 30.0, 6.0)
  assert current_limit is None
  assert next_limit is not None
  assert NavigationRouteData.steering_request(current, 10.0) == 0


def test_stale_next_maneuver_has_no_speed_limit():
  current = {"fresh": True, "kind": "turn", "direction": -1,
             "distance": 100.0, "turn_type": 12, "text": "left", "next": None}
  current_limit, next_limit = NavigationRouteData.speed_limits_kph(current)
  assert current_limit is not None
  assert next_limit is None


def test_map_curve_speed_calculation_is_cached_at_5hz():
  class CountingNavi(NavigationRouteData):
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


def test_map_steering_blend_is_strong_only_at_low_speed():
  assert NavigationRouteData.map_steering_blend(0.0) == 0.60
  assert NavigationRouteData.map_steering_blend(20.0) == 0.60
  assert math.isclose(NavigationRouteData.map_steering_blend(25.0), 0.50)
  assert math.isclose(NavigationRouteData.map_steering_blend(35.0), 0.30)
  assert math.isclose(NavigationRouteData.map_steering_blend(45.0), 0.10)
  assert NavigationRouteData.map_steering_blend(50.0) == 0.0
  assert NavigationRouteData.map_steering_blend(80.0) == 0.0


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
  left = NavigationRouteData.route_curvature_profile(route_turn_state(True), distances)
  right = NavigationRouteData.route_curvature_profile(route_turn_state(False), distances)

  assert left is not None and max(left) > 0.02 and min(left) >= 0.0
  assert right is not None and min(right) < -0.02 and max(right) <= 0.0


def test_route_curvature_rejects_stale_or_direction_mismatched_route():
  distances = [float(x) for x in range(0, 61, 2)]
  stale = route_turn_state(True)
  stale["route_fresh"] = False
  assert NavigationRouteData.route_curvature_profile(stale, distances) is None

  mismatched = route_turn_state(True)
  mismatched["direction"] = 1
  assert NavigationRouteData.route_curvature_profile(mismatched, distances) is None


def test_integrated_route_curvature_builds_smooth_relative_path():
  distances = [float(x) for x in range(0, 51, 5)]
  integrated = NavigationRouteData.integrate_curvature_profile([0.02] * len(distances), distances)
  assert integrated is not None
  y_values, headings = integrated
  assert y_values[0] == 0.0 and headings[0] == 0.0
  assert all(b >= a for a, b in zip(y_values, y_values[1:]))
  assert all(b >= a for a, b in zip(headings, headings[1:]))
  assert headings[-1] <= math.radians(85.0)


def test_route_curvature_profile_is_cached_at_5hz():
  class CountingNavi(NavigationRouteData):
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
  profile = NavigationRouteData.route_curvature_profile(state, distances, 0.03)
  assert profile is not None and max(profile) > 0.02 and min(profile) >= 0.0


def model_line(y):
  return SimpleNamespace(x=[0.0, 10.0, 20.0, 30.0], y=[y] * 4)


def lane_model(left_edge, right_edge):
  return SimpleNamespace(
    laneLines=[model_line(5.4), model_line(1.8), model_line(-1.8), model_line(-5.4)],
    laneLineProbs=[0.8, 0.9, 0.9, 0.8],
    roadEdges=[model_line(left_edge), model_line(right_edge)],
    roadEdgeStds=[0.1, 0.1],
  )


def noo_state(available, distance=400.0, direction=1):
  return {
    "fresh": True, "route_fresh": True, "lane_fresh": True,
    "off_route": False, "turn_type": 6, "direction": direction,
    "distance": distance,
    "lane_current": {"count": len(available), "current_lane": 1,
                     "available": available, "distance_m": distance},
  }


def confirm_noo(controller, state, ego_lane, left_open=True, right_open=True, **kwargs):
  result = 0
  for _ in range(controller.CONFIRM_FRAMES):
    result = controller.update(state, ego_lane, 25.0, left_open, right_open, **kwargs)
  return result


def test_camera_lane_position_uses_model_edges_not_tmap_lane_number():
  left = NavigationRouteData.camera_lane_position(lane_model(2.5, -9.0))
  middle = NavigationRouteData.camera_lane_position(lane_model(5.4, -5.4))
  right = NavigationRouteData.camera_lane_position(lane_model(9.0, -2.5))
  assert (left["count"], left["current"]) == (3, 1)
  assert (middle["count"], middle["current"]) == (3, 2)
  assert (right["count"], right["current"]) == (3, 3)


def test_noo_requests_only_the_nearest_route_compatible_lane():
  controller = NavigationLaneChangeController()
  ego = {"count": 3, "current": 2, "confidence": 0.9}
  assert confirm_noo(controller, noo_state([0, 0, 1]), ego) == 1
  assert controller.current_lane == 2 and controller.target_lane == 3


def test_fork_lane_change_requires_noo_lane_guidance():
  controller = NavigationLaneChangeController()
  ego = {"count": 3, "current": 2, "confidence": 0.9}
  state = noo_state([0, 0, 1])
  state["lane_fresh"] = False
  state["lane_current"] = None
  # A bare fork direction must not guess which lane the vehicle occupies.
  assert confirm_noo(controller, state, ego) == 0
  state = noo_state([0, 0, 1])
  assert confirm_noo(controller, state, ego) == 1


def test_noo_rejects_count_mismatch_opposite_plan_and_late_change():
  controller = NavigationLaneChangeController()
  ego = {"count": 3, "current": 2, "confidence": 0.9}
  assert controller.update(noo_state([0, 1]), ego, 25.0, True, True) == 0
  assert controller.update(noo_state([1, 0, 0], direction=1), ego, 25.0, True, True) == 0
  assert confirm_noo(controller, noo_state([0, 0, 1], distance=50.0), ego) == 0


def test_noo_waits_for_camera_confirmation_between_two_lane_changes():
  controller = NavigationLaneChangeController()
  state = noo_state([0, 0, 1], distance=400.0)
  lane1 = {"count": 3, "current": 1, "confidence": 0.9}
  lane2 = {"count": 3, "current": 2, "confidence": 0.9}
  lane3 = {"count": 3, "current": 3, "confidence": 0.9}
  assert confirm_noo(controller, state, lane1) == 1
  assert controller.update(state, lane1, 25.0, True, True, lane_change_started=True) == 1
  assert controller.update(state, lane1, 25.0, True, True, lane_change_finished=True) == 1
  assert controller.update(state, lane1, 25.0, True, True) == 0
  assert controller.update(state, lane2, 25.0, True, True) == 0
  assert confirm_noo(controller, state, lane2) == 1
  controller.update(state, lane2, 25.0, True, True, lane_change_finished=True)
  controller.update(state, lane2, 25.0, True, True)
  assert controller.update(state, lane3, 25.0, True, True) == 0
  assert controller.completed


def test_noo_driver_cancel_latches_until_the_maneuver_changes():
  controller = NavigationLaneChangeController()
  state = noo_state([0, 0, 1])
  ego = {"count": 3, "current": 2, "confidence": 0.9}
  assert controller.update(state, ego, 25.0, True, True, driver_cancel=True) == 0
  assert confirm_noo(controller, state, ego) == 0
  next_state = noo_state([0, 0, 1])
  next_state["turn_type"] = 43
  assert confirm_noo(controller, next_state, ego) == 1


def test_noo_uses_fresh_lane_ahead_after_current_lane_is_satisfied():
  controller = NavigationLaneChangeController()
  state = noo_state([1, 1, 1], distance=180.0)
  state["lane_ahead_fresh"] = True
  state["lane_ahead"] = {"count": 3, "available": [0, 0, 1], "distance_m": 420.0}
  state["next"] = {"fresh": True, "direction": 1, "distance": 420.0, "turn_type": 43}
  ego = {"count": 3, "current": 2, "confidence": 0.9}
  assert confirm_noo(controller, state, ego) == 1
  assert controller.target_lane == 3



def shoulder_model(left_edge, right_edge, edge_std=0.1):
  return SimpleNamespace(
    laneLines=[model_line(5.4), model_line(1.8), model_line(-1.8), model_line(-5.4)],
    laneLineProbs=[0.8, 0.9, 0.9, 0.8],
    roadEdges=[model_line(left_edge), model_line(right_edge)],
    roadEdgeStds=[edge_std, edge_std],
  )


def test_paved_shoulder_does_not_block_the_lane_plan():
  # 3 lanes + a 3.0 m right shoulder: the camera counts 4, TMAP says 3.
  ego = NavigationRouteData.camera_lane_position(shoulder_model(5.4, -8.4))
  assert ego["count"] == 4 and ego["current"] == 2
  plan = NavigationLaneChangeController.lane_plan(noo_state([0, 0, 1]), ego)
  assert plan is not None
  assert (plan["count"], plan["current"], plan["target"], plan["direction"]) == (3, 2, 3, 1)


def test_wide_median_shifts_the_resolved_lane_index():
  # 3 lanes + a 2.2 m left median: the phantom lane is on the left.
  ego = NavigationRouteData.camera_lane_position(shoulder_model(7.6, -5.4))
  assert ego["count"] == 4 and ego["current"] == 3
  plan = NavigationLaneChangeController.lane_plan(noo_state([0, 0, 1]), ego)
  assert plan is not None
  assert (plan["current"], plan["target"], plan["direction"]) == (2, 3, 1)


def test_extra_whole_lane_still_fails_closed():
  # A full extra lane on the right is not a shoulder, so the disagreement with
  # TMAP is real and NOO must not guess.
  ego = NavigationRouteData.camera_lane_position(shoulder_model(5.4, -9.0))
  assert ego["count"] == 4
  assert NavigationLaneChangeController.lane_plan(noo_state([0, 0, 1]), ego) is None


def test_two_extra_camera_lanes_fail_closed():
  ego = NavigationRouteData.camera_lane_position(shoulder_model(9.0, -8.4))
  assert ego["count"] == 5
  assert NavigationLaneChangeController.lane_plan(noo_state([0, 0, 1]), ego) is None


def test_uncertain_road_edges_are_rejected_by_distance_not_probability():
  assert NavigationRouteData.camera_lane_position(shoulder_model(5.4, -5.4, 0.4)) is not None
  assert NavigationRouteData.camera_lane_position(shoulder_model(5.4, -5.4, 0.7)) is None


def test_lane_change_wait_times_out_without_camera_data():
  controller = NavigationLaneChangeController()
  state = noo_state([0, 0, 1])
  ego = NavigationRouteData.camera_lane_position(lane_model(5.4, -5.4))
  assert confirm_noo(controller, state, ego) == 1
  controller.update(state, ego, 25.0, True, True, lane_change_started=True)
  controller.update(state, ego, 25.0, True, True, lane_change_finished=True)
  # Camera geometry drops out right after the lane change.
  for _ in range(NavigationLaneChangeController.LANE_UPDATE_FRAMES + 1):
    assert controller.update(state, None, 25.0, True, True) == 0
  assert controller.canceled
  assert controller.waiting_from_lane == 0


def test_median_and_shoulder_together_pick_the_rounded_up_side():
  # 좌 1.6 m 중앙분리대(차로폭의 0.44 → 세지 않음)와 우 3.0 m 갓길(0.83 →
  # 차로 하나로 셈)이 함께 있는 흔한 고속도로. 유령차로는 갓길 쪽 하나뿐이므로
  # 자차 인덱스는 그대로 두고 차로수만 맞춘다.
  ego = NavigationRouteData.camera_lane_position(shoulder_model(7.0, -8.4))
  assert ego["count"] == 4
  plan = NavigationLaneChangeController.lane_plan(noo_state([0, 0, 1]), ego)
  assert plan is not None
  assert (plan["count"], plan["current"], plan["target"]) == (3, 2, 3)


def test_both_sides_rounded_up_fail_closed():
  # 양쪽 다 반올림에서 올라가면(좌 2.6 m 중앙분리대 / 우 3.0 m 갓길) 어느 쪽이
  # 유령차로인지 정할 수 없어 판정을 포기한다.
  ego = NavigationRouteData.camera_lane_position(shoulder_model(8.0, -8.4))
  assert ego["count"] == 5
  assert NavigationLaneChangeController.lane_plan(noo_state([0, 0, 1]), ego) is None

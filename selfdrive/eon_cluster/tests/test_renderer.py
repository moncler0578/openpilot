import json
import time

from selfdrive.eon_cluster.renderer import HudRenderer, read_navi_state


def test_stale_navi_state_is_rejected(tmp_path):
  path = tmp_path / "state.json"
  path.write_text(json.dumps({"updated_at_ms": int(time.time() * 1000) - 40000}))
  assert read_navi_state(str(path)) == {}


def test_fresh_navi_state_is_loaded(tmp_path):
  path = tmp_path / "state.json"
  path.write_text(json.dumps({"updated_at_ms": int(time.time() * 1000), "route": {"remain_distance_m": 1000}}))
  assert read_navi_state(str(path))["route"]["remain_distance_m"] == 1000


def test_portrait_jpeg_geometry():
  renderer = HudRenderer(1920, 462, 50)
  frame = renderer.render(72.0, 90.0, True, {})
  jpeg = renderer.encode_portrait_jpeg(frame)
  from PIL import Image
  import io
  with Image.open(io.BytesIO(jpeg)) as image:
    assert image.size == (462, 1920)


def test_lightweight_scene_uses_camera_free_vector_path():
  renderer = HudRenderer(1920, 462, 50)
  scene = {
    "path": [(0.0, 0.0), (20.0, 0.1), (50.0, 0.3), (100.0, 0.8)],
    "lanes": [
      {"points": [(0.0, -1.8), (50.0, -1.7), (100.0, -1.5)], "probability": 0.9},
      {"points": [(0.0, 1.8), (50.0, 1.7), (100.0, 1.5)], "probability": 0.9},
    ],
    "edges": [],
    "leads": [{"distance": 35.0, "lateral": 0.1, "relative_speed": -2.0}],
  }
  frame = renderer.render(82.0, 90.0, True, {"speed": {"road_limit_kph": 80}}, scene)
  assert frame.size == (1920, 462)
  assert (25, 104, 205) in set(frame.getdata())


def test_cluster_overlays_and_swapped_layout_render():
  renderer = HudRenderer(1920, 462, 50)
  scene = {
    "driving_mode": 1,
    "tpms": {"fl": 30.0, "fr": 35.0, "rl": 36.0, "rr": 37.0},
    "panel_layout": 1,
    "parked": True,
    "trip_report": {"duration_s": 3600, "distance_m": 42000, "average_speed_kph": 42, "max_speed_kph": 101},
  }
  frame = renderer.render(82.0, 90.0, True, {}, scene)
  colors = set(frame.getdata())
  assert frame.size == (1920, 462)
  assert (40, 210, 125) in colors
  assert any(red > 200 and green < 80 and blue < 80 for red, green, blue in colors)

  scene["alert"] = {"text1": "TAKE CONTROL", "text2": "System Unresponsive", "status": "critical"}
  alert_frame = renderer.render(82.0, 90.0, True, {}, scene)
  colors = set(alert_frame.getdata())
  assert (225, 55, 55) in colors

from types import SimpleNamespace

from selfdrive.eon_cluster.scene import extract_driving_scene, final_lateral_path


def polyline(xs, ys):
  return SimpleNamespace(x=xs, y=ys)


def test_final_lateral_path_uses_planner_y_and_model_time_geometry():
  lateral = SimpleNamespace(mpcSolutionValid=True, dPathPoints=[0.0, 0.5, 2.0])
  model = SimpleNamespace(
    position=SimpleNamespace(z=[0.0, 0.1, 0.2]),
    velocity=SimpleNamespace(x=[10.0], y=[0.0], z=[0.0]),
  )
  assert final_lateral_path(lateral, model, [0.0, 0.1, 0.4]) == [
    [0.0, 0.0, 0.0], [1.0, 0.5, 0.1], [4.0, 2.0, 0.2],
  ]


def test_final_lateral_path_rejects_invalid_mpc_and_missing_velocity():
  model = SimpleNamespace(position=SimpleNamespace(z=[]),
                          velocity=SimpleNamespace(x=[], y=[], z=[]))
  invalid = SimpleNamespace(mpcSolutionValid=False, dPathPoints=[0.0, 1.0])
  missing_speed = SimpleNamespace(mpcSolutionValid=True, dPathPoints=[0.0, 1.0])
  assert final_lateral_path(invalid, model, [0.0, 1.0]) == []
  assert final_lateral_path(missing_speed, model, [0.0, 1.0]) == []


def test_extract_driving_scene_from_model_and_radar():
  model = SimpleNamespace(
    position=polyline([0.0, 20.0, 60.0], [0.0, 0.2, 0.6]),
    laneLines=[
      polyline([0.0, 40.0, 80.0], [-1.8, -1.7, -1.5]),
      polyline([0.0, 40.0, 80.0], [1.8, 1.7, 1.5]),
    ],
    laneLineProbs=[0.9, 0.8],
    roadEdges=[],
    roadEdgeStds=[],
  )
  radar = SimpleNamespace(
    leadOne=SimpleNamespace(status=True, dRel=32.0, yRel=0.3, vRel=-1.5),
    leadTwo=SimpleNamespace(status=True, dRel=58.0, yRel=-0.2, vRel=2.0),
  )
  scene = extract_driving_scene(model, radar)
  assert len(scene["path"]) == 3
  assert len(scene["lanes"]) == 2
  assert scene["lanes"][0]["probability"] == 0.9
  assert scene["leads"] == [
    {"distance": 32.0, "lateral": 0.3, "relative_speed": -1.5},
    {"distance": 58.0, "lateral": -0.2, "relative_speed": 2.0},
  ]


def test_lane_geometry_is_smoothed_and_low_confidence_is_hidden():
  xs = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
  noisy = [-1.8, -1.45, -1.72, -1.30, -1.48, -1.05, -1.18]
  model = SimpleNamespace(
    position=polyline(xs, [0.0] * len(xs)),
    laneLines=[
      polyline(xs, noisy),
      polyline(xs, [1.8, 2.2, 1.5, 2.3, 1.4, 2.5, 1.3]),
    ],
    laneLineProbs=[0.9, 0.3],
    roadEdges=[],
    roadEdgeStds=[],
  )
  radar = SimpleNamespace(
    leadOne=SimpleNamespace(status=False, dRel=0.0, yRel=0.0, vRel=0.0),
    leadTwo=SimpleNamespace(status=False, dRel=0.0, yRel=0.0, vRel=0.0),
  )
  scene = extract_driving_scene(model, radar)
  assert len(scene["lanes"]) == 1
  smoothed = [point[1] for point in scene["lanes"][0]["points"]]
  assert smoothed != noisy
  raw_roughness = sum(abs(noisy[index + 1] - 2.0 * noisy[index] + noisy[index - 1])
                      for index in range(1, len(noisy) - 1))
  smooth_roughness = sum(abs(smoothed[index + 1] - 2.0 * smoothed[index] + smoothed[index - 1])
                         for index in range(1, len(smoothed) - 1))
  assert smooth_roughness < raw_roughness
  for index, value in enumerate(smoothed):
    neighborhood = noisy[max(0, index - 2):min(len(noisy), index + 3)]
    assert min(neighborhood) <= value <= max(neighborhood)


def test_road_edges_are_smoothed_and_low_confidence_is_hidden():
  xs = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
  noisy_edge = [-4.2, -3.7, -4.1, -3.5, -3.9, -3.2, -3.6]
  model = SimpleNamespace(
    position=polyline(xs, [0.0] * len(xs)),
    laneLines=[],
    laneLineProbs=[],
    roadEdges=[
      polyline(xs, noisy_edge),
      polyline(xs, [4.2, 5.0, 3.7, 5.2, 3.6, 5.3, 3.5]),
    ],
    roadEdgeStds=[0.2, 0.75],
  )
  radar = SimpleNamespace(
    leadOne=SimpleNamespace(status=False, dRel=0.0, yRel=0.0, vRel=0.0),
    leadTwo=SimpleNamespace(status=False, dRel=0.0, yRel=0.0, vRel=0.0),
  )
  scene = extract_driving_scene(model, radar)
  assert len(scene["edges"]) == 1
  assert scene["edges"][0]["probability"] == 0.8
  smoothed = [point[1] for point in scene["edges"][0]["points"]]
  assert smoothed != noisy_edge
  raw_roughness = sum(abs(noisy_edge[index + 1] - 2.0 * noisy_edge[index] + noisy_edge[index - 1])
                      for index in range(1, len(noisy_edge) - 1))
  smooth_roughness = sum(abs(smoothed[index + 1] - 2.0 * smoothed[index] + smoothed[index - 1])
                         for index in range(1, len(smoothed) - 1))
  assert smooth_roughness < raw_roughness
  for index, value in enumerate(smoothed):
    neighborhood = noisy_edge[max(0, index - 2):min(len(noisy_edge), index + 3)]
    assert min(neighborhood) <= value <= max(neighborhood)


def test_invalid_or_far_leads_are_ignored():
  empty_model = SimpleNamespace(position=None, laneLines=[], laneLineProbs=[], roadEdges=[], roadEdgeStds=[])
  radar = SimpleNamespace(
    leadOne=SimpleNamespace(status=True, dRel=200.0, yRel=0.0, vRel=0.0),
    leadTwo=SimpleNamespace(status=False, dRel=20.0, yRel=0.0, vRel=0.0),
  )
  scene = extract_driving_scene(empty_model, radar)
  assert scene["path"] == []
  assert scene["leads"] == []


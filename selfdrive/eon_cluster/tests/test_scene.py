from types import SimpleNamespace

from selfdrive.eon_cluster.scene import extract_driving_scene


def polyline(xs, ys):
  return SimpleNamespace(x=xs, y=ys)


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
    leadTwo=SimpleNamespace(status=False, dRel=0.0, yRel=0.0, vRel=0.0),
  )
  scene = extract_driving_scene(model, radar)
  assert len(scene["path"]) == 3
  assert len(scene["lanes"]) == 2
  assert scene["lanes"][0]["probability"] == 0.9
  assert scene["leads"] == [{"distance": 32.0, "lateral": 0.3, "relative_speed": -1.5}]


def test_invalid_or_far_leads_are_ignored():
  empty_model = SimpleNamespace(position=None, laneLines=[], laneLineProbs=[], roadEdges=[], roadEdgeStds=[])
  radar = SimpleNamespace(
    leadOne=SimpleNamespace(status=True, dRel=200.0, yRel=0.0, vRel=0.0),
    leadTwo=SimpleNamespace(status=False, dRel=20.0, yRel=0.0, vRel=0.0),
  )
  scene = extract_driving_scene(empty_model, radar)
  assert scene["path"] == []
  assert scene["leads"] == []

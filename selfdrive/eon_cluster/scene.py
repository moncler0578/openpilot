import math


MAX_POLYLINE_POINTS = 24


def _field(message, name, default=None):
  try:
    return getattr(message, name)
  except Exception:
    return default


def _finite_float(value, default=0.0):
  try:
    number = float(value)
  except (TypeError, ValueError):
    return default
  return number if math.isfinite(number) else default


def _point_series(polyline):
  raw_xs = _field(polyline, "x", [])
  raw_ys = _field(polyline, "y", [])
  xs = list(raw_xs) if raw_xs is not None else []
  ys = list(raw_ys) if raw_ys is not None else []
  count = min(len(xs), len(ys))
  if count <= 0:
    return []
  stride = max(1, int(math.ceil(float(count) / MAX_POLYLINE_POINTS)))
  points = []
  for index in range(0, count, stride):
    x = _finite_float(xs[index], -1.0)
    y = _finite_float(ys[index])
    if 0.0 <= x <= 160.0:
      points.append((x, y))
  if count > 1 and (count - 1) % stride and len(points) < MAX_POLYLINE_POINTS:
    x = _finite_float(xs[-1], -1.0)
    y = _finite_float(ys[-1])
    if 0.0 <= x <= 160.0:
      points.append((x, y))
  return points


def _probability(values, index, default=1.0):
  try:
    return max(0.0, min(1.0, _finite_float(values[index], default)))
  except (IndexError, TypeError):
    return default


def _lead(radar_state, name):
  lead = _field(radar_state, name)
  if lead is None or not bool(_field(lead, "status", False)):
    return None
  distance = _finite_float(_field(lead, "dRel", -1.0), -1.0)
  if distance <= 0.0 or distance > 160.0:
    return None
  return {
    "distance": distance,
    "lateral": _finite_float(_field(lead, "yRel", 0.0)),
    "relative_speed": _finite_float(_field(lead, "vRel", 0.0)),
  }


def extract_driving_scene(model, radar_state):
  raw_lane_probabilities = _field(model, "laneLineProbs", [])
  lane_probabilities = list(raw_lane_probabilities) if raw_lane_probabilities is not None else []
  raw_lanes = _field(model, "laneLines", [])
  lanes = []
  for index, lane in enumerate(list(raw_lanes) if raw_lanes is not None else []):
    points = _point_series(lane)
    if len(points) >= 2:
      lanes.append({"points": points, "probability": _probability(lane_probabilities, index)})

  edges = []
  raw_edge_stds = _field(model, "roadEdgeStds", [])
  edge_stds = list(raw_edge_stds) if raw_edge_stds is not None else []
  raw_edges = _field(model, "roadEdges", [])
  for index, edge in enumerate(list(raw_edges) if raw_edges is not None else []):
    points = _point_series(edge)
    if len(points) >= 2:
      std = max(0.0, _finite_float(edge_stds[index], 0.0)) if index < len(edge_stds) else 0.0
      edges.append({"points": points, "probability": max(0.2, min(1.0, 1.0 - std))})

  leads = []
  for name in ("leadOne", "leadTwo"):
    lead = _lead(radar_state, name)
    if lead is not None:
      leads.append(lead)

  return {
    "path": _point_series(_field(model, "position")),
    "lanes": lanes,
    "edges": edges,
    "leads": leads,
  }

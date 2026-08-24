import math
import statistics


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


def _near_line_y(line, maximum_x=30.0):
  """Robust near-field lateral position for HUD-only lane placement."""
  xs = list(_field(line, "x", []) or [])
  ys = list(_field(line, "y", []) or [])
  values = [_finite_float(y, float("nan")) for x, y in zip(xs, ys)
            if math.isfinite(_finite_float(x, float("nan"))) and
            0.0 <= _finite_float(x) <= maximum_x and
            math.isfinite(_finite_float(y, float("nan")))]
  return statistics.median(values) if len(values) >= 2 else None


ROAD_EDGE_STD_MAX = 0.5


def camera_lane_position(model):
  """Estimate the ego lane from modelV2 lane lines and road edges.

  modelV2 does not publish a semantic "lane 2" value. It does publish the two
  ego-lane boundaries and both road edges, so the number of lane-width spaces
  between them gives a conservative camera-relative lane index. This metadata
  is display-only and never feeds lateral control.
  """
  lanes = list(_field(model, "laneLines", []) or [])
  edges = list(_field(model, "roadEdges", []) or [])
  lane_probs = list(_field(model, "laneLineProbs", []) or [])
  edge_stds = list(_field(model, "roadEdgeStds", []) or [])
  if len(lanes) < 3 or len(edges) < 2:
    return None

  try:
    inner = [_near_line_y(lanes[1]), _near_line_y(lanes[2])]
    road = [_near_line_y(edges[0]), _near_line_y(edges[1])]
    inner_conf = min(_finite_float(lane_probs[1], 0.0), _finite_float(lane_probs[2], 0.0))
    # roadEdgeStds is a standard deviation in metres, not a probability, so it
    # is compared against a distance threshold (same rule as NOO control).
    edge_std = max(_finite_float(edge_stds[0], 9.9), _finite_float(edge_stds[1], 9.9))
  except (IndexError, TypeError):
    return None
  if any(value is None for value in inner + road) or inner_conf < 0.45 or \
     edge_std > ROAD_EDGE_STD_MAX:
    return None

  lane_left, lane_right = max(inner), min(inner)
  road_left, road_right = max(road), min(road)
  lane_width = lane_left - lane_right
  if not 2.5 <= lane_width <= 4.5 or road_left < lane_left or road_right > lane_right:
    return None

  left_lanes = int(round(max(0.0, road_left - lane_left) / lane_width))
  right_lanes = int(round(max(0.0, lane_right - road_right) / lane_width))
  total = 1 + left_lanes + right_lanes
  current = 1 + left_lanes
  if not 1 <= total <= 8 or not 1 <= current <= total:
    return None
  return {
    "n": total,
    "cur": current,
    "confidence": round(inner_conf, 2),
    "laneWidth": round(lane_width, 2),
  }


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


def _smooth_polyline(points):
  """Locally smooth display geometry without a global-curve overshoot."""
  if len(points) < 3:
    return points

  ordered = sorted(points, key=lambda point: point[0])
  smoothed = []
  for index, (x, y) in enumerate(ordered):
    start = max(0, index - 2)
    end = min(len(ordered), index + 3)
    weighted_sum = 0.0
    weight_total = 0.0
    for neighbor in range(start, end):
      distance = abs(neighbor - index)
      weight = (3.0, 2.0, 1.0)[min(2, distance)]
      weighted_sum += float(ordered[neighbor][1]) * weight
      weight_total += weight
    local_y = weighted_sum / max(1.0, weight_total)
    # A convex local average cannot swing outside the nearby model points,
    # unlike the previous whole-line quadratic fit on long road curves.
    smoothed.append((float(x), local_y * 0.78 + float(y) * 0.22))
  return smoothed


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


def final_lateral_path(lateral_plan, model, time_indices, limit=33):
  """Build the optimized MPC path and retain model road elevation.

  dPathPoints is only the reference passed into the solver. LateralPlanner
  publishes the solver's x/y state trajectory as mpcPathX/mpcPathY. Keep model
  Z at the same shooting-node index so the renderer retains road elevation.
  """
  if not bool(_field(lateral_plan, "mpcSolutionValid", False)):
    return []
  raw_xs = _field(lateral_plan, "mpcPathX", [])
  raw_ys = _field(lateral_plan, "mpcPathY", [])
  xs = list(raw_xs) if raw_xs is not None else []
  ys = list(raw_ys) if raw_ys is not None else []
  if not xs or not ys:
    return []

  position = _field(model, "position")
  zs = list(_field(position, "z", []) or [])
  count = min(len(xs), len(ys), len(time_indices), int(limit))
  if count < 2:
    return []
  return [[round(_finite_float(xs[i]), 2),
           round(_finite_float(ys[i]), 2),
           round(_finite_float(zs[i] if i < len(zs) else 0.0), 2)]
          for i in range(count)]


def extract_hud_scene(model, radar_state, include_debug_counts=False):
  """Extract only geometry that the external HUD actually paints."""
  leads = []
  for name in ("leadOne", "leadTwo"):
    lead = _lead(radar_state, name)
    if lead is not None:
      leads.append(lead)

  scene = {
    "path": _point_series(_field(model, "position")),
    "lanes": [],
    "edges": [],
    "leads": leads,
  }
  if include_debug_counts:
    try:
      scene["lane_count"] = len(_field(model, "laneLines", []) or [])
    except TypeError:
      scene["lane_count"] = 0
    try:
      scene["edge_count"] = len(_field(model, "roadEdges", []) or [])
    except TypeError:
      scene["edge_count"] = 0
  return scene


def extract_driving_scene(model, radar_state):
  raw_lane_probabilities = _field(model, "laneLineProbs", [])
  lane_probabilities = list(raw_lane_probabilities) if raw_lane_probabilities is not None else []
  raw_lanes = _field(model, "laneLines", [])
  lanes = []
  for index, lane in enumerate(list(raw_lanes) if raw_lanes is not None else []):
    probability = _probability(lane_probabilities, index)
    points = _point_series(lane)
    if len(points) >= 2 and probability >= 0.45:
      # Keep the modelV2 index: laneLines[1] and [2] bound the ego lane, and
      # that mapping is lost once a weak line is filtered out of the list.
      lanes.append({"points": _smooth_polyline(points), "probability": probability,
                    "index": index})

  edges = []
  raw_edge_stds = _field(model, "roadEdgeStds", [])
  edge_stds = list(raw_edge_stds) if raw_edge_stds is not None else []
  raw_edges = _field(model, "roadEdges", [])
  for index, edge in enumerate(list(raw_edges) if raw_edges is not None else []):
    points = _point_series(edge)
    std = max(0.0, _finite_float(edge_stds[index], 0.0)) if index < len(edge_stds) else 0.0
    probability = max(0.0, min(1.0, 1.0 - std))
    if len(points) >= 2 and probability >= 0.40:
      # Road edges need the same display-only spatial smoothing as lane lines.
      # Otherwise noisy model points become exaggerated bends after the
      # perspective projection on the shallow 9.2-inch HUD.
      edges.append({"points": _smooth_polyline(points), "probability": probability,
                    "index": index})

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

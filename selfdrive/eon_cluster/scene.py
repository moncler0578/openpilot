import math
import statistics


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

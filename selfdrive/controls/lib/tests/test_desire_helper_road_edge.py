from types import SimpleNamespace

from selfdrive.controls.lib.desire_helper import DesireHelper, ROAD_EDGE_OPEN_CONFIRM_FRAMES


def line(y_values):
  count = len(y_values)
  return SimpleNamespace(x=[30.0 * i / (count - 1) for i in range(count)],
                         y=list(y_values))


def model(left_edge, right_edge, probs=(0.8, 0.9, 0.9, 0.8), edge_stds=(0.1, 0.1)):
  count = len(left_edge)
  return SimpleNamespace(
    roadEdges=[line(left_edge), line(right_edge)],
    laneLines=[line([-5.4] * count), line([-1.8] * count),
               line([1.8] * count), line([5.4] * count)],
    laneLineProbs=list(probs),
    roadEdgeStds=list(edge_stds),
  )


def test_close_guardrails_block_both_directions():
  md = model([-2.5] * 8, [2.5] * 8)
  assert DesireHelper._road_edge_detected(md, -1)
  assert DesireHelper._road_edge_detected(md, 1)


def test_adjacent_lanes_remain_open_both_directions():
  md = model([-5.4] * 8, [5.4] * 8)
  assert not DesireHelper._road_edge_detected(md, -1)
  assert not DesireHelper._road_edge_detected(md, 1)


def test_far_path_outlier_does_not_open_close_left_edge():
  md = model([-2.5] * 7 + [-6.5], [5.4] * 8)
  assert DesireHelper._road_edge_detected(md, -1)


def test_left_centre_line_without_outer_lane_blocks_change():
  md = model([-7.0] * 8, [5.4] * 8, probs=(0.05, 0.9, 0.9, 0.8))
  assert DesireHelper._road_edge_detected(md, -1)


def test_right_current_lane_uses_probability_index_two():
  md = model([-5.4] * 8, [4.6] * 8, probs=(0.8, 0.05, 0.9, 0.05))
  assert DesireHelper._road_edge_detected(md, 1)


def test_invalid_model_data_fails_closed():
  assert DesireHelper._road_edge_detected(None, -1)
  assert DesireHelper._road_edge_detected(SimpleNamespace(), 1)


def test_weak_outer_line_does_not_block_when_target_lane_space_is_wide_enough():
  md = model([-5.4] * 8, [5.4] * 8,
             probs=(0.8, 0.9, 0.9, 0.05), edge_stds=(0.1, 0.9))
  assert not DesireHelper._road_edge_detected(md, 1)


def test_open_geometry_must_remain_stable_for_point_two_seconds():
  helper = DesireHelper.__new__(DesireHelper)
  helper.road_edge_open_count = {-1: 0, 1: 0}
  open_md = model([-5.4] * 8, [5.4] * 8)
  for _ in range(ROAD_EDGE_OPEN_CONFIRM_FRAMES - 1):
    assert helper._road_edge_blocked(open_md, 1)
  assert not helper._road_edge_blocked(open_md, 1)


def test_close_edge_blocks_immediately_and_resets_open_confirmation():
  helper = DesireHelper.__new__(DesireHelper)
  helper.road_edge_open_count = {-1: 0, 1: 0}
  open_md = model([-5.4] * 8, [5.4] * 8)
  close_md = model([-5.4] * 8, [2.5] * 8)
  for _ in range(ROAD_EDGE_OPEN_CONFIRM_FRAMES):
    helper._road_edge_blocked(open_md, 1)
  assert helper.road_edge_open_count[1] == ROAD_EDGE_OPEN_CONFIRM_FRAMES
  assert helper._road_edge_blocked(close_md, 1)
  assert helper.road_edge_open_count[1] == 0

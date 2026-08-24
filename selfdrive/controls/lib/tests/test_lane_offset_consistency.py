"""LanelessOffset 이 레인리스에서만 적용되는지 확인.

레인모드는 직진인데 레인리스에서만 한쪽으로 쏠릴 때 쓰는 보정이므로,
차선이 잡히는 비중만큼 자동으로 사라져야 한다.
"""
import numpy as np

from selfdrive.controls.lib.lane_planner import LanePlanner


def planner(model_y=0.0, lane_y=1.85, prob=0.95):
  lp = LanePlanner()
  n = 33
  lp.ll_t = np.linspace(0.0, 10.0, n)
  lp.ll_x = np.linspace(0.0, 200.0, n)
  lp.lll_y = np.full(n, lane_y) + lp.camera_offset
  lp.rll_y = np.full(n, -lane_y) + lp.camera_offset
  lp.lll_prob = lp.rll_prob = prob
  lp.lll_std = lp.rll_std = 0.1
  lp.lane_width = 2 * lane_y
  path = np.zeros((n, 3))
  path[:, 0] = lp.ll_x
  path[:, 1] = model_y
  return lp, path


def target(lanelines_active, laneless_offset=0.0, model_y=0.0):
  lp, path = planner(model_y=model_y)
  lp.laneless_offset = laneless_offset
  lp.param_read_frame = 5          # 파라미터 재읽기가 값을 덮어쓰지 않게
  return float(lp.get_d_path(20.0, lp.ll_t, path, lanelines_active)[0, 1])


def test_offset_is_fully_applied_in_laneless():
  assert abs(target(0.0, laneless_offset=-0.10) - (-0.10)) < 1e-6


def test_offset_does_not_leak_into_lane_mode():
  base = target(1.0)
  assert abs(target(1.0, laneless_offset=-0.10) - base) < 0.005


def test_offset_fades_between_modes():
  mid = target(0.5, laneless_offset=-0.10)
  assert -0.10 < mid < 0.0


def test_model_path_is_not_mutated_in_place():
  lp, path = planner()
  lp.laneless_offset = -0.10
  before = path[:, 1].copy()
  lp.get_d_path(20.0, lp.ll_t, path, 0.0)
  assert np.allclose(path[:, 1], before)

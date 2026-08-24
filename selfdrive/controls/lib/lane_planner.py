import numpy as np
from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp, clip, mean
from common.realtime import DT_MDL
from selfdrive.hardware import EON, TICI
from selfdrive.swaglog import cloudlog
from common.params import Params

TRAJECTORY_SIZE = 33
ADJUST_OFFSET_LIMIT = 0.4   # 여유공간 보정 최대치(m)

# CAMERA_OFFSET, PATH_OFFSET 하드코딩 제거
# → lateral_planner.py에서 Params 기반으로 주입됨
CAMERA_OFFSET = -0.06          # ← 이 줄 추가 (controlsd.py import 호환용)
DEFAULT_CAMERA_OFFSET = -0.06

ENABLE_ZORROBYTE = True
ENABLE_INC_LANE_PROB = True

class LanePlanner:
  def __init__(self, wide_camera=False):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.lane_width_estimate = FirstOrderFilter(3.7, 9.95, DT_MDL)
    self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    self.lane_width = 3.7

    self.lll_prob = 0.
    self.rll_prob = 0.

    # UI 디버그용 : 차량 중심에서 좌/우 차선까지의 거리(m)
    self.lane_width_left = 0.
    self.lane_width_right = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.wide_camera = wide_camera

    # camera_offset: 하드웨어 기본값 고정 (사용자 조정은 offset_total 로 일원화)
    # 초기값은 DEFAULT_CAMERA_OFFSET 사용 (wide_camera 부호 반전은 lateral_planner에서 처리)
    self.camera_offset = -DEFAULT_CAMERA_OFFSET if wide_camera else DEFAULT_CAMERA_OFFSET

    self.readings = []
    self.frame = 0

    # ── carrot lane_planner_2 이식 : 좌/우 여유공간 필터 + 경로 오프셋 ──
    #   lane_width_left/right : 차선 바깥쪽 여유폭(m). 캐롯은 모델의
    #   meta.laneWidthLeft 를 쓰지만 이 포크 모델에는 없어서
    #   도로경계(roadEdges) - 차선 거리로 근사한다.
    self.le_y = np.zeros(TRAJECTORY_SIZE)
    self.re_y = np.zeros(TRAJECTORY_SIZE)
    self.lane_width_left = 0.0
    self.lane_width_right = 0.0
    self.lane_width_left_filtered = FirstOrderFilter(1.0, 1.0, DT_MDL)
    self.lane_width_right_filtered = FirstOrderFilter(1.0, 1.0, DT_MDL)
    self.lane_offset_filtered = FirstOrderFilter(0.0, 2.0, DT_MDL)
    self.lane_offset = 0.0
    self.params = Params()
    self.adjust_lane_offset = 0.0
    # 레인리스에서만 쓰는 횡보정. 모델 경로 자체가 한쪽으로 치우쳐 있을 때
    # 쓴다(양수 = 왼쪽). 차선이 잡히는 비중만큼 자동으로 사라지므로 레인모드
    # 주행에는 영향이 없다.
    self.laneless_offset = 0.0
    self.param_read_frame = 0

  def parse_model(self, md):
    lane_lines = md.laneLines
    if len(lane_lines) == 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      self.ll_x = lane_lines[1].x

      self.lll_y = np.array(lane_lines[1].y) + self.camera_offset
      self.rll_y = np.array(lane_lines[2].y) + self.camera_offset
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

      # 좌/우 차선까지의 횡거리 (카메라 오프셋 반영된 값의 절대값)
      self.lane_width_left = float(abs(self.lll_y[0]))
      self.lane_width_right = float(abs(self.rll_y[0]))

    # 도로경계까지의 거리 → 차선 바깥 여유폭
    edges = md.roadEdges
    if len(edges) >= 2 and len(edges[0].t) == TRAJECTORY_SIZE:
      self.le_y = np.array(edges[0].y) + md.roadEdgeStds[0] * 0.4 + self.camera_offset
      self.re_y = np.array(edges[1].y) - md.roadEdgeStds[1] * 0.4 + self.camera_offset
      # 좌: 좌차선 ~ 좌측 도로경계 / 우: 우차선 ~ 우측 도로경계
      self.lane_width_left = float(max(0.0, abs(self.le_y[0]) - abs(self.lll_y[0])))
      self.lane_width_right = float(max(0.0, abs(self.re_y[0]) - abs(self.rll_y[0])))

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

  def get_d_path(self, v_ego, path_t, path_xyz, lanelines_active):
    path_xyz = path_xyz.copy()
    lane_line_blend = float(clip(float(lanelines_active), 0.0, 1.0))
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    if ENABLE_ZORROBYTE:
      if l_prob > 0.5 and r_prob > 0.5:
        self.frame += 1
        if self.frame > 20:
          self.frame = 0
          current_lane_width = clip(abs(self.rll_y[0] - self.lll_y[0]), 2.5, 3.5)
          self.readings.append(current_lane_width)
          self.lane_width = mean(self.readings)
          if len(self.readings) >= 30:
            self.readings.pop(0)

      if abs(self.rll_y[0] - self.lll_y[0]) > self.lane_width:
        r_prob = r_prob / interp(l_prob, [0, 1], [1, 3])

    else:
      self.lane_width_certainty.update(l_prob * r_prob)
      current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
      self.lane_width_estimate.update(current_lane_width)
      speed_lane_width = interp(v_ego, [0., 31.], [2.8, 3.5])
      self.lane_width = self.lane_width_certainty.x * self.lane_width_estimate.x + \
                        (1 - self.lane_width_certainty.x) * speed_lane_width

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    self.d_prob = l_prob + r_prob - l_prob * r_prob

    if ENABLE_INC_LANE_PROB and self.d_prob > 0.65:
      self.d_prob = min(self.d_prob * 1.3, 1.0)

    lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)

    # ── carrot 이식 1 : 좌/우 여유폭 필터링 (1초) ─────────────────────────
    if self.lane_width_left > 0:
      self.lane_width_left_filtered.update(self.lane_width_left)
    if self.lane_width_right > 0:
      self.lane_width_right_filtered.update(self.lane_width_right)

    # ── carrot 이식 2 : 여유공간 비대칭 시 경로 오프셋 ────────────────────
    #   AdjustLaneOffset (cm 단위 정수 파라미터). 0 이면 동작 안함.
    #   양쪽 다 여유(>2.2m) 또는 양쪽 다 빡빡(<2.0m) 하면 보정하지 않고,
    #   한쪽만 여유가 있을 때 그 반대쪽(좁은 쪽)에서 떨어지도록 민다.
    self.param_read_frame += 1
    if self.param_read_frame % 20 == 1:      # 1초 주기
      try:
        self.adjust_lane_offset = float(self.params.get("AdjustLaneOffset", encoding="utf8") or "0") * 0.01
      except (TypeError, ValueError):
        self.adjust_lane_offset = 0.0
      try:
        self.laneless_offset = float(self.params.get("LanelessOffset", encoding="utf8") or "0") * 0.01
      except (TypeError, ValueError):
        self.laneless_offset = 0.0

    lwl = self.lane_width_left_filtered.x
    lwr = self.lane_width_right_filtered.x
    offset_lane = 0.0
    if self.adjust_lane_offset > 0.0:
      if lwl > 2.2 and lwr > 2.2:
        offset_lane = 0.0
      elif lwl < 2.0 and lwr < 2.0:
        offset_lane = 0.0
      elif lwl > lwr:
        # 좌측이 여유 → 좌로(+) 이동. 차로가 좁으면(2.5m 미만) 하지 않음
        offset_lane = interp(self.lane_width, [2.5, 2.9], [0.0, self.adjust_lane_offset])
      else:
        offset_lane = interp(self.lane_width, [2.5, 2.9], [0.0, -self.adjust_lane_offset])
    offset_lane = clip(offset_lane, -ADJUST_OFFSET_LIMIT, ADJUST_OFFSET_LIMIT)

    # d_prob 가 낮으면 오프셋도 서서히 0 으로 (2초 필터)
    self.lane_offset_filtered.update(interp(self.d_prob, [0.0, 0.3], [0.0, offset_lane]))
    self.lane_offset = float(self.lane_offset_filtered.x)

    safe_idxs = np.isfinite(self.ll_t)
    effective_d_prob = 0.0
    if safe_idxs[0] and lane_line_blend > 0.0:
      lane_path_y_interp = np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
      effective_d_prob = self.d_prob * lane_line_blend
      path_xyz[:,1] = effective_d_prob * lane_path_y_interp + (1.0 - effective_d_prob) * path_xyz[:,1]
      # 차선경로가 쓰이는 비중만큼만 여유공간 보정을 적용한다.
      path_xyz[:,1] += self.lane_offset * effective_d_prob
    elif not safe_idxs[0]:
      cloudlog.warning("Lateral mpc - NaNs in laneline times, ignoring")

    # 모델 경로가 쓰이는 비중(= 1 - effective_d_prob)만큼만 레인리스 보정을
    # 적용한다. 차선이 잘 보이면 0 에 가까워지므로 레인모드는 그대로다.
    if self.laneless_offset != 0.0:
      path_xyz[:,1] += self.laneless_offset * (1.0 - effective_d_prob)
    return path_xyz

import numpy as np
from common.conversions import Conversions as CV
from common.realtime import sec_since_boot, DT_MDL
from common.numpy_fast import interp
from selfdrive.controls.lib.lane_planner import LanePlanner
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import N as LAT_MPC_N
from selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED
from selfdrive.controls.lib.desire_helper import DesireHelper, AUTO_LCA_START_TIME
from selfdrive.controls.lib.navigation_route import NavigationRouteData
import cereal.messaging as messaging
from cereal import log
from common.params import Params

LaneChangeState = log.LateralPlan.LaneChangeState

TRAJECTORY_SIZE = 33

DEFAULT_PATH_COST = 1.0
DEFAULT_LATERAL_MOTION_COST = 0.11
DEFAULT_LATERAL_ACCEL_COST = 0.0
DEFAULT_LATERAL_JERK_COST = 0.04
DEFAULT_STEERING_RATE_COST = 550.0
LANE_MODE_BLEND_TIME = 0.6
NOO_MAP_BLEND_MAX = 0.60
NOO_MAP_BLEND_FULL_SPEED_KPH = 20.0
NOO_MAP_BLEND_ZERO_SPEED_KPH = 50.0
NOO_MAP_BLEND_IN_TIME = 0.5
NOO_MAP_BLEND_OUT_TIME = 0.25
NOO_MAP_MAX_LAT_ACCEL = 2.0
NOO_MAP_MAX_CURVATURE = 0.06

# 기본값 상수
DEFAULT_CAMERA_OFFSET = -0.06
DEFAULT_OFFSET_TOTAL = 0.0



class LateralPlanner:
  def __init__(self, CP, wide_camera=False, debug=False):
    self.params = Params()
    self.wide_camera = wide_camera

    # Keep the hardware camera offset fixed and apply the cached user
    # OffsetTotal once to the final path.
    self.offset_total = DEFAULT_OFFSET_TOTAL

    self.LP = LanePlanner(wide_camera=wide_camera)

    self.DH = DesireHelper()

    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.velocity_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw_rate = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros((TRAJECTORY_SIZE,))
    self.v_plan = np.zeros((TRAJECTORY_SIZE,))
    self.v_ego = 0.0
    self.l_lane_change_prob = 0.0
    self.r_lane_change_prob = 0.0
    self.d_path_w_lines_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.d_path_xyz = np.zeros((TRAJECTORY_SIZE, 3))  # 방어용 초기화

    self.debug_mode = debug

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))

    self.dynamic_lane_profile = 0
    self.dynamic_lane_profile_status = True
    self.dynamic_lane_profile_status_buffer = True
    self.use_lane_line_mode = False
    self.lane_line_blend = None
    self.noo_map_blend = 0.0
    self.noo_map_profile_cache = None
    self.noo_map_path_cache = None

    self.param_read_counter = 0
    self.path_cost = DEFAULT_PATH_COST
    self.lateral_motion_cost = DEFAULT_LATERAL_MOTION_COST
    self.lateral_accel_cost = DEFAULT_LATERAL_ACCEL_COST
    self.lateral_jerk_cost = DEFAULT_LATERAL_JERK_COST
    self.steering_rate_cost = DEFAULT_STEERING_RATE_COST
    self.read_param(force=True)

  def _read_mpc_cost(self, key, default, minimum, maximum, scale=1000.0):
    try:
      raw = float(self.params.get(key, encoding="utf8") or str(round(default * scale)))
    except (TypeError, ValueError):
      raw = default * scale
    return float(np.clip(raw / scale, minimum, maximum))

  def read_param(self, force=False):
    # modelV2 drives this planner at 20 Hz. Params storage only needs a 1 Hz
    # refresh; cached values are used for every MPC update in between.
    if force or self.param_read_counter % 20 == 0:
      try:
        self.dynamic_lane_profile = int(
          self.params.get("DynamicLaneProfile", encoding="utf8") or "0")
      except (TypeError, ValueError):
        self.dynamic_lane_profile = 0
      self.offset_total = self._read_offset_total()
      # Store fractional MPC weights as integers scaled by 1000 so the
      # existing integer Params UI can adjust them without precision loss.
      self.path_cost = self._read_mpc_cost(
        "MpcPathCost", DEFAULT_PATH_COST, 0.1, 5.0)
      self.lateral_motion_cost = self._read_mpc_cost(
        "MpcLateralMotionCost", DEFAULT_LATERAL_MOTION_COST, 0.0, 1.0)
      self.lateral_accel_cost = self._read_mpc_cost(
        "MpcLateralAccelCost", DEFAULT_LATERAL_ACCEL_COST, 0.0, 1.0)
      self.lateral_jerk_cost = self._read_mpc_cost(
        "MpcLateralJerkCost", DEFAULT_LATERAL_JERK_COST, 0.0, 0.5)
      # Very small values can make the DH steering response abrupt, while
      # values above this range add little stability and noticeably delay
      # lane/path changes. Keep the live UI setting inside a safe range.
      self.steering_rate_cost = self._read_mpc_cost(
        "SteeringRateCost", DEFAULT_STEERING_RATE_COST, 200.0, 1200.0, scale=1.0)
    self.param_read_counter += 1

  def _read_offset_total(self):
    try:
      val = float(self.params.get("OffsetTotal", encoding="utf8") or str(DEFAULT_OFFSET_TOTAL))
    except (TypeError, ValueError):
      val = DEFAULT_OFFSET_TOTAL
    return max(-1.0, min(1.0, val))

  def reset_mpc(self, x0=np.zeros(4)):
    self.x0 = x0
    self.lat_mpc.reset(x0=self.x0)

  def update(self, sm):
    self.read_param()

    measured_curvature = sm['controlsState'].curvature

    md = sm['modelV2']
    self.LP.parse_model(md)
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      self.t_idxs = np.array(md.position.t)
      self.plan_yaw = np.array(md.orientation.z)
      self.plan_yaw_rate = np.array(md.orientationRate.z)
      self.velocity_xyz = np.column_stack([md.velocity.x, md.velocity.y, md.velocity.z])
      car_speed = np.linalg.norm(self.velocity_xyz, axis=1)
      self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
      self.v_ego = self.v_plan[0]

    lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
    self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, md)

    if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
      self.LP.lll_prob *= self.DH.lane_change_ll_prob
      self.LP.rll_prob *= self.DH.lane_change_ll_prob

    lane_mode_speed = 10 * CV.MPH_TO_MS
    if self.v_ego >= lane_mode_speed + 2 * CV.KPH_TO_MS:
      self.use_lane_line_mode = True
    elif self.v_ego < lane_mode_speed - 2 * CV.KPH_TO_MS:
      self.use_lane_line_mode = False

    # Fixed selections must match the setting at every speed. The low-speed
    # lane-line fallback belongs only to Auto mode.
    if self.dynamic_lane_profile == 0:
      use_laneless = False
    elif self.dynamic_lane_profile == 1:
      use_laneless = True
    else:
      use_laneless = not self.use_lane_line_mode or self.get_dynamic_lane_profile()

    self.dynamic_lane_profile_status = use_laneless
    lane_line_blend_target = 0.0 if use_laneless else 1.0
    if self.lane_line_blend is None:
      self.lane_line_blend = lane_line_blend_target
    else:
      # Avoid a lateral target jump when Auto mode changes between the
      # lane-line and model paths. At 20 Hz this completes in about 0.6 s.
      max_blend_step = DT_MDL / LANE_MODE_BLEND_TIME
      self.lane_line_blend += np.clip(lane_line_blend_target - self.lane_line_blend,
                                      -max_blend_step, max_blend_step)
    self.d_path_w_lines_xyz = self.LP.get_d_path(
      self.v_ego, self.t_idxs, self.path_xyz, self.lane_line_blend)
    # Feed the selected lane/model blend into MPC. Previously this result was
    # only published for display while MPC kept following the raw model path.
    self.path_xyz = self.d_path_w_lines_xyz.copy()

    self.lat_mpc.set_weights(self.path_cost, self.lateral_motion_cost,
                             self.lateral_accel_cost, self.lateral_jerk_cost,
                             self.steering_rate_cost)

    # offset_total 을 최종 결정된 path_xyz 에 적용 (레인모드/레인리스 공통)
    self.path_xyz[:, 1] += self.offset_total

    # Reuse the path-distance vector for both interpolations. The trajectory is
    # unchanged between them, so a second NumPy norm only wastes planner CPU.
    path_distance = np.linalg.norm(self.path_xyz, axis=1)
    sample_distances = self.v_ego * self.t_idxs[:LAT_MPC_N + 1]
    y_pts = np.interp(sample_distances,
                      path_distance, self.path_xyz[:, 1])
    heading_pts = np.interp(sample_distances,
                            path_distance, self.plan_yaw)
    yaw_rate_pts = self.plan_yaw_rate[:LAT_MPC_N + 1]

    map_profile = None
    map_path = None
    navigation_state = self.DH.navigation_state
    noo_map_requested = (self.DH.noo_turn_direction != 0 and not self.DH.noo_driver_cancel and
                         navigation_state.get('fresh', False) and
                         navigation_state.get('route_fresh', False) and
                         navigation_state.get('kind') in ('turn', 'uturn') and
                         3.0 <= float(navigation_state.get('distance', -1.0)) <= 60.0 and
                         self.v_ego <= 50.0 * CV.KPH_TO_MS)
    if noo_map_requested:
      speed_sq = max(self.v_ego * self.v_ego, 1.0)
      max_curvature = min(NOO_MAP_MAX_CURVATURE, NOO_MAP_MAX_LAT_ACCEL / speed_sq)
      map_profile = self.DH.navigation_route.cached_route_curvature_profile(
        navigation_state, sample_distances, max_curvature=max_curvature)
      if map_profile is not None:
        map_path = NavigationRouteData.integrate_curvature_profile(map_profile, sample_distances)

    fresh_map_path = map_path is not None
    if fresh_map_path:
      self.noo_map_profile_cache = map_profile
      self.noo_map_path_cache = map_path
    elif not noo_map_requested and not self.DH.noo_driver_cancel and self.noo_map_blend > 0.0:
      # Keep the last vehicle-relative shape only for the short normal exit
      # fade. Invalid/stale data and driver cancellation never use the cache.
      map_profile = self.noo_map_profile_cache
      map_path = self.noo_map_path_cache

    map_invalid = noo_map_requested and map_path is None
    if self.DH.noo_driver_cancel or map_invalid:
      self.noo_map_blend = 0.0
      self.noo_map_profile_cache = None
      self.noo_map_path_cache = None
    else:
      speed_kph = self.v_ego * CV.MS_TO_KPH
      # Stronger map assistance is restricted to low speed. Preserve the
      # existing 50 km/h zero point and all curvature, lateral-acceleration,
      # driver-cancel, and Panda torque limits.
      blend_target = NavigationRouteData.map_steering_blend(
        speed_kph, NOO_MAP_BLEND_MAX,
        NOO_MAP_BLEND_FULL_SPEED_KPH, NOO_MAP_BLEND_ZERO_SPEED_KPH,
      ) if fresh_map_path else 0.0
      blend_time = NOO_MAP_BLEND_IN_TIME if blend_target > self.noo_map_blend else NOO_MAP_BLEND_OUT_TIME
      blend_step = NOO_MAP_BLEND_MAX * DT_MDL / blend_time
      self.noo_map_blend += float(np.clip(blend_target - self.noo_map_blend, -blend_step, blend_step))
      if self.noo_map_blend <= 0.0:
        self.noo_map_profile_cache = None
        self.noo_map_path_cache = None

    if map_path is not None and self.noo_map_blend > 0.0:
      map_y, map_heading = (np.asarray(map_path[0]), np.asarray(map_path[1]))
      map_y += y_pts[0]
      map_heading += heading_pts[0]
      y_pts = (1.0 - self.noo_map_blend) * y_pts + self.noo_map_blend * map_y
      heading_pts = ((1.0 - self.noo_map_blend) * heading_pts +
                     self.noo_map_blend * map_heading)
      map_yaw_rate = np.asarray(map_profile) * self.v_plan[:LAT_MPC_N + 1]
      yaw_rate_pts = ((1.0 - self.noo_map_blend) * yaw_rate_pts +
                      self.noo_map_blend * map_yaw_rate)
    self.y_pts = y_pts

    assert len(y_pts) == LAT_MPC_N + 1
    assert len(heading_pts) == LAT_MPC_N + 1
    assert len(yaw_rate_pts) == LAT_MPC_N + 1
    lateral_factor = np.clip(self.factor1 - (self.factor2 * self.v_plan**2), 0.0, np.inf)
    p = np.column_stack([self.v_plan, lateral_factor])
    self.lat_mpc.run(self.x0,
                     p,
                     y_pts,
                     heading_pts,
                     yaw_rate_pts)
    self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])

    mpc_nans = np.isnan(self.lat_mpc.x_sol[:, 3]).any()
    t = sec_since_boot()
    if mpc_nans or self.lat_mpc.solution_status != 0:
      self.reset_mpc()
      self.x0[3] = measured_curvature * self.v_ego
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.lat_mpc.cost > 1e6 or mpc_nans:
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0

  def get_dynamic_lane_profile(self):
    """True = 레인리스 경로 사용, False = 레인모드(차선) 경로 사용.
    DynamicLaneProfile 하나로만 결정한다. (0=레인모드 1=레인리스 2=오토)
    """
    if self.dynamic_lane_profile == 1:
      return True
    elif self.dynamic_lane_profile == 0:
      return False
    elif self.dynamic_lane_profile == 2:
      # laneless while lane change in progress
      if self.DH.lane_change_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
        return True
      elif self.DH.lane_change_state == LaneChangeState.off:
        if self.LP.lll_prob < 0.3 and self.LP.rll_prob < 0.3:
          self.dynamic_lane_profile_status_buffer = True
        elif self.LP.lll_prob > 0.5 and self.LP.rll_prob > 0.5:
          self.dynamic_lane_profile_status_buffer = False
        if self.dynamic_lane_profile_status_buffer:
          return True
    return False

  def publish(self, sm, pm):
    plan_solution_valid = self.solution_invalid_cnt < 2
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlan = plan_send.lateralPlan
    lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlan.laneWidth = float(self.LP.lane_width)
    # dPathPoints is the reference passed into the MPC. Publish the optimized
    # x/y state trajectory separately so visualizers do not mistake the
    # reference for the path the MPC actually solved.
    lateralPlan.dPathPoints = self.y_pts.tolist()
    lateralPlan.mpcPathX = self.lat_mpc.x_sol[:, 0].tolist()
    lateralPlan.mpcPathY = self.lat_mpc.x_sol[:, 1].tolist()
    lateralPlan.psis = self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()
    lateralPlan.curvatures = (self.lat_mpc.x_sol[0:CONTROL_N, 3] / self.v_ego).tolist()
    lateralPlan.curvatureRates = [float(x / self.v_ego) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]

    lateralPlan.lProb = float(self.LP.lll_prob)
    lateralPlan.rProb = float(self.LP.rll_prob)
    lateralPlan.dProb = float(self.LP.d_prob)

    lateralPlan.mpcSolutionValid = bool(plan_solution_valid)
    lateralPlan.solverExecutionTime = self.lat_mpc.solve_time
    if self.debug_mode:
      lateralPlan.solverCost = self.lat_mpc.cost
      lateralPlan.solverState = log.LateralPlan.SolverState.new_message()
      lateralPlan.solverState.x = self.lat_mpc.x_sol.tolist()
      lateralPlan.solverState.u = self.lat_mpc.u_sol.flatten().tolist()

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = not self.dynamic_lane_profile_status
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction
    lateralPlan.nooMapBlend = float(self.noo_map_blend)
    # Publish the active guidance/model desire even when the optional TMAP
    # polyline is unavailable. nooMapBlend separately tells the HUD whether
    # route curvature is actually contributing to steering.
    lateralPlan.nooTurnDirection = int(
      self.DH.noo_turn_direction if not self.DH.noo_driver_cancel else 0)
    lateralPlan.nooCurrentLane = int(self.DH.noo_current_lane)
    lateralPlan.nooTargetLane = int(self.DH.noo_target_lane)
    lateralPlan.nooLaneChangeDirection = int(self.DH.noo_direction)
    lateralPlan.nooCameraLaneCount = int(self.DH.noo_camera_lane_count)
    lateralPlan.nooRouteLaneCount = int(self.DH.noo_route_lane_count)

    plan_send.lateralPlan.dPathWLinesX = [float(x) for x in self.d_path_w_lines_xyz[:, 0]]
    plan_send.lateralPlan.dPathWLinesY = [float(y) for y in self.d_path_w_lines_xyz[:, 1]]

    lateralPlan.laneChangePrev = self.DH.prev_lane_change
    lateralPlan.laneChangeEdgeBlock = (self.DH.lane_change_state == LaneChangeState.preLaneChange) and self.DH.road_edge

    lateralPlan.autoLaneChangeEnabled = self.DH.auto_lane_change_enabled
    lateralPlan.autoLaneChangeTimer = int(AUTO_LCA_START_TIME) - int(self.DH.auto_lane_change_timer)

    lateralPlan.dynamicLaneProfile = int(self.dynamic_lane_profile)

    # ── UI 하단 중앙 차선/오프셋 디버그 문자열 ──
    # 레인모드/레인리스는 글자 대신 onroad.cc 가 문자열 왼쪽에 점으로 그린다
    # (파랑=레인모드 / 노랑=레인리스). dynamicLaneProfileStatus 를 쓴다.
    offset_cm = self.offset_total * 100.0
    tail = f'offset={offset_cm:.1f}cm'
    if abs(self.LP.lane_offset) > 0.005:
      tail += f' lane={self.LP.lane_offset * 100.0:+.0f}cm'
    if self.noo_map_blend > 0.005:
      tail += f' noomap={self.noo_map_blend * 100.0:.0f}%'
    # NOO 상태는 offset 앞의 별도 칸에 넣는다. onroad.cc 의 하단 표시는
    # "offset" 이 나오는 칸부터 잘라내므로, tail 에 붙이면 EON 화면에서는
    # 보이지 않는다(rlog 에만 남는다).
    noo_text = ''
    if self.DH.noo_turn_state:
      noo_text += f'nooturn={self.DH.noo_turn_state} '
    if self.DH.noo_current_lane > 0 and self.DH.noo_target_lane > 0:
      noo_text += f'noo={self.DH.noo_current_lane}>{self.DH.noo_target_lane}'
    elif self.DH.noo_camera_lane_count or self.DH.noo_route_lane_count:
      # No plan yet: show why. cam = camera lane count, map = TMAP lane count.
      noo_text += f'noo=cam{self.DH.noo_camera_lane_count}/map{self.DH.noo_route_lane_count}'
    lateralPlan.latDebugText = (
      f"{self.LP.lane_width_left:.1f}m | "
      f"{self.LP.lane_width:.1f}m | "
      f"{self.LP.lane_width_right:.1f}m | "
      + (f"{noo_text.strip()} | " if noo_text else "")
      + f"{tail}"
    )
    lateralPlan.dynamicLaneProfileStatus = bool(self.dynamic_lane_profile_status)

    pm.send('lateralPlan', plan_send)

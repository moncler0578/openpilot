#!/usr/bin/env python3
import os
import numpy as np

from common.realtime import sec_since_boot
from common.numpy_fast import clip, interp
from selfdrive.swaglog import cloudlog
from selfdrive.modeld.constants import index_function
from selfdrive.controls.lib.radar_helpers import _LEAD_ACCEL_TAU
from common.conversions import Conversions as CV

if __name__ == '__main__':  # generating code
  from third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython  # pylint: disable=no-name-in-module, import-error

from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

SOURCES = ['lead0', 'lead1', 'cruise', 'e2e']

X_DIM = 3
U_DIM = 1
PARAM_DIM = 7
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

X_EGO_OBSTACLE_COST = 5.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.0
A_CHANGE_COST = 200.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.8
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'


CRUISE_GAP_BP = [1., 2., 3., 4.]
CRUISE_GAP_V = [1.1, 1.2, 1.4, 1.6]

DIFF_RADAR_VISION = 1.0


N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
MIN_ACCEL = -4.0
MAX_ACCEL = 2.5
T_FOLLOW = 1.45
COMFORT_BRAKE = 2.4
STOP_DISTANCE = 6.0
STOP_DISTANCE_E2E = 6.0

# ── Lead 부드러운 전환 파라미터 ──────────────────────────────────────────────
LEAD_DETECT_RAMP_T = 0.05  # c3-wip: no extra lead-input delay
T_FOLLOW_RISE_RATE = 0.30
T_FOLLOW_DECEL_RISE_RATE = 0.60
LEAD_RAMP_BYPASS_TTC = 5.0 # 이보다 급한 접근은 ramp 없이 즉시 원본 lead 반영
# ────────────────────────────────────────────────────────────────────────────

# ── Lever A (commit d897f06): 고속 + 선행차 접근 시 추종거리 선제 확대 ─────────
# 고속 늦은 감지로 인한 충돌 우려 대응. t_follow를 미리 키워 제동 명령을 앞당긴다.
# vRel<0(접근) & TTC<임계 & 고속(≥70km/h) 삼중 게이트라 정속 추종·저속에선 무동작.
# (원본 Lever C=JLeadFactor3 속도연동 증폭은 이 포크에 jLead 신호가 없어 제외)
HIGH_SPEED_BRAKE_KPH = 70.0          # 이 속도(km/h) 이상에서만 선제 확대 적용
HIGH_SPEED_BRAKE_TTC = 7.0           # 접근 TTC(초)가 이 값 미만이면 활성
HIGH_SPEED_TF_BOOST  = 0.0          # disabled in the c3-wip cost model
# ────────────────────────────────────────────────────────────────────────────

def get_stopped_equivalence_factor(v_lead, v_ego=0., t_follow=T_FOLLOW, stop_dist=STOP_DISTANCE, krkeegan=False):
  if not krkeegan:
    return (v_lead**2) / (2 * COMFORT_BRAKE)

  # KRKeegan: lead 거리값을 고의로 늘려 solver가 더 빠른 가속을 유발하도록 함
  v_diff_offset = 0
  if np.all(v_lead - v_ego > 0):
    v_diff_offset = ((v_lead - v_ego) * 1.)
    v_diff_offset = np.clip(v_diff_offset, 0, stop_dist / 2)
    v_diff_offset = np.maximum(v_diff_offset * ((10 - v_ego) / 10), 0)

  distance = (v_lead**2) / (2 * COMFORT_BRAKE) + v_diff_offset
  return distance


def get_safe_obstacle_distance(v_ego, t_follow=T_FOLLOW, stop_dist=STOP_DISTANCE):
  return (v_ego**2) / (2 * COMFORT_BRAKE) + t_follow * v_ego + stop_dist

def desired_follow_distance(v_ego, v_lead):
  return get_safe_obstacle_distance(v_ego) - get_stopped_equivalence_factor(v_lead)


def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # set up states & controls
  x_ego = SX.sym('x_ego')
  v_ego = SX.sym('v_ego')
  a_ego = SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  # controls
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # live parameters
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  prev_a = SX.sym('prev_a')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  stop_dist = SX.sym('stop_dist')

  model.p = vertcat(a_min, a_max, x_obstacle, prev_a, lead_t_follow, lead_danger_factor, stop_dist)

  # dynamics model
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model


def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]

  # set dimensions
  ocp.dims.N = N

  # set cost module
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))

  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  prev_a = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]
  lead_danger_factor = ocp.model.p[5]
  stop_dist = ocp.model.p[6]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow, stop_dist)

  costs = [((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - prev_a,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  constraints = vertcat(v_ego,
                        (a_ego - a_min),
                        (a_max - a_ego),
                        ((x_obstacle - x_ego) - lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, T_FOLLOW, LEAD_DANGER_FACTOR, STOP_DISTANCE])

  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  # set prediction horizon
  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, mode='acc'):
    self.mode = mode
    self.applyLongDynamicCost = False
    self.prev_accel_constraint = True
    self.a_desired = 0.
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)

    # ── CarrotPilot Auto-Tuner: 학습된 GAP별 추종거리 (초 리스트, None=미사용) ──
    # longitudinal_planner.read_param()에서 5초 주기로 갱신됨.
    self.tfollow_gaps = None
    self.t_follow_speed_ratio = 1.2
    self.t_follow_decel_boost = 0.5
    self._tf_applied = 0.0
    self._tf_v_ego_kph = 0.0
    # ────────────────────────────────────────────────────────────────────

    # ── Lead 인식 부드러운 전환 상태 변수 ──────────────────────────────────
    self._lead_detected   = False
    self._lead_detect_t   = 0.0       # 인식 후 경과 시간 (ramp 용)
    self.driving_mode_tf = 1.0        # MyDrivingMode 추종거리 배율 (planner 가 갱신)
    self.desired_distance = 0.0       # UI 표시용 목표 차간거리(m)
    self.traffic_stop_active = False
    self.traffic_stop_distance = 0.0
    # ────────────────────────────────────────────────────────────────────

    self.reset()
    self.source = SOURCES[2]

  def reset(self):
    self.solver.reset()
    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.prev_a = np.array(self.a_solution)
    self.j_solution = np.zeros(N)
    self.yref = np.zeros((N+1, COST_DIM))
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N,1))
    self.params = np.zeros((N+1, PARAM_DIM))
    self.t_follow = T_FOLLOW
    self.stop_dist = STOP_DISTANCE
    # ACC/E2E 각각 독립적으로 조절되는 정지거리(m). UI 파라미터 ACCStopDistance /
    # E2EStopDistance 로 실시간 갱신됨. 기본값은 각 모드의 기존 고정값과 동일.
    self.stop_dist_acc = STOP_DISTANCE
    self.stop_dist_e2e = STOP_DISTANCE_E2E
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))
    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    # timers
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0
    self.x0 = np.zeros(X_DIM)

    # ── Lead 필터 상태 리셋 ────────────────────────────────────────────────
    self._lead_detected   = False
    self._lead_detect_t   = 0.0
    self._tf_applied = 0.0
    self._tf_v_ego_kph = 0.0
    # ────────────────────────────────────────────────────────────────────

    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      # reduce the cost on (a-a_prev) later in the horizon.
      W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def get_cost_multipliers(self, v_lead0, v_lead1):
    v_ego = self.x0[1]
    v_ego_bps = [0, 10]
    TFs = [1.2, 1.45, 1.8]
    # TF에 의한 a, j, d cost 변경
    a_change_tf = interp(self.t_follow, TFs, [.8, 1., 1.1])
    j_ego_tf    = interp(self.t_follow, TFs, [.8, 1., 1.1])
    d_zone_tf   = interp(self.t_follow, TFs, [1.3, 1., 1.])

    j_ego_v_ego    = 1
    a_change_v_ego = 1
    if (v_lead0 - v_ego >= 0) and (v_lead1 - v_ego >= 0):
      v_ego_bps = [0, 10]
      j_ego_v_ego    = interp(v_ego, v_ego_bps, [0.05, 1.0])
      a_change_v_ego = interp(v_ego, v_ego_bps, [0.05, 1.0])

    j_ego    = min(j_ego_tf, j_ego_v_ego)
    a_change = min(a_change_tf, a_change_v_ego)
    return (a_change, j_ego, d_zone_tf)

  def set_weights(self, v_ego=0., a_desired=0., prev_accel_constraint=True, v_lead0=0, v_lead1=0):
    self.prev_accel_constraint = prev_accel_constraint
    self.a_desired = a_desired

    if not prev_accel_constraint:
      self.prev_a = np.full(N+1, a_desired)

    if self.mode == 'acc':
      a_change_cost = A_CHANGE_COST if prev_accel_constraint else 0.0

      if self.applyLongDynamicCost:
        cost_multipliers = self.get_cost_multipliers(v_lead0, v_lead1)
        cost_weights = [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST,
                        a_change_cost * cost_multipliers[0],
                        J_EGO_COST * cost_multipliers[1]]
        constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST,
                                   DANGER_ZONE_COST * cost_multipliers[2]]
      else:
        cost_weights = [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST, a_change_cost, J_EGO_COST]
        constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]

    elif self.mode == 'blended':
      a_change_cost = 40.0 if prev_accel_constraint else 0.0
      cost_weights = [0., 0.1, 0.2, 5.0, a_change_cost, 1.0]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]
    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner cost set')
    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.:
      for i in range(0, N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2)/2.)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    lead_xv = np.column_stack((x_lead_traj, v_lead_traj))
    return lead_xv

  def process_lead(self, lead):
    v_ego = self.x0[1]
    if lead is not None and lead.status:
      x_lead = lead.dRel if lead.radar else max(lead.dRel - DIFF_RADAR_VISION, 0.)
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
    else:
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU

    min_x_lead = ((v_ego + v_lead)/2) * (v_ego - v_lead) / (-MIN_ACCEL * 2)
    x_lead = clip(x_lead, min_x_lead, 1e8)
    v_lead = clip(v_lead, 0.0, 1e8)
    a_lead = clip(a_lead, -10., 5.)
    lead_xv = self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)
    return lead_xv

  def set_accel_limits(self, min_a, max_a):
    self.cruise_min_a = min_a
    self.max_a = max_a

  def update(self, carstate, radarstate, v_cruise, x, v, a, j, prev_accel_constraint=True):
    # engage 직후에는 직전 가속도 유지 비용(A_CHANGE_COST)을 빼서
    # 필요한 감속으로 곧바로 갈 수 있게 한다. (upstream 동작 복원)
    self.prev_accel_constraint = prev_accel_constraint
    v_ego = self.x0[1]
    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    # ── Lead 인식 전환 ramp ──────────────────────────────────────────────
    DT_UPDATE   = 0.05  # update 주기 50 ms
    lead_status = radarstate.leadOne.status

    if lead_status:
      if not self._lead_detected:
        self._lead_detect_t = 0.0
        self._lead_detected = True
      self._lead_detect_t = min(self._lead_detect_t + DT_UPDATE, LEAD_DETECT_RAMP_T)
    else:
      if self._lead_detected:
        self._lead_detected = False
        self._lead_detect_t = 0.0

    # 0.0(새 인식) → 1.0(안정 추종)
    lead_ramp = self._lead_detect_t / LEAD_DETECT_RAMP_T
    # ─────────────────────────────────────────────────────────────────────

    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    # neokii
    cruise_gap = int(clip(carstate.cruiseGap, 1., 4.)) if carstate.cruiseGap > 0 else 4
    if self.tfollow_gaps is not None:
      # ── CarrotPilot Auto-Tuner: 학습된 GAP별 추종거리 사용 ──────────────
      # GAP1~4 모두 단계별 TFollowGap 값을 적용한다.
      tr = interp(float(cruise_gap), CRUISE_GAP_BP, self.tfollow_gaps)
      # ─────────────────────────────────────────────────────────────────
    else:
      tr = interp(float(cruise_gap), CRUISE_GAP_BP, CRUISE_GAP_V)

    # ── MyDrivingMode: GAP1~4 base 추종거리에 캐럿 모드 배율 적용 ──────────
    tr *= self.driving_mode_tf
    speed_scale = interp(v_ego * CV.MS_TO_KPH, [0.0, 100.0], [1.0, self.t_follow_speed_ratio])
    tr *= speed_scale

    # carrot-wip parity: prevent a gap reduction while decelerating, add a
    # configurable deceleration boost, ramp increases progressively, and
    # apply reductions immediately.
    gap_values = self.tfollow_gaps if self.tfollow_gaps is not None else CRUISE_GAP_V
    tr_max = max(gap_values) * self.driving_mode_tf * speed_scale
    decel_extra = float(interp(carstate.aEgo, [-2.5, -1.0, -0.3, 0.0],
                               [0.50, 0.25, 0.06, 0.0])) * self.t_follow_decel_boost
    tr_target = float(np.clip(tr + decel_extra, max(0.6, min(gap_values)), min(2.0, tr_max + decel_extra)))
    if self._tf_applied <= 0.0:
      tr = tr_target
    else:
      if carstate.aEgo <= -0.2 and tr_target < self._tf_applied:
        tr_target = self._tf_applied
      if tr_target > self._tf_applied:
        rise_rate = T_FOLLOW_DECEL_RISE_RATE if decel_extra > 0.02 else T_FOLLOW_RISE_RATE
        tr = min(tr_target, self._tf_applied + rise_rate * DT_UPDATE)
      else:
        tr = tr_target
    self._tf_applied = float(tr)
    # ────────────────────────────────────────────────────────────────────

    self.t_follow = tr
    self.desired_distance = float(tr * v_ego + STOP_DISTANCE)   # UI 표시용
    self.stop_dist = self.stop_dist_e2e if self.traffic_stop_active else \
                     (self.stop_dist_acc if self.mode == 'acc' else self.stop_dist_e2e)

    # ── 새 lead의 MPC 입력 점진 반영 ──────────────────────────────────────
    # 여유 있는 최초 인식은 가상의 비제약 lead에서 원본으로
    # 보간해 순간 제동을 줄이고, 안전거리 안쪽/TTC<5초인 접근은 즉시 원본을 사용한다.
    lead_urgent = False
    lead_input_ramp = 1.0
    _lead = radarstate.leadOne
    if _lead.status and _lead.dRel > 0.0:
      closing_speed = max(v_ego - _lead.vLead, 0.0)
      lead_ttc = _lead.dRel / closing_speed if closing_speed > 0.1 else 1e3
      safe_drel = (get_safe_obstacle_distance(v_ego, self.t_follow, self.stop_dist)
                   - get_stopped_equivalence_factor(max(_lead.vLead, 0.0)))
      lead_urgent = _lead.dRel <= max(safe_drel, 0.0) or lead_ttc < LEAD_RAMP_BYPASS_TTC
      lead_input_ramp = 1.0 if lead_urgent else lead_ramp

      if lead_input_ramp < 1.0:
        virtual_x = max(float(_lead.dRel), float(safe_drel) + 10.0, 50.0)
        virtual_v = max(float(_lead.vLead), v_ego + 10.0)
        virtual_xv = self.extrapolate_lead(virtual_x, virtual_v, 0.0, _LEAD_ACCEL_TAU)
        lead_xv_0 = virtual_xv * (1.0 - lead_input_ramp) + lead_xv_0 * lead_input_ramp
    # ─────────────────────────────────────────────────────────────────────

    # ── Lever A (commit d897f06): 고속 + 선행차 접근 → 추종거리 선제 확대 ──────
    # TTC로 매 프레임 새로 계산되므로 접근이 해소되면 자동으로 0.
    # 삼중 게이트(고속·접근(vRel<0)·TTC<임계)라 정속 추종·저속에선 무동작 → 평상시 부드러움 유지.
    if _lead.status and v_ego * CV.MS_TO_KPH >= HIGH_SPEED_BRAKE_KPH \
       and _lead.dRel > 0.0 and _lead.vRel < 0.0:
      _ttc = _lead.dRel / -_lead.vRel
      _tf_boost = float(interp(_ttc, [3.0, HIGH_SPEED_BRAKE_TTC], [HIGH_SPEED_TF_BOOST, 0.0]))
      _tf_boost *= float(interp(v_ego * CV.MS_TO_KPH, [HIGH_SPEED_BRAKE_KPH, 110.0], [0.5, 1.0]))
      _tf_boost *= lead_input_ramp
      self.t_follow += _tf_boost
    # ─────────────────────────────────────────────────────────────────────

    # planner에서 저장된 값 + lead 속도 함께 전달
    self.set_weights(v_ego=v_ego,
                     a_desired=self.a_desired,
                     prev_accel_constraint=self.prev_accel_constraint,
                     v_lead0=lead_xv_0[0, 1],
                     v_lead1=lead_xv_1[0, 1])

    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(
      lead_xv_0[:,1], self.x_sol[:,1], self.t_follow, self.stop_dist,
      krkeegan=self.applyLongDynamicCost)
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(
      lead_xv_1[:,1], self.x_sol[:,1], self.t_follow, self.stop_dist,
      krkeegan=self.applyLongDynamicCost)

    self.params[:,0] = MIN_ACCEL
    self.params[:,1] = self.max_a

    if self.mode == 'acc':
      self.params[:,5] = LEAD_DANGER_FACTOR

      v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)
      v_upper = v_ego + (T_IDXS * self.max_a * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1),
                                 v_lower,
                                 v_upper)
      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, self.t_follow, self.stop_dist)
      obstacles = [lead_0_obstacle, lead_1_obstacle, cruise_obstacle]
      if self.traffic_stop_active:
        obstacles.append(np.full(N+1, max(0.0, self.traffic_stop_distance)))
      x_obstacles = np.column_stack(obstacles)
      self.source = SOURCES[np.argmin(x_obstacles[0])]

      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))

      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)

    elif self.mode == 'blended':

      self.params[:,5] = 1.0

      obstacles = [lead_0_obstacle, lead_1_obstacle]
      if self.traffic_stop_active:
        obstacles.append(np.full(N+1, max(0.0, self.traffic_stop_distance)))
      x_obstacles = np.column_stack(obstacles)
      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))

      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)

      self.source = 'e2e' if x_and_cruise[1,0] < x_and_cruise[1,1] else 'cruise'

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner update')

    self.yref[:,1] = x
    self.yref[:,2] = v
    self.yref[:,3] = a
    self.yref[:,5] = j
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.prev_a)
    self.params[:,4] = self.t_follow
    self.params[:,6] = self.stop_dist

    self.run()
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    if self.mode == 'blended':
      if any((lead_0_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], self.t_follow, self.stop_dist)) - self.x_sol[:, 0] < 0.0):
        self.source = 'lead0'
      if any((lead_1_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], self.t_follow, self.stop_dist)) - self.x_sol[:, 0] < 0.0) and \
         (lead_1_obstacle[0] - lead_0_obstacle[0]):
        self.source = 'lead1'


  def run(self):
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:,1]
    self.a_solution = self.x_sol[:,2]
    self.j_solution = self.u_sol[:,0]

    self.prev_a = np.interp(T_IDXS + 0.05, T_IDXS, self.a_solution)

    t = sec_since_boot()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
  # AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)

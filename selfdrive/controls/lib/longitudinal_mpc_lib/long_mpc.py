#!/usr/bin/env python3
import os
import numpy as np

from cereal import car, log
from common.realtime import sec_since_boot, DT_MDL
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

XState = log.LongitudinalPlan.XState
ButtonType = car.CarState.ButtonEvent.Type

X_DIM = 3
U_DIM = 1
PARAM_DIM = 8
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

X_EGO_OBSTACLE_COST = 6.0
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
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0

LEAD_DEPARTURE_MIN_VREL = 0.3
LEAD_DEPARTURE_FULL_VREL = 1.65
LEAD_DEPARTURE_MAX_SPEED = 9.0
LEAD_DEPARTURE_BRAKING_ACCEL = -0.2
LEAD_DEPARTURE_MIN_COST_MULTIPLIER = 0.45


def get_lead_departure_cost_multiplier(v_ego, v_lead, a_lead, lead_status):
  """Reduce response-smoothing costs while a low-speed lead pulls away.

  This only changes how quickly the MPC can react. It does not move the lead,
  shorten the requested following distance, or raise the acceleration limit.
  The assist is removed as soon as the lead starts braking.
  """
  if (not lead_status or v_ego >= LEAD_DEPARTURE_MAX_SPEED or
      v_lead - v_ego <= LEAD_DEPARTURE_MIN_VREL or
      a_lead <= LEAD_DEPARTURE_BRAKING_ACCEL):
    return 1.0

  speed_factor = interp(v_ego, [0.0, LEAD_DEPARTURE_MAX_SPEED], [1.0, 0.0])
  relative_speed_factor = interp(v_lead - v_ego,
                                 [LEAD_DEPARTURE_MIN_VREL, LEAD_DEPARTURE_FULL_VREL],
                                 [0.0, 1.0])
  lead_accel_factor = interp(a_lead, [LEAD_DEPARTURE_BRAKING_ACCEL, 0.0], [0.0, 1.0])
  assist = speed_factor * relative_speed_factor * lead_accel_factor
  return 1.0 - (1.0 - LEAD_DEPARTURE_MIN_COST_MULTIPLIER) * assist


def get_stopped_equivalence_factor(v_lead, v_ego=0., t_follow=T_FOLLOW, stop_dist=STOP_DISTANCE, krkeegan=False,
                                   comfort_brake=COMFORT_BRAKE):
  if not krkeegan:
    return (v_lead**2) / (2 * comfort_brake)

  # KRKeegan: lead 거리값을 고의로 늘려 solver가 더 빠른 가속을 유발하도록 함
  v_diff_offset = 0
  if np.all(v_lead - v_ego > 0):
    v_diff_offset = ((v_lead - v_ego) * 1.)
    v_diff_offset = np.clip(v_diff_offset, 0, stop_dist / 2)
    v_diff_offset = np.maximum(v_diff_offset * ((10 - v_ego) / 10), 0)

  distance = (v_lead**2) / (2 * comfort_brake) + v_diff_offset
  return distance


def get_safe_obstacle_distance(v_ego, t_follow=T_FOLLOW, stop_dist=STOP_DISTANCE, comfort_brake=COMFORT_BRAKE):
  return (v_ego**2) / (2 * comfort_brake) + t_follow * v_ego + stop_dist

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
  comfort_brake = SX.sym('comfort_brake')

  model.p = vertcat(a_min, a_max, x_obstacle, prev_a, lead_t_follow, lead_danger_factor, stop_dist,
                    comfort_brake)

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
  comfort_brake = ocp.model.p[7]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow, stop_dist, comfort_brake)

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
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, T_FOLLOW, LEAD_DANGER_FACTOR, STOP_DISTANCE,
                                   COMFORT_BRAKE])

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
    self.softHoldMode = 1
    self.softHoldTimer = 0
    self.xState = XState.cruise
    self.prev_accel_constraint = True
    self.a_desired = 0.
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)

    # User-configured following time for each cruise-gap level.
    # longitudinal_planner.read_param() refreshes these values periodically.
    self.tfollow_gaps = None
    self.t_follow_speed_ratio = 1.2
    self.t_follow_decel_boost = 0.3
    self._tf_base_applied = 0.0
    self._tf_applied = 0.0
    self._tf_v_ego_kph = 0.0
    # ────────────────────────────────────────────────────────────────────

    # UI 파라미터로 조절되는 제동 튜닝값. longitudinal_planner.read_param()이 주기적으로 갱신한다.
    self.comfort_brake = COMFORT_BRAKE            # ComfortBrake     : 접근 감속 기준(m/s²)
    self.x_ego_obstacle_cost = X_EGO_OBSTACLE_COST  # XEgoObstacleCost: 차간거리 추종 강도
    # ────────────────────────────────────────────────────────────────────

    self.desired_distance = 0.0       # UI 표시용 목표 차간거리(m)
    self.traffic_stop_active = False
    self.traffic_stop_distance = 0.0
    self.stop_distance = STOP_DISTANCE
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
    self.params[:,7] = self.comfort_brake
    self.t_follow = T_FOLLOW
    self.stop_dist = self.stop_distance
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))
    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    self.softHoldTimer = 0
    self.xState = XState.cruise
    # timers
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0
    self.x0 = np.zeros(X_DIM)

    self._tf_base_applied = 0.0
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

  def get_cost_multipliers(self, v_lead0, v_lead1, a_lead0=0.0, lead0_status=False):
    v_ego = self.x0[1]
    v_ego_bps = [0, 10]
    TFs = [1.2, 1.45, 1.8]
    # TF에 의한 a, j, d cost 변경
    a_change_tf = interp(self.t_follow, TFs, [.8, 1., 1.1])
    j_ego_tf    = interp(self.t_follow, TFs, [.8, 1., 1.1])
    d_zone_tf   = interp(self.t_follow, TFs, [1.3, 1., 1.])

    # Restore the normal smoothing costs immediately when the primary lead
    # starts braking, even if it is still momentarily faster than ego.
    if lead0_status and a_lead0 <= LEAD_DEPARTURE_BRAKING_ACCEL:
      return (1.0, 1.0, d_zone_tf)

    j_ego_v_ego    = 1
    a_change_v_ego = 1
    if (v_lead0 - v_ego >= 0) and (v_lead1 - v_ego >= 0):
      v_ego_bps = [0, 10]
      j_ego_v_ego    = interp(v_ego, v_ego_bps, [0.05, 1.0])
      a_change_v_ego = interp(v_ego, v_ego_bps, [0.05, 1.0])

    j_ego    = min(j_ego_tf, j_ego_v_ego)
    a_change = min(a_change_tf, a_change_v_ego)
    return (a_change, j_ego, d_zone_tf)

  def set_weights(self, v_ego=0., a_desired=0., prev_accel_constraint=True,
                  v_lead0=0, v_lead1=0, a_lead0=0.0, lead0_status=False):
    self.prev_accel_constraint = prev_accel_constraint
    self.a_desired = a_desired

    if self.mode == 'acc':
      a_change_cost = A_CHANGE_COST if prev_accel_constraint else 40

      departure_cost_multiplier = get_lead_departure_cost_multiplier(
        v_ego, v_lead0, a_lead0, lead0_status)

      if self.applyLongDynamicCost:
        cost_multipliers = self.get_cost_multipliers(v_lead0, v_lead1, a_lead0, lead0_status)
        accel_change_multiplier = min(cost_multipliers[0], departure_cost_multiplier)
        jerk_multiplier = min(cost_multipliers[1], departure_cost_multiplier)
        cost_weights = [self.x_ego_obstacle_cost, X_EGO_COST, V_EGO_COST, A_EGO_COST,
                        a_change_cost * accel_change_multiplier,
                        J_EGO_COST * jerk_multiplier]
        constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST,
                                   DANGER_ZONE_COST * cost_multipliers[2]]
      else:
        cost_weights = [self.x_ego_obstacle_cost, X_EGO_COST, V_EGO_COST, A_EGO_COST,
                        a_change_cost * departure_cost_multiplier,
                        J_EGO_COST * departure_cost_multiplier]
        constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]

    elif self.mode == 'blended':
      # Keep a finite previous-acceleration cost through standstill and restart,
      # matching apilot-c2 and avoiding a target-acceleration step at re-engagement.
      a_change_cost = 40.0
      cost_weights = [0., 0.1, 0.2, 5.0, a_change_cost, 1.0]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, 50.0]
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

  def update(self, carstate, radarstate, controls, v_cruise, x, v, a, j, prev_accel_constraint=True):
    # engage 직후에는 직전 가속도 유지 비용(A_CHANGE_COST)을 빼서
    # 필요한 감속으로 곧바로 갈 수 있게 한다. (upstream 동작 복원)
    self.prev_accel_constraint = prev_accel_constraint
    v_ego = self.x0[1]
    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    # aPilot C2 soft hold state is owned by the longitudinal MPC.
    soft_hold_available = controls.enabled and self.softHoldMode > 0
    resume_pressed = any(event.pressed and event.type in (ButtonType.accelCruise, ButtonType.resumeCruise)
                         for event in carstate.buttonEvents)
    if not soft_hold_available:
      self.softHoldTimer = 0
      if self.xState == XState.softHold:
        self.xState = XState.cruise
    elif self.xState == XState.softHold:
      self.softHoldTimer = 0
      if carstate.gasPressed or resume_pressed:
        self.xState = XState.cruise
    elif carstate.brakePressed and v_ego < 0.1:
      self.softHoldTimer += 1
      if self.softHoldTimer * DT_MDL >= 0.7:
        self.xState = XState.softHold
    else:
      self.softHoldTimer = 0
      self.xState = XState.lead if self.status else XState.cruise

    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    # apilot-c2 방식: 가속/정속 중에만 갭별 TR과 속도 보정을 갱신하고,
    # 감속 중에는 직전 TR을 유지한다.
    cruise_gap = int(clip(controls.longCruiseGap, 1., 4.))
    safe_mode_factor = float(clip(controls.mySafeModeFactor, 0.5, 1.0))
    gap_values = self.tfollow_gaps if self.tfollow_gaps is not None else CRUISE_GAP_V
    v_ego_kph = v_ego * CV.MS_TO_KPH
    if self._tf_base_applied <= 0.0 or v_ego_kph >= self._tf_v_ego_kph:
      tr = interp(float(cruise_gap), CRUISE_GAP_BP, gap_values)
      speed_scale = interp(v_ego_kph, [0.0, 100.0], [1.0, self.t_follow_speed_ratio])
      self._tf_base_applied = max(0.6, float(tr * speed_scale * (2.0 - safe_mode_factor)))
    self._tf_v_ego_kph = v_ego_kph

    # Keep the base gap from shrinking while decelerating, then add a bounded
    # deceleration-only margin.  Increase it gradually so the virtual obstacle
    # cannot jump toward ego and cause a brake step; remove it immediately once
    # deceleration ends so acceleration recovery is not delayed.
    decel_margin = interp(carstate.aEgo, [-2.5, -1.0, -0.2, 0.0],
                           [0.25, 0.12, 0.02, 0.0]) * self.t_follow_decel_boost
    tf_target = self._tf_base_applied + decel_margin
    if self._tf_applied <= 0.0:
      self._tf_applied = tf_target
    elif tf_target > self._tf_applied:
      self._tf_applied = min(tf_target, self._tf_applied + 0.1 * DT_MDL)
    else:
      self._tf_applied = tf_target

    self.t_follow = self._tf_applied
    self.stop_dist = self.stop_distance * (2.0 - safe_mode_factor)
    self.desired_distance = float(self.t_follow * v_ego + self.stop_dist)   # UI 표시용

    # planner에서 저장된 값 + lead 속도 함께 전달
    self.set_weights(v_ego=v_ego,
                     a_desired=self.a_desired,
                     prev_accel_constraint=self.prev_accel_constraint,
                     v_lead0=lead_xv_0[0, 1],
                     v_lead1=lead_xv_1[0, 1],
                     a_lead0=radarstate.leadOne.aLeadK if radarstate.leadOne.status else 0.0,
                     lead0_status=radarstate.leadOne.status)

    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(
      lead_xv_0[:,1], self.x_sol[:,1], self.t_follow, self.stop_dist,
      krkeegan=self.applyLongDynamicCost, comfort_brake=self.comfort_brake)
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(
      lead_xv_1[:,1], self.x_sol[:,1], self.t_follow, self.stop_dist,
      krkeegan=self.applyLongDynamicCost, comfort_brake=self.comfort_brake)

    self.params[:,0] = MIN_ACCEL
    self.params[:,1] = self.max_a

    if self.mode == 'acc':
      self.params[:,5] = LEAD_DANGER_FACTOR

      v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)
      v_upper = v_ego + (T_IDXS * self.max_a * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1),
                                 v_lower,
                                 v_upper)
      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, self.t_follow, self.stop_dist, self.comfort_brake)
      obstacles = [lead_0_obstacle, lead_1_obstacle, cruise_obstacle]
      if self.traffic_stop_active:
        obstacles.append(np.full(N+1, max(0.0, self.traffic_stop_distance)))
      x_obstacles = np.column_stack(obstacles)
      self.source = SOURCES[np.argmin(x_obstacles[0])]

      # aPilot C2 ACC is obstacle-based. Do not feed the model's E2E
      # position/velocity/acceleration references into the ACC solve.
      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0

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
    self.params[:,7] = self.comfort_brake

    self.run()
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    if self.mode == 'blended':
      if any((lead_0_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], self.t_follow, self.stop_dist, self.comfort_brake)) - self.x_sol[:, 0] < 0.0):
        self.source = 'lead0'
      if any((lead_1_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], self.t_follow, self.stop_dist, self.comfort_brake)) - self.x_sol[:, 0] < 0.0) and \
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

import math

from cereal import log
from common.numpy_fast import interp
from common.realtime import DT_CTRL
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from selfdrive.controls.lib.latcontrol_pid import ERROR_RATE_FRAME
from selfdrive.modeld.constants import T_IDXS
from common.params import Params

import numpy as np

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]

# ── carrot 이식 : 예측 횡저크(lateral jerk) 를 friction 입력에 섞는다 ──────────
# 모델의 acceleration.y 예측을 미분해 앞으로의 저크를 구하고, 부호가 유지되는
# 구간에서 가장 작은 값을 골라 쓴다. 커브 진입 시 friction 을 미리 실어
# 초기 응답을 살리되, S 자처럼 부호가 뒤집히는 구간에서는 개입하지 않는다.
LAT_PLAN_MIN_IDX = 5


def _sign(x):
  return 1.0 if x > 0.0 else (-1.0 if x < 0.0 else 0.0)


def get_predicted_lateral_jerk(lat_accels, t_diffs):
  return (np.diff(lat_accels) / t_diffs).tolist()


def get_lookahead_value(future_vals, current_val):
  if len(future_vals) == 0:
    return current_val
  same_sign = [v for v in future_vals if _sign(v) == _sign(current_val)]
  if len(same_sign) < len(future_vals):
    return 0.0
  return min(same_sign + [current_val], key=lambda x: abs(x))
# ─────────────────────────────────────────────────────────────────────────────

# ── 긴 커브 안쪽 파고듦 대응 (적분 와인드업 억제) ────────────────────────────
# 원인: steerRatio 과대 등으로 생긴 미세 +오차가 긴 커브 내내 적분에 쌓여 토크가
#       서서히 증가 → 점점 안쪽으로 파고듦. 두 가지로 막는다.
#  (1) 소오차 적분 동결: 추종이 거의 맞은 뒤(잔차 작을 때) 새 누적 정지.
#  (2) 적분 누설(leak): 정상 추종 중에도 쌓인 적분을 시간상수로 흘려보냄(pid.py).
# 둘은 보완적이라 함께 사용한다(freeze=더 안 쌓이게 / leak=쌓인 걸 빼게).
#
# ★ 곡률 조건부로 변경 (2026-08-17)
#   위 두 장치는 '긴 커브'를 위한 것인데, 직진 구간에도 그대로 걸리면 반대 문제를
#   만든다. 노면 캠버·얼라인먼트로 생기는 완만한 드리프트는 오차가 작아서
#   데드존 안에 들어가 버리고, 그걸 메워야 할 적분은 동결 + 누설로 계속 빠진다.
#   교차로처럼 차선이 사라져 횡위치 기준이 없어지는 구간에서는 이 드리프트를
#   상류(경로)가 잡아주지 못하므로, 차가 차로를 대각선으로 흘러 나가게 된다.
#   → 요구 곡률의 크기로 커브/직진을 구분해서 강도를 보간한다.
#      커브에서는 종전과 동일(0.05 / leak 0.999), 직진에서는 거의 원본 동작
#      (0.005 / leak 0.99975)으로 두어 정상상태 편차를 적분이 메울 수 있게 한다.
#   ※ 기존 주석의 "0.999 @100Hz → τ≈1.0s" 는 오기다. τ = 1/(rate*(1-leak)) 이므로
#     0.999 → τ≈10s, 0.99975 → τ≈40s. 즉 누설은 원래 의도보다 훨씬 약하게
#     걸려 있었고, 실제로 적분을 막고 있던 주범은 데드존 쪽이다.
CURVE_CURV_BP = [0.0015, 0.0060]   # 1/m. 0.0015≈R667m(완만) / 0.0060≈R167m(뚜렷한 커브)
ERR_FREEZE_DEADZONE_V = [0.005, 0.05]   # 토크 단위(0~1). [직진, 커브]
I_LEAK_FACTOR_V = [0.99975, 0.999]      # @100Hz τ≈[40s, 10s]. [직진, 커브]

# 커브 판정 필터: 올라갈 때는 빠르게(0.3s), 내려올 때는 느리게(2.0s) 따라간다.
# 커브 도중 잠깐 곡률이 펴져도 데드존이 풀렸다 걸렸다 하지 않게 하고, 커브를
# 빠져나온 뒤에는 2초 정도 여유를 두고 직진 설정으로 되돌아가게 하기 위한 것.
CURVE_FILTER_RISE_TAU = 0.3
CURVE_FILTER_FALL_TAU = 2.0


class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.torque_params = CP.lateralTuning.torque
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             k_f=self.torque_params.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max,
                             i_leak_factor=I_LEAK_FACTOR_V[0])
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = self.torque_params.useSteeringAngle
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg

    # 커브 판정용 요구곡률 크기 필터 (0 = 직진)
    self.curve_mag = 0.0

    # ── 토크 튜닝을 Params 로 (carrot 방식). nTune 파일은 더 이상 쓰지 않는다 ──
    self.params = Params()
    self.frame = 0
    self.lateral_torque_custom = 0
    self.latAccelFactor_default = self.torque_params.latAccelFactor
    self.friction_default = self.torque_params.friction
    self.kp_default = self.torque_params.kp
    self.ki_default = self.torque_params.ki
    self.kf_default = self.torque_params.kf
    self.kd_default = self.torque_params.kd

    # friction 입력 계수 (carrot 기본값)
    self.lat_accel_friction_factor = 0.7
    self.lat_jerk_friction_factor = 0.4
    self.desired_lat_jerk_time = 0.3
    self.t_diffs = np.diff(T_IDXS)
    self.friction_upper_idx = len(T_IDXS)
    self.predicted_lateral_jerk = []
    self.predicted_lateral_jerk_frame_id = -1
    self.read_torque_params(force=True)

  def reset(self):
    super().reset()
    # Do not carry steering learned before a stop into the next launch.
    # Hyundai's command-side torque limiter will ramp the fresh output from 0.
    self.pid.reset()
    self.curve_mag = 0.0

  def _pget(self, key, default):
    try:
      v = self.params.get(key, encoding="utf8")
      return float(v) if v not in (None, "") else default
    except (TypeError, ValueError):
      return default

  def read_torque_params(self, force=False):
    custom = int(self._pget("LateralTorqueCustom", 0))
    prev_custom = getattr(self, "lateral_torque_custom", 0)

    if custom > 0:
      # 수동 토크값(설정 화면)만 사용한다. 자동 학습 경로는 제거되었다.
      # Missing Params must preserve the active vehicle tune. Hard-coded
      # generic fallbacks made Custom mode jump away from the Genesis DH tune.
      self.torque_params.latAccelFactor = self._pget(
        "LateralTorqueAccelFactor", round(self.latAccelFactor_default * 1000.0)) * 0.001
      self.torque_params.friction = self._pget(
        "LateralTorqueFriction", round(self.friction_default * 1000.0)) * 0.001
      self.pid._k_p = [[0], [self._pget(
        "LateralTorqueKpV", round(self.kp_default * 100.0)) * 0.01]]
      self.pid._k_i = [[0], [self._pget(
        "LateralTorqueKiV", round(self.ki_default * 100.0)) * 0.01]]
      self.pid.k_f = self._pget(
        "LateralTorqueKf", round(self.kf_default * 100.0)) * 0.01
      self.pid._k_d = [[0], [self._pget(
        "LateralTorqueKd", round(self.kd_default * 100.0)) * 0.01]]
    elif prev_custom > 0 or force:
      self.torque_params.latAccelFactor = self.latAccelFactor_default
      self.torque_params.friction = self.friction_default
      self.pid._k_p = [[0], [self.kp_default]]
      self.pid._k_i = [[0], [self.ki_default]]
      self.pid.k_f = self.kf_default
      self.pid._k_d = [[0], [self.kd_default]]
    self.lateral_torque_custom = custom

    self.lat_accel_friction_factor = self._pget("LatAccelFrictionFactor", 70) * 0.01
    self.lat_jerk_friction_factor = self._pget("LatJerkFrictionFactor", 40) * 0.01
    self.desired_lat_jerk_time = max(
      0.1, self._pget("SteerActuatorDelay", 10) * 0.01 + 0.3)
    self.friction_upper_idx = next(
      (i for i, t in enumerate(T_IDXS) if t > max(self.desired_lat_jerk_time, 0.1)),
      len(T_IDXS))

  def _update_curve_state(self, desired_curvature):
    """요구곡률 크기를 비대칭 필터로 추종해 커브/직진 강도(0~1)를 만든다."""
    target = abs(float(desired_curvature))
    tau = CURVE_FILTER_RISE_TAU if target > self.curve_mag else CURVE_FILTER_FALL_TAU
    alpha = min(1.0, DT_CTRL / tau)
    self.curve_mag += alpha * (target - self.curve_mag)
    return interp(self.curve_mag, CURVE_CURV_BP, [0.0, 1.0])

  def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk, model_data=None):
    self.frame += 1
    if self.frame % 100 == 0:     # 1초 주기 라이브 반영
      self.read_torque_params()
    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_torque = 0.0
      pid_log.active = False
      angle_steers_des = 0.0
      pid_log.latAccelFactor = self.torque_params.latAccelFactor
      pid_log.latAccelOffset = self.torque_params.latAccelOffset
      pid_log.friction = self.torque_params.friction
      # 비활성 구간에서 커브 상태를 들고 있으면 재인게이지 직후 직진에서도
      # 커브용 강한 데드존이 걸린다.
      self.curve_mag = 0.0
    else:
      if self.use_steering_angle:
        actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      else:
        actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      #desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      low_speed_factor = interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y)**2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      error = setpoint - measurement
      gravity_adjusted_lateral_accel = desired_lateral_accel - params.roll * ACCELERATION_DUE_TO_GRAVITY
      pid_log.error = self.torque_from_lateral_accel(error, self.torque_params, error,
                                                     lateral_accel_deadzone, friction_compensation=False)

      # ── friction 입력 : 횡가속도 오차 + 앞으로의 횡저크 (carrot 이식) ──
      accel_error = desired_lateral_accel - actual_lateral_accel
      lookahead_lateral_jerk = 0.0
      # modelV2 updates at 20 Hz while lateral control runs at 100 Hz. Skip the
      # calculation entirely when steering-angle control will discard it, and
      # otherwise reuse one prediction for each model period.
      if not self.use_steering_angle and model_data is not None and \
         len(model_data.acceleration.y) >= len(T_IDXS):
        try:
          model_frame_id = int(model_data.frameId)
          if model_frame_id != self.predicted_lateral_jerk_frame_id or \
             len(self.predicted_lateral_jerk) != len(self.t_diffs):
            self.predicted_lateral_jerk = get_predicted_lateral_jerk(
              model_data.acceleration.y, self.t_diffs)
            self.predicted_lateral_jerk_frame_id = model_frame_id
          desired_lateral_jerk = (float(interp(self.desired_lat_jerk_time, T_IDXS, model_data.acceleration.y))
                                  - desired_lateral_accel) / self.desired_lat_jerk_time
          lookahead_lateral_jerk = get_lookahead_value(
              self.predicted_lateral_jerk[LAT_PLAN_MIN_IDX:self.friction_upper_idx],
              desired_lateral_jerk)
        except (ValueError, ZeroDivisionError):
          lookahead_lateral_jerk = 0.0

      friction_input = (self.lat_accel_friction_factor * accel_error
                        + self.lat_jerk_friction_factor * lookahead_lateral_jerk)

      ff = self.torque_from_lateral_accel(gravity_adjusted_lateral_accel, self.torque_params,
                                          friction_input,
                                          lateral_accel_deadzone, friction_compensation=True)

      # 소오차 구간 적분 동결(anti-windup): 추종이 거의 맞은 뒤에도 미세 +오차가
      # 긴 커브 내내 적분에 쌓여 서서히 안쪽으로 파고드는 현상을 차단한다.
      # error 는 횡가속도 오차의 토크 환산값(pid_log.error). 작은 잔차는 새 누적 정지.
      # (이미 쌓인 적분은 pid.py 의 i_leak_factor 로 계속 흘러나간다)
      #
      # ★ 강도를 커브 정도에 따라 보간한다. 직진에서는 데드존을 거의 0 으로,
      #   누설 시간상수를 길게 두어 정상상태 편차(캠버·얼라인먼트)를 적분이
      #   메울 수 있게 한다. 커브에서는 종전 값 그대로 유지.
      curve_ratio = self._update_curve_state(desired_curvature)
      err_freeze_deadzone = interp(curve_ratio, [0.0, 1.0], ERR_FREEZE_DEADZONE_V)
      self.pid.i_leak_factor = interp(curve_ratio, [0.0, 1.0], I_LEAK_FACTOR_V)

      low_error = abs(pid_log.error) < err_freeze_deadzone
      freeze_integrator = steer_limited or CS.steeringPressed or CS.vEgo < 5 or low_error
      output_torque = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = -output_torque
      pid_log.actualLateralAccel = actual_lateral_accel
      pid_log.desiredLateralAccel = desired_lateral_accel
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited)

      pid_log.latAccelFactor = self.torque_params.latAccelFactor
      pid_log.latAccelOffset = self.torque_params.latAccelOffset
      pid_log.friction = self.torque_params.friction

      angle_steers_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll)) + params.angleOffsetDeg

    #TODO left is positive in this convention
    return -output_torque, angle_steers_des, pid_log

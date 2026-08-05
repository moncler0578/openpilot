import copy
import random
from math import sqrt

import numpy as np
from common.numpy_fast import clip, interp, mean
from cereal import car
from common.realtime import DT_CTRL
from common.conversions import Conversions as CV
from selfdrive.car.hyundai.values import Buttons
from common.params import Params
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, V_CRUISE_MIN, V_CRUISE_DELTA_KM, V_CRUISE_DELTA_MI, CONTROL_N
from selfdrive.controls.lib.low_speed_long import read_cruise_speed_min
from selfdrive.controls.lib.lateral_planner import TRAJECTORY_SIZE
from selfdrive.controls.lib.carrot_navi_atc import CarrotNaviAtc

from selfdrive.road_speed_limiter import road_speed_limiter_get_max_speed, road_speed_limiter_get_active, \
  get_road_speed_limiter

SYNC_MARGIN = 3.
# do not modify
MIN_SET_SPEED_KPH = V_CRUISE_MIN
MAX_SET_SPEED_KPH = V_CRUISE_MAX

ALIVE_COUNT = [8, 10]
WAIT_COUNT = [12, 14, 16, 18]
AliveIndex = 0
WaitIndex = 0

MIN_CURVE_SPEED = 32. * CV.KPH_TO_MS

EventName = car.CarEvent.EventName

ButtonType = car.CarState.ButtonEvent.Type
ButtonPrev = ButtonType.unknown
ButtonCnt = 0
LongPressed = False

class SccSmoother:

  @staticmethod
  def get_alive_count():
    global AliveIndex
    count = ALIVE_COUNT[AliveIndex]
    AliveIndex += 1
    if AliveIndex >= len(ALIVE_COUNT):
      AliveIndex = 0
    return count

  @staticmethod
  def get_wait_count():
    global WaitIndex
    count = WAIT_COUNT[WaitIndex]
    WaitIndex += 1
    if WaitIndex >= len(WAIT_COUNT):
      WaitIndex = 0
    return count

  def kph_to_clu(self, kph):
    return int(kph * CV.KPH_TO_MS * self.speed_conv_to_clu)

  def __init__(self):

    self.params = Params()
    self.read_param()

    self.param_read_counter = 0

    self.speed_conv_to_ms = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
    self.speed_conv_to_clu = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

    self.min_set_speed_clu = self.kph_to_clu(self.min_set_speed_kph)
    self.max_set_speed_clu = self.kph_to_clu(MAX_SET_SPEED_KPH)

    self.target_speed = 0.

    self.started_frame = 0
    self.wait_timer = 0
    self.alive_timer = 0
    self.btn = Buttons.NONE

    self.alive_count = ALIVE_COUNT
    random.shuffle(WAIT_COUNT)

    self.slowing_down = False
    self.slowing_down_alert = False
    self.slowing_down_sound_alert = False
    self.active_cam = False
    self.over_speed_limit = False

    self.max_speed_clu = 0.
    self.limited_lead = False

    self.curve_speed_ms = 0.
    self.stock_weight = 0.

    self.auto_speed_up_ratio = 0.0
    self._pause_auto_speed_up = False
    self.auto_gas_resume_guard = True
    self.carrot_atc = CarrotNaviAtc()
    self.initial_gap_applied = False

  def read_param(self):
    self.longcontrol = self.params.get_bool('LongControlEnabled')
    self.slow_on_curves = self.params.get_bool('SccSmootherSlowOnCurves')
    self.sync_set_speed_while_gas_pressed = self.params.get_bool('SccSmootherSyncGasPressed')
    self.is_metric = self.params.get_bool('IsMetric')
    self.e2e_long = self.params.get_bool('ExperimentalMode')
    self.autoascc = self.params.get_bool('AutoAscc')
    self.auto_gas_resume_guard = self.params.get_bool('AutoGasResumeGuard')
    self.min_set_speed_kph = read_cruise_speed_min(self.params)
    if hasattr(self, "speed_conv_to_clu"):
      self.min_set_speed_clu = self.kph_to_clu(self.min_set_speed_kph)
    initial_gap = int(clip(self.params.get_int("InitialCruiseGap"), 0, 4))
    if hasattr(self, "initial_cruise_gap") and initial_gap != self.initial_cruise_gap:
      self.initial_gap_applied = False
    self.initial_cruise_gap = initial_gap
    # AutoSpeedUptoRoadSpeedLimit : 도로제한속도 대비 자동 증속 상한(%). 0 = 사용 안함
    try:
      self.auto_speed_up_ratio = float(self.params.get("AutoSpeedUptoRoadSpeedLimit", encoding="utf8") or "0") * 0.01
    except (TypeError, ValueError):
      self.auto_speed_up_ratio = 0.0
    try:
      self.carrot_atc_mode = int(self.params.get("CarrotAutoTurnControl", encoding="utf8") or "0")
      self.carrot_atc_speed = float(self.params.get("CarrotAutoTurnSpeed", encoding="utf8") or "30")
      self.carrot_atc_end_time = float(self.params.get("CarrotAutoTurnEndTime", encoding="utf8") or "6")
    except (TypeError, ValueError):
      self.carrot_atc_mode, self.carrot_atc_speed, self.carrot_atc_end_time = 0, 30.0, 6.0

  def reset(self):

    self.wait_timer = 0
    self.alive_timer = 0
    self.btn = Buttons.NONE
    self.target_speed = 0.

    self.max_speed_clu = 0.
    self.curve_speed_ms = 0.

    self.slowing_down = False
    self.slowing_down_alert = False
    self.slowing_down_sound_alert = False

  @staticmethod
  def create_clu11(packer, bus, clu11, button):
    values = copy.copy(clu11)
    values["CF_Clu_CruiseSwState"] = button
    values["CF_Clu_AliveCnt1"] = (values["CF_Clu_AliveCnt1"] + 1) % 0x10
    return packer.make_can_msg("CLU11", bus, values)

  def is_active(self, frame):
    return frame - self.started_frame <= max(ALIVE_COUNT) + max(WAIT_COUNT)

  def inject_events(self, events):
    if self.slowing_down_sound_alert:
      self.slowing_down_sound_alert = False
      events.add(EventName.slowingDownSpeedSound)
    elif self.slowing_down_alert:
      events.add(EventName.slowingDownSpeed)

  def get_initial_gap_button(self, ascc_enabled, CS):
    if self.initial_cruise_gap == 0:
      self.initial_gap_applied = True
      return Buttons.NONE

    if self.initial_gap_applied:
      return Buttons.NONE

    if CS.cruise_buttons == Buttons.GAP_DIST:
      self.initial_gap_applied = True
      if self.btn == Buttons.GAP_DIST:
        self.btn = Buttons.NONE
        self.alive_timer = 0
        self.wait_timer = 0
      return Buttons.NONE

    if not ascc_enabled:
      return Buttons.NONE

    current_gap = int(clip(CS.out.cruiseGap, 1, 4))
    if current_gap == self.initial_cruise_gap:
      self.initial_gap_applied = True
      return Buttons.NONE

    return Buttons.GAP_DIST

  def cal_max_speed(self, frame, CC, CS, sm, clu11_speed, controls):

    # kph
    road_speed_limiter = get_road_speed_limiter()
    apply_limit_speed, road_limit_speed, left_dist, first_started, max_speed_log = \
      road_speed_limiter.get_max_speed(clu11_speed, self.is_metric)

    curv_limit = 0
    self.cal_curve_speed(sm, CS.out.vEgo, frame)
    if self.slow_on_curves and self.curve_speed_ms >= MIN_CURVE_SPEED:
      max_speed_clu = min(controls.v_cruise_kph * CV.KPH_TO_MS, self.curve_speed_ms) * self.speed_conv_to_clu
      curv_limit = int(max_speed_clu)
    else:
      max_speed_clu = self.kph_to_clu(controls.v_cruise_kph)

    self.active_cam = road_limit_speed > 0 and left_dist > 0

    if road_speed_limiter.roadLimitSpeed is not None:
      camSpeedFactor = clip(road_speed_limiter.roadLimitSpeed.camSpeedFactor, 1.0, 1.1)
      self.over_speed_limit = road_speed_limiter.roadLimitSpeed.camLimitSpeedLeftDist > 0 and \
                              0 < road_limit_speed * camSpeedFactor < clu11_speed + 2
    else:
      self.over_speed_limit = False

    max_speed_log = ""

    if apply_limit_speed >= self.kph_to_clu(10):

      if first_started:
        self.max_speed_clu = clu11_speed

      max_speed_clu = min(max_speed_clu, apply_limit_speed)

      if clu11_speed > apply_limit_speed:

        if not self.slowing_down_alert and not self.slowing_down:
          self.slowing_down_sound_alert = True
          self.slowing_down = True

        self.slowing_down_alert = True

      else:
        self.slowing_down_alert = False

    else:
      self.slowing_down_alert = False
      self.slowing_down = False

    lead_speed = self.get_long_lead_speed(CS, clu11_speed, sm)

    if lead_speed >= self.min_set_speed_clu:
      if lead_speed < max_speed_clu:
        max_speed_clu = min(max_speed_clu, lead_speed)

        if not self.limited_lead:
          self.max_speed_clu = clu11_speed + 3.
          self.limited_lead = True
    else:
      self.limited_lead = False

    if self.carrot_atc_mode in (2, 3) and not CS.out.brakePressed:
      atc_limit_kph = self.carrot_atc.speed_limit_kph(
        self.carrot_atc.update(), self.carrot_atc_speed, self.carrot_atc_end_time)
      if atc_limit_kph is not None:
        max_speed_clu = min(max_speed_clu, self.kph_to_clu(atc_limit_kph))

    self.update_max_speed(int(max_speed_clu + 0.5),
                          curv_limit != 0 and curv_limit == int(max_speed_clu))

    return road_limit_speed, left_dist, max_speed_log

  def update(self, enabled, can_sends, packer, CC, CS, frame, controls):

    if self.param_read_counter % 100 == 0:
      self.read_param()
    self.param_read_counter += 1

    # mph or kph
    clu11_speed = CS.clu11["CF_Clu_Vanz"]

    road_limit_speed, left_dist, max_speed_log = self.cal_max_speed(frame, CC, CS, controls.sm, clu11_speed, controls)

    # kph
    controls.applyMaxSpeed = float(clip(CS.cruiseState_speed * CV.MS_TO_KPH, self.min_set_speed_kph,
                                                self.max_speed_clu * self.speed_conv_to_ms * CV.MS_TO_KPH))
    CC.sccSmoother.longControl = self.longcontrol
    CC.sccSmoother.applyMaxSpeed = controls.applyMaxSpeed
    CC.sccSmoother.cruiseMaxSpeed = controls.v_cruise_kph

    ascc_enabled = CS.acc_mode and enabled and CS.cruiseState_enabled \
                   and 1 < CS.cruiseState_speed < 255 and not CS.brake_pressed

    # apilot-c2 style: request the configured initial gap with the real Hyundai
    # GAP button message. Stop automating as soon as the target is reported;
    # a physical GAP press always cancels the one-time adjustment so the
    # driver's selection has priority.
    initial_gap_button = self.get_initial_gap_button(ascc_enabled, CS)

    # Auto-resume Cruise Set Speed by JangPoo
    dRel = 0.
    lead = self.get_lead(controls.sm)
    if lead is not None:
      dRel = lead.dRel

    # Auto-resume Cruise Set Speed by JangPoo
    ascc_auto_set = enabled and (clu11_speed > 30 or (CS.obj_valid and dRel > 1)) \
                    and CS.gas_pressed and CS.prev_cruiseState_speed and not CS.cruiseState_speed

    # ── carrot(_gas_pressed_count == -1) 이식 : 재개 안전조건 ────────────────
    #   깜빡이를 켰거나 앞차가 지나치게 가까우면 자동 재개하지 않는다.
    #   캐롯은 이 경우 크루즈를 끄지만, 이 포크는 순정 SCC 가 크루즈를 쥐고 있어
    #   "재개하지 않음"까지만 한다.
    if ascc_auto_set and self.auto_gas_resume_guard:
      if CS.leftBlinker or CS.rightBlinker:
        ascc_auto_set = False
      elif 0 < dRel < CS.out.vEgo * 0.8:
        ascc_auto_set = False

    if not self.longcontrol:
      if (not ascc_enabled or CS.standstill or CS.cruise_buttons != Buttons.NONE) and not ascc_auto_set:
        self.reset()
        self.wait_timer = max(ALIVE_COUNT) + max(WAIT_COUNT)
        return

    if not ascc_enabled and not ascc_auto_set:
      self.reset()

    self.cal_target_speed(CS, clu11_speed, controls)
    self.auto_speed_up(CS, controls, road_limit_speed)

    CC.sccSmoother.logMessage = max_speed_log

    if self.wait_timer > 0:
      self.wait_timer -= 1
    elif (ascc_enabled and not CS.out.cruiseState.standstill) or ascc_auto_set:
      if self.alive_timer == 0:
        if initial_gap_button != Buttons.NONE and CS.cruise_buttons == Buttons.NONE:
          self.btn = initial_gap_button
        elif ascc_enabled:
          if self.autoascc:
            self.btn = self.get_button(CS.cruiseState_speed * self.speed_conv_to_clu)
        elif ascc_auto_set and clu11_speed < 30:
          if self.autoascc:
            self.btn = Buttons.SET_DECEL
        else:
          if self.autoascc:
            self.btn = Buttons.RES_ACCEL
        self.alive_count = SccSmoother.get_alive_count()

      if self.btn != Buttons.NONE:

        can_sends.append(SccSmoother.create_clu11(packer, CS.scc_bus, CS.clu11, self.btn))

        if self.alive_timer == 0:
          self.started_frame = frame

        self.alive_timer += 1

        if self.alive_timer >= self.alive_count:
          self.alive_timer = 0
          self.wait_timer = SccSmoother.get_wait_count()
          self.btn = Buttons.NONE
      else:
        if self.longcontrol and self.target_speed >= self.min_set_speed_clu:
          self.target_speed = 0.
    else:
      if self.longcontrol:
        self.target_speed = 0.

  def get_button(self, current_set_speed):

    if self.target_speed < self.min_set_speed_clu:
      return Buttons.NONE

    error = self.target_speed - current_set_speed
    if abs(error) < 0.9:
      return Buttons.NONE

    return Buttons.RES_ACCEL if error > 0 else Buttons.SET_DECEL

  def auto_speed_up(self, CS, controls, road_limit_speed):
    """caroot(_auto_speed_up) 이식.
    앞차가 내 설정속도보다 빠르고 도로제한속도 아래이면 설정속도를 자동으로 올린다.
    선행차가 없으면 아무것도 하지 않는다 (빈 도로에서 멋대로 올라가지 않게).
    """
    if self.auto_speed_up_ratio <= 0.0 or road_limit_speed <= 0:
      return

    # SET 을 누르면 자동증속 일시중단, RES 를 누르면 해제
    if CS.cruise_buttons == Buttons.SET_DECEL:
      self._pause_auto_speed_up = True
    elif CS.cruise_buttons == Buttons.RES_ACCEL:
      self._pause_auto_speed_up = False
    if self._pause_auto_speed_up:
      return

    road_limit_kph = road_limit_speed * self.auto_speed_up_ratio
    if road_limit_kph < 1.0:
      return

    lead = self.get_lead(controls.sm)
    if lead is None:
      return
    v_lead_kph = lead.vLeadK * CV.MS_TO_KPH

    # 롱컨에서는 controls.v_cruise_kph 가 매 프레임 순정 설정속도로 덮어써지므로
    # (update_cruise_buttons) 직접 쓰지 않고 target_speed 를 올려
    # get_button() 이 RES_ACCEL 을 내보내도록 한다. → AutoAscc 가 켜져 있어야 동작.
    set_speed_kph = CS.cruiseState_speed * CV.MS_TO_KPH
    if set_speed_kph <= 0:
      return

    if v_lead_kph + 5 > set_speed_kph \
       and set_speed_kph < road_limit_kph \
       and lead.dRel < 60:
      new_kph = min(set_speed_kph + 5, road_limit_kph)
      self.target_speed = max(self.target_speed, self.kph_to_clu(new_kph))

  def get_lead(self, sm):

    radar = sm['radarState']
    if radar.leadOne.status:
      return radar.leadOne

    return None

  def get_long_lead_speed(self, CS, clu11_speed, sm):
    # Lead following belongs to LongitudinalMpc. Applying another lead-derived
    # set-speed reduction here made the same lead deceleration happen twice.
    return 0

  def cal_curve_speed(self, sm, v_ego, frame):

    if frame % 20 == 0:
      md = sm['modelV2']
      if len(md.position.x) == TRAJECTORY_SIZE and len(md.position.y) == TRAJECTORY_SIZE:
        x = md.position.x
        y = md.position.y
        dy = np.gradient(y, x)
        d2y = np.gradient(dy, x)
        curv = d2y / (1 + dy ** 2) ** 1.5

        start = int(interp(v_ego, [10., 27.], [10, TRAJECTORY_SIZE-10]))
        curv = curv[start:min(start+10, TRAJECTORY_SIZE)]
        a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
        v_curvature = np.sqrt(a_y_max / np.clip(np.abs(curv), 1e-4, None))
        model_speed = np.mean(v_curvature) * 0.85

        if model_speed < v_ego:
          self.curve_speed_ms = float(max(model_speed, MIN_CURVE_SPEED))
        else:
          self.curve_speed_ms = 255.

        if np.isnan(self.curve_speed_ms):
          self.curve_speed_ms = 255.
      else:
        self.curve_speed_ms = 255.

  def cal_target_speed(self, CS, clu11_speed, controls):

    if not self.longcontrol:
      if CS.gas_pressed and self.sync_set_speed_while_gas_pressed and CS.cruise_buttons == Buttons.NONE:
        if clu11_speed + SYNC_MARGIN > self.kph_to_clu(controls.v_cruise_kph):
          set_speed = clip(clu11_speed + SYNC_MARGIN, self.min_set_speed_clu, self.max_set_speed_clu)
          controls.v_cruise_kph = set_speed * self.speed_conv_to_ms * CV.MS_TO_KPH

      self.target_speed = self.kph_to_clu(controls.v_cruise_kph)

      if self.max_speed_clu > self.min_set_speed_clu:
        self.target_speed = clip(self.target_speed, self.min_set_speed_clu, self.max_speed_clu)

    elif CS.cruiseState_enabled:
      if CS.gas_pressed and self.sync_set_speed_while_gas_pressed and CS.cruise_buttons == Buttons.NONE:
        if clu11_speed + SYNC_MARGIN > self.kph_to_clu(controls.v_cruise_kph):
          set_speed = clip(clu11_speed + SYNC_MARGIN, self.min_set_speed_clu, self.max_set_speed_clu)
          self.target_speed = set_speed

  def update_max_speed(self, max_speed, limited_curv):

    if not self.longcontrol or self.max_speed_clu <= 0:
      self.max_speed_clu = max_speed
    else:
      kp = 0.01 if limited_curv else 0.01
      error = max_speed - self.max_speed_clu
      self.max_speed_clu = self.max_speed_clu + error * kp

  def get_apply_accel(self, CS, sm, accel, stopping):
    # Carrot planner/longcontrol already owns start, stop and delay
    # compensation. Pass its command through unchanged to Hyundai SCC.
    return accel

  def get_stock_cam_accel(self, apply_accel, stock_accel, scc11):
    stock_cam = scc11["Navi_SCC_Camera_Act"] == 2 and scc11["Navi_SCC_Camera_Status"] == 2
    if stock_cam:
      self.stock_weight += DT_CTRL / 3.
    else:
      self.stock_weight -= DT_CTRL / 3.

    self.stock_weight = clip(self.stock_weight, 0., 1.)

    accel = stock_accel * self.stock_weight + apply_accel * (1. - self.stock_weight)
    return min(accel, apply_accel), stock_cam

  @staticmethod
  def update_cruise_buttons(controls, CS, longcontrol):  # called by controlds's state_transition

    car_set_speed = CS.cruiseState.speed * CV.MS_TO_KPH
    is_cruise_enabled = car_set_speed != 0 and car_set_speed != 255 and CS.cruiseState.enabled and controls.CP.pcmCruise

    if is_cruise_enabled:
      if longcontrol:
        v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
      else:
        v_cruise_kph = SccSmoother.update_v_cruise(controls.v_cruise_kph, CS.buttonEvents, controls.enabled,
                                                   controls.is_metric, controls.cruise_speed_min)
    else:
      v_cruise_kph = 0

    if controls.is_cruise_enabled != is_cruise_enabled:
      controls.is_cruise_enabled = is_cruise_enabled

      if controls.is_cruise_enabled:
        v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
      else:
        v_cruise_kph = 0

      controls.LoC.reset(v_pid=CS.vEgo)

    controls.v_cruise_kph = v_cruise_kph

  @staticmethod
  def update_v_cruise(v_cruise_kph, buttonEvents, enabled, metric, min_set_speed_kph=MIN_SET_SPEED_KPH):

    global ButtonCnt, LongPressed, ButtonPrev
    if enabled:
      if ButtonCnt:
        ButtonCnt += 1
      for b in buttonEvents:
        if b.pressed and not ButtonCnt and (b.type == ButtonType.accelCruise or b.type == ButtonType.decelCruise):
          ButtonCnt = 1
          ButtonPrev = b.type
        elif not b.pressed and ButtonCnt:
          if not LongPressed and b.type == ButtonType.accelCruise:
            v_cruise_kph += 1 if metric else 1 * CV.MPH_TO_KPH
          elif not LongPressed and b.type == ButtonType.decelCruise:
            v_cruise_kph -= 1 if metric else 1 * CV.MPH_TO_KPH
          LongPressed = False
          ButtonCnt = 0
      if ButtonCnt > 70:
        LongPressed = True
        V_CRUISE_DELTA = V_CRUISE_DELTA_KM if metric else V_CRUISE_DELTA_MI
        if ButtonPrev == ButtonType.accelCruise:
          v_cruise_kph += V_CRUISE_DELTA - v_cruise_kph % V_CRUISE_DELTA
        elif ButtonPrev == ButtonType.decelCruise:
          v_cruise_kph -= V_CRUISE_DELTA - -v_cruise_kph % V_CRUISE_DELTA
        ButtonCnt %= 70
      v_cruise_kph = clip(v_cruise_kph, min_set_speed_kph, MAX_SET_SPEED_KPH)

    return v_cruise_kph

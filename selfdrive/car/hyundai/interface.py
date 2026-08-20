#!/usr/bin/env python3
import time
from typing import List

from cereal import car
from common.conversions import Conversions as CV
from selfdrive.car.hyundai.values import CAR, Buttons, CarControllerParams
from selfdrive.car.hyundai.cruise_buttons import button_transitions, main_button_transitions
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from common.params import Params
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN

GearShifter = car.CarState.GearShifter
EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type
BUTTONS_DICT = {
  Buttons.RES_ACCEL: ButtonType.accelCruise,
  Buttons.SET_DECEL: ButtonType.decelCruise,
  Buttons.GAP_DIST: ButtonType.gapAdjustCruise,
  Buttons.CANCEL: ButtonType.cancel,
}

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.cp2 = self.CS.get_can2_parser(CP)
    # AutoLaneChangeSpeed 실시간 반영용 (VIP 패널에서 10km/h 단위 조절)
    self._lc_speed_params = Params()
    self.lane_change_speed_min = LANE_CHANGE_SPEED_MIN
    self._lc_speed_params_t = 0.0

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[], disable_radar=False):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.openpilotLongitudinalControl = Params().get_bool('LongControlEnabled')

    ret.carName = "hyundai"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy, 0)]

    tire_stiffness_factor = 1.
    ret.maxSteeringAngleDeg = 1000.

    ret.steerFaultMaxAngle = 85
    # Match apilot-c2: 89 valid requests followed by at most two reset frames.
    ret.steerFaultMaxFrames = 89

    ret.disableLateralLiveTuning = False

    # Hyundai/Kia always uses the torque lateral controller.
    #
    # 2026-08-19: 원래 여기에 `except:` 로 모든 예외를 삼키고 2.5/0.01 폴백을
    # 쓰는 코드가 있었다. 그 탓에 torque_data/override.yaml 파일명이 깨져 있던
    # 동안(보이지 않는 U+200E → FileNotFoundError) 차량 정의 토크값이 한 번도
    # 로드되지 않았는데도 아무 표시가 없었다. ajouatom/apilot c2-master 및
    # 업스트림과 동일하게 폴백 없이 그대로 호출한다. 로딩이 실패하면 조용히
    # 다른 값으로 달리는 대신 기동 단계에서 즉시 드러난다.
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.steerRatio = 16.5
    ret.steerActuatorDelay = 0.25
    ret.steerLimitTimer = 3.0

    # longitudinal
    # Carrot longitudinal controller uses one live-adjustable PID gain instead
    # of speed-banded gains. This also enables the settings-panel values to be
    # reloaded by longcontrol.py while driving.
    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kpV = [1.0]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.0]
    ret.longitudinalTuning.kf = 1.0

    long_delay = Params().get_int("LongActuatorDelay")
    ret.longitudinalActuatorDelay = (long_delay * 0.01) if long_delay > 0 else 0.5
    ret.longitudinalActuatorDelayLowerBound = 0.5
    ret.longitudinalActuatorDelayUpperBound = 0.5

    ret.startingState = False
    ret.startAccel = 2.0
    ret.stopAccel = -0.6
    ret.stoppingDecelRate = 1.2
    ret.vEgoStopping = 0.3
    ret.vEgoStarting = 0.2

    # genesis
    if candidate == CAR.GENESIS:
      ret.mass = 1900. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.GENESIS_G70:
      ret.mass = 1640. + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.GENESIS_G80:
      ret.mass = 1855. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.GENESIS_EQ900:
      ret.mass = 2200
      ret.wheelbase = 3.15
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.GENESIS_EQ900_L:
      ret.mass = 2290
      ret.wheelbase = 3.45
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.GENESIS_G90:
      ret.mass = 2150
      ret.wheelbase = 3.16
      ret.centerToFront = ret.wheelbase * 0.4
    # hyundai
    elif candidate in [CAR.SANTA_FE]:
      ret.mass = 1694 + STD_CARGO_KG
      ret.wheelbase = 2.766
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.SANTA_FE_2022, CAR.SANTA_FE_HEV_2022]:
      ret.mass = 1750 + STD_CARGO_KG
      ret.wheelbase = 2.766
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.SONATA, CAR.SONATA_HEV, CAR.SONATA21_HEV]:
      ret.mass = 1513. + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.centerToFront = ret.wheelbase * 0.4
      tire_stiffness_factor = 0.65
    elif candidate in [CAR.SONATA19, CAR.SONATA19_HEV]:
      ret.mass = 4497. * CV.LB_TO_KG
      ret.wheelbase = 2.804
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.SONATA_LF_TURBO:
      ret.mass = 1590. + STD_CARGO_KG
      ret.wheelbase = 2.805
      tire_stiffness_factor = 0.65
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.PALISADE:
      ret.mass = 1999. + STD_CARGO_KG
      ret.wheelbase = 2.90
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.ELANTRA, CAR.ELANTRA_GT_I30]:
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.ELANTRA_2021:
      ret.mass = (2800. * CV.LB_TO_KG) + STD_CARGO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 13.27 * 1.15
      tire_stiffness_factor = 0.65
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.ELANTRA_HEV_2021:
      ret.mass = (3017. * CV.LB_TO_KG) + STD_CARGO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 13.27 * 1.15
      tire_stiffness_factor = 0.65
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.KONA:
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.KONA_HEV, CAR.KONA_EV]:
      ret.mass = 1395. + STD_CARGO_KG
      ret.wheelbase = 2.6
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.IONIQ, CAR.IONIQ_EV_LTD, CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV]:
      ret.mass = 1490. + STD_CARGO_KG
      ret.wheelbase = 2.7
      tire_stiffness_factor = 0.385
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.GRANDEUR_IG, CAR.GRANDEUR_IG_HEV]:
      tire_stiffness_factor = 0.8
      ret.mass = 1570. + STD_CARGO_KG
      ret.wheelbase = 2.845
      ret.centerToFront = ret.wheelbase * 0.385
      ret.steerRatio = 16.
    elif candidate in [CAR.GRANDEUR_IG_FL, CAR.GRANDEUR_IG_FL_HEV]:
      tire_stiffness_factor = 0.8
      ret.mass = 1600. + STD_CARGO_KG
      ret.wheelbase = 2.885
      ret.centerToFront = ret.wheelbase * 0.385
      ret.steerRatio = 17.
    elif candidate == CAR.VELOSTER:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      tire_stiffness_factor = 0.9
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.TUCSON_TL_SCC:
      ret.mass = 1594. + STD_CARGO_KG
      ret.wheelbase = 2.67
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    # kia
    elif candidate == CAR.SORENTO:
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.K5, CAR.K5_HEV]:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.K5_2021]:
      ret.mass = 3228. * CV.LB_TO_KG
      ret.wheelbase = 2.85
      tire_stiffness_factor = 0.7
    elif candidate == CAR.STINGER:
      tire_stiffness_factor = 1.125
      ret.mass = 1825.0 + STD_CARGO_KG
      ret.wheelbase = 2.906
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.FORTE:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.CEED:
      ret.mass = 1350. + STD_CARGO_KG
      ret.wheelbase = 2.65
      tire_stiffness_factor = 0.6
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.SPORTAGE:
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.NIRO_EV, CAR.NIRO_HEV, CAR.NIRO_HEV_2021]:
      ret.mass = 1737. + STD_CARGO_KG
      ret.wheelbase = 2.7
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.SELTOS:
      ret.mass = 1310. + STD_CARGO_KG
      ret.wheelbase = 2.6
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.MOHAVE:
      ret.mass = 2285. + STD_CARGO_KG
      ret.wheelbase = 2.895
      ret.centerToFront = ret.wheelbase * 0.5
      tire_stiffness_factor = 0.8
    elif candidate in [CAR.K7, CAR.K7_HEV]:
      tire_stiffness_factor = 0.7
      ret.mass = 1650. + STD_CARGO_KG
      ret.wheelbase = 2.855
      ret.centerToFront = ret.wheelbase * 0.4
      ret.steerRatio = 17.25
    elif candidate == CAR.K9:
      ret.mass = 2005. + STD_CARGO_KG
      ret.wheelbase = 3.15
      ret.centerToFront = ret.wheelbase * 0.4
      tire_stiffness_factor = 0.8

    ret.radarTimeStep = (1.0 / 50)

    if ret.centerToFront == 0:
      ret.centerToFront = ret.wheelbase * 0.4

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.steerRatioRear = 0.
    ret.steerControlType = car.CarParams.SteerControlType.torque

    ret.stoppingControl = True

    ret.enableBsm = 0x58b in fingerprint[0]
    ret.enableAutoHold = 1151 in fingerprint[0]

    ret.mdpsBus = 1 if 593 in fingerprint[1] and 1296 not in fingerprint[1] else 0
    ret.sasBus = 1 if 688 in fingerprint[1] and 1296 not in fingerprint[1] else 0
    ret.sccBus = 0 if 1056 in fingerprint[0] else 1 if 1056 in fingerprint[1] and 1296 not in fingerprint[1] \
                                                                     else 2 if 1056 in fingerprint[2] else -1

    if ret.sccBus >= 0:
      ret.hasScc13 = 1290 in fingerprint[ret.sccBus]
      ret.hasScc14 = 905 in fingerprint[ret.sccBus]

    ret.hasEms = 608 in fingerprint[0] and 809 in fingerprint[0]
    ret.hasLfaHda = 1157 in fingerprint[0]

    ret.radarOffCan = ret.sccBus == -1
    ret.pcmCruise = not ret.radarOffCan

    # apilot-c2 engagement: lateral control is always available from the cruise
    # MAIN switch, so community safety must always be active for this model.
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiCommunity, 0)]
    return ret

  def _update(self, c: car.CarControl) -> car.CarState:
    pass

  def update(self, c: car.CarControl, can_strings: List[bytes]) -> car.CarState:
    self.cp.update_strings(can_strings)
    self.cp2.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp2, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp2.can_valid and self.cp_cam.can_valid
    ret.canTimeout = any(cp.bus_timeout for cp in self.can_parsers if cp is not None)

    if self.CP.pcmCruise and not self.CC.scc_live:
      self.CP.pcmCruise = False
    elif self.CC.scc_live and not self.CP.pcmCruise:
      self.CP.pcmCruise = True

    # apilot-c2 engagement model: do NOT touch ret.cruiseState.enabled here.
    # It stays the raw stock SCC state; engagement is driven by the MAIN switch
    # edge override applied after create_common_events() below.

    # turning indicator alert logic
    t = time.monotonic()
    if t - self._lc_speed_params_t > 1.0:
      try:
        auto_lc_speed_kph = int(self._lc_speed_params.get('AutoLaneChangeSpeed', encoding='utf8') or '50')
      except (TypeError, ValueError):
        auto_lc_speed_kph = 50
      self.lane_change_speed_min = auto_lc_speed_kph * CV.KPH_TO_MS
      self._lc_speed_params_t = t
    if not self.CC.keep_steering_turn_signals and (ret.leftBlinker or ret.rightBlinker or self.CC.turning_signal_timer) and ret.vEgo < self.lane_change_speed_min - 1.2:
      self.CC.turning_indicator_alert = True
    else:
      self.CC.turning_indicator_alert = False

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 0.2) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 0.7):
      self.low_speed_alert = False

    buttonEvents = []
    button_samples = getattr(self.CS, 'cruise_button_samples', [self.CS.cruise_buttons])
    for button, pressed in button_transitions(self.CS.prev_cruise_buttons, button_samples):
      be = car.CarState.ButtonEvent.new_message()
      be.pressed = pressed
      be.type = BUTTONS_DICT.get(button, ButtonType.unknown)
      buttonEvents.append(be)

    main_samples = getattr(self.CS, 'cruise_main_button_samples', [self.CS.cruise_main_button])
    for pressed in main_button_transitions(self.CS.prev_cruise_main_button, main_samples):
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.altButton3
      be.pressed = pressed
      buttonEvents.append(be)
    ret.buttonEvents = buttonEvents

    events = self.create_common_events(ret)

    # ---- apilot-c2 engagement model (ajouatom/openpilot apilot-c2) ----
    # Engage on the cruise MAIN switch rising edge; disengage only when MAIN
    # turns off. Stock SCC enable/disable (brake, cancel, RES/SET) only moves
    # the longitudinal state in CruiseHelper and never drops lateral control.
    # This replaces the enabled-edge pcmEnable/pcmDisable from common events.
    if EventName.pcmEnable in events.events:
      events.events.remove(EventName.pcmEnable)
    if EventName.pcmDisable in events.events:
      events.events.remove(EventName.pcmDisable)
    if ret.cruiseState.available and not self.CS.out.cruiseState.available:
      events.add(EventName.pcmEnable)
    elif not ret.cruiseState.available:
      events.add(EventName.pcmDisable)

    if self.CC.longcontrol and self.CS.cruise_unavail:
      events.add(EventName.brakeUnavailable)
    if self.low_speed_alert and not self.CS.mdps_bus:
      events.add(EventName.belowSteerSpeed)
    if self.CC.turning_indicator_alert:
      events.add(EventName.turningIndicatorOn)

    # handle button presses
    for b in ret.buttonEvents:
      # CANCEL stops ACC and blocks automatic resume in CruiseHelper, but keeps
      # lateral control engaged. Cruise MAIN off fully disengages through the
      # pcmDisable event generated from cruiseState.available above.

      if self.CC.longcontrol:
        # RES/SET can re-engage controls even while stock SCC is live. MAIN
        # stays on after an immediate disable (e.g. commIssue), so no new
        # pcmEnable edge exists - this is the recovery path.
        if b.type in [ButtonType.accelCruise, ButtonType.decelCruise] and not b.pressed:
          events.add(EventName.buttonEnable)
        if not self.CC.scc_live:
          if EventName.wrongCarMode in events.events:
            events.events.remove(EventName.wrongCarMode)
          if EventName.pcmDisable in events.events:
            events.events.remove(EventName.pcmDisable)
      elif not self.CC.longcontrol and ret.cruiseState.enabled:
        # do enable on decel button only
        if b.type == ButtonType.decelCruise and not b.pressed:
          events.add(EventName.buttonEnable)
          
    # scc smoother
    if self.CC.scc_smoother is not None:
      self.CC.scc_smoother.inject_events(events)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  # scc smoother - hyundai only
  def apply(self, c, controls):
    return self.CC.update(c, self.CS, controls)

from cereal import car
from common.numpy_fast import interp
from selfdrive.car.hyundai.values import DBC, FEATURES, CAR, HYBRID_CAR, EV_HYBRID_CAR, Buttons, CarControllerParams
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from common.conversions import Conversions as CV
from common.params import Params
from selfdrive.car.hyundai.cruise_buttons import collect_button_samples

GearShifter = car.CarState.GearShifter


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    if self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
      self.shifter_values = can_define.dv["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
      self.shifter_values = can_define.dv["TCU12"]["CUR_GR"]
    else:  # preferred and elect gear methods use same definition
      self.shifter_values = can_define.dv["LVR12"]["CF_Lvr_Gear"]

    #Auto detection for setup
    self.no_radar = CP.sccBus == -1
    self.mdps_bus = CP.mdpsBus
    self.sas_bus = CP.sasBus
    self.scc_bus = CP.sccBus
    self.has_scc13 = CP.hasScc13 or CP.carFingerprint in FEATURES["has_scc13"]
    self.has_scc14 = CP.hasScc14 or CP.carFingerprint in FEATURES["has_scc14"]
    self.has_lfa_hda = CP.hasLfaHda
    self.leftBlinker = False
    self.rightBlinker = False
    self.cruise_main_button = 0
    self.cruise_button_samples = []
    self.cruise_main_button_samples = []
    self.mdps_error_cnt = 0
    self.cruise_unavail_cnt = 0
    self.engine_oil_temp_seen = False
    self.engine_coolant_temp_seen = False
    self.engine_rpm_seen = False
    self.distance_to_empty_seen = False
    self.outside_temp_seen = False
    self.outside_temp_c = -1000.
    self.parking_sensor_age = 10000

    self.apply_steer = 0.
    
    self.params = CarControllerParams(CP)
    
    # scc smoother
    self.acc_mode = False
    self.cruise_gap = 1
    self.brake_pressed = False
    self.gas_pressed = False
    self.standstill = False
    self.cruiseState_enabled = False
    self.cruiseState_speed = 0
    # Auto-resume Cruise Set Speed by JangPoo
    self.prev_cruiseState_speed = 0
    self.obj_valid = 0

    self.use_cluster_speed = Params().get_bool('UseClusterSpeed')
    self.long_control_enabled = Params().get_bool('LongControlEnabled')

  def update(self, cp, cp2, cp_cam):
    cp_mdps = cp2 if self.mdps_bus else cp
    cp_sas = cp2 if self.sas_bus else cp
    cp_scc = cp2 if self.scc_bus == 1 else cp_cam if self.scc_bus == 2 else cp

    self.prev_cruise_buttons = self.cruise_buttons
    self.prev_cruise_main_button = self.cruise_main_button
    self.prev_left_blinker = self.leftBlinker
    self.prev_right_blinker = self.rightBlinker

    ret = car.CarState.new_message()

    ret.frontLeftDoorOpen = bool(cp.vl["CGW1"]["CF_Gway_DrvDrSw"])
    ret.frontRightDoorOpen = bool(cp.vl["CGW1"]["CF_Gway_AstDrSw"])
    ret.rearLeftDoorOpen = bool(cp.vl["CGW2"]["CF_Gway_RLDrSw"])
    ret.rearRightDoorOpen = bool(cp.vl["CGW2"]["CF_Gway_RRDrSw"])
    ret.doorOpen = any([ret.frontLeftDoorOpen, ret.frontRightDoorOpen,
                        ret.rearLeftDoorOpen, ret.rearRightDoorOpen])
    ret.trunkOpen = bool(cp.vl["CGW1"]["CF_Gway_TrunkTgSw"])
    ret.hoodOpen = bool(cp.vl["CGW1"]["CF_Gway_HoodSw"])
    ret.frontLeftWindowOpen = bool(cp.vl["CGW4"]["CF_Gway_DrvWdwStat"])
    ret.frontRightWindowOpen = bool(cp.vl["CGW4"]["CF_Gway_AstWdwStat"])
    ret.rearLeftWindowOpen = bool(cp.vl["CGW4"]["CF_Gway_RLWdwState"])
    ret.rearRightWindowOpen = bool(cp.vl["CGW4"]["CF_Gway_RRWdwState"])

    # CGW4 indicators already include occupant/warning logic for passenger
    # and rear seats.  Keep the proven driver switch as a fallback.
    belt_warning = any(int(cp.vl["CGW4"][sig]) != 0 for sig in (
      "CF_Gway_AstSeatBeltInd", "CF_Gway_RCSeatBeltInd",
      "CF_Gway_RLSeatBeltInd", "CF_Gway_RRSeatBeltInd"))
    ret.seatbeltUnlatched = (cp.vl["CGW1"]["CF_Gway_DrvSeatBeltSw"] == 0 or
                             belt_warning)

    if cp.vl_all["CLU13"]["CF_Clu_DTE"]:
      self.distance_to_empty_seen = True
    ret.distanceToEmptyKm = cp.vl["CLU13"]["CF_Clu_DTE"] if self.distance_to_empty_seen else -1.
    ret.lowFuelWarning = int(cp.vl["CLU13"]["CF_Clu_LowfuelWarn"]) != 0

    # Genesis DH's cluster displays DATC11.CR_Datc_OutTempC.  FATC11 and
    # mirror-sensor values can be several degrees hotter after heat soak, so
    # using whichever frame arrived first made the remote HUD disagree with
    # the cluster.  Keep DH locked to the cluster-ready DATC value; retain the
    # multi-source fallback for other Hyundai variants.
    if self.CP.carFingerprint == CAR.GENESIS:
      outside_temp_sources = (("DATC11", "CR_Datc_OutTempC"),)
    else:
      outside_temp_sources = (
        ("FATC11", "CR_Fatc_OutTemp"),
        ("FATC11", "CR_Fatc_OutTempSns"),
        ("DATC11", "CR_Datc_OutTempC"),
        ("CGW3", "C_MirOutTempSns"),
      )
    for msg, sig in outside_temp_sources:
      values = cp.vl_all[msg][sig]
      if values:
        outside_temp = float(values[-1])
        if -39.5 < outside_temp <= 80.:
          self.outside_temp_seen = True
          self.outside_temp_c = outside_temp
          break
    ret.outsideTempC = self.outside_temp_c if self.outside_temp_seen else -1000.

    self.is_set_speed_in_mph = bool(cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"])
    self.speed_conv_to_ms = CV.MPH_TO_MS if self.is_set_speed_in_mph else CV.KPH_TO_MS

    cluSpeed = cp.vl["CLU11"]["CF_Clu_Vanz"]
    decimal = cp.vl["CLU11"]["CF_Clu_VanzDecimal"]
    if 0. < decimal < 0.5:
      cluSpeed += decimal

    ret.vEgoCluster = cluSpeed * self.speed_conv_to_ms

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHL_SPD11"]["WHL_SPD_FL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_FR"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RR"],
    )

    vEgoRawClu = cluSpeed * self.speed_conv_to_ms
    vEgoClu, aEgoClu = self.update_clu_speed_kf(vEgoRawClu)

    vEgoRawWheel = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    vEgoRawWheel = interp(vEgoRawWheel, [0., 10.], [(vEgoRawWheel + vEgoRawClu) / 2., vEgoRawWheel])
    vEgoWheel, aEgoWheel = self.update_speed_kf(vEgoRawWheel)

    if self.use_cluster_speed:
      ret.vEgoRaw = vEgoRawClu
      ret.vEgo = vEgoClu
      ret.aEgo = aEgoClu
    else:
      ret.vEgoRaw = vEgoRawWheel
      ret.vEgo = vEgoWheel
      ret.aEgo = aEgoWheel

    ret.vCluRatio = (vEgoWheel / vEgoClu) if (vEgoClu > 3. and vEgoWheel > 3.) else 1.0
    ret.aBasis = cp.vl["TCS13"]["aBasis"]

    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringAngleDeg = cp_sas.vl["SAS11"]["SAS_Angle"]
    ret.steeringRateDeg = cp_sas.vl["SAS11"]["SAS_Speed"]
    ret.yawRate = cp.vl["ESP12"]["YAW_RATE"]
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["CGW1"]["CF_Gway_TurnSigLh"],
                                                            cp.vl["CGW1"]["CF_Gway_TurnSigRh"])
    # DH model years expose manual/AUTO exterior-lamp activity on different
    # gateway variants.  OR the actual low beam with every available tail
    # activity source so a manually selected lamp is not missed.
    ret.lowBeam = any((
      bool(cp.vl["CGW1"]["CF_Gway_HeadLampLow"]),
      bool(cp.vl["CGW2"]["CF_Gway_AvTail"]),
      bool(cp.vl["CGW2"]["CF_Gway_ExtTailAct"]),
      bool(cp.vl["CGW2"]["CF_Gway_IntTailAct"]),
      int(cp.vl["GW_IPM_PE_1"]["C_TailLampActivity"]) != 0,
    ))
    ret.highBeam = bool(cp.vl["CGW1"]["CF_Gway_HeadLampHigh"])
    ret.frontFogLight = bool(cp.vl["CGW1"]["CF_Gway_Frt_Fog_Act"])
    # Preserve the physical stalk position instead of reducing it to a single
    # on/off bit.  AUTO and intermittent are separate CGW1 signals depending
    # on whether the vehicle is equipped with a rain sensor.
    if cp.vl["CGW1"]["CF_Gway_WiperHighSw"]:
      ret.wiperMode = 4
    elif cp.vl["CGW1"]["CF_Gway_WiperLowSw"]:
      ret.wiperMode = 3
    elif cp.vl["CGW1"]["CF_Gway_WiperAutoSw"]:
      ret.wiperMode = 1
    elif cp.vl["CGW1"]["CF_Gway_WiperIntSw"]:
      ret.wiperMode = 2
    elif cp.vl["CGW1"]["CF_Gway_WiperMistSw"]:
      ret.wiperMode = 5
    else:
      ret.wiperMode = 0
    ret.steeringTorque = cp_mdps.vl["MDPS12"]["CR_Mdps_StrColTq"]
    ret.steeringTorqueEps = cp_mdps.vl["MDPS12"]["CR_Mdps_OutTq"] / 10.  # scale to Nm
    ret.steeringPressed = abs(ret.steeringTorque) > self.params.STEER_THRESHOLD

    # MDPS12 exposes independent LKAS fault indicators. Debounce them so the
    # intentional two-frame high-angle reset pulse is ignored, while a real
    # MDPS rejection still releases latActive and gets a chance to recover.
    mdps_fault = (cp_mdps.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0 or
                  cp_mdps.vl["MDPS12"]["CF_Mdps_ToiFlt"] != 0 or
                  cp_mdps.vl["MDPS12"]["CF_Mdps_FailStat"] != 0)
    if not ret.standstill and mdps_fault:
      self.mdps_error_cnt += 1
    else:
      self.mdps_error_cnt = 0

    ret.steerFaultTemporary = self.mdps_error_cnt > 50

    if self.CP.enableAutoHold:
      ret.autoHold = cp.vl["ESP11"]["AVH_STAT"]

    # cruise state
    ret.cruiseState.enabled = (cp_scc.vl["SCC12"]["ACCMode"] != 0) if not self.no_radar else \
                                      cp.vl["LVR12"]["CF_Lvr_CruiseSet"] != 0
    ret.cruiseState.available = (cp_scc.vl["SCC11"]["MainMode_ACC"] != 0) if not self.no_radar else \
                                      cp.vl["EMS16"]["CRUISE_LAMP_M"] != 0
    # Match aPilot C2: openpilot longitudinal owns stop/start state, so do not
    # feed the stock SCC standstill latch back into LongControl.
    ret.cruiseState.standstill = (not self.CP.openpilotLongitudinalControl and not self.no_radar and
                                  cp_scc.vl["SCC11"]["SCCInfoDisplay"] == 4.)

    ret.cruiseState.enabledAcc = ret.cruiseState.enabled

    if ret.cruiseState.enabled:
      ret.cruiseState.speed = cp_scc.vl["SCC11"]["VSetDis"] * self.speed_conv_to_ms if not self.no_radar else \
                                         cp.vl["LVR12"]["CF_Lvr_CruiseSet"] * self.speed_conv_to_ms
    else:
      ret.cruiseState.speed = 0
    self.cruise_main_button_samples = collect_button_samples(
      self.prev_cruise_main_button,
      cp.vl["CLU11"]["CF_Clu_CruiseSwMain"],
      cp.vl_all["CLU11"]["CF_Clu_CruiseSwMain"],
    )
    self.cruise_button_samples = collect_button_samples(
      self.prev_cruise_buttons,
      cp.vl["CLU11"]["CF_Clu_CruiseSwState"],
      cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"],
    )
    if self.cruise_main_button_samples:
      self.cruise_main_button = self.cruise_main_button_samples[-1]
    if self.cruise_button_samples:
      self.cruise_buttons = self.cruise_button_samples[-1]

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["TCS13"]["DriverBraking"] != 0
    ret.brakeHoldActive = cp.vl["TCS15"]["AVH_LAMP"] == 2  # 0 OFF, 1 ERROR, 2 ACTIVE, 3 READY
    ret.parkingBrake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1
    #ret.parkingBrake = cp.vl["CGW1"]["CF_Gway_ParkBrakeSw"]

    # TODO: Check this
    ret.brakeLights = bool(cp.vl["TCS13"]["BrakeLight"] or ret.brakePressed)
    ret.gasPressed = cp.vl["TCS13"]["DriverOverride"] == 1

    if self.CP.carFingerprint in EV_HYBRID_CAR:
      if self.CP.carFingerprint in HYBRID_CAR:
        ret.gas = cp.vl["E_EMS11"]["CR_Vcu_AccPedDep_Pos"] / 254.
      else:
        ret.gas = cp.vl["E_EMS11"]["Accel_Pedal_Pos"] / 254.

    if self.CP.hasEms:
      ret.gas = cp.vl["EMS12"]["PV_AV_CAN"] / 100.
      ret.gasPressed = bool(cp.vl["EMS16"]["CF_Ems_AclAct"])
      if cp.vl_all["EMS12"]["TEMP_ENG"]:
        self.engine_coolant_temp_seen = True
      if cp.vl_all["EMS19"]["CR_Ems_EngOilTemp"]:
        self.engine_oil_temp_seen = True
      ret.engineCoolantTempC = cp.vl["EMS12"]["TEMP_ENG"] if self.engine_coolant_temp_seen else -1000.
      ret.engineOilTempC = cp.vl["EMS19"]["CR_Ems_EngOilTemp"] if self.engine_oil_temp_seen else -1000.
      # 엔진 회전수. DH(GENESIS 2015-2016)는 EMS11(0x316)을 아예 보내지 않고
      # EMS_366(0x366)의 'N' 으로 올린다 — can_printer 실측에서 0x366 데이터
      # 1cbe09... 의 바이트1~2 가 0x09BE, x0.25 = 623.5rpm(공회전)로 확인됨.
      # 차종에 따라 EMS11 쪽인 경우도 있어 살아있는 값을 쓴다. 둘 다 checks 에는
      # 없으므로 메시지가 없어도 canValid 는 안 깨진다.
      rpm_raw = cp.vl["EMS_366"]["N"] or cp.vl["EMS11"]["N"]
      if rpm_raw:
        self.engine_rpm_seen = True
      ret.engineRpm = rpm_raw if self.engine_rpm_seen else 0.

    # TODO: refactor gear parsing in function
    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # as this seems to be standard over all cars, but is not the preferred method.
    ret.gearStep = 0
    if self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
      gear = cp.vl["CLU15"]["CF_Clu_Gear"]
      ret.gearStep = int(cp.vl["LVR11"]["CF_Lvr_GearInf"])
    elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
      gear = cp.vl["TCU12"]["CUR_GR"]
      ret.gearStep = int(gear) if 1 <= int(gear) <= 8 else 0
    elif self.CP.carFingerprint in FEATURES["use_elect_gears"]:
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
      ret.gearStep = int(cp.vl["ELECT_GEAR"]["Elect_Gear_Step"])
    else:
      gear = cp.vl["LVR12"]["CF_Lvr_Gear"]
      ret.gearStep = int(cp.vl["LVR11"]["CF_Lvr_GearInf"])

    # 1~8 단만 유효, 그 외(0=P/N, 14=R)는 0 처리
    if not 1 <= ret.gearStep <= 8:
      ret.gearStep = 0

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    if self.CP.carFingerprint in FEATURES["use_fca"]:
      ret.stockAeb = cp.vl["FCA11"]["FCA_CmdAct"] != 0
      ret.stockFcw = cp.vl["FCA11"]["CF_VSM_Warn"] == 2
      ret.aebSystemFault = cp.vl["FCA11"]["FCA_Failinfo"] != 0
    else:
      ret.stockAeb = cp_scc.vl["SCC12"]["AEB_CmdAct"] != 0
      ret.stockFcw = cp_scc.vl["SCC12"]["CF_VSM_Warn"] == 2
      ret.aebSystemFault = cp_scc.vl["SCC12"]["AEB_Failinfo"] != 0

    # Genesis DH exposes the same six parking-distance zones used by the
    # factory cluster in PAS11.  Keep a separate validity bit so vehicles that
    # do not publish PAS11 cannot look like a valid all-clear state.
    pas = cp.vl["PAS11"]
    if cp.vl_all["PAS11"]["CF_Gway_PASSystemOn"]:
      self.parking_sensor_age = 0
    else:
      self.parking_sensor_age += 1
    # PAS11 is slower than carState.  Hold the last sample between frames, but
    # clear it within one second if the optional message disappears.
    ret.parkingSensors.valid = self.parking_sensor_age < 100
    ret.parkingSensors.frontLeft = int(pas["CF_Gway_PASDisplayFLH"])
    ret.parkingSensors.frontCenter = int(pas["CF_Gway_PASDisplayFCTR"])
    ret.parkingSensors.frontRight = int(pas["CF_Gway_PASDisplayFRH"])
    ret.parkingSensors.rearLeft = int(pas["CF_Gway_PASDisplayRLH"])
    ret.parkingSensors.rearCenter = int(pas["CF_Gway_PASDisplayRCTR"])
    ret.parkingSensors.rearRight = int(pas["CF_Gway_PASDisplayRRH"])

    # Blind Spot Detection and Lane Change Assist signals
    if self.CP.enableBsm:
      # LCA normal operating modes are OFF, passive and active.  Values from
      # the fail state upward request the factory "check blind-spot system"
      # warning instead of being mistaken for an ordinary mirror indication.
      ret.blindSpotSystemFault = int(cp.vl["LCA11"]["CF_Lca_Stat"]) >= 3
      ret.leftBlindspot = cp.vl["LCA11"]["CF_Lca_IndLeft"] != 0
      ret.rightBlindspot = cp.vl["LCA11"]["CF_Lca_IndRight"] != 0
    else:
      ret.blindSpotSystemFault = False
      ret.leftBlindspot = False
      ret.rightBlindspot = False

    # save the entire LKAS11, CLU11, SCC12 and MDPS12
    self.lkas11 = cp_cam.vl["LKAS11"]
    self.clu11 = cp.vl["CLU11"]
    self.scc11 = cp_scc.vl["SCC11"]
    self.scc12 = cp_scc.vl["SCC12"]
    self.mdps12 = cp_mdps.vl["MDPS12"]
    self.lfahda_mfc = cp_cam.vl["LFAHDA_MFC"]
    self.steer_state = cp_mdps.vl["MDPS12"]["CF_Mdps_ToiActive"] #0 NOT ACTIVE, 1 ACTIVE
    self.cruise_unavail_cnt += 1 if cp.vl["TCS13"]["CF_VSM_Avail"] != 1 and cp.vl["TCS13"]["ACCEnable"] != 0 else -self.cruise_unavail_cnt
    self.cruise_unavail = self.cruise_unavail_cnt > 100

    self.lead_distance = cp_scc.vl["SCC11"]["ACC_ObjDist"] if not self.no_radar else 0
    if self.has_scc13:
      self.scc13 = cp_scc.vl["SCC13"]
    if self.has_scc14:
      self.scc14 = cp_scc.vl["SCC14"]

    # scc smoother
    driver_override = cp.vl["TCS13"]["DriverOverride"]
    self.acc_mode = cp_scc.vl["SCC12"]["ACCMode"] != 0
    # Ignore transient/invalid SCC gap values so gap-button automation and the
    # longitudinal controller always see a real Hyundai gap step (1 through 4).
    if not self.no_radar:
      cruise_gap = int(cp_scc.vl["SCC11"]["TauGapSet"])
      if 1 <= cruise_gap <= 4:
        self.cruise_gap = cruise_gap
    self.gas_pressed = ret.gasPressed or driver_override == 1
    self.brake_pressed = ret.brakePressed or driver_override == 2
    self.standstill = ret.standstill or ret.cruiseState.standstill
    self.cruiseState_enabled = ret.cruiseState.enabled
    self.cruiseState_speed = ret.cruiseState.speed
    ret.cruiseGap = self.cruise_gap

    tpms_unit = cp.vl["TPMS11"]["UNIT"] * 0.725 if int(cp.vl["TPMS11"]["UNIT"]) > 0 else 1.
    ret.tpms.fl = tpms_unit * cp.vl["TPMS11"]["PRESSURE_FL"]
    ret.tpms.fr = tpms_unit * cp.vl["TPMS11"]["PRESSURE_FR"]
    ret.tpms.rl = tpms_unit * cp.vl["TPMS11"]["PRESSURE_RL"]
    ret.tpms.rr = tpms_unit * cp.vl["TPMS11"]["PRESSURE_RR"]

    # Auto-resume Cruise Set Speed by JangPoo
    self.prev_cruiseState_speed = self.cruiseState_speed if self.cruiseState_speed else self.prev_cruiseState_speed
    self.obj_valid = cp_scc.vl["SCC11"]["ObjValid"]
    if self.prev_cruise_buttons == Buttons.CANCEL:
      self.prev_cruiseState_speed = 0

    return ret

  @staticmethod
  def get_can_parser(CP):

    signals = [
      # sig_name, sig_address
      ("WHL_SPD_FL", "WHL_SPD11"),
      ("WHL_SPD_FR", "WHL_SPD11"),
      ("WHL_SPD_RL", "WHL_SPD11"),
      ("WHL_SPD_RR", "WHL_SPD11"),

      ("YAW_RATE", "ESP12"),

      ("CF_Gway_DrvSeatBeltInd", "CGW4"),
      ("CF_Gway_AstSeatBeltInd", "CGW4"),
      ("CF_Gway_RCSeatBeltInd", "CGW4"),
      ("CF_Gway_RLSeatBeltInd", "CGW4"),
      ("CF_Gway_RRSeatBeltInd", "CGW4"),
      ("CF_Gway_DrvWdwStat", "CGW4"),
      ("CF_Gway_AstWdwStat", "CGW4"),
      ("CF_Gway_RLWdwState", "CGW4"),
      ("CF_Gway_RRWdwState", "CGW4"),

      ("CF_Gway_DrvSeatBeltSw", "CGW1"),
      ("CF_Gway_DrvDrSw", "CGW1"),       # Driver Door
      ("CF_Gway_AstDrSw", "CGW1"),       # Passenger door
      ("CF_Gway_TrunkTgSw", "CGW1"),     # Trunk / tailgate
      ("CF_Gway_HoodSw", "CGW1"),        # Hood
      ("CF_Gway_RLDrSw", "CGW2"),        # Rear reft door
      ("CF_Gway_RRDrSw", "CGW2"),        # Rear right door
      ("CF_Gway_TurnSigLh", "CGW1"),
      ("CF_Gway_TurnSigRh", "CGW1"),
      ("CF_Gway_HeadLampLow", "CGW1"),
      ("CF_Gway_AvTail", "CGW2"),
      ("CF_Gway_ExtTailAct", "CGW2"),
      ("CF_Gway_IntTailAct", "CGW2"),
      ("C_TailLampActivity", "GW_IPM_PE_1"),
      ("CF_Gway_HeadLampHigh", "CGW1"),
      ("CF_Gway_Frt_Fog_Act", "CGW1"),
      ("CF_Gway_WiperIntSw", "CGW1"),
      ("CF_Gway_WiperLowSw", "CGW1"),
      ("CF_Gway_WiperHighSw", "CGW1"),
      ("CF_Gway_WiperAutoSw", "CGW1"),
      ("CF_Gway_WiperMistSw", "CGW1"),
      ("CF_Gway_ParkBrakeSw", "CGW1"),   # Parking Brake

      ("CYL_PRES", "ESP12"),

      ("CF_Clu_CruiseSwState", "CLU11"),
      ("CF_Clu_CruiseSwMain", "CLU11"),
      ("CF_Clu_SldMainSW", "CLU11"),
      ("CF_Clu_ParityBit1", "CLU11"),
      ("CF_Clu_VanzDecimal" , "CLU11"),
      ("CF_Clu_Vanz", "CLU11"),
      ("CF_Clu_SPEED_UNIT", "CLU11"),
      ("CF_Clu_DetentOut", "CLU11"),
      ("CF_Clu_RheostatLevel", "CLU11"),
      ("CF_Clu_CluInfo", "CLU11"),
      ("CF_Clu_AmpInfo", "CLU11"),
      ("CF_Clu_AliveCnt1", "CLU11"),

      ("CF_Clu_DTE", "CLU13"),
      ("CF_Clu_LowfuelWarn", "CLU13"),

      # DH exterior ambient temperature.  No frequency checks: model years
      # use different climate/gateway frames and an absent variant must not
      # invalidate the rest of carState.
      ("CR_Fatc_OutTemp", "FATC11"),
      ("CR_Fatc_OutTempSns", "FATC11"),
      ("CR_Datc_OutTempC", "DATC11"),
      ("C_MirOutTempSns", "CGW3"),

      ("ACCEnable", "TCS13"),
      ("BrakeLight", "TCS13"),
      ("aBasis", "TCS13"),
      ("DriverBraking", "TCS13"),
      ("PBRAKE_ACT", "TCS13"),
      ("DriverOverride", "TCS13"),
      ("CF_VSM_Avail", "TCS13"),

      ("ESC_Off_Step", "TCS15"),
      ("AVH_LAMP", "TCS15"),

      #("CF_Lvr_GearInf", "LVR11"),        # Transmission Gear (0 = N or P, 1-8 = Fwd, 14 = Rev)

      ("MainMode_ACC", "SCC11"),
      ("SCCInfoDisplay", "SCC11"),
      ("AliveCounterACC", "SCC11"),
      ("VSetDis", "SCC11"),
      ("ObjValid", "SCC11"),
      ("DriverAlertDisplay", "SCC11"),
      ("TauGapSet", "SCC11"),
      ("ACC_ObjStatus", "SCC11"),
      ("ACC_ObjLatPos", "SCC11"),
      ("ACC_ObjDist", "SCC11"), #TK211X value is 204.6
      ("ACC_ObjRelSpd", "SCC11"),
      ("Navi_SCC_Curve_Status", "SCC11"),
      ("Navi_SCC_Curve_Act", "SCC11"),
      ("Navi_SCC_Camera_Act", "SCC11"),
      ("Navi_SCC_Camera_Status", "SCC11"),

      ("ACCMode", "SCC12"),
      ("CF_VSM_Prefill", "SCC12"),
      ("CF_VSM_DecCmdAct", "SCC12"),
      ("CF_VSM_HBACmd", "SCC12"),
      ("CF_VSM_Warn", "SCC12"),
      ("CF_VSM_Stat", "SCC12"),
      ("CF_VSM_BeltCmd", "SCC12"),
      ("ACCFailInfo", "SCC12"),
      ("StopReq", "SCC12"),
      ("CR_VSM_DecCmd", "SCC12"),
      ("aReqRaw", "SCC12"), #aReqMax
      ("TakeOverReq", "SCC12"),
      ("PreFill", "SCC12"),
      ("aReqValue", "SCC12"), #aReqMin
      ("CF_VSM_ConfMode", "SCC12"),
      ("AEB_Failinfo", "SCC12"),
      ("AEB_Status", "SCC12"),
      ("AEB_CmdAct", "SCC12"),
      ("AEB_StopReq", "SCC12"),
      ("CR_VSM_Alive", "SCC12"),
      ("CR_VSM_ChkSum", "SCC12"),

      ("SCCDrvModeRValue", "SCC13"),
      ("SCC_Equip", "SCC13"),
      ("AebDrvSetStatus", "SCC13"),

      ("JerkUpperLimit", "SCC14"),
      ("JerkLowerLimit", "SCC14"),
      ("SCCMode2", "SCC14"),
      ("ComfortBandUpper", "SCC14"),
      ("ComfortBandLower", "SCC14"),

      ("UNIT", "TPMS11"),
      ("PRESSURE_FL", "TPMS11"),
      ("PRESSURE_FR", "TPMS11"),
      ("PRESSURE_RL", "TPMS11"),
      ("PRESSURE_RR", "TPMS11"),

      # Factory-cluster parking sensor sectors.  This frame is optional and
      # deliberately omitted from checks; vl_all is used to detect presence.
      ("CF_Gway_PASDisplayFLH", "PAS11"),
      ("CF_Gway_PASDisplayFCTR", "PAS11"),
      ("CF_Gway_PASDisplayFRH", "PAS11"),
      ("CF_Gway_PASDisplayRLH", "PAS11"),
      ("CF_Gway_PASDisplayRCTR", "PAS11"),
      ("CF_Gway_PASDisplayRRH", "PAS11"),
      ("CF_Gway_PASSystemOn", "PAS11"),
    ]

    checks = [
      # address, frequency
      ("TCS13", 50),
      ("TCS15", 10),
      ("CLU11", 50),
      ("ESP12", 100),
      ("CGW1", 10),
      ("CGW2", 5),
      ("CGW4", 5),
      ("WHL_SPD11", 50),
    ]

    if CP.sccBus == 0 and CP.pcmCruise:
      checks += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]
    if CP.mdpsBus == 0:
      signals += [
        ("CR_Mdps_StrColTq", "MDPS12"),
        ("CF_Mdps_Def", "MDPS12"),
        ("CF_Mdps_ToiActive", "MDPS12"),
        ("CF_Mdps_ToiUnavail", "MDPS12"),
        ("CF_Mdps_ToiFlt", "MDPS12"),
        ("CF_Mdps_MsgCount2", "MDPS12"),
        ("CF_Mdps_Chksum2", "MDPS12"),
        ("CF_Mdps_SErr", "MDPS12"),
        ("CR_Mdps_StrTq", "MDPS12"),
        ("CF_Mdps_FailStat", "MDPS12"),
        ("CR_Mdps_OutTq", "MDPS12")
      ]
      checks += [
        ("MDPS12", 50)
      ]
    if CP.sasBus == 0:
      signals += [
        ("SAS_Angle", "SAS11"),
        ("SAS_Speed", "SAS11"),
      ]
      checks += [
        ("SAS11", 100)
      ]
    if CP.sccBus == -1:
      signals += [
        ("CRUISE_LAMP_M", "EMS16"),
        ("CF_Lvr_CruiseSet", "LVR12"),
    ]
    if CP.carFingerprint in FEATURES["use_cluster_gears"]:
      signals += [
        ("CF_Clu_Gear", "CLU15"),
        ("CF_Lvr_GearInf", "LVR11"),        # 변속단수 (0 = N or P, 1-8 = Fwd, 14 = Rev)
      ]
    elif CP.carFingerprint in FEATURES["use_tcu_gears"]:
      signals += [
        ("CUR_GR", "TCU12"),
      ]
    elif CP.carFingerprint in FEATURES["use_elect_gears"]:
      signals += [
        ("Elect_Gear_Shifter", "ELECT_GEAR"),
        ("Elect_Gear_Step", "ELECT_GEAR"),  # 변속단수 (EV/HEV)
    ]
    else:
      signals += [
        ("CF_Lvr_Gear","LVR12"),
        ("CF_Lvr_GearInf", "LVR11"),        # 변속단수 (0 = N or P, 1-8 = Fwd, 14 = Rev)
      ]

    if CP.carFingerprint in EV_HYBRID_CAR:
      if CP.carFingerprint in HYBRID_CAR:
        signals += [
          ("CR_Vcu_AccPedDep_Pos", "E_EMS11")
        ]
      else:
        signals += [
          ("Accel_Pedal_Pos", "E_EMS11"),
        ]
      checks += [
        ("E_EMS11", 50),
      ]

    else:
      signals += [
        ("PV_AV_CAN", "EMS12"),
        ("TEMP_ENG", "EMS12"),
        ("CF_Ems_AclAct", "EMS16"),
        ("CR_Ems_EngOilTemp", "EMS19"),
        ("N", "EMS_366"),                 # 엔진 회전수 (rpm) — DH 는 이쪽
        ("N", "EMS11"),                   # 엔진 회전수 (rpm) — 다른 차종 대비. 둘 다 checks 에는 안 넣는다
      ]
      checks += [
        ("EMS12", 100),
        ("EMS16", 100),
      ]

    if CP.carFingerprint in FEATURES["use_fca"]:
      signals += [
        ("FCA_CmdAct", "FCA11"),
        ("CF_VSM_Warn", "FCA11"),
        ("FCA_Failinfo", "FCA11"),
      ]

      if not CP.openpilotLongitudinalControl:
        checks += [("FCA11", 50)]

    if CP.carFingerprint in [CAR.SANTA_FE, CAR.SANTA_FE_2022]:
      checks.remove(("TCS13", 50))

    if CP.enableBsm:
      signals += [
        ("CF_Lca_Stat", "LCA11"),
        ("CF_Lca_IndLeft", "LCA11"),
        ("CF_Lca_IndRight", "LCA11"),
      ]
      checks += [("LCA11", 50)]

    if CP.enableAutoHold:
      signals += [
        ("AVH_STAT", "ESP11"),
        ("LDM_STAT", "ESP11"),
      ]
      checks += [("ESP11", 50)]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0, enforce_checks=False)

  @staticmethod
  def get_can2_parser(CP):
    signals = []
    checks = []
    if CP.mdpsBus == 1:
      signals += [
        ("CR_Mdps_StrColTq", "MDPS12"),
        ("CF_Mdps_Def", "MDPS12"),
        ("CF_Mdps_ToiActive", "MDPS12"),
        ("CF_Mdps_ToiUnavail", "MDPS12"),
        ("CF_Mdps_ToiFlt", "MDPS12"),
        ("CF_Mdps_MsgCount2", "MDPS12"),
        ("CF_Mdps_Chksum2", "MDPS12"),
        ("CF_Mdps_SErr", "MDPS12"),
        ("CR_Mdps_StrTq", "MDPS12"),
        ("CF_Mdps_FailStat", "MDPS12"),
        ("CR_Mdps_OutTq", "MDPS12")
      ]
      checks += [
        ("MDPS12", 50)
      ]

    if CP.sasBus == 1:
      signals += [
        ("SAS_Angle", "SAS11"),
        ("SAS_Speed", "SAS11"),
      ]
      checks += [
        ("SAS11", 100)
      ]
    if CP.sccBus == 1:
      signals += [
        ("MainMode_ACC", "SCC11"),
        ("SCCInfoDisplay", "SCC11"),
        ("AliveCounterACC", "SCC11"),
        ("VSetDis", "SCC11"),
        ("ObjValid", "SCC11"),
        ("DriverAlertDisplay", "SCC11"),
        ("TauGapSet", "SCC11"),
        ("ACC_ObjStatus", "SCC11"),
        ("ACC_ObjLatPos", "SCC11"),
        ("ACC_ObjDist", "SCC11"),
        ("ACC_ObjRelSpd", "SCC11"),
        ("Navi_SCC_Curve_Status", "SCC11"),
        ("Navi_SCC_Curve_Act", "SCC11"),
        ("Navi_SCC_Camera_Act", "SCC11"),
        ("Navi_SCC_Camera_Status", "SCC11"),


        ("ACCMode", "SCC12"),
        ("CF_VSM_Prefill", "SCC12"),
        ("CF_VSM_DecCmdAct", "SCC12"),
        ("CF_VSM_HBACmd", "SCC12"),
        ("CF_VSM_Warn", "SCC12"),
        ("CF_VSM_Stat", "SCC12"),
        ("CF_VSM_BeltCmd", "SCC12"),
        ("ACCFailInfo", "SCC12"),
        ("StopReq", "SCC12"),
        ("CR_VSM_DecCmd", "SCC12"),
        ("aReqRaw", "SCC12"), #aReqMax
        ("TakeOverReq", "SCC12"),
        ("PreFill", "SCC12"),
        ("aReqValue", "SCC12"), #aReqMin
        ("CF_VSM_ConfMode", "SCC12"),
        ("AEB_Failinfo", "SCC12"),
        ("AEB_Status", "SCC12"),
        ("AEB_CmdAct", "SCC12"),
        ("AEB_StopReq", "SCC12"),
        ("CR_VSM_Alive", "SCC12"),
        ("CR_VSM_ChkSum", "SCC12"),

        ("SCCDrvModeRValue", "SCC13"),
        ("SCC_Equip", "SCC13"),
        ("AebDrvSetStatus", "SCC13"),

        ("JerkUpperLimit", "SCC14"),
        ("JerkLowerLimit", "SCC14"),
        ("SCCMode2", "SCC14"),
        ("ComfortBandUpper", "SCC14"),
        ("ComfortBandLower", "SCC14"),

      ]
      checks += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]
    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 1, enforce_checks=False)

  @staticmethod
  def get_cam_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("CF_Lkas_LdwsActivemode", "LKAS11"),
      ("CF_Lkas_LdwsSysState", "LKAS11"),
      ("CF_Lkas_SysWarning", "LKAS11"),
      ("CF_Lkas_LdwsLHWarning", "LKAS11"),
      ("CF_Lkas_LdwsRHWarning", "LKAS11"),
      ("CF_Lkas_HbaLamp", "LKAS11"),
      ("CF_Lkas_FcwBasReq", "LKAS11"),
      ("CF_Lkas_ToiFlt", "LKAS11"),
      ("CF_Lkas_HbaSysState", "LKAS11"),
      ("CF_Lkas_FcwOpt", "LKAS11"),
      ("CF_Lkas_HbaOpt", "LKAS11"),
      ("CF_Lkas_FcwSysState", "LKAS11"),
      ("CF_Lkas_FcwCollisionWarning", "LKAS11"),
      ("CF_Lkas_MsgCount", "LKAS11"),
      ("CF_Lkas_FusionState", "LKAS11"),
      ("CF_Lkas_FcwOpt_USM", "LKAS11"),
      ("CF_Lkas_LdwsOpt_USM", "LKAS11"),
    ]

    checks = [
      ("LKAS11", 100)
    ]
    if CP.sccBus == 2:
      signals += [
        ("MainMode_ACC", "SCC11"),
        ("SCCInfoDisplay", "SCC11"),
        ("AliveCounterACC", "SCC11"),
        ("VSetDis", "SCC11"),
        ("ObjValid", "SCC11"),
        ("DriverAlertDisplay", "SCC11"),
        ("TauGapSet", "SCC11"),
        ("ACC_ObjStatus", "SCC11"),
        ("ACC_ObjLatPos", "SCC11"),
        ("ACC_ObjDist", "SCC11"),
        ("ACC_ObjRelSpd", "SCC11"),
        ("Navi_SCC_Curve_Status", "SCC11"),
        ("Navi_SCC_Curve_Act", "SCC11"),
        ("Navi_SCC_Camera_Act", "SCC11"),
        ("Navi_SCC_Camera_Status", "SCC11"),

        ("ACCMode", "SCC12"),
        ("CF_VSM_Prefill", "SCC12"),
        ("CF_VSM_DecCmdAct", "SCC12"),
        ("CF_VSM_HBACmd", "SCC12"),
        ("CF_VSM_Warn", "SCC12"),
        ("CF_VSM_Stat", "SCC12"),
        ("CF_VSM_BeltCmd", "SCC12"),
        ("ACCFailInfo", "SCC12"),
        ("StopReq", "SCC12"),
        ("CR_VSM_DecCmd", "SCC12"),
        ("aReqRaw", "SCC12"), #aReqMax
        ("TakeOverReq", "SCC12"),
        ("PreFill", "SCC12"),
        ("aReqValue", "SCC12"), #aReqMin
        ("CF_VSM_ConfMode", "SCC12"),
        ("AEB_Failinfo", "SCC12"),
        ("AEB_Status", "SCC12"),
        ("AEB_CmdAct", "SCC12"),
        ("AEB_StopReq", "SCC12"),
        ("CR_VSM_Alive", "SCC12"),
        ("CR_VSM_ChkSum", "SCC12"),

        ("SCCDrvModeRValue", "SCC13"),
        ("SCC_Equip", "SCC13"),
        ("AebDrvSetStatus", "SCC13"),

        ("JerkUpperLimit", "SCC14"),
        ("JerkLowerLimit", "SCC14"),
        ("SCCMode2", "SCC14"),
        ("ComfortBandUpper", "SCC14"),
        ("ComfortBandLower", "SCC14"),
      ]
      checks += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]

      if CP.hasLfaHda:
        signals += [
          ("HDA_USM", "LFAHDA_MFC"),
          ("HDA_Active", "LFAHDA_MFC"),
          ("HDA_Icon_State", "LFAHDA_MFC"),
          ("HDA_LdwSysState", "LFAHDA_MFC"),
          ("HDA_Icon_Wheel", "LFAHDA_MFC"),
        ]
        checks += [("LFAHDA_MFC", 20)]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2, enforce_checks=False)

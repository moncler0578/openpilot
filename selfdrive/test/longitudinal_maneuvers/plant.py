#!/usr/bin/env python3
import numpy as np

from cereal import log
import cereal.messaging as messaging
from common.realtime import Ratekeeper, DT_MDL
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from selfdrive.modeld.constants import T_IDXS


class Plant():
  def __init__(self, lead_relevancy=False, speed=0.0, distance_lead=2.0,
               only_lead2=False, only_radar=False):
    self.rate = 1. / DT_MDL

    self.v_lead_prev = 0.0

    self.distance = 0.
    self.speed = speed
    self.acceleration = 0.0

    # lead car
    self.distance_lead = distance_lead
    self.lead_relevancy = lead_relevancy
    self.only_lead2=only_lead2
    self.only_radar=only_radar

    self.rk = Ratekeeper(self.rate, print_delay_threshold=100.0)
    self.ts = 1. / self.rate

    from selfdrive.car.hyundai.values import CAR
    from selfdrive.car.hyundai.interface import CarInterface
    CP = CarInterface.get_params(CAR.GRANDEUR_IG)
    CP.openpilotLongitudinalControl = True
    self.planner = LongitudinalPlanner(CP, init_v=self.speed)

  def current_time(self):
    return float(self.rk.frame) / self.rate

  def step(self, v_lead=0.0, prob=1.0, v_cruise=50.):
    # ******** publish a fake model going straight and fake calibration ********
    # note that this is worst case for MPC, since model will delay long mpc by one time step
    radar = messaging.new_message('radarState')
    control = messaging.new_message('controlsState')
    car_state = messaging.new_message('carState')
    model = messaging.new_message('modelV2')
    a_lead = (v_lead - self.v_lead_prev)/self.ts
    self.v_lead_prev = v_lead

    if self.lead_relevancy:
      d_rel = np.maximum(0., self.distance_lead - self.distance)
      v_rel = v_lead - self.speed
      if self.only_radar:
        status = True
      elif prob > .5:
        status = True
      else:
        status = False
    else:
      d_rel = 200.
      v_rel = 0.
      prob = 0.0
      status = False

    lead = log.RadarState.LeadData.new_message()
    lead.dRel = float(d_rel)
    lead.yRel = float(0.0)
    lead.vRel = float(v_rel)
    lead.aRel = float(a_lead - self.acceleration)
    lead.vLead = float(v_lead)
    lead.vLeadK = float(v_lead)
    lead.aLeadK = float(a_lead)
    lead.aLeadTau = float(1.5)
    lead.status = status
    lead.modelProb = float(prob)
    lead.radar = True
    if not self.only_lead2:
      radar.radarState.leadOne = lead
    radar.radarState.leadTwo = lead

    control.controlsState.longControlState = LongCtrlState.pid
    control.controlsState.vCruise = float(v_cruise * 3.6)
    control.controlsState.enabled = True
    control.controlsState.forceDecel = False
    control.controlsState.myDrivingMode = 3
    control.controlsState.mySafeModeFactor = 1.0
    car_state.carState.vEgo = float(self.speed)
    car_state.carState.aEgo = float(self.acceleration)
    car_state.carState.vCluRatio = 1.0
    car_state.carState.standstill = self.speed < 0.01

    # Straight, speed-aligned model trajectory. ACC maneuvers use radar leads,
    # but the current planner still requires a complete modelV2 message.
    model.modelV2.position.x = (np.asarray(T_IDXS) * self.speed).tolist()
    model.modelV2.position.y = np.zeros(len(T_IDXS)).tolist()
    model.modelV2.position.z = np.zeros(len(T_IDXS)).tolist()
    model.modelV2.velocity.x = np.full(len(T_IDXS), self.speed).tolist()
    model.modelV2.acceleration.x = np.full(len(T_IDXS), self.acceleration).tolist()
    model.modelV2.temporalPose.trans = [self.speed, 0.0, 0.0]

    # ******** get controlsState messages for plotting ***
    sm = {'radarState': radar.radarState,
          'carState': car_state.carState,
          'controlsState': control.controlsState,
          'modelV2': model.modelV2}
    self.planner.update(sm, read=False)
    self.speed = self.planner.v_desired_filter.x
    self.acceleration = self.planner.a_desired
    fcw = self.planner.fcw
    self.distance_lead = self.distance_lead + v_lead * self.ts

    # ******** run the car ********
    #print(self.distance, speed)
    if self.speed <= 0:
      self.speed = 0
      self.acceleration = 0
    self.distance = self.distance + self.speed * self.ts

    # *** radar model ***
    if self.lead_relevancy:
      d_rel = np.maximum(0., self.distance_lead - self.distance)
      v_rel = v_lead - self.speed
    else:
      d_rel = 200.
      v_rel = 0.

    # print at 5hz
    if (self.rk.frame % (self.rate // 5)) == 0:
      print("%2.2f sec   %6.2f m  %6.2f m/s  %6.2f m/s2   lead_rel: %6.2f m  %6.2f m/s"
            % (self.current_time(), self.distance, self.speed, self.acceleration, d_rel, v_rel))


    # ******** update prevs ********
    self.rk.monitor_time()

    return {
      "distance": self.distance,
      "speed": self.speed,
      "acceleration": self.acceleration,
      "distance_lead": self.distance_lead,
      "fcw": fcw,
    }

# simple engage in standalone mode
def plant_thread():
  plant = Plant()
  while 1:
    plant.step()


if __name__ == "__main__":
  plant_thread()

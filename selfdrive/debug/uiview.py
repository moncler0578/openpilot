#!/usr/bin/env python3
import time
from cereal import car, log, messaging
from common.params import Params
from selfdrive.hardware import HARDWARE
from selfdrive.manager.process_config import managed_processes

if __name__ == "__main__":
  CP = car.CarParams(notCar=True)
  Params().put("CarParams", CP.to_bytes())

  # The external HUD path is S9-only.
  procs = ['camerad', 'ui', 'modeld', 'calibrationd', 'remote_hud']

  HARDWARE.set_power_save(False)

  for p in procs:
    managed_processes[p].start()
  
  pm = messaging.PubMaster(['controlsState', 'deviceState', 'pandaStates', 'carParams', 'carState'])
  
  msgs = {s: messaging.new_message(s) for s in ['controlsState', 'deviceState', 'carParams']}
  msgs['deviceState'].deviceState.started = True
  msgs['carParams'].carParams.openpilotLongitudinalControl = True
  
  msgs['pandaStates'] = messaging.new_message('pandaStates', 1)
  msgs['pandaStates'].pandaStates[0].ignitionLine = True
  msgs['pandaStates'].pandaStates[0].pandaType = log.PandaState.PandaType.uno
  
  speed = 0.
  try:
    while True:
      # 20 Hz is enough for UI/HUD preview and avoids evicting slower EON
      # subscribers while modeld and the selected external HUD are active.
      time.sleep(1 / 20)
      
      msgs['carState'] = messaging.new_message('carState')
      msgs['carState'].carState.vEgoCluster = speed
      
      speed += 0.02
      if speed > 40.:
        speed = 0.
      
      for s in msgs:
        pm.send(s, msgs[s])
        if hasattr(msgs[s], "clear_write_flag"):
          msgs[s].clear_write_flag()
  except KeyboardInterrupt:
    for p in procs:
      managed_processes[p].stop()

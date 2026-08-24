#!/usr/bin/env python3
# uiview + Carrot 미리보기
#  기본 uiview.py 에 더해, 아래 요소가 화면에 보이도록 가짜 데이터를 함께 publish/기록한다.
#   - 지도 경로 패널(drawCarrotNavi)   : /dev/shm/carrot_navi_route.json 을 매초 갱신
#   - 좌상단 wifi IP                    : deviceState.wifiIpAddress
#   - 차간거리(GAP) 막대                : controlsState.longCruiseGap
#   - NDA + LIMIT                       : roadLimitSpeed(active/roadLimitSpeed)
#   - 하단 중앙 디버그(offset 제거 확인) : lateralPlan.latDebugText
#  ※ 리드박스(앞차)는 모델/레이더 리드가 필요해 벤치 uiview 로는 안 뜬다(실차 onroad 필요).
#  사용:  cd /data/openpilot && python selfdrive/debug/uiview_carrot.py   (종료 Ctrl+C)
#  먼저 수정한 ui 를 scons 로 빌드해 두어야 그 UI 가 뜬다.
import os
import time
import json
from cereal import car, log, messaging
from common.params import Params
from selfdrive.hardware import HARDWARE
from selfdrive.manager.process_config import managed_processes

ROUTE_FILE = "/dev/shm/carrot_navi_route.json"


def build_route():
  # 동탄 부근 가짜 경로 (lat/lon 은 패널에서 자동 스케일되므로 대략값이면 됨)
  pts = [
    {"lat": 37.2000, "lon": 127.0700},
    {"lat": 37.2018, "lon": 127.0722},
    {"lat": 37.2032, "lon": 127.0710},
    {"lat": 37.2051, "lon": 127.0742},
    {"lat": 37.2069, "lon": 127.0731},
    {"lat": 37.2088, "lon": 127.0760},
  ]
  return {
    "updated_at_ms": int(time.time() * 1000),        # fresh(35s) 유지용
    "vehicle": {"lat": 37.2003, "lon": 127.0703, "road_name": "동탄솔빛로"},
    "guidance_current": {"main_text": "동탄", "distance_m": 1100, "turn_type": 12},
    "guidance_next": {"main_text": "동탄솔빛로", "distance_m": 2600, "turn_type": 13},
    "route": {"remain_distance_m": 3100, "remain_time_sec": 300, "polyline": pts},
    "speed": {"road_limit_kph": 60},
  }


def write_route():
  tmp = ROUTE_FILE + ".tmp"
  with open(tmp, "w") as f:
    json.dump(build_route(), f)
  os.replace(tmp, ROUTE_FILE)


if __name__ == "__main__":
  CP = car.CarParams(notCar=True)
  Params().put("CarParams", CP.to_bytes())

  # Include the S9-only external HUD publisher in bench preview.
  procs = ['camerad', 'ui', 'modeld', 'calibrationd', 'remote_hud']
  HARDWARE.set_power_save(False)
  for p in procs:
    managed_processes[p].start()

  pm = messaging.PubMaster(['controlsState', 'deviceState', 'pandaStates',
                            'carParams', 'carState', 'roadLimitSpeed', 'lateralPlan'])

  msgs = {s: messaging.new_message(s) for s in ['controlsState', 'deviceState', 'carParams']}
  msgs['deviceState'].deviceState.started = True
  msgs['deviceState'].deviceState.wifiIpAddress = "192.168.0.77"     # 좌상단에 표시될 IP
  msgs['carParams'].carParams.openpilotLongitudinalControl = True
  # Bench preview represents an engaged drive so path status color is green.
  msgs['controlsState'].controlsState.enabled = True

  msgs['pandaStates'] = messaging.new_message('pandaStates', 1)
  msgs['pandaStates'].pandaStates[0].ignitionLine = True
  msgs['pandaStates'].pandaStates[0].pandaType = log.PandaState.PandaType.uno

  # NDA(active==1) + LIMIT 60
  msgs['roadLimitSpeed'] = messaging.new_message('roadLimitSpeed')
  msgs['roadLimitSpeed'].roadLimitSpeed.active = 1
  msgs['roadLimitSpeed'].roadLimitSpeed.roadLimitSpeed = 60

  # 하단 중앙 디버그 (offset 제거 확인 : 화면엔 "4.6m | 2.9m | 2.6m" 로 나와야 함)
  msgs['lateralPlan'] = messaging.new_message('lateralPlan')
  msgs['lateralPlan'].lateralPlan.latDebugText = "4.6m | 2.9m | 2.6m | offset=0.0cm"

  speed = 0.0
  last_route = 0.0
  try:
    while True:
      # 20 Hz is enough for a bench preview and avoids evicting slower EON
      # subscribers while modeld and the external HUD are both active.
      time.sleep(1 / 20)

      msgs['carState'] = messaging.new_message('carState')
      msgs['carState'].carState.vEgoCluster = speed
      msgs['carState'].carState.cruiseGap = 2          # GAP 막대 2칸
      msgs['controlsState'].controlsState.longCruiseGap = 2

      speed += 0.02
      if speed > 40.0:
        speed = 0.0

      now = time.time()
      if now - last_route > 1.0:
        write_route()          # 지도 패널 fresh 유지 (매초)
        last_route = now

      for s in msgs:
        pm.send(s, msgs[s])
        # Reused capnp builders otherwise retain their serialized write flag,
        # emit warnings, and leak memory throughout a long bench session.
        if hasattr(msgs[s], "clear_write_flag"):
          msgs[s].clear_write_flag()
  except KeyboardInterrupt:
    for p in procs:
      managed_processes[p].stop()
    try:
      os.remove(ROUTE_FILE)
    except OSError:
      pass

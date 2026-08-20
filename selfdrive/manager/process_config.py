import os

from selfdrive.hardware import EON, TICI, PC
from selfdrive.manager.process import PythonProcess, NativeProcess, DaemonProcess

WEBCAM = os.getenv("USE_WEBCAM") is not None

procs = [
  #DaemonProcess("manage_athenad", "selfdrive.athena.manage_athenad", "AthenadPid"),
  # due to qualcomm kernel bugs SIGKILLing camerad sometimes causes page table corruption
  NativeProcess("camerad", "selfdrive/camerad", ["./camerad"], unkillable=True, driverview=True),
  NativeProcess("clocksd", "selfdrive/clocksd", ["./clocksd"]),
  # 2026-08-19: 운전자 감시 미사용. 스냅드래곤821에서 이 신경망이 상시 도는
  # 비용이 커서 끈다. 설정 화면의 운전자 카메라 미리보기는 camerad 담당이라
  # 그대로 동작한다. 되살리려면 enabled=(not PC or WEBCAM) 로 되돌리고
  # controlsd.py 의 ignore 목록에서 driverMonitoringState 를 빼면 된다.
  NativeProcess("dmonitoringmodeld", "selfdrive/modeld", ["./dmonitoringmodeld"], enabled=False, driverview=True),
  #NativeProcess("logcatd", "selfdrive/logcatd", ["./logcatd"]),
  # Disable route logging on EON to remove its CPU, compression, and disk-I/O
  # load from the driving path. This intentionally disables rlog/qlog output.
  NativeProcess("loggerd", "selfdrive/loggerd", ["./loggerd"], enabled=False),
  NativeProcess("modeld", "selfdrive/modeld", ["./modeld"]),
  NativeProcess("navd", "selfdrive/ui/navd", ["./navd"], enabled=(PC or TICI), persistent=True),
  # procLog 구독자는 selfdrive/debug/live_cpu_and_temp.py 와 테스트뿐이고
  # loggerd 가 꺼져 있어 기록되지도 않는다. UI/HUD 의 CPU 표시는 thermald 의
  # deviceState.cpuUsagePercent 라 이것과 무관하다.
  NativeProcess("proclogd", "selfdrive/proclogd", ["./proclogd"], enabled=False),
  NativeProcess("sensord", "selfdrive/sensord", ["./sensord"], enabled=not PC, persistent=EON, sigkill=EON),
  NativeProcess("ubloxd", "selfdrive/locationd", ["./ubloxd"], enabled=(not PC or WEBCAM)),
  NativeProcess("ui", "selfdrive/ui", ["./ui"], persistent=True, watchdog_max_dt=(5 if TICI else None)),
  NativeProcess("soundd", "selfdrive/ui/soundd", ["./soundd"], persistent=True),
  NativeProcess("locationd", "selfdrive/locationd", ["./locationd"]),
  NativeProcess("boardd", "selfdrive/boardd", ["./boardd"], enabled=False),
  PythonProcess("calibrationd", "selfdrive.locationd.calibrationd"),
  PythonProcess("carrotnavid", "selfdrive.carrot_navi_server", enabled=EON, persistent=True),
  # External HUD output is S9-only. EON publishes compact telemetry/native
  # TMAP assets plus live S9 render tuning; Android renders/JPEG-encodes/USB.
  PythonProcess("remote_hud", "selfdrive.eon_cluster.remote_hud_s9", enabled=EON, persistent=True),
  PythonProcess("controlsd", "selfdrive.controls.controlsd"),
  # loggerd 가 꺼져 있어 지울 로그가 없다.
  PythonProcess("deleter", "selfdrive.loggerd.deleter", enabled=False, persistent=True),
  # dmonitoringmodeld 가 꺼져 driverState 가 안 오므로 같이 끈다.
  PythonProcess("dmonitoringd", "selfdrive.monitoring.dmonitoringd", enabled=False, driverview=True),
  #PythonProcess("logmessaged", "selfdrive.logmessaged", persistent=True),
  PythonProcess("pandad", "selfdrive.boardd.pandad", persistent=True),
  PythonProcess("paramsd", "selfdrive.locationd.paramsd"),
  PythonProcess("plannerd", "selfdrive.controls.plannerd"),
  PythonProcess("radard", "selfdrive.controls.radard"),
  PythonProcess("thermald", "selfdrive.thermald.thermald", persistent=True),
  PythonProcess("timezoned", "selfdrive.timezoned", enabled=TICI, persistent=True),
  #PythonProcess("tombstoned", "selfdrive.tombstoned", enabled=not PC, persistent=True),
  #PythonProcess("updated", "selfdrive.updated", enabled=not PC, persistent=True),
  # loggerd 가 꺼져 있어 올릴 로그가 없다. 5~60초 주기 스캔·네트워크 시도 제거.
  PythonProcess("uploader", "selfdrive.loggerd.uploader", enabled=False, persistent=True),
  #PythonProcess("statsd", "selfdrive.statsd", persistent=True),

  # EON only
  PythonProcess("rtshield", "selfdrive.rtshield", enabled=EON),
  PythonProcess("shutdownd", "selfdrive.hardware.eon.shutdownd", enabled=EON),
  PythonProcess("androidd", "selfdrive.hardware.eon.androidd", enabled=EON, persistent=True),

  # Experimental
  PythonProcess("rawgpsd", "selfdrive.sensord.rawgps.rawgpsd", enabled=os.path.isfile("/persist/comma/use-quectel-rawgps")),
]

managed_processes = {p.name: p for p in procs}

import signal
import time

import cereal.messaging as messaging
from common.params import Params

from selfdrive.eon_cluster.renderer import HudRenderer, read_navi_state
from selfdrive.eon_cluster.scene import extract_driving_scene
PARAM_ENABLED = "EonClusterHud"
PARAM_CONNECTED = "EonClusterHudConnected"
PARAM_BRIGHTNESS = "EonClusterHudBrightness"
PARAM_FPS = "EonClusterHudFps"
PARAM_JPEG_QUALITY = "EonClusterHudJpegQuality"
RECONNECT_INTERVAL_S = 5.0


def _param_int(params, key, default, minimum, maximum):
  try:
    raw = params.get(key)
    value = int(raw) if raw is not None else default
  except (TypeError, ValueError):
    value = default
  return max(minimum, min(maximum, value))


def _field(message, name, default=0.0):
  try:
    return getattr(message, name)
  except Exception:
    return default


def main():
  params = Params()
  params.put_bool(PARAM_CONNECTED, False)
  running = [True]

  def stop(_signum, _frame):
    running[0] = False

  signal.signal(signal.SIGINT, stop)
  signal.signal(signal.SIGTERM, stop)
  sm = messaging.SubMaster(["carState", "controlsState", "deviceState", "modelV2", "radarState"])
  display = None
  renderer = None
  next_connect = 0.0
  next_frame = 0.0

  try:
    while running[0]:
      if not params.get_bool(PARAM_ENABLED):
        if display is not None:
          display.close()
          display = None
          renderer = None
          params.put_bool(PARAM_CONNECTED, False)
        time.sleep(1.0)
        continue

      now = time.monotonic()
      if display is None:
        if now < next_connect:
          time.sleep(min(0.2, next_connect - now))
          continue
        fps = _param_int(params, PARAM_FPS, 10, 5, 15)
        brightness = _param_int(params, PARAM_BRIGHTNESS, 65, 10, 100)
        quality = _param_int(params, PARAM_JPEG_QUALITY, 58, 35, 75)
        try:
          # Keep the default-disabled process harmless even on images missing
          # an optional USB/crypto runtime dependency.
          from selfdrive.eon_cluster.usb_display import TurzxDisplay
          display = TurzxDisplay(brightness=brightness, frame_rate=fps)
          display.open()
          renderer = HudRenderer(display.landscape_size[0], display.landscape_size[1], quality)
          params.put_bool(PARAM_CONNECTED, True)
          next_frame = now
          print("EON cluster connected: pid=0x%04x, %dx%d, %d fps" %
                (display.product_id, display.landscape_size[0], display.landscape_size[1], fps), flush=True)
        except Exception as exc:
          if display is not None:
            display.close()
          display = None
          renderer = None
          params.put_bool(PARAM_CONNECTED, False)
          next_connect = now + RECONNECT_INTERVAL_S
          print("EON cluster waiting for TURZX display: %s" % exc, flush=True)
          continue

      fps = _param_int(params, PARAM_FPS, 10, 5, 15)
      interval = 1.0 / fps
      if now < next_frame:
        time.sleep(min(0.02, next_frame - now))
        continue
      next_frame = max(next_frame + interval, now)
      sm.update(0)
      car_state = sm["carState"]
      controls_state = sm["controlsState"]
      speed_mps = float(_field(car_state, "vEgoCluster", _field(car_state, "vEgo", 0.0)))
      cruise_kph = float(_field(controls_state, "vCruiseCluster", _field(controls_state, "vCruise", 0.0)))
      enabled = bool(_field(controls_state, "enabled", False))
      try:
        scene = extract_driving_scene(sm["modelV2"], sm["radarState"])
        frame = renderer.render(speed_mps * 3.6, cruise_kph, enabled, read_navi_state(), scene)
        display.send_jpeg(renderer.encode_portrait_jpeg(frame))
      except Exception as exc:
        print("EON cluster USB frame failed: %s" % exc, flush=True)
        display.close()
        display = None
        renderer = None
        params.put_bool(PARAM_CONNECTED, False)
        next_connect = time.monotonic() + RECONNECT_INTERVAL_S
  finally:
    if display is not None:
      display.close()
    params.put_bool(PARAM_CONNECTED, False)


if __name__ == "__main__":
  main()

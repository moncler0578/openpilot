# EON TURZX HUD

Low-load external HUD for the EON `g_c2hud` branch. It renders a camera-free
driving scene from `modelV2` and `radarState`, reuses the existing TMap receiver
outputs in `/dev/shm`, and sends the combined JPEG dashboard to a supported
TURZX USB display.

Supported devices:

- `1cbe:0092` (9.2 inch, 1920x462)
- `1cbe:0123` (12.3 inch, 1920x720)

The process is disabled by default. Enable and tune it with Params:

```sh
cd /data/openpilot
python - <<'PY'
from common.params import Params
p = Params()
p.put_bool("EonClusterHud", True)
p.put("EonClusterHudFps", "10")
p.put("EonClusterHudBrightness", "65")
p.put("EonClusterHudJpegQuality", "58")
p.put("EonClusterHudPanelLayout", "0")
PY
```

The left 60% of the display is a lightweight synthetic driving scene with
model lanes, the planned path, radar leads, current speed, cruise speed, and
road speed limit. The right 40% keeps the TMap map, turn guidance, lane image,
and remaining distance. No road-camera pixels are copied or encoded.

The lightweight HUD also mirrors active openpilot alerts, shows Hyundai/Kia
TPMS values and the current ECO/SAFE/NORM/FAST driving mode, and replaces the
navigation panel with a trip summary while the vehicle is in Park. Set
`EonClusterHudPanelLayout` to `1` to move the driving view to the right and the
information panel to the left.

Start at 10 FPS. The accepted FPS range is deliberately limited to 5-15 FPS
to protect EON thermal and scheduling headroom. Camera rendering, OpenGL scene
capture, and software H.264 are intentionally excluded. The display is dimmed
when the process is disabled or stopped.

`EonClusterHudConnected` reports the live USB connection state. The process
waits without rendering while the display is absent and retries every 5 seconds.

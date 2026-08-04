# EON TURZX HUD

Low-load external HUD for the EON `g_c2hud` branch. It reuses the existing
TMap receiver outputs in `/dev/shm` and sends a camera-free JPEG dashboard to a
supported TURZX USB display.

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
PY
```

Start at 10 FPS. The accepted FPS range is deliberately limited to 5-15 FPS
to protect EON thermal and scheduling headroom. Camera rendering and software
H.264 are intentionally excluded. The display is dimmed when the process is
disabled or stopped.

`EonClusterHudConnected` reports the live USB connection state. The process
waits without rendering while the display is absent and retries every 5 seconds.

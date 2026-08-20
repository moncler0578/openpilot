# S9 Remote TURZX HUD

This branch uses one external-HUD path only:

- EON publishes compact driving telemetry over UDP 7210.
- EON forwards native compressed TMAP assets over TCP 7211.
- The Galaxy S9 renders the dashboard, creates the HUD JPEG, and sends it to the TURZX USB display.
- The legacy direct-EON renderer and USB output process are not started.

Enable it with the **S9 외부 HUD 사용** toggle. The following Params are sent to the S9 at runtime:

- `EonClusterHudFps`: 0 pauses S9 rendering; 1-15 FPS otherwise.
- `EonClusterHudMapFps`: 2-5 FPS for the native TMAP map stream.
- `EonClusterHudBrightness`: 0 automatic; 1-100 fixed.
- `EonClusterHudJpegQuality`: 20-95.
- `EonClusterHudScreenMode`: 1 automatic, 2 live debug, 3 fixed trip report.
- `EonClusterHudTheme`: 0 automatic, 1 dark, 2 light.
- `EonClusterHudOrientation`: 0 normal, 2 rotated 180 degrees.
- `EonClusterHudMirror`: 0 normal, 1 mirrored.
- `EonClusterHudLanguage`: 0 Korean, 1 English.
- `EonClusterHudRadarInfo`: 0 hidden; 1/3 relative speed; 2/4 distance and relative speed.

The display layout is fixed at 4:2:4. The left panel uses model lane/path and radar data, the middle panel shows system information, and the right panel uses the original TMAP frame and selected native TMAP guidance assets.

`EonClusterHudConnected` reports only the recent S9 UDP acknowledgement. It is useful as a network status indicator but is not treated as proof that the TURZX USB display is working. The EON driving UI therefore remains complete and independent of S9/USB status.

TMAP render files are written atomically under `/dev/shm`. Only validated JPEG frames refresh the map watchdog. A stale map is removed after five seconds even if the TMAP control socket disconnects, and a later valid frame recreates it automatically.

The S9 application must support telemetry protocol v4 and the TCP asset tags `MAP1`, `TBT1`, `TBT2`, and `LANE`.

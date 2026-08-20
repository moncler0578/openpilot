from pathlib import Path


ROOT = Path(__file__).parents[3]
UI_DIR = ROOT / "selfdrive" / "ui" / "qt"


def test_s9_connection_does_not_hide_eon_driving_ui():
  onroad = (UI_DIR / "onroad.cc").read_text(encoding="utf-8")
  header = (UI_DIR / "onroad.h").read_text(encoding="utf-8")
  assert 'getBool("EonClusterHudConnected")' not in onroad
  assert "eon_cluster_hud_connected" not in onroad
  assert "eon_cluster_hud_connected" not in header
  assert "drawLaneLines(p, s);" in onroad
  assert "drawCarrotPlot(p);" in onroad
  assert "drawCarrotLead(p);" in onroad
  assert "updateCarrotNavi(true);" in onroad
  assert "drawCarrotNavi(p);" in onroad


def test_image_loading_is_gated_separately_from_json_state():
  navi = (UI_DIR / "onroad_navi.inc").read_text(encoding="utf-8")
  image_guard = navi.index("if (load_images)")
  json_update = navi.index('root.value("updated_at_ms")')
  assert image_guard < json_update
  assert "void NvgWindow::drawCarrotNavi" in navi
  draw_body = navi.split("void NvgWindow::drawCarrotNavi", 1)[1]
  assert "updateCarrotNavi();" not in draw_body


def test_s9_hud_params_are_exposed_in_settings():
  settings = (UI_DIR / "offroad" / "settings.cc").read_text(encoding="utf-8")
  assert '"EonClusterHud", "S9 외부 HUD 사용"' in settings
  assert "EonClusterHudOutputMode" not in settings
  assert '"EonClusterHudFps", "S9 HUD 프레임"' in settings
  assert '"../assets/offroad/icon_road.png", 0, 15, 1, 0, 10' in settings
  assert '"EonClusterHudMapFps", "S9 HUD 지도 프레임"' in settings
  assert '"EonClusterHudBrightness", "S9 HUD 밝기"' in settings
  assert '"EonClusterHudJpegQuality", "S9 HUD 화질"' in settings
  assert '"../assets/offroad/icon_road.png", 20, 95, 1, 0, 58' in settings
  assert '"EonClusterHudScreenMode", "S9 HUD 우측 화면"' in settings
  assert '"EonClusterHudOrientation", "S9 HUD 화면 회전"' in settings
  assert '"EonClusterHudMirror", "S9 HUD 좌우 반전"' in settings
  assert '"EonClusterHudLanguage", "S9 HUD 언어"' in settings
  assert '"EonClusterHudRadarInfo", "S9 HUD 레이더 정보"' in settings


def test_manager_starts_only_s9_hud_publisher():
  processes = (ROOT / "selfdrive" / "manager" / "process_config.py").read_text(encoding="utf-8")
  remote = (ROOT / "selfdrive" / "eon_cluster" / "remote_hud.py").read_text(encoding="utf-8")
  params = (ROOT / "selfdrive" / "common" / "params.cc").read_text(encoding="utf-8")

  assert 'PythonProcess("remote_hud", "selfdrive.eon_cluster.remote_hud_s9", enabled=EON' in processes
  assert 'PythonProcess("eon_cluster"' not in processes
  assert "EonClusterHudOutputMode" not in remote
  assert "_migrate_legacy_remote_mode" not in remote
  assert 'params.put_bool(PARAM_ENABLED, True)' not in remote
  assert '{"EonClusterHudOutputMode", PERSISTENT}' not in params


def test_uiview_starts_only_s9_hud_publisher():
  debug_dir = ROOT / "selfdrive" / "debug"
  for script_name in ("uiview.py", "uiview_carrot.py"):
    script = (debug_dir / script_name).read_text(encoding="utf-8")
    process_line = next(line for line in script.splitlines() if line.strip().startswith("procs ="))
    assert "'remote_hud'" in process_line
    assert "'eon_cluster'" not in process_line


def test_map_activity_requires_valid_jpeg_and_stale_clear_survives_disconnect():
  server = (ROOT / "selfdrive" / "carrot_navi_server.py").read_text(encoding="utf-8")
  valid_marker = 'image = image_payload[start:end + 2]'
  activity_marker = "self.last_map_rx = now"
  first_activity = server.index(activity_marker, server.index(valid_marker))
  assert first_activity > server.index(valid_marker)
  watchdog = server.split("def _map_watchdog_loop", 1)[1].split("def local_ip", 1)[0]
  assert "if not control_present:\n        continue" not in watchdog
  assert "if age >= MAP_STALE_S and control_present:" in watchdog
  assert "if age >= MAP_STALE_CLEAR_S and not already_cleared:" in watchdog


def test_remote_ack_is_status_only():
  remote = (ROOT / "selfdrive" / "eon_cluster" / "remote_hud.py").read_text(encoding="utf-8")
  onroad = (UI_DIR / "onroad.cc").read_text(encoding="utf-8")
  assert 'PARAM_CONNECTED = "EonClusterHudConnected"' in remote
  assert "params.put_bool(PARAM_CONNECTED, value)" in remote
  assert "_publish_connected(params, published, connected)" in remote
  assert "EonClusterHudConnected" not in onroad

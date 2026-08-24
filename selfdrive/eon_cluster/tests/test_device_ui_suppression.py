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


def test_eon_driving_path_matches_valid_mpc_with_model_fallback():
  ui = (ROOT / "selfdrive" / "ui" / "ui.cc").read_text(encoding="utf-8")
  assert "getMpcSolutionValid()" in ui
  assert "getMpcPathX()" in ui
  assert "getMpcPathY()" in ui
  assert "const float z = i < static_cast<int>(line_z.size()) ? line_z[i] : 0.0f;" in ui
  assert 's->sm->alive("lateralPlan") && s->sm->valid("lateralPlan")' in ui
  assert "!update_mpc_path_data(s, lateral_plan, model_position" in ui
  assert "update_line_data(s, model_position, path_width" in ui


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
  assert '"EonClusterHud", "S9 HUD 사용"' in settings
  assert '"EonClusterHudOutputTarget", "HUD 출력 대상"' in settings
  assert '"1: 외부 HUD / 2: S9 화면 / 3: 동시 출력"' in settings
  assert '"EonClusterHudOutputMode", "S9 HUD 표시 내용"' in settings
  assert '"EonClusterHudLayoutMode", "S9 HUD 화면 구성"' in settings
  assert '"2: 주행 + 티맵만(우측 정보판 숨김, 티맵 폭 확장)"' in settings
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
  assert '{"EonClusterHudOutputMode", PERSISTENT}' in params
  assert '{"EonClusterHudLayoutMode", PERSISTENT}' in params
  assert '{"EonClusterHudOutputTarget", PERSISTENT}' in params


def test_s9_output_target_reaches_android_renderer():
  manager = (ROOT / "selfdrive" / "manager" / "manager.py").read_text(encoding="utf-8")
  remote = (ROOT / "selfdrive" / "eon_cluster" / "remote_hud_s9.py").read_text(encoding="utf-8")
  service = (ROOT / "selfdrive" / "eon_cluster" / "android_hud" / "app" / "src" /
             "main" / "java" / "ai" / "comma" / "remotehud" / "HudService.java").read_text(encoding="utf-8")

  assert '("EonClusterHudOutputTarget", "3")' in manager
  assert 'packet["hudOutputTarget"] = _bounded_int("EonClusterHudOutputTarget", 3, 1, 3)' in remote
  assert 'packet["hudLayoutMode"] = _bounded_int("EonClusterHudLayoutMode", 1, 1, 2)' in remote
  assert '("EonClusterHudLayoutMode", "1")' in manager
  assert 'currentState.optInt("hudOutputTarget", 3)' in service
  assert 'return configuredOutputTarget == 1 || configuredOutputTarget == 3;' in service
  assert 'return configuredOutputTarget == 2 || configuredOutputTarget == 3;' in service


def test_s9_tmap_first_switch_uses_nmirror_favorite_and_internal_fullscreen_activity():
  android = ROOT / "selfdrive" / "eon_cluster" / "android_hud" / "app" / "src" / "main"
  manifest = (android / "AndroidManifest.xml").read_text(encoding="utf-8")
  fullscreen = (android / "java" / "ai" / "comma" / "remotehud" /
                "HudFullscreenActivity.java").read_text(encoding="utf-8")
  favorite = (android / "java" / "ai" / "comma" / "remotehud" /
              "HudFavoriteActivity.java").read_text(encoding="utf-8")
  service = (android / "java" / "ai" / "comma" / "remotehud" /
             "HudService.java").read_text(encoding="utf-8")
  prefs = (android / "java" / "ai" / "comma" / "remotehud" /
           "AppPrefs.java").read_text(encoding="utf-8")

  fullscreen_decl = manifest.split('android:name=".HudFullscreenActivity"', 1)[1].split(
      '<activity', 1)[0]
  main_decl = manifest.split('android:name=".MainActivity"', 1)[1].split('</activity>', 1)[0]
  favorite_decl = manifest.split('android:name=".HudFavoriteActivity"', 1)[1].split(
      '</activity>', 1)[0]
  assert 'android:name=".HudFullscreenActivity"' in manifest
  assert 'android:label="EON HUD"' in manifest
  assert 'android:screenOrientation="landscape"' in manifest
  assert 'android:exported="false"' in fullscreen_decl
  assert "android.intent.category.LAUNCHER" not in fullscreen_decl
  assert "android.intent.category.LAUNCHER" in main_decl
  assert "android.intent.category.LAUNCHER" in favorite_decl
  assert 'android:label="HUD 전환"' in favorite_decl
  assert 'android:icon="@drawable/ic_hud_switch"' in favorite_decl
  assert "SYSTEM_ALERT_WINDOW" not in manifest
  assert "SYSTEM_UI_FLAG_IMMERSIVE_STICKY" in fullscreen
  assert "WindowInsets.Type.statusBars()" in fullscreen
  on_create = fullscreen.split("public void onCreate", 1)[1].split("protected void onResume", 1)[0]
  assert on_create.index("setContentView(frameView);") < on_create.index("hideSystemUi();")
  assert "decorView.getWindowInsetsController()" in fullscreen
  assert "getWindow().getInsetsController()" not in fullscreen
  assert "ACTION_SHOW_TMAP" in fullscreen
  assert "finishAndRemoveTask();" in fullscreen
  assert "static boolean shouldCloseForFavoriteLaunch()" in fullscreen
  assert "sincePause >= 0L && sincePause < 1000L" in fullscreen
  assert "HudFullscreenActivity.shouldCloseForFavoriteLaunch()" in favorite
  assert "HudFullscreenActivity.ACTION_SHOW_HUD" in favorite
  assert "HudFullscreenActivity.ACTION_SHOW_TMAP" in favorite
  assert "Intent.FLAG_ACTIVITY_NEW_TASK" in favorite
  assert "HudSwitchOverlay" not in service
  assert 'GUIDE_SHOWN = "guide_shown_v37"' in prefs
  assert "HudService.drawFullscreenFrame" in fullscreen
  assert "static boolean drawFullscreenFrame" in service
  assert "AppPrefs.getDisplayProfile(service)" in service
  assert "AppPrefs.DISPLAY_PROFILE_GENESIS_8" in service
  assert "AppPrefs.DISPLAY_PROFILE_GENESIS_9_2" in service
  assert "PHONE_8_WIDTH = 800" in service
  assert "PHONE_8_HEIGHT = 480" in service
  assert "PHONE_8_SIDEBAR = 0" in service
  assert "PHONE_9_WIDTH = 1280" in service
  assert "PHONE_9_HEIGHT = 720" in service
  assert "PHONE_9_SIDEBAR = 0" in service
  assert "renderNativePhone(currentState, AppPrefs.getDisplayProfile(this)," in service
  assert "profile == service.phoneNativeProfile" in service
  assert "service.phoneNativeFrame" in service
  assert "frame.getWidth() * scale" in service
  assert "frame.getHeight() * scale" in service
  assert "phoneDestination.set(left, top, left + drawWidth, top + drawHeight)" in service
  assert "nativeLayoutRendering = true" in service
  assert "drawFrame(c, s, map, tbtCurrent, tbtNext, lane)" in service
  assert "desired / nativeScaleX" in service
  assert "desired / nativeScaleY" in service
  assert "nativeScaleY / nativeScaleX" in service
  assert "c.clipRect(MAP_LEFT, 0f, mapRight(), HEIGHT)" in service
  assert "c.clipRect(0f, 0f, DRIVE_RIGHT, HEIGHT)" in service
  assert "NATIVE_SYSTEM_RATIO = 0.15f" in service
  assert "drawNativeSystemPanel(c, p, s)" in service
  assert "float equalizeX = nativeScaleY / nativeScaleX" in service
  assert "float contentScale = targetWidthPx" in service
  assert "NATIVE_GAUGE_RAISE_PX = 42f" in service
  assert 'currentState.optInt("hudLayoutMode", 1)' in service
  assert "return configuredLayoutMode == 2 ? WIDTH : MAP_RIGHT;" in service
  assert "if (configuredLayoutMode == 1 && nativeLayoutRendering)" in service
  assert service.count("-NATIVE_GAUGE_RAISE_PX / nativeWidgetScale") == 2
  assert "NATIVE_CARD_SHIFT_PX = 18f" in service
  assert "NATIVE_NOO_SHIFT_PX = 24f" in service
  assert 's.optDouble("atcBlend", 0d)' in service
  assert 's.optInt("atcDirection", 0)' in service
  assert '"NOO %s %.0f%%"' in service
  assert "drawNativeS9Remote(c, p, nativeWidth, nativeHeight, targetWidthPx)" in service
  assert "float unit = height / 480f" in service
  assert "RPM_LABEL_BASELINE" not in service
  assert "c.drawBitmap(phoneFrame, phoneNativeSource" not in service
  assert "drawNativeMapCard" not in service
  assert "drawNativeStatusBar" not in service
  assert "c.drawBitmap(phoneFrame, 0f, 0f, phonePreviewPaint);" in service
  assert "new RectF(0f, 0f, width, height)" not in service
  main = (android / "java" / "ai" / "comma" / "remotehud" /
          "MainActivity.java").read_text(encoding="utf-8")
  assert '"자동 감지 (기존 원본 비율)"' in main
  assert '"제네시스 순정 8인치  ·  800×480"' in main
  assert '"제네시스 순정 9.2인치  ·  1280×720"' in main
  assert "RadioGroup profileGroup" in main
  assert "option.setChecked(true)" in main
  assert 'displayProfileValue.setText("✓ 현재 적용: " + selected)' in main
  assert "Spinner" not in main
  assert "AppPrefs.setDisplayProfile" in main
  assert "ACTION_MANAGE_OVERLAY_PERMISSION" not in main
  assert not (android / "java" / "ai" / "comma" / "remotehud" /
              "HudSwitchOverlay.java").exists()
  assert not (android / "java" / "ai" / "comma" / "remotehud" /
              "PhoneHudOverlay.java").exists()


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


def test_osm_cache_is_versioned_and_refreshes_stale_tiles_safely():
  osm = (ROOT / "selfdrive" / "eon_cluster" / "android_hud" / "app" / "src" / "main" /
         "java" / "ai" / "comma" / "remotehud" / "OsmWorld.java").read_text(encoding="utf-8")
  assert 'CACHE_VERSION = "v3_"' in osm
  assert "CACHE_TTL_MS = 7L * 24L * 60L * 60L * 1000L" in osm
  assert "REFRESH_FAIL_COOLDOWN_MS = 90L * 60L * 1000L" in osm
  assert "refreshDueAt" in osm
  assert "boolean stale = loaded && (due == null || now >= due)" in osm
  assert "boolean stale = tile != null && now >= refreshDue" in osm
  assert "tiles.put(key, tile);" in osm
  assert "if (tile == null || stale)" in osm
  assert "cached.setLastModified(now);" in osm
  assert "cached.setLastModified(System.currentTimeMillis())" not in osm
  assert "failedAt.put(key, now);" in osm

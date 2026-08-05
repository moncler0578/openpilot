from pathlib import Path


UI_DIR = Path(__file__).parents[2] / "ui" / "qt"


def test_external_hud_keeps_json_updates_but_skips_tmap_draw():
  onroad = (UI_DIR / "onroad.cc").read_text(encoding="utf-8")
  assert 'Params().getBool("EonClusterHudConnected")' in onroad
  assert "updateCarrotNavi(!eon_cluster_hud_connected);" in onroad
  assert "if (!eon_cluster_hud_connected) drawCarrotNavi(p);" in onroad


def test_image_loading_is_gated_separately_from_json_state():
  navi = (UI_DIR / "onroad_navi.inc").read_text(encoding="utf-8")
  image_guard = navi.index("if (load_images)")
  json_update = navi.index('root.value("updated_at_ms")')
  assert image_guard < json_update
  assert "void NvgWindow::drawCarrotNavi" in navi
  draw_body = navi.split("void NvgWindow::drawCarrotNavi", 1)[1]
  assert "updateCarrotNavi();" not in draw_body


def test_external_hud_params_are_exposed_in_settings():
  settings = (UI_DIR / "offroad" / "settings.cc").read_text(encoding="utf-8")
  assert 'ParamControl("EonClusterHud"' in settings
  assert 'ParamValueControlF("EonClusterHudFps"' in settings
  assert '"../assets/offroad/icon_road.png", 5, 15, 1, 0, 10' in settings
  assert 'ParamValueControlF("EonClusterHudBrightness"' in settings
  assert '"../assets/offroad/icon_road.png", 10, 100, 5, 0, 65' in settings
  assert 'ParamValueControlF("EonClusterHudJpegQuality"' in settings
  assert '"../assets/offroad/icon_road.png", 35, 75, 1, 0, 58' in settings

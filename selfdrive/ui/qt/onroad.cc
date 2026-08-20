#include "selfdrive/ui/qt/onroad.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <string>

#include <QDebug>
#include <QSound>
#include <QMouseEvent>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPainterPath>
#include <QPair>

#include <ctime>

#include "selfdrive/common/timing.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/input.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#endif

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(bdr_s);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  QStackedLayout *road_view_layout = new QStackedLayout;
  road_view_layout->setStackingMode(QStackedLayout::StackAll);
  nvg = new NvgWindow(VISION_STREAM_RGB_ROAD, this);
  road_view_layout->addWidget(nvg);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addLayout(road_view_layout);

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);

  // screen recoder - neokii

  record_timer = std::make_shared<QTimer>();

  QWidget* recorder_widget = new QWidget(this);
  QVBoxLayout * recorder_layout = new QVBoxLayout (recorder_widget);
  recorder_layout->setMargin(35);
  recorder = new ScreenRecoder(this);
  QObject::connect(record_timer.get(), &QTimer::timeout, recorder, &ScreenRecoder::update_screen);
  QObject::connect(recorder, &ScreenRecoder::recordingChanged, [=](bool recording) {
    if (recording) record_timer->start(1000 / UI_FREQ);
    else record_timer->stop();
  });
  recorder_layout->addWidget(recorder);
  recorder_layout->setAlignment(recorder, Qt::AlignRight | Qt::AlignBottom);

  stacked_layout->addWidget(recorder_widget);
  recorder_widget->raise();
  alerts->raise();

}

void OnroadWindow::updateState(const UIState &s) {
  // Keep NvgWindow state in sync with carState (including blind-spot signals).
  nvg->updateState(s);

  if (!mapbox_param_initialized || s.sm->frame % UI_FREQ == 0) {
    const bool enabled = params.getBool("ShowMapboxMap");
    if (!mapbox_param_initialized || enabled != mapbox_enabled) {
      mapbox_enabled = enabled;
      mapbox_param_initialized = true;
      nvg->setMapImageEnabled(mapbox_enabled);
#ifdef ENABLE_MAPS
      if (map != nullptr) static_cast<MapWindow *>(map)->setMapEnabled(mapbox_enabled);
#endif
    }
  }

  const auto car_state = (*s.sm)["carState"].getCarState();
  brake_lights = car_state.getBrakeLights();
  left_blindspot = car_state.getLeftBlindspot();
  right_blindspot = car_state.getRightBlindspot();
  steering_angle_deg = car_state.getSteeringAngleDeg();

  QColor bgColor = bg_colors[s.status];
  Alert alert = Alert::get(*(s.sm), s.scene.started_frame);
  if (s.sm->updated("controlsState") || !alert.equal({})) {
    if (alert.type == "controlsUnresponsive") {
      bgColor = bg_colors[STATUS_ALERT];
    } else if (alert.type == "controlsUnresponsivePermanent") {
      bgColor = bg_colors[STATUS_DISENGAGED];
    }
    alerts->updateAlert(alert, bgColor);
  }

  if (s.scene.map_on_left) {
    split->setDirection(QBoxLayout::LeftToRight);
  } else {
    split->setDirection(QBoxLayout::RightToLeft);
  }

  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
  }
  update();
}

void OnroadWindow::mouseReleaseEvent(QMouseEvent* e) {

  QPoint endPos = e->pos();
  int dx = endPos.x() - startPos.x();
  int dy = endPos.y() - startPos.y();
  if(std::abs(dx) > 250 || std::abs(dy) > 200) {

    if(std::abs(dx) < std::abs(dy)) {

      if(dy < 0) { // upward
        Params().remove("CalibrationParams");
        Params().remove("LiveParameters");
        QTimer::singleShot(1500, []() {
          Params().putBool("SoftRestartTriggered", true);
        });

        QSound::play("../assets/sounds/reset_calibration.wav");
      }
      else { // downward
        QTimer::singleShot(500, []() {
          Params().putBool("SoftRestartTriggered", true);
        });
      }
    }
    else if(std::abs(dx) > std::abs(dy)) {
      if(dx < 0) { // right to left
        if(recorder)
          recorder->toggle();
      }
      else { // left to right
        if(recorder)
          recorder->toggle();
      }
    }

    return;
  }

    int tap_x = endPos.x();
    int tap_y = endPos.y();
    if (tap_x > 20 && tap_x < 200 &&
        tap_y > height() - 140 && tap_y < height() - 40) {
      int cur = (*(uiState()->sm))["controlsState"].getControlsState().getMyDrivingMode();
      if (cur < 1 || cur > 4) cur = 3;
      int next = cur % 4 + 1;   // SAFE→ECO→NORM→FAST→SAFE
      Params().put("MyDrivingMode", std::to_string(next));
      return;
    }

  if (map != nullptr && mapbox_enabled) {
    bool sidebarVisible = geometry().x() > 0;
    map->setVisible(!sidebarVisible && !map->isVisible());
  }

  // propagation event to parent(HomeWindow)
  QWidget::mouseReleaseEvent(e);
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  startPos = e->pos();
  //QWidget::mousePressEvent(e);
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && (uiState()->prime_type || !MAPBOX_TOKEN.isEmpty())) {
      MapWindow * m = new MapWindow(get_mapbox_settings());
      map = m;

      QObject::connect(uiState(), &UIState::offroadTransition, m, &MapWindow::offroadTransition);

      m->setFixedWidth(topWidget(this)->width() / 2);
      split->addWidget(m, 0, Qt::AlignRight);

      // Make map visible after adding to split
      m->offroadTransition(offroad);
      mapbox_enabled = params.getBool("ShowMapboxMap");
      mapbox_param_initialized = true;
      m->setMapEnabled(mapbox_enabled);
    }
  }
#endif

  alerts->updateAlert({}, bg);

  // update stream type
  bool wide_cam = Hardware::TICI() && Params().getBool("EnableWideCamera");
  nvg->setStreamType(wide_cam ? VISION_STREAM_RGB_WIDE_ROAD : VISION_STREAM_RGB_ROAD);

  if(offroad && recorder) {
    recorder->stop(false);
  }

}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  const QColor state_color(bg.red(), bg.green(), bg.blue(), 255);
  p.fillRect(rect(), state_color);

  if (brake_lights) {
    p.fillRect(QRect(0, height() - bdr_s, width(), bdr_s), QColor(255, 105, 105));
  }

  const QColor bsd_color(255, 215, 0);
  if (left_blindspot) {
    p.fillRect(QRect(0, 0, bdr_s, height()), bsd_color);
  }
  if (right_blindspot) {
    p.fillRect(QRect(width() - bdr_s, 0, bdr_s, height()), bsd_color);
  }

  constexpr int steer_axis_width = 8;
  const float steer = std::clamp(steering_angle_deg, -90.0f, 90.0f);
  const float center_x = width() / 2.0f;
  const float steer_end_x = std::clamp(center_x - center_x * steer / 90.0f, 0.0f, (float)width());
  const QColor steer_orange(255, 145, 40);

  if (std::abs(steer) > 0.1f) {
    QColor faded_orange = steer_orange;
    faded_orange.setAlpha(20);

    // Keep the center vivid and fade toward the steering direction.
    QLinearGradient steer_gradient(center_x, 0.0f, steer_end_x, 0.0f);
    steer_gradient.setColorAt(0.0, steer_orange);
    steer_gradient.setColorAt(1.0, faded_orange);
    p.fillRect(QRectF(std::min(center_x, steer_end_x), 0.0f,
                      std::abs(steer_end_x - center_x), bdr_s), steer_gradient);
  }

  // Fixed center axis makes the neutral point visible at every steering angle.
  p.fillRect(QRectF(center_x - steer_axis_width / 2.0f, 0.0f, steer_axis_width, bdr_s), steer_orange);
}

// ***** onroad widgets *****

// OnroadAlerts
void OnroadAlerts::updateAlert(const Alert &a, const QColor &color) {
  if (!alert.equal(a) || color != bg) {
    alert = a;
    bg = color;
    update();
  }
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::ControlsState::AlertSize::NONE) {
    return;
  }
  static std::map<cereal::ControlsState::AlertSize, const int> alert_sizes = {
    {cereal::ControlsState::AlertSize::SMALL, 271},
    {cereal::ControlsState::AlertSize::MID, 420},
    {cereal::ControlsState::AlertSize::FULL, height()},
  };
  int h = alert_sizes[alert.size];
  QRect r = QRect(0, height() - h, width(), h);

  QPainter p(this);

  // 텍스트만 남기고 상태색(경고/주의 등)을 나타내던 하단 배경 박스와
  // 그라디언트는 그리지 않는다. r 은 레이아웃 계산에는 계속 사용한다.
  p.setRenderHint(QPainter::TextAntialiasing);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  if (alert.size == cereal::ControlsState::AlertSize::SMALL) {
    configFont(p, "Open Sans", 59, "Bold");
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    configFont(p, "Open Sans", 70, "Bold");
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    configFont(p, "Open Sans", 53, "SemiBold");
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    configFont(p, "Open Sans", l ? 106 : 142, "Bold");
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    configFont(p, "Open Sans", 70, "SemiBold");
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}

// NvgWindow

NvgWindow::NvgWindow(VisionStreamType type, QWidget* parent) : last_update_params(0), fps_filter(UI_FREQ, 3, 1. / UI_FREQ), accel_filter(UI_FREQ, .5, 1. / UI_FREQ), CameraViewWidget("camerad", type, true, parent) {
}

void NvgWindow::initializeGL() {
  CameraViewWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);

  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size - 5, img_size - 5});
	
  // neokii
  ic_nda = QPixmap("../assets/images/img_nda.png");
  ic_hda = QPixmap("../assets/images/img_hda.png");
  ic_tire_pressure = QPixmap("../assets/images/img_tire_pressure.png");
  ic_speed_bg = QPixmap("../assets/images/speed_bg.png");
  ic_speed_bump = QPixmap("../assets/images/speed_bump.png");
}

void NvgWindow::updateState(const UIState &s) {	
  const SubMaster &sm = *(s.sm);
  const auto cs = sm["controlsState"].getControlsState();

  setProperty("status", s.status);

  // update engageability and DM icons at 2Hz
  if (sm.frame % (UI_FREQ / 2) == 0) {
    setProperty("engageable", cs.getEngageable() || cs.getEnabled());
    setProperty("experimentalMode", cs.getExperimentalMode());
  }

}

void NvgWindow::updateFrameMat(int w, int h) {
  CameraViewWidget::updateFrameMat(w, h);

  UIState *s = uiState();
  s->fb_w = w;
  s->fb_h = h;
  auto intrinsic_matrix = s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
  float zoom = ZOOM / intrinsic_matrix.v[0];
  if (s->wide_camera) {
    zoom *= 0.5;
  }
  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2, h / 2 + y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

void NvgWindow::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();
	
  const UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);
	
  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    painter.drawPolygon(scene.lane_line_vertices[i].v, scene.lane_line_vertices[i].cnt);
  }

  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));
    painter.drawPolygon(scene.road_edge_vertices[i].v, scene.road_edge_vertices[i].cnt);
  }
	
  // C3 path mode 14: split the projected vehicle corridor into left/right
  // bands, leaving a narrow 10% gap through the center.
  QColor path_color;
  if (show_path_status_color) {
    path_color = QColor(0, 153, 0, 120);  // engaged, no lead
    if (!s->engaged()) {
      path_color = QColor(0, 0, 0, 120);
    } else {
      const auto lead = sm["radarState"].getRadarState().getLeadOne();
      const auto accels = sm["longitudinalPlan"].getLongitudinalPlan().getAccels();
      const float accel = accels.size() > 0 ? accels[0] : 0.0f;
      if (lead.getStatus()) {
        if (std::abs(accel) < 0.5f) path_color = QColor(255, 255, 0, 120);  // steady
        else if (accel >= 0.5f)     path_color = QColor(255, 153, 0, 120);  // accelerating
        else                        path_color = QColor(255, 0, 0, 120);    // decelerating
      }
    }
  } else {
    path_color = QColor::fromHslF(197 / 360., 1.0, 0.55, 0.7);
  }

  // Braking is indicated only by the bottom status bar.
  painter.setPen(Qt::NoPen);
  painter.setBrush(path_color);
  const int track_count = scene.track_vertices.cnt;
  const int half = track_count / 2;
  if (half >= 2 && track_count % 2 == 0 && track_count <= TRAJECTORY_SIZE * 2) {
    // Fixed-size stack buffers avoid per-frame heap allocation on EON.
    std::array<QPointF, TRAJECTORY_SIZE * 2> left_path;
    std::array<QPointF, TRAJECTORY_SIZE * 2> right_path;

    // track_vertices contains one path edge followed by the opposite edge
    // in reverse order. Pair both sides and reproduce C3 mode 14 (45/10/45).
    for (int i = 0; i < half; ++i) {
      const QPointF &left = scene.track_vertices.v[i];
      const QPointF &right = scene.track_vertices.v[track_count - 1 - i];
      left_path[i] = left;
      right_path[i] = left + (right - left) * 0.55;
    }
    for (int i = half - 1; i >= 0; --i) {
      const QPointF &left = scene.track_vertices.v[i];
      const QPointF &right = scene.track_vertices.v[track_count - 1 - i];
      const int reverse_idx = track_count - 1 - i;
      left_path[reverse_idx] = left + (right - left) * 0.45;
      right_path[reverse_idx] = right;
    }

    painter.drawPolygon(left_path.data(), track_count);
    painter.drawPolygon(right_path.data(), track_count);
  } else {
    painter.drawPolygon(scene.track_vertices.v, track_count);
  }

  painter.restore();
}

// S9 외부 HUD 가 실제로 살아 있는지.
//
// EonClusterHudConnected 만 보면 remote_hud 프로세스가 True 를 남긴 채
// 죽었을 때 EON 화면이 영영 비어 있게 된다. 그래서 remote_hud 가 2초마다
// 갱신하는 EonClusterHudHeartbeat(epoch 초) 도 같이 확인한다.
// 하트비트가 10초 이상 멈추면 죽은 것으로 보고 EON 이 다시 그린다.
//
// 파라미터는 파일 I/O 라 1초에 한 번만 읽는다.
static bool s9HudActive() {
  static Params s9_params;
  static uint64_t last_check = 0;
  static bool active = false;

  const uint64_t now = millis_since_boot();
  if (last_check != 0 && now - last_check < 1000) {
    return active;
  }
  last_check = now;

  // 2026-08-19: EonClusterHudConnected(폰 ACK) 조건을 뺐다.
  // ACK 는 와이파이가 잠깐 흔들리기만 해도 2초 만에 False 가 되고, 그때마다
  // EON 이 지도·ATC 박스를 다시 그렸다. 이제는 remote_hud 프로세스가 살아
  // 있는지(하트비트)만 본다. 프로세스가 죽거나 EonClusterHud 를 끄면
  // 10초 뒤 EON 화면이 원래대로 돌아온다.
  active = false;
  const std::string beat = s9_params.get("EonClusterHudHeartbeat");
  if (!beat.empty()) {
    const long long stamp = atoll(beat.c_str());
    const long long wall = (long long)time(NULL);
    active = stamp > 0 && wall >= stamp && (wall - stamp) < 10;
  }
  return active;
}

void NvgWindow::paintGL() {
}

void NvgWindow::paintEvent(QPaintEvent *event) {


  UIState *s = uiState();
  const cereal::ModelDataV2::Reader &model = (*s->sm)["modelV2"].getModelV2();

  QPainter p(this);

  // Keep the complete EON driving UI independent from the S9/USB HUD state.
  // A live S9 UDP acknowledgement does not prove that TURZX USB output works.
  p.beginNativePainting();
  CameraViewWidget::paintGL();
  p.endNativePainting();

  if (s->worldObjectsVisible())
    drawHud(p, model);

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  double fps = fps_filter.update(1. / dt * 1000);
  if (fps < 15 && cur_draw_t - last_slow_fps_log_t >= 5000) {
    LOGW("slow frame rate: %.2f fps", fps);
    last_slow_fps_log_t = cur_draw_t;
  }
  prev_draw_t = cur_draw_t;
}

void NvgWindow::showEvent(QShowEvent *event) {
  CameraViewWidget::showEvent(event);

  auto now = millis_since_boot();
  if(now - last_update_params > 1000*5) {
    last_update_params = now;
    ui_update_params(uiState());
  }

  prev_draw_t = millis_since_boot();
}

void NvgWindow::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void NvgWindow::drawTextWithColor(QPainter &p, int x, int y, const QString &text, QColor& color) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(color);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void NvgWindow::drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity, bool rotation, float angle) {
  p.save();
  p.setOpacity(opacity);
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);

  if (rotation) {
    p.translate(x, y);
    p.rotate(-angle);           // 조향각만큼 회전 (반시계)
    QRect r = img.rect();
    r.moveCenter(QPoint(0, 0));
    p.drawPixmap(r, img);
  } else {
    p.drawPixmap(x - img_size / 2, y - img_size / 2, img_size, img_size, img);
  }

  p.restore();
}

void NvgWindow::drawText2(QPainter &p, int x, int y, int flags, const QString &text, const QColor& color) {
  QFontMetrics fm(p.font());
  QRect rect = fm.boundingRect(text);
  rect.adjust(-1, -1, 1, 1);
  p.setPen(color);
  p.drawText(QRect(x, y, rect.width()+1, rect.height()), flags, text);
}

void NvgWindow::drawHud(QPainter &p, const cereal::ModelDataV2::Reader &model) {

  p.setRenderHint(QPainter::Antialiasing);
  p.setPen(Qt::NoPen);
  p.setOpacity(1.);

  // Header gradient
  QLinearGradient bg(0, header_h - (header_h / 2.5), 0, header_h);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), header_h, bg);

  UIState *s = uiState();

  const SubMaster &sm = *(s->sm);
  (void)sm;
  (void)model;

  // S9 외부 HUD 가 붙어 있으면 내비 패널은 그쪽에서 이미 그린다.
  // 차선·경로·리드는 EON 화면에서 조향 확인용으로 계속 남긴다.
  const bool s9_hud = s9HudActive();

  drawLaneLines(p, s);
  drawCarrotPlot(p);
  drawCarrotLead(p);
  // 이미지 로드만 건너뛰고 JSON 파싱은 계속한다 —
  // drawCarrotHud 의 MAP/NDA/HDA 표시가 이 상태를 쓴다.
  updateCarrotNavi(!s9_hud);
  if (!s9_hud) {
    drawCarrotNavi(p);
  }
  drawCarrotHud(p);
  drawSpeedLimit(p);
  drawCarrotInfo(p);
  drawCarrotBottom(p);

  if (sm["carControl"].getCarControl().getHudControl().getSoftHold()) {
    p.save();
    // The bottom lane-status row starts at height() - 62; leave a 30 px gap.
    const QRect soft_hold_rect(0, height() - 172, width(), 80);
    configFont(p, "Open Sans", 59, "Bold");
    p.setPen(QColor(0, 0, 0, 220));
    p.drawText(soft_hold_rect.translated(3, 4), Qt::AlignHCenter | Qt::AlignBottom, "SOFTHOLD");
    p.setPen(QColor(255, 255, 255));
    p.drawText(soft_hold_rect, Qt::AlignHCenter | Qt::AlignBottom, "SOFTHOLD");
    p.restore();
  }

  drawBottomIcons(p);

  drawTextAnim(p);   // 팝업 애니메이션은 항상 맨 위
}

#include "selfdrive/ui/qt/onroad_navi.inc"
#include "selfdrive/ui/qt/onroad_plot.inc"

static const QColor get_tpms_color(float tpms) {
    if(tpms < 5 || tpms > 60) // N/A
        return QColor(255, 255, 255, 220);
    if(tpms < 31)
        return QColor(255, 90, 90, 220);
    return QColor(255, 255, 255, 220);
}

static const QString get_tpms_text(float tpms) {
    if(tpms < 5 || tpms > 60)
        return "";

    char str[32];
    snprintf(str, sizeof(str), "%.0f", round(tpms));
    return QString(str);
}

void NvgWindow::drawBottomIcons(QPainter &p) {
  p.save();
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  auto scc_smoother = sm["carControl"].getCarControl().getSccSmoother();

  // tire pressure  --- carrot hud panel과 위치가 겹쳐서 비활성화 (필요하면 if(false) -> if(true))
  if (false) {
    const int w = 58;
    const int h = 126;
    const int x = 110;
    const int y = height() - h - 85;

    auto tpms = car_state.getTpms();
    const float fl = tpms.getFl();
    const float fr = tpms.getFr();
    const float rl = tpms.getRl();
    const float rr = tpms.getRr();

    p.setOpacity(0.8);
    p.drawPixmap(x, y, w, h, ic_tire_pressure);

    configFont(p, "Open Sans", 38, "Bold");

    QFontMetrics fm(p.font());
    QRect rcFont = fm.boundingRect("9");

    int center_x = x + 3;
    int center_y = y + h/2;
    const int marginX = (int)(rcFont.width() * 2.7f);
    const int marginY = (int)((h/2 - rcFont.height()) * 0.7f);

    drawText2(p, center_x-marginX, center_y-marginY-rcFont.height(), Qt::AlignRight, get_tpms_text(fl), get_tpms_color(fl));
    drawText2(p, center_x+marginX, center_y-marginY-rcFont.height(), Qt::AlignLeft, get_tpms_text(fr), get_tpms_color(fr));
    drawText2(p, center_x-marginX, center_y+marginY, Qt::AlignRight, get_tpms_text(rl), get_tpms_color(rl));
    drawText2(p, center_x+marginX, center_y+marginY, Qt::AlignLeft, get_tpms_text(rr), get_tpms_color(rr));
  }

  // cruise gap / brake / auto hold --- carrot hud panel 로 대체되어 제거함
  (void)scc_smoother;
  (void)car_state;

  // 현재 시간과 날짜를 항상 표시 (carrot.cc :: drawDateTime 과 동일 위치/크기)
  const int dt_x = 170;
  const int dt_y = 185;   // 상단 정보줄과 겹치지 않도록 내림 (org: 120)
  QDateTime now = QDateTime::currentDateTime();

  p.setOpacity(1.0);
  ctText(p, dt_x, dt_y, now.toString("HH:mm"), 100, QColor(255, 255, 255, 255), true, true);

  // tm_wday 순서와 맞추기 위해 일요일부터
  static const char *weekdays_ko[] = {"일", "월", "화", "수", "목", "금", "토"};
  int wday = now.date().dayOfWeek() % 7;   // Qt: 1=월 ... 7=일  ->  0=일 ... 6=토
  QString date_str = now.toString("MM-dd") + "(" +
                     QString::fromUtf8(weekdays_ko[wday]) + ")";
  ctText(p, dt_x, dt_y + 70, date_str, 60, QColor(255, 255, 255, 255), true, true);


  p.restore();
}

#define CT_WHITE        QColor(255, 255, 255, 255)
#define CT_BLACK_A(a)   QColor(0, 0, 0, a)
#define CT_GREEN        QColor(0, 203, 0, 255)
#define CT_GREEN_A(a)   QColor(0, 203, 0, a)
#define CT_OCHRE        QColor(218, 111, 37, 255)
#define CT_ORANGE_A(a)  QColor(255, 175, 3, a)
#define CT_RED_A(a)     QColor(201, 34, 49, a)
#define CT_YELLOW_A(a)  QColor(218, 202, 37, a)
#define CT_BLUE_A(a)    QColor(0, 0, 255, a)
#define CT_GREY_A(a)    QColor(191, 191, 191, a)
#define CT_WHITE_A(a)   QColor(255, 255, 255, a)

void NvgWindow::ctRect(QPainter &p, const QRect &r, const QColor &fill, int corner,
                       int borderWidth, const QColor &borderColor) {
  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  if (borderWidth > 0) p.setPen(QPen(borderColor, borderWidth));
  else                 p.setPen(Qt::NoPen);
  p.setBrush(QBrush(fill));
  p.drawRoundedRect(r, corner, corner);
  p.restore();
}

// nanovg 의 NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM 과 동일하게 동작:
// x = 가로 중심, y = 글자 하단
void NvgWindow::ctText(QPainter &p, int x, int y, const QString &text, int size,
                       const QColor &color, bool bold, bool shadow) {
  if (text.isEmpty()) return;
  configFont(p, "Open Sans", size, bold ? "Bold" : "Regular");
  const int h = (int)(size * 1.55f);
  QRect r(x - 700, y - h, 1400, h);
  if (shadow) {
    p.setPen(CT_BLACK_A(200));
    p.drawText(r.translated(3, 4), Qt::AlignHCenter | Qt::AlignBottom, text);
  }
  p.setPen(color);
  p.drawText(r, Qt::AlignHCenter | Qt::AlignBottom, text);
}

void NvgWindow::ctTextIn(QPainter &p, const QRect &box, const QString &text, int size,
                         const QColor &color, bool bold) {
  if (text.isEmpty()) return;
  configFont(p, "Open Sans", size, bold ? "Bold" : "Regular");
  QFontMetrics fm(p.font());

  int cap = fm.capHeight();
  if (cap <= 0) cap = (int)(fm.ascent() * 0.72f);   // 폰트가 capHeight 를 못주면 근사

  int w = fm.boundingRect(text).width();
  int baseline = box.center().y() + cap / 2;

  p.setPen(color);
  p.drawText(box.x() + (box.width() - w) / 2, baseline, text);
}

void NvgWindow::drawCarrotInfo(QPainter &p) {
  const uint64_t now = millis_since_boot();
  const QSize cache_size(width(), 72);
  const bool cache_invalid = carrot_info_cache.isNull() || carrot_info_cache_size != cache_size;
  if (cache_invalid) {
    carrot_info_cache = QImage(cache_size, QImage::Format_ARGB32_Premultiplied);
    carrot_info_cache_size = cache_size;
  }
  if (cache_invalid || now - carrot_info_last_render >= 200) {
    carrot_info_cache.fill(Qt::transparent);
    QPainter cache_painter(&carrot_info_cache);
    cache_painter.setRenderHint(QPainter::Antialiasing);
    cache_painter.setRenderHint(QPainter::TextAntialiasing);
    drawCarrotInfoContent(cache_painter);
    cache_painter.end();
    carrot_info_last_render = now;
  }
  p.drawImage(QPoint(0, 0), carrot_info_cache);
}

void NvgWindow::drawCarrotInfoContent(QPainter &p) {
  p.save();

  const SubMaster &sm = *(uiState()->sm);
  const auto car_params      = sm["carParams"].getCarParams();
  const auto controls_state  = sm["controlsState"].getControlsState();
  const auto live_params     = sm["liveParameters"].getLiveParameters();
  const auto torque_state    = controls_state.getLateralControlState().getTorqueState();

  // ---- 좌상단 : wifi IP / SCC ----
  QString ip_head = QString::fromUtf8(sm["deviceState"].getDeviceState().getWifiIpAddress().cStr());
  if (ip_head.isEmpty()) ip_head = "--";

  int scc_bus = (int)car_params.getSccBus();
  QString scc_num  = (scc_bus < 0) ? QString("none") : QString::number(scc_bus);
  QString scc_tail = QString(car_params.getHasScc13() ? "+13" : "") +
                     QString(car_params.getHasScc14() ? "+14" : "");
  QString left_head = ip_head + "  SCC ";

  // ---- 우상단 : 토크값 / SR ----
  QString right_str;
  right_str.sprintf("LT(%.2f/%.3f)  SR(%.2f/%.2f)",
                    torque_state.getLatAccelFactor(),
                    torque_state.getFriction(),
                    controls_state.getSteerRatio(),
                    live_params.getSteerRatio());

  configFont(p, "Open Sans", 34, "Regular");
  QRect line(35, 12, width() - 55, 48);   // 상태바(bdr_s=20) 안쪽

  // 우상단은 한 덩어리
  p.setPen(QColor(0xff, 0xff, 0xff, 200));
  p.drawText(line, Qt::AlignRight | Qt::AlignVCenter, right_str);

  // 좌상단 : 차량명은 보통 굵기, SCC 부분은 전부 흰색 + 굵게
  {
    QFontMetrics fm_r(p.font());
    int cap = fm_r.capHeight();
    if (cap <= 0) cap = (int)(fm_r.ascent() * 0.72f);
    int base_y = line.center().y() + cap / 2;
    int tx = line.x();

    QString name_part = left_head;
    name_part.chop(4);                          // 뒤쪽 "SCC " 분리

    p.setPen(QColor(0xff, 0xff, 0xff, 200));
    p.drawText(tx, base_y, name_part);
    tx += fm_r.horizontalAdvance(name_part);

    // "SCC" 라벨 : 흰색 굵은 글씨
    configFont(p, "Open Sans", 34, "Bold");
    QFontMetrics fm_b(p.font());
    p.setPen(QColor(255, 255, 255, 255));
    p.drawText(tx, base_y, "SCC ");
    tx += fm_b.horizontalAdvance("SCC ");

    // 번호 : 빨간 배지 + 흰 글씨
    {
      int num_w = fm_b.horizontalAdvance(scc_num);
      QRect badge(tx, line.center().y() - 21, num_w + 22, 42);
      ctRect(p, badge, QColor(190, 0, 0, 255), 8);
      ctTextIn(p, badge, scc_num, 34, QColor(255, 255, 255, 255));
      tx += badge.width() + 6;
    }

    // 부가표기(+13/+14) : 흰색 굵은 글씨
    if (!scc_tail.isEmpty()) {
      configFont(p, "Open Sans", 34, "Bold");
      p.setPen(QColor(255, 255, 255, 255));
      p.drawText(tx, base_y, scc_tail);
    }

    configFont(p, "Open Sans", 34, "Regular");  // 이후 그리기용으로 원복
  }

  p.restore();
}

void NvgWindow::drawCarrotBottom(QPainter &p) {
  const uint64_t now = millis_since_boot();
  const int cache_y = std::max(0, height() - 72);
  const QSize cache_size(width(), height() - cache_y);
  const bool cache_invalid = carrot_bottom_cache.isNull() || carrot_bottom_cache_size != cache_size;
  if (cache_invalid) {
    carrot_bottom_cache = QImage(cache_size, QImage::Format_ARGB32_Premultiplied);
    carrot_bottom_cache_size = cache_size;
  }
  if (cache_invalid || now - carrot_bottom_last_render >= 200) {
    carrot_bottom_cache.fill(Qt::transparent);
    QPainter cache_painter(&carrot_bottom_cache);
    cache_painter.setRenderHint(QPainter::Antialiasing);
    cache_painter.setRenderHint(QPainter::TextAntialiasing);
    cache_painter.translate(0, -cache_y);
    drawCarrotBottomContent(cache_painter);
    cache_painter.end();
    carrot_bottom_last_render = now;
  }
  p.drawImage(QPoint(0, cache_y), carrot_bottom_cache);
}

void NvgWindow::drawCarrotBottomContent(QPainter &p) {
  p.save();

  const SubMaster &sm = *(uiState()->sm);

  const int line_y = height() - 62;
  const int line_h = 48;

  QString lat_debug = QString::fromUtf8(
      sm["lateralPlan"].getLateralPlan().getLatDebugText().cStr());
  {
    int off = lat_debug.indexOf("offset", 0, Qt::CaseInsensitive);
    if (off >= 0) {
      int sep = lat_debug.lastIndexOf('|', off);
      lat_debug = (sep >= 0 ? lat_debug.left(sep) : lat_debug.left(off)).trimmed();
    }
  }
  if (!lat_debug.isEmpty()) {
    int right_limit = width() - 240;
    int left_limit = 240;
    int avail = right_limit - left_limit;

    if (avail > 100) {
      if (lat_debug != lat_debug_font_text || avail != lat_debug_font_width) {
        lat_debug_font_text = lat_debug;
        lat_debug_font_width = avail;
        lat_debug_font_size = 34;
        for (; lat_debug_font_size > 22; lat_debug_font_size -= 2) {
          configFont(p, "Open Sans", lat_debug_font_size, "Regular");
          if (QFontMetrics(p.font()).boundingRect(lat_debug).width() <= avail) break;
        }
      } else {
        configFont(p, "Open Sans", lat_debug_font_size, "Regular");
      }
      p.setPen(QColor(0xff, 0xff, 0xff, 200));
      p.drawText(QRect(left_limit, line_y, avail, line_h),
                 Qt::AlignHCenter | Qt::AlignVCenter, lat_debug);
    }
  }

  p.restore();
}

void NvgWindow::ctTextAnimStart(int x, int y, const QString &text, int size, const QColor &color, bool enabled) {
  if (!enabled) return;
  anim_x = x;
  anim_y = y;
  anim_text = text;
  anim_size = size;
  anim_color = color;
  anim_time = 130;
}

void NvgWindow::drawTextAnim(QPainter &p) {
  if (anim_time <= 0) return;
  anim_time -= 10;
  if (anim_time <= 0) { anim_time = 0; return; }

  int t = std::min(anim_time, 100);          // 보간 계수 100(시작) → 0(도착)
  const int a_max = 100;
  int x    = (width() / 2 * t + anim_x * (a_max - t)) / a_max;
  int y    = ((height() - 400) * t + anim_y * (a_max - t)) / a_max;
  int size = (350 * t + anim_size * (a_max - t)) / a_max;
  if (size < 1) return;

  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  p.setRenderHint(QPainter::TextAntialiasing);
  // 등장 직후에는 두꺼운 검정 외곽선을 덧대 강조
  ctText(p, x, y, anim_text, size, anim_color, true, anim_time >= 100);
  p.restore();
}

// 리드 박스 폭 제한 (앞차가 가까울 때 화면을 다 덮지 않도록)
#define LEAD_BOX_MIN_W 180.0f
#define LEAD_BOX_MAX_W 320.0f

void NvgWindow::drawCarrotLead(QPainter &p) {
  p.save();
  p.setRenderHint(QPainter::Antialiasing);

  UIState *s = uiState();
  const UIScene &scene = s->scene;
  const bool is_metric = scene.is_metric;
  const float m_to_disp = is_metric ? 1.0f : (float)METER_TO_FOOT;

  // ---- 리드2 프레임 (있을 때만, 뒤쪽에 먼저) ----
  if (scene.lead_status[1]) {
    float xl = scene.lead_left[1].x();
    float xr = scene.lead_right[1].x();
    float ly = scene.lead_left[1].y();
    float w2 = std::clamp(xr - xl, 80.0f, LEAD_BOX_MAX_W * 0.8f);
    QRect box2((int)(xl - 10), (int)(ly - w2 * 0.8f), (int)(w2 + 20), (int)(w2 * 0.8f));
    p.setPen(QPen(QColor(218, 111, 37, 255), 2));
    p.setBrush(QColor(0, 0, 0, 10));
    p.drawRoundedRect(box2, 15, 15);
  }

  // ---- 리드1 프레임 ----
  if (scene.lead_status[0]) {
    float xl = scene.lead_left[0].x();
    float xr = scene.lead_right[0].x();
    float ly = scene.lead_left[0].y();
    // 앞차가 가까우면 투영 폭이 폭발하므로 상한을 두고, 프레임 간 EMA 로 눌러준다.
    float w_raw = std::clamp(xr - xl, LEAD_BOX_MIN_W, LEAD_BOX_MAX_W);
    float cx_raw = std::clamp((xl + xr) / 2.0f, 200.0f, (float)width() - 200.0f);
    float cy_raw = std::clamp(ly, 150.0f, (float)height() - 120.0f);

    const float a = 0.85f;   // 클수록 부드럽고 느리다
    if (lead_box_w <= 0.0f) { lead_box_w = w_raw; lead_box_x = cx_raw; lead_box_y = cy_raw; }
    lead_box_w = lead_box_w * a + w_raw * (1.0f - a);
    lead_box_x = lead_box_x * a + cx_raw * (1.0f - a);
    lead_box_y = lead_box_y * a + cy_raw * (1.0f - a);

    float w1 = lead_box_w;
    float cx = lead_box_x;
    ly = lead_box_y;

    // 레이더가 잡은 리드면 주황, 비전만이면 파랑
    QColor stroke = scene.lead_radar[0] ? QColor(255, 175, 3, 255) : QColor(0, 0, 255, 255);
    QRect box((int)(cx - w1 / 2 - 10), (int)(ly - w1 * 0.8f), (int)(w1 + 20), (int)(w1 * 0.8f));
    p.setPen(QPen(stroke, 3));
    p.setBrush(QColor(0, 0, 0, 10));
    p.drawRoundedRect(box, 15, 15);

    // ---- 거리 두 개 : 좌 레이더 / 우 비전 ----
    int dy = (int)ly + 60;
    QString str;

    if (scene.lead_radar_dist > 0.0f) {
      str.sprintf("%.1f", scene.lead_radar_dist * m_to_disp);
      QRect tag((int)(cx - 80 - 45), dy - 35, 90, 42);
      ctRect(p, tag, QColor(255, 175, 3, 255), 15);
      ctTextIn(p, tag, str, 40, QColor(255, 255, 255, 255));
    }
    if (scene.lead_vision_dist > 0.0f) {
      str.sprintf("%.1f", scene.lead_vision_dist * m_to_disp);
      QRect tag((int)(cx + 80 - 45), dy - 35, 90, 42);
      ctRect(p, tag, QColor(0, 0, 255, 255), 15);
      ctTextIn(p, tag, str, 40, QColor(255, 255, 255, 255));
    }
  }

  if (!scene.lead_status[0]) lead_box_w = 0.0f;   // 리드 사라지면 EMA 초기화

  p.restore();
}

void NvgWindow::drawCarrotDeviceState(QPainter &p) {
  const int bx = 140;
  const int by = height() - 230;
  const QRect cache_rect(20, by - 245, 470, 110);
  const QSize cache_size = cache_rect.size();
  const uint64_t now = millis_since_boot();

  const bool cache_invalid = carrot_device_state_cache.isNull() || carrot_device_state_cache_size != cache_size;
  if (cache_invalid) {
    carrot_device_state_cache = QImage(cache_size, QImage::Format_ARGB32_Premultiplied);
    carrot_device_state_cache_size = cache_size;
  }
  if (cache_invalid || now - carrot_device_state_last_render >= 200) {
    carrot_device_state_cache.fill(Qt::transparent);
    QPainter panel(&carrot_device_state_cache);
    panel.setRenderHint(QPainter::Antialiasing);
    panel.setRenderHint(QPainter::TextAntialiasing);
    panel.translate(-cache_rect.x(), -cache_rect.y());

    const SubMaster &sm = *(uiState()->sm);
    const auto device_state = sm["deviceState"].getDeviceState();
    const auto car_state = sm["carState"].getCarState();
    float cpu_temp = 0.f;
    const auto cpu_temp_c = device_state.getCpuTempC();
    if (std::size(cpu_temp_c) > 0) {
      for (int i = 0; i < (int)std::size(cpu_temp_c); ++i) cpu_temp += cpu_temp_c[i];
      cpu_temp /= (float)std::size(cpu_temp_c);
    }
    float cpu_usage = 0.f;
    const auto cpu_usage_percent = device_state.getCpuUsagePercent();
    if (std::size(cpu_usage_percent) > 0) {
      for (int i = 0; i < (int)std::size(cpu_usage_percent); ++i) cpu_usage += cpu_usage_percent[i];
      cpu_usage /= (float)std::size(cpu_usage_percent);
    }

    int dx = bx - 35;
    const int dy = by - 200;
    const QColor box = CT_GREEN_A(190);
    QString str;

    QRect ds_box(dx - 65, dy - 38, 130, 90);
    ctRect(panel, ds_box, (cpu_temp > 80 && blink_timer <= 8) ? CT_RED_A(255) : box, 15, 2);
    ctTextIn(panel, QRect(ds_box.x(), ds_box.y(), ds_box.width(), 34), "CPU", 25, CT_WHITE);
    str.sprintf("%.0f\u00B0C", cpu_temp);
    ctTextIn(panel, QRect(ds_box.x(), ds_box.y() + 34, ds_box.width(), 56), str, 40, CT_WHITE);

    dx += 150;
    ds_box.moveLeft(dx - 65);
    const auto tpms = car_state.getTpms();
    const std::array<float, 4> pressures = {
      tpms.getFl(), tpms.getFr(), tpms.getRl(), tpms.getRr()
    };
    ctRect(panel, ds_box, CT_BLACK_A(220), 15, 2, CT_WHITE_A(170));

    panel.save();
    panel.setPen(QPen(CT_WHITE_A(120), 1));
    panel.drawLine(ds_box.center().x(), ds_box.top() + 3,
                   ds_box.center().x(), ds_box.bottom() - 3);
    panel.drawLine(ds_box.left() + 3, ds_box.center().y(),
                   ds_box.right() - 3, ds_box.center().y());
    panel.restore();

    const int cell_w = ds_box.width() / 2;
    const int cell_h = ds_box.height() / 2;
    const std::array<QRect, 4> cells = {
      QRect(ds_box.left(), ds_box.top(), cell_w, cell_h),
      QRect(ds_box.left() + cell_w, ds_box.top(), ds_box.width() - cell_w, cell_h),
      QRect(ds_box.left(), ds_box.top() + cell_h, cell_w, ds_box.height() - cell_h),
      QRect(ds_box.left() + cell_w, ds_box.top() + cell_h,
            ds_box.width() - cell_w, ds_box.height() - cell_h)
    };
    for (int i = 0; i < 4; ++i) {
      QString pressure = get_tpms_text(pressures[i]);
      if (pressure.isEmpty()) pressure = "--";
      ctTextIn(panel, cells[i], pressure, 40, get_tpms_color(pressures[i]), true);
    }

    dx += 150;
    ds_box.moveLeft(dx - 65);
    ctRect(panel, ds_box, (cpu_usage > 90 && blink_timer <= 8) ? CT_RED_A(255) : box, 15, 2);
    ctTextIn(panel, QRect(ds_box.x(), ds_box.y(), ds_box.width(), 34), "CPU", 25, CT_WHITE);
    str.sprintf("%.0f%%", cpu_usage);
    ctTextIn(panel, QRect(ds_box.x(), ds_box.y() + 34, ds_box.width(), 56), str, 40, CT_WHITE);
    panel.end();

    carrot_device_state_last_render = now;
  }

  p.drawImage(cache_rect.topLeft(), carrot_device_state_cache);
}

void NvgWindow::drawCarrotHud(QPainter &p) {
  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  p.setRenderHint(QPainter::TextAntialiasing);
  p.setOpacity(1.0);

  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  const auto car_state    = sm["carState"].getCarState();
  const auto controls_state = sm["controlsState"].getControlsState();
  const auto scc_smoother = sm["carControl"].getCarControl().getSccSmoother();
  const auto road_limit   = sm["roadLimitSpeed"].getRoadLimitSpeed();

  const bool  is_metric   = s->scene.is_metric;
  const float ms_to_disp  = is_metric ? MS_TO_KPH : MS_TO_MPH;
  const float kph_to_disp = is_metric ? 1.0f : KM_TO_MILE;

  blink_timer = (blink_timer + 1) % 16;

  // 파라미터는 매 프레임 읽지 않고 1초에 한번만
  if (++carrot_param_timer >= UI_FREQ) {
    carrot_param_timer = 0;
    Params params;
    int m = controls_state.getMyDrivingMode();
    my_driving_mode = (m >= 1 && m <= 4) ? m : 3;
    carrot_atc_mode = std::atoi(params.get("CarrotAutoTurnControl").c_str());
    carrot_atc_speed = std::atoi(params.get("CarrotAutoTurnSpeed").c_str());
    carrot_atc_end_time = std::atoi(params.get("CarrotAutoTurnEndTime").c_str());
    carrot_bump_speed = std::atoi(params.get("AutoNaviSpeedBumpSpeed").c_str());
    if (carrot_atc_speed < 30 || carrot_atc_speed > 60) carrot_atc_speed = 30;
    if (carrot_atc_end_time < 2 || carrot_atc_end_time > 12) carrot_atc_end_time = 6;
    if (carrot_bump_speed < 10 || carrot_bump_speed > 100) carrot_bump_speed = 35;
    std::string sga = params.get("ShowGearAnimation");
    show_gear_animation = sga.empty() ? 1 : std::atoi(sga.c_str());
    std::string sch = params.get("ShowCarrotHud");
    show_carrot_hud = sch.empty() ? 1 : std::atoi(sch.c_str());
    std::string spsc = params.get("ShowPathStatusColor");
    show_path_status_color = spsc.empty() ? 1 : std::atoi(spsc.c_str());
    show_route_map_always = params.getBool("ShowRouteMapAlways");
  }

  if (!show_carrot_hud) { p.restore(); return; }

  // ---- 기준 좌표 (carrot.cc 와 동일) ----
  const int x  = 140;
  const int y  = height() - 500;
  const int bx = x;
  const int by = y + 270;

  // ---- 단속 카메라 감지 ----
  const int cam_limit = road_limit.getCamLimitSpeed();
  const int cam_dist  = road_limit.getCamLimitSpeedLeftDist();
  const int cam_type  = road_limit.getCamType();
  const int sec_limit = road_limit.getSectionLimitSpeed();
  const int sec_dist  = road_limit.getSectionLeftDist();
  const bool bump_detected = cam_type == 22 && cam_dist > 0;
  const bool cam_detected = (!bump_detected && cam_limit > 0 && cam_dist > 0) ||
                            (sec_limit > 0 && sec_dist > 0);

  // ---- 패널 배경 ----
  QColor bg_color = CT_BLACK_A(90);
  ctRect(p, QRect(bx - 120, by - 270, 475, 495), bg_color, 30, 2, CT_WHITE);

  // ---- 현재 속도 ----
  float v_ego_disp = std::max(0.0f, (float)car_state.getVEgoCluster()) * ms_to_disp;
  ctText(p, bx, by + 50, QString::number((int)(v_ego_disp + 0.5f)), 120, CT_WHITE, true, true);
  if (!ic_speed_bg.isNull()) {
    p.setOpacity(1.0);
    p.drawPixmap(QRect(bx - 100, by - 60, 350, 150), ic_speed_bg);
  }

  // ---- 크루즈 설정 속도 ----
  float cruise_max = scc_smoother.getCruiseMaxSpeed();
  // Match the cluster's stable openpilot engagement state. SCC12.ACCMode can
  // briefly become zero during brake/override transitions while control stays engaged.
  bool is_cruise_set = controls_state.getEnabled() &&
                       cruise_max >= 10.0f && cruise_max < 255.0f;
  QString cruise_str = is_cruise_set
                     ? QString::number((int)(cruise_max * kph_to_disp + 0.5f))
                     : QString("--");
  ctText(p, bx + 170, by + 20, cruise_str, 60, CT_GREEN, true, true);

  // ---- 실제 적용 속도/원인 : c3-wip carrot.cc 의 desiredSpeed/desiredSource 표시 ----
  float show_speed = 0.0f;      // kph
  QString apply_source = QString::fromUtf8(scc_smoother.getApplySource().cStr());
  float apply_max = scc_smoother.getApplyMaxSpeed();
  if (is_cruise_set && apply_max > 0 && std::abs(apply_max - cruise_max) > 0.5f) {
    show_speed = apply_max;
  }

  // VisionTurnController 커브 감속
  {
    const auto long_plan = sm["longitudinalPlan"].getLongitudinalPlan();
    auto vtc = long_plan.getVisionTurnControllerState();
    bool vtc_active = (vtc == cereal::LongitudinalPlan::VisionTurnControllerState::ENTERING ||
                       vtc == cereal::LongitudinalPlan::VisionTurnControllerState::TURNING);
    if (vtc_active && s->engaged()) {
      float v_turn = long_plan.getVisionTurnSpeed() * 3.6f;   // m/s -> kph
      if (v_turn > 0 && (show_speed <= 0.0f || v_turn < show_speed)) {
        show_speed = v_turn;
        apply_source = "vturn";
      }
    }
  }

  if (show_speed > 0.0f) {
    const QColor apply_color = CT_OCHRE;
    ctText(p, bx + 250, by - 50,  QString::number((int)(show_speed * kph_to_disp + 0.5f)),
           50, apply_color, true, true);
    if (!apply_source.isEmpty()) {
      ctText(p, bx + 250, by - 100, apply_source, 30, apply_color, true, true);
    }
  }

  // ---- 주행모드 (SAFE / ECO / NORM / FAST) ----
  QString mode_str = "NORM";
  QColor  mode_color = CT_GREY_A(210);
  switch (my_driving_mode) {
    case 1: mode_str = "SAFE"; mode_color = CT_ORANGE_A(210); break;
    case 2: mode_str = "ECO";  mode_color = CT_GREEN_A(210);  break;
    case 3: mode_str = "NORM"; mode_color = CT_GREY_A(210);   break;
    case 4: mode_str = "FAST"; mode_color = CT_RED_A(210);    break;
  }
  {
    int dx = bx - 50;
    int dy = by + 175;
    QRect mode_box(dx - 55, dy - 38, 110, 48);
    ctRect(p, mode_box, mode_color, 15, 2);
    ctTextIn(p, mode_box, mode_str, 32, CT_WHITE);
  }

  // ---- 차간거리(GAP) 막대 ----
  int gap = controls_state.getLongCruiseGap();
  {
    int   dx  = bx + 270;
    int   dy  = by + 185;
    float ddy = 80 / 4.0f;
    for (int i = 0; i < gap && i < 4; i++) {
      ctRect(p, QRect(dx, (int)(dy - ddy * (i + 1) + 2), 70, (int)ddy - 2),
             CT_GREEN_A(210), 4, 3, CT_WHITE);
    }
  }

  // ---- 기어 (carrot.cc 와 동일: D 에서는 변속단수 표시) ----
  {
    QString gear_str = "M";
    switch (car_state.getGearShifter()) {
      case cereal::CarState::GearShifter::UNKNOWN: gear_str = "U"; break;
      case cereal::CarState::GearShifter::PARK:    gear_str = "P"; break;
      case cereal::CarState::GearShifter::DRIVE:
        if (car_state.getGearStep() > 0) gear_str = QString::number(car_state.getGearStep());
        else                             gear_str = "D";
        break;
      case cereal::CarState::GearShifter::NEUTRAL: gear_str = "N"; break;
      case cereal::CarState::GearShifter::REVERSE: gear_str = "R"; break;
      case cereal::CarState::GearShifter::SPORT:   gear_str = "S"; break;
      case cereal::CarState::GearShifter::LOW:     gear_str = "L"; break;
      case cereal::CarState::GearShifter::BRAKE:   gear_str = "B"; break;
      case cereal::CarState::GearShifter::ECO:     gear_str = "E"; break;
      default: gear_str = "M"; break;
    }
    int dx = bx + 305;
    int dy = by + 60;
    QRect gear_box(dx - 35, dy - 70, 70, 80);
    ctRect(p, gear_box, CT_GREEN_A(210), 15, 3, CT_WHITE);
    ctTextIn(p, gear_box, gear_str, 70, CT_WHITE);

    // 기어가 바뀌면 팝업 애니메이션 시작
    if (!gear_str_last.isEmpty() && gear_str_last != gear_str) {
      ctTextAnimStart(gear_box.center().x(), gear_box.bottom(), gear_str, 70, CT_WHITE,
                      show_gear_animation != 0);
    }
    gear_str_last = gear_str;
  }

  // ---- MAP / NDA / HDA (carrot 의 APN/APM 자리. roadLimitSpeed.active) ----
  //      목적지 안내 중 : MAP
  //      active < 2    : 일반도로(NDA)
  //      active >= 2   : 고속도로/자동차전용도로(HDA)
  {
    int active = road_limit.getActive();
    int dx = bx + 200;
    int dy = by + 175;
    QRect nda_box(dx - 55, dy - 38, 110, 48);

    const qint64 wall_now = QDateTime::currentMSecsSinceEpoch();
    bool navi_active = carrot_navi_route.size() >= 2 &&
                       (carrot_navi_remain_distance > 0 || carrot_navi_remain_time > 0) &&
                       carrot_navi_updated_at != 0 &&
                       (wall_now - static_cast<qint64>(carrot_navi_updated_at)) <= 35000;

    if (navi_active) {
      ctRect(p, nda_box, CT_BLUE_A(210), 15, 2);
      ctTextIn(p, nda_box, "MAP", 40, CT_WHITE);
    } else if (active >= 2) {
      ctRect(p, nda_box, CT_GREEN, 15, 2);
      ctTextIn(p, nda_box, "HDA", 40, CT_WHITE);
    } else if (active > 0) {
      ctRect(p, nda_box, CT_GREEN, 15, 2);
      ctTextIn(p, nda_box, "NDA", 40, CT_WHITE);
    } else {
      ctRect(p, nda_box, CT_RED_A(210), 15, 2);
      ctTextIn(p, nda_box, "NDA", 40, CT_WHITE);
    }
  }

  // ---- LIMIT / CAM ----
  {
    int dx = bx + 75;
    int dy = by + 175;
    int disp_speed = 0;
    QColor limit_color;
    QColor limit_text_color = CT_WHITE;

    if (bump_detected) {
      disp_speed = (int)(carrot_bump_speed * kph_to_disp + 0.5f);
      limit_color = CT_YELLOW_A(210);
      limit_text_color = CT_BLACK_A(230);
      ctText(p, dx, dy - 45, "BUMP", 30, CT_WHITE, true);
    } else if (cam_detected) {
      int limit = (cam_dist > 0) ? cam_limit : sec_limit;
      disp_speed = (int)(limit * kph_to_disp + 0.5f);
      limit_color = (blink_timer <= 8) ? CT_RED_A(210) : CT_YELLOW_A(210);
      ctText(p, dx, dy - 45, "CAM", 30, CT_WHITE, true);
    } else {
      int limit = road_limit.getRoadLimitSpeed();
      if (limit <= 0 && carrot_navi_speed_limit > 0 && carrot_navi_updated_at != 0 &&
          QDateTime::currentMSecsSinceEpoch() - (qint64)carrot_navi_updated_at <= 35000) {
        limit = carrot_navi_speed_limit;   // 내비 중 carrot 제한속도 폴백
      }
      disp_speed = (int)(limit * kph_to_disp + 0.5f);
      bool over = (car_state.getVEgoCluster() * 3.6f > limit + 2) && limit > 0;
      limit_color = over ? CT_RED_A(210) : CT_WHITE_A(210);
      // 흰 박스 위에서는 흰 글씨가 안보이므로 검정으로
      if (!over) limit_text_color = CT_BLACK_A(230);
      ctText(p, dx, dy - 45, "LIMIT", 30, CT_WHITE, true);
    }
    QRect limit_box(dx - 55, dy - 38, 110, 48);
    ctRect(p, limit_box, limit_color, 15, 2);
    ctTextIn(p, limit_box, QString::number(disp_speed), 40, limit_text_color);
  }

  // ---- CPU 온도 / 타이어 공기압 / CPU 사용률 (항상 표시) ----
  drawCarrotDeviceState(p);

  p.restore();
}

void NvgWindow::drawSpeedLimit(QPainter &p) {
  p.save();
	
  const SubMaster &sm = *(uiState()->sm);
  auto roadLimitSpeed = sm["roadLimitSpeed"].getRoadLimitSpeed();

  int camLimitSpeed = roadLimitSpeed.getCamLimitSpeed();
  int camLimitSpeedLeftDist = roadLimitSpeed.getCamLimitSpeedLeftDist();
  int camType = roadLimitSpeed.getCamType();

  int sectionLimitSpeed = roadLimitSpeed.getSectionLimitSpeed();
  int sectionLeftDist = roadLimitSpeed.getSectionLeftDist();

  int limit_speed = 0;
  int left_dist = 0;
  const bool bump_detected = camType == 22 && camLimitSpeedLeftDist > 0;

  if(bump_detected) {
    left_dist = camLimitSpeedLeftDist;
  }
  else if(camLimitSpeed > 0 && camLimitSpeedLeftDist > 0) {
    limit_speed = camLimitSpeed;
    left_dist = camLimitSpeedLeftDist;
  }
  else if(sectionLimitSpeed > 0 && sectionLeftDist > 0) {
    limit_speed = sectionLimitSpeed;
    left_dist = sectionLeftDist;
  }

  // NDA/HDA 아이콘 --- carrot hud panel 안의 NDA/HDA 텍스트로 대체되어 제거함

  if(bump_detected)
  {
    const int icon_size = 154;
    const int x = 30;
    const int y = 270;
    const int icon_width = 123;
    const QRect icon_rect(x + (icon_size - icon_width) / 2, y, icon_width, icon_size);

    if (!ic_speed_bump.isNull()) {
      p.drawPixmap(icon_rect, ic_speed_bump, ic_speed_bump.rect());
    } else {
      configFont(p, "Open Sans", 38, "Bold");
      p.setPen(QColor(255, 210, 0, 255));
      p.drawText(QRect(x, y, icon_size, icon_size), Qt::AlignCenter, "BUMP");
    }

    QString str_left_dist;
    if(left_dist >= 1000)
      str_left_dist.sprintf("%.1fkm", left_dist / 1000.f);
    else if(left_dist > 0)
      str_left_dist.sprintf("%dm", left_dist);

    if(!str_left_dist.isEmpty()) {
      configFont(p, "Open Sans", 48, "Bold");
      p.setPen(QColor(255, 255, 255, 230));
      p.drawText(QRect(x - 24, y + icon_size + 2, icon_size + 48, 60),
                 Qt::AlignCenter, str_left_dist);
    }
  }
  else if(limit_speed > 10 && limit_speed < 130)
  {
    int radius_ = 154;  // 기존 192에서 20% 축소 (숫자와 함께 원 전체도 축소)

    int x = 30;
    int y = 270;

    p.setPen(Qt::NoPen);
    p.setBrush(QBrush(QColor(255, 0, 0, 255)));
    QRect rect = QRect(x, y, radius_, radius_);
    p.drawEllipse(rect);

    p.setBrush(QBrush(QColor(255, 255, 255, 255)));

    const int tickness = 11;  // 기존 14에서 20% 축소
    rect.adjust(tickness, tickness, -tickness, -tickness);
    p.drawEllipse(rect);

    QString str_limit_speed, str_left_dist;
    str_limit_speed.sprintf("%d", limit_speed);

    if(left_dist >= 1000)
      str_left_dist.sprintf("%.1fkm", left_dist / 1000.f);
    else if(left_dist > 0)
      str_left_dist.sprintf("%dm", left_dist);

    configFont(p, "Open Sans", 64, "Bold");  // 기존 80에서 20% 축소
    p.setPen(QColor(0, 0, 0, 230));
    p.drawText(rect, Qt::AlignCenter, str_limit_speed);

    if(str_left_dist.length() > 0) {
      configFont(p, "Open Sans", 48, "Bold");  // 기존 60에서 20% 축소
      rect.translate(0, radius_/2 + 36);  // 기존 +45에서 20% 축소
      rect.adjust(-24, 0, 24, 0);  // 기존 ±30에서 20% 축소
      p.setPen(QColor(255, 255, 255, 230));
      p.drawText(rect, Qt::AlignCenter, str_left_dist);
    }
  }
  else {
    auto controls_state = sm["controlsState"].getControlsState();
    int sccStockCamAct = (int)controls_state.getSccStockCamAct();
    int sccStockCamStatus = (int)controls_state.getSccStockCamStatus();

    if(sccStockCamAct == 2 && sccStockCamStatus == 2) {
      int radius_ = 154;  // 기존 192에서 20% 축소

      int x = 30;
      int y = 270;

      p.setPen(Qt::NoPen);

      p.setBrush(QBrush(QColor(255, 0, 0, 255)));
      QRect rect = QRect(x, y, radius_, radius_);
      p.drawEllipse(rect);

      p.setBrush(QBrush(QColor(255, 255, 255, 255)));

      const int tickness = 11;  // 기존 14에서 20% 축소
      rect.adjust(tickness, tickness, -tickness, -tickness);
      p.drawEllipse(rect);

      configFont(p, "Open Sans", 56, "Bold");  // 기존 70에서 20% 축소 (원 크기와 비례 유지)
      p.setPen(QColor(0, 0, 0, 230));
      p.drawText(rect, Qt::AlignCenter, "CAM");
    }
  }

  p.restore();
}

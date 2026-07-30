#include "selfdrive/ui/qt/onroad.h"

#include <cmath>

#include <QDebug>
#include <QSound>
#include <QMouseEvent>
#include <QDateTime>
#include <QPainterPath>

#include "selfdrive/common/timing.h"
#include "selfdrive/ui/qt/util.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#endif

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  const int vertical_border = 30;
  main_layout->setContentsMargins(bdr_s, vertical_border, bdr_s, vertical_border);
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
	QObject::connect(record_timer.get(), &QTimer::timeout, [=]() {
    if(recorder) {
      recorder->update_screen();
    }
  });
	record_timer->start(1000/UI_FREQ);

  QWidget* recorder_widget = new QWidget(this);
  QVBoxLayout * recorder_layout = new QVBoxLayout (recorder_widget);
  recorder_layout->setMargin(35);
  recorder = new ScreenRecoder(this);
  recorder_layout->addWidget(recorder);
  recorder_layout->setAlignment(recorder, Qt::AlignRight | Qt::AlignBottom);

  stacked_layout->addWidget(recorder_widget);
  recorder_widget->raise();
  alerts->raise();

}

void OnroadWindow::updateState(const UIState &s) {
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
    update();
  }
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

  // ── ChevronInfo 탭 토글 ──────────────────────────────
  {
    int tap_x = endPos.x();
    int tap_y = endPos.y();
    int center_x = width() / 2;
    int bottom_y = height() * 2 / 3;

    if (std::abs(tap_x - center_x) < 200 && tap_y > bottom_y) {
      int cur = std::atoi(Params().get("ChevronInfo").c_str());
      int next = (cur + 1) % 5;  // 0(Off)→1(Dist)→2(Spd)→3(TTC)→4(All)→0
      Params().put("ChevronInfo", std::to_string(next));
      return;
    }
  }

  if (map != nullptr) {
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
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
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

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  p.setBrush(QBrush(bg));
  p.drawRect(r);

  QLinearGradient g(0, r.y(), 0, r.bottom());
  g.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.05));
  g.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0.35));

  p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
  p.setBrush(QBrush(g));
  p.fillRect(r, g);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setRenderHint(QPainter::TextAntialiasing);
  if (alert.size == cereal::ControlsState::AlertSize::SMALL) {
    configFont(p, "Open Sans", 74, "SemiBold");
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    configFont(p, "Open Sans", 88, "Bold");
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    configFont(p, "Open Sans", 66, "Regular");
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    configFont(p, "Open Sans", l ? 132 : 177, "Bold");
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    configFont(p, "Open Sans", 88, "Regular");
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}

// NvgWindow

NvgWindow::NvgWindow(VisionStreamType type, QWidget* parent) : last_update_params(0), fps_filter(UI_FREQ, 3, 1. / UI_FREQ), accel_filter(UI_FREQ, .5, 1. / UI_FREQ), CameraViewWidget("camerad", type, true, parent) {
  change_popup_enabled = uiState()->animatedValuePopupEnabled();
  QObject::connect(uiState(), &UIState::animatedValuePopupChanged, this, [this](bool enabled) {
    change_popup_enabled = enabled;
    change_popup_initialized = false;
    camera_sign_initialized = false;
    if (!change_popup_enabled) change_popup_frames = 0;
    if (!change_popup_enabled) camera_sign_popup_frames = 0;
  });
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
  ic_brake = QPixmap("../assets/images/img_brake_disc.png").scaled(img_size, img_size, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
  ic_autohold_warning = QPixmap("../assets/images/img_autohold_warning.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  ic_autohold_active = QPixmap("../assets/images/img_autohold_active.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  ic_nda = QPixmap("../assets/images/img_nda.png");
  ic_hda = QPixmap("../assets/images/img_hda.png");
  ic_tire_pressure = QPixmap("../assets/images/img_tire_pressure.png");
  ic_turn_signal_l = QPixmap("../assets/images/turn_signal_l.png");
  ic_turn_signal_r = QPixmap("../assets/images/turn_signal_r.png");
  ic_satellite = QPixmap("../assets/images/satellite.png");
  ic_speed_bg = QPixmap("../assets/images/speed_bg.png");
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

  // blind spot state sync
  auto car_state = sm["carState"].getCarState();
  //auto controls_state = sm["controlsState"].getControlsState();
  setProperty("left_blindspot",  car_state.getLeftBlindspot());
  setProperty("right_blindspot", car_state.getRightBlindspot());

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

  //Blind Spot Warnings
  painter.setPen(Qt::NoPen);

  // 왼쪽 barrier: 감지=빨강(alpha 0.45), 평상시=흰색(alpha 0.10)
  painter.setBrush(left_blindspot
      ? QColor::fromRgbF(1.0, 0.0, 0.0, 0.45)
      : QColor::fromRgbF(1.0, 1.0, 1.0, 0.10));
  painter.drawPolygon(scene.lane_barrier_vertices[0].v, scene.lane_barrier_vertices[0].cnt);

  // 오른쪽 barrier: 감지=빨강(alpha 0.45), 평상시=흰색(alpha 0.10)
  painter.setBrush(right_blindspot
      ? QColor::fromRgbF(1.0, 0.0, 0.0, 0.45)
      : QColor::fromRgbF(1.0, 1.0, 1.0, 0.10));
  painter.drawPolygon(scene.lane_barrier_vertices[1].v, scene.lane_barrier_vertices[1].cnt);
	
  // paint path
  QLinearGradient bg(0, height(), 0, height() / 4);
  float start_hue, end_hue;
  if (sm["controlsState"].getControlsState().getExperimentalMode()) {
    const auto &acceleration = sm["modelV2"].getModelV2().getAcceleration();
    float acceleration_future = 0;
    if (acceleration.getZ().size() > 16) {
      acceleration_future = acceleration.getX()[16];  // 2.5 seconds
    }
    if (scene.dynamic_lane_profile_status) {
      start_hue = 60;
      // speed up: 120, slow down: 0
      end_hue = fmax(fmin(start_hue + acceleration_future * 45, 148), 0);
    } else {
      start_hue = 240;
      // speed up: 300, slow down: 180
      end_hue = fmin(fmax(start_hue + acceleration_future * 45, 180), 328);
    }
    // FIXME: painter.drawPolygon can be slow if hue is not rounded
    end_hue = int(end_hue * 100 + 0.5) / 100;

    bg.setColorAt(0.0, QColor::fromHslF(start_hue / 360., 0.97, 0.56, 0.4));
    bg.setColorAt(0.5, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.0));
  } else if (scene.dynamic_lane_profile_status) {
    bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
    bg.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.0));
  } else {
    bg.setColorAt(0.0, QColor::fromHslF(197 / 360., 1.0, 0.55, 0.7));
    bg.setColorAt(0.5, QColor::fromHslF(200 / 360., 1.0, 0.70, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(200 / 360., 1.0, 0.70, 0.0));
  }
  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices.v, scene.track_vertices.cnt);

  painter.restore();
}

void NvgWindow::drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd, bool is_radar) {
  painter.save();
  const float speedBuff = 10.;
  const float leadBuff = 40.;
  const float d_rel = lead_data.getX()[0];
  const float v_rel = lead_data.getV()[0];

  float fillAlpha = 0;
  if (d_rel < leadBuff) {
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float y = std::fmin(height() - sz * .6, (float)vd.y());

  float g_xo = sz / 5;
  float g_yo = sz / 10;

  QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};
  painter.setBrush(is_radar ? QColor(86, 121, 216, 255) : QColor(218, 202, 37, 255));
  painter.drawPolygon(glow, std::size(glow));

  // chevron
  QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
  painter.setBrush(redColor(fillAlpha));
  painter.drawPolygon(chevron, std::size(chevron));

  painter.restore();
}

void NvgWindow::drawLeadStatus(QPainter &p) {
  UIState *s = uiState();
  auto &sm = *(s->sm);

  if (!sm.updated("radarState")) return;

  const auto &radar_state = sm["radarState"].getRadarState();
  const auto &lead_one    = radar_state.getLeadOne();
  const auto &lead_two    = radar_state.getLeadTwo();

  bool has_lead_one = lead_one.getStatus();
  bool has_lead_two = lead_two.getStatus();

  // 리드 없으면 서서히 페이드아웃
  if (!has_lead_one && !has_lead_two) {
    lead_status_alpha = std::max(0.0f, lead_status_alpha - 0.05f);
    if (lead_status_alpha <= 0.0f) return;
  } else {
    lead_status_alpha = std::min(1.0f, lead_status_alpha + 0.1f);
  }

  // L1: 항상 시도 (has_lead_one 여부 무관하게 위치 유지)
  if (true) {
    drawLeadStatusAtPosition(p, lead_one, s->scene.lead_vertices[0], "L1");
  }

  // L2: 두 리드 거리 차이 3m 이상일 때만
  if (has_lead_two &&
      std::abs(lead_one.getDRel() - lead_two.getDRel()) > 3.0) {
    drawLeadStatusAtPosition(p, lead_two, s->scene.lead_vertices[1], "L2");
  }
}

void NvgWindow::drawLeadStatusAtPosition(QPainter &p,
                                         const cereal::RadarState::LeadData::Reader &lead_data,
                                         const QPointF &chevron_pos,
                                         const QString &label) {
  UIState *s = uiState();
  auto &sm   = *(s->sm);

  float d_rel = lead_data.getDRel();
  float v_rel = lead_data.getVRel();
  float v_ego = sm["carState"].getCarState().getVEgo();
  bool is_metric = s->scene.is_metric;

  // 파라미터에서 표시 모드 읽기 (0=Off, 1=Distance, 2=Speed, 3=Time, 4=All)
  int chevron_data = std::atoi(Params().get("ChevronInfo").c_str());
  if (chevron_data == 0) return;

  // 쉐브론 크기 (drawLead 와 동일 로직)
  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;

  QFont content_font = p.font();
  content_font.setPixelSize(45);
  content_font.setBold(true);
  p.setFont(content_font);

  const int chevron_types = 3;
  const int chevron_all   = chevron_types + 1; // value 4 = All

  QStringList chevron_text[chevron_types];

  // 1) Distance
  if (chevron_data == 1 || chevron_data == chevron_all) {
    float val = std::max(0.0f, d_rel);
    QString unit = is_metric ? "m" : "ft";
    if (!is_metric) val *= 3.28084f;
    chevron_text[0].append(QString::number(val, 'f', 0) + " " + unit);
  }

  // 2) Absolute speed
  if (chevron_data == 2 || chevron_data == chevron_all) {
    int pos = (chevron_data == 2) ? 0 : 1;
    float val = std::max(0.0f,
        (v_rel + v_ego) * (is_metric ? static_cast<float>(MS_TO_KPH)
                                     : static_cast<float>(MS_TO_MPH)));
    chevron_text[pos].append(QString::number(val, 'f', 0) + " " +
                             (is_metric ? "km/h" : "mph"));
  }

  // 3) Time-to-contact
  if (chevron_data == 3 || chevron_data == chevron_all) {
    int pos = (chevron_data == 3) ? 0 : 2;
    float val = (d_rel > 0 && v_ego > 0)
                    ? std::max(0.0f, d_rel / v_ego)
                    : 0.0f;
    QString ttc_str = (val > 0 && val < 200)
                          ? QString::number(val, 'f', 1) + "s"
                          : "---";
    chevron_text[pos].append(ttc_str);
  }

  // 빈 항목 제거하여 최종 라인 목록 생성
  QStringList text_lines;
  for (int i = 0; i < chevron_types; ++i) {
    if (!chevron_text[i].isEmpty())
      text_lines.append(chevron_text[i]);
  }
  if (text_lines.isEmpty()) return;

  // 텍스트 박스 위치 (쉐브론 아래 중앙)
  const float str_w = 190.0f;
  const float str_h = 58.0f;
  float text_x = chevron_pos.x() - str_w / 2;
  float text_y = chevron_pos.y() + sz + 15;

  // 화면 경계 클램핑
  text_x = std::clamp(text_x, 10.0f, (float)width() - str_w - 10);

  QPoint shadow_offset(2, 2);

  for (int i = 0; i < text_lines.size(); ++i) {
    const QString &line = text_lines[i];
    if (line.isEmpty()) continue;

    QRect textRect((int)text_x,
                   (int)(text_y + i * str_h),
                   (int)str_w,
                   (int)str_h);

    // 그림자
    p.setPen(QColor(0, 0, 0, (int)(200 * lead_status_alpha)));
    p.drawText(textRect.translated(shadow_offset.x(), shadow_offset.y()),
               Qt::AlignBottom | Qt::AlignHCenter, line);

    // 본문 색상 결정
    QColor text_color = QColor(255, 255, 255, (int)(255 * lead_status_alpha));

    p.setPen(text_color);
    p.drawText(textRect, Qt::AlignBottom | Qt::AlignHCenter, line);
  }

  p.setPen(Qt::NoPen);
}

void NvgWindow::paintGL() {
}

void NvgWindow::paintEvent(QPaintEvent *event) {


  UIState *s = uiState();
  const cereal::ModelDataV2::Reader &model = (*s->sm)["modelV2"].getModelV2();

  QPainter p(this);

  p.beginNativePainting();
  CameraViewWidget::paintGL();
  p.endNativePainting();

  if (s->worldObjectsVisible())
    drawHud(p, model);

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  double fps = fps_filter.update(1. / dt * 1000);
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
  }
  prev_draw_t = cur_draw_t;
}

void NvgWindow::showEvent(QShowEvent *event) {
  CameraViewWidget::showEvent(event);

  change_popup_initialized = false;
  change_popup_frames = 0;
  camera_sign_initialized = false;
  camera_sign_missing_frames = 0;
  camera_sign_popup_frames = 0;

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

  drawLaneLines(p, s);

  auto leads =  model.getLeadsV3();
  if (leads[0].getProb() > .5) {
    drawLead(p, leads[0], s->scene.lead_vertices[0], s->scene.lead_radar[0]);
  }
  if (leads[1].getProb() > .5 && (std::abs(leads[1].getX()[0] - leads[0].getX()[0]) > 3.0)) {
    drawLead(p, leads[1], s->scene.lead_vertices[1], s->scene.lead_radar[1]);
  }

  drawLeadStatus(p);
  drawCarrotUi(p);
}

void NvgWindow::drawCarrotUi(QPainter &p) {
  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  p.setRenderHint(QPainter::TextAntialiasing);
  p.setRenderHint(QPainter::SmoothPixmapTransform);

  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  const auto car_state = sm["carState"].getCarState();
  const auto controls_state = sm["controlsState"].getControlsState();
  const auto car_params = sm["carParams"].getCarParams();
  const auto live_params = sm["liveParameters"].getLiveParameters();
  const auto device_state = sm["deviceState"].getDeviceState();
  const auto road_limit = sm["roadLimitSpeed"].getRoadLimitSpeed();
  const auto longitudinal_plan = sm["longitudinalPlan"].getLongitudinalPlan();

  // The c3 UI was designed at 1440x720. Scale all HUD geometry as one unit so
  // it keeps the same composition on EON/C2 and C3 aspect ratios.
  const float sx = width() / 1440.0f;
  const float sy = height() / 720.0f;
  p.scale(sx, sy);

  auto font = [&](int px, bool bold = true) {
    configFont(p, "Open Sans", px, bold ? "Bold" : "Regular");
    QFont hinted_font = p.font();
    hinted_font.setHintingPreference(QFont::PreferFullHinting);
    hinted_font.setStyleStrategy(QFont::PreferAntialias);
    p.setFont(hinted_font);
  };
  auto outlined_text = [&](const QRectF &r, int flags, const QString &text, int px,
                           const QColor &color = Qt::white) {
    // Render text directly in device pixels. The HUD geometry is stretched
    // from the C3's 2:1 canvas to the EON's 16:9 screen; rasterizing glyphs
    // after that non-uniform transform makes small text look blurred.
    const QTransform hud_transform = p.worldTransform();
    const QRectF device_rect = hud_transform.mapRect(r);
    const qreal text_scale = std::min(std::abs(hud_transform.m11()),
                                      std::abs(hud_transform.m22()));

    p.save();
    p.resetTransform();
    const int device_px = std::max(1, qRound(px * text_scale));
    font(device_px);
    const int draw_flags = flags | Qt::TextDontClip;
    const qreal shadow_offset = device_px >= 40 ? 2.0 : 1.0;
    p.setPen(QColor(0, 0, 0, 210));
    p.drawText(device_rect.translated(shadow_offset, shadow_offset),
               draw_flags, text);
    p.setPen(color);
    p.drawText(device_rect, draw_flags, text);
    p.restore();
  };
  auto panel = [&](const QRectF &r, int corner_radius = 15, int background_alpha = 165) {
    p.setPen(QPen(QColor(255, 255, 255, 210), 3));
    p.setBrush(QColor(0, 0, 0, background_alpha));
    p.drawRoundedRect(r, corner_radius, corner_radius);
  };
  auto speed_limit_sign = [&](const QPointF &center, qreal sign_radius, int speed, int text_size) {
    const QRectF sign_rect(center.x() - sign_radius, center.y() - sign_radius,
                           sign_radius * 2.0, sign_radius * 2.0);
    p.setPen(QPen(QColor(220, 35, 35), std::max(6.0, sign_radius * 0.16)));
    p.setBrush(Qt::white);
    p.drawEllipse(sign_rect);
    outlined_text(sign_rect.adjusted(sign_radius * 0.13, sign_radius * 0.13,
                                     -sign_radius * 0.13, -sign_radius * 0.13),
                  Qt::AlignHCenter | Qt::AlignVCenter,
                  QString::number(speed), text_size, Qt::black);
  };

  QString fingerprint = QString::fromUtf8(car_params.getCarFingerprint().cStr()).toUpper();
  if (fingerprint.isEmpty()) fingerprint = "OPENPILOT";
  outlined_text(QRectF(20, 1, 700, 32), Qt::AlignLeft | Qt::AlignVCenter, fingerprint, 22);
  const bool scc2_detected = car_params.getOpenpilotLongitudinalControl() || car_params.getSccBus() == 2;
  if (scc2_detected) {
    font(22);
    const int fingerprint_width = QFontMetrics(p.font()).horizontalAdvance(fingerprint);
    outlined_text(QRectF(28 + fingerprint_width, 1, 110, 32),
                  Qt::AlignLeft | Qt::AlignVCenter, "SCC2", 22, QColor(255, 55, 55));
  }

  const auto torque_state = controls_state.getLateralControlState().getTorqueState();
  QString tune;
  tune.sprintf("%s  LT(%.2f/%.2f), SR(%.1f)",
               s->lat_control.c_str(), torque_state.getLatAccelFactor(),
               torque_state.getFriction(), controls_state.getSteerRatio());
  outlined_text(QRectF(810, 1, 390, 32), Qt::AlignRight | Qt::AlignVCenter, tune, 20);

  // Compact TPMS row in the otherwise unused upper-right corner.
  const auto tpms = car_state.getTpms();
  const float tire_pressures[] = {tpms.getFl(), tpms.getFr(), tpms.getRl(), tpms.getRr()};
  const QString tire_labels[] = {"FL", "FR", "RL", "RR"};
  for (int i = 0; i < 4; ++i) {
    const bool valid = tire_pressures[i] >= 5.0f && tire_pressures[i] <= 60.0f;
    const QColor value_color = valid && tire_pressures[i] < 31.0f
                                   ? QColor(255, 85, 85)
                                   : QColor(255, 255, 255);
    const QString value = valid ? QString::number(qRound(tire_pressures[i])) : "--";
    const int column = i % 2;
    const int row = i / 2;
    const qreal x = 1210 + column * 101;
    const qreal y = row * 16;
    outlined_text(QRectF(x, y, 30, 16), Qt::AlignRight | Qt::AlignVCenter,
                  tire_labels[i], 11, QColor(185, 185, 185));
    outlined_text(QRectF(x + 33, y, 48, 16), Qt::AlignLeft | Qt::AlignVCenter,
                  value, 15, value_color);
  }

  // Time/date block.
  QDateTime now = QDateTime::currentDateTime();
  outlined_text(QRectF(38, 34, 220, 76), Qt::AlignLeft | Qt::AlignVCenter,
                now.toString("hh:mm"), 76);
  outlined_text(QRectF(42, 105, 220, 50), Qt::AlignLeft | Qt::AlignVCenter,
                now.toString("MM-dd(ddd)"), 42);

  // Left c3 instrument panel.
  const QRectF instrument(32, 366, 320, 330);
  panel(instrument, 20, 82);

  float cpu_temp = 0.0f;
  const auto cpu_temps = device_state.getCpuTempC();
  for (float temp : cpu_temps) cpu_temp += temp;
  if (cpu_temps.size() > 0) cpu_temp /= cpu_temps.size();
  int cpu_usage = 0;
  const auto cpu_usage_list = device_state.getCpuUsagePercent();
  for (int usage : cpu_usage_list) cpu_usage += usage;
  if (cpu_usage_list.size() > 0) cpu_usage /= cpu_usage_list.size();
  int memory_usage = device_state.getMemoryUsagePercent();
  int disk_usage = std::clamp((int)std::round(100.0f - device_state.getFreeSpacePercent()), 0, 100);

  struct DeviceItem { QString label; QString value; };
  DeviceItem items[] = {
    {"CPU", QString::number((int)std::round(cpu_temp)) + "°C"},
    {"MEM", QString::number(memory_usage) + "%"},
    {"DISK", QString::number(disk_usage) + "%"},
  };
  for (int i = 0; i < 3; ++i) {
    QRectF r(47 + i * 99, 387, 87, 62);
    p.setPen(QPen(QColor(0, 0, 0), 2));
    p.setBrush(QColor(0, 125, 20, 225));
    p.drawRoundedRect(r, 9, 9);
    outlined_text(QRectF(r.left(), r.top() + 2, r.width(), 25), Qt::AlignHCenter | Qt::AlignVCenter,
                  items[i].label, 18);
    outlined_text(QRectF(r.left(), r.top() + 24, r.width(), 36), Qt::AlignHCenter | Qt::AlignVCenter,
                  items[i].value, 29);
  }

  const float speed_factor = s->scene.is_metric ? MS_TO_KPH : MS_TO_MPH;
  int speed = std::max(0, (int)std::round(car_state.getVEgoCluster() * speed_factor));
  int set_speed = (int)std::round(controls_state.getVCruiseCluster());

  // Keep the speed artwork inside the instrument panel and preserve its
  // original aspect ratio. Drawing it before the values also keeps the digits
  // crisp when the artwork contains translucent pixels.
  const QRect speed_art_bounds(45, 462, 302, 113);
  p.drawPixmap(speed_art_bounds, ic_speed_bg);
  outlined_text(QRectF(47, 488, 145, 110), Qt::AlignHCenter | Qt::AlignVCenter,
                QString::number(speed), 92);
  outlined_text(QRectF(175, 489, 92, 58), Qt::AlignHCenter | Qt::AlignVCenter,
                QString::number(set_speed > 0 && set_speed < 255 ? set_speed : 0), 50,
                QColor(0, 230, 30));

  QString gear_text = "U";
  const auto gear_shifter = car_state.getGearShifter();
  if (gear_shifter == cereal::CarState::GearShifter::PARK) {
    gear_text = "P";
  } else if (gear_shifter == cereal::CarState::GearShifter::DRIVE) {
    const int gear_step = car_state.getGearStep();
    gear_text = gear_step > 0 ? QString::number(gear_step) : "D";
  } else if (gear_shifter == cereal::CarState::GearShifter::NEUTRAL) {
    gear_text = "N";
  } else if (gear_shifter == cereal::CarState::GearShifter::REVERSE) {
    gear_text = "R";
  } else if (gear_shifter == cereal::CarState::GearShifter::SPORT) {
    gear_text = "S";
  } else if (gear_shifter == cereal::CarState::GearShifter::LOW) {
    gear_text = "L";
  } else if (gear_shifter == cereal::CarState::GearShifter::BRAKE) {
    gear_text = "B";
  } else if (gear_shifter == cereal::CarState::GearShifter::ECO) {
    gear_text = "E";
  } else if (gear_shifter == cereal::CarState::GearShifter::MANUMATIC) {
    gear_text = "M";
  }
  p.setPen(QPen(QColor(255, 255, 255, 220), 2));
  p.setBrush(QColor(0, 125, 20, 225));
  p.drawRoundedRect(QRectF(278, 525, 55, 64), 9, 9);
  outlined_text(QRectF(278, 525, 55, 64), Qt::AlignHCenter | Qt::AlignVCenter,
                gear_text, 43);

  int road_speed = road_limit.getRoadLimitSpeed();
  outlined_text(QRectF(45, 609, 76, 30), Qt::AlignHCenter | Qt::AlignVCenter, "LIMIT", 19);
  outlined_text(QRectF(128, 609, 72, 30), Qt::AlignHCenter | Qt::AlignVCenter, "ROUTE", 19);
  outlined_text(QRectF(207, 609, 72, 30), Qt::AlignHCenter | Qt::AlignVCenter, "APILOT", 16);
  panel(QRectF(45, 642, 76, 39), 9);
  panel(QRectF(128, 642, 72, 39), 9);
  panel(QRectF(207, 642, 72, 39), 9);
  outlined_text(QRectF(45, 642, 76, 39), Qt::AlignHCenter | Qt::AlignVCenter,
                road_speed > 0 ? QString::number(road_speed) : "--", 28);
  // A positive maneuver distance is the authoritative navigation signal.
  // Generic/stale TMAP notification text must not open the route panel.
  const bool tmap_tbt_active = road_limit.getTbtDist() > 0;
  const QString route_status = tmap_tbt_active ? "APN" :
                               road_limit.getActive() ? "NDA" : "OFF";
  outlined_text(QRectF(128, 642, 72, 39), Qt::AlignHCenter | Qt::AlignVCenter,
                route_status, 24,
                road_limit.getActive() ? QColor(80, 255, 80) : QColor(210, 210, 210));
  const bool is_e2e = longitudinal_plan.getMpcMode() == 1;
  outlined_text(QRectF(207, 642, 72, 39), Qt::AlignHCenter | Qt::AlignVCenter,
                is_e2e ? "E2E" : "ACC", 24,
                is_e2e ? QColor(0, 230, 130) : QColor(220, 220, 220));

  int cruise_gap = (int)controls_state.getLongCruiseGap();
  if (cruise_gap < 1 || cruise_gap > 4) {
    cruise_gap = (int)car_state.getCruiseGap();
  }
  cruise_gap = std::clamp(cruise_gap, 1, 4);
  p.setPen(QPen(QColor(255, 255, 255, 210), 1));
  p.setBrush(QColor(0, 180, 45, 225));
  for (int i = 0; i < cruise_gap; ++i) {
    p.drawRoundedRect(QRectF(290, 672 - i * 9, 48, 7), 2, 2);
  }

  // c3-style value-change popup, simplified to one animated text draw so the
  // EON only pays the rendering cost for 12 frames after an actual change.
  if (!change_popup_enabled) {
    change_popup_frames = 0;
  } else if (!change_popup_initialized) {
    last_popup_gear = gear_text;
    last_popup_gap = cruise_gap;
    change_popup_initialized = true;
  } else {
    bool popup_triggered = false;
    if (gear_text != last_popup_gear) {
      change_popup_text = gear_text;
      change_popup_target = QPointF(305.5, 557);
      change_popup_color = QColor(255, 255, 255);
      change_popup_target_size = 43;
      popup_triggered = true;
    } else if (cruise_gap != last_popup_gap) {
      change_popup_text = QString::number(cruise_gap);
      change_popup_target = QPointF(314, 654);
      change_popup_color = QColor(80, 255, 80);
      change_popup_target_size = 40;
      popup_triggered = true;
    }
    if (popup_triggered) change_popup_frames = change_popup_total_frames;
    last_popup_gear = gear_text;
    last_popup_gap = cruise_gap;
  }

  // APilot/TMAP lightweight navigation.  Drawing a schematic route instead of
  // running a web map keeps GPU/RAM use low enough for EON while retaining the
  // live road name, maneuver, remaining distance and ETA.
  const int tbt_dist = road_limit.getTbtDist();
  const int turn_type = road_limit.getTbtTurnType();
  const QString tbt_text = QString::fromUtf8(road_limit.getTbtMainText().cStr());
  const QString road_name = QString::fromUtf8(road_limit.getPosRoadName().cStr());
  const QString goal_name = QString::fromUtf8(road_limit.getGoalName().cStr());
  const int goal_dist = road_limit.getGoPosDist();
  const int goal_time = road_limit.getGoPosTime();
  const bool navigation_active = road_limit.getActive() && tmap_tbt_active;
  int limit_speed = road_limit.getCamLimitSpeed();
  int left_dist = road_limit.getCamLimitSpeedLeftDist();
  bool is_section = false;
  if (limit_speed <= 0 || left_dist <= 0) {
    limit_speed = road_limit.getSectionLimitSpeed();
    left_dist = road_limit.getSectionLeftDist();
    is_section = true;
  }
  const bool camera_sign_visible = limit_speed > 0 && left_dist > 0;
  if (!change_popup_enabled) {
    camera_sign_popup_frames = 0;
  } else if (!camera_sign_initialized) {
    camera_sign_was_visible = camera_sign_visible;
    camera_sign_missing_frames = 0;
    camera_sign_initialized = true;
  } else if (camera_sign_visible) {
    if (!camera_sign_was_visible) {
      camera_sign_popup_frames = camera_sign_popup_total_frames;
    }
    camera_sign_was_visible = true;
    camera_sign_missing_frames = 0;
  } else if (++camera_sign_missing_frames > 10) {
    // Ignore brief receiver dropouts so the same camera does not pop repeatedly.
    camera_sign_was_visible = false;
  }

  qreal camera_sign_scale = 1.0;
  if (camera_sign_popup_frames > 0) {
    const qreal elapsed = camera_sign_popup_total_frames - camera_sign_popup_frames;
    const qreal progress = elapsed / std::max(1, camera_sign_popup_total_frames - 1);
    const qreal eased = 1.0 - std::pow(1.0 - progress, 3.0);
    camera_sign_scale = 0.68 + 0.32 * eased;
  }
  const QString camera_distance = left_dist >= 1000
                                      ? QString::number(left_dist / 1000.0, 'f', 1) + "km"
                                      : QString::number(left_dist) + "m";

  if (navigation_active) {
    const QRectF nav_map(885, 28, 525, 455);
    panel(nav_map, 22, 82);

    // Dark, perspective-like road surface with a single inexpensive route
    // path. This is intentionally vector-only: no Mapbox, tiles or textures.
    QLinearGradient nav_bg(nav_map.topLeft(), nav_map.bottomLeft());
    nav_bg.setColorAt(0, QColor(22, 29, 35, 112));
    nav_bg.setColorAt(1, QColor(5, 8, 11, 118));
    p.setPen(Qt::NoPen);
    p.setBrush(nav_bg);
    p.drawRoundedRect(nav_map.adjusted(3, 3, -3, -3), 19, 19);

    p.save();
    p.setClipRect(nav_map.adjusted(4, 4, -4, -4));
    p.setPen(QPen(QColor(80, 90, 96, 150), 3));
    for (int y = 105; y < 480; y += 72) p.drawLine(900, y, 1395, y - 25);

    QPainterPath route;
    route.moveTo(1148, 474);
    route.cubicTo(1148, 390, 1160, 330, 1142, 270);
    const bool left_turn = turn_type == 11 || turn_type == 16 || turn_type == 17;
    const bool right_turn = turn_type == 12 || turn_type == 18;
    if (left_turn) {
      route.cubicTo(1125, 215, 1065, 205, 980, 205);
    } else if (right_turn) {
      route.cubicTo(1160, 215, 1225, 205, 1340, 205);
    } else {
      route.cubicTo(1138, 210, 1150, 140, 1150, 75);
    }
    p.setPen(QPen(QColor(0, 190, 105), 19, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPath(route);
    p.setPen(QPen(QColor(145, 255, 205), 5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPath(route);

    QPainterPath vehicle_arrow;
    vehicle_arrow.moveTo(1148, 390);
    vehicle_arrow.lineTo(1128, 428);
    vehicle_arrow.lineTo(1148, 418);
    vehicle_arrow.lineTo(1168, 428);
    vehicle_arrow.closeSubpath();
    p.setPen(QPen(Qt::white, 3));
    p.setBrush(QColor(220, 230, 235));
    p.drawPath(vehicle_arrow);
    p.restore();

    if (!road_name.isEmpty()) {
      outlined_text(QRectF(905, 42, 485, 42), Qt::AlignHCenter | Qt::AlignVCenter,
                    road_name, 31);
    }

    // Keep camera/section distance visible while APilot TBT navigation is active.
    if (limit_speed > 0 && left_dist > 0) {
      const QRectF camera_cue(1190, 95, 200, 96);
      panel(camera_cue, 14);
      speed_limit_sign(QPointF(1237, 143), 35 * camera_sign_scale, limit_speed,
                       qRound(39 * camera_sign_scale));
      outlined_text(QRectF(1273, 101, 108, 36), Qt::AlignHCenter | Qt::AlignVCenter,
                    is_section ? "SECTION" : "CAMERA", 22);
      outlined_text(QRectF(1273, 137, 108, 42), Qt::AlignHCenter | Qt::AlignVCenter,
                    camera_distance, 32);
    }

    const QRectF nav_card(885, 490, 525, 198);
    panel(nav_card, 20, 82);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 165, 40, 235));
    p.drawRoundedRect(QRectF(900, 512, 108, 156), 12, 12);

    // Maneuver arrow (T map codes: 11 left, 12 right, 13 U-turn).
    QPainterPath turn_arrow;
    if (turn_type == 11 || turn_type == 16 || turn_type == 17) {
      turn_arrow.moveTo(980, 578); turn_arrow.lineTo(940, 578);
      turn_arrow.lineTo(940, 548); turn_arrow.lineTo(914, 580);
      turn_arrow.lineTo(940, 612); turn_arrow.lineTo(940, 592);
      turn_arrow.lineTo(980, 592);
    } else if (turn_type == 12 || turn_type == 18) {
      turn_arrow.moveTo(925, 578); turn_arrow.lineTo(965, 578);
      turn_arrow.lineTo(965, 548); turn_arrow.lineTo(992, 580);
      turn_arrow.lineTo(965, 612); turn_arrow.lineTo(965, 592);
      turn_arrow.lineTo(925, 592);
    } else if (turn_type == 13) {
      turn_arrow.moveTo(975, 610); turn_arrow.cubicTo(980, 550, 920, 548, 925, 595);
      turn_arrow.moveTo(908, 576); turn_arrow.lineTo(925, 600); turn_arrow.lineTo(945, 579);
    } else {
      turn_arrow.moveTo(954, 620); turn_arrow.lineTo(954, 552);
      turn_arrow.moveTo(930, 575); turn_arrow.lineTo(954, 548); turn_arrow.lineTo(978, 575);
    }
    p.setPen(QPen(Qt::white, 13, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.setBrush(Qt::NoBrush);
    p.drawPath(turn_arrow);

    QString tbt_distance = tbt_dist >= 1000 ? QString::number(tbt_dist / 1000.0, 'f', 1) + " km"
                                             : QString::number(tbt_dist) + " m";
    outlined_text(QRectF(905, 625, 98, 35), Qt::AlignHCenter | Qt::AlignVCenter,
                  tbt_distance, 25);
    outlined_text(QRectF(1025, 510, 360, 60), Qt::AlignLeft | Qt::AlignVCenter,
                  !tbt_text.isEmpty() ? tbt_text : road_name, 35);

    QString remain;
    if (goal_dist > 0) {
      remain = goal_dist >= 1000 ? QString::number(goal_dist / 1000.0, 'f', 1) + " km"
                                 : QString::number(goal_dist) + " m";
    }
    if (goal_time > 0) {
      const int minutes = std::max(1, goal_time / 60);
      remain = QString("도착 %1분  ").arg(minutes) + remain;
    }
    outlined_text(QRectF(1025, 570, 360, 50), Qt::AlignLeft | Qt::AlignVCenter,
                  remain, 30);
    outlined_text(QRectF(1025, 625, 360, 40), Qt::AlignLeft | Qt::AlignVCenter,
                  !goal_name.isEmpty() ? goal_name : road_name, 27, QColor(220, 220, 220));
  }

  // Larger camera/section notification is used when APilot has no active TBT route.
  if (!navigation_active && limit_speed > 0 && left_dist > 0) {
    QRectF cue(1000, 500, 380, 165);
    panel(cue, 18);
    speed_limit_sign(QPointF(1080, 560), 51 * camera_sign_scale, limit_speed,
                     qRound(57 * camera_sign_scale));
    outlined_text(QRectF(1150, 520, 205, 55), Qt::AlignLeft | Qt::AlignVCenter,
                  is_section ? "SECTION" : "CAMERA", 31);
    outlined_text(QRectF(1150, 575, 205, 55), Qt::AlignLeft | Qt::AlignVCenter,
                  camera_distance, 47);
  }
  if (camera_sign_popup_frames > 0) --camera_sign_popup_frames;

  // c3 diagnostic footer.
  QString footer;
  footer.sprintf("lanemode  %.1fm | %.1fm | offset=%+.1fcm  turn→%dkm/h",
                 s->scene.lane_line_probs[1] * 3.0f,
                 s->scene.lane_line_probs[2] * 3.0f,
                 live_params.getAngleOffsetDeg() * 10.0f,
                 road_speed);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 165));
  p.drawRect(QRectF(355, 696, 805, 23));
  outlined_text(QRectF(360, 696, 795, 23), Qt::AlignHCenter | Qt::AlignVCenter,
                footer, 19);

  QString wifi_address = QString::fromUtf8(device_state.getWifiIpAddress().cStr()).trimmed();
  if (wifi_address.isEmpty() || wifi_address == "N/A") {
    wifi_address = "--";
  }
  outlined_text(QRectF(1170, 690, 245, 28), Qt::AlignRight | Qt::AlignVCenter,
                "WIFI " + wifi_address, 18);

  if (change_popup_frames > 0) {
    const float elapsed = change_popup_total_frames - change_popup_frames;
    const float progress = elapsed / std::max(1, change_popup_total_frames - 1);
    const float eased = 1.0f - std::pow(1.0f - progress, 3.0f);
    const QPointF start(720, 350);
    const QPointF center(start.x() + (change_popup_target.x() - start.x()) * eased,
                         start.y() + (change_popup_target.y() - start.y()) * eased);
    const int popup_size = qRound(200 + (change_popup_target_size - 200) * eased);
    const QRectF popup_rect(center.x() - 220, center.y() - popup_size * 0.7,
                            440, popup_size * 1.4);
    outlined_text(popup_rect, Qt::AlignHCenter | Qt::AlignVCenter,
                  change_popup_text, popup_size, change_popup_color);
    --change_popup_frames;
  }

  p.restore();
}

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
  const auto controls_state = sm["controlsState"].getControlsState();

  // tire pressure
  {
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

  int x = radius / 2 + (bdr_s * 2) + (radius + 50);
  const int y = rect().bottom() - footer_h / 2 - 10;

  // cruise gap
  static int last_valid_gap = 4;
  int gap = (int)controls_state.getLongCruiseGap();
  if (gap < 1 || gap > 4) {
    gap = (int)car_state.getCruiseGap();
  }
  if (gap >= 1 && gap <= 4) {
    last_valid_gap = gap;
  } else {
    gap = last_valid_gap;
  }

  p.setOpacity(1.0);
  p.setPen(Qt::NoPen);
  p.setBrush(QBrush(QColor(0, 0, 0, 255 * .1f)));
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);

  QString str;
  float textSize = 50.f;
  QColor textColor = QColor(255, 255, 255, 200);

  if(gap <= 0) {
    str = "N/A";
  }
  else {
    str.sprintf("%d", (int)gap);
    textColor = QColor(120, 255, 120, 200);
    textSize = 70.f;
  }

  configFont(p, "Open Sans", 35, "Bold");
  drawText(p, x, y-20, "GAP", 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y+50, str, textColor);

  // brake
  x = radius / 2 + (bdr_s * 2) + (radius + 50) * 2;
  bool brake_valid = car_state.getBrakeLights();
  float img_alpha = brake_valid ? 1.0f : 0.15f;
  float bg_alpha = brake_valid ? 0.3f : 0.1f;
  drawIcon(p, x, y, ic_brake, QColor(0, 0, 0, (255 * bg_alpha)), img_alpha);

  // auto hold
  int autohold = car_state.getAutoHold();
  if(autohold >= 0) {
    x = radius / 2 + (bdr_s * 2) + (radius + 50) * 3;
    img_alpha = autohold > 0 ? 1.0f : 0.15f;
    bg_alpha = autohold > 0 ? 0.3f : 0.1f;
    drawIcon(p, x, y, autohold > 1 ? ic_autohold_warning : ic_autohold_active,
            QColor(0, 0, 0, (255 * bg_alpha)), img_alpha);
  }

  // 현재 시간 표시
  if (width() > 1200) {
    QTextOption textOpt = QTextOption(Qt::AlignLeft);
    p.setOpacity(1.0);
    p.setPen(QColor(255, 255, 255, 230));

    configFont(p, "Open Sans", 65, "Bold");
    p.drawText(QRect(270, 30, width(), 70),
               QDateTime::currentDateTime().toString("hh:mm"), textOpt);

    configFont(p, "Open Sans", 60, "Bold");
    p.drawText(QRect(270, 110, width(), 70),
               QDateTime::currentDateTime().toString("MM-dd(ddd)"), textOpt);
  }

  // engage-ability icon
  {
    float steer_angle = sm["carState"].getCarState().getSteeringAngleDeg();
    QColor engageBgColor = bg_colors[uiState()->status];
    engageBgColor.setAlpha(166);
    drawIcon(p, rect().right() - radius / 2 - bdr_s * 2, radius / 2 + int(bdr_s * 1.5),
             experimentalMode ? experimental_img : engage_img,
             engageBgColor, 1.0,
             true,
             steer_angle);
  }

  p.restore();
}

void NvgWindow::drawMaxSpeed(QPainter &p) {
  p.save();
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  const auto controls_state = sm["controlsState"].getControlsState();
  const auto car_control = sm["carControl"].getCarControl();
  bool is_metric = s->scene.is_metric;
  bool long_control = s->scene.longitudinal_control;

  // kph
  float applyMaxSpeed = controls_state.getVCruise();
  float cruiseMaxSpeed = controls_state.getVCruiseCluster();
  // Show the values only while longitudinal control is actually active.
  // vCruiseCluster can be initialized while only lateral control is active.
  bool is_cruise_set = car_control.getLongActive() &&
                       (cruiseMaxSpeed > 0 && cruiseMaxSpeed < 255);

  QRect rc(30, 30, 184, 202);
  p.setPen(QPen(QColor(0xff, 0xff, 0xff, 100), 10));
  p.setBrush(QColor(0, 0, 0, 100));
  p.drawRoundedRect(rc, 20, 20);
  p.setPen(Qt::NoPen);

  if (is_cruise_set) {
    char str[256];
    if (is_metric)
        snprintf(str, sizeof(str), "%d", (int)(applyMaxSpeed + 0.5));
    else
        snprintf(str, sizeof(str), "%d", (int)(applyMaxSpeed*KM_TO_MILE + 0.5));

    configFont(p, "Open Sans", 45, "Bold");
    drawText(p, rc.center().x(), 100, str, 255);

    if (is_metric)
        snprintf(str, sizeof(str), "%d", (int)(cruiseMaxSpeed + 0.5));
    else
        snprintf(str, sizeof(str), "%d", (int)(cruiseMaxSpeed*KM_TO_MILE + 0.5));

    configFont(p, "Open Sans", 76, "Bold");
    drawText(p, rc.center().x(), 195, str, 255);
  } else {
    if(long_control) {
      configFont(p, "Open Sans", 48, "sans-semibold");
      drawText(p, rc.center().x(), 100, "OP", 100);
    }
    else {
      configFont(p, "Open Sans", 48, "sans-semibold");
      drawText(p, rc.center().x(), 100, "MAX", 100);
    }

    configFont(p, "Open Sans", 76, "sans-semibold");
    drawText(p, rc.center().x(), 195, "N/A", 100);
  }
  p.restore();
}

void NvgWindow::drawSpeed(QPainter &p) {
  p.save();
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  float cur_speed = std::max(0.0, sm["carState"].getCarState().getVEgoCluster() * (s->scene.is_metric ? MS_TO_KPH : MS_TO_MPH));
  auto car_state = sm["carState"].getCarState();
  float accel = car_state.getAEgo();

  QColor color = QColor(255, 255, 255, 230);

  if(accel > 0) {
    int a = (int)(255.f - (180.f * (accel/2.f)));
    a = std::min(a, 255);
    a = std::max(a, 80);
    color = QColor(a, a, 255, 230);
  }
  else {
    int a = (int)(255.f - (255.f * (-accel/3.f)));
    a = std::min(a, 255);
    a = std::max(a, 60);
    color = QColor(255, a, a, 230);
  }

  QString speed;
  speed.sprintf("%.0f", cur_speed);
  configFont(p, "Open Sans", 176, "Bold");
  drawTextWithColor(p, rect().center().x(), 230, speed, color);

  configFont(p, "Open Sans", 66, "Regular");
  drawText(p, rect().center().x(), 310, s->scene.is_metric ? "km/h" : "mph", 200);

  p.restore();	
}

void NvgWindow::drawSpeedLimit(QPainter &p) {
  p.save();
	
  const SubMaster &sm = *(uiState()->sm);
  auto roadLimitSpeed = sm["roadLimitSpeed"].getRoadLimitSpeed();

  int activeNDA = roadLimitSpeed.getActive();

  int camLimitSpeed = roadLimitSpeed.getCamLimitSpeed();
  int camLimitSpeedLeftDist = roadLimitSpeed.getCamLimitSpeedLeftDist();

  int sectionLimitSpeed = roadLimitSpeed.getSectionLimitSpeed();
  int sectionLeftDist = roadLimitSpeed.getSectionLeftDist();

  int limit_speed = 0;
  int left_dist = 0;

  if(camLimitSpeed > 0 && camLimitSpeedLeftDist > 0) {
    limit_speed = camLimitSpeed;
    left_dist = camLimitSpeedLeftDist;
  }
  else if(sectionLimitSpeed > 0 && sectionLeftDist > 0) {
    limit_speed = sectionLimitSpeed;
    left_dist = sectionLeftDist;
  }

  if(activeNDA > 0)
  {
      int w = 120;
      int h = 54;
      int x = (width() + (bdr_s*2))/2 - w/2 - bdr_s;
      int y = 40 - bdr_s;

      p.setOpacity(1.f);
      p.drawPixmap(x, y, w, h, activeNDA == 1 ? ic_nda : ic_hda);
  }

  // APilot: E2E/ACC 모드 텍스트 (NDA/HDA 아이콘 옆에 표시)
  {
    auto longitudinalPlan = sm["longitudinalPlan"].getLongitudinalPlan();
    bool isE2E = longitudinalPlan.getMpcMode() == 1;
    QString modeText = isE2E ? "E2E" : "ACC";
    QColor modeColor = isE2E ? QColor(0, 230, 130, 220) : QColor(230, 230, 230, 220);

    int ndaW = 120;
    int ndaX = (width() + (bdr_s*2))/2 - ndaW/2 - bdr_s;
    int ndaY = 40 - bdr_s;
    int textX = ndaX + ndaW + 60;
    int textY = ndaY + 27;

    configFont(p, "Open Sans", 40, "Bold");
    drawTextWithColor(p, textX, textY, modeText, modeColor);
  }

  if(limit_speed > 10 && limit_speed < 130)
  {
    int radius_ = 192;

    int x = 30;
    int y = 270;

    p.setPen(Qt::NoPen);
    p.setBrush(QBrush(QColor(255, 0, 0, 255)));
    QRect rect = QRect(x, y, radius_, radius_);
    p.drawEllipse(rect);

    p.setBrush(QBrush(QColor(255, 255, 255, 255)));

    const int tickness = 14;
    rect.adjust(tickness, tickness, -tickness, -tickness);
    p.drawEllipse(rect);

    QString str_limit_speed, str_left_dist;
    str_limit_speed.sprintf("%d", limit_speed);

    if(left_dist >= 1000)
      str_left_dist.sprintf("%.1fkm", left_dist / 1000.f);
    else if(left_dist > 0)
      str_left_dist.sprintf("%dm", left_dist);

    configFont(p, "Open Sans", 80, "Bold");
    p.setPen(QColor(0, 0, 0, 230));
    p.drawText(rect, Qt::AlignCenter, str_limit_speed);

    if(str_left_dist.length() > 0) {
      configFont(p, "Open Sans", 60, "Bold");
      rect.translate(0, radius_/2 + 45);
      rect.adjust(-30, 0, 30, 0);
      p.setPen(QColor(255, 255, 255, 230));
      p.drawText(rect, Qt::AlignCenter, str_left_dist);
    }
  }
  else {
    auto controls_state = sm["controlsState"].getControlsState();
    int sccStockCamAct = (int)controls_state.getSccStockCamAct();
    int sccStockCamStatus = (int)controls_state.getSccStockCamStatus();

    if(sccStockCamAct == 2 && sccStockCamStatus == 2) {
      int radius_ = 192;

      int x = 30;
      int y = 270;

      p.setPen(Qt::NoPen);

      p.setBrush(QBrush(QColor(255, 0, 0, 255)));
      QRect rect = QRect(x, y, radius_, radius_);
      p.drawEllipse(rect);

      p.setBrush(QBrush(QColor(255, 255, 255, 255)));

      const int tickness = 14;
      rect.adjust(tickness, tickness, -tickness, -tickness);
      p.drawEllipse(rect);

      configFont(p, "Open Sans", 70, "Bold");
      p.setPen(QColor(0, 0, 0, 230));
      p.drawText(rect, Qt::AlignCenter, "CAM");
    }
  }

  p.restore();
}

void NvgWindow::drawSteer(QPainter &p) {
  p.save();

  int x = 30;
  int y = 540;

  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  auto car_control = sm["carControl"].getCarControl();

  float steer_angle = car_state.getSteeringAngleDeg();
  float desire_angle = car_control.getActuators().getSteeringAngleDeg();

  configFont(p, "Open Sans", 50, "Bold");

  QString str;
  int width = 192;

  str.sprintf("%.0f°", steer_angle);
  QRect rect = QRect(x, y, width, width);

  p.setPen(QColor(255, 255, 255, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  str.sprintf("%.0f°", desire_angle);
  rect.setRect(x, y + 80, width, width);

  p.setPen(QColor(155, 255, 155, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  p.restore();
}

template <class T>
float interp(float x, std::initializer_list<T> x_list, std::initializer_list<T> y_list, bool extrapolate)
{
  std::vector<T> xData(x_list);
  std::vector<T> yData(y_list);
  int size = xData.size();

  int i = 0;
  if(x >= xData[size - 2]) {
    i = size - 2;
  }
  else {
    while ( x > xData[i+1] ) i++;
  }
  T xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];
  if (!extrapolate) {
    if ( x < xL ) yR = yL;
    if ( x > xR ) yL = yR;
  }

  T dydx = ( yR - yL ) / ( xR - xL );
  return yL + dydx * ( x - xL );
}

void NvgWindow::drawThermal(QPainter &p) {
  p.save();

  const SubMaster &sm = *(uiState()->sm);
  auto deviceState = sm["deviceState"].getDeviceState();

  const auto cpuTempC = deviceState.getCpuTempC();
  //const auto gpuTempC = deviceState.getGpuTempC();
  float ambientTemp = deviceState.getAmbientTempC();

  float cpuTemp = 0.f;
  //float gpuTemp = 0.f;

  if(std::size(cpuTempC) > 0) {
    for(int i = 0; i < std::size(cpuTempC); i++) {
      cpuTemp += cpuTempC[i];
    }
    cpuTemp = cpuTemp / (float)std::size(cpuTempC);
  }

  int w = 192;
  int x = width() - (30 + w);
  int y = 450;

  QString str;
  QRect rect;

  configFont(p, "Open Sans", 50, "Bold");
  str.sprintf("%.0f°C", cpuTemp);
  rect = QRect(x, y, w, w);

  int r = interp<float>(cpuTemp, {50.f, 90.f}, {200.f, 255.f}, false);
  int g = interp<float>(cpuTemp, {50.f, 90.f}, {255.f, 200.f}, false);
  p.setPen(QColor(r, g, 200, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  y += 55;
  configFont(p, "Open Sans", 25, "Bold");
  rect = QRect(x, y, w, w);
  p.setPen(QColor(255, 255, 255, 200));
  p.drawText(rect, Qt::AlignCenter, "CPU");

  y += 80;
  configFont(p, "Open Sans", 50, "Bold");
  str.sprintf("%.0f°C", ambientTemp);
  rect = QRect(x, y, w, w);
  r = interp<float>(ambientTemp, {35.f, 60.f}, {200.f, 255.f}, false);
  g = interp<float>(ambientTemp, {35.f, 60.f}, {255.f, 200.f}, false);
  p.setPen(QColor(r, g, 200, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  y += 55;
  configFont(p, "Open Sans", 25, "Bold");
  rect = QRect(x, y, w, w);
  p.setPen(QColor(255, 255, 255, 200));
  p.drawText(rect, Qt::AlignCenter, "AMBIENT");

  p.restore();
}

void NvgWindow::drawTurnSignals(QPainter &p) {
  p.save();
	
  static int blink_index = 0;
  static int blink_wait = 0;
  static double prev_ts = 0.0;

  if(blink_wait > 0) {
    blink_wait--;
    blink_index = 0;
  }
  else {
    const SubMaster &sm = *(uiState()->sm);
    auto car_state = sm["carState"].getCarState();
    bool left_on = car_state.getLeftBlinker();
    bool right_on = car_state.getRightBlinker();

    const float img_alpha = 0.8f;
    const int fb_w = width() / 2 - 200;
    const int center_x = width() / 2;
    const int w = fb_w / 25;
    const int h = 160;
    const int gap = fb_w / 25;
    const int margin = (int)(fb_w / 3.8f);
    const int base_y = (height() - h) / 2;
    const int draw_count = 8;

    int x = center_x;
    int y = base_y;

    if(left_on) {
      for(int i = 0; i < draw_count; i++) {
        float alpha = img_alpha;
        int d = std::abs(blink_index - i);
        if(d > 0)
          alpha /= d*2;

        p.setOpacity(alpha);
        float factor = (float)draw_count / (i + draw_count);
        p.drawPixmap(x - w - margin, y + (h-h*factor)/2, w*factor, h*factor, ic_turn_signal_l);
        x -= gap + w;
      }
    }

    x = center_x;
    if(right_on) {
      for(int i = 0; i < draw_count; i++) {
        float alpha = img_alpha;
        int d = std::abs(blink_index - i);
        if(d > 0)
          alpha /= d*2;

        float factor = (float)draw_count / (i + draw_count);
        p.setOpacity(alpha);
        p.drawPixmap(x + margin, y + (h-h*factor)/2, w*factor, h*factor, ic_turn_signal_r);
        x += gap + w;
      }
    }

    if(left_on || right_on) {

      double now = millis_since_boot();
      if(now - prev_ts > 900/UI_FREQ) {
        prev_ts = now;
        blink_index++;
      }

      if(blink_index >= draw_count) {
        blink_index = draw_count - 1;
        blink_wait = UI_FREQ/4;
      }
    }
    else {
      blink_index = 0;
    }
  }

  p.restore();
}

void NvgWindow::drawGpsStatus(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto gps = sm["gpsLocationExternal"].getGpsLocationExternal();
  float accuracy = gps.getAccuracy();
  if(accuracy < 0.01f || accuracy > 20.f)
    return;

  int w = 120;
  int h = 100;
  int x = width() - w - 30;
  int y = 30;

  p.save();

  p.setOpacity(0.8);
  p.drawPixmap(x, y, w, h, ic_satellite);

  configFont(p, "Open Sans", 40, "Bold");
  p.setPen(QColor(255, 255, 255, 200));
  p.setRenderHint(QPainter::TextAntialiasing);

  QRect rect = QRect(x, y + h + 10, w, 40);
  rect.adjust(-30, 0, 30, 0);

  QString str;
  str.sprintf("%.1fm", accuracy);
  p.drawText(rect, Qt::AlignHCenter, str);
	
  p.restore();
}

void NvgWindow::drawDebugText(QPainter &p) {
  p.save();
  const SubMaster &sm = *(uiState()->sm);
  QString str, temp;

  int y = 80;
  const int height = 60;

  const int text_x = width()/2 + 250;

  auto controls_state = sm["controlsState"].getControlsState();
  auto car_control = sm["carControl"].getCarControl();
  auto car_state = sm["carState"].getCarState();

  float applyAccel = controls_state.getApplyAccel();

  float aReqValue = controls_state.getAReqValue();
  float aReqValueMin = controls_state.getAReqValueMin();
  float aReqValueMax = controls_state.getAReqValueMax();

  float vEgo = car_state.getVEgo();
  float vEgoRaw = car_state.getVEgoRaw();
  int longControlState = (int)controls_state.getLongControlState();
  float vPid = controls_state.getVPid();
  float upAccelCmd = controls_state.getUpAccelCmd();
  float uiAccelCmd = controls_state.getUiAccelCmd();
  float ufAccelCmd = controls_state.getUfAccelCmd();
  float accel = car_control.getActuators().getAccel();

  const char* long_state[] = {"off", "pid", "stopping", "starting"};

  configFont(p, "Open Sans", 35, "Regular");
  p.setPen(QColor(255, 255, 255, 200));
  p.setRenderHint(QPainter::TextAntialiasing);

  str.sprintf("State: %s\n", long_state[longControlState]);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("vEgo: %.2f/%.2f\n", vEgo*3.6f, vEgoRaw*3.6f);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("vPid: %.2f/%.2f\n", vPid, vPid*3.6f);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("P: %.3f\n", upAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("I: %.3f\n", uiAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("F: %.3f\n", ufAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("Accel: %.3f\n", accel);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("Apply: %.3f, Stock: %.3f\n", applyAccel, aReqValue);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("%.3f (%.3f/%.3f)\n", aReqValue, aReqValueMin, aReqValueMax);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("aEgo: %.3f, %.3f\n", car_state.getAEgo(), car_state.getABasis());
  p.drawText(text_x, y, str);

  auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
  auto lead_one = sm["modelV2"].getModelV2().getLeadsV3()[0];

  float radar_dist = lead_radar.getStatus() && lead_radar.getRadar() ? lead_radar.getDRel() : 0;
  float vision_dist = lead_one.getProb() > .5 ? (lead_one.getX()[0] - 1.5) : 0;

  y += height;
  str.sprintf("Lead: %.1f/%.1f/%.1f\n", radar_dist, vision_dist, (radar_dist - vision_dist));
  p.drawText(text_x, y, str);

  p.restore();
}


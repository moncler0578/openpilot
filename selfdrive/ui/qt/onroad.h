#pragma once
#include <array>
#include <QImage>
#include <QStackedLayout>
#include <QWidget>
#include "selfdrive/common/util.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"
#include "selfdrive/ui/ui.h"
#include <QTimer>
#include <QMap>
#include <QPointF>
#include <QVector>
#include "selfdrive/ui/qt/screenrecorder/screenrecorder.h"

// ***** onroad widgets *****
class OnroadAlerts : public QWidget {
  Q_OBJECT
public:
  OnroadAlerts(QWidget *parent = 0) : QWidget(parent) {};
  void updateAlert(const Alert &a, const QColor &color);
protected:
  void paintEvent(QPaintEvent*) override;
private:
  QColor bg;
  Alert alert = {};
};

// container window for the NVG UI
class NvgWindow : public CameraViewWidget {
  Q_OBJECT
  Q_PROPERTY(bool engageable MEMBER engageable NOTIFY valueChanged);
  Q_PROPERTY(bool experimentalMode MEMBER experimentalMode NOTIFY valueChanged);
  Q_PROPERTY(int status MEMBER status NOTIFY valueChanged);
  Q_PROPERTY(float ang_str MEMBER ang_str NOTIFY valueChanged);

public:
  explicit NvgWindow(VisionStreamType type, QWidget* parent = 0);
  void updateState(const UIState &s);
  void setMapImageEnabled(bool enabled) { show_map_image = enabled; }

signals:
  void valueChanged();

protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  void updateFrameMat(int w, int h) override;
  void drawLaneLines(QPainter &painter, const UIState *s);
  void drawCarrotLead(QPainter &p);

  inline QColor redColor(int alpha = 255)   { return QColor(201, 34, 49, alpha); }
  inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }
  inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }

  double prev_draw_t = 0;
  double last_slow_fps_log_t = 0;
  bool engageable = false;
  bool experimentalMode = false;
  int status = STATUS_DISENGAGED;
  float ang_str = 0;
  
  FirstOrderFilter fps_filter;
  FirstOrderFilter accel_filter;

  // neokii
  void drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity,
                bool rotation = false, float angle = 0.0f);
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);
  void drawText2(QPainter &p, int x, int y, int flags, const QString &text, const QColor &color);
  void drawTextWithColor(QPainter &p, int x, int y, const QString &text, QColor &color);
  void paintEvent(QPaintEvent *event) override;

  const int radius   = 192;
  const int img_size = (radius / 2) * 1.5;
  uint64_t last_update_params;

  QPixmap engage_img;
  QPixmap experimental_img;

  // neokii
  QPixmap ic_nda;
  QPixmap ic_hda;
  QPixmap ic_tire_pressure;

  void drawBottomIcons(QPainter &p);

  // ===== CarrotPilot style HUD (ported from ajouatom/openpilot c3-wip : carrot.cc drawHud) =====
  void drawCarrotHud(QPainter &p);
  void ctRect(QPainter &p, const QRect &r, const QColor &fill, int corner,
              int borderWidth = 0, const QColor &borderColor = QColor(255, 255, 255, 255));
  void ctText(QPainter &p, int x, int y, const QString &text, int size,
              const QColor &color, bool bold = true, bool shadow = false);
  // 박스 안에 상하좌우 정중앙으로 글자 배치
  void ctTextIn(QPainter &p, const QRect &box, const QString &text, int size,
                const QColor &color, bool bold = true);
  // 화면 최상단 좌/우 정보줄 (carrot 의 top_left / top_right 와 동일 위치)
  void drawCarrotInfo(QPainter &p);
  void drawCarrotInfoContent(QPainter &p);
  // 화면 우하단 정보줄 (wifi IP)
  void drawCarrotBottom(QPainter &p);
  void drawCarrotBottomContent(QPainter &p);
  void drawCarrotDeviceState(QPainter &p);
  void drawCarrotNavi(QPainter &p);
  void drawCarrotPlot(QPainter &p);
  void updateCarrotNavi(bool load_images = true);

  QPixmap ic_speed_bg;
  QPixmap ic_speed_bump;
  int  blink_timer = 0;
  int  carrot_param_timer = 0;
  int  my_driving_mode = 3;
  bool noo_enabled = false;
  int  noo_turn_speed = 30;
  int  noo_turn_end_time = 6;
  int  carrot_bump_speed = 35;
  int  show_gear_animation = 1;
  int  show_carrot_hud = 1;
  int  show_path_status_color = 1;
  bool show_map_image = true;
  bool show_route_map_always = false;
  uint64_t carrot_navi_last_read = 0;
  uint64_t carrot_navi_updated_at = 0;
  uint64_t carrot_navi_guidance_updated_at = 0;
  uint64_t carrot_navi_route_updated_at = 0;
  QVector<QPointF> carrot_navi_route;
  double carrot_navi_lat = 0.0;
  double carrot_navi_lon = 0.0;
  QString carrot_navi_road;
  QString carrot_navi_instruction;
  QString carrot_navi_next_instruction;
  int carrot_navi_distance = -1;
  int carrot_navi_turn_type = 0;
  int carrot_navi_next_distance = -1;
  int carrot_navi_next_turn_type = 0;
  int carrot_navi_remain_distance = -1;
  int carrot_navi_remain_time = -1;
  int carrot_navi_speed_limit = 0;
  QString carrot_navi_safety_description;
  QVector<int> carrot_navi_lane_types;
  QVector<int> carrot_navi_lane_active;
  bool carrot_navi_lanes_ahead = false;
  QPixmap carrot_navi_lane_image;
  QPixmap carrot_navi_lane_scaled_image;
  int carrot_navi_lane_scaled_width = 0;
  int carrot_navi_lane_scaled_height = 0;
  qint64 carrot_navi_lane_image_mtime = 0;
  float lead_box_w = 0.0f, lead_box_x = 0.0f, lead_box_y = 0.0f;   // 리드박스 EMA

  // EON-friendly C3 plot: one selected mode, 10 Hz, 20 seconds of history.
  static constexpr int CARROT_PLOT_SAMPLES = 200;
  std::array<std::array<float, CARROT_PLOT_SAMPLES>, 3> carrot_plot_samples = {};
  int carrot_plot_mode = 0;
  int carrot_plot_mode_prev = -1;
  int carrot_plot_count = 0;
  int carrot_plot_next = 0;
  bool carrot_plot_param_initialized = false;
  uint64_t carrot_plot_last_param_read = 0;
  uint64_t carrot_plot_last_sample = 0;
  float carrot_plot_prev_lead_accel = 0.0f;
  float carrot_plot_lead_jerk = 0.0f;
  bool carrot_plot_lead_accel_valid = false;
  QString carrot_plot_title;
  QImage carrot_plot_cache;
  QSize carrot_plot_cache_size;

  // Low-frequency text/status layers. Driving-critical overlays stay at UI_FREQ.
  QImage carrot_info_cache;
  QImage carrot_bottom_cache;
  QImage carrot_device_state_cache;
  QSize carrot_info_cache_size;
  QSize carrot_bottom_cache_size;
  QSize carrot_device_state_cache_size;
  uint64_t carrot_info_last_render = 0;
  uint64_t carrot_bottom_last_render = 0;
  uint64_t carrot_device_state_last_render = 0;

  // ── 팝업 애니메이션 (carrot ui_draw_text_a 이식) ──
  void ctTextAnimStart(int x, int y, const QString &text, int size, const QColor &color, bool enabled);
  void drawTextAnim(QPainter &p);
  int     anim_time = 0;        // 0 이면 비활성
  int     anim_x = 0, anim_y = 0, anim_size = 0;
  QString anim_text;
  QColor  anim_color = QColor(255, 255, 255, 255);
  QString gear_str_last;
  QString lat_debug_font_text;
  int lat_debug_font_width = 0;
  int lat_debug_font_size = 34;
  // ============================================================================
  void drawSpeedLimit(QPainter &p);
  void drawHud(QPainter &p, const cereal::ModelDataV2::Reader &model);
};

// container for all onroad widgets
class OnroadWindow : public QWidget {
  Q_OBJECT
public:
  OnroadWindow(QWidget* parent = 0);
  bool isMapVisible() const { return map && map->isVisible(); }

protected:
  void mousePressEvent(QMouseEvent* e) override;
  void mouseReleaseEvent(QMouseEvent* e) override;
  void paintEvent(QPaintEvent *event) override;

private:
  OnroadAlerts *alerts;
  NvgWindow *nvg;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  bool brake_lights = false;
  bool left_blindspot = false;
  bool right_blindspot = false;
  float steering_angle_deg = 0.0f;
  QWidget *map = nullptr;
  QHBoxLayout *split;
  bool mapbox_enabled = true;
  bool mapbox_param_initialized = false;

  // neokii
  ScreenRecoder *recorder;
  std::shared_ptr<QTimer> record_timer;
  QPoint startPos;

  Params params;

private slots:
  void offroadTransition(bool offroad);
  void updateState(const UIState &s);
};

#pragma once
#include <QStackedLayout>
#include <QWidget>
#include "selfdrive/common/util.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"
#include "selfdrive/ui/ui.h"
#include <QTimer>
#include <QMap>
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
  Q_PROPERTY(bool left_blindspot MEMBER left_blindspot);   // blind spot
  Q_PROPERTY(bool right_blindspot MEMBER right_blindspot); // blind spot

public:
  explicit NvgWindow(VisionStreamType type, QWidget* parent = 0);
  void updateState(const UIState &s);

signals:
  void valueChanged();

protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  void updateFrameMat(int w, int h) override;
  void drawLaneLines(QPainter &painter, const UIState *s);
  void drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data,
                const QPointF &vd, bool is_radar);

  // ChevronInfo: lead status display
  void drawLeadStatus(QPainter &p);
  void drawLeadStatusAtPosition(QPainter &p,
                                const cereal::RadarState::LeadData::Reader &lead_data,
                                const QPointF &chevron_pos,
                                const QString &label);

  inline QColor redColor(int alpha = 255)   { return QColor(201, 34, 49, alpha); }
  inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }
  inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }

  double prev_draw_t = 0;
  bool engageable = false;
  bool experimentalMode = false;
  int status = STATUS_DISENGAGED;
  float ang_str = 0;

  bool left_blindspot  = false; // blind spot
  bool right_blindspot = false; // blind spot

  // ChevronInfo: fade alpha for lead status overlay
  float lead_status_alpha = 0.0f;

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
  QPixmap ic_brake;
  QPixmap ic_autohold_warning;
  QPixmap ic_autohold_active;
  QPixmap ic_nda;
  QPixmap ic_hda;
  QPixmap ic_tire_pressure;
  QPixmap ic_turn_signal_l;
  QPixmap ic_turn_signal_r;
  QPixmap ic_satellite;
  QPixmap ic_speed_bg;

  // Lightweight c3-style value-change popup for EON.
  bool change_popup_enabled = true;
  bool change_popup_initialized = false;
  QString last_popup_gear;
  int last_popup_gap = 0;
  QString change_popup_text;
  QPointF change_popup_target;
  QColor change_popup_color = Qt::white;
  int change_popup_target_size = 40;
  int change_popup_frames = 0;
  const int change_popup_total_frames = 12;

  // Pop the speed-limit sign once when a new camera/section is detected.
  bool camera_sign_initialized = false;
  bool camera_sign_was_visible = false;
  int camera_sign_missing_frames = 0;
  int camera_sign_popup_frames = 0;
  const int camera_sign_popup_total_frames = 15;

  void drawMaxSpeed(QPainter &p);
  void drawSpeed(QPainter &p);
  void drawBottomIcons(QPainter &p);
  void drawSpeedLimit(QPainter &p);
  void drawSteer(QPainter &p);
  void drawThermal(QPainter &p);
  void drawTurnSignals(QPainter &p);
  void drawGpsStatus(QPainter &p);
  void drawDebugText(QPainter &p);
  void drawHud(QPainter &p, const cereal::ModelDataV2::Reader &model);
  void drawCarrotUi(QPainter &p);
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
  QWidget *map = nullptr;
  QHBoxLayout *split;

  // neokii
  ScreenRecoder *recorder;
  std::shared_ptr<QTimer> record_timer;
  QPoint startPos;

  Params params;

private slots:
  void offroadTransition(bool offroad);
  void updateState(const UIState &s);
};

#include "selfdrive/ui/qt/offroad/settings.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <string>
#include <map>

#include <QDebug>

#ifndef QCOM
#include "selfdrive/ui/qt/offroad/networking.h"
#endif

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

#include "selfdrive/common/params.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"

#include <QComboBox>
#include <QAbstractItemView>
#include <QScroller>
#include <QListView>
#include <QListWidget>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPainter>
#include <QPainterPath>
#include <QMouseEvent>
#include <QDateTime>
#include <QSet>
#include <QFile>
#include <QDir>

// ── CameraOffset Control ─────────────────────────────────────────
// step: 0.01m, range: -0.20 ~ 0.20
CameraOffsetControl::CameraOffsetControl(const QString &title,
                                         const QString &desc,
                                         const QString &icon,
                                         QWidget *parent)
    : AbstractControl(title, desc, icon, parent) {

  QWidget *btn_widget = new QWidget();
  QHBoxLayout *btn_layout = new QHBoxLayout(btn_widget);
  btn_layout->setContentsMargins(0, 8, 0, 8);
  btn_layout->setSpacing(12);

  const QString btn_style = R"(
    QPushButton {
      font-size: 40px;
      font-weight: bold;
      border-radius: 10px;
      background-color: #393939;
      color: #ffffff;
      min-width: 80px;
      max-width: 80px;
      min-height: 70px;
    }
    QPushButton:pressed { background-color: #4a4a4a; }
  )";

  minus_btn = new QPushButton("−");
  minus_btn->setStyleSheet(btn_style);
  connect(minus_btn, &QPushButton::clicked, [=]() { changeValue(-1); });

  value_label = new QLabel();
  value_label->setAlignment(Qt::AlignCenter);
  value_label->setStyleSheet("font-size: 40px; color: #ffffff; min-width: 140px;");

  plus_btn = new QPushButton("+");
  plus_btn->setStyleSheet(btn_style);
  connect(plus_btn, &QPushButton::clicked, [=]() { changeValue(+1); });

  btn_layout->addStretch();
  btn_layout->addWidget(minus_btn);
  btn_layout->addWidget(value_label);
  btn_layout->addWidget(plus_btn);

  qobject_cast<QVBoxLayout*>(layout())->addWidget(btn_widget);
  refresh();
}

void CameraOffsetControl::changeValue(int delta) {
  std::string raw = params.get("CameraOffset");
  double val = raw.empty() ? -0.06 : std::stod(raw);
  val += delta * 0.01;
  val = std::round(val * 100.0) / 100.0;
  val = std::max(-0.20, std::min(0.20, val));
  params.put("CameraOffset", std::to_string(val));
  refresh();
}

void CameraOffsetControl::refresh() {
  std::string raw = params.get("CameraOffset");
  double val = raw.empty() ? -0.06 : std::stod(raw);
  val = std::round(val * 100.0) / 100.0;
  value_label->setText(QString::number(val, 'f', 2) + " m");
  minus_btn->setEnabled(val > -0.20);
  plus_btn->setEnabled(val < 0.20);
}

// ── PathOffset Control ───────────────────────────────────────────
// step: 0.01m, range: -1.00 ~ 1.00
PathOffsetControl::PathOffsetControl(const QString &title,
                                     const QString &desc,
                                     const QString &icon,
                                     QWidget *parent)
    : AbstractControl(title, desc, icon, parent) {

  QWidget *btn_widget = new QWidget();
  QHBoxLayout *btn_layout = new QHBoxLayout(btn_widget);
  btn_layout->setContentsMargins(0, 8, 0, 8);
  btn_layout->setSpacing(12);

  const QString btn_style = R"(
    QPushButton {
      font-size: 40px;
      font-weight: bold;
      border-radius: 10px;
      background-color: #393939;
      color: #ffffff;
      min-width: 80px;
      max-width: 80px;
      min-height: 70px;
    }
    QPushButton:pressed { background-color: #4a4a4a; }
  )";

  minus_btn = new QPushButton("−");
  minus_btn->setStyleSheet(btn_style);
  connect(minus_btn, &QPushButton::clicked, [=]() { changeValue(-1); });

  value_label = new QLabel();
  value_label->setAlignment(Qt::AlignCenter);
  value_label->setStyleSheet("font-size: 40px; color: #ffffff; min-width: 140px;");

  plus_btn = new QPushButton("+");
  plus_btn->setStyleSheet(btn_style);
  connect(plus_btn, &QPushButton::clicked, [=]() { changeValue(+1); });

  btn_layout->addStretch();
  btn_layout->addWidget(minus_btn);
  btn_layout->addWidget(value_label);
  btn_layout->addWidget(plus_btn);

  qobject_cast<QVBoxLayout*>(layout())->addWidget(btn_widget);
  refresh();
}

void PathOffsetControl::changeValue(int delta) {
  std::string raw = params.get("PathOffset");
  double val = raw.empty() ? 0.0 : std::stod(raw);
  val += delta * 0.01;
  val = std::round(val * 100.0) / 100.0;
  val = std::max(-1.00, std::min(1.00, val));
  params.put("PathOffset", std::to_string(val));
  refresh();
}

void PathOffsetControl::refresh() {
  std::string raw = params.get("PathOffset");
  double val = raw.empty() ? 0.0 : std::stod(raw);
  val = std::round(val * 100.0) / 100.0;
  value_label->setText(QString::number(val, 'f', 2) + " m");
  minus_btn->setEnabled(val > -1.00);
  plus_btn->setEnabled(val < 1.00);
}

// ── ChevronInfo Control ─────────────────────────────────────────
ChevronInfoControl::ChevronInfoControl(const QString &title,
                                       const QString &desc,
                                       const QString &icon,
                                       QWidget *parent)
    : AbstractControl(title, desc, icon, parent) {

  QWidget *btn_widget = new QWidget();
  QHBoxLayout *btn_layout = new QHBoxLayout(btn_widget);
  btn_layout->setContentsMargins(0, 8, 0, 8);
  btn_layout->setSpacing(8);

  for (int i = 0; i < labels.size(); i++) {
    buttons[i] = new QPushButton(labels[i]);
    buttons[i]->setFixedHeight(70);
    buttons[i]->setCheckable(true);
    buttons[i]->setStyleSheet(R"(
      QPushButton {
        font-size: 30px;
        border-radius: 10px;
        background-color: #393939;
        color: #aaaaaa;
      }
      QPushButton:checked {
        background-color: #0064ff;
        color: #ffffff;
      }
      QPushButton:pressed {
        background-color: #4a4a4a;
      }
    )");

    connect(buttons[i], &QPushButton::clicked, [=]() {
      params.put("ChevronInfo", std::to_string(i));
      refresh();
    });

    btn_layout->addWidget(buttons[i]);
  }

  // AbstractControl의 메인 레이아웃(QVBoxLayout)에 버튼 행 추가
  qobject_cast<QVBoxLayout*>(layout())->addWidget(btn_widget);
  refresh();
}

void ChevronInfoControl::refresh() {
  int val = std::atoi(params.get("ChevronInfo").c_str());
  val = std::clamp(val, 0, 4);
  for (int i = 0; i < labels.size(); i++) {
    buttons[i]->setChecked(i == val);
  }
}

// ── AutoLaneChangeTimer Control ─────────────────────────────────
AutoLaneChangeTimerControl::AutoLaneChangeTimerControl(const QString &title,
                                                       const QString &desc,
                                                       const QString &icon,
                                                       QWidget *parent)
    : AbstractControl(title, desc, icon, parent) {

  // 버튼을 제목 아래에 배치
  QWidget *btn_widget = new QWidget();
  QHBoxLayout *btn_layout = new QHBoxLayout(btn_widget);
  btn_layout->setContentsMargins(0, 8, 0, 8);
  btn_layout->setSpacing(8);

  for (int i = 0; i < labels.size(); i++) {
    buttons[i] = new QPushButton(labels[i]);
    buttons[i]->setFixedHeight(70);
    buttons[i]->setCheckable(true);
    buttons[i]->setStyleSheet(R"(
      QPushButton {
        font-size: 30px;
        border-radius: 10px;
        background-color: #393939;
        color: #aaaaaa;
      }
      QPushButton:checked {
        background-color: #0064ff;
        color: #ffffff;
      }
      QPushButton:pressed {
        background-color: #4a4a4a;
      }
    )");

    connect(buttons[i], &QPushButton::clicked, [=]() {
      params.put("AutoLaneChangeTimer", std::to_string(i));
      refresh();
    });

    btn_layout->addWidget(buttons[i]);
  }

  qobject_cast<QVBoxLayout*>(layout())->addWidget(btn_widget);
  refresh();
}

void AutoLaneChangeTimerControl::refresh() {
  int val = std::atoi(params.get("AutoLaneChangeTimer").c_str());
  val = std::clamp(val, 0, (int)labels.size() - 1);
  for (int i = 0; i < labels.size(); i++) {
    buttons[i]->setChecked(i == val);
  }
}

// ── DynamicLaneProfile Control ──────────────────────────────────
DynamicLaneProfileControl::DynamicLaneProfileControl(const QString &title,
                                                     const QString &desc,
                                                     const QString &icon,
                                                     QWidget *parent)
    : AbstractControl(title, desc, icon, parent) {

  // 버튼을 제목 아래에 배치
  QWidget *btn_widget = new QWidget();
  QHBoxLayout *btn_layout = new QHBoxLayout(btn_widget);
  btn_layout->setContentsMargins(0, 8, 0, 8);
  btn_layout->setSpacing(8);

  for (int i = 0; i < labels.size(); i++) {
    buttons[i] = new QPushButton(labels[i]);
    buttons[i]->setFixedHeight(70);
    buttons[i]->setCheckable(true);
    buttons[i]->setStyleSheet(R"(
      QPushButton {
        font-size: 30px;
        border-radius: 10px;
        background-color: #393939;
        color: #aaaaaa;
      }
      QPushButton:checked {
        background-color: #0064ff;
        color: #ffffff;
      }
      QPushButton:pressed {
        background-color: #4a4a4a;
      }
    )");

    connect(buttons[i], &QPushButton::clicked, [=]() {
      params.put("DynamicLaneProfile", std::to_string(i));
      refresh();
    });

    btn_layout->addWidget(buttons[i]);
  }

  qobject_cast<QVBoxLayout*>(layout())->addWidget(btn_widget);
  refresh();
}

void DynamicLaneProfileControl::refresh() {
  int val = std::atoi(params.get("DynamicLaneProfile").c_str());
  val = std::clamp(val, 0, (int)labels.size() - 1);
  for (int i = 0; i < labels.size(); i++) {
    buttons[i]->setChecked(i == val);
  }
}

// ── CarrotPilot Auto-Tuner (commit 9dd5e2c port) ─────────────────────────
// nTune 토크 파일 헬퍼: latcontrol_torque가 /data/ntune/lat_torque*.json 을
// 라이브 리로드하므로 토크 파라미터 복원은 이 파일에 기록
static QString findNtuneTorqueFileS() {
  QDir dir("/data/ntune");
  const QStringList files = dir.entryList({"lat_torque*.json"}, QDir::Files, QDir::Name);
  if (!files.isEmpty()) return dir.filePath(files.first());
  return "/data/ntune/lat_torque_v4.json";
}

static void writeNtuneTorqueValueS(const QString &key, double value) {
  QString path = findNtuneTorqueFileS();
  QJsonObject obj;
  QFile f(path);
  if (f.open(QIODevice::ReadOnly)) {
    obj = QJsonDocument::fromJson(f.readAll()).object();
    f.close();
  }
  obj[key] = value;
  QDir().mkpath("/data/ntune");
  if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    f.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));
    f.close();
    // nTune.write_config와 동일하게 0666 권한 유지
    f.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner |
                     QFileDevice::ReadGroup | QFileDevice::WriteGroup |
                     QFileDevice::ReadOther | QFileDevice::WriteOther);
  }
}

static void writeNtuneCommonValueS(const QString &key, double value) {
  QString path = "/data/ntune/common.json";
  QJsonObject obj;
  QFile f(path);
  if (f.open(QIODevice::ReadOnly)) {
    obj = QJsonDocument::fromJson(f.readAll()).object();
    f.close();
  }
  obj[key] = value;
  QDir().mkpath("/data/ntune");
  if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    f.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));
    f.close();
    f.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner |
                     QFileDevice::ReadGroup | QFileDevice::WriteGroup |
                     QFileDevice::ReadOther | QFileDevice::WriteOther);
  }
}

// 포팅판 학습 대상 파라미터의 공장 기본값 (python carrot_learning.py 와 동일)
static const std::map<std::string, std::string> kAutoTunerDefaults = {
  {"CruiseMaxVals0", "180"},
  {"CruiseMaxVals1", "120"},
  {"CruiseMaxVals2", "80"},
  {"CruiseMaxVals3", "60"},
  {"TFollowGap1", "100"},
  {"TFollowGap2", "140"},
  {"TFollowGap3", "200"},
  {"TFollowGap4", "200"},
  {"PathOffset", "0.0"},
  {"CarrotLongActuatorDelay", "0.4"},   // _LONG_DELAY_DEFAULT (carrot_learning.py)
  {"CarrotLongKf", "1.0"},              // _LONG_KF_DEFAULT (carrot_learning.py)
  // ── Auto-Tuner Phase 6: 비전 커브 감속 (carrot_learning.py _TURN_*_DEFAULTS와 동일) ──
  {"TurnEnteringDecel0", "-10"},
  {"TurnEnteringDecel1", "-30"},
  {"TurnTurningAcc0", "120"},
  {"TurnTurningAcc1", "94"},
  {"TurnTurningAcc2", "90"},
  {"TurnTurningAcc3", "80"},
  {"TurnTurningAcc4", "-10"},
  {"TurnLeavingAcc", "50"},
};

// AutoTunerGraphWidget
AutoTunerGraphWidget::AutoTunerGraphWidget(QWidget *parent) : QWidget(parent) {
  setAttribute(Qt::WA_OpaquePaintEvent, false);
}

void AutoTunerGraphWidget::setData(const QList<QString> &ts, const QMap<QString, QList<double>> &histories, const QMap<QString, QColor> &cols) {
  timestamps = ts;
  param_histories = histories;
  colors = cols;
  selected_index = -1;
  update();
}

void AutoTunerGraphWidget::setSelectedParam(const QString &param) {
  selected_param = param;
  update();
}

void AutoTunerGraphWidget::setHiddenParams(const QSet<QString> &params) {
  hidden_params = params;
  update();
}

void AutoTunerGraphWidget::mousePressEvent(QMouseEvent *event) {
  if (timestamps.isEmpty()) return;

  int margin_left = 80;
  int margin_right = 40;
  QRect graph_rect = rect().adjusted(margin_left, 80, -margin_right, -40);

  int steps_x = timestamps.size() - 1;
  if (steps_x < 1) steps_x = 1;

  int click_x = event->x();
  int closest_idx = 0;
  int min_dist = 999999;

  for (int i = 0; i < timestamps.size(); i++) {
    int node_x = graph_rect.left() + i * graph_rect.width() / steps_x;
    int dist = std::abs(click_x - node_x);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  if (min_dist < 60) {
    selected_index = closest_idx;
  } else {
    selected_index = -1;
  }
  update();
}

// Show-All(전체) 뷰에서 제외할 대규모 스케일 파라미터 판정 (commit e06a7dd 21f7994a)
// 조향계열(PathOffset/latAccelFactor/friction/steerActuatorDelay)은 값이 작아 같이
// 그리면 바닥에 깔리므로, 대규모(CruiseMaxVals/TFollowGap/Turn*)는 개별 선택 시에만 표시.
// Turn*(TurnEnteringDecel/TurnTurningAcc/TurnLeavingAcc)도 x100 정수 저장이라
// CruiseMaxVals/TFollowGap과 동일한 스케일(-30~200) — 빠뜨리면 소규모 nTune
// 파라미터들이 Show-All에서 바닥에 깔린다.
static bool isLargeScaleParam(const QString &param) {
  return param.startsWith("CruiseMaxVals") || param.startsWith("TFollowGap") ||
         param.startsWith("TurnEnteringDecel") || param.startsWith("TurnTurningAcc") ||
         param == "TurnLeavingAcc";
}

void AutoTunerGraphWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Background
  painter.fillRect(rect(), QColor("#1f1f1f"));

  if (timestamps.isEmpty() || param_histories.isEmpty()) {
    painter.setPen(QColor("#888888"));
    { QFont _f("Arial"); _f.setPixelSize(34); painter.setFont(_f); }
    painter.drawText(rect(), Qt::AlignCenter, "No historical data to display");
    return;
  }

  int margin_left = 80;
  int margin_right = 40;
  int margin_top = 80;
  int margin_bottom = 40;
  QRect graph_rect = rect().adjusted(margin_left, margin_top, -margin_right, -margin_bottom);

  int steps_x = timestamps.size() - 1;
  if (steps_x < 1) steps_x = 1;

  // Draw Grid Lines (Vertical & Horizontal)
  painter.setPen(QPen(QColor("#2d2d2d"), 2, Qt::SolidLine));

  // X grid lines
  for (int i = 0; i <= steps_x; i++) {
    int x = graph_rect.left() + i * graph_rect.width() / steps_x;
    painter.drawLine(x, graph_rect.top(), x, graph_rect.bottom());
  }

  // Y grid lines
  int steps_y = 4;
  for (int i = 0; i <= steps_y; i++) {
    int y = graph_rect.top() + i * graph_rect.height() / steps_y;
    painter.drawLine(graph_rect.left(), y, graph_rect.right(), y);
  }

  double global_min = 0.0;
  double global_max = 0.0;
  bool first_val = true;

  // 전체 파라미터 공통 글로벌 축 범위 계산 (일관된 스케일 유지)
  // 접힌 그룹(hidden_params)은 제외, Show-All(미선택) 뷰에서는 대규모 스케일도 제외
  for (const QString &param : param_histories.keys()) {
    if (hidden_params.contains(param)) continue;                          // 접힌 그룹: 그래프에서 제외
    if (selected_param.isEmpty() && isLargeScaleParam(param)) continue;
    QList<double> values = param_histories[param];
    if (values.size() != timestamps.size()) continue;
    for (double val : values) {
      if (first_val) {
        global_min = val;
        global_max = val;
        first_val = false;
      } else {
        if (val < global_min) global_min = val;
        if (val > global_max) global_max = val;
      }
    }
  }

  // Draw Line Paths
  painter.setBrush(Qt::NoBrush);
  for (const QString &param : param_histories.keys()) {
    if (hidden_params.contains(param)) continue;                          // 접힌 그룹: 그래프에서 제외
    if (selected_param.isEmpty() && isLargeScaleParam(param)) continue;
    QList<double> values = param_histories[param];
    if (values.size() != timestamps.size()) continue;

    double min_val = global_min;
    double max_val = global_max;
    double diff = max_val - min_val;

    bool is_highlighted = selected_param.isEmpty() || (selected_param == param);
    int opacity = 255;
    QColor color;

    if (!selected_param.isEmpty()) {
      if (selected_param == param) {
        color = colors.value(param, QColor(Qt::white));
        opacity = 255;
      } else {
        color = QColor("#444444");  // 비선택 파라미터는 어두운 회색
        opacity = 80;
      }
    } else {
      color = colors.value(param, QColor(Qt::white));
      opacity = 255;
    }

    color.setAlpha(opacity);
    painter.setBrush(Qt::NoBrush);

    // y 좌표 계산 (상하 12% 패딩 공통 적용)
    auto calc_y = [&](double val) -> int {
      if (diff < 1e-5) return graph_rect.top() + graph_rect.height() / 2;
      int pad = graph_rect.height() * 12 / 100;
      int usable = graph_rect.height() - 2 * pad;
      return graph_rect.bottom() - pad - (int)((val - min_val) / diff * usable);
    };

    bool is_selected = (selected_param == param);
    // Show-All(미선택) 뷰: 실선 두껍게/점선 넓게, 상세 뷰: 기존 폭 유지 (commit e06a7dd)
    int solid_w = is_selected ? 8 : (selected_param.isEmpty() ? 5 : 2);
    int dot_w = std::max(1, solid_w / 2);   // 점선(미변경) 구간은 절반 폭

    // ── 구간별 라인: 값이 바뀐 구간=실선, 그대로인 구간=점선 ──
    for (int i = 0; i < values.size() - 1; i++) {
      int x0 = graph_rect.left() + i * graph_rect.width() / steps_x;
      int x1 = graph_rect.left() + (i + 1) * graph_rect.width() / steps_x;
      int y0 = calc_y(values[i]);
      int y1 = calc_y(values[i + 1]);
      bool changed = std::abs(values[i + 1] - values[i]) > 1e-9;

      QPen seg_pen(color, changed ? solid_w : dot_w);
      if (!selected_param.isEmpty() && !is_selected) {
        seg_pen.setStyle(Qt::DotLine);                // 비선택 파라미터는 전체 점선
      } else if (changed) {
        seg_pen.setStyle(Qt::SolidLine);
      } else {
        // 미변경 구간: 넓은 간격 점선 (Show-All에서 1:4)
        seg_pen.setStyle(Qt::CustomDashLine);
        QVector<qreal> dashes; dashes << 1 << (selected_param.isEmpty() ? 4 : 3);
        seg_pen.setDashPattern(dashes);
      }
      painter.setPen(seg_pen);
      painter.drawLine(x0, y0, x1, y1);
    }

    // ── 노드 + 값 라벨: 변경점(직전과 다른 값) + 첫 점에만 ──
    for (int i = 0; i < values.size(); i++) {
      bool changed = (i == 0) || (std::abs(values[i] - values[i - 1]) > 1e-9);
      if (!changed) continue;                         // 변경점만 강조 (commit e06a7dd)
      if (!is_highlighted) continue;

      int x = graph_rect.left() + i * graph_rect.width() / steps_x;
      int y = calc_y(values[i]);

      painter.setBrush(color);
      painter.setPen(Qt::NoPen);
      int dot_size = is_selected ? 16 : 10;
      painter.drawEllipse(QPoint(x, y), dot_size / 2, dot_size / 2);

      // 라벨 색 = 파라미터 자기 색 (가독성, commit e06a7dd)
      painter.setPen(color);
      { QFont _f("Arial", -1, QFont::Bold); _f.setPixelSize(is_selected ? 20 : 16); painter.setFont(_f); }
      QString val_str = QString::number(values[i], 'g', 4);

      int lbl_w = 110;
      int lbl_h = 34;

      // 가로: 좌/우 끝 노드는 안쪽으로 밀어 영역 침범 방지
      int lbl_x;
      Qt::Alignment h_align = Qt::AlignHCenter;
      if (x - lbl_w / 2 < graph_rect.left()) {
        lbl_x = x + 8; h_align = Qt::AlignLeft;
      } else if (x + lbl_w / 2 > graph_rect.right()) {
        lbl_x = x - lbl_w - 8; h_align = Qt::AlignRight;
      } else {
        lbl_x = x - lbl_w / 2;
      }

      // 세로: 노드 위쪽, 상단 근접 시 아래로 뒤집기 (라벨 오프셋 상향)
      int lbl_y = y - lbl_h - 12;
      if (lbl_y < graph_rect.top()) lbl_y = y + 12;

      painter.drawText(QRect(lbl_x, lbl_y, lbl_w, lbl_h),
                       h_align | Qt::AlignVCenter, val_str);
    }
  }

  // 터치/클릭 시 세로 가이드 라인 + 시간 툴팁
  if (selected_index >= 0 && selected_index < timestamps.size()) {
    int x = graph_rect.left() + selected_index * graph_rect.width() / steps_x;

    // Vertical Guide
    painter.setPen(QPen(QColor("#ffaa00"), 2.5, Qt::DashLine));
    painter.drawLine(x, graph_rect.top(), x, graph_rect.bottom());

    // Tooltip Background & Text
    QString date_str = timestamps[selected_index];
    { QFont _f("Arial", -1, QFont::Bold); _f.setPixelSize(24); painter.setFont(_f); }

    QFontMetrics fm = painter.fontMetrics();
    int txt_w = fm.horizontalAdvance(date_str) + 24;
    int txt_h = 40;
    QRect tooltip_rect(x - txt_w / 2, margin_top - txt_h - 8, txt_w, txt_h);

    // Boundary check
    if (tooltip_rect.left() < 10) tooltip_rect.moveLeft(10);
    if (tooltip_rect.right() > width() - 10) tooltip_rect.moveRight(width() - 10);

    painter.setBrush(QColor("#2d2d2d"));
    painter.setPen(QPen(QColor("#ffaa00"), 2));
    painter.drawRoundedRect(tooltip_rect, 10, 10);
    painter.setPen(QColor("#ffffff"));
    painter.drawText(tooltip_rect, Qt::AlignCenter, date_str);
  }
}

// AutoTunerHistoryPanel
AutoTunerHistoryPanel::AutoTunerHistoryPanel(QWidget* parent) : QFrame(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setContentsMargins(20, 20, 20, 20);
  main_layout->setSpacing(20);

  // Left Column: Parameters List
  QVBoxLayout *left_layout = new QVBoxLayout();
  left_layout->setSpacing(15);

  QLabel *list_title = new QLabel("Parameters");
  list_title->setStyleSheet("font-size: 42px; font-weight: bold; color: white;");
  left_layout->addWidget(list_title);

  QScrollArea *scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setFixedWidth(340);
  // 좁은 패널(340px)이라 카드 리스트만큼 굵게는 못 하지만, 기본 스크롤바보다는
  // 충분히 두껍게(핸들 폭 30px)해 손가락으로 잡기 쉽게 함.
  scroll->setStyleSheet(R"(
    QScrollArea { background: transparent; }
    QWidget { background: transparent; }
    QScrollBar:vertical {
      background: #2b2b2b;
      width: 30px;
      margin: 0px;
      border-radius: 15px;
    }
    QScrollBar::handle:vertical {
      background: #6b7280;
      min-height: 60px;
      border-radius: 13px;
    }
    QScrollBar::handle:vertical:pressed {
      background: #9ca3af;
    }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
      height: 0px;
      background: none;
    }
    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
      background: none;
    }
  )");
  QScroller::grabGesture(scroll->viewport(), QScroller::LeftMouseButtonGesture);

  QWidget *scroll_widget = new QWidget();
  param_list_layout = new QVBoxLayout(scroll_widget);
  param_list_layout->setContentsMargins(0, 0, 0, 0);
  param_list_layout->setSpacing(8);
  scroll->setWidget(scroll_widget);
  left_layout->addWidget(scroll, 1);

  // Apply LAT / LONG 토글 (미설정 시 기본 ON으로 시드 — python 기본값과 일치)
  QVBoxLayout *toggles_layout = new QVBoxLayout();
  toggles_layout->setSpacing(10);
  toggles_layout->setContentsMargins(0, 10, 0, 0);

  {
    Params p;
    if (p.get("CarrotTunerApplyLat").empty()) p.putBool("CarrotTunerApplyLat", true);
    if (p.get("CarrotTunerApplyLong").empty()) p.putBool("CarrotTunerApplyLong", true);
  }

  // (commit dff7287) 터치 면적을 키우기 위해 가로로 길던 버튼을 정사각형에 가깝게
  // 바꾸고 두 개를 나란히(좌우) 배치한다. 글씨는 3줄로 표시.
  QPushButton *lat_toggle = new QPushButton(this);
  lat_toggle->setFixedHeight(150);
  QPushButton *long_toggle = new QPushButton(this);
  long_toggle->setFixedHeight(150);

  auto updateToggles = [=]() {
    bool apply_lat = Params().getBool("CarrotTunerApplyLat");
    bool apply_long = Params().getBool("CarrotTunerApplyLong");
    // 활성(ON)/비활성(OFF) 색상은 포크 기존 배색(빨강/회색) 유지. 3줄 텍스트가
    // 박스를 넘지 않도록 폰트 크기만 26px→22px로 축소 (commit dff7287).
    QString on_style = "background-color: #bb3333; font-size: 22px; font-weight: bold; border-radius: 10px; color: white;";
    QString off_style = "background-color: #4a5568; font-size: 22px; font-weight: bold; border-radius: 10px; color: white;";
    lat_toggle->setText(tr("Apply LAT\n(Steering)\n%1").arg(apply_lat ? "ON" : "OFF"));
    lat_toggle->setStyleSheet(apply_lat ? on_style : off_style);
    long_toggle->setText(tr("Apply LONG\n(Accel/Brake)\n%1").arg(apply_long ? "ON" : "OFF"));
    long_toggle->setStyleSheet(apply_long ? on_style : off_style);
  };
  updateToggles();

  connect(lat_toggle, &QPushButton::clicked, this, [=]() {
    bool current = Params().getBool("CarrotTunerApplyLat");
    Params().putBool("CarrotTunerApplyLat", !current);
    updateToggles();
  });
  connect(long_toggle, &QPushButton::clicked, this, [=]() {
    bool current = Params().getBool("CarrotTunerApplyLong");
    Params().putBool("CarrotTunerApplyLong", !current);
    updateToggles();
  });

  // (commit dff7287) 두 토글을 좌우로 나란히 배치 (각각 정사각형에 가까운 형태)
  QHBoxLayout *apply_row = new QHBoxLayout();
  apply_row->setSpacing(10);
  apply_row->addWidget(lat_toggle);
  apply_row->addWidget(long_toggle);
  toggles_layout->addLayout(apply_row);
  left_layout->addLayout(toggles_layout);

  // Right Column: Chart + Controls
  QVBoxLayout *right_layout = new QVBoxLayout();
  right_layout->setSpacing(20);

  QHBoxLayout *header_layout = new QHBoxLayout();
  header_layout->addStretch();

  QPushButton *btn_card_list = new QPushButton("View Card Type");
  btn_card_list->setStyleSheet("background-color: #10b981; font-size: 40px; border-radius: 10px; color: white; font-weight: bold; padding: 0px 50px;");
  btn_card_list->setFixedHeight(110);
  connect(btn_card_list, &QPushButton::clicked, this, [=]() {
    AutoTunerCardListDialog dlg(this);
    dlg.exec();
    refreshHistory();  // 카드 리스트에서 Restore/Delete 후 그래프 갱신
  });
  header_layout->addWidget(btn_card_list);

  QPushButton *btn_all = new QPushButton("Show All Parameters");
  btn_all->setStyleSheet("background-color: #0ea5e9; font-size: 40px; border-radius: 10px; color: white; font-weight: bold; padding: 0px 50px;");
  btn_all->setFixedHeight(110);
  connect(btn_all, &QPushButton::clicked, this, [=]() {
    if (graph_widget) graph_widget->setSelectedParam("");
    selected_param = "";
    updateLabelColors();
  });
  header_layout->addWidget(btn_all);

  QPushButton *btn_clear = new QPushButton("Clear All Logs");
  btn_clear->setStyleSheet("background-color: #eab308; font-size: 40px; border-radius: 10px; color: white; font-weight: bold; padding: 0px 50px;");
  btn_clear->setFixedHeight(110);
  connect(btn_clear, &QPushButton::clicked, this, &AutoTunerHistoryPanel::clearAll);
  header_layout->addWidget(btn_clear);

  QPushButton *close_btn = new QPushButton("Close");
  close_btn->setStyleSheet("background-color: #bb3333; font-size: 40px; border-radius: 10px; color: white; font-weight: bold; padding: 0px 50px;");
  close_btn->setFixedHeight(110);
  connect(close_btn, &QPushButton::clicked, this, [=]() {
    QWidget* w = this->window();
    if (w) {
      QDialog* dlg = qobject_cast<QDialog*>(w);
      if (dlg) dlg->reject();
      else w->close();
    }
  });
  header_layout->addWidget(close_btn);

  right_layout->addLayout(header_layout);

  graph_widget = new AutoTunerGraphWidget(this);
  graph_widget->setMinimumHeight(750);
  right_layout->addWidget(graph_widget, 1);

  main_layout->addLayout(left_layout);
  main_layout->addLayout(right_layout, 1);

  // 파라미터별 고정 색상 (포팅판 학습 대상에 맞춤)
  param_colors.clear();
  param_colors["CruiseMaxVals0"] = QColor("#3b82f6");  // Blue
  param_colors["CruiseMaxVals1"] = QColor("#60a5fa");  // Light Blue
  param_colors["CruiseMaxVals2"] = QColor("#10b981");  // Mint
  param_colors["CruiseMaxVals3"] = QColor("#84cc16");  // Lime
  param_colors["TFollowGap1"] = QColor("#06b6d4");     // Cyan
  param_colors["TFollowGap2"] = QColor("#14b8a6");     // Teal
  param_colors["TFollowGap3"] = QColor("#ffffff");     // White
  param_colors["TFollowGap4"] = QColor("#a855f7");     // Purple
  param_colors["PathOffset"] = QColor("#e879f9");      // Light Magenta
  param_colors["latAccelFactor"] = QColor("#f59e0b");  // Amber (토크)
  param_colors["friction"] = QColor("#f43f5e");        // Rose (토크)
  param_colors["steerActuatorDelay"] = QColor("#fb923c"); // Orange (조향 지연)
  // 곡선(Curve) 그룹: Entering=Green 계열, Turning=Yellow 계열, Leaving=단독 Teal
  param_colors["TurnEnteringDecel0"] = QColor("#22c55e"); // Green
  param_colors["TurnEnteringDecel1"] = QColor("#16a34a"); // Dark Green
  param_colors["TurnTurningAcc0"] = QColor("#eab308");    // Yellow
  param_colors["TurnTurningAcc1"] = QColor("#facc15");    // Light Yellow
  param_colors["TurnTurningAcc2"] = QColor("#fde047");    // Pale Yellow
  param_colors["TurnTurningAcc3"] = QColor("#ca8a04");    // Dark Yellow
  param_colors["TurnTurningAcc4"] = QColor("#a16207");    // Brown Yellow
  param_colors["TurnLeavingAcc"] = QColor("#2dd4bf");     // Teal

  refreshHistory();
}

void AutoTunerHistoryPanel::showEvent(QShowEvent *event) {
  refreshHistory();
  QFrame::showEvent(event);
}

void AutoTunerHistoryPanel::refreshHistory() {
  // Clear parameter list layout
  QLayoutItem *child;
  while ((child = param_list_layout->takeAt(0)) != nullptr) {
    if (child->widget()) delete child->widget();
    delete child;
  }
  param_labels.clear();

  QString raw = QString::fromStdString(Params().get("CarrotLearningHistory"));
  if (raw.isEmpty()) {
    if (graph_widget) {
      graph_widget->setData(QList<QString>(), QMap<QString, QList<double>>(), QMap<QString, QColor>());
    }
    return;
  }

  QJsonArray arr = QJsonDocument::fromJson(raw.toUtf8()).array();

  // 차트 가독성을 위해 최대 10개 시점만 사용 (과거 → 최신 순)
  int chart_limit = 30;
  int n_points = std::min((int)arr.size(), chart_limit);
  QList<QString> timestamps;
  QList<QJsonObject> entries;
  for (int i = n_points - 1; i >= 0; i--) {
    QJsonObject item = arr[i].toObject();
    timestamps.append(item["timestamp"].toString());
    entries.append(item);
  }

  // 1. 타임라인에 등장하는 모든 파라미터 수집 + 각 파라미터의 그룹 라벨 기억
  //    (가속/조향/곡선/거리/주행 등으로 좌측 리스트를 묶기 위함)
  QSet<QString> param_set;
  group_params.clear();
  for (const auto& entry : entries) {
    QJsonObject changes = entry["changes"].toObject();
    for (const QString& group : changes.keys()) {
      QJsonObject g_items = changes[group].toObject();
      for (const QString& key : g_items.keys()) {
        param_set.insert(key);
        if (!group_params[group].contains(key)) {
          group_params[group].append(key);
        }
      }
    }
  }

  // 그룹 표시 순서: 가속 → 조향 → 곡선 → 거리 → 주행 → (초기값 프로파일 등 기타)
  auto groupRank = [](const QString &g) -> int {
    if (g.contains("Acceleration") && !g.contains("Profile")) return 0;
    if (g.contains("Steering")) return 1;
    if (g.contains("Curve")) return 2;
    if (g.contains("Following Distance") && !g.contains("Profile")) return 3;
    if (g.contains("Driving")) return 4;
    return 10;  // 초기값 프로파일 및 기타 그룹은 뒤에
  };
  group_order = group_params.keys();
  std::stable_sort(group_order.begin(), group_order.end(),
                   [&](const QString &a, const QString &b) {
                     int ra = groupRank(a), rb = groupRank(b);
                     if (ra != rb) return ra < rb;
                     return a.compare(b, Qt::CaseInsensitive) < 0;
                   });

  // 2. 파라미터별 타임라인 값 보간 (이전 값 유지, 미래에서 초기값 추출)
  QMap<QString, QList<double>> param_histories;
  QMap<QString, double> last_values;

  for (int t = 0; t < n_points; t++) {
    QJsonObject changes = entries[t]["changes"].toObject();
    QMap<QString, double> current_changes;
    for (const QString& group : changes.keys()) {
      QJsonObject g_items = changes[group].toObject();
      for (const QString& key : g_items.keys()) {
        current_changes[key] = g_items[key].toObject()["recommended"].toDouble();
      }
    }

    for (const QString& param : param_set) {
      if (current_changes.contains(param)) {
        double val = current_changes[param];
        last_values[param] = val;
        param_histories[param].append(val);
      } else {
        if (last_values.contains(param)) {
          param_histories[param].append(last_values[param]);
        } else {
          // 미래 시점에서 최초 등장하는 'current' 값을 초기값으로 사용
          double initial_val = 0.0;
          for (int future_t = t; future_t < n_points; future_t++) {
            QJsonObject f_changes = entries[future_t]["changes"].toObject();
            bool found = false;
            for (const QString& group : f_changes.keys()) {
              QJsonObject fg_items = f_changes[group].toObject();
              if (fg_items.contains(param)) {
                initial_val = fg_items[param].toObject()["current"].toDouble();
                found = true;
                break;
              }
            }
            if (found) break;
          }
          last_values[param] = initial_val;
          param_histories[param].append(initial_val);
        }
      }
    }
  }

  // 미등록 파라미터에 동적 색상 배정
  QList<QColor> palette = {
    QColor("#3b82f6"), QColor("#10b981"), QColor("#f59e0b"), QColor("#8b5cf6"),
    QColor("#ec4899"), QColor("#06b6d4"), QColor("#84cc16"), QColor("#f43f5e"),
    QColor("#14b8a6"), QColor("#a855f7")
  };
  int color_idx = 0;
  for (const QString &param : param_set) {
    if (!param_colors.contains(param)) {
      param_colors[param] = palette[color_idx % palette.size()];
      color_idx++;
    }
  }

  // 각 그룹 내 파라미터는 알파벳순 정렬
  for (const QString &group : group_order) {
    group_params[group].sort(Qt::CaseInsensitive);
  }

  // 더 이상 존재하지 않는 그룹은 collapsed 상태에서 제거 (이력 변경 대응)
  for (const QString &g : collapsed_groups.values()) {
    if (!group_params.contains(g)) collapsed_groups.remove(g);
  }

  // 좌측 리스트를 그룹 단위로 렌더링
  rebuildParamList();

  if (graph_widget) {
    graph_widget->setData(timestamps, param_histories, param_colors);
  }

  if (!param_set.contains(selected_param)) {
    selected_param = "";
    if (graph_widget) graph_widget->setSelectedParam("");
  }
  applyHiddenParams();
  updateLabelColors();
}

// 좌측 파라미터 리스트를 그룹 헤더 + (펼쳐진 경우) 소속 파라미터 버튼으로 다시 그린다.
// 그룹 헤더를 누르면 toggleGroup()으로 접기/펴기 + 그래프 표시가 토글된다.
void AutoTunerHistoryPanel::rebuildParamList() {
  // 기존 항목 제거
  QLayoutItem *child;
  while ((child = param_list_layout->takeAt(0)) != nullptr) {
    if (child->widget()) delete child->widget();
    delete child;
  }
  param_labels.clear();

  // 그룹 라벨에서 표시용 짧은 이름 추출: "거리 (Following Distance)" → "Following Distance"
  // 단, 이력 데이터의 그룹 키(=정렬/매칭 기준)는 그대로 두고 표시명만 바꾼다.
  auto groupShortName = [](const QString &group) -> QString {
    QString name;
    int open = group.indexOf('(');
    int close = group.lastIndexOf(')');
    if (open >= 0 && close > open) {
      name = group.mid(open + 1, close - open - 1).trimmed();
    } else {
      name = group.trimmed();
    }
    // 표시명만 "Following Distance" → "Following Gap" 으로 치환
    if (name == "Following Distance") name = "Following Gap";
    return name;
  };

  for (const QString &group : group_order) {
    bool collapsed = collapsed_groups.contains(group);

    // ── 그룹 헤더 (클릭 시 접기/펴기) ──
    QPushButton *header_btn = new QPushButton();
    // 헤더 바는 파라미터 항목(#252525)과 확실히 구분되도록 더 밝은 회색으로
    header_btn->setStyleSheet("text-align: left; padding: 0px 12px; border-radius: 10px; background-color: #5a5f6b; color: white; font-size: 26px;");
    header_btn->setFixedHeight(60);

    QHBoxLayout *header_layout = new QHBoxLayout(header_btn);
    header_layout->setContentsMargins(12, 0, 12, 0);
    header_layout->setSpacing(12);

    // 접힘/펼침 표시 화살표
    QLabel *arrow = new QLabel(collapsed ? "▸" : "▾");
    arrow->setStyleSheet(QString("color: %1; font-size: 26px; font-weight: bold; background: transparent;")
                             .arg(collapsed ? "#777777" : "#ffffff"));
    header_layout->addWidget(arrow);

    QLabel *header_lbl = new QLabel(groupShortName(group));
    header_lbl->setStyleSheet(QString("color: %1; font-size: 26px; font-weight: bold; background: transparent;")
                                  .arg(collapsed ? "#777777" : "#ffffff"));
    header_layout->addWidget(header_lbl, 1);

    connect(header_btn, &QPushButton::clicked, this, [=]() { toggleGroup(group); });
    param_list_layout->addWidget(header_btn);

    // ── 소속 파라미터들 (접혀있으면 그리지 않음) ──
    if (collapsed) continue;
    for (const QString &param : group_params[group]) {
      QColor color = param_colors.value(param, QColor(Qt::white));

      QPushButton *btn = new QPushButton();
      btn->setStyleSheet("text-align: left; padding: 0px 15px; border-radius: 10px; background-color: #252525; color: white; font-size: 28px;");
      btn->setFixedHeight(55);

      QHBoxLayout *btn_layout = new QHBoxLayout(btn);
      btn_layout->setContentsMargins(22, 0, 10, 0);  // 그룹 소속 표시를 위해 약간 들여쓰기
      btn_layout->setSpacing(15);

      QLabel *dot = new QLabel();
      dot->setFixedSize(20, 20);
      dot->setStyleSheet(QString("background-color: %1; border-radius: 10px;").arg(color.name()));
      btn_layout->addWidget(dot);

      QLabel *lbl = new QLabel(param);
      lbl->setStyleSheet("color: white; font-size: 28px; font-weight: bold; background: transparent;");
      btn_layout->addWidget(lbl, 1);
      param_labels[param] = lbl;

      connect(btn, &QPushButton::clicked, this, [=]() {
        if (graph_widget) graph_widget->setSelectedParam(param);
        selected_param = param;
        updateLabelColors();
      });

      param_list_layout->addWidget(btn);
    }
  }
  param_list_layout->addStretch();
}

// 접혀있는 그룹들의 소속 파라미터를 모아 그래프에서 숨긴다.
void AutoTunerHistoryPanel::applyHiddenParams() {
  QSet<QString> hidden;
  for (const QString &group : collapsed_groups.values()) {
    for (const QString &param : group_params.value(group)) {
      hidden.insert(param);
    }
  }
  if (graph_widget) graph_widget->setHiddenParams(hidden);
}

// 그룹 헤더 클릭 핸들러: 접기/펴기 상태를 토글하고 리스트·그래프를 갱신한다.
void AutoTunerHistoryPanel::toggleGroup(const QString &group) {
  if (collapsed_groups.contains(group)) {
    collapsed_groups.remove(group);
  } else {
    collapsed_groups.insert(group);
    // 접는 그룹에 현재 선택된 파라미터가 있으면 선택 해제 (숨겨지므로)
    if (!selected_param.isEmpty() && group_params.value(group).contains(selected_param)) {
      selected_param = "";
      if (graph_widget) graph_widget->setSelectedParam("");
    }
  }
  rebuildParamList();
  applyHiddenParams();
  updateLabelColors();
}

void AutoTunerHistoryPanel::clearAll() {
  if (ConfirmationDialog::confirm("모든 이력을 삭제하고 파라미터를 공장 기본값으로 되돌리시겠습니까?", this)) {
    Params params;
    for (const auto& [key, val] : kAutoTunerDefaults) {
      params.put(key, val);
    }
    params.remove("CarrotLearningHistory");
    params.putBool("CarrotLearningClear", true);  // 누적 학습 데이터도 초기화 (python 측 처리)
    refreshHistory();
  }
}

void AutoTunerHistoryPanel::updateLabelColors() {
  for (auto k : param_labels.keys()) {
    if (!param_labels[k]) continue;
    if (selected_param.isEmpty()) {
      param_labels[k]->setStyleSheet("color: white; font-size: 28px; font-weight: bold; background: transparent;");
    } else if (k == selected_param) {
      param_labels[k]->setStyleSheet("color: red; font-size: 28px; font-weight: bold; background: transparent;");
    } else {
      param_labels[k]->setStyleSheet("color: #777777; font-size: 28px; font-weight: bold; background: transparent;");
    }
  }
}

// AutoTunerHistoryDialog
AutoTunerHistoryDialog::AutoTunerHistoryDialog(QWidget *parent) : QDialogBase(parent) {
  QFrame *container = new QFrame(this);
  container->setStyleSheet("QFrame { background-color: #1B1B1B; border-radius: 20px; }");

  QVBoxLayout *main_layout = new QVBoxLayout(container);
  main_layout->setContentsMargins(20, 20, 20, 20);
  main_layout->setSpacing(20);

  AutoTunerHistoryPanel *panel = new AutoTunerHistoryPanel(this);
  main_layout->addWidget(panel, 1);

  QVBoxLayout *outer_layout = new QVBoxLayout(this);
  outer_layout->setContentsMargins(30, 30, 30, 30);
  outer_layout->addWidget(container);
}

void AutoTunerHistoryDialog::showEvent(QShowEvent *event) {
  // 구형 QDialogBase는 신형 DialogBase와 달리 exec()에서 전체화면 처리를 하지 않음
  setMainWindow(this);
  QDialog::showEvent(event);
}

// AutoTunerCardListDialog
AutoTunerCardListDialog::AutoTunerCardListDialog(QWidget *parent) : QDialogBase(parent) {
  QFrame *container = new QFrame(this);
  container->setStyleSheet("QFrame { background-color: #1B1B1B; border-radius: 20px; }");

  QVBoxLayout *main_layout = new QVBoxLayout(container);
  main_layout->setContentsMargins(50, 50, 50, 50);
  main_layout->setSpacing(30);

  // Header layout: Title and Close button
  QHBoxLayout *header_layout = new QHBoxLayout();
  QLabel *title = new QLabel("Tuning History Card List", this);
  title->setStyleSheet("font-size: 60px; font-weight: bold; color: white;");
  header_layout->addWidget(title);
  header_layout->addStretch();

  QPushButton *close_btn = new QPushButton("Close", this);
  close_btn->setFixedSize(250, 100);
  close_btn->setStyleSheet("background-color: #bb3333; font-size: 40px; border-radius: 10px; color: white;");
  connect(close_btn, &QPushButton::clicked, this, &AutoTunerCardListDialog::reject);
  header_layout->addWidget(close_btn);
  main_layout->addLayout(header_layout);

  // Scroll Area
  QScrollArea *scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  // 터치로 잡기 쉽도록 스크롤바를 굵게(핸들 최소폭 90px) 키움 — 기본 Qt 스크롤바는
  // 차량 터치스크린에서 손가락으로 집기엔 너무 얇았음 (사용자 피드백).
  scroll->setStyleSheet(R"(
    QScrollArea { background: transparent; }
    QWidget { background: transparent; }
    QScrollBar:vertical {
      background: #2b2b2b;
      width: 46px;
      margin: 0px;
      border-radius: 23px;
    }
    QScrollBar::handle:vertical {
      background: #6b7280;
      min-height: 90px;
      border-radius: 20px;
    }
    QScrollBar::handle:vertical:pressed {
      background: #9ca3af;
    }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
      height: 0px;
      background: none;
    }
    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
      background: none;
    }
  )");
  QScroller::grabGesture(scroll->viewport(), QScroller::LeftMouseButtonGesture);

  QWidget *scroll_widget = new QWidget();
  list_layout = new QVBoxLayout(scroll_widget);
  list_layout->setContentsMargins(0, 0, 0, 0);
  list_layout->setSpacing(10);
  scroll->setWidget(scroll_widget);
  main_layout->addWidget(scroll, 1);

  QVBoxLayout *outer_layout = new QVBoxLayout(this);
  outer_layout->setContentsMargins(100, 100, 100, 100);
  outer_layout->addWidget(container);

  refreshHistory();
}

void AutoTunerCardListDialog::showEvent(QShowEvent *event) {
  // 구형 QDialogBase는 신형 DialogBase와 달리 exec()에서 전체화면 처리를 하지 않음
  setMainWindow(this);
  QDialog::showEvent(event);
}

void AutoTunerCardListDialog::refreshHistory() {
  // Clear list layout
  QLayoutItem *child;
  while ((child = list_layout->takeAt(0)) != nullptr) {
    if (child->widget()) delete child->widget();
    delete child;
  }

  QString raw = QString::fromStdString(Params().get("CarrotLearningHistory"));
  if (raw.isEmpty()) {
    QLabel *lbl = new QLabel("No historical data to display", this);
    lbl->setStyleSheet("font-size: 45px; color: #888888;");
    lbl->setAlignment(Qt::AlignCenter);
    list_layout->addWidget(lbl);
    list_layout->addStretch();
    return;
  }

  QJsonArray arr = QJsonDocument::fromJson(raw.toUtf8()).array();

  for (int i = 0; i < arr.size(); i++) {
    QJsonObject item = arr[i].toObject();
    QString id = item["id"].toString();
    QString time_str = item["timestamp"].toString();
    QJsonObject changes = item["changes"].toObject();

    QFrame *row = new QFrame();
    row->setStyleSheet("background-color: #2b2b2b; border-radius: 15px; padding: 5px 25px;");
    QHBoxLayout *row_layout = new QHBoxLayout(row);
    row_layout->setContentsMargins(25, 5, 25, 5);

    QString text = QString("<span style='font-size: 35px; color: #aaaaaa;'>[%1 Applied]</span><br>").arg(time_str);

    for (const QString& group : changes.keys()) {
      QJsonObject g_items = changes[group].toObject();
      QString short_group = group.split(" ").first();

      for (const QString& key : g_items.keys()) {
        QJsonObject info = g_items[key].toObject();

        // float 파라미터(PathOffset 등) 표시 처리
        bool is_float = info["is_float"].toBool(false);
        QString cur_str, rec_str;
        if (is_float) {
          cur_str = QString::number(info["current"].toDouble(), 'f', 3);
          rec_str = QString::number(info["recommended"].toDouble(), 'f', 3);
        } else {
          cur_str = QString::number(info["current"].toInt());
          rec_str = QString::number(info["recommended"].toInt());
        }

        text += QString("<span style='font-size: 40px; color: white;'>"
                        "<span style='color:#aaaaaa;'>[%1]</span> <b>%2</b> "
                        "<span style='font-size:35px; color:#bbbbbb;'>[%3]</span> &nbsp;:&nbsp; "
                        "%4 ➔ <span style='color:#00ff00; font-weight:bold;'>%5</span></span><br>")
                  .arg(short_group)
                  .arg(key)
                  .arg(info["band_kph"].toString())
                  .arg(cur_str)
                  .arg(rec_str);
      }
    }

    QLabel *lbl = new QLabel(text);
    lbl->setWordWrap(true);
    row_layout->addWidget(lbl, 1);

    // 정합성 보호를 위해 최신 항목만 Restore/Delete 가능
    bool is_latest = (i == 0);

    QPushButton *btn_restore = new QPushButton("Restore");
    if (is_latest) {
      btn_restore->setStyleSheet("background-color: #178644; font-size: 40px; padding: 20px; border-radius: 10px; color: white; font-weight: bold;");
    } else {
      btn_restore->setStyleSheet("background-color: #333333; font-size: 40px; padding: 20px; border-radius: 10px; color: #666666;");
    }
    btn_restore->setEnabled(is_latest);
    btn_restore->setFixedSize(220, 110);
    connect(btn_restore, &QPushButton::clicked, this, [=]() { restoreItem(id); });
    row_layout->addWidget(btn_restore);

    QPushButton *btn_del = new QPushButton("Delete");
    if (is_latest) {
      btn_del->setStyleSheet("background-color: #555555; font-size: 40px; padding: 20px; border-radius: 10px; color: white; font-weight: bold;");
    } else {
      btn_del->setStyleSheet("background-color: #333333; font-size: 40px; padding: 20px; border-radius: 10px; color: #666666;");
    }
    btn_del->setEnabled(is_latest);
    btn_del->setFixedSize(220, 110);
    connect(btn_del, &QPushButton::clicked, this, [=]() { deleteItem(id); });
    row_layout->addWidget(btn_del);

    list_layout->addWidget(row);
  }
  list_layout->addStretch();
}

void AutoTunerCardListDialog::restoreItem(const QString& id) {
  if (ConfirmationDialog::confirm("파라미터를 이 시점 이전 상태로 복원하시겠습니까?", this)) {
    QString raw = QString::fromStdString(Params().get("CarrotLearningHistory"));
    QJsonArray arr = QJsonDocument::fromJson(raw.toUtf8()).array();
    QJsonArray new_arr;

    for (int i = 0; i < arr.size(); i++) {
      QJsonObject entry = arr[i].toObject();
      if (entry["id"].toString() == id) {
        QJsonObject changes = entry["changes"].toObject();
        for (const QString& group : changes.keys()) {
          QJsonObject g_items = changes[group].toObject();
          for (const QString& key : g_items.keys()) {
            QJsonObject info = g_items[key].toObject();
            // 적용 전 'current' 값으로 원복 (ntune / float / int 구분)
            if (info["ntune"].toString() == "torque") {
              writeNtuneTorqueValueS(key, info["current"].toDouble());
            } else if (info["ntune"].toString() == "common") {
              writeNtuneCommonValueS(key, info["current"].toDouble());
            } else if (info["is_float"].toBool(false)) {
              double prev_val = info["current"].toDouble();
              Params().put(key.toStdString(), QString::number(prev_val, 'f', 3).toStdString());
            } else {
              int prev_val = info["current"].toInt();
              Params().put(key.toStdString(), std::to_string(prev_val));
            }
          }
        }
      } else {
        new_arr.append(entry);
      }
    }

    if (new_arr.isEmpty()) {
      Params().remove("CarrotLearningHistory");
    } else {
      Params().put("CarrotLearningHistory", QJsonDocument(new_arr).toJson(QJsonDocument::Compact).toStdString());
    }
    refreshHistory();
    ConfirmationDialog::alert("이전 값으로 복원되었습니다.", this);
  }
}

void AutoTunerCardListDialog::deleteItem(const QString& id) {
  if (ConfirmationDialog::confirm("이 항목을 삭제하시겠습니까?", this)) {
    QString raw = QString::fromStdString(Params().get("CarrotLearningHistory"));
    QJsonArray arr = QJsonDocument::fromJson(raw.toUtf8()).array();
    QJsonArray new_arr;
    for (int i = 0; i < arr.size(); i++) {
      if (arr[i].toObject()["id"].toString() != id) {
        new_arr.append(arr[i]);
      }
    }
    if (new_arr.isEmpty()) {
      Params().remove("CarrotLearningHistory");
    } else {
      Params().put("CarrotLearningHistory", QJsonDocument(new_arr).toJson(QJsonDocument::Compact).toStdString());
    }
    refreshHistory();
  }
}
// ─────────────────────────────────────────────────────────────────────────

TogglesPanel::TogglesPanel(SettingsWindow *parent) : ListWidget(parent) {
  std::vector<std::tuple<QString, QString, QString, QString>> toggle_defs{
    {
      "OpenpilotEnabledToggle",
      "Enable openpilot",
      "Use the openpilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off.",
      "../assets/offroad/icon_openpilot.png",
    },
    {
      "IsLdwEnabled",
      "Enable Lane Departure Warnings",
      "Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31 mph (50 km/h).",
      "../assets/offroad/icon_warning.png",
    },
    {
      "IsRHD",
      "Enable Right-Hand Drive",
      "Allow openpilot to obey left-hand traffic conventions and perform driver monitoring on right driver seat.",
      "../assets/offroad/icon_openpilot_mirrored.png",
    },
    {
      "IsMetric",
      "Use Metric System",
      "Display speed in km/h instead of mph.",
      "../assets/offroad/icon_metric.png",
    },
    {
      "RecordFront",
      "Record and Upload Driver Camera",
      "Upload data from the driver facing camera and help improve the driver monitoring algorithm.",
      "../assets/offroad/icon_monitoring.png",
    },
    {
      "ExperimentalMode",
      "Experimental mode",
      "알파 수준 기능을 활성화합니다. (실험 기능/안전 경고를 확인하세요)",
      "../assets/img_experimental_white.svg",
    },
    {
      "ExperimentalLongitudinalEnabled",
      "Experimental openpilot longitudinal control",
      "<b>WARNING: openpilot longitudinal control is experimental for this car and will disable AEB.</b><br>\
          openpilot defaults to the car's built-in ACC instead of openpilot's longitudinal control on this car. Enable this to switch to openpilot longitudinal control.",
      "../assets/offroad/icon_speed_limit.png",
    },
#ifdef ENABLE_MAPS
    {
      "NavSettingTime24h",
      "Show ETA in 24h format",
      "Use 24h format instead of am/pm",
      "../assets/offroad/icon_metric.png",
    },
#endif
  };

  for (auto &[param, title, desc, icon] : toggle_defs) {
    auto toggle = new ParamControl(param, title, desc, icon, this);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    addItem(toggle);
    toggles[param.toStdString()] = toggle;
  }

  toggles["ExperimentalMode"]->setActiveIcon("../assets/img_experimental.svg");
  toggles["ExperimentalMode"]->setConfirmation(true, true);
  toggles["ExperimentalLongitudinalEnabled"]->setConfirmation(true, false);

  connect(toggles["ExperimentalLongitudinalEnabled"], &ToggleControl::toggleFlipped, [=]() {
    updateToggles();
  });
}

void TogglesPanel::expandToggleDescription(const QString &param) {
  toggles[param.toStdString()]->showDescription();
}

void TogglesPanel::showEvent(QShowEvent *event) {
  updateToggles();
}

void TogglesPanel::updateToggles() {
  auto e2e_toggle = toggles["ExperimentalMode"];
  auto op_long_toggle = toggles["ExperimentalLongitudinalEnabled"];
  const QString e2e_description = tr("\
    openpilot defaults to driving in <b>chill mode</b>.\
    Experimental mode enables <b>alpha-level features</b> that aren't ready for chill mode. \
    Experimental features are listed below:\
    <br> \
    <h4>🌮 End-to-End Longitudinal Control 🌮</h4> \
    Let the driving model control the gas and brakes. openpilot will drive as it thinks a human would, including stopping for red lights and stop signs.");

  auto cp_bytes = params.get("CarParamsPersistent");
  if (!cp_bytes.empty()) {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(cp_bytes.data(), cp_bytes.size()));
    cereal::CarParams::Reader CP = cmsg.getRoot<cereal::CarParams>();

    if (!CP.getExperimentalLongitudinalAvailable()) {
      params.remove("ExperimentalLongitudinalEnabled");
    }
    op_long_toggle->setVisible(CP.getExperimentalLongitudinalAvailable());

    const bool op_long = CP.getOpenpilotLongitudinalControl() && !CP.getExperimentalLongitudinalAvailable();
    const bool exp_long_enabled = CP.getExperimentalLongitudinalAvailable() && params.getBool("ExperimentalLongitudinalEnabled");
    if (op_long || exp_long_enabled) {
      e2e_toggle->setEnabled(true);
      e2e_toggle->setDescription(e2e_description);
    } else {
      e2e_toggle->setEnabled(false);
      params.remove("ExperimentalMode");
      const QString no_long = "openpilot longitudinal control is not currently available for this car.";
      const QString exp_long = "Enable experimental longitudinal control to enable this.";
      e2e_toggle->setDescription("<b>" + (CP.getExperimentalLongitudinalAvailable() ? exp_long : no_long) + "</b><br><br>" + e2e_description);
    }
    e2e_toggle->refresh();
  } else {
    e2e_toggle->setDescription(e2e_description);
    op_long_toggle->setVisible(true);
  }
}

DevicePanel::DevicePanel(SettingsWindow *parent) : ListWidget(parent) {
  setSpacing(50);
  addItem(new LabelControl("Dongle ID", getDongleId().value_or("N/A")));
  addItem(new LabelControl("Serial", params.get("HardwareSerial").c_str()));

  QHBoxLayout *reset_layout = new QHBoxLayout();
  reset_layout->setSpacing(30);

  QPushButton *restart_openpilot_btn = new QPushButton("Soft restart");
  restart_openpilot_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  reset_layout->addWidget(restart_openpilot_btn);
  QObject::connect(restart_openpilot_btn, &QPushButton::released, [=]() {
    emit closeSettings();
    QTimer::singleShot(1000, []() {
      Params().putBool("SoftRestartTriggered", true);
    });
  });

  QPushButton *reset_calib_btn = new QPushButton("Reset Calibration");
  reset_calib_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  reset_layout->addWidget(reset_calib_btn);
  QObject::connect(reset_calib_btn, &QPushButton::released, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to reset calibration and live params?", this)) {
      Params().remove("CalibrationParams");
      Params().remove("LiveParameters");
      emit closeSettings();
      QTimer::singleShot(1000, []() {
        Params().putBool("SoftRestartTriggered", true);
      });
    }
  });

  addItem(reset_layout);

  auto dcamBtn = new ButtonControl("Driver Camera", "PREVIEW",
                                   "Preview the driver facing camera to help optimize device mounting position for best driver monitoring experience. (vehicle must be off)");
  connect(dcamBtn, &ButtonControl::clicked, [=]() { emit showDriverView(); });
  addItem(dcamBtn);

  auto resetCalibBtn = new ButtonControl("Reset Calibration", "RESET", " ");
  connect(resetCalibBtn, &ButtonControl::showDescription, this, &DevicePanel::updateCalibDescription);
  connect(resetCalibBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm("Are you sure you want to reset calibration?", this)) {
      params.remove("CalibrationParams");
    }
  });
  addItem(resetCalibBtn);

  if (!params.getBool("Passive")) {
    auto retrainingBtn = new ButtonControl("Review Training Guide", "REVIEW", "Review the rules, features, and limitations of openpilot");
    connect(retrainingBtn, &ButtonControl::clicked, [=]() {
      if (ConfirmationDialog::confirm("Are you sure you want to review the training guide?", this)) {
        emit reviewTrainingGuide();
      }
    });
    addItem(retrainingBtn);
  }

  if (Hardware::TICI()) {
    auto regulatoryBtn = new ButtonControl("Regulatory", "VIEW", "");
    connect(regulatoryBtn, &ButtonControl::clicked, [=]() {
      const std::string txt = util::read_file("../assets/offroad/fcc.html");
      ConfirmationDialog::rich(QString::fromStdString(txt), this);
    });
    addItem(regulatoryBtn);
  }

  QHBoxLayout *power_layout = new QHBoxLayout();
  power_layout->setSpacing(30);

  QPushButton *rebuild_btn = new QPushButton("Rebuild");
  rebuild_btn->setObjectName("rebuild_btn");
  power_layout->addWidget(rebuild_btn);
  QObject::connect(rebuild_btn, &QPushButton::clicked, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to rebuild?", this)) {
      std::system("cd /data/openpilot && scons -c");
      std::system("rm /data/openpilot/.sconsign.dblite");
      std::system("rm /data/openpilot/prebuilt");
      std::system("rm -rf /tmp/scons_cache");
      if (Hardware::TICI())
        std::system("sudo reboot");
      else
        std::system("reboot");
    }
  });

  QPushButton *reboot_btn = new QPushButton("Reboot");
  reboot_btn->setObjectName("reboot_btn");
  power_layout->addWidget(reboot_btn);
  QObject::connect(reboot_btn, &QPushButton::clicked, this, &DevicePanel::reboot);

  QPushButton *poweroff_btn = new QPushButton("Power Off");
  poweroff_btn->setObjectName("poweroff_btn");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::clicked, this, &DevicePanel::poweroff);

  if (Hardware::TICI()) {
    connect(uiState(), &UIState::offroadTransition, poweroff_btn, &QPushButton::setVisible);
  }

  setStyleSheet(R"(
    #reboot_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #reboot_btn:pressed { background-color: #4a4a4a; }
    #rebuild_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #rebuild_btn:pressed { background-color: #4a4a4a; }
    #poweroff_btn { height: 120px; border-radius: 15px; background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
  )");
  addItem(power_layout);
}

void DevicePanel::updateCalibDescription() {
  QString desc =
      "openpilot requires the device to be mounted within 4° left or right and "
      "within 5° up or 8° down. openpilot is continuously calibrating, resetting is rarely required.";
  std::string calib_bytes = Params().get("CalibrationParams");
  if (!calib_bytes.empty()) {
    try {
      AlignedBuffer aligned_buf;
      capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
      auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
      if (calib.getCalStatus() != 0) {
        double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
        double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
        desc += QString(" Your device is pointed %1° %2 and %3° %4.")
                    .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? "down" : "up",
                         QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? "left" : "right");
      }
    } catch (kj::Exception) {
      qInfo() << "invalid CalibrationParams";
    }
  }
  qobject_cast<ButtonControl *>(sender())->setDescription(desc);
}

void DevicePanel::reboot() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm("Are you sure you want to reboot?", this)) {
      if (!uiState()->engaged()) {
        Params().putBool("DoReboot", true);
      }
    }
  } else {
    ConfirmationDialog::alert("Disengage to Reboot", this);
  }
}

void DevicePanel::poweroff() {
  if (!uiState()->engaged()) {
    if (ConfirmationDialog::confirm("Are you sure you want to power off?", this)) {
      if (!uiState()->engaged()) {
        Params().putBool("DoShutdown", true);
      }
    }
  } else {
    ConfirmationDialog::alert("Disengage to Power Off", this);
  }
}

SoftwarePanel::SoftwarePanel(QWidget* parent) : ListWidget(parent) {
  gitBranchLbl = new LabelControl("Git Branch");
  gitCommitLbl = new LabelControl("Git Commit");
  osVersionLbl = new LabelControl("OS Version");
  versionLbl = new LabelControl("Version", "", QString::fromStdString(params.get("ReleaseNotes")).trimmed());
  lastUpdateLbl = new LabelControl("Last Update Check", "", "The last time openpilot successfully checked for an update. The updater only runs while the car is off.");
  updateBtn = new ButtonControl("Check for Update", "");
  connect(updateBtn, &ButtonControl::clicked, [=]() {
    if (params.getBool("IsOffroad")) {
      fs_watch->addPath(QString::fromStdString(params.getParamPath("LastUpdateTime")));
      fs_watch->addPath(QString::fromStdString(params.getParamPath("UpdateFailedCount")));
      updateBtn->setText("CHECKING");
      updateBtn->setEnabled(false);
    }
    std::system("pkill -1 -f selfdrive.updated");
  });

  auto uninstallBtn = new ButtonControl("Uninstall " + getBrand(), "UNINSTALL");
  connect(uninstallBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm("Are you sure you want to uninstall?", this)) {
      params.putBool("DoUninstall", true);
    }
  });
  connect(uiState(), &UIState::offroadTransition, uninstallBtn, &QPushButton::setEnabled);

  QWidget *widgets[] = {versionLbl, lastUpdateLbl, updateBtn, gitBranchLbl, gitCommitLbl, osVersionLbl, uninstallBtn};
  for (QWidget* w : widgets) {
    addItem(w);
  }

  fs_watch = new QFileSystemWatcher(this);
  QObject::connect(fs_watch, &QFileSystemWatcher::fileChanged, [=](const QString path) {
    if (path.contains("UpdateFailedCount") && std::atoi(params.get("UpdateFailedCount").c_str()) > 0) {
      lastUpdateLbl->setText("failed to fetch update");
      updateBtn->setText("CHECK");
      updateBtn->setEnabled(true);
    } else if (path.contains("LastUpdateTime")) {
      updateLabels();
    }
  });
}

void SoftwarePanel::showEvent(QShowEvent *event) {
  updateLabels();
}

void SoftwarePanel::updateLabels() {
  QString lastUpdate = "";
  auto tm = params.get("LastUpdateTime");
  if (!tm.empty()) {
    lastUpdate = timeAgo(QDateTime::fromString(QString::fromStdString(tm + "Z"), Qt::ISODate));
  }

  versionLbl->setText(getBrandVersion());
  lastUpdateLbl->setText(lastUpdate);
  updateBtn->setText("CHECK");
  updateBtn->setEnabled(true);
  gitBranchLbl->setText(QString::fromStdString(params.get("GitBranch")));
  gitCommitLbl->setText(QString::fromStdString(params.get("GitCommit")).left(10));
  osVersionLbl->setText(QString::fromStdString(Hardware::get_os_version()).trimmed());
}

C2NetworkPanel::C2NetworkPanel(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 0, 50, 0);

  ListWidget *list = new ListWidget();
  list->setSpacing(30);
#ifdef QCOM
  auto wifiBtn = new ButtonControl("Wi-Fi Settings", "OPEN");
  QObject::connect(wifiBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_wifi(); });
  list->addItem(wifiBtn);

  auto tetheringBtn = new ButtonControl("Tethering Settings", "OPEN");
  QObject::connect(tetheringBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_tethering(); });
  list->addItem(tetheringBtn);
#endif
  ipaddress = new LabelControl("IP Address", "");
  list->addItem(ipaddress);

  list->addItem(new SshToggle());
  list->addItem(new SshControl());
  layout->addWidget(list);
  layout->addStretch(1);
}

void C2NetworkPanel::showEvent(QShowEvent *event) {
  ipaddress->setText(getIPAddress());
}

QString C2NetworkPanel::getIPAddress() {
  std::string result = util::check_output("ifconfig wlan0");
  if (result.empty()) return "";

  const std::string inetaddrr = "inet addr:";
  std::string::size_type begin = result.find(inetaddrr);
  if (begin == std::string::npos) return "";

  begin += inetaddrr.length();
  std::string::size_type end = result.find(' ', begin);
  if (end == std::string::npos) return "";

  return result.substr(begin, end - begin).c_str();
}

QWidget *network_panel(QWidget *parent) {
#ifdef QCOM
  return new C2NetworkPanel(parent);
#else
  return new Networking(parent);
#endif
}

static QStringList get_list(const char* path)
{
  QStringList stringList;
  QFile textFile(path);
  if(textFile.open(QIODevice::ReadOnly))
  {
      QTextStream textStream(&textFile);
      while (true)
      {
        QString line = textStream.readLine();
        if (line.isNull())
            break;
        else
            stringList.append(line);
      }
  }
  return stringList;
}

void SettingsWindow::showEvent(QShowEvent *event) {
  setCurrentPanel(0);
}

void SettingsWindow::setCurrentPanel(int index, const QString &param) {
  panel_widget->setCurrentIndex(index);
  nav_btns->buttons()[index]->setChecked(true);
  if (!param.isEmpty()) {
    emit expandToggleDescription(param);
  }
}

SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {

  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  sidebar_layout->setMargin(0);
  panel_widget = new QStackedWidget();
  panel_widget->setStyleSheet(R"(
    border-radius: 30px;
    background-color: #292929;
  )");

  QPushButton *close_btn = new QPushButton("← Back");
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 50px;
      font-weight: bold;
      margin: 0px;
      padding: 15px;
      border-width: 0;
      border-radius: 30px;
      color: #dddddd;
      background-color: #444444;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(300, 110);
  sidebar_layout->addSpacing(10);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignRight);
  sidebar_layout->addSpacing(10);
  QObject::connect(close_btn, &QPushButton::clicked, this, &SettingsWindow::closeSettings);

  DevicePanel *device = new DevicePanel(this);
  QObject::connect(device, &DevicePanel::reviewTrainingGuide, this, &SettingsWindow::reviewTrainingGuide);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);
  QObject::connect(device, &DevicePanel::closeSettings, this, &SettingsWindow::closeSettings);

  TogglesPanel *toggles = new TogglesPanel(this);
  QObject::connect(this, &SettingsWindow::expandToggleDescription, toggles, &TogglesPanel::expandToggleDescription);

  QList<QPair<QString, QWidget *>> panels = {
    {"Device", device},
    {"Network", network_panel(this)},
    {"Toggles", toggles},
    {"Software", new SoftwarePanel(this)},
    {"Community", new CommunityPanel(this)},
    {"VIP", new VIPPanel(this)},
  };

#ifdef ENABLE_MAPS
  auto map_panel = new MapPanel(this);
  panels.push_back({"Navigation", map_panel});
  QObject::connect(map_panel, &MapPanel::closeSettings, this, &SettingsWindow::closeSettings);
#endif

  const int padding = panels.size() > 3 ? 25 : 35;

  nav_btns = new QButtonGroup(this);
  for (auto &[name, panel] : panels) {
    QPushButton *btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setStyleSheet(QString(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 60px;
        font-weight: 500;
        padding-top: %1px;
        padding-bottom: %1px;
      }
      QPushButton:checked {
        color: white;
      }
      QPushButton:pressed {
        color: #ADADAD;
      }
    )").arg(padding));

    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignRight);

    const int lr_margin = name != "Network" ? 50 : 0;
    panel->setContentsMargins(lr_margin, 25, lr_margin, 25);

    ScrollView *panel_frame = new ScrollView(panel, this);
    panel_widget->addWidget(panel_frame);

    QObject::connect(btn, &QPushButton::clicked, [=, w = panel_frame]() {
      btn->setChecked(true);
      panel_widget->setCurrentWidget(w);
    });
  }
  sidebar_layout->setContentsMargins(50, 50, 100, 50);

  QHBoxLayout *main_layout = new QHBoxLayout(this);

  sidebar_widget->setFixedWidth(500);
  main_layout->addWidget(sidebar_widget);
  main_layout->addWidget(panel_widget);

  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    SettingsWindow {
      background-color: black;
    }
  )");
}

void SettingsWindow::hideEvent(QHideEvent *event) {
#ifdef QCOM
  HardwareEon::close_activities();
#endif
}


/////////////////////////////////////////////////////////////////////////

CommunityPanel::CommunityPanel(QWidget* parent) : QWidget(parent) {

  main_layout = new QStackedLayout(this);

  homeScreen = new QWidget(this);
  QVBoxLayout* vlayout = new QVBoxLayout(homeScreen);
  vlayout->setContentsMargins(0, 20, 0, 20);

  QString selected = QString::fromStdString(Params().get("SelectedCar"));

  QPushButton* selectCarBtn = new QPushButton(selected.length() ? selected : "Select your car");
  selectCarBtn->setObjectName("selectCarBtn");
  connect(selectCarBtn, &QPushButton::clicked, [=]() { main_layout->setCurrentWidget(selectCar); });

  homeWidget = new QWidget(this);
  QVBoxLayout* toggleLayout = new QVBoxLayout(homeWidget);
  homeWidget->setObjectName("homeWidget");

  ScrollView *scroller = new ScrollView(homeWidget, this);
  scroller->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  main_layout->addWidget(homeScreen);

  selectCar = new SelectCar(this);
  connect(selectCar, &SelectCar::backPress, [=]() { main_layout->setCurrentWidget(homeScreen); });
  connect(selectCar, &SelectCar::selectedCar, [=]() {
     QString selected = QString::fromStdString(Params().get("SelectedCar"));
     selectCarBtn->setText(selected.length() ? selected : "Select your car");
     main_layout->setCurrentWidget(homeScreen);
  });
  main_layout->addWidget(selectCar);

  QString lateral_control = QString::fromStdString(Params().get("LateralControl"));
  if(lateral_control.length() == 0)
    lateral_control = "TORQUE";

  QPushButton* lateralControlBtn = new QPushButton(lateral_control);
  lateralControlBtn->setObjectName("lateralControlBtn");
  connect(lateralControlBtn, &QPushButton::clicked, [=]() { main_layout->setCurrentWidget(lateralControl); });

  lateralControl = new LateralControl(this);
  connect(lateralControl, &LateralControl::backPress, [=]() { main_layout->setCurrentWidget(homeScreen); });
  connect(lateralControl, &LateralControl::selected, [=]() {
     QString lateral_control = QString::fromStdString(Params().get("LateralControl"));
     if(lateral_control.length() == 0)
       lateral_control = "TORQUE";
     lateralControlBtn->setText(lateral_control);
     main_layout->setCurrentWidget(homeScreen);
  });
  main_layout->addWidget(lateralControl);

  QHBoxLayout* layoutBtn = new QHBoxLayout(homeWidget);
  layoutBtn->addWidget(lateralControlBtn);
  layoutBtn->addSpacing(10);
  layoutBtn->addWidget(selectCarBtn);

  vlayout->addSpacing(10);
  vlayout->addLayout(layoutBtn, 0);
  vlayout->addSpacing(10);
  vlayout->addWidget(scroller, 1);

  QPalette pal = palette();
  pal.setColor(QPalette::Background, QColor(0x29, 0x29, 0x29));
  setAutoFillBackground(true);
  setPalette(pal);

  setStyleSheet(R"(
    #back_btn, #selectCarBtn, #lateralControlBtn {
      font-size: 50px;
      margin: 0px;
      padding: 20px;
      border-width: 0;
      border-radius: 30px;
      color: #dddddd;
      background-color: #444444;
    }
  )");

  toggleLayout->addWidget(new TimeZoneSelectCombo());
  toggleLayout->addWidget(horizontal_line());

  QList<ParamControl*> toggles;
  toggles.append(new ParamControl("UseLanelines",
                                            "Use lane lines instead of e2e",
                                            "",
                                            "../assets/offroad/icon_openpilot.png",
                                            this));
  toggles.append(new ParamControl("UseClusterSpeed",
                                            "Use Cluster Speed",
                                            "Use cluster speed instead of wheel speed.",
                                            "../assets/offroad/icon_road.png",
                                            this));
  toggles.append(new ParamControl("AutoAscc",
                                            "Ascc auto set",
                                            "Ascc auto set 적용",
                                            "../assets/offroad/icon_road.png",
                                            this));
  toggles.append(new ParamControl("LongControlEnabled",
                                            "Enable HKG Long Control",
                                            "warnings: it is beta, be careful!! Openpilot will control the speed of your car",
                                            "../assets/offroad/icon_road.png",
                                            this));
  toggles.append(new ParamControl("IsLdwsCar",
                                            "LDWS",
                                            "If your car only supports LDWS, turn it on.",
                                            "../assets/offroad/icon_openpilot.png",
                                            this));
  toggles.append(new ParamControl("LaneChangeEnabled",
                                            "Enable Lane Change Assist",
                                            "Perform assisted lane changes with openpilot by checking your surroundings for safety, activating the turn signal and gently nudging the steering wheel towards your desired lane. openpilot is not capable of checking if a lane change is safe. You must continuously observe your surroundings to use this feature.",
                                            "../assets/offroad/icon_road.png",
                                            this));
  toggles.append(new ParamControl("AutoLaneChangeEnabled",
                                            "Enable Auto Lane Change(Nudgeless)",
                                            "warnings: it is beta, be careful!!",
                                            "../assets/offroad/icon_road.png",
                                            this));

  for(ParamControl *toggle : toggles) {
    if(main_layout->count() != 0) {
      toggleLayout->addWidget(horizontal_line());
    }
    toggleLayout->addWidget(toggle);
  }

  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("SccSmootherSlowOnCurves",
                                            "SCC기반 커브감속",
                                            "",
                                            "../assets/offroad/icon_road.png",
                                            this));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("TurnVisionControl",
                                           "비젼기반 커브감속",
                                           "Use vision path predictions to estimate the appropiate speed to drive through turns ahead.",
                                            "../assets/offroad/icon_road.png",
                                            this));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("VisionCurveLaneless",
                                           "커브 구간 레인리스 모드",
                                           "비전 기반 커브 감속이 활성화된 경우, 커브 구간에서 자동으로 레인리스(e2e) 모드로 전환합니다.\n"
                                           "(DynamicLaneProfile Auto 모드에서만 동작)",
                                            "../assets/offroad/icon_road.png",
                                            this));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("SccSmootherSyncGasPressed",
                                            "Sync set speed on gas pressed",
                                            "",
                                            "../assets/offroad/icon_road.png",
                                            this));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("StockNaviDecelEnabled",
                                            "Stock Navi based deceleration",
                                            "Use the stock navi based deceleration for longcontrol",
                                            "../assets/offroad/icon_road.png",
                                            this));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("KeepSteeringTurnSignals",
                                            "Keep steering while turn signals",
                                            "",
                                            "../assets/offroad/icon_openpilot.png",
                                            this));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("HapticFeedbackWhenSpeedCamera",
                                            "Haptic feedback (speed-cam alert)",
                                            "Haptic feedback when a speed camera is detected",
                                            "../assets/offroad/icon_openpilot.png",
                                            this));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("DisableOpFcw",
                                            "Disable Openpilot FCW",
                                            "",
                                            "../assets/offroad/icon_shell.png",
                                            this));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("ShowDebugUI",
                                            "Show Debug UI",
                                            "",
                                            "../assets/offroad/icon_shell.png",
                                            this));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("HumanFollowing",
                                            "Human-Like Following",
                                            "선행차 속도에 따라 자연스러운 가감속을 적용합니다.\n"
                                            "빠른 선행차: 부드럽게 따라붙기 / 느린 선행차: 자연스럽게 감속.\n"
                                            "(Long Control 활성화 시 동작)",
                                            "../assets/offroad/icon_road.png",
                                            this));
}

SelectCar::SelectCar(QWidget* parent): QWidget(parent) {

  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setMargin(20);
  main_layout->setSpacing(20);

  QPushButton* back = new QPushButton("Back");
  back->setObjectName("back_btn");
  back->setFixedSize(500, 100);
  connect(back, &QPushButton::clicked, [=]() { emit backPress(); });
  main_layout->addWidget(back, 0, Qt::AlignLeft);

  QListWidget* list = new QListWidget(this);
  list->setStyleSheet("QListView {padding: 40px; background-color: #393939; border-radius: 15px; height: 140px;} QListView::item{height: 100px}");
  QScroller::grabGesture(list->viewport(), QScroller::LeftMouseButtonGesture);
  list->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

  list->addItem("[ Not selected ]");

  QStringList items = get_list("/data/params/d/SupportedCars");
  list->addItems(items);
  list->setCurrentRow(0);

  QString selected = QString::fromStdString(Params().get("SelectedCar"));

  int index = 0;
  for(QString item : items) {
    if(selected == item) {
        list->setCurrentRow(index + 1);
        break;
    }
    index++;
  }

  QObject::connect(list, QOverload<QListWidgetItem*>::of(&QListWidget::itemClicked),
    [=](QListWidgetItem* item){
    if(list->currentRow() == 0)
        Params().remove("SelectedCar");
    else
        Params().put("SelectedCar", list->currentItem()->text().toStdString());
    emit selectedCar();
    });

  main_layout->addWidget(list);
}

LateralControl::LateralControl(QWidget* parent): QWidget(parent) {

  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setMargin(20);
  main_layout->setSpacing(20);

  QPushButton* back = new QPushButton("Back");
  back->setObjectName("back_btn");
  back->setFixedSize(500, 100);
  connect(back, &QPushButton::clicked, [=]() { emit backPress(); });
  main_layout->addWidget(back, 0, Qt::AlignLeft);

  QListWidget* list = new QListWidget(this);
  list->setStyleSheet("QListView {padding: 40px; background-color: #393939; border-radius: 15px; height: 140px;} QListView::item{height: 100px}");
  QScroller::grabGesture(list->viewport(), QScroller::LeftMouseButtonGesture);
  list->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

  QStringList items = {"TORQUE", "LQR", "INDI"};
  list->addItems(items);
  list->setCurrentRow(0);

  QString selectedControl = QString::fromStdString(Params().get("LateralControl"));

  int index = 0;
  for(QString item : items) {
    if(selectedControl == item) {
        list->setCurrentRow(index);
        break;
    }
    index++;
  }

  QObject::connect(list, QOverload<QListWidgetItem*>::of(&QListWidget::itemClicked),
    [=](QListWidgetItem* item){
    Params().put("LateralControl", list->currentItem()->text().toStdString());
    emit selected();
    QTimer::singleShot(1000, []() {
        Params().putBool("SoftRestartTriggered", true);
      });
    });

  main_layout->addWidget(list);
}

/////////////////////////////////////////////////////////////////////////

VIPPanel::VIPPanel(QWidget* parent) : QWidget(parent) {
  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 20, 50, 20);
  layout->setSpacing(0);

  ListWidget* list = new ListWidget(this);
  list->setSpacing(0);

    // ── Camera Offset ────────────────────────────────────────────
  // 레인모드에서 차선 인식 좌표 보정. 0.01m 단위, -0.20 ~ +0.20m
  auto *cam_offset = new CameraOffsetControl(
      "Camera Offset",
      "카메라 위치 보정값입니다. 레인모드에서 차선 인식 좌표에 적용됩니다.\n"
      "왼쪽으로 이동: 양수(+) / 오른쪽으로 이동: 음수(−)\n"
      "범위: −0.20 ~ +0.20m  /  기본값: −0.06m",
      "../assets/offroad/icon_road.png",
      this);
  cam_offset->showDescription();
  list->addItem(cam_offset);

  list->addItem(horizontal_line());

  // ── Path Offset ──────────────────────────────────────────────
  // 레인모드 + 레인리스 모드 모두 적용. 0.01m 단위, -1.00 ~ +1.00m
  auto *path_offset = new PathOffsetControl(
      "Path Offset",
      "주행 경로 좌우 보정값입니다. 레인모드·레인리스 모드 모두 적용됩니다.\n"
      "왼쪽으로 이동: 양수(+) / 오른쪽으로 이동: 음수(−)\n"
      "범위: −1.00 ~ +1.00m  /  기본값: 0.00m",
      "../assets/offroad/icon_road.png",
      this);
  path_offset->showDescription();
  list->addItem(path_offset);

  list->addItem(horizontal_line());
  list->addItem(new ParamControl("DynamicLaneProfileToggle",
                                  "Enable Dynamic Lane Profile",
                                  "Dynamic Lane Profile 기능을 활성화합니다.\n"
                                  "활성화 시 아래 모드 선택이 적용됩니다.",
                                  "../assets/offroad/icon_road.png",
                                  this));

  list->addItem(horizontal_line());
  auto *dlp_control = new DynamicLaneProfileControl(
      "Dynamic Lane Profile Mode",
      "Lane only: 항상 차선 기반\n"
      "Lane less: 항상 차선 미사용(e2e)\n"
      "Auto: 차선 인식률에 따라 자동 전환",
      "../assets/offroad/icon_road.png",
      this);
  dlp_control->showDescription();
  list->addItem(dlp_control);

  list->addItem(horizontal_line());

  // ── AutoLaneChangeTimer ──────────────────────────────────────
  auto *lc_timer = new AutoLaneChangeTimerControl(
      "Auto Lane Change Timer",
      "차선변경 자동 시작까지의 대기 시간을 설정합니다.\n"
      "즉시: 조건 충족 즉시 / 0.1s ~ 2.0s: 해당 시간 대기 후 자동 차선변경",
      "../assets/offroad/icon_road.png",
      this);
  lc_timer->showDescription();
  list->addItem(lc_timer);

  list->addItem(horizontal_line());

  // ── ChevronInfo ──────────────────────────────────────────────
  auto *chevron_info = new ChevronInfoControl(
      "Display Metrics Below Chevron",
      "Display useful metrics below the chevron that tracks the lead car "
      "(only applicable to cars with openpilot longitudinal control).",
      "../assets/offroad/icon_road.png",
      this);
  chevron_info->showDescription();
  list->addItem(chevron_info);

  // ── CarrotPilot Auto-Tuner (commit 9dd5e2c port) ─────────────
  list->addItem(horizontal_line());

  auto *learnToggle = new ParamControl("CarrotLearningActive",
      "Auto-Tuner: 주행 기반 학습",
      "운전자 개입(가속/브레이크/조향)을 학습하여 주차(P단) 시 파라미터 조정을 추천합니다.\n"
      "학습 대상: CruiseMaxVals0~3(가속) / TFollowGap1~4(추종거리) / PathOffset(직진 편차) /\n"
      "TurnEnteringDecel·TurnTurningAcc·TurnLeavingAcc(비전 커브 감속)\n"
      "1회 적용 시 변동폭 ±15 제한, 추종거리 최소 0.90초 보장.",
      "../assets/offroad/icon_shell.png",
      this);
  list->addItem(learnToggle);

  list->addItem(horizontal_line());
  auto *autoApplyToggle = new ParamControl("CarrotLearningAutoApply",
      "Auto-Tuner: 추천 자동 적용",
      "활성화 시 주차(P단) 전환 때 팝업 없이 추천값을 자동 적용하고 이력에 기록합니다.\n"
      "비활성화 시 P단 전환 때 선택 적용 팝업이 표시됩니다.",
      "../assets/offroad/icon_shell.png",
      this);
  list->addItem(autoApplyToggle);

  list->addItem(horizontal_line());
  QPushButton* viewHistoryBtn = new QPushButton("View Tuning History");
  viewHistoryBtn->setObjectName("viewHistoryBtn");
  viewHistoryBtn->setStyleSheet(R"(
    QPushButton {
      margin-top: 10px; margin-bottom: 20px; padding: 10px; height: 120px; border-radius: 15px;
      color: #FFFFFF; background-color: #2C2CE2;
      font-size: 50px; font-weight: 400;
    }
    QPushButton:pressed {
      background-color: #2424FF;
    }
  )");
  connect(viewHistoryBtn, &QPushButton::clicked, [=]() {
    AutoTunerHistoryDialog dlg(this);
    dlg.exec();
  });
  list->addItem(viewHistoryBtn);

  // ── Factory Reset 버튼 (commit e06a7dd) ──
  // Params 기반 학습 대상(CruiseMaxVals/TFollowGap/PathOffset/Turn*)만 공장 기본값 복원 +
  // 학습 데이터/이력 삭제. nTune 조향값(latAccelFactor/friction/steerActuatorDelay)은
  // 차량별 기준값이라 의도적으로 제외 (사용자 nTune 세팅 보호).
  QPushButton* factoryResetBtn = new QPushButton("Auto-Tuner: Factory Reset");
  factoryResetBtn->setObjectName("factoryResetBtn");
  factoryResetBtn->setStyleSheet(R"(
    QPushButton {
      margin-top: 10px; margin-bottom: 20px; padding: 10px; height: 120px; border-radius: 15px;
      color: #FFFFFF; background-color: #8a1d1d;
      font-size: 50px; font-weight: 400;
    }
    QPushButton:pressed {
      background-color: #B02525;
    }
  )");
  connect(factoryResetBtn, &QPushButton::clicked, [=]() {
    if (ConfirmationDialog::confirm("학습 파라미터(가속/추종거리/직진보정/커브감속)를 모두 공장 기본값으로 되돌리고 학습 데이터·이력을 삭제하시겠습니까?\n\n(조향 nTune 값은 변경되지 않습니다)", this)) {
      Params p;
      for (const auto& [key, val] : kAutoTunerDefaults) {
        p.put(key, val);
      }
      p.remove("CarrotLearningHistory");
      p.remove("CarrotLearningRecommend");
      p.putBool("CarrotLearningPopupReady", false);
      p.putBool("CarrotLearningClear", true);       // 누적 학습 데이터 삭제 (python 처리)
      p.putBool("CarrotTunerFactoryReset", true);   // onroad 인스턴스 재동기화 신호
      ConfirmationDialog::alert("공장 기본값으로 초기화되었습니다.", this);
    }
  });
  list->addItem(factoryResetBtn);

  // 학습 비활성 시 이력/초기화 버튼 숨김 (원본 커밋의 동적 표시 로직)
  bool learn_on = Params().getBool("CarrotLearningActive");
  viewHistoryBtn->setVisible(learn_on);
  factoryResetBtn->setVisible(learn_on);
  connect(learnToggle, &ToggleControl::toggleFlipped, [=](bool state) {
    viewHistoryBtn->setVisible(state);
    factoryResetBtn->setVisible(state);
  });

  ScrollView *scroller = new ScrollView(list, this);
  scroller->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  layout->addWidget(scroller);
}

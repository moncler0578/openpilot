#include "selfdrive/ui/qt/offroad/settings.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <map>
#include <string>
#include <tuple>

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
#include <QJsonDocument>
#include <QJsonObject>
#include <QDateTime>
#include <QStringList>
#include <QFile>
#include <QDir>

namespace {

struct StepperWidgets {
  QWidget *container;
  QPushButton *minus;
  QLabel *value;
  QPushButton *plus;
};

StepperWidgets makeStepperWidgets(int value_width) {
  QWidget *container = new QWidget();
  QHBoxLayout *layout = new QHBoxLayout(container);
  layout->setContentsMargins(0, 8, 0, 8);
  layout->setSpacing(12);

  const QString button_style = R"(
    QPushButton {
      font-size: 48px;
      font-weight: bold;
      border-radius: 14px;
      background-color: #393939;
      color: #ffffff;
      min-width: 150px;
      max-width: 150px;
      min-height: 100px;
      max-height: 100px;
    }
    QPushButton:pressed { background-color: #4a4a4a; }
  )";

  QPushButton *minus = new QPushButton("−");
  minus->setStyleSheet(button_style);
  QLabel *value = new QLabel();
  value->setAlignment(Qt::AlignCenter);
  value->setStyleSheet(QString("font-size: 40px; color: #ffffff; min-width: %1px;")
                           .arg(value_width));
  QPushButton *plus = new QPushButton("+");
  plus->setStyleSheet(button_style);

  layout->addStretch();
  layout->addWidget(minus);
  layout->addWidget(value);
  layout->addWidget(plus);
  return {container, minus, value, plus};
}

}  // namespace

// ── Offset Total Control ─────────────────────────────────────────
OffsetTotalControl::OffsetTotalControl(const QString &title,
                                       const QString &desc,
                                       const QString &icon,
                                       QWidget *parent)
    : AbstractControl(title, desc, icon, parent) {

  const StepperWidgets stepper = makeStepperWidgets(140);
  minus_btn = stepper.minus;
  value_label = stepper.value;
  plus_btn = stepper.plus;
  connect(minus_btn, &QPushButton::clicked, [=]() { changeValue(-1); });
  connect(plus_btn, &QPushButton::clicked, [=]() { changeValue(+1); });
  qobject_cast<QVBoxLayout*>(layout())->addWidget(stepper.container);
  refresh();
}

void OffsetTotalControl::changeValue(int delta) {
  std::string raw = params.get("OffsetTotal");
  double val = raw.empty() ? 0.0 : std::stod(raw);
  val += delta * 0.01;
  val = std::round(val * 100.0) / 100.0;
  val = std::max(-1.00, std::min(1.00, val));
  params.put("OffsetTotal", std::to_string(val));
  refresh();
}

void OffsetTotalControl::refresh() {
  std::string raw = params.get("OffsetTotal");
  double val = raw.empty() ? 0.0 : std::stod(raw);
  val = std::round(val * 100.0) / 100.0;
  value_label->setText(QString::number(val, 'f', 2) + " m");
  minus_btn->setEnabled(val > -1.00);
  plus_btn->setEnabled(val < 1.00);
}

// ── AdjustLaneOffset Control ─────────────────────────────────────
AdjustLaneOffsetControl::AdjustLaneOffsetControl(const QString &title,
                                                 const QString &desc,
                                                 const QString &icon,
                                                 QWidget *parent)
    : AbstractControl(title, desc, icon, parent) {

  const StepperWidgets stepper = makeStepperWidgets(140);
  minus_btn = stepper.minus;
  value_label = stepper.value;
  plus_btn = stepper.plus;
  connect(minus_btn, &QPushButton::clicked, [=]() { changeValue(-1); });
  connect(plus_btn, &QPushButton::clicked, [=]() { changeValue(+1); });
  qobject_cast<QVBoxLayout*>(layout())->addWidget(stepper.container);
  refresh();
}

void AdjustLaneOffsetControl::changeValue(int delta) {
  int val = std::atoi(params.get("AdjustLaneOffset").c_str());
  val += delta * 5;                       // 5cm 단위
  val = std::max(0, std::min(40, val));   // 0 ~ 40cm (내부 클리핑 0.4m)
  params.put("AdjustLaneOffset", std::to_string(val));
  refresh();
}

void AdjustLaneOffsetControl::refresh() {
  int val = std::atoi(params.get("AdjustLaneOffset").c_str());
  val = std::max(0, std::min(40, val));
  value_label->setText(val == 0 ? "OFF" : QString::number(val) + " cm");
  minus_btn->setEnabled(val > 0);
  plus_btn->setEnabled(val < 40);
}

// ── LanelessOffset Control ───────────────────────────────────────
LanelessOffsetControl::LanelessOffsetControl(const QString &title,
                                             const QString &desc,
                                             const QString &icon,
                                             QWidget *parent)
    : AbstractControl(title, desc, icon, parent) {

  const StepperWidgets stepper = makeStepperWidgets(140);
  minus_btn = stepper.minus;
  value_label = stepper.value;
  plus_btn = stepper.plus;
  connect(minus_btn, &QPushButton::clicked, [=]() { changeValue(-1); });
  connect(plus_btn, &QPushButton::clicked, [=]() { changeValue(+1); });
  qobject_cast<QVBoxLayout*>(layout())->addWidget(stepper.container);
  refresh();
}

void LanelessOffsetControl::changeValue(int delta) {
  int val = std::atoi(params.get("LanelessOffset").c_str());
  val += delta;                             // 1cm 단위
  val = std::max(-30, std::min(30, val));   // -30 ~ +30cm
  params.put("LanelessOffset", std::to_string(val));
  refresh();
}

void LanelessOffsetControl::refresh() {
  int val = std::atoi(params.get("LanelessOffset").c_str());
  val = std::max(-30, std::min(30, val));
  value_label->setText(val == 0 ? QString("OFF")
                                : QString(val > 0 ? "왼쪽 " : "오른쪽 ")
                                    + QString::number(std::abs(val)) + " cm");
  minus_btn->setEnabled(val > -30);
  plus_btn->setEnabled(val < 30);
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

// ── nTune steering parameter helpers ───────────────────────────────────
// Read and write the manual nTune steering settings.
static QString findNtuneTorqueFileS() {
  QDir dir("/data/ntune");
  const QStringList files = dir.entryList({"lat_torque*.json"}, QDir::Files, QDir::Name);
  if (!files.isEmpty()) return dir.filePath(files.first());
  return "/data/ntune/lat_torque_v4.json";
}

static void writeNtuneJsonValue(const QString &path, const QString &key, double value) {
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

static void writeNtuneTorqueValueS(const QString &key, double value) {
  writeNtuneJsonValue(findNtuneTorqueFileS(), key, value);
}

static void writeNtuneCommonValueS(const QString &key, double value) {
  writeNtuneJsonValue("/data/ntune/common.json", key, value);
}

// 토크값(latAccelFactor/friction)은 nTune JSON 이 아니라 Params 에 있다.
static const std::map<QString, std::pair<QString, double>> kTorqueParamKey = {
  {"latAccelFactor", {"LateralTorqueAccelFactor", 1000.0}},
  {"friction",       {"LateralTorqueFriction",    1000.0}},
};

static bool torqueParamOf(const QString &key, QString *pkey, double *scale) {
  auto it = kTorqueParamKey.find(key);
  if (it == kTorqueParamKey.end()) return false;
  *pkey = it->second.first;
  *scale = it->second.second;
  return true;
}

static double readNtuneValueS(const QString &group, const QString &key, double def) {
  QString pkey; double scale;
  if (group == "torque" && torqueParamOf(key, &pkey, &scale)) {
    std::string v = Params().get(pkey.toStdString());
    return v.empty() ? def : std::atof(v.c_str()) / scale;
  }
  QString path = (group == "torque") ? findNtuneTorqueFileS() : "/data/ntune/common.json";
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) return def;
  QJsonObject obj = QJsonDocument::fromJson(f.readAll()).object();
  f.close();
  return obj.contains(key) ? obj[key].toDouble(def) : def;
}

// ── Param Value Control (정수 Params) ────────────────────────────
ParamValueControlF::ParamValueControlF(const QString &param, const QString &title, const QString &desc,
                                       const QString &icon, int vmin, int vmax, int step, int decimals,
                                       int vdefault, QWidget *parent)
    : AbstractControl(title, desc, icon, parent),
      param_(param), vmin_(vmin), vmax_(vmax), step_(step), decimals_(decimals), vdefault_(vdefault) {

  const StepperWidgets stepper = makeStepperWidgets(170);
  minus_btn = stepper.minus;
  value_label = stepper.value;
  plus_btn = stepper.plus;
  connect(minus_btn, &QPushButton::clicked, [=]() { changeValue(-1); });
  connect(plus_btn, &QPushButton::clicked, [=]() { changeValue(+1); });
  qobject_cast<QVBoxLayout*>(layout())->addWidget(stepper.container);
  refresh();
}

void ParamValueControlF::changeValue(int delta) {
  std::string cur = params.get(param_.toStdString());
  int v = cur.empty() ? vdefault_ : std::atoi(cur.c_str());
  const int step = std::max(1, step_);

  // 저장된 값이 step 격자에서 벗어나 있으면(APM 등 다른 도구가 1 단위로 쓴 경우)
  // 그냥 step 을 더하면 격자에 영원히 못 올라탄다. 예: step 5, 현재 97 -> 102/92 만
  // 오가고 100 에 닿지 못한다. 그래서 격자 밖이면 먼저 누른 방향의 격자점으로 붙인다.
  const int base = (int)std::floor((double)v / (double)step) * step;   // 음수 범위도 안전
  v = (v == base) ? v + delta * step
                  : ((delta > 0) ? base + step : base);

  v = std::max(vmin_, std::min(vmax_, v));
  params.put(param_.toStdString(), std::to_string(v));
  refresh();
}

void ParamValueControlF::refresh() {
  std::string cur = params.get(param_.toStdString());
  int v = cur.empty() ? vdefault_ : std::atoi(cur.c_str());
  v = std::max(vmin_, std::min(vmax_, v));
  if (param_ == "TrafficStopMode") {
    static const QStringList modes = {"ACC", "AUTO", "APILOT"};
    value_label->setText(modes[v]);
  } else if (param_ == "InitMyDrivingMode") {
    static const QStringList modes = {"SAFE", "ECO", "NORMAL", "FAST", "AUTO"};
    value_label->setText(modes[v - 1]);
  } else if (vmin_ == 0 && vmax_ == 1) {
    value_label->setText(v > 0 ? "ON" : "OFF");
  } else {
    value_label->setText(QString::number(v));
  }
  minus_btn->setEnabled(v > vmin_);
  plus_btn->setEnabled(v < vmax_);
}

// ── nTune Value Control ──────────────────────────────────────────
NtuneValueControl::NtuneValueControl(const QString &group, const QString &key,
                                     const QString &title, const QString &desc, const QString &icon,
                                     double vmin, double vmax, double step, int decimals,
                                     double vdefault, QWidget *parent)
    : AbstractControl(title, desc, icon, parent),
      group_(group), key_(key), vmin_(vmin), vmax_(vmax), step_(step),
      vdefault_(vdefault), decimals_(decimals) {

  const StepperWidgets stepper = makeStepperWidgets(170);
  minus_btn = stepper.minus;
  value_label = stepper.value;
  plus_btn = stepper.plus;
  connect(minus_btn, &QPushButton::clicked, [=]() { changeValue(-1); });
  connect(plus_btn, &QPushButton::clicked, [=]() { changeValue(+1); });
  qobject_cast<QVBoxLayout*>(layout())->addWidget(stepper.container);
  refresh();
}

void NtuneValueControl::changeValue(int delta) {
  double v = readNtuneValueS(group_, key_, vdefault_);
  const double step = (step_ > 0.0) ? step_ : 1.0;

  // 격자 스냅 (ParamValueControlF 와 같은 이유). 예: step 0.005, 현재 0.097 이면
  // 0.102/0.092 만 오가므로 0.100 에 닿지 못한다. 격자 밖이면 먼저 붙인다.
  const double eps = step * 1e-6;
  const double base = std::floor(v / step + eps) * step;
  v = (std::fabs(v - base) < eps) ? v + delta * step
                                  : ((delta > 0) ? base + step : base);

  // 부동소수 오차 정리
  double round_scale = std::pow(10.0, decimals_);
  v = std::round(v * round_scale) / round_scale;
  v = std::max(vmin_, std::min(vmax_, v));

  QString pkey; double param_scale;
  if (group_ == "torque" && torqueParamOf(key_, &pkey, &param_scale)) {
    Params().put(pkey.toStdString(), std::to_string((int)std::llround(v * param_scale)));
    Params().put("LateralTorqueCustom", "1");
  } else if (group_ == "torque") {
    writeNtuneTorqueValueS(key_, v);
  } else {
    writeNtuneCommonValueS(key_, v);
  }
  refresh();
}

void NtuneValueControl::refresh() {
  double v = readNtuneValueS(group_, key_, vdefault_);
  if (decimals_ == 0) {
    value_label->setText(v > 0.5 ? "ON" : "OFF");
  } else {
    value_label->setText(QString::number(v, 'f', decimals_));
  }
  minus_btn->setEnabled(v > vmin_ + 1e-9);
  plus_btn->setEnabled(v < vmax_ - 1e-9);
}

TogglesPanel::TogglesPanel(SettingsWindow *parent) : QWidget(parent) {
  main_layout = new QStackedLayout(this);
  home = new ListWidget(this);
  main_layout->addWidget(home);

  QString selected = QString::fromStdString(params.get("SelectedCar"));
  select_car_btn = new QPushButton(selected.length() ? selected : "Select your car");
  select_car_btn->setObjectName("selectCarBtn");
  home->addItem(select_car_btn);

  auto select_car = new SelectCar(this);
  connect(select_car_btn, &QPushButton::clicked, [=]() { main_layout->setCurrentWidget(select_car); });
  connect(select_car, &SelectCar::backPress, [=]() { main_layout->setCurrentWidget(home); });
  connect(select_car, &SelectCar::selectedCar, [=]() {
    QString selected = QString::fromStdString(params.get("SelectedCar"));
    select_car_btn->setText(selected.length() ? selected : "Select your car");
    main_layout->setCurrentWidget(home);
  });
  main_layout->addWidget(select_car);

  setStyleSheet(R"(
    #back_btn, #selectCarBtn {
      font-size: 50px;
      margin: 0px;
      padding: 20px;
      border-width: 0;
      border-radius: 30px;
      color: #dddddd;
      background-color: #444444;
    }
  )");

  std::vector<std::tuple<QString, QString, QString, QString>> toggle_defs{
    {
      "OpenpilotEnabledToggle",
      "OPENPILOT ENABLED",
      "켜짐: 차간거리·속도 및 차선유지 보조를 사용합니다. 항상 전방을 주시해야 하며 설정은 차량 전원이 꺼진 뒤 적용됩니다.",
      "../assets/offroad/icon_openpilot.png",
    },
    {
      "IsLdwEnabled",
      "LDW ENABLED",
      "켜짐: 50km/h 이상에서 방향지시등 없이 차선을 벗어나면 경고합니다.",
      "../assets/offroad/icon_warning.png",
    },
    {
      "IsMetric",
      "METRIC UNITS",
      "켜짐: 속도를 km/h로 표시 / 꺼짐: mph로 표시합니다.",
      "../assets/offroad/icon_metric.png",
    },
    {
      "UseClusterSpeed",
      "USE CLUSTER SPEED",
      "켜짐: 제어용 차량속도를 계기판 속도로 사용합니다. / 꺼짐(권장): 휠 속도를 사용합니다. 변경 후 다음 주행부터 적용됩니다.",
      "../assets/offroad/icon_road.png",
    },
    {
      "RecordFront",
      "DRIVER CAMERA RECORDING",
      "켜짐: 운전자 모니터링 개선을 위해 실내 카메라 영상을 녹화하고 업로드합니다.",
      "../assets/offroad/icon_monitoring.png",
    },
    {
      "ExperimentalMode",
      "EXPERIMENTAL MODE",
      "켜짐: E2E 종방향 제어 등 알파 수준 기능을 사용합니다. 시험 기능이므로 안전 경고를 확인하십시오.",
      "../assets/img_experimental_white.svg",
    },
    {
      "ExperimentalLongitudinalEnabled",
      "EXPERIMENTAL LONGITUDINAL",
      "<b>주의: 이 차량의 종방향 제어는 시험 기능이며 순정 AEB가 비활성화될 수 있습니다.</b><br>\
          켜짐: 순정 ACC 대신 오픈파일럿이 가속과 제동을 제어합니다.",
      "../assets/offroad/icon_speed_limit.png",
    },
#ifdef ENABLE_MAPS
    {
      "NavSettingTime24h",
      "24-HOUR TIME",
      "켜짐: 도착예정시간을 24시간제로 표시 / 꺼짐: 오전·오후 형식으로 표시합니다.",
      "../assets/offroad/icon_metric.png",
    },
#endif
  };

  for (auto &[param, title, desc, icon] : toggle_defs) {
    auto toggle = new ParamControl(param, title, desc, icon, home);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    home->addItem(toggle);
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
  main_layout->setCurrentWidget(home);
  toggles[param.toStdString()]->showDescription();
}

void TogglesPanel::showEvent(QShowEvent *event) {
  main_layout->setCurrentWidget(home);
  updateToggles();
}

void TogglesPanel::updateToggles() {
  auto e2e_toggle = toggles["ExperimentalMode"];
  auto op_long_toggle = toggles["ExperimentalLongitudinalEnabled"];
  const QString e2e_description = tr("\
    오픈파일럿은 기본적으로 안정적인 <b>일반모드</b>로 주행합니다.\
    실험모드는 아직 일반모드에 포함되지 않은 <b>알파 수준 기능</b>을 활성화합니다.\
    주요 실험 기능:\
    <br> \
    <h4>🌮 E2E 종방향 제어 🌮</h4> \
    주행 모델이 가속과 제동을 제어하며 적색 신호와 정지표지판 정지를 포함한 사람과 유사한 주행을 시도합니다.");

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
      const QString no_long = "현재 차량에서는 오픈파일럿 종방향 제어를 사용할 수 없습니다.";
      const QString exp_long = "먼저 오픈파일럿 실험 종방향 제어를 켜십시오.";
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
    {"UI 설정", new UISettingsPanel(this)},
    {"조향", new VIPPanel(this)},
    {"Cruise", new CruisePanel(this)},
    {"롱컨", new LongitudinalPanel(this)},
    {"네비MAP", new CommunityPanel(this)},
  };

#ifdef ENABLE_MAPS
  auto map_panel = new MapPanel(this);
  panels.push_back({"Navigation", map_panel});
  QObject::connect(map_panel, &MapPanel::closeSettings, this, &SettingsWindow::closeSettings);
#endif

  const int padding = panels.size() > 7 ? 10 : (panels.size() > 3 ? 25 : 35);

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

  homeWidget = new QWidget(this);
  QVBoxLayout* toggleLayout = new QVBoxLayout(homeWidget);
  homeWidget->setObjectName("homeWidget");

  ScrollView *scroller = new ScrollView(homeWidget, this);
  scroller->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  main_layout->addWidget(homeScreen);

  vlayout->addSpacing(10);
  vlayout->addWidget(scroller, 1);

  QPalette pal = palette();
  pal.setColor(QPalette::Background, QColor(0x29, 0x29, 0x29));
  setAutoFillBackground(true);
  setPalette(pal);

  toggleLayout->addWidget(new ParamControl(
      "NavigationOnOpenpilot", "NAVIGATION ON OPENPILOT",
      "티맵 경로 기반 회전조향·회전감속과 분기·권장차로 차선변경을 하나로 수행합니다. "
      "차선변경은 modelV2 현재차로, 차로 수, 도로경계와 BSD가 모두 확실할 때 한 차로씩만 작동합니다.",
      "../assets/offroad/icon_road.png", this));

  toggleLayout->addWidget(new ParamValueControlF(
      "NooMode", "NOO MODE",
      "NOO 안에서 쓸 기능을 고릅니다.\n0: 회전조향 + 자동차선변경 / 1: 회전조향만 / 2: 자동차선변경만\n1·2 를 고르면 나머지 하나는 회전감속까지 함께 꺼집니다.",
      "../assets/offroad/icon_road.png", 0, 2, 1, 0, 2, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "NooTurnSpeed", "NOO TURN SPEED",
      "NOO 회전 구간 목표속도(km/h)입니다. 값 증가(+): 더 빠르게 회전 / 값 감소(-): 더 많이 감속.",
      "../assets/offroad/icon_speed_limit.png", 30, 60, 5, 0, 30, this));

  toggleLayout->addWidget(new ParamValueControlF(
      "NooTurnEndTime", "NOO DECEL TIMING",
      "NOO 회전감속 준비시간(초)입니다. 값 증가(+): 더 일찍 감속 시작 / 값 감소(-): 회전에 가까워져 감속.",
      "../assets/offroad/icon_road.png", 2, 12, 1, 0, 6, this));

  toggleLayout->addWidget(horizontal_line());

  toggleLayout->addWidget(new ParamControl("TurnVisionControl",
                                           "VISION / MAP CURVE CONTROL",
                                           "켜짐: 비전 모델과 티맵 경로 중 더 낮은 커브 목표속도를 적용합니다. / 꺼짐: 커브 자동감속을 사용하지 않습니다.",
                                            "../assets/offroad/icon_road.png",
                                            this));
  toggleLayout->addWidget(new ParamValueControlF(
      "AutoCurveSpeedFactor", "VISION CURVE SPEED FACTOR",
      "비전 모델의 커브 판단 강도입니다. 값 증가(+): 커브에서 더 많이 감속 / 값 감소(-): 감속을 줄임.",
      "../assets/offroad/icon_road.png", 50, 300, 5, 0, 120, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "AutoCurveSpeedLowerLimit", "MINIMUM CURVE SPEED",
      "비전·티맵 커브 목표속도의 하한입니다. 값 증가(+): 커브 속도가 빨라짐 / 값 감소(-): 더 낮은 속도까지 감속.",
      "../assets/offroad/icon_speed_limit.png", 5, 80, 5, 0, 30, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "MapTurnSpeedFactor", "TMAP CURVE SPEED FACTOR",
      "티맵 경로의 커브 목표속도 비율입니다. 값 증가(+): 커브 속도가 빨라지고 감속이 줄어듦 / 값 감소(-): 더 느리게 통과.",
      "../assets/offroad/icon_road.png", 50, 150, 5, 0, 90, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "AutoNaviSpeedDecelRate", "MAP CURVE DECEL RATE",
      "티맵 커브 진입 감속 강도(×0.01m/s²)입니다. 값 증가(+): 늦고 강하게 감속 / 값 감소(-): 일찍 부드럽게 감속.",
      "../assets/offroad/icon_road.png", 10, 300, 10, 0, 120, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "AutoNaviSpeedCtrlEnd", "CAMERA DECEL END TIME (SEC)",
      "C3 방식의 카메라 감속 완료지점입니다. 값 증가(+): 카메라에서 더 먼 지점까지 감속을 완료합니다.",
      "../assets/offroad/icon_speed_limit.png", 3, 20, 1, 0, 7, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "AutoNaviSpeedBumpTime", "SPEED BUMP DECEL TIME (SEC)",
      "C3 방식의 방지턱 감속 완료지점입니다. 목표속도로 이 시간만큼 주행할 거리 전에 감속을 완료합니다.",
      "../assets/offroad/icon_speed_limit.png", 1, 50, 1, 0, 1, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "AutoNaviSpeedBumpSpeed", "SPEED BUMP TARGET SPEED (km/h)",
      "C3 방식의 고정 방지턱 통과 목표속도입니다. 카메라 안전비율은 적용하지 않습니다.",
      "../assets/offroad/icon_speed_limit.png", 10, 100, 5, 0, 35, this));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("StockNaviDecelEnabled",
                                            "STOCK NAVI DECEL",
                                            "켜짐: 순정 내비게이션의 제한속도·카메라 정보를 종방향 감속에 사용합니다.",
                                            "../assets/offroad/icon_road.png",
                                            this));
  toggleLayout->addWidget(horizontal_line());
  toggleLayout->addWidget(new ParamControl("HapticFeedbackWhenSpeedCamera",
                                            "SPEED CAMERA HAPTIC",
                                            "켜짐: 과속카메라가 감지되면 핸들 진동으로 알립니다.",
                                            "../assets/offroad/icon_openpilot.png",
                                            this));

  toggleLayout->addWidget(horizontal_line());

  // ── S9 외부 클러스터 HUD ──────────────────────────────────
  toggleLayout->addWidget(new ParamControl(
      "EonClusterHud", "S9 EXTERNAL HUD",
      "EON 주행 데이터를 S9 앱으로 전송합니다. 아래 출력 대상에서 외부 HUD와 S9 화면을 선택할 수 있습니다.",
      "../assets/offroad/icon_road.png", this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudFps", "S9 HUD FPS",
      "EON 주행정보 송신과 S9 렌더링 속도입니다. 권장값 7 / 0은 화면 정지(연결유지 2Hz) / 10 초과는 EON 송신 10Hz 상한.",
      "../assets/offroad/icon_road.png", 0, 15, 1, 0, 7, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudMapFps", "S9 HUD MAP FPS",
      "티맵 지도 수신·파일검사·S9 전송 속도입니다. 권장값 3 / 부하 최소 2 / 움직임 우선 5.",
      "../assets/offroad/icon_road.png", 2, 5, 1, 0, 3, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudBrightness", "S9 HUD BRIGHTNESS", "S9 외부 HUD 밝기: 0 자동 / 1~100 고정",
      "../assets/offroad/icon_road.png", 0, 100, 5, 0, 65, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudJpegQuality", "S9 HUD JPEG QUALITY",
      "S9에서 생성해 외부 HUD로 보내는 JPEG 품질입니다. 권장값 55 / 선명도 우선 60. EON CPU 영향은 작습니다.",
      "../assets/offroad/icon_road.png", 20, 95, 1, 0, 55, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudOutputTarget", "S9 HUD OUTPUT TARGET",
      "1: 외부 HUD / 2: S9 화면 / 3: 동시 출력",
      "../assets/offroad/icon_road.png", 1, 3, 1, 0, 3, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudOutputMode", "S9 HUD OUTPUT MODE",
      "1: 주행 / 지도 / 시스템   2: 실시간 디버그   3: S9 리모트(폰 상태·USB 진단)",
      "../assets/offroad/icon_road.png", 1, 3, 1, 0, 1, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudLayoutMode", "S9 HUD LAYOUT MODE",
      "1: 주행 + 티맵 + 시스템 정보 / 2: 주행 + 티맵만(우측 정보판 숨김, 티맵 폭 확장)",
      "../assets/offroad/icon_road.png", 1, 2, 1, 0, 1, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudCarStyle", "S9 HUD CAR STYLE",
      "1: 사진 / 2: 3D 박스. 자차와 앞차에 적용됩니다.",
      "../assets/offroad/icon_road.png", 1, 2, 1, 0, 1, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudBsdStyle", "S9 HUD BSD STYLE",
      "1: 경계막대만 / 2: 옅은 띠 / 3: 진한 띠. 옆차 앞뒤 위치는 알 수 없어 차선 전체를 표시합니다.",
      "../assets/offroad/icon_road.png", 1, 3, 1, 0, 2, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudRoadSigns", "S9 HUD ROAD SIGNS",
      "주행씬 도로 바닥에 그리는 표시입니다.\n"
      "0: 끔 / 1: 제한속도만 / 2: 과속방지턱만 / 3: 둘 다",
      "../assets/offroad/icon_road.png", 0, 3, 1, 0, 3, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudRoadZ", "S9 HUD ROAD Z",
      "오르막·내리막 표현 강도(%). 100: 모델 값 그대로 / 0: 평지 / 음수: 위아래 반전(오르막이 꺼져 보일 때 -100).",
      "../assets/offroad/icon_road.png", -300, 300, 10, 0, 100, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudPitchDyn", "S9 HUD PITCH DYN",
      "가감속·요철로 차가 기울 때 수평선이 따라 움직이는 정도(%). 0: 끔(정지 캘리브만), 클수록 화면이 많이 흔들립니다.",
      "../assets/offroad/icon_road.png", 0, 200, 10, 0, 60, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudBuildings", "S9 HUD BUILDINGS",
      "0: 끔 / 1: 켬. 건물은 실제 지형이 아니라 속도감용 장식입니다.",
      "../assets/offroad/icon_road.png", 0, 1, 1, 0, 1, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudTmapIcon", "S9 HUD TMAP ICON",
      "0(기본): 앱 내장 화살표 그림.\n1: 티맵 compact 스트림 사용 — 이 스트림은 아이콘이 아니라 화살표+거리+도로명이 한 장에 그려진 미니 배너라, 켜면 검은 박스나 배너 중복으로 보일 수 있습니다.",
      "../assets/offroad/icon_road.png", 0, 1, 1, 0, 0, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudJunction", "S9 HUD JUNCTION",
      "티맵 분기 실사 이미지와 도착정보 바.\n0: 끔 / 1: 실사 이미지만 / 2: 실사 + 도착·분·km 바",
      "../assets/offroad/icon_road.png", 0, 2, 1, 0, 2, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudScreenMode", "S9 HUD SCREEN MODE", "1: 자동(길안내/주행리포트) / 2: 실시간 디버그 / 3: 주행리포트 고정",
      "../assets/offroad/icon_road.png", 1, 3, 1, 0, 1, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudTheme", "S9 HUD THEME", "0: 자동 / 1: 다크 / 2: 라이트. S9의 시스템·우측 정보 패널에 실시간 적용됩니다.",
      "../assets/offroad/icon_road.png", 0, 2, 1, 0, 0, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudOrientation", "S9 HUD ORIENTATION", "0: 기본 / 2: 180도 회전",
      "../assets/offroad/icon_road.png", 0, 2, 2, 0, 0, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudMirror", "S9 HUD MIRROR", "0: 기본 / 1: 좌우 미러",
      "../assets/offroad/icon_road.png", 0, 1, 1, 0, 0, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudPathFlip", "S9 HUD PATH FLIP",
      "0: 기본 / 1: 차선·경로 리본·옆차 위치만 좌우반전 (화면 전체 미러와는 별개, 진단용)",
      "../assets/offroad/icon_road.png", 0, 1, 1, 0, 0, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudLanguage", "S9 HUD LANGUAGE", "0: 한국어 / 1: English",
      "../assets/offroad/icon_road.png", 0, 1, 1, 0, 0, this));
  toggleLayout->addWidget(new ParamValueControlF(
      "EonClusterHudRadarInfo", "S9 HUD RADAR INFO", "0: 숨김 / 1,3: 앞차 상대속도 / 2,4: 앞차 거리+상대속도",
      "../assets/offroad/icon_road.png", 0, 4, 1, 0, 4, this));
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

CruisePanel::CruisePanel(QWidget* parent) : QWidget(parent) {
  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 20, 50, 20);
  layout->setSpacing(0);

  ListWidget* list = new ListWidget(this);
  list->setSpacing(0);

  list->addItem(new ParamValueControlF(
      "CruiseSpeedMin", "CRUISE SPEED MIN (km/h)",
      "롱컨의 최저 설정속도입니다. 값 증가(+): 처음 설정되는 속도가 높아짐 / 값 감소(-): 더 낮은 속도로 설정 가능.",
      "../assets/offroad/icon_road.png", 5, 30, 1, 0, 30, this));

  list->addItem(new ParamControl(
      "ApplyLongDynamicCost", "DYNAMIC FOLLOWING RESPONSE",
      "켜짐: 앞차 속도와 차간거리에 따라 가감속 반응을 동적으로 조정합니다. 저속에서 앞차가 멀어지면 가속 반응이 빨라질 수 있습니다.",
      "../assets/offroad/icon_road.png", this));

  list->addItem(new ParamValueControlF(
      "SpeedFromPCM", "SPEED FROM PCM",
      "1: 순정 SCC 설정속도 사용 / 2: 오픈파일럿 설정속도와 사용자 크루즈 버튼 설정 사용.",
      "../assets/offroad/icon_road.png", 1, 2, 1, 0, 2, this));

  list->addItem(new ParamValueControlF(
      "AutoGasTokSpeed", "AUTO GAS TOK SPEED (km/h)",
      "가속페달 해제 후 자동재개가 가능한 최저속도입니다. 값 증가(+): 더 높은 속도에서만 재개 / 값 감소(-): 저속에서도 재개.",
      "../assets/offroad/icon_road.png", 5, 60, 1, 0, 30, this));

  list->addItem(new ParamValueControlF(
      "AutoGasCancelSpeed", "AUTO GAS CANCEL SPEED (km/h)",
      "이 속도 미만에서는 가속페달 해제 자동재개를 취소합니다. 값 증가(+): 자동재개 조건이 엄격해짐 / 값 감소(-): 저속 재개가 쉬워짐.",
      "../assets/offroad/icon_road.png", 0, 60, 1, 0, 30, this));

  list->addItem(new ParamValueControlF(
      "CruiseButtonMode", "CRUISE BUTTON MODE",
      "0: 기본 증감 / 1: RES·SET 사용자 단위 / 2: SET으로 현재속도 동기화 / 3: RES로 지정속도표 순환.",
      "../assets/offroad/icon_road.png", 0, 3, 1, 0, 0, this));

  list->addItem(new ParamValueControlF(
      "CruiseSpeedUnit", "CRUISE SPEED UNIT (km/h)",
      "버튼 모드 1~3의 속도 증감 단위입니다. 값 증가(+): 한 번에 속도가 크게 변함 / 값 감소(-): 세밀하게 변함.",
      "../assets/offroad/icon_road.png", 1, 20, 1, 0, 10, this));

  list->addItem(new ParamValueControlF(
      "CruiseSpeedUnitBasic", "CRUISE SPEED UNIT BASIC (km/h)",
      "버튼 모드 0에서 짧게 누를 때의 증감 단위입니다. 값 증가(+): 큰 폭으로 변경 / 값 감소(-): 작은 폭으로 변경.",
      "../assets/offroad/icon_road.png", 1, 10, 1, 0, 1, this));

  list->addItem(new ParamValueControlF(
      "CruiseButtonLongDelay", "CRUISE BUTTON LONG DELAY",
      "RES/SET 길게누름 판정시간(×0.01초)입니다. 값 증가(+): 더 오래 눌러야 작동 / 값 감소(-): 빠르게 길게누름으로 판정.",
      "../assets/offroad/icon_road.png", 30, 150, 5, 0, 40, this));

  const std::array<std::tuple<const char*, const char*, int, int>, 5> cruise_speed_table = {{
    {"CruiseSpeed1", "CRUISE SPEED1 (km/h)", 30, 0},
    {"CruiseSpeed2", "CRUISE SPEED2 (km/h)", 50, 30},
    {"CruiseSpeed3", "CRUISE SPEED3 (km/h)", 70, 30},
    {"CruiseSpeed4", "CRUISE SPEED4 (km/h)", 90, 30},
    {"CruiseSpeed5", "CRUISE SPEED5 (km/h)", 110, 30},
  }};
  for (const auto& [key, title, default_value, min_value] : cruise_speed_table) {
    list->addItem(new ParamValueControlF(
        key, title, "버튼 모드 3의 순환 설정속도입니다. 1단계 값 0: 현재 도로 제한속도 사용 / 값 증가(+): 해당 단계 속도가 높아짐.",
        "../assets/offroad/icon_road.png", min_value, 145, 5, 0, default_value, this));
  }

  list->addItem(new ParamValueControlF(
      "AutoSpeedUptoRoadSpeedLimit", "AUTO SPEED UPTO ROAD SPEED LIMIT (%)",
      "앞차를 따라 설정속도를 올릴 수 있는 도로 제한속도 비율입니다. 값 증가(+): 더 높은 속도까지 증속 / 값 감소(-): 증속 상한이 낮아짐 / 0: 끔.",
      "../assets/offroad/icon_road.png", 0, 120, 5, 0, 0, this));

  list->addItem(new ParamValueControlF(
      "AutoRoadSpeedAdjust", "AUTO ROAD SPEED ADJUST (%)",
      "0: 설정속도 유지 / +값 증가: 제한속도 하락을 더 많이 반영 / +값 감소: 천천히 반영 / 음수: 새 제한속도로 즉시 변경.",
      "../assets/offroad/icon_road.png", -100, 100, 10, 0, 0, this));

  list->addItem(new ParamValueControlF(
      "AutoRoadSpeedLimitOffset", "AUTO ROAD SPEED LIMIT OFFSET (km/h)",
      "도로 제한속도에 더하는 값입니다. 양수(+): 제한속도보다 높게 설정 / 음수(-): 제한속도보다 낮게 설정.",
      "../assets/offroad/icon_road.png", -30, 30, 1, 0, 0, this));

  list->addItem(new ParamValueControlF(
      "AutoNaviSpeedSafetyFactor", "AUTO NAVI SPEED SAFETY FACTOR (%)",
      "카메라·구간단속 목표속도 비율입니다. 값 증가(+): 목표속도가 높아져 감속이 줄어듦 / 값 감소(-): 더 낮게 감속 / 100: 원래 속도.",
      "../assets/offroad/icon_road.png", 80, 120, 1, 0, 105, this));

  list->addItem(new ParamControl(
      "AutoGasResumeGuard", "AUTO GAS RESUME GUARD",
      "켜짐: 가속페달 해제 자동재개 전에 크루즈 가능 상태와 안전조건을 확인합니다.",
      "../assets/offroad/icon_road.png", this));

  list->addItem(new ParamControl(
      "SccSmootherSyncGasPressed", "SCC SMOOTHER SYNC GAS PRESSED",
      "켜짐: 가속페달로 설정속도보다 빨라지면 현재 차량속도에 맞춰 설정속도를 올립니다.",
      "../assets/offroad/icon_road.png", this));

  list->addItem(new ParamValueControlF(
      "AutoResumeFromGas", "AUTO RESUME FROM GAS",
      "0: 끔 / 1: 조건 충족 시 재개 / 2: 조건 충족 또는 0.4초 미만의 짧은 가속 후 재개.",
      "../assets/offroad/icon_road.png", 0, 2, 1, 0, 1, this));

  list->addItem(new ParamValueControlF(
      "AutoResumeFromGasSpeedMode", "AUTO RESUME FROM GAS SPEED MODE",
      "0: 현재속도 / 1: 이전 설정속도 / 2: 앞차가 있을 때 이전 설정속도 / 3: 정지점 60m 초과·가속 1초 초과 시 이전 설정속도로 재개.",
      "../assets/offroad/icon_road.png", 0, 3, 1, 0, 0, this));

  list->addItem(new ParamControl(
      "AutoResumeFromBrakeRelease", "AUTO RESUME FROM BRAKE RELEASE",
      "켜짐: 브레이크를 놓을 때 조향·신호·앞차 거리 또는 속도 안전조건을 만족하면 롱컨을 재개합니다.",
      "../assets/offroad/icon_road.png", this));

  list->addItem(new ParamValueControlF(
      "AutoResumeFromBrakeCarSpeed", "AUTO RESUME FROM BRAKE CAR SPEED (km/h)",
      "앞차가 없을 때 필요한 최저 재개속도입니다. 값 증가(+): 더 높은 속도에서만 재개 / 값 감소(-): 저속에서도 재개.",
      "../assets/offroad/icon_road.png", 5, 60, 1, 0, 30, this));

  list->addItem(new ParamValueControlF(
      "AutoResumeFromBrakeReleaseDist", "AUTO RESUME FROM BRAKE RELEASE DIST",
      "브레이크 해제 재개에 필요한 앞차 거리입니다. 값 증가(+): 앞차가 더 멀어야 재개 / 값 감소(-): 가까워도 재개.",
      "../assets/offroad/icon_road.png", 2, 50, 1, 0, 10, this));

  ScrollView *scroller = new ScrollView(list, this);
  scroller->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  layout->addWidget(scroller);
}
/////////////////////////////////////////////////////////////////////////

LongitudinalPanel::LongitudinalPanel(QWidget* parent) : QWidget(parent) {
  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 20, 50, 20);
  layout->setSpacing(0);

  ListWidget* list = new ListWidget(this);
  list->setSpacing(0);

  list->addItem(new ParamControl(
      "LongControlEnabled", "LONGITUDINAL CONTROL",
      "켜짐: 배선 개조가 완료된 현대·기아 차량에서 오픈파일럿이 가속과 제동을 제어합니다. 배선 개조가 안 된 차량에서는 켜지 마십시오.",
      "../assets/offroad/icon_road.png", this));
  list->addItem(horizontal_line());

  list->addItem(new ParamValueControlF(
      "TrafficStopMode", "E2E / ACC MODE",
      "0 ACC: 신호정지 끔 / 1 AUTO: 원거리 신호정지·출발준비에서 E2E / 2 APILOT: AUTO 조건과 비전 앞차까지 E2E.",
      "../assets/img_experimental_white.svg", 0, 2, 1, 0, 2, this));

  list->addItem(new ParamValueControlF(
      "ShowPlotMode", "DRIVING ANALYSIS GRAPH",
      "0 끔 / 1 가속도 / 2 속도·가속도 / 3 모델 / 4 앞차 / 5 앞차 저크 / 6 조향토크 / 7 조향각 / 8 곡률. EON에서는 선택한 그래프 하나만 10Hz로 표시합니다.",
      "../assets/offroad/icon_openpilot.png", 0, 8, 1, 0, 0, this));

  list->addItem(new ParamControl(
      "MixRadarInfo", "RADAR / VISION ACCEL BLEND",
      "켜짐: 레이더 앞차와 비전 모델의 가속도 변화를 혼합해 출발·감속 반응을 보완합니다. / 꺼짐: 레이더 정보를 우선합니다.",
      "../assets/offroad/icon_road.png"));

  list->addItem(new ParamValueControlF(
      "StartAccelApply", "START ACCEL",
      "정지 후 출발 가속도(×0.02m/s²)입니다. 값 증가(+): 더 빠르고 강하게 출발 / 값 감소(-): 천천히 출발 / 0: 추가 출발가속 끔.",
      "../assets/offroad/icon_openpilot.png", 0, 100, 5, 0, 0, this));

  list->addItem(new ParamValueControlF(
      "JerkStartLimit", "START JERK LIMIT",
      "정지 후 출발 초기 저크 제한(×0.1m/s³)입니다. 값 증가(+): 가속 반응이 빨라짐 / 값 감소(-): 출발이 부드러워짐. 권장값: 10.",
      "../assets/offroad/icon_openpilot.png", 5, 50, 1, 0, 10, this));

  list->addItem(new ParamValueControlF(
      "StopAccelApply", "STOPPING ACCEL",
      "정지 마무리 제동값(×-0.02m/s²)입니다. 값 증가(+): 정지 직전 제동이 강해짐 / 값 감소(-): 부드러워짐 / 0: 추가 제동 끔.",
      "../assets/offroad/icon_openpilot.png", 0, 100, 5, 0, 30, this));

  list->addItem(new ParamValueControlF(
      "StoppingDecelRate", "STOPPING DECEL RATE",
      "정지 마무리 제동이 증가하는 속도(×0.01m/s³)입니다. 값 증가(+): 제동이 빠르게 강해짐 / 값 감소(-): 정지 직전 제동이 부드러워짐. 기본값: 120.",
      "../assets/offroad/icon_openpilot.png", 20, 200, 1, 0, 120, this));

  list->addItem(new ParamValueControlF(
      "StandstillHoldApply", "STANDSTILL HOLD",
      "차량이 완전히 멈춘 뒤 유지하는 제동값(×-0.02m/s²)입니다. 정지 접근 제동에는 영향을 주지 않습니다. 기본값 55 = -1.10m/s².",
      "../assets/offroad/icon_openpilot.png", 10, 100, 5, 0, 55, this));

  list->addItem(new ParamValueControlF(
      "StandstillHoldRate", "STANDSTILL HOLD RATE",
      "완전정지 후 유지 제동값까지 증가하는 속도(×0.01m/s³)입니다. 값 증가(+): 더 빨리 고정 / 값 감소(-): 더 부드럽게 고정. 기본값: 120.",
      "../assets/offroad/icon_openpilot.png", 20, 200, 1, 0, 120, this));

  list->addItem(new ParamValueControlF(
      "SoftHoldMode", "SOFT HOLD MODE",
      "0: 끔, 1: 브레이크를 놓은 뒤 정지 유지, 2: aPilot SCC 호환 모드(일부 차량은 오토홀드/EPB가 작동할 수 있음). 가속페달 또는 RES/+로 해제합니다.",
      "../assets/offroad/icon_openpilot.png", 0, 2, 1, 0, 1, this));

  list->addItem(new ParamValueControlF(
      "TrafficStopAccel", "TRAFFIC STOP DECEL",
      "신호정지 감속 비율입니다. 값 증가(+): 늦고 강하게 감속 / 값 감소(-): 일찍 부드럽게 감속. aPilot 기본값: 80%.",
      "../assets/offroad/icon_road.png", 10, 120, 10, 0, 80, this));

  list->addItem(new ParamValueControlF(
      "TrafficStopDistanceAdjust", "TRAFFIC STOP DISTANCE ADJUST (cm)",
      "신호정지에만 적용됩니다. 양수(+): 정지선에 더 가까이 정차 / 음수(-): 정지선에서 더 멀리 정차. aPilot 기본값: +400cm.",
      "../assets/offroad/icon_road.png", -1000, 1000, 10, 0, 400, this));

  list->addItem(horizontal_line());

  const std::array<std::tuple<const char*, const char*, int>, 7> accel_controls = {{
    {"CruiseMaxVals1", "CRUISE MAX VALS1 (0 km/h)", 160},
    {"CruiseMaxVals20", "CRUISE MAX VALS20 (20 km/h)", 140},
    {"CruiseMaxVals2", "CRUISE MAX VALS2 (40 km/h)", 120},
    {"CruiseMaxVals3", "CRUISE MAX VALS3 (60 km/h)", 100},
    {"CruiseMaxVals4", "CRUISE MAX VALS4 (80 km/h)", 80},
    {"CruiseMaxVals5", "CRUISE MAX VALS5 (110 km/h)", 70},
    {"CruiseMaxVals6", "CRUISE MAX VALS6 (140 km/h)", 60},
  }};
  for (const auto& [key, title, default_value] : accel_controls) {
    list->addItem(new ParamValueControlF(
        key, title, "해당 속도 기준점의 최대 가속 상한값(×0.01m/s²)입니다. 중간 속도는 인접한 두 기준점 값을 보간합니다. 값 증가(+): 허용 가속이 크고 강해짐 / 값 감소(-): 허용 가속이 작고 부드러워짐. 주행 중 약 1초 내에 반영됩니다.",
        "../assets/offroad/icon_openpilot.png", 10, 250, 5, 0, default_value, this));
  }

  list->addItem(new ParamValueControlF(
      "NoLeadCruiseAccelFactor", "NO-LEAD CRUISE ACCEL (%)",
      "앞차가 없을 때 설정속도로 복귀하는 최대가속 비율입니다. CRUISE MAX 값에 이 비율을 곱하며, 설정속도에 가까워질수록 자동으로 더 낮아집니다. 값 증가(+): 빠른 속도 복귀 / 값 감소(-): 부드러운 속도 복귀. 권장값: 65%.",
      "../assets/offroad/icon_openpilot.png", 30, 100, 5, 0, 65, this));
  list->addItem(new ParamValueControlF(
      "NoLeadCruiseJerkLimit", "NO-LEAD ACCEL RAMP (X0.01m/s³)",
      "앞차가 없을 때 가속 명령이 증가하는 속도입니다. 값 증가(+): 가속이 빨리 강해짐 / 값 감소(-): 가속이 천천히 부드럽게 증가. 감속과 앞차 추종에는 적용하지 않습니다. 권장값: 25.",
      "../assets/offroad/icon_openpilot.png", 5, 100, 5, 0, 25, this));

  list->addItem(horizontal_line());

  const std::array<std::tuple<const char*, const char*, int>, 4> gap_controls = {{
    {"TFollowGap1", "T-FOLLOW GAP1", 110},
    {"TFollowGap2", "T-FOLLOW GAP2", 120},
    {"TFollowGap3", "T-FOLLOW GAP3", 140},
    {"TFollowGap4", "T-FOLLOW GAP4", 160},
  }};
  for (const auto& [key, title, default_value] : gap_controls) {
    list->addItem(new ParamValueControlF(
        key, title, "해당 GAP의 추종시간(×0.01초)입니다. 값 증가(+): 앞차와 멀어짐 / 값 감소(-): 앞차와 가까워짐.",
        "../assets/offroad/icon_openpilot.png", 70, 300, 1, 0, default_value, this));
  }

  list->addItem(new ParamValueControlF(
      "TFollowSpeedRatio", "HIGH-SPEED T-FOLLOW RATIO (%)",
      "속도가 높아질 때 차간시간을 늘리는 비율입니다. 값 증가(+): 고속에서 차간거리 증가 / 값 감소(-): 고속 차간거리 감소.",
      "../assets/offroad/icon_openpilot.png", 100, 300, 5, 0, 120, this));
  list->addItem(new ParamValueControlF(
      "TFollowDecelBoost", "DECEL T-FOLLOW BOOST (%)",
      "앞차 추종 중 내 차가 감속할 때만 차간시간을 조금 늘립니다. 값 증가(+): 재정지 앞차에 더 여유 있게 부드럽게 제동 / 0: 사용 안 함. 권장값: 30.",
      "../assets/offroad/icon_openpilot.png", 0, 100, 5, 0, 30, this));
  list->addItem(new ParamValueControlF(
      "RadarReactionFactor", "RADAR REACTION FACTOR (%)",
      "앞차 가감속이 유지될 것으로 예측하는 정도입니다. 100: 기본 반응 / 값 감소(-): 앞차 감속을 더 오래 예상해 일찍 제동. 권장값: 70.",
      "../assets/offroad/icon_road.png", 20, 200, 5, 0, 70, this));
  list->addItem(new ParamValueControlF(
      "PrevCruiseGap", "PREVIOUS CRUISE GAP",
      "마지막 GAP을 저장·복원합니다. 값 증가(+): 더 먼 GAP / 값 감소(-): 더 가까운 GAP.",
      "../assets/offroad/icon_openpilot.png", 1, 4, 1, 0, 4, this));
  list->addItem(new ParamValueControlF(
      "MySafeModeFactor", "SAFE T-FOLLOW RATIO (%)",
      "ECO·SAFE 모드의 차간거리 보정값입니다. 값 증가(+): 차간거리가 짧아짐 / 값 감소(-): 차간거리가 길어짐.",
      "../assets/offroad/icon_openpilot.png", 50, 100, 5, 0, 80, this));
  list->addItem(new ParamValueControlF(
      "MyEcoModeFactor", "ECO ACCEL RATIO (%)",
      "ECO 최대가속 비율이며 SAFE에도 함께 적용됩니다. 값 증가(+): 가속이 빨라짐 / 값 감소(-): 가속이 느려짐.",
      "../assets/offroad/icon_openpilot.png", 10, 95, 5, 0, 80, this));
  list->addItem(new ParamValueControlF(
      "InitMyDrivingMode", "INITIAL DRIVING MODE",
      "시동 후 시작 모드입니다. 1: SAFE / 2: ECO / 3: NORMAL / 4: FAST / 5: AUTO.",
      "../assets/offroad/icon_openpilot.png", 1, 5, 1, 0, 3, this));

  list->addItem(horizontal_line());

  list->addItem(new ParamValueControlF(
      "LongTuningKpV", "LONGITUDINAL KP", "현재 속도오차 반응값(×0.01)입니다. 값 증가(+): 가감속 반응이 빠르고 강해짐 / 값 감소(-): 반응이 부드럽고 느려짐.",
      "../assets/offroad/icon_openpilot.png", 0, 200, 5, 0, 100, this));
  list->addItem(new ParamValueControlF(
      "LongTuningKiV", "LONGITUDINAL KI", "누적 속도오차 보정값(×0.001)입니다. 값 증가(+): 지속 오차를 빨리 보정 / 값 감소(-): 천천히 보정. 과도하면 출렁일 수 있습니다.",
      "../assets/offroad/icon_openpilot.png", 0, 2000, 5, 0, 200, this));
  list->addItem(new ParamValueControlF(
      "LongTuningKf", "LONGITUDINAL KF", "목표 가속도 반영값(×0.01)입니다. 값 증가(+): 가감속 명령이 강해짐 / 값 감소(-): 명령이 약해짐.",
      "../assets/offroad/icon_openpilot.png", 0, 200, 5, 0, 100, this));
  list->addItem(new ParamValueControlF(
      "LongitudinalActuatorDelayLowerBound", "LONG ACTUATOR DELAY MIN",
      "가속·제동의 짧은 지연 보정값(×0.01초)입니다. 값 증가(+): 더 미리 반응 / 값 감소(-): 반응 시점을 늦춤 / 0: 차량 기본값.",
      "../assets/offroad/icon_openpilot.png", 0, 100, 5, 0, 0, this));
  list->addItem(new ParamValueControlF(
      "LongitudinalActuatorDelayUpperBound", "LONG ACTUATOR DELAY MAX",
      "가속·제동의 긴 지연 보정값(×0.01초)입니다. 값 증가(+): 더 먼 미래를 보고 일찍 반응 / 값 감소(-): 반응이 늦어짐 / 0: 차량 기본값.",
      "../assets/offroad/icon_openpilot.png", 0, 100, 5, 0, 0, this));

  list->addItem(horizontal_line());

  list->addItem(new ParamValueControlF(
      "ComfortBrake", "COMFORT BRAKE (X0.01m/s²)",
      "앞차에 접근할 때 상정하는 감속 능력입니다. 값 증가(+): 제동을 더 늦게 시작하고 강하게 / 값 감소(-): 더 일찍 부드럽게 감속. 앞차와 속도가 같을 때는 영향이 없습니다. 기본값: 250.",
      "../assets/offroad/icon_road.png", 150, 400, 5, 0, 250, this));

  list->addItem(new ParamValueControlF(
      "XEgoObstacleCost", "LEAD DISTANCE COST (X0.01)",
      "목표 차간거리 오차를 얼마나 급하게 없앨지 정합니다. 값 증가(+): 제동 시작이 이르고 단단해짐 / 값 감소(-): 제동이 늦고 부드러워짐. 기본값: 600.",
      "../assets/offroad/icon_road.png", 100, 1200, 25, 0, 600, this));

  list->addItem(new ParamValueControlF(
      "StopDistance", "STOP DISTANCE (cm)",
      "앞차 정지와 신호정지의 기본 여유거리입니다. 값 증가(+): 더 멀리 정차 / 값 감소(-): 더 가까이 정차. aPilot 기본값: 600cm.",
      "../assets/offroad/icon_road.png", 200, 1000, 50, 0, 600, this));

  ScrollView *scroller = new ScrollView(list, this);
  scroller->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  layout->addWidget(scroller);
}

/////////////////////////////////////////////////////////////////////////

UISettingsPanel::UISettingsPanel(QWidget* parent) : QWidget(parent) {
  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 20, 50, 20);
  layout->setSpacing(0);

  ListWidget* list = new ListWidget(this);
  list->setSpacing(0);

  list->addItem(new ParamControl(
      "ShowCarrotHud", "LEFT HUD PANEL",
      "켜짐: 속도·크루즈·GAP·기어·주행모드·제한속도 HUD를 표시합니다.",
      "../assets/offroad/icon_road.png", this));
  list->addItem(new ParamControl(
      "ShowMapboxMap", "RIGHT MAP IMAGE",
      "켜짐: 우측 Mapbox 지도와 NOO 구간의 티맵 지도 이미지를 표시합니다. / 꺼짐: 지도 이미지를 즉시 숨깁니다. 티맵 방향·거리·카메라 감속 기능은 유지됩니다.",
      "../assets/offroad/icon_road.png", this));
  list->addItem(new ParamControl(
      "ShowRouteMapAlways", "ALWAYS SHOW ROUTE MAP",
      "켜짐: 목적지 경로가 활성화된 동안 NOO 구간이 아니어도 우측 티맵 지도 이미지를 계속 표시합니다. / 꺼짐: NOO 진입 구간에서만 표시합니다. '우측 지도 이미지 표시'가 켜져 있어야 적용됩니다.",
      "../assets/offroad/icon_road.png", this));
  list->addItem(new ParamControl(
      "ShowGearAnimation", "GEAR POPUP ANIMATION",
      "켜짐: 변속단이 바뀔 때 중앙 팝업 애니메이션을 표시합니다.",
      "../assets/offroad/icon_road.png", this));
  list->addItem(horizontal_line());
  auto *status_color = new ParamControl(
      "ShowPathStatusColor", "PATH STATUS COLOR",
      "켜짐: 활성=녹색, 정속=노란색, 가속=주황색, 감속=빨간색, 비활성=검은색으로 표시합니다.",
      "../assets/offroad/icon_road.png", this);
  status_color->showDescription();
  list->addItem(status_color);
  list->addItem(new ParamValueControlF(
      "ShowPathWidth", "PATH WIDTH (cm)",
      "차량 중심에서 경로 한쪽 끝까지의 표시 폭입니다. 값 증가(+): 경로가 넓게 표시 / 값 감소(-): 좁게 표시. 제어 경로에는 영향이 없습니다.",
      "../assets/offroad/icon_road.png", 30, 150, 10, 0, 90, this));

  ScrollView *scroller = new ScrollView(list, this);
  scroller->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  layout->addWidget(scroller);
}

/////////////////////////////////////////////////////////////////////////

VIPPanel::VIPPanel(QWidget* parent) : QWidget(parent) {
  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setContentsMargins(50, 20, 50, 20);
  layout->setSpacing(0);

  ListWidget* list = new ListWidget(this);
  list->setSpacing(0);


  // ── 조향 실시간 튜닝 (nTune 파일 직접 조절) ───────────────────
  list->addItem(horizontal_line());

  list->addItem(new ParamValueControlF("CustomSteerRatio",
      "SR",
      "조향비(×0.01)입니다. 값 증가(+): 같은 커브에서 조향량이 커짐 / 값 감소(-): 조향량이 작아짐.\n"
      "실시간 조향비 사용이 켜져 있으면 이 값 대신 학습값을 사용합니다.\n"
      "범위: 1000 ~ 2000  /  기본값: 1650 (=16.50)",
      "../assets/offroad/icon_openpilot.png", 1000, 2000, 10, 0, 1650, this));

  list->addItem(new ParamControl("UseLiveSteerRatio",
      "LIVE STEER RATIO",
      "켜짐: liveParameters가 학습한 조향비 사용 / 꺼짐: 위의 고정 조향비 사용.",
      "../assets/offroad/icon_openpilot.png", this));

  list->addItem(new ParamValueControlF("SteerActuatorDelay",
      "SAD",
      "조향 지연 보정값(×0.01초)입니다. 값 증가(+): 더 미리 조향 / 값 감소(-): 조향 시작이 늦어짐.\n"
      "너무 높으면 커브 안쪽으로 치우치고 너무 낮으면 바깥쪽으로 밀릴 수 있습니다.\n"
      "범위: 0 ~ 80  /  기본값: 25 (=0.25초, 차량 정의값)",
      "../assets/offroad/icon_openpilot.png", 0, 80, 1, 0, 25, this));

  list->addItem(new ParamValueControlF("MpcPathCost",
      "MPC PATH COST (X0.001)",
      "모델 경로를 따라가려는 비용입니다. 값 증가(+): 경로 추종이 강해짐 / 값 감소(-): 움직임이 부드러워짐.\n"
      "표시값 1000 = 실제 1.000  /  기존 기본값: 1000  /  범위: 100 ~ 5000",
      "../assets/offroad/icon_openpilot.png", 100, 5000, 50, 0, 1000, this));

  list->addItem(new ParamValueControlF("MpcLateralMotionCost",
      "MPC LATERAL MOTION COST (X0.001)",
      "차량의 좌우 이동 자체를 억제하는 비용입니다. 값 증가(+): 안정적이지만 경로 변화가 느림 / 값 감소(-): 빠르게 이동.\n"
      "표시값 110 = 실제 0.110  /  기존 기본값: 110  /  범위: 0 ~ 1000",
      "../assets/offroad/icon_openpilot.png", 0, 1000, 10, 0, 110, this));

  list->addItem(new ParamValueControlF("MpcLateralAccelCost",
      "MPC LATERAL ACCEL COST (X0.001)",
      "횡가속을 억제하는 비용입니다. 값 증가(+): 완만한 조향 / 값 감소(-): 경로 변화에 적극 대응.\n"
      "표시값 0 = 실제 0.000  /  기존 기본값: 0  /  범위: 0 ~ 1000",
      "../assets/offroad/icon_openpilot.png", 0, 1000, 10, 0, 0, this));

  list->addItem(new ParamValueControlF("MpcLateralJerkCost",
      "MPC LATERAL JERK COST (X0.001)",
      "횡가속도의 급격한 변화를 억제하는 비용입니다. 값 증가(+): 부드러움 / 값 감소(-): 반응이 빨라짐.\n"
      "표시값 40 = 실제 0.040  /  기존 기본값: 40  /  범위: 0 ~ 500",
      "../assets/offroad/icon_openpilot.png", 0, 500, 5, 0, 40, this));

  list->addItem(new ParamValueControlF("SteeringRateCost",
      "MPC STEERING RATE COST",
      "목표 조향의 급격한 변화를 억제하는 MPC 비용값입니다.\n"
      "값 감소(-): 차선·경로 변화에 빠르게 반응 / 값 증가(+): 부드럽지만 반응이 느려짐.\n"
      "제네시스 DH 권장값: 550  /  기존 고정값: 700  /  범위: 200 ~ 1200",
      "../assets/offroad/icon_openpilot.png", 200, 1200, 25, 0, 550, this));

  list->addItem(new ParamControl("LateralTorqueCustom",
      "CUSTOM TORQUE CONTROL",
      "켜짐: 아래 토크 수동값 적용 / 꺼짐: 차량 정의값 사용.\n"
      "차량 정의값 = torque_data/params.yaml 의 GENESIS 2015-2016 (FACTOR 2.747 / FRICTION 0.098)\n"
      "+ kp 1.00 / ki 0.10 / kf 1.00. 코드에 있는 값이라 화면에서는 못 바꿉니다.",
      "../assets/offroad/icon_openpilot.png", this));
  
  list->addItem(new NtuneValueControl("torque", "latAccelFactor",
      "LAT ACCEL FACTOR",
      "횡가속도 대비 토크 계수입니다. 값 증가(+): 조향이 약해짐 / 값 감소(-): 조향이 강해짐.\n"
      "범위: 0.50 ~ 4.50  /  기본값: 2.747 (DH 차량 정의값)",
      "../assets/offroad/icon_openpilot.png", 0.5, 4.5, 0.01, 3, 2.747, this));

  list->addItem(new NtuneValueControl("torque", "friction",
      "FRICTION",
      "정지마찰 보상값입니다. 값 증가(+): 중앙 부근 조향 반응이 빨라짐 / 값 감소(-): 반응이 부드럽고 느려짐.\n"
      "너무 크면 직진에서 좌우로 흔들리고, MDPS 부하가 커집니다.\n"
      "범위: 0.000 ~ 0.200  /  기본값: 0.098 (DH 차량 정의값)",
      "../assets/offroad/icon_openpilot.png", 0.0, 0.2, 0.005, 3, 0.098, this));

  list->addItem(new ParamValueControlF("LateralTorqueKpV",
      "TORQUE KP", "현재 조향오차 반응값(×0.01)입니다. 값 증가(+): 조향 반응이 강하고 빠름 / 값 감소(-): 부드럽고 느림. 기본값: 100 (=1.00, 차량 정의값과 동일).",
      "../assets/offroad/icon_openpilot.png", 0, 500, 5, 0, 100, this));

  list->addItem(new ParamValueControlF("LateralTorqueKiV",
      "TORQUE KI", "누적 조향오차 보정값(×0.01)입니다. 값 증가(+): 지속 오차를 빨리 보정 / 값 감소(-): 천천히 보정.\n"
      "값이 크면 긴 커브에서 적분이 쌓여 안쪽으로 파고들 수 있습니다.\n"
      "DH 차량 정의 기본값: 10 (=0.10).",
      "../assets/offroad/icon_openpilot.png", 0, 200, 1, 0, 10, this));

  list->addItem(new ParamValueControlF("LateralTorqueKf",
      "TORQUE KF", "목표 조향토크 반영값(×0.01)입니다. 값 증가(+): 전체 조향 명령이 강해짐 / 값 감소(-): 약해짐. 기본값: 100 (=1.00, 차량 정의값과 동일).",
      "../assets/offroad/icon_openpilot.png", 0, 200, 5, 0, 100, this));

  list->addItem(new ParamValueControlF("LateralTorqueKd",
      "TORQUE KD", "급격한 조향변화 억제값(×0.01)입니다. 값 증가(+): 변화가 억제되어 안정적이나 둔해짐 / 값 감소(-): 반응이 빨라짐. 기본값: 0.",
      "../assets/offroad/icon_openpilot.png", 0, 200, 5, 0, 0, this));

  list->addItem(new ParamValueControlF("LatAccelFrictionFactor",
      "LAT ACCEL FRICTION FACTOR",
      "횡가속도 오차를 마찰보상에 반영하는 비율(×0.01)입니다. 값 증가(+): 커브 조향 반응이 강해짐 / 값 감소(-): 부드러워짐. 기본값: 70.",
      "../assets/offroad/icon_openpilot.png", 0, 300, 5, 0, 70, this));

  list->addItem(new ParamValueControlF("LatJerkFrictionFactor",
      "LAT JERK FRICTION FACTOR",
      "예측 횡저크 반영비율(×0.01)입니다. 값 증가(+): 커브 진입 조향이 빨라짐 / 값 감소(-): 진입 반응이 느려짐 / 0: 사용 안 함. 기본값: 40.",
      "../assets/offroad/icon_openpilot.png", 0, 200, 5, 0, 40, this));

  list->addItem(horizontal_line());



  list->addItem(horizontal_line());

  // ── Offset Total ─────────────────────────────────────────────
  // 레인모드 + 레인리스 모드 모두 적용. 0.01m 단위, -1.00 ~ +1.00m
  list->addItem(horizontal_line());

  auto *path_offset = new OffsetTotalControl(
      "통합 경로 좌우보정",
      "주행 경로 좌우 통합 보정값입니다. 레인모드·레인리스 모두 적용됩니다.\n"
      "카메라 오프셋은 하드웨어 기본값으로 고정되고 이 값 하나로 조정합니다.\n"
      "왼쪽으로 이동: 양수(+) / 오른쪽으로 이동: 음수(−)\n"
      "범위: −1.00 ~ +1.00m  /  기본값: 0.00m",
      "../assets/offroad/icon_road.png",
      this);
  path_offset->showDescription();
  list->addItem(path_offset);

  list->addItem(horizontal_line());

  // ── Adjust Lane Offset ───────────────────────────────────────
  auto *lane_offset = new AdjustLaneOffsetControl(
      "차선 여유공간 자동보정",
      "좌우 여유공간이 비대칭일 때 여유 있는 쪽으로 경로를 옮깁니다.\n"
      "좁은 도로에서 대형차 옆을 지날 때 효과가 있습니다.\n"
      "양쪽 다 여유가 있거나 양쪽 다 좁으면 동작하지 않습니다.\n"
      "값 증가(+): 여유 있는 쪽으로 더 많이 이동 / 값 감소(-): 이동량이 줄어듭니다.\n"
      "범위: 0 ~ 40cm (5cm 단위)  /  기본값: OFF",
      "../assets/offroad/icon_road.png",
      this);
  lane_offset->showDescription();
  list->addItem(lane_offset);

  list->addItem(horizontal_line());

  // ── Laneless Offset ──────────────────────────────────────────
  auto *laneless_offset = new LanelessOffsetControl(
      "레인리스 좌우보정",
      "차선을 쓰지 않는 레인리스 구간에서만 적용되는 좌우 보정입니다.\n"
      "레인모드는 직진인데 레인리스에서만 한쪽으로 쏠릴 때 사용합니다.\n"
      "차선이 잡히는 비중만큼 자동으로 줄어들어 레인모드 주행에는 영향이 없습니다.\n"
      "왼쪽으로 이동: 양수(+) / 오른쪽으로 이동: 음수(−)\n"
      "범위: −30 ~ +30cm (1cm 단위)  /  기본값: OFF",
      "../assets/offroad/icon_road.png",
      this);
  laneless_offset->showDescription();
  list->addItem(laneless_offset);


  list->addItem(horizontal_line());
  auto *dlp_control = new DynamicLaneProfileControl(
      "DYNAMIC LANE PROFILE",
      "차선 사용: 항상 차선 기반 / 차선 미사용: 항상 E2E 경로 / 자동: 차선 인식률에 따라 자동 전환.",
      "../assets/offroad/icon_road.png",
      this);
  dlp_control->showDescription();
  list->addItem(dlp_control);

  list->addItem(horizontal_line());

  // ── AutoLaneChangeTimer ──────────────────────────────────────
  auto *lc_timer = new AutoLaneChangeTimerControl(
      "AUTO LANE CHANGE DELAY",
      "차선변경 자동 시작까지의 대기 시간을 설정합니다.\n"
      "값 증가(+): 방향지시등 후 더 오래 기다림 / 값 감소(-): 더 빨리 차선변경 / 즉시: 조건 충족 즉시 시작.",
      "../assets/offroad/icon_road.png",
      this);
  lc_timer->showDescription();
  list->addItem(lc_timer);

  // ── AutoLaneChangeSpeed ────────────────────────────────────────
  list->addItem(new ParamValueControlF("AutoLaneChangeSpeed",
      "AUTO LANE CHANGE MIN SPEED",
      "자동·방향지시등 차선변경 허용 최저속도(km/h)입니다. 값 증가(+): 더 높은 속도에서만 작동 / 값 감소(-): 저속에서도 작동.",
      "../assets/offroad/icon_road.png", 0, 100, 10, 0, 50, this));

  list->addItem(horizontal_line());

  list->addItem(new ParamControl(
      "LaneChangeEnabled", "LANE CHANGE ASSIST",
      "켜짐: 방향지시등과 운전자 조향 입력으로 차선변경을 보조합니다. 주변 차량의 안전 여부는 운전자가 직접 확인해야 합니다.",
      "../assets/offroad/icon_road.png", this));
  list->addItem(new ParamControl(
      "AutoLaneChangeEnabled", "AUTO LANE CHANGE",
      "켜짐: 방향지시등 작동 후 별도의 핸들 입력 없이 차선변경을 시작합니다. 시험 기능이므로 주변을 직접 확인하십시오.",
      "../assets/offroad/icon_road.png", this));
  list->addItem(new ParamControl(
      "KeepSteeringTurnSignals", "KEEP STEERING WITH BLINKER",
      "켜짐: 방향지시등 작동 중에도 조향 제어를 유지합니다. / 꺼짐: 차량 조건에 따라 조향이 제한될 수 있습니다.",
      "../assets/offroad/icon_openpilot.png", this));

  ScrollView *scroller = new ScrollView(list, this);
  scroller->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  layout->addWidget(scroller);
}

#include "selfdrive/ui/qt/home.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QScrollArea>
#include <QScroller>
#include "selfdrive/ui/qt/offroad/experimental_mode.h"
#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/drive_stats.h"
#include "selfdrive/ui/qt/widgets/prime.h"

// ── CarrotPilot Auto-Tuner: Params 저장 헬퍼 ─────────────────────────────
// Python carrot_learning.py의 배율과 동일하게 저장한다.
static bool writeTorqueParamValue(Params &params, const QString &key, double value) {
  if (key == "latAccelFactor") {
    params.put("LateralTorqueAccelFactor", QString::number(value * 1000.0, 'f', 0).toStdString());
  } else if (key == "friction") {
    params.put("LateralTorqueFriction", QString::number(value * 1000.0, 'f', 0).toStdString());
  } else {
    return false;
  }
  params.putBool("LateralTorqueCustom", true);
  return true;
}

static bool writeCommonParamValue(Params &params, const QString &key, double value) {
  if (key == "steerActuatorDelay") {
    params.put("SteerActuatorDelay", QString::number(value * 100.0, 'f', 0).toStdString());
  } else if (key == "steerRatio") {
    params.put("CustomSteerRatio", QString::number(value * 100.0, 'f', 0).toStdString());
  } else {
    return false;
  }
  return true;
}

// ── CarrotPilot Auto-Tuner (commit 9dd5e2c port) ─────────────────────────

// AutoTunerGuideDialog
AutoTunerGuideDialog::AutoTunerGuideDialog(const QString &html_content, QWidget *parent) : QDialogBase(parent) {
  setWindowFlags(Qt::Popup | Qt::FramelessWindowHint);
  setAttribute(Qt::WA_TranslucentBackground);
  setStyleSheet(R"(
    QDialogBase { background: transparent; }
    #container { background-color: #1b1b1b; border-radius: 20px; }
    QLabel { color: #dddddd; font-size: 45px; margin: 20px; }
    QPushButton { padding: 20px; height: 100px; font-size: 45px; border-radius: 10px; color: white; background-color: #465BEA; }
    QPushButton:pressed { background-color: #3049F4; }
  )");

  QVBoxLayout *outer_layout = new QVBoxLayout(this);
  outer_layout->setContentsMargins(40, 40, 40, 40);

  QFrame *container = new QFrame(this);
  container->setObjectName("container");
  QVBoxLayout *main_layout = new QVBoxLayout(container);
  main_layout->setContentsMargins(20, 20, 20, 20);

  QLabel *text = new QLabel(html_content);
  text->setWordWrap(true);
  text->setAlignment(Qt::AlignTop | Qt::AlignLeft);

  QScrollArea *scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setStyleSheet("QScrollArea { background: transparent; } QWidget { background: transparent; }");
  QScroller::grabGesture(scroll->viewport(), QScroller::LeftMouseButtonGesture);
  scroll->setWidget(text);
  main_layout->addWidget(scroll, 1);

  QPushButton *btn_ok = new QPushButton("확인");
  btn_ok->setFixedWidth(400);
  main_layout->addWidget(btn_ok, 0, Qt::AlignCenter);

  outer_layout->addWidget(container);
  connect(btn_ok, &QPushButton::clicked, this, &QDialog::accept);
}

void AutoTunerGuideDialog::showEvent(QShowEvent *event) {
  setMainWindow(this);
  QDialog::showEvent(event);
}

// AutoTunerDialog
AutoTunerDialog::AutoTunerDialog(const QString &title_text, const QJsonObject &recs, QWidget *parent)
    : QDialogBase(parent), recommendations(recs) {
  setWindowFlags(Qt::Popup | Qt::FramelessWindowHint);
  setAttribute(Qt::WA_TranslucentBackground);
  setStyleSheet(R"(
    QDialogBase { background: transparent; }
    #container { background-color: #2b2b2b; border-radius: 30px; border: 2px solid #555555; }
    QLabel { color: white; }
    QCheckBox { font-size: 45px; color: white; spacing: 20px; }
    QCheckBox::indicator { width: 50px; height: 50px; }
    QPushButton { padding: 25px; font-size: 45px; font-weight: 500; border-radius: 10px; color: white; background-color: #444444; }
    QPushButton:pressed { background-color: #333333; }
  )");

  QVBoxLayout *outer_layout = new QVBoxLayout(this);
  outer_layout->setContentsMargins(150, 40, 150, 40);

  QFrame *container = new QFrame(this);
  container->setObjectName("container");
  QVBoxLayout *main_layout = new QVBoxLayout(container);
  main_layout->setContentsMargins(40, 30, 40, 30);
  main_layout->setSpacing(15);

  QLabel *title = new QLabel(title_text);
  title->setStyleSheet("font-size: 55px; font-weight: bold; margin-bottom: 10px;");
  title->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(title);

  QScrollArea *scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setStyleSheet("QScrollArea { background: transparent; } QWidget { background: transparent; }");
  QScroller::grabGesture(scroll->viewport(), QScroller::LeftMouseButtonGesture);

  QWidget *scroll_widget = new QWidget();
  QVBoxLayout *scroll_layout = new QVBoxLayout(scroll_widget);
  scroll_layout->setContentsMargins(0, 0, 0, 0);
  scroll_layout->setSpacing(15);

  for (const QString& group : recommendations.keys()) {
    QJsonObject group_items = recommendations[group].toObject();
    QString short_group = group.split(" ").first();

    for (const QString& key : group_items.keys()) {
      QJsonObject info = group_items[key].toObject();

      // float 파라미터(OffsetTotal, latAccelFactor, steerActuatorDelay 등)와
      // int 파라미터(TFollowGap, Turn*)를 구분하여 표시
      bool is_float = info["is_float"].toBool(false);
      QString cur_str, rec_str;
      if (is_float) {
        cur_str = QString::number(info["current"].toDouble(), 'f', 3);
        rec_str = QString::number(info["recommended"].toDouble(), 'f', 3);
      } else {
        cur_str = QString::number(info["current"].toInt());
        rec_str = QString::number(info["recommended"].toInt());
      }

      QString item_text = QString("<span style='color:#aaaaaa;'>[%1]</span> <b>%2</b> "
                                  "<span style='font-size:40px; color:#bbbbbb;'>[%3]</span> &nbsp;:&nbsp; "
                                  "%4 ➔ <span style='color:#00ff00; font-weight:bold;'>%5</span>")
                                  .arg(short_group)
                                  .arg(key)
                                  .arg(info["band_kph"].toString())
                                  .arg(cur_str)
                                  .arg(rec_str);

      QHBoxLayout *item_layout = new QHBoxLayout();
      item_layout->setContentsMargins(0, 0, 0, 15);
      item_layout->setSpacing(20);

      QCheckBox *item_cb = new QCheckBox();
      item_cb->setChecked(true);
      item_cb->setStyleSheet("QCheckBox::indicator { width: 50px; height: 50px; }");

      QLabel *item_label = new QLabel(item_text);
      item_label->setStyleSheet("font-size: 45px; color: white;");
      item_label->setWordWrap(true);

      item_layout->addWidget(item_cb);
      item_layout->addWidget(item_label, 1);

      scroll_layout->addLayout(item_layout);
      item_checkboxes[key] = item_cb;
    }
  }
  scroll_layout->addStretch();
  scroll->setWidget(scroll_widget);
  main_layout->addWidget(scroll, 1);

  QHBoxLayout *btn_layout = new QHBoxLayout();

  QPushButton *btn_guide = new QPushButton("사용 안내 (Guide)");
  btn_guide->setStyleSheet("background-color: #3b5998;");

  QPushButton *btn_later = new QPushButton("나중에 (Later)");
  btn_later->setStyleSheet("background-color: #555555;");

  QPushButton *btn_clear = new QPushButton("학습 초기화 (Clear)");
  btn_clear->setStyleSheet("background-color: #8a1d1d;");

  QPushButton *btn_apply = new QPushButton("선택 적용 (Apply Selected)");
  btn_apply->setStyleSheet("background-color: #178644;");

  btn_layout->addWidget(btn_guide);
  btn_layout->addWidget(btn_later);
  btn_layout->addWidget(btn_clear);
  btn_layout->addWidget(btn_apply);
  main_layout->addLayout(btn_layout);

  outer_layout->addWidget(container);

  connect(btn_guide, &QPushButton::clicked, this, [=]() {
    // 포팅판 가이드: 이 포크에서 실제 학습되는 항목만 안내
    QString guide_html = R"(
    <div style='font-size: 45px;'>
    <div style='text-align:center; font-size: 55px; font-weight: bold; margin-bottom: 20px;'>🥕 Auto-Tuner 사용 안내</div><hr>
    <div style='font-size: 50px; font-weight: bold; margin-top: 20px; margin-bottom: 10px;'>📊 데이터 수집 및 적용 방식</div>
    <ul>
    <li><b>주행 중 데이터 수집</b>: 인게이지 상태에서 <b>오버라이드(가속 페달)</b>와 <b>개입(브레이크/조향)</b> 순간을 중점 수집합니다.</li>
    <li><b>패턴 분석</b>: 현재 설정값과 운전자 성향의 차이를 분석하여 이상적인 파라미터를 계산합니다.</li>
    <li><b>추천 및 적용</b>: 주차(P단) 시 팝업으로 추천값을 안내하며, <b>[선택 적용]</b>을 누르면 즉시 반영됩니다.</li>
    </ul><hr>
    <div style='font-size: 50px; font-weight: bold; margin-top: 20px; margin-bottom: 10px;'>⚙️ 그룹별 튜닝 항목</div>
    <b>🛣️ [거리] TFollowGap1~4</b><br>
    크루즈 GAP 단계별 추종 거리 시간(x0.01초). 추종 중 가속 페달을 자주 밟으면(거리가 넓다고 판단) 감소,
    브레이크를 자주 밟으면 증가 추천. 최소 0.90초 보장.<br><br>
    <b>🔄 [조향] OffsetTotal / latAccelFactor / friction / steerActuatorDelay</b><br>
    - <b>OffsetTotal</b>(m): 직진 평균 편차가 누적되면 경로 좌우 보정.<br>
    - <b>latAccelFactor</b>: 커브 조향 개입 방향에 따라 조향력 강도 조정 (Params).<br>
    - <b>friction</b>: 직선 미세 개입이 잦으면 마찰보상 상향 (Params).<br>
    - <b>steerActuatorDelay</b>: 커브 개입이 잦으면(반응 느림) 조향 지연을 낮춰 더 빠르게 (Params).<br>
    <hr>
    <div style='font-size: 50px; font-weight: bold; margin-top: 20px; margin-bottom: 10px;'>💡 참고</div>
    - <b>CarrotLearningAutoApply=1</b> 설정 시 팝업 없이 P단 전환 때 자동 적용됩니다.<br>
    - <b>[학습 초기화]</b>는 추천을 적용하지 않고 누적 데이터만 삭제합니다.<br>
    - 조향 학습값은 Params에 저장되어 제어기에 실시간 반영됩니다.<br>
    - 적용 이력은 CarrotLearningHistory 파라미터(JSON, 최대 50개)에서 확인할 수 있습니다.
    </div>
    )";
    AutoTunerGuideDialog *d = new AutoTunerGuideDialog(guide_html, this);
    d->exec();
    d->deleteLater();
  });

  connect(btn_later, &QPushButton::clicked, this, &QDialog::reject);

  connect(btn_clear, &QPushButton::clicked, [=]() {
    if (ConfirmationDialog::confirm("적용하지 않고 현재까지의 모든 학습 데이터를 삭제하시겠습니까?", this)) {
      Params().putBool("CarrotLearningClear", true);
      this->reject();
    }
  });

  connect(btn_apply, &QPushButton::clicked, this, &QDialog::accept);
}

QJsonObject AutoTunerDialog::getSelectedItems() {
  QJsonObject selected;
  for (const QString& group : recommendations.keys()) {
    QJsonObject group_items = recommendations[group].toObject();
    QJsonObject selected_group_items;

    for (const QString& key : group_items.keys()) {
      if (item_checkboxes.contains(key) && item_checkboxes[key]->isChecked()) {
        selected_group_items[key] = group_items[key];
      }
    }

    if (!selected_group_items.isEmpty()) {
      selected[group] = selected_group_items;
    }
  }
  return selected;
}
// ─────────────────────────────────────────────────────────────────────────

// HomeWindow: the container for the offroad and onroad UIs

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  slayout = new QStackedLayout();
  main_layout->addLayout(slayout);

  home = new OffroadHome(this);
  QObject::connect(home, &OffroadHome::openSettings, this, &HomeWindow::openSettings);
  slayout->addWidget(home);

  onroad = new OnroadWindow(this);
  slayout->addWidget(onroad);

  body = new BodyWindow(this);
  slayout->addWidget(body);
  body->setEnabled(false);

  driver_view = new DriverViewWindow(this);
  connect(driver_view, &DriverViewWindow::done, [=] {
    showDriverView(false);
  });
  slayout->addWidget(driver_view);
  setAttribute(Qt::WA_NoSystemBackground);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &HomeWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &HomeWindow::offroadTransition);
}

void HomeWindow::showSidebar(bool show) {
  sidebar->setVisible(show);
}

void HomeWindow::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  // switch to the generic robot UI
  if (onroad->isVisible() && !body->isEnabled() && sm["carParams"].getCarParams().getNotCar()) {
    body->setEnabled(true);
    slayout->setCurrentWidget(body);
  }

  // ── CarrotPilot Auto-Tuner: P단 전환 추천 팝업 (1초 주기로 체크) ──
  // python(carrot_learning.py)이 P단 전환 시 CarrotLearningPopupReady=1 을 세팅.
  // AutoApply=1 이면 python 쪽에서 이미 적용 완료 후 신호를 내리므로 여기까지 오지 않음.
  static int carrot_tuner_frame = 0;
  if (carrot_tuner_frame++ % 20 == 0) {
    Params params;
    if (params.getBool("CarrotLearningPopupReady")) {
      // 중복 팝업 방지를 위해 즉시 플래그 해제
      params.putBool("CarrotLearningPopupReady", false);

      QString raw = QString::fromStdString(params.get("CarrotLearningRecommend"));
      QJsonDocument doc = QJsonDocument::fromJson(raw.toUtf8());
      if (!raw.isEmpty() && doc.isObject()) {
        QJsonObject obj = doc.object();

        AutoTunerDialog *dialog = new AutoTunerDialog("Auto-Tuner: 주행 패턴 학습 완료!", obj, this);

        connect(dialog, &QDialog::accepted, [=]() {
          Params p;
          QJsonObject selected = dialog->getSelectedItems();
          if (!selected.isEmpty()) {
            // 1) 이력 기록 (python 측 포맷과 동일: id / timestamp / changes)
            QJsonArray history_array;
            QString history_raw = QString::fromStdString(p.get("CarrotLearningHistory"));
            if (!history_raw.isEmpty()) {
              QJsonDocument h_doc = QJsonDocument::fromJson(history_raw.toUtf8());
              if (h_doc.isArray()) history_array = h_doc.array();
            }
            QJsonObject history_entry;
            history_entry["timestamp"] = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
            history_entry["changes"] = selected;
            history_entry["id"] = QString::number(QDateTime::currentMSecsSinceEpoch());

            history_array.prepend(history_entry);
            while (history_array.size() > 50) history_array.removeLast();
            p.put("CarrotLearningHistory",
                  QJsonDocument(history_array).toJson(QJsonDocument::Compact).toStdString());

            // 2) 선택된 파라미터 적용 (토크/공통 학습값은 Params 배율로 저장)
            for (const QString& group : selected.keys()) {
              QJsonObject group_items = selected[group].toObject();
              for (const QString& key : group_items.keys()) {
                QJsonObject info = group_items[key].toObject();
                if (info["ntune"].toString() == "torque") {
                  writeTorqueParamValue(p, key, info["recommended"].toDouble());
                } else if (info["ntune"].toString() == "common") {
                  writeCommonParamValue(p, key, info["recommended"].toDouble());
                } else if (info["is_float"].toBool(false)) {
                  double rec = info["recommended"].toDouble();
                  p.put(key.toStdString(), QString::number(rec, 'f', 3).toStdString());
                } else {
                  int rec = info["recommended"].toInt();
                  p.put(key.toStdString(), std::to_string(rec));
                }
              }
            }
          }
          // 3) 누적 학습 데이터 초기화 신호 (python 쪽에서 처리) + 추천 제거
          Params().putBool("CarrotLearningClear", true);
          Params().remove("CarrotLearningRecommend");
          dialog->deleteLater();
        });

        connect(dialog, &QDialog::rejected, [=]() {
          dialog->deleteLater();
        });

        setMainWindow(dialog);
      }
    }
  }
}

void HomeWindow::offroadTransition(bool offroad) {
  sidebar->setVisible(offroad);
  if (offroad) {
    slayout->setCurrentWidget(home);
  } else {
    slayout->setCurrentWidget(onroad);
  }
}

void HomeWindow::showDriverView(bool show) {
  if (show) {
    emit closeSettings();
    slayout->setCurrentWidget(driver_view);
  } else {
    slayout->setCurrentWidget(home);
  }
  sidebar->setVisible(show == false);
}

void HomeWindow::mouseReleaseEvent(QMouseEvent* e) {
  // Handle sidebar collapsing
  if ((onroad->isVisible() || body->isVisible()) && (!sidebar->isVisible() || e->x() > sidebar->width())) {
    sidebar->setVisible(!sidebar->isVisible() && !onroad->isMapVisible());
  }
}

void HomeWindow::mouseDoubleClickEvent(QMouseEvent* e) {
  const SubMaster &sm = *(uiState()->sm);
  if (sm["carParams"].getCarParams().getNotCar()) {
    if (onroad->isVisible()) {
      slayout->setCurrentWidget(body);
    } else if (body->isVisible()) {
      slayout->setCurrentWidget(onroad);
    }
  }
}

// OffroadHome: the offroad home page

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(40, 40, 40, 45);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();
  header_layout->setContentsMargins(15, 15, 15, 0);
  header_layout->setSpacing(16);

  date = new QLabel();
  header_layout->addWidget(date, 1, Qt::AlignHCenter | Qt::AlignLeft);

  update_notif = new QPushButton("UPDATE");
  update_notif->setVisible(false);
  update_notif->setStyleSheet("background-color: #364DEF;");
  QObject::connect(update_notif, &QPushButton::clicked, [=]() { center_layout->setCurrentIndex(1); });
  header_layout->addWidget(update_notif, 0, Qt::AlignHCenter | Qt::AlignRight);

  alert_notif = new QPushButton();
  alert_notif->setVisible(false);
  alert_notif->setStyleSheet("background-color: #E22C2C;");
  QObject::connect(alert_notif, &QPushButton::clicked, [=] { center_layout->setCurrentIndex(2); });
  header_layout->addWidget(alert_notif, 0, Qt::AlignHCenter | Qt::AlignRight);

  header_layout->addWidget(new QLabel(getBrandVersion()), 0, Qt::AlignHCenter | Qt::AlignRight);

  main_layout->addLayout(header_layout);

  // main content
  main_layout->addSpacing(25);
  center_layout = new QStackedLayout();

  // Vertical experimental button and drive stats layout
  QWidget* statsAndExperimentalModeButtonWidget = new QWidget(this);
  QVBoxLayout* statsAndExperimentalModeButton = new QVBoxLayout(statsAndExperimentalModeButtonWidget);
  statsAndExperimentalModeButton->setSpacing(30);
  statsAndExperimentalModeButton->setMargin(0);

  ExperimentalModeButton *experimental_mode = new ExperimentalModeButton(this);
  QObject::connect(experimental_mode, &ExperimentalModeButton::openSettings, this, &OffroadHome::openSettings);

  statsAndExperimentalModeButton->addWidget(experimental_mode, 1);
  statsAndExperimentalModeButton->addWidget(new DriveStats, 1);

  // Horizontal experimental + drive stats and setup widget
  QWidget* statsAndSetupWidget = new QWidget(this);
  QHBoxLayout* statsAndSetup = new QHBoxLayout(statsAndSetupWidget);
  statsAndSetup->setMargin(0);
  statsAndSetup->setSpacing(30);
  statsAndSetup->addWidget(statsAndExperimentalModeButtonWidget, 1);
  statsAndSetup->addWidget(new SetupWidget);

  center_layout->addWidget(statsAndSetupWidget);

  // add update & alerts widgets
  update_widget = new UpdateAlert();
  QObject::connect(update_widget, &UpdateAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(update_widget);
  alerts_widget = new OffroadAlert();
  QObject::connect(alerts_widget, &OffroadAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(alerts_widget);

  main_layout->addLayout(center_layout, 1);

  // set up refresh timer
  timer = new QTimer(this);
  timer->callOnTimeout(this, &OffroadHome::refresh);

  setStyleSheet(R"(
    * {
     color: white;
    }
    OffroadHome {
      background-color: black;
    }
    OffroadHome > QPushButton {
      padding: 15px 30px;
      border-radius: 5px;
      font-size: 40px;
      font-weight: 500;
    }
    OffroadHome > QLabel {
      font-size: 55px;
    }
  )");
}

void OffroadHome::showEvent(QShowEvent *event) {
  refresh();
  timer->start(10 * 1000);
}

void OffroadHome::hideEvent(QHideEvent *event) {
  timer->stop();
}

void OffroadHome::refresh() {
  date->setText(QDateTime::currentDateTime().toString("dddd, MMMM d"));

  bool updateAvailable = update_widget->refresh();
  int alerts = alerts_widget->refresh();

  // pop-up new notification
  int idx = center_layout->currentIndex();
  if (!updateAvailable && !alerts) {
    idx = 0;
  } else if (updateAvailable && (!update_notif->isVisible() || (!alerts && idx == 2))) {
    idx = 1;
  } else if (alerts && (!alert_notif->isVisible() || (!updateAvailable && idx == 1))) {
    idx = 2;
  }
  center_layout->setCurrentIndex(idx);

  update_notif->setVisible(updateAvailable);
  alert_notif->setVisible(alerts);
  if (alerts) {
    alert_notif->setText(QString::number(alerts) + (alerts > 1 ? " ALERTS" : " ALERT"));
  }
}

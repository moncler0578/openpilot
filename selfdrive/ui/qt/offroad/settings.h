#pragma once

#include <QButtonGroup>
#include <QFileSystemWatcher>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QWidget>
#include <QStackedLayout>
#include <QJsonObject>
#include <QMap>
#include <QList>
#include <QColor>
#include <QSet>
#include <QStringList>

#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"

// ********** settings window + top-level panels **********
class SettingsWindow : public QFrame {
  Q_OBJECT

public:
  explicit SettingsWindow(QWidget *parent = 0);
  void setCurrentPanel(int index, const QString &param = "");

protected:
  void hideEvent(QHideEvent *event) override;
  void showEvent(QShowEvent *event) override;

signals:
  void closeSettings();
  void reviewTrainingGuide();
  void showDriverView();
  void expandToggleDescription(const QString &param);

private:
  QPushButton *sidebar_alert_widget;
  QWidget *sidebar_widget;
  QButtonGroup *nav_btns;
  QStackedWidget *panel_widget;
};

class DevicePanel : public ListWidget {
  Q_OBJECT
public:
  explicit DevicePanel(SettingsWindow *parent);
signals:
  void reviewTrainingGuide();
  void showDriverView();
  void closeSettings();

private slots:
  void poweroff();
  void reboot();
  void updateCalibDescription();

private:
  Params params;
};

class TogglesPanel : public ListWidget {
  Q_OBJECT
public:
  explicit TogglesPanel(SettingsWindow *parent);
  void showEvent(QShowEvent *event) override;

public slots:
  void expandToggleDescription(const QString &param);

private:
  Params params;
  std::map<std::string, ParamControl*> toggles;

  void updateToggles();
};

class SoftwarePanel : public ListWidget {
  Q_OBJECT
public:
  explicit SoftwarePanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  void updateLabels();

  LabelControl *gitBranchLbl;
  LabelControl *gitCommitLbl;
  LabelControl *osVersionLbl;
  LabelControl *versionLbl;
  LabelControl *lastUpdateLbl;
  ButtonControl *updateBtn;

  Params params;
  QFileSystemWatcher *fs_watch;
};

class C2NetworkPanel: public QWidget {
  Q_OBJECT
public:
  explicit C2NetworkPanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  QString getIPAddress();
  LabelControl *ipaddress;
};

class SelectCar : public QWidget {
  Q_OBJECT
public:
  explicit SelectCar(QWidget* parent = 0);

private:

signals:
  void backPress();
  void selectedCar();
};

class LateralControl : public QWidget {
  Q_OBJECT
public:
  explicit LateralControl(QWidget* parent = 0);

private:

signals:
  void backPress();
  void selected();
};

class CommunityPanel : public QWidget {
  Q_OBJECT

private:
  QStackedLayout* main_layout = nullptr;
  QWidget* homeScreen = nullptr;
  SelectCar* selectCar = nullptr;
  LateralControl* lateralControl = nullptr;
  QWidget* homeWidget;

public:
  explicit CommunityPanel(QWidget *parent = nullptr);
};

class VIPPanel : public QWidget {
  Q_OBJECT
public:
  explicit VIPPanel(QWidget* parent = nullptr);
};

class LongitudinalPanel : public QWidget {
  Q_OBJECT
public:
  explicit LongitudinalPanel(QWidget* parent = nullptr);
};

// ── AutoLaneChangeTimer Control ─────────────────────────────────
class AutoLaneChangeTimerControl : public AbstractControl {
  Q_OBJECT
public:
  AutoLaneChangeTimerControl(const QString &title, const QString &desc,
                              const QString &icon, QWidget *parent = nullptr);
  void refresh();
private:
  QPushButton *buttons[6];
  Params params;
  // 인덱스 0~5 → 즉시 / 0.1s / 0.5s / 1.0s / 1.5s / 2.0s
  const QStringList labels = {"즉시", "0.1s", "0.5s", "1.0s", "1.5s", "2.0s"};
};

// ── DynamicLaneProfile Control ──────────────────────────────────
class DynamicLaneProfileControl : public AbstractControl {
  Q_OBJECT
public:
  DynamicLaneProfileControl(const QString &title, const QString &desc,
                             const QString &icon, QWidget *parent = nullptr);
  void refresh();
private:
  QPushButton *buttons[3];
  Params params;
  // 0: Lane only, 1: Lane less, 2: Auto
  const QStringList labels = {"Lane only", "Lane less", "Auto"};
};

class OffsetTotalControl : public AbstractControl {
  Q_OBJECT
public:
  OffsetTotalControl(const QString &title, const QString &desc,
                     const QString &icon, QWidget *parent = nullptr);
  void refresh();
private:
  void changeValue(int delta);
  QPushButton *minus_btn, *plus_btn;
  QLabel *value_label;
  Params params;
};

// nTune JSON(/data/ntune/common.json, lat_torque_v4.json) 값을 화면에서 직접 조절한다.
// nTune 이 dnotify 로 파일 변경을 감시하므로 저장 즉시 주행 중에도 반영된다.
class NtuneValueControl : public AbstractControl {
  Q_OBJECT
public:
  NtuneValueControl(const QString &group, const QString &key,
                    const QString &title, const QString &desc, const QString &icon,
                    double vmin, double vmax, double step, int decimals,
                    double vdefault, QWidget *parent = nullptr);
  void refresh();
private:
  void changeValue(int delta);
  QString group_, key_;
  double vmin_, vmax_, step_, vdefault_;
  int decimals_;
  QPushButton *minus_btn, *plus_btn;
  QLabel *value_label;
};

// 정수 Params 값을 화면에서 조절 (carrot CValueControl 방식)
class ParamValueControlF : public AbstractControl {
  Q_OBJECT
public:
  ParamValueControlF(const QString &param, const QString &title, const QString &desc,
                     const QString &icon, int vmin, int vmax, int step, int decimals,
                     int vdefault, QWidget *parent = nullptr);
  void refresh();
private:
  void changeValue(int delta);
  QString param_;
  int vmin_, vmax_, step_, decimals_, vdefault_;
  QPushButton *minus_btn, *plus_btn;
  QLabel *value_label;
  Params params;
};

class AdjustLaneOffsetControl : public AbstractControl {
  Q_OBJECT
public:
  AdjustLaneOffsetControl(const QString &title, const QString &desc,
                          const QString &icon, QWidget *parent = nullptr);
  void refresh();
private:
  void changeValue(int delta);
  QPushButton *minus_btn, *plus_btn;
  QLabel *value_label;
  Params params;
};

// ── CarrotPilot Auto-Tuner (commit 9dd5e2c port) ─────────────────────────

// 파라미터 변화 추이 라인 그래프 위젯
class AutoTunerGraphWidget : public QWidget {
  Q_OBJECT
public:
  explicit AutoTunerGraphWidget(QWidget *parent = nullptr);
  void setData(const QList<QString> &timestamps, const QMap<QString, QList<double>> &param_histories, const QMap<QString, QColor> &colors);
  void setSelectedParam(const QString &param);
  void setHiddenParams(const QSet<QString> &params);

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;

private:
  QList<QString> timestamps;
  QMap<QString, QList<double>> param_histories;
  QMap<QString, QColor> colors;
  QString selected_param;
  QSet<QString> hidden_params;
  int selected_index = -1;
};

// 이력 카드 리스트 다이얼로그 (Restore / Delete)
class AutoTunerCardListDialog : public QDialogBase {
  Q_OBJECT
public:
  explicit AutoTunerCardListDialog(QWidget *parent = nullptr);

protected:
  void showEvent(QShowEvent *event) override;

private slots:
  void refreshHistory();
  void deleteItem(const QString& id);
  void restoreItem(const QString& id);

private:
  QVBoxLayout *list_layout;
};

// 이력 패널 (그래프 + 파라미터 목록 + LAT/LONG 토글)
class AutoTunerHistoryPanel : public QFrame {
  Q_OBJECT
public:
  explicit AutoTunerHistoryPanel(QWidget* parent = nullptr);

public slots:
  void refreshHistory();
  void updateLabelColors();

private slots:
  void clearAll();

private:
  void rebuildParamList();
  void toggleGroup(const QString &group);
  void applyHiddenParams();
  AutoTunerGraphWidget *graph_widget;
  QVBoxLayout *param_list_layout;
  QMap<QString, QLabel*> param_labels;
  QString selected_param;
  QMap<QString, QColor> param_colors;
  // 좌측 파라미터 리스트를 그룹(가속/조향/곡선/거리/주행 등)으로 묶어
  // 그룹 헤더 클릭 시 접기/펴기 + 그래프 표시 토글을 지원하기 위한 상태
  QStringList group_order;                  // 표시 순서대로 정렬된 그룹 라벨
  QMap<QString, QStringList> group_params;  // 그룹 라벨 → 소속 파라미터들
  QSet<QString> collapsed_groups;           // 접혀있는(그래프 숨김) 그룹 라벨

protected:
  void showEvent(QShowEvent *event) override;
};

// 이력 패널을 담는 다이얼로그
class AutoTunerHistoryDialog : public QDialogBase {
  Q_OBJECT
public:
  explicit AutoTunerHistoryDialog(QWidget *parent = nullptr);

protected:
  void showEvent(QShowEvent *event) override;
};

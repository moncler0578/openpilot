#include <QCoreApplication>
#include <QEvent>
#include <QVariant>

#include "selfdrive/ui/qt/offroad/settings.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/controls.h"

namespace {

class VIPPopupToggleInstaller : public QObject {
public:
  explicit VIPPopupToggleInstaller(QObject *parent = nullptr) : QObject(parent) {}

protected:
  bool eventFilter(QObject *obj, QEvent *event) override {
    if (event->type() == QEvent::Show) {
      auto *panel = qobject_cast<VIPPanel *>(obj);
      if (panel != nullptr && !panel->property("animatedPopupToggleAdded").toBool()) {
        auto *list = panel->findChild<ListWidget *>();
        if (list != nullptr) {
          list->addItem(horizontal_line());
          list->addItem(new ParamControl(
              "AnimatedValuePopup",
              "팝업 활성화",
              "설정속도와 제한속도 등이 변경될 때 온로드 화면에 팝업을 표시합니다.",
              "../assets/offroad/icon_shell.png",
              panel));
          panel->setProperty("animatedPopupToggleAdded", true);
        }
      }
    }

    return QObject::eventFilter(obj, event);
  }
};

void installVIPPopupToggle() {
  if (qApp != nullptr) {
    qApp->installEventFilter(new VIPPopupToggleInstaller(qApp));
  }
}

}  // namespace

Q_COREAPP_STARTUP_FUNCTION(installVIPPopupToggle)

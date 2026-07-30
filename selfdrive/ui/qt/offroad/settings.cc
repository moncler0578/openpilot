#include "selfdrive/ui/qt/offroad/settings.h"

#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/controls.h"

// settings_base.cc의 VIPPanel 생성 과정에서 ChevronInfo 바로 다음에
// 일반 ParamControl 토글을 한 번 추가한다.
template <typename ParentT, typename ItemT>
static inline void addAnimatedPopupToggleAfter(ParentT *, ItemT *) {}

static inline void addAnimatedPopupToggleAfter(VIPPanel *panel, ChevronInfoControl *) {
  auto *list = panel->findChild<ListWidget *>();
  if (list == nullptr) return;

  list->addItem(horizontal_line());
  list->addItem(new ParamControl(
      "AnimatedValuePopup",
      "팝업 활성화",
      "온로드 화면의 값 변경 팝업을 활성화합니다.",
      "../assets/offroad/icon_shell.png",
      panel));
}

#define addItem(item) addItem(item); addAnimatedPopupToggleAfter(this, item)
#include "selfdrive/ui/qt/offroad/settings_base.cc"
#undef addItem

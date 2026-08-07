from selfdrive.controls.lib.soft_hold import SOFT_HOLD_ENTRY_TIME, SoftHoldController

DT_CTRL = 0.01


def enter_soft_hold(controller):
  frames = int(round(SOFT_HOLD_ENTRY_TIME / DT_CTRL))
  for _ in range(frames):
    controller.update(True, True, False, 0.0, False, True, DT_CTRL)


def test_requires_sustained_full_stop():
  controller = SoftHoldController()
  frames = int(round(SOFT_HOLD_ENTRY_TIME / DT_CTRL)) - 1
  for _ in range(frames):
    assert not controller.update(True, True, False, 0.0, False, True, DT_CTRL)
  assert controller.update(True, True, False, 0.0, False, True, DT_CTRL)


def test_does_not_enter_while_creeping():
  controller = SoftHoldController()
  for _ in range(int(2.0 / DT_CTRL)):
    assert not controller.update(True, True, False, 0.2, False, True, DT_CTRL)


def test_requires_drive_gear():
  controller = SoftHoldController()
  for _ in range(int(2.0 / DT_CTRL)):
    assert not controller.update(True, True, False, 0.0, False, False, DT_CTRL)


def test_lead_or_signal_change_cannot_release_hold():
  controller = SoftHoldController()
  enter_soft_hold(controller)
  for _ in range(int(2.0 / DT_CTRL)):
    assert controller.update(True, False, False, 0.0, False, True, DT_CTRL)


def test_gas_releases_hold():
  controller = SoftHoldController()
  enter_soft_hold(controller)
  assert not controller.update(True, False, True, 0.0, False, True, DT_CTRL)
  assert controller.released


def test_resume_releases_hold():
  controller = SoftHoldController()
  enter_soft_hold(controller)
  assert not controller.update(True, False, False, 0.0, True, True, DT_CTRL)
  assert controller.released


def test_disabling_feature_resets_hold():
  controller = SoftHoldController()
  enter_soft_hold(controller)
  assert not controller.update(False, False, False, 0.0, False, True, DT_CTRL)
  assert not controller.released


def test_leaving_drive_resets_without_launch_release():
  controller = SoftHoldController()
  enter_soft_hold(controller)
  assert not controller.update(True, False, False, 0.0, False, False, DT_CTRL)
  assert not controller.released

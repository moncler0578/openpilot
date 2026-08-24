import copy
import random

from selfdrive.car.hyundai.values import Buttons
from selfdrive.car.hyundai.cruise_buttons import button_pressed_in_samples


# aPilot C2 physical-button cadence.
ALIVE_COUNT = [6, 8]
WAIT_COUNT = [12, 13, 14, 15, 16]
AliveIndex = 0
WaitIndex = 0


class SccSmoother:
  """Hyundai CLU11/SCC transport for policy decisions made by CruiseHelper."""

  @staticmethod
  def get_alive_count():
    global AliveIndex
    count = ALIVE_COUNT[AliveIndex]
    AliveIndex = (AliveIndex + 1) % len(ALIVE_COUNT)
    return count

  @staticmethod
  def get_wait_count():
    global WaitIndex
    count = WAIT_COUNT[WaitIndex]
    WaitIndex = (WaitIndex + 1) % len(WAIT_COUNT)
    return count

  def __init__(self):
    self.cruise_helper = None
    self.started_frame = 0
    self.wait_timer = 0
    self.alive_timer = 0
    self.btn = Buttons.NONE
    self.alive_count = ALIVE_COUNT[0]
    random.shuffle(WAIT_COUNT)

  def reset(self):
    self.wait_timer = 0
    self.alive_timer = 0
    self.btn = Buttons.NONE
    if self.cruise_helper is not None:
      self.cruise_helper.reset_scc_target()

  @staticmethod
  def create_clu11(packer, bus, clu11, button):
    values = copy.copy(clu11)
    values["CF_Clu_CruiseSwState"] = button
    values["CF_Clu_AliveCnt1"] = (values["CF_Clu_AliveCnt1"] + 1) % 0x10
    return packer.make_can_msg("CLU11", bus, values)

  def is_active(self, frame):
    return frame - self.started_frame <= max(ALIVE_COUNT) + max(WAIT_COUNT)

  def inject_events(self, events):
    if self.cruise_helper is not None:
      self.cruise_helper.inject_events(events)

  def update(self, _enabled, can_sends, packer, CC, CS, frame, controls):
    self.cruise_helper = controls.cruise_helper
    longcontrol = controls.CP.openpilotLongitudinalControl
    _, ascc_enabled = self.cruise_helper.update_scc(CC, CS, frame, controls, longcontrol)

    # apilot-c2 drives SCC11/12 directly during openpilot longitudinal
    # control. Sending an additional physical RES/SET pulse here can make the
    # cluster report that cruise conditions are not met during lead departure.
    # Keep CruiseHelper's speed/HUD calculations above, but reserve CLU11
    # button transport below for stock ACC only.
    if longcontrol:
      self.reset()
      return

    physical_button_pressed = button_pressed_in_samples(
      CS.cruise_buttons, getattr(CS, 'cruise_button_samples', ()))
    if not ascc_enabled or CS.standstill or physical_button_pressed:
      self.reset()
      self.wait_timer = max(ALIVE_COUNT) + max(WAIT_COUNT)
      return

    if not ascc_enabled:
      self.reset()

    if self.wait_timer > 0:
      self.wait_timer -= 1
      return

    if not (ascc_enabled and not CS.out.cruiseState.standstill):
      return

    if self.alive_timer == 0:
      if ascc_enabled and self.cruise_helper.auto_cruise_control:
        current_set_speed = CS.cruiseState_speed * self.cruise_helper.speed_conv_to_clu
        self.btn = self.cruise_helper.get_button(current_set_speed)
      self.alive_count = SccSmoother.get_alive_count()

    if self.btn == Buttons.NONE:
      return

    can_sends.append(SccSmoother.create_clu11(packer, CS.scc_bus, CS.clu11, self.btn))
    if self.alive_timer == 0:
      self.started_frame = frame
    self.alive_timer += 1
    if self.alive_timer >= self.alive_count:
      self.alive_timer = 0
      self.wait_timer = SccSmoother.get_wait_count()
      self.btn = Buttons.NONE

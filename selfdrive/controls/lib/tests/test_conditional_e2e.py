from selfdrive.controls.lib.conditional_e2e import (E2E_MODE_RELEASE_HOLD_TIME,
                                                    E2E_VISION_LEAD_CONFIRM_TIME,
                                                    ConditionalE2EController,
                                                    adjust_stop_distance_for_decel)


DT_MDL = 0.05


def test_traffic_stop_decel_adjusts_virtual_obstacle():
  stop_distance = 100.0
  v_ego = 20.0

  expected_default = stop_distance - (v_ego ** 2 / (2.0 * 2.0) - v_ego ** 2 / (2.0 * 2.5))
  assert adjust_stop_distance_for_decel(stop_distance, v_ego, 0.8) == expected_default
  assert adjust_stop_distance_for_decel(stop_distance, v_ego, 1.0) == stop_distance
  assert adjust_stop_distance_for_decel(stop_distance, v_ego, 1.2) > adjust_stop_distance_for_decel(stop_distance, v_ego, 1.0)
  assert adjust_stop_distance_for_decel(5.0, v_ego, 0.1) == 0.0


def test_traffic_stop_distance_adjust_moves_virtual_obstacle_only():
  assert adjust_stop_distance_for_decel(20.0, 0.0, 1.0, 4.0) == 24.0
  assert adjust_stop_distance_for_decel(2.0, 0.0, 1.0, -4.0) == 0.0


def update(controller, **overrides):
  args = dict(
    available=True,
    experimental_mode=False,
    traffic_stop_mode=2,
    driving_mode=3,
    model_valid=True,
    model_x=200.0,
    model_y=0.0,
    model_v0=10.0,
    model_v_end=10.0,
    v_ego=10.0,
    steering_angle_deg=0.0,
    gas_pressed=False,
    brake_pressed=False,
    right_blinker=False,
    lead_present=False,
    radar_lead_present=False,
    radar_lead_distance=0.0,
    vision_lead_present=False,
  )
  args.update(overrides)
  return controller.update(**args)


def enter_stop(controller, distance=50.0, v_ego=10.0):
  return update(controller, model_x=distance, model_v0=10.0,
                model_v_end=1.0, v_ego=v_ego)


def test_fixed_acc_and_e2e_modes():
  controller = ConditionalE2EController(DT_MDL)
  assert update(controller, traffic_stop_mode=0) == 'acc'
  assert update(controller, experimental_mode=True) == 'blended'
  assert update(controller, experimental_mode=True, traffic_stop_mode=0) == 'blended'
  assert update(controller, experimental_mode=True, available=False) == 'acc'


def test_auto_uses_e2e_for_far_stop_and_acc_for_close_stop():
  controller = ConditionalE2EController(DT_MDL)
  assert enter_stop(controller, distance=80.0) == 'blended'
  assert controller.stopping

  controller.reset()
  assert enter_stop(controller, distance=30.0) == 'acc'
  assert controller.stopping


def test_confirmed_vision_lead_selects_e2e():
  controller = ConditionalE2EController(DT_MDL)
  for _ in range(9):
    assert update(controller, lead_present=True, vision_lead_present=True) == 'acc'
  assert update(controller, lead_present=True, vision_lead_present=True) == 'blended'

  controller.reset()
  for _ in range(10):
    mode = update(controller, traffic_stop_mode=1, lead_present=True,
                  vision_lead_present=True)
  assert mode == 'acc'


def test_confirmed_vision_lead_release_is_hysteretic():
  controller = ConditionalE2EController(DT_MDL)
  confirm_frames = round(E2E_VISION_LEAD_CONFIRM_TIME / DT_MDL)
  for _ in range(confirm_frames):
    mode = update(controller, lead_present=True, vision_lead_present=True)
  assert mode == 'blended'

  for _ in range(confirm_frames - 1):
    mode = update(controller, lead_present=False, vision_lead_present=False)
    assert mode == 'blended'
  assert update(controller, lead_present=False, vision_lead_present=False) == 'acc'


def test_radar_lead_stays_acc_and_suppresses_new_signal_stop():
  controller = ConditionalE2EController(DT_MDL)
  mode = enter_stop(controller, distance=80.0)
  assert mode == 'blended'

  mode = update(controller, model_x=80.0, model_v0=10.0, model_v_end=1.0,
                lead_present=True, radar_lead_present=True, radar_lead_distance=30.0)
  assert mode == 'acc'
  assert not controller.stopping

  controller.reset()
  mode = update(controller, model_x=80.0, model_v0=10.0, model_v_end=1.0,
                lead_present=True, radar_lead_present=True, radar_lead_distance=100.0)
  assert mode == 'acc'
  assert not controller.stopping


def test_turning_and_right_blinker_suppress_new_signal_stop():
  controller = ConditionalE2EController(DT_MDL)
  assert enter_stop(controller, distance=80.0, v_ego=10.0) == 'blended'

  controller.reset()
  mode = update(controller, model_x=80.0, model_v0=10.0, model_v_end=1.0,
                steering_angle_deg=6.0)
  assert mode == 'acc'
  assert not controller.stopping

  controller.reset()
  mode = update(controller, model_x=80.0, model_v0=10.0, model_v_end=1.0,
                right_blinker=True)
  assert mode == 'acc'
  assert not controller.stopping


def test_fast_mode_disables_auto_signal_control_but_not_fixed_e2e():
  controller = ConditionalE2EController(DT_MDL)
  assert enter_stop(controller, distance=80.0) == 'blended'
  assert update(controller, driving_mode=4) == 'acc'
  assert not controller.stopping
  assert update(controller, experimental_mode=True, driving_mode=4) == 'blended'


def test_confirmed_start_and_gas_enter_departure_prepare():
  controller = ConditionalE2EController(DT_MDL)
  enter_stop(controller, distance=10.0, v_ego=1.0)
  # The model-velocity moving average must first clear the preceding stop
  # prediction; after that, the C3-style 0.2 s confirmation is four frames.
  for _ in range(8):
    mode = update(controller, model_x=100.0, model_v0=10.0,
                  model_v_end=8.0, v_ego=1.0)
    assert not controller.prepare
  mode = update(controller, model_x=100.0, model_v0=10.0,
                model_v_end=8.0, v_ego=1.0)
  assert mode == 'blended'
  assert controller.prepare

  controller.reset()
  enter_stop(controller, distance=30.0)
  assert update(controller, model_x=30.0, model_v0=10.0,
                model_v_end=1.0, gas_pressed=True) == 'blended'
  assert controller.prepare


def test_brake_cancels_departure_prepare():
  controller = ConditionalE2EController(DT_MDL)
  enter_stop(controller, distance=30.0)
  update(controller, model_x=30.0, model_v0=10.0,
         model_v_end=1.0, gas_pressed=True)
  assert controller.prepare
  update(controller, model_x=30.0, model_v0=10.0,
         model_v_end=1.0, brake_pressed=True)
  assert controller.stopping
  assert not controller.prepare


def test_departure_prepare_stops_immediately_but_holds_blended_mode():
  controller = ConditionalE2EController(DT_MDL)
  controller.prepare = True
  hold_frames = round(E2E_MODE_RELEASE_HOLD_TIME / DT_MDL)

  mode = update(controller, model_x=10.0, model_v0=1.0,
                model_v_end=0.0, v_ego=0.1)
  assert mode == 'blended'
  assert controller.stopping
  assert not controller.prepare

  for _ in range(hold_frames - 1):
    mode = update(controller, model_x=10.0, model_v0=1.0,
                  model_v_end=0.0, v_ego=0.1)
    assert mode == 'blended'

  assert update(controller, model_x=10.0, model_v0=1.0,
                model_v_end=0.0, v_ego=0.1) == 'acc'


def test_invalid_model_falls_back_safely():
  controller = ConditionalE2EController(DT_MDL)
  assert update(controller, model_valid=False) == 'acc'
  assert update(controller, model_valid=False, experimental_mode=True) == 'blended'

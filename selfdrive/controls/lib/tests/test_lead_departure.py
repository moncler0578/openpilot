from selfdrive.controls.lib.lead_departure import LEAD_DEPARTURE_CONFIRM_TIME, LEAD_DEPARTURE_OVERRIDE_TIME, \
  LeadDepartureController


DT = 0.01


def update(controller, **overrides):
  args = dict(
    active=True,
    standstill=True,
    plan_released=False,
    brake_pressed=False,
    gas_pressed=False,
    lead_status=True,
    lead_distance=7.6,
    lead_speed=0.0,
    dt=DT,
  )
  args.update(overrides)
  return controller.update(**args)


def test_lead_departure_requires_speed_distance_and_confirmation():
  controller = LeadDepartureController()
  assert not update(controller)
  assert not update(controller, lead_distance=8.0, lead_speed=0.2)

  result = False
  for _ in range(int(LEAD_DEPARTURE_CONFIRM_TIME / DT) + 1):
    result = update(controller, lead_distance=8.0, lead_speed=0.4)
  assert result


def test_lead_departure_rejects_single_frame_noise():
  controller = LeadDepartureController()
  assert not update(controller)
  assert not update(controller, lead_distance=8.0, lead_speed=0.4)
  assert not update(controller, lead_distance=8.0, lead_speed=0.0)

  for _ in range(int(LEAD_DEPARTURE_CONFIRM_TIME / DT) - 1):
    assert not update(controller, lead_distance=8.0, lead_speed=0.4)


def test_lead_departure_resets_for_driver_or_invalid_lead():
  controller = LeadDepartureController()
  assert not update(controller)
  for _ in range(int(LEAD_DEPARTURE_CONFIRM_TIME / DT) + 1):
    departing = update(controller, lead_distance=8.0, lead_speed=0.4)
  assert departing
  assert not update(controller, lead_distance=8.0, lead_speed=0.4, brake_pressed=True)
  assert not update(controller, lead_distance=8.0, lead_speed=0.4, lead_status=False)


def test_lead_departure_hands_back_when_plan_releases():
  controller = LeadDepartureController()
  assert not update(controller)
  for _ in range(int(LEAD_DEPARTURE_CONFIRM_TIME / DT) + 1):
    departing = update(controller, lead_distance=8.0, lead_speed=0.4)
  assert departing
  assert not update(controller, standstill=False, plan_released=True,
                    lead_distance=8.2, lead_speed=0.5)


def test_lead_departure_rejects_unsafe_distance():
  for distance in (2.5, 45.0):
    controller = LeadDepartureController()
    for _ in range(int(LEAD_DEPARTURE_CONFIRM_TIME / DT) + 2):
      assert not update(controller, lead_distance=distance, lead_speed=0.5)


def test_lead_departure_override_has_a_time_limit():
  controller = LeadDepartureController()
  assert not update(controller)
  for _ in range(int(LEAD_DEPARTURE_CONFIRM_TIME / DT) + 1):
    departing = update(controller, lead_distance=8.0, lead_speed=0.4)
  assert departing

  result = True
  for _ in range(int(LEAD_DEPARTURE_OVERRIDE_TIME / DT) + 1):
    result = update(controller, standstill=False, lead_distance=8.5, lead_speed=0.5)
  assert not result

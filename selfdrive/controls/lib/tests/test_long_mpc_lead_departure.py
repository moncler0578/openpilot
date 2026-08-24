import pytest

from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_lead_departure_cost_multiplier


def test_lead_departure_assist_requires_a_real_departing_lead():
  assert get_lead_departure_cost_multiplier(0.0, 2.0, 0.5, False) == 1.0
  assert get_lead_departure_cost_multiplier(2.0, 2.2, 0.5, True) == 1.0


def test_lead_departure_assist_reacts_early_at_low_speed():
  multiplier = get_lead_departure_cost_multiplier(0.0, 1.5, 0.0, True)
  assert multiplier == pytest.approx(0.35)


def test_lead_departure_assist_fades_with_ego_speed():
  low_speed = get_lead_departure_cost_multiplier(2.0, 3.5, 0.0, True)
  higher_speed = get_lead_departure_cost_multiplier(10.0, 11.5, 0.0, True)
  assert 0.35 < low_speed < higher_speed < 1.0
  assert get_lead_departure_cost_multiplier(12.0, 14.0, 0.0, True) == 1.0


def test_lead_braking_immediately_disables_departure_assist():
  assert get_lead_departure_cost_multiplier(2.0, 4.0, -0.2, True) == 1.0
  assert get_lead_departure_cost_multiplier(2.0, 4.0, -1.0, True) == 1.0

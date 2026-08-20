from common.conversions import Conversions as CV
from common.numpy_fast import clip, interp


MAX_SET_SPEED_KPH = 145
AUTO_SPEED_UP_RATE_KPH_S = 5.0

# aPilot C2 CruiseMax table. This module is intentionally dependency-light so
# planner, controlsd policy and unit tests all share the exact same mapping.
CRUISE_MAX_ACCEL_BP = [0.0, 40.0 * CV.KPH_TO_MS, 60.0 * CV.KPH_TO_MS,
                       80.0 * CV.KPH_TO_MS, 110.0 * CV.KPH_TO_MS,
                       140.0 * CV.KPH_TO_MS]
CRUISE_MAX_VAL_KEYS = ["CruiseMaxVals1", "CruiseMaxVals2", "CruiseMaxVals3",
                       "CruiseMaxVals4", "CruiseMaxVals5", "CruiseMaxVals6"]
CRUISE_MAX_VAL_DEFAULTS = [1.60, 1.20, 1.00, 0.80, 0.70, 0.60]


def get_cruise_max_accel(v_ego, cruise_max_vals, driving_mode,
                         eco_mode_factor=1.0, safe_mode_factor=1.0):
  """Return the shared CruiseMax upper bound for planner and final control."""
  values = cruise_max_vals if len(cruise_max_vals) == len(CRUISE_MAX_ACCEL_BP) \
    else CRUISE_MAX_VAL_DEFAULTS
  mode = int(clip(driving_mode, 1, 4))
  if mode == 1:  # SAFE = ECO multiplied by the SAFE factor
    mode_factor = eco_mode_factor * safe_mode_factor
  elif mode == 2:  # ECO
    mode_factor = eco_mode_factor
  else:  # NORMAL / FAST
    mode_factor = 1.0
  return float(max(0.0, interp(v_ego, CRUISE_MAX_ACCEL_BP, values) * mode_factor))


def apply_cruise_max_limit(accel, stopping, cruise_max_accel):
  """Clamp the final SCC acceleration request to the CruiseMax policy.

  CruiseMax used to bound the planner trajectory only. LongControl runs its PID
  against CarControllerParams.ACCEL_MIN/MAX, so the error term could add
  acceleration on top of the already-capped planned feedforward, and the
  LongCtrlState.starting launch accel bypassed the cap entirely. Applying the
  same policy to the last value before SCC12 makes the UI setting the real
  upper bound. Braking and stopping requests are passed through untouched.
  """
  if stopping or accel <= 0.0:
    return float(accel)
  return float(min(accel, cruise_max_accel))


def select_auto_driving_mode(initial_mode, current_mode, driving_index):
  """Map AUTO to SAFE/NORMAL while preserving manually selected ECO/FAST."""
  if initial_mode != 5 or driving_index <= 0.0 or current_mode in (2, 4):
    return current_mode
  if driving_index < 20.0:
    return 3
  if driving_index > 80.0:
    return 1
  return current_mode


def get_auto_speed_up_target(set_speed_kph, road_limit_kph, dt=0.01):
  """Rate-limit automatic set-speed increases and enforce the global maximum."""
  bounded_limit = float(clip(road_limit_kph, 0.0, MAX_SET_SPEED_KPH))
  return min(set_speed_kph + AUTO_SPEED_UP_RATE_KPH_S * dt,
             bounded_limit, float(MAX_SET_SPEED_KPH))

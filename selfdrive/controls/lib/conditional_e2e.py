#!/usr/bin/env python3
from collections import deque
from statistics import fmean, median


# Match C3's confirmed green/start response. The old-model branch still uses
# the filtered model trajectory and distance guard below to reject a single
# noisy far-path frame.
E2E_START_CONFIRM_TIME = 0.2
E2E_START_MIN_DISTANCE = 60.0
E2E_FAR_STOP_DISTANCE = 40.0
E2E_VISION_LEAD_DISTANCE = 90.0
E2E_VISION_LEAD_CONFIRM_TIME = 0.5
E2E_MODE_RELEASE_HOLD_TIME = 0.5
TRAFFIC_STOP_SOLVER_COMFORT_BRAKE = 2.5
TRAFFIC_STOP_APILOT_COMFORT_BRAKE = 2.5


def adjust_stop_distance_for_decel(stop_distance, v_ego, decel_factor, distance_adjust=0.0):
  """Emulate a variable MPC comfort-brake value with a fixed-parameter solver.

  aPilot changes the comfort-brake MPC parameter while stopping for a traffic
  signal. This branch uses a pre-generated solver with that value compiled in,
  so shifting the virtual stop obstacle by the equivalent braking-distance
  delta provides the same earlier/later braking request without regenerating
  the acados solver.
  """
  factor = max(0.1, min(1.2, float(decel_factor)))
  speed = max(0.0, float(v_ego))
  base_distance = speed ** 2 / (2.0 * TRAFFIC_STOP_SOLVER_COMFORT_BRAKE)
  adjusted_distance = speed ** 2 / (2.0 * TRAFFIC_STOP_APILOT_COMFORT_BRAKE * factor)
  return max(0.0, float(stop_distance) + float(distance_adjust) -
             (adjusted_distance - base_distance))


class ConditionalE2EController:
  """Select ACC/blended MPC and retain the model-predicted traffic-stop state."""

  def __init__(self, dt):
    self.dt = dt
    self.vision_lead_confirm_frames = max(1, round(E2E_VISION_LEAD_CONFIRM_TIME / dt))
    self.mode_release_hold_frames = max(1, round(E2E_MODE_RELEASE_HOLD_TIME / dt))
    self.reset()

  def reset(self):
    self.stopping = False
    self.prepare = False
    self.stop_distance = 0.0
    self.stop_sign_count = 0
    self.start_sign_count = 0
    self.vision_lead_count = 0
    self.vision_lead_latched = False
    self.mode_release_hold_count = 0
    self.model_v_history = deque(maxlen=10)
    self.stop_x_median_history = deque(maxlen=3)
    self.stop_x_history = deque(maxlen=15)

  @property
  def traffic_state(self):
    return 2 if self.prepare else (1 if self.stopping else 0)

  @property
  def vision_lead_confirmed(self):
    return self.vision_lead_latched

  def select_mode(self, experimental_mode, traffic_stop_mode):
    if experimental_mode:
      return 'blended'
    if traffic_stop_mode == 0:
      return 'acc'
    far_stop = self.stopping and self.stop_distance > E2E_FAR_STOP_DISTANCE
    apilot_vision_lead = traffic_stop_mode == 2 and self.vision_lead_confirmed
    hold_blended = self.mode_release_hold_count > 0
    return 'blended' if self.prepare or far_stop or apilot_vision_lead or hold_blended else 'acc'

  def update(self, *, available, experimental_mode, traffic_stop_mode, driving_mode, model_valid,
             model_x, model_y, model_v0, model_v_end, v_ego,
             steering_angle_deg, gas_pressed, brake_pressed, right_blinker,
             lead_present, radar_lead_present, radar_lead_distance,
             vision_lead_present):
    if not available:
      self.reset()
      return 'acc'

    traffic_stop_mode = max(0, min(2, traffic_stop_mode))
    if traffic_stop_mode == 0:
      self.reset()
      return 'blended' if experimental_mode else 'acc'

    # aPilot disables traffic-light stopping in HIGH/FAST mode. Explicit E2E
    # still remains blended, matching ExperimentalMode behavior.
    if driving_mode == 4:
      self.reset()
      return 'blended' if experimental_mode else 'acc'

    if not model_valid:
      self.reset()
      return 'blended' if experimental_mode else 'acc'

    if self.mode_release_hold_count > 0:
      self.mode_release_hold_count -= 1

    self.model_v_history.append(float(model_v_end))
    model_v = fmean(self.model_v_history)
    self.stop_x_median_history.append(float(model_x))
    self.stop_x_history.append(float(median(self.stop_x_median_history)))
    filtered_stop_x = max(0.0, fmean(self.stop_x_history))
    v_ego_kph = v_ego * 3.6

    if v_ego_kph < 1.0:
      raw_stop_sign = model_x < 20.0 and model_v < 10.0
    elif v_ego_kph < 80.0:
      raw_stop_sign = (model_x < 120.0 and
                       (model_v < 3.0 or model_v < model_v0 * 0.7) and
                       abs(model_y) < 5.0)
    else:
      raw_stop_sign = False

    # Keep the aPilot start alternatives, with the existing distance guard and
    # sustained confirmation that prevent one noisy model frame from launching.
    raw_start_sign = (not raw_stop_sign and model_x > E2E_START_MIN_DISTANCE and
                      (model_v > 5.0 or model_v > model_v0 + 2.0))
    self.stop_sign_count = self.stop_sign_count + 1 if raw_stop_sign else 0
    self.start_sign_count = self.start_sign_count + 1 if raw_start_sign else 0
    stop_sign = self.stop_sign_count > 0 and not right_blinker
    start_sign = self.start_sign_count * self.dt >= E2E_START_CONFIRM_TIME

    # Confirm both acquisition and release. Radar/vision classification can
    # flicker for a frame near standstill; dropping E2E immediately creates a
    # sharp ACC/E2E acceleration discontinuity exactly when brake hold is
    # handing off to launch control.
    if vision_lead_present:
      self.vision_lead_count = min(self.vision_lead_confirm_frames,
                                   self.vision_lead_count + 1)
      if self.vision_lead_count >= self.vision_lead_confirm_frames:
        self.vision_lead_latched = True
    else:
      self.vision_lead_count = max(0, self.vision_lead_count - 1)
      if self.vision_lead_count == 0:
        self.vision_lead_latched = False
    radar_lead_before_stop = (radar_lead_present and radar_lead_distance > 0.0 and
                              radar_lead_distance - filtered_stop_x < 2.0)

    if self.stopping:
      if start_sign or gas_pressed:
        self.stopping = False
        self.prepare = True
        self.mode_release_hold_count = 0
        self.stop_distance = 0.0
      elif radar_lead_before_stop:
        # The real lead is closer than the model stop line; let ACC follow it.
        self.stopping = False
        self.prepare = False
        self.stop_distance = 0.0
      elif v_ego < 0.1:
        self.stop_distance = 0.0
      elif stop_sign:
        self.stop_distance = max(filtered_stop_x, v_ego ** 2 / 4.0)
      else:
        self.stop_distance = max(0.0, self.stop_distance - v_ego * self.dt)

    elif self.prepare:
      prepare_abort = (v_ego_kph < 2.0 and not start_sign and
                       not lead_present and not gas_pressed)
      if brake_pressed or prepare_abort:
        self.prepare = False
        self.stopping = True
        self.mode_release_hold_count = self.mode_release_hold_frames
        self.stop_distance = 0.0 if v_ego < 0.1 else filtered_stop_x
      elif v_ego_kph > 5.0 and model_x > E2E_START_MIN_DISTANCE:
        self.prepare = False
        self.mode_release_hold_count = self.mode_release_hold_frames

    elif (stop_sign and not lead_present and
          abs(steering_angle_deg) <= 5.0 and not gas_pressed):
      self.stopping = True
      self.stop_distance = 0.0 if v_ego < 0.1 else max(filtered_stop_x, v_ego ** 2 / 4.0)

    return self.select_mode(experimental_mode, traffic_stop_mode)

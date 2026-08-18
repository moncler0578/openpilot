#!/usr/bin/env python3
"""
LiveTorque self-learning lateral controller parameter estimator.

Backport of ajouatom/openpilot (hoya/c3-atune)'s selfdrive/locationd/torqued.py,
adapted to this (older, pre-livePose/carOutput) fork:
  - uses `liveLocationKalman` instead of `livePose`/PoseCalibrator (this fork's
    calibratedOrientationNED / angularVelocityCalibrated are already in the
    vehicle-aligned calibrated frame, same fields latcontrol_torque.py already
    reads via `llk`).
  - uses `carControl.actuatorsOutput.steer` instead of a separate `carOutput`
    message (this fork has no carOutput split -- actuatorsOutput lives inside
    carControl instead, populated in controlsd.py).
  - uses a fixed lag (CP.steerActuatorDelay) instead of a live `liveDelay`
    estimator (this fork has no locationd liveDelay process).
  - PointBuckets/ParameterEstimator base classes inlined here since this fork's
    selfdrive/locationd/ has no shared helpers.py for them.

Output feeds LatControlTorque.update_live_torque_params(), which already exists
in latcontrol_torque.py but, before this backport, was never called.

NOTE: this only changes the *live-learning* torque parameters. It does not
touch the manual "LateralTorqueCustom" override path, which still takes
priority whenever it's enabled (read_torque_params() in latcontrol_torque.py
early-returns to the manual values in that case).

SAFETY HISTORY (2026-08-18): the first version of this file used
`carState.steeringTorqueEps` (physical Nm, per this car's DBC scaling) as the
learning signal instead of the actuator command (normalized [-1, 1]) -- a
genuine units mismatch. Combined with restoring a previously-cached filtered
value directly (instant step, no ramp-in) on process start, a driver reported
an immediate MDPS fault on real hardware right after a drive where this was
active. Fixed by: (1) switching to `actuatorsOutput.steer`, (2) never
restoring the cached *filtered* value or decay -- only cached raw points are
kept, so every start re-earns its way to a learned value instead of snapping
to one, (3) an independent ~2s ramp in
LatControlTorque.update_live_torque_params() as a second line of defense,
(4) tightening FRICTION_SANITY, and (5) making the whole feature opt-in via
live_tune.live_torque_enabled() / the "LiveTorqueEnabled" Params key
(default off) instead of always-on.
"""
import os
import time
from collections import deque, defaultdict

import numpy as np

import cereal.messaging as messaging
from cereal import car, log
from common.params import Params, put_nonblocking
from common.realtime import config_realtime_process, DT_MDL
from common.filter_simple import FirstOrderFilter
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY

HISTORY = 5  # secs
POINTS_PER_BUCKET = 1500
MIN_POINTS_TOTAL = 4000
FIT_POINTS_TOTAL = 2000
MIN_VEL = 15  # m/s
FRICTION_FACTOR = 1.5  # ~85% of data coverage
FACTOR_SANITY = 0.3
# Tightened from the upstream default of 0.5 (i.e. friction could swing up to
# +150% of the vehicle's calibrated default) after a real-car MDPS fault on
# this low-margin, poorly-supported platform (Genesis DH 2015-2016) traced to
# an abrupt learned-friction change -- 2026-08-18.
FRICTION_SANITY = 0.3
STEER_MIN_THRESHOLD = 0.02
MIN_FILTER_DECAY = 50
MAX_FILTER_DECAY = 250
LAT_ACC_THRESHOLD = 1
STEER_BUCKET_BOUNDS = [(-0.5, -0.3), (-0.3, -0.2), (-0.2, -0.1), (-0.1, 0), (0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.5)]
MIN_BUCKET_POINTS = np.array([100, 300, 500, 500, 500, 500, 300, 100])
MIN_ENGAGE_BUFFER = 2  # secs

# loop is polled on liveLocationKalman (~20Hz in this fork's services.py)
PUBLISH_EVERY_N_FRAMES = 5     # -> 4Hz publish, matches services.py "liveTorqueParameters" freq
CACHE_EVERY_N_FRAMES = 1200    # -> ~60s at 20Hz

VERSION = 1  # bump this to invalidate old parameter caches


def slope2rot(slope):
  sin = np.sqrt(slope ** 2 / (slope ** 2 + 1))
  cos = np.sqrt(1 / (slope ** 2 + 1))
  return np.array([[cos, -sin], [sin, cos]])


class PointBuckets:
  """Minimal inline replacement for selfdrive/locationd/helpers.py's PointBuckets
  (that module doesn't exist in this fork)."""

  def __init__(self, x_bounds, min_points, min_points_total, points_per_bucket, rowsize):
    self.x_bounds = x_bounds
    self.buckets = {bounds: deque(maxlen=points_per_bucket) for bounds in x_bounds}
    self.buckets_min_points = dict(zip(x_bounds, min_points))
    self.min_points_total = min_points_total
    self.rowsize = rowsize

  def __len__(self):
    return sum(len(v) for v in self.buckets.values())

  def add_point(self, x, y):
    for bound_min, bound_max in self.x_bounds:
      if bound_min <= x < bound_max:
        self.buckets[(bound_min, bound_max)].append([x, 1.0, y])
        break

  def is_valid(self):
    individual_points = all(len(v) >= min_pts for v, min_pts in zip(self.buckets.values(), self.buckets_min_points.values()))
    total_points = self.__len__() >= self.min_points_total
    return individual_points and total_points

  def is_calculable(self):
    return all(len(v) > 0 for v in self.buckets.values())

  def get_points(self, num_points=None):
    points = np.array([point for bucket in self.buckets.values() for point in bucket])
    if num_points is None:
      return points
    return points[np.random.choice(np.arange(len(points)), min(len(points), num_points), replace=False)]

  def get_valid_percent(self):
    return 100.0 * sum(min(len(v), min_pts) for v, min_pts in
                        zip(self.buckets.values(), self.buckets_min_points.values())) / max(1, self.min_points_total)

  def load_points(self, points):
    for x, y in points:
      self.add_point(x, y)


class TorqueBuckets(PointBuckets):
  pass


class TorqueEstimator:
  def __init__(self, CP):
    self.CP_fingerprint = CP.carFingerprint
    self.hist_len = int(HISTORY / DT_MDL)
    self.lag = float(getattr(CP, "steerActuatorDelay", 0.2)) + 0.2

    self.min_bucket_points = MIN_BUCKET_POINTS
    self.min_points_total = MIN_POINTS_TOTAL
    self.fit_points = FIT_POINTS_TOTAL
    self.factor_sanity = FACTOR_SANITY
    self.friction_sanity = FRICTION_SANITY

    self.offline_friction = 0.0
    self.offline_latAccelFactor = 0.0
    self.use_params = CP.lateralTuning.which() == 'torque'

    if self.use_params:
      self.offline_friction = CP.lateralTuning.torque.friction
      self.offline_latAccelFactor = CP.lateralTuning.torque.latAccelFactor

    self.resets = 0.0
    self.reset()

    initial_params = {
      'latAccelFactor': self.offline_latAccelFactor,
      'latAccelOffset': 0.0,
      'frictionCoefficient': self.offline_friction,
      'points': []
    }
    self.decay = MIN_FILTER_DECAY
    self.min_lataccel_factor = (1.0 - self.factor_sanity) * self.offline_latAccelFactor
    self.max_lataccel_factor = (1.0 + self.factor_sanity) * self.offline_latAccelFactor
    self.min_friction = (1.0 - self.friction_sanity) * self.offline_friction
    self.max_friction = (1.0 + self.friction_sanity) * self.offline_friction

    # Restore cached *points* only -- never the cached filtered latAccelFactor/
    # latAccelOffset/frictionCoefficient, and never the cached decay.
    # filtered_params below always starts from the vehicle's calibrated
    # defaults (offline_latAccelFactor/offline_friction/0.0) with a fresh,
    # fast-reacting decay, so every process start (including right after a
    # park stop, which restarts nothing in this fork, but covers any future
    # restart/reboot case) re-earns its way to the learned value instead of
    # snapping straight to whatever was last cached. Combined with the
    # ramp in LatControlTorque.update_live_torque_params(), a stale or bad
    # cached point set can, at worst, bias how fast estimate_params() moves
    # things over the following minute -- it can no longer apply instantly.
    # (Fixed 2026-08-18 after a real-car MDPS fault traced to an abrupt
    # learned-friction change; the previous version set filtered_params['x']
    # and self.decay directly from the cache here.)
    params = Params()
    torque_cache = params.get("LiveTorqueParameters")
    if torque_cache is not None:
      try:
        with log.Event.from_bytes(torque_cache) as log_evt:
          cache_ltp = log_evt.liveTorqueParameters
        if cache_ltp.carFingerprint == CP.carFingerprint and cache_ltp.version == VERSION:
          initial_params['points'] = cache_ltp.points
          self.filtered_points.load_points(initial_params['points'])
          cloudlog.info("torqued: restored cached points only (not filtered values/decay)")
      except Exception:
        cloudlog.exception("torqued: failed to restore cached torque points")
        params.remove("LiveTorqueParameters")

    self.filtered_params = {}
    for param in initial_params:
      if param == 'points':
        continue
      self.filtered_params[param] = FirstOrderFilter(initial_params[param], self.decay, DT_MDL)

  def reset(self):
    self.resets += 1.0
    self.decay = MIN_FILTER_DECAY
    self.raw_points = defaultdict(lambda: deque(maxlen=self.hist_len))
    self.filtered_points = TorqueBuckets(x_bounds=STEER_BUCKET_BOUNDS,
                                          min_points=self.min_bucket_points,
                                          min_points_total=self.min_points_total,
                                          points_per_bucket=POINTS_PER_BUCKET,
                                          rowsize=3)

  def estimate_params(self):
    points = self.filtered_points.get_points(self.fit_points)
    try:
      _, _, v = np.linalg.svd(points, full_matrices=False)
      slope, offset = -v.T[0:2, 2] / v.T[2, 2]
      _, spread = np.matmul(points[:, [0, 2]], slope2rot(slope)).T
      friction_coeff = np.std(spread) * FRICTION_FACTOR
    except np.linalg.LinAlgError as e:
      cloudlog.exception(f"torqued: error computing live torque params: {e}")
      slope = offset = friction_coeff = np.nan
    return slope, offset, friction_coeff

  def update_params(self, params):
    self.decay = min(self.decay + DT_MDL, MAX_FILTER_DECAY)
    for param, value in params.items():
      self.filtered_params[param].update(value)
      self.filtered_params[param].update_alpha(self.decay)

  def handle_log(self, t, which, msg):
    if which == "carControl":
      self.raw_points["carControl_t"].append(t + self.lag)
      self.raw_points["lat_active"].append(bool(msg.latActive))
      # No separate carOutput message in this fork -- carControl.actuatorsOutput
      # is the equivalent field ("Any car specific rate limits or quirks
      # applied by the CarController are reflected in actuatorsOutput and
      # matches what is sent to the car" per car.capnp, populated in
      # controlsd.py as `CC.actuatorsOutput = self.last_actuators`). .steer is
      # normalized to [-1, 1] (LatControl.steer_max stays 1.0 for
      # LatControlTorque in this fork; it's never overridden to the car's raw
      # CAN torque scale) -- the same sign/units convention latAccelFactor/
      # friction are calibrated against.
      # NOTE: this fork previously used carState.steeringTorqueEps here, which
      # is actually physical Nm (see carstate.py: "CR_Mdps_OutTq / 10 # scale to
      # Nm") -- a real units mismatch, fixed 2026-08-18 after a real-car MDPS
      # fault traced to a bad learned friction value.
      self.raw_points["steer_torque_t"].append(t + self.lag)
      self.raw_points["steer_torque"].append(-msg.actuatorsOutput.steer)
    elif which == "carState":
      self.raw_points["carState_t"].append(t + self.lag)
      self.raw_points["vego"].append(msg.vEgo)
      self.raw_points["steer_override"].append(bool(msg.steeringPressed))
    elif which == "liveLocationKalman":
      if len(self.raw_points['steer_torque']) == 0 or len(self.raw_points['carState_t']) < 2:
        return
      # calibratedOrientationNED / angularVelocityCalibrated are already in the
      # vehicle-aligned "calibrated" frame in this fork -- same fields
      # latcontrol_torque.py's `llk` parameter already reads.
      roll = msg.calibratedOrientationNED.value[0] if msg.calibratedOrientationNED.valid else 0.0
      yaw_rate = msg.angularVelocityCalibrated.value[2] if msg.angularVelocityCalibrated.valid else 0.0

      lat_active = np.interp(np.arange(t - MIN_ENGAGE_BUFFER, t + self.lag, DT_MDL),
                              self.raw_points['carControl_t'], self.raw_points['lat_active']).astype(bool) \
        if len(self.raw_points['carControl_t']) > 1 else np.array([False])
      steer_override = np.interp(np.arange(t - MIN_ENGAGE_BUFFER, t + self.lag, DT_MDL),
                                  self.raw_points['carState_t'], self.raw_points['steer_override']).astype(bool) \
        if len(self.raw_points['carState_t']) > 1 else np.array([True])
      vego = np.interp(t, self.raw_points['carState_t'], self.raw_points['vego'])
      steer = np.interp(t, self.raw_points['steer_torque_t'], self.raw_points['steer_torque']).item()
      lateral_acc = (vego * yaw_rate) - (np.sin(roll) * ACCELERATION_DUE_TO_GRAVITY).item()

      if len(lat_active) and all(lat_active) and not any(steer_override) and (vego > MIN_VEL) and (abs(steer) > STEER_MIN_THRESHOLD):
        if abs(lateral_acc) <= LAT_ACC_THRESHOLD:
          self.filtered_points.add_point(steer, lateral_acc)

  def get_msg(self, valid=True, with_points=False):
    msg = messaging.new_message('liveTorqueParameters')
    msg.valid = valid
    ltp = msg.liveTorqueParameters
    ltp.version = VERSION
    ltp.useParams = self.use_params
    ltp.carFingerprint = self.CP_fingerprint

    if self.filtered_points.is_calculable():
      latAccelFactor, latAccelOffset, frictionCoeff = self.estimate_params()
      ltp.latAccelFactorRaw = float(latAccelFactor)
      ltp.latAccelOffsetRaw = float(latAccelOffset)
      ltp.frictionCoefficientRaw = float(frictionCoeff)

      if self.filtered_points.is_valid():
        if any(val is None or np.isnan(val) for val in [latAccelFactor, latAccelOffset, frictionCoeff]):
          cloudlog.exception("torqued: live torque parameters are invalid.")
          ltp.liveValid = False
          self.reset()
        else:
          ltp.liveValid = True
          latAccelFactor = float(np.clip(latAccelFactor, self.min_lataccel_factor, self.max_lataccel_factor))
          frictionCoeff = float(np.clip(frictionCoeff, self.min_friction, self.max_friction))
          self.update_params({'latAccelFactor': latAccelFactor, 'latAccelOffset': latAccelOffset, 'frictionCoefficient': frictionCoeff})

    if with_points:
      ltp.points = self.filtered_points.get_points()[:, [0, 2]].tolist()

    ltp.latAccelFactorFiltered = float(self.filtered_params['latAccelFactor'].x)
    ltp.latAccelOffsetFiltered = float(self.filtered_params['latAccelOffset'].x)
    ltp.frictionCoefficientFiltered = float(self.filtered_params['frictionCoefficient'].x)
    ltp.totalBucketPoints = len(self.filtered_points)
    ltp.calPerc = int(self.filtered_points.get_valid_percent())
    ltp.decay = self.decay
    ltp.maxResets = self.resets
    return msg


def main():
  config_realtime_process([0, 1, 2, 3], 5)

  DEBUG = bool(int(os.getenv("DEBUG", "0")))

  pm = messaging.PubMaster(['liveTorqueParameters'])
  sm = messaging.SubMaster(['carControl', 'carState', 'liveLocationKalman'], poll=['liveLocationKalman'])

  params = Params()
  CP = car.CarParams.from_bytes(params.get("CarParams", block=True))

  estimator = TorqueEstimator(CP)

  if estimator.use_params:
    cloudlog.info(f"torqued: started for {CP.carFingerprint}")
  else:
    cloudlog.warning(f"torqued: {CP.carFingerprint} is not on torque lateral tuning, exiting")
    return

  while True:
    sm.update()

    ok_all = sm.all_alive() and sm.all_freq_ok() and sm.all_valid()

    if ok_all:
      for which in sm.updated.keys():
        if sm.updated[which]:
          t = sm.logMonoTime[which] * 1e-9
          estimator.handle_log(t, which, sm[which])
    elif DEBUG:
      cloudlog.warning(f"torqued: waiting on inputs, alive={sm.alive} valid={sm.valid}")

    if sm.frame % PUBLISH_EVERY_N_FRAMES == 0:
      pm.send('liveTorqueParameters', estimator.get_msg(valid=ok_all))

    if sm.frame % CACHE_EVERY_N_FRAMES == 0:
      msg = estimator.get_msg(valid=ok_all, with_points=True)
      put_nonblocking("LiveTorqueParameters", msg.to_bytes())


if __name__ == "__main__":
  main()

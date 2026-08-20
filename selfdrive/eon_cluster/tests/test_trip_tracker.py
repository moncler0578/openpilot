from selfdrive.eon_cluster.trip import TripTracker


def test_trip_tracker_resets_and_accumulates_moving_summary():
  tracker = TripTracker()
  tracker.update(True, 36.0, 10.0)
  tracker.update(True, 36.0, 11.0)
  report = tracker.snapshot()
  assert report["duration_s"] == 1.0
  assert report["distance_m"] == 10.0
  assert report["average_speed_kph"] == 36.0
  assert report["max_speed_kph"] == 36.0

  tracker.update(False, 0.0, 12.0)
  tracker.update(True, 0.0, 20.0)
  assert tracker.snapshot()["distance_m"] == 0.0


def test_trip_tracker_records_engagement_and_acceleration_events():
  tracker = TripTracker()
  tracker.update(True, 36.0, 10.0, enabled=True, accel=0.0)
  tracker.update(True, 36.0, 11.0, enabled=True, accel=2.2)
  tracker.update(True, 36.0, 12.0, enabled=True, accel=2.3)
  tracker.update(True, 36.0, 13.0, enabled=False, accel=-2.7)
  report = tracker.snapshot()
  assert report["engaged_time_s"] == 2.0
  assert report["max_accel"] == 2.3
  assert report["max_decel"] == -2.7
  assert report["hard_accel_count"] == 1
  assert report["hard_brake_count"] == 1

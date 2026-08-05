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

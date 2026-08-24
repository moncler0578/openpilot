from selfdrive.car.hyundai.values import Buttons


def collect_button_samples(previous, latest, all_values):
  """Return every button sample received by CANParser in chronological order.

  CANParser.vl only contains the last value in an update.  vl_all preserves a
  quick press/release (or a mid-press button change) when more than one CLU11
  frame is drained in the same control cycle.  The latest-value fallback keeps
  this compatible with parsers and tests that do not populate vl_all.
  """
  samples = [int(value) for value in all_values]
  latest = int(latest)
  if not samples and latest != int(previous):
    samples.append(latest)
  return samples


def button_transitions(previous, samples, unpressed=Buttons.NONE):
  """Convert raw samples to ordered (button, pressed) transitions.

  A direct RES->SET transition must release RES before pressing SET.  Without
  the release, the cruise long-press state remains attached to the old button
  and can change set speed in the wrong direction.
  """
  transitions = []
  state = int(previous)
  unpressed = int(unpressed)

  for sample in samples:
    current = int(sample)
    if current == state:
      continue
    if state != unpressed:
      transitions.append((state, False))
    if current != unpressed:
      transitions.append((current, True))
    state = current

  return transitions


def main_button_transitions(previous, samples):
  """Return all cruise-main switch edges contained in this parser update."""
  transitions = []
  state = bool(previous)
  for sample in samples:
    current = bool(sample)
    if current != state:
      transitions.append(current)
      state = current
  return transitions


def button_pressed_in_samples(current, samples, unpressed=Buttons.NONE):
  """True when a physical cruise button was down at any point this cycle."""
  unpressed = int(unpressed)
  return int(current) != unpressed or any(int(sample) != unpressed for sample in samples)

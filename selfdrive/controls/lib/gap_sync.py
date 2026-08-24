def select_physical_gap(current_gap, physical_gap, controls_enabled, gap_button_event):
  """Return the gap to keep and whether it represents a driver-selected change."""
  if not controls_enabled and not gap_button_event:
    return current_gap, False

  gap = int(physical_gap)
  if 1 <= gap <= 4 and gap != current_gap:
    return gap, True
  return current_gap, False


def select_software_gap(current_gap, gap_button_pressed):
  """Cycle Hyundai gap 1→2→3→4→1 like aPilot C2."""
  gap = int(current_gap)
  if not 1 <= gap <= 4:
    gap = 4
  if not gap_button_pressed:
    return gap, False
  return (gap + 1 if gap < 4 else 1), True

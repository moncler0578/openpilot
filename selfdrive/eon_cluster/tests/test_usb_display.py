import selfdrive.eon_cluster.usb_display as usb_display_module
from selfdrive.eon_cluster.usb_display import (
  CMD_BRIGHTNESS,
  CMD_FRAME_RATE,
  CMD_ORIENTATION,
  CMD_UPLOAD_JPEG,
  TurzxDisplay,
  UsbEventMonitor,
  _command_packet,
)


def test_vendor_command_packet_has_expected_envelope():
  packet = _command_packet(CMD_UPLOAD_JPEG, {8: 1, 11: 2})
  assert len(packet) == 512
  assert packet[-2:] == b"\xa1\x1a"
  assert packet != bytes(512)


def test_live_display_settings_send_only_when_changed(monkeypatch):
  display = TurzxDisplay(brightness=65, frame_rate=10)
  commands = []
  monkeypatch.setattr(display, "_send_optional_command",
                      lambda command, fields=None, **kwargs: commands.append((command, fields)))
  assert display.set_brightness(42)
  assert display.set_frame_rate(12)
  assert display.set_orientation(2)
  assert not display.set_brightness(42)
  assert not display.set_frame_rate(12)
  assert not display.set_orientation(2)
  assert commands == [(CMD_BRIGHTNESS, {8: 42}), (CMD_FRAME_RATE, {8: 12}),
                      (CMD_ORIENTATION, {8: 2})]


def test_usb_event_monitor_matches_only_expected_panel():
  monitor = UsbEventMonitor.__new__(UsbEventMonitor)
  monitor.expected_product_id = 0x0092
  monitor.socket = None
  assert monitor._matches(b"add@/devices/test\0ACTION=add\0SUBSYSTEM=usb\0PRODUCT=1cbe/0092/1\0")
  assert not monitor._matches(b"add@/devices/test\0ACTION=add\0SUBSYSTEM=usb\0PRODUCT=1cbe/0123/1\0")
  assert not monitor._matches(b"remove@/devices/test\0ACTION=remove\0SUBSYSTEM=usb\0PRODUCT=1cbe/0092/1\0")


def test_jpeg_drains_stale_reply_before_next_frame(monkeypatch):
  display = TurzxDisplay()
  events = []
  monkeypatch.setattr(usb_display_module, "_command_packet", lambda *_args, **_kwargs: b"header")
  monkeypatch.setattr(display, "_drain_input",
                      lambda attempts, timeout_ms: events.append(("drain", attempts, timeout_ms)))
  monkeypatch.setattr(display, "_write_no_ack",
                      lambda payload, timeout_ms: events.append(("write", payload, timeout_ms)))
  display.send_jpeg(b"jpeg")
  assert events == [
    ("drain", 2, 2),
    ("write", b"headerjpeg", 2000),
  ]

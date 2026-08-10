from selfdrive.eon_cluster.usb_display import CMD_UPLOAD_JPEG, _command_packet


def test_vendor_command_packet_has_expected_envelope():
  packet = _command_packet(CMD_UPLOAD_JPEG, {8: 1, 11: 2})
  assert len(packet) == 512
  assert packet[-2:] == b"\xa1\x1a"
  assert packet != bytes(512)

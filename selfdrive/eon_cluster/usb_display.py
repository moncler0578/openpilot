import struct
import time

import usb1
from Crypto.Cipher import DES


VENDOR_ID = 0x1CBE
PRODUCT_SIZES = {
  0x0092: (1920, 462),
  0x0123: (1920, 720),
}
CMD_SYNC = 10
CMD_BRIGHTNESS = 14
CMD_FRAME_RATE = 15
CMD_UPLOAD_JPEG = 101
COMMAND_TIMEOUT_MS = 2000
FRAME_TIMEOUT_MS = 2000


def _command_packet(command_id, fields=None):
  packet = bytearray(500)
  packet[0] = command_id
  packet[2] = 0x1A
  packet[3] = 0x6D
  midnight = time.mktime(time.localtime()[:3] + (0, 0, 0, 0, 0, -1))
  packet[4:8] = struct.pack("<I", int((time.time() - midnight) * 1000))
  for index, value in (fields or {}).items():
    packet[index] = int(value) & 0xFF
  cipher = DES.new(b"slv3tuzx", DES.MODE_CBC, b"slv3tuzx")
  # The vendor packet is 500 bytes, while DES-CBC requires an 8-byte block.
  padded = bytes(packet).ljust((len(packet) + 7) // 8 * 8, b"\x00")
  encrypted = cipher.encrypt(padded)
  result = bytearray(512)
  result[:len(encrypted)] = encrypted
  result[510:512] = b"\xa1\x1a"
  return bytes(result)


class TurzxDisplay(object):
  """Small usb1 transport for the TURZX JPEG protocol used by carrot HUD."""

  def __init__(self, brightness=65, frame_rate=10):
    self.brightness = max(0, min(100, int(brightness)))
    self.frame_rate = max(1, min(30, int(frame_rate)))
    self.context = None
    self.handle = None
    self.product_id = None
    self.endpoint_out = None
    self.endpoint_in = None

  @property
  def landscape_size(self):
    return PRODUCT_SIZES[self.product_id]

  def open(self):
    self.context = usb1.USBContext()
    selected = None
    for device in self.context.getDeviceList(skip_on_error=True):
      if device.getVendorID() == VENDOR_ID and device.getProductID() in PRODUCT_SIZES:
        selected = device
        break
    if selected is None:
      self.close()
      raise IOError("supported TURZX display not found")

    self.product_id = selected.getProductID()
    self.handle = selected.open()
    if hasattr(self.handle, "setAutoDetachKernelDriver"):
      self.handle.setAutoDetachKernelDriver(True)
    self.handle.claimInterface(0)
    self._find_endpoints(selected)
    self._send_command(CMD_SYNC)
    time.sleep(0.2)
    self._send_command(CMD_FRAME_RATE, {8: self.frame_rate})
    self._send_command(CMD_BRIGHTNESS, {8: int(self.brightness * 102 / 100)})

  def _find_endpoints(self, device):
    for setting in device.iterSettings():
      if setting.getNumber() != 0:
        continue
      for endpoint in setting.iterEndpoints():
        address = endpoint.getAddress()
        if address & 0x80:
          self.endpoint_in = address
        else:
          self.endpoint_out = address
    if self.endpoint_out is None or self.endpoint_in is None:
      raise IOError("TURZX USB bulk endpoints not found")

  def _exchange(self, payload, timeout_ms):
    if self.handle is None:
      raise IOError("TURZX display is not open")
    written = self.handle.bulkWrite(self.endpoint_out, payload, timeout=timeout_ms)
    if written != len(payload):
      raise IOError("short TURZX USB write: %d/%d" % (written, len(payload)))
    return bytes(self.handle.bulkRead(self.endpoint_in, 512, timeout=timeout_ms))

  def _send_command(self, command_id, fields=None):
    return self._exchange(_command_packet(command_id, fields), COMMAND_TIMEOUT_MS)

  def send_jpeg(self, jpeg):
    header = _command_packet(CMD_UPLOAD_JPEG, {
      8: (len(jpeg) >> 24) & 0xFF,
      9: (len(jpeg) >> 16) & 0xFF,
      10: (len(jpeg) >> 8) & 0xFF,
      11: len(jpeg) & 0xFF,
    })
    return self._exchange(header + jpeg, FRAME_TIMEOUT_MS)

  def close(self):
    if self.handle is not None:
      try:
        self._send_command(CMD_BRIGHTNESS, {8: 0})
      except Exception:
        pass
      try:
        self.handle.releaseInterface(0)
      except Exception:
        pass
      try:
        self.handle.close()
      except Exception:
        pass
    self.handle = None
    if self.context is not None:
      try:
        self.context.close()
      except Exception:
        pass
    self.context = None
    self.product_id = None
    self.endpoint_out = None
    self.endpoint_in = None

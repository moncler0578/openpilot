import errno
import os
import select
import socket
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
CMD_ORIENTATION = 13
CMD_BRIGHTNESS = 14
CMD_FRAME_RATE = 15
CMD_UPLOAD_JPEG = 101
COMMAND_TIMEOUT_MS = 2000
FRAME_TIMEOUT_MS = 2000
USB_DISCONNECT_ERRNOS = {
  errno.ENODEV,
  errno.ENXIO,
  errno.EIO,
  getattr(errno, "ESHUTDOWN", 108),
}
USB_DISCONNECT_CODES = {-4}  # LIBUSB_ERROR_NO_DEVICE
USB_DISCONNECT_TEXT = (
  "no such device",
  "device not found",
  "disconnected",
  "entity not found",
  "device has been disconnected",
)


class UsbEventMonitor(object):
  """Non-blocking Linux USB hotplug monitor with polling fallback."""

  def __init__(self, expected_product_id=None):
    self.expected_product_id = expected_product_id
    self.socket = None
    if not hasattr(socket, "AF_NETLINK"):
      return
    for port_id in (0, os.getpid()):
      candidate = None
      try:
        candidate = socket.socket(socket.AF_NETLINK, socket.SOCK_DGRAM, 15)
        candidate.bind((port_id, 1))
        candidate.setblocking(False)
        self.socket = candidate
        break
      except OSError:
        if candidate is not None:
          candidate.close()

  def _matches(self, payload):
    fields = {}
    for part in payload.decode("utf-8", errors="replace").split("\0"):
      if "=" in part:
        key, value = part.split("=", 1)
        fields[key] = value
      elif "@" in part:
        fields.setdefault("ACTION", part.split("@", 1)[0])
    if fields.get("SUBSYSTEM") != "usb" or fields.get("ACTION") not in ("add", "bind", "change", "move"):
      return False
    product = fields.get("PRODUCT", "").split("/")
    if len(product) >= 2:
      try:
        vendor_id, product_id = int(product[0], 16), int(product[1], 16)
      except ValueError:
        return False
      return vendor_id == VENDOR_ID and (
        self.expected_product_id is None or product_id == self.expected_product_id
      )
    return fields.get("DEVTYPE") == "usb_device"

  def poll(self):
    if self.socket is None:
      return False
    matched = False
    try:
      readable, _, _ = select.select([self.socket], [], [], 0.0)
      while readable:
        matched = self._matches(self.socket.recv(8192)) or matched
        readable, _, _ = select.select([self.socket], [], [], 0.0)
    except (OSError, ValueError):
      self.close()
    return matched

  def close(self):
    if self.socket is not None:
      try:
        self.socket.close()
      except OSError:
        pass
    self.socket = None


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
  """Small usb1 transport for the TURZX JPEG protocol used by carrot HUD.

  The 9.2-inch panel follows carrot-wip's current transport behaviour:
  sync waits for its reply, while FPS/brightness and live JPEG frames are
  written without requiring an ACK.  This avoids treating optional/stale
  replies as a disconnect and repeatedly reinitializing the panel.
  """

  def __init__(self, brightness=65, frame_rate=10, orientation=0, expected_product_id=None):
    self.brightness = max(0, min(100, int(brightness)))
    self.frame_rate = max(1, min(30, int(frame_rate)))
    self.orientation = int(orientation) if int(orientation) in (0, 2) else 0
    self.expected_product_id = expected_product_id
    self.context = None
    self.handle = None
    self.product_id = None
    self.endpoint_out = None
    self.endpoint_in = None
    self.disconnected = False

  @property
  def landscape_size(self):
    return PRODUCT_SIZES[self.product_id]

  def open(self):
    self.context = usb1.USBContext()
    selected = None
    for device in self.context.getDeviceList(skip_on_error=True):
      product_id = device.getProductID()
      if device.getVendorID() != VENDOR_ID or product_id not in PRODUCT_SIZES:
        continue
      if self.expected_product_id is not None and product_id != self.expected_product_id:
        continue
      selected = device
      break
    if selected is None:
      self.close()
      if self.expected_product_id is not None:
        raise IOError("TURZX display not found: pid=0x%04x" % self.expected_product_id)
      raise IOError("supported TURZX display not found")

    self.product_id = selected.getProductID()
    self.handle = selected.open()
    self.disconnected = False
    if hasattr(self.handle, "setAutoDetachKernelDriver"):
      self.handle.setAutoDetachKernelDriver(True)
    self.handle.claimInterface(0)
    self._find_endpoints(selected)

    # carrot-wip keeps SYNC as the only initialization command requiring ACK.
    self._send_command(CMD_SYNC)
    time.sleep(0.2)
    self._send_optional_command(CMD_ORIENTATION, {8: self.orientation}, gap_s=0.03, drain_attempts=0)
    self._send_optional_command(CMD_FRAME_RATE, {8: self.frame_rate}, gap_s=0.05, drain_attempts=1)
    self._send_optional_command(CMD_BRIGHTNESS, {8: int(self.brightness * 102 / 100)}, drain_attempts=0)

  def set_frame_rate(self, frame_rate):
    frame_rate = max(1, min(30, int(frame_rate)))
    if frame_rate == self.frame_rate:
      return False
    self._send_optional_command(CMD_FRAME_RATE, {8: frame_rate}, gap_s=0.05, drain_attempts=1)
    self.frame_rate = frame_rate
    return True

  def set_brightness(self, brightness):
    brightness = max(0, min(100, int(brightness)))
    if brightness == self.brightness:
      return False
    self._send_optional_command(CMD_BRIGHTNESS, {8: int(brightness * 102 / 100)}, drain_attempts=0)
    self.brightness = brightness
    return True

  def set_orientation(self, orientation):
    orientation = int(orientation)
    if orientation not in (0, 2):
      orientation = 0
    if orientation == self.orientation:
      return False
    self._send_optional_command(CMD_ORIENTATION, {8: orientation}, gap_s=0.03, drain_attempts=0)
    self.orientation = orientation
    return True

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

  @staticmethod
  def _exception_indicates_disconnect(exc):
    visited = set()
    current = exc
    while current is not None and id(current) not in visited:
      visited.add(id(current))
      if getattr(current, "errno", None) in USB_DISCONNECT_ERRNOS:
        return True
      if getattr(current, "value", None) in USB_DISCONNECT_CODES:
        return True
      if getattr(current, "backend_error_code", None) in USB_DISCONNECT_CODES:
        return True
      text = str(current).lower()
      if any(pattern in text for pattern in USB_DISCONNECT_TEXT):
        return True
      current = getattr(current, "__cause__", None) or getattr(current, "__context__", None)
    return False

  def _mark_disconnected(self):
    self.disconnected = True
    self.handle = None
    self.endpoint_out = None
    self.endpoint_in = None

  def _raise_usb_error(self, message, exc):
    if self._exception_indicates_disconnect(exc):
      self._mark_disconnected()
      raise IOError("TURZX USB display disconnected")
    raise IOError("%s: %s" % (message, exc))

  def _exchange(self, payload, timeout_ms):
    if self.handle is None:
      raise IOError("TURZX display is not open")
    try:
      written = self.handle.bulkWrite(self.endpoint_out, payload, timeout=timeout_ms)
      if written != len(payload):
        raise IOError("short TURZX USB write: %d/%d" % (written, len(payload)))
      return bytes(self.handle.bulkRead(self.endpoint_in, 512, timeout=timeout_ms))
    except Exception as exc:
      self._raise_usb_error("TURZX USB exchange failed", exc)

  def _write_no_ack(self, payload, timeout_ms):
    if self.handle is None:
      raise IOError("TURZX display is not open")
    try:
      written = self.handle.bulkWrite(self.endpoint_out, payload, timeout=timeout_ms)
      if written != len(payload):
        raise IOError("short TURZX USB write: %d/%d" % (written, len(payload)))
    except Exception as exc:
      self._raise_usb_error("TURZX USB write failed", exc)

  def _drain_input(self, attempts=1, timeout_ms=2):
    if self.handle is None or self.endpoint_in is None:
      return
    for _ in range(max(0, attempts)):
      try:
        self.handle.bulkRead(self.endpoint_in, 512, timeout=timeout_ms)
      except Exception as exc:
        if self._exception_indicates_disconnect(exc):
          self._mark_disconnected()
          raise IOError("TURZX USB display disconnected")
        # A timeout here simply means there is no optional/stale response.
        return

  def _send_command(self, command_id, fields=None):
    return self._exchange(_command_packet(command_id, fields), COMMAND_TIMEOUT_MS)

  def _send_optional_command(self, command_id, fields=None, gap_s=0.0, drain_attempts=0):
    self._write_no_ack(_command_packet(command_id, fields), COMMAND_TIMEOUT_MS)
    if gap_s > 0.0:
      time.sleep(gap_s)
    if drain_attempts:
      self._drain_input(attempts=drain_attempts)

  def send_jpeg(self, jpeg):
    header = _command_packet(CMD_UPLOAD_JPEG, {
      8: (len(jpeg) >> 24) & 0xFF,
      9: (len(jpeg) >> 16) & 0xFF,
      10: (len(jpeg) >> 8) & 0xFF,
      11: len(jpeg) & 0xFF,
    })
    # A no-ACK response can arrive after the short post-write timeout. Drain it
    # immediately before the next frame, as carrot-wip does, so a stale reply
    # cannot leave the panel showing only the first frame until USB reconnect.
    self._drain_input(attempts=2, timeout_ms=2)
    self._write_no_ack(header + jpeg, FRAME_TIMEOUT_MS)

  def close(self):
    handle = self.handle
    if handle is not None:
      # Do not issue another USB command after a physical disconnect.  Older
      # behaviour could turn one failure into a repeated init/off blink loop.
      if not self.disconnected:
        try:
          self._send_optional_command(CMD_BRIGHTNESS, {8: 0}, drain_attempts=0)
        except Exception:
          pass
      try:
        handle.releaseInterface(0)
      except Exception:
        pass
      try:
        handle.close()
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

#!/usr/bin/env python3
import glob
import os
import select
import termios
import time
from typing import Optional

import cereal.messaging as messaging
from selfdrive.swaglog import cloudlog

DEVICE_PATTERNS = (
  "/dev/ttyUSB*",
  "/dev/ttyACM*",
)
BAUDRATES = (115200, 9600, 38400, 57600)
READ_SIZE = 4096
RECONNECT_DELAY = 1.0


def _baud_constant(baudrate: int):
  return getattr(termios, "B%d" % baudrate)


def _configure_port(fd: int, baudrate: int) -> None:
  attrs = termios.tcgetattr(fd)
  attrs[0] = 0
  attrs[1] = 0
  attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
  attrs[3] = 0
  attrs[4] = _baud_constant(baudrate)
  attrs[5] = _baud_constant(baudrate)
  attrs[6][termios.VMIN] = 0
  attrs[6][termios.VTIME] = 1
  termios.tcsetattr(fd, termios.TCSANOW, attrs)
  termios.tcflush(fd, termios.TCIFLUSH)


def _candidate_devices():
  devices = []
  preferred = os.getenv("USB_GPS_DEVICE")
  if preferred:
    devices.append(preferred)
  for pattern in DEVICE_PATTERNS:
    devices.extend(sorted(glob.glob(pattern)))
  return list(dict.fromkeys(devices))


def _open_device() -> Optional[int]:
  baudrates = BAUDRATES
  requested_baud = os.getenv("USB_GPS_BAUD")
  if requested_baud:
    try:
      baudrates = (int(requested_baud),)
    except ValueError:
      cloudlog.warning("usb_gpsd: invalid USB_GPS_BAUD=%s", requested_baud)

  for device in _candidate_devices():
    for baudrate in baudrates:
      try:
        fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        _configure_port(fd, baudrate)
        cloudlog.warning("usb_gpsd: connected to %s at %d baud", device, baudrate)
        return fd
      except (OSError, termios.error):
        try:
          os.close(fd)
        except (OSError, UnboundLocalError):
          pass
  return None


def main() -> None:
  pm = messaging.PubMaster(["ubloxRaw"])

  while True:
    fd = _open_device()
    if fd is None:
      time.sleep(RECONNECT_DELAY)
      continue

    try:
      while True:
        ready, _, _ = select.select([fd], [], [], 1.0)
        if not ready:
          continue

        data = os.read(fd, READ_SIZE)
        if not data:
          raise OSError("USB GPS disconnected")

        msg = messaging.new_message("ubloxRaw")
        msg.ubloxRaw = data
        pm.send("ubloxRaw", msg)
    except (OSError, select.error) as e:
      cloudlog.warning("usb_gpsd: disconnected: %s", e)
    finally:
      try:
        os.close(fd)
      except OSError:
        pass
      time.sleep(RECONNECT_DELAY)


if __name__ == "__main__":
  main()

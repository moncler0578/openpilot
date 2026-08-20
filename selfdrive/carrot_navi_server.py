#!/usr/bin/env python3
import base64
import errno
import hashlib
import json
import os
import socket
import struct
import threading
import time
import uuid

from common.params import Params


PORT = 7714
DISCOVERY_PORT = 7705
STATE_FILE = "/dev/shm/carrot_navi_route.json"
MAP_FILE = "/dev/shm/carrot_navi_map.jpg"
JSON_NAMES = (
  "vehicle", "guidance_current", "guidance_next", "lane_current", "lane_ahead",
  "speed", "traffic_signal", "crossroad", "route", "navigation_status",
  "app_status", "camera_state", "composition_state",
)
IMAGE_NAMES = (
  "tbt_current_compact", "tbt_current_full", "tbt_next", "traffic_signal",
  "lane_top", "lane_bottom", "safety_primary", "safety_secondary",
  "safety_section", "crossroad_minimized", "crossroad_expanded",
  "center_tbt_icon", "center_tbt_text", "center_tbt_fee",
)
RENDER_NAMES = ("map_main",)
ENABLED = {
  "vehicle", "guidance_current", "guidance_next", "lane_current", "lane_ahead",
  "route", "navigation_status", "speed",
}
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
PROTOCOL_VERSION = 2
BINARY_HEADER = struct.Struct(">4sBBBBIIQQIHH")
MAX_MAP_FRAME_BYTES = 2 * 1024 * 1024
MAX_LANE_FRAME_BYTES = 512 * 1024
# Keep the native TMAP map as the base and also capture TMAP's own guidance
# widgets.  The EON never redraws these; remote_hud.py forwards the original
# compressed assets to the S9, which places them over the map at HUD scale.
OVERLAY_FILES = {
  "tbt_current_full": "/dev/shm/carrot_navi_tbt_current_full.png",
  "tbt_next": "/dev/shm/carrot_navi_tbt_next.png",
  "lane_bottom": "/dev/shm/carrot_navi_lane_bottom.png",
}
MAP_RENDER_WIDTH = 640
MAP_RENDER_HEIGHT = 384
PARAM_MAP_FPS = "EonClusterHudMapFps"
MAP_RENDER_FPS_DEFAULT = 5
MAP_RENDER_FPS_MIN = 2
MAP_RENDER_FPS_MAX = 5
STATE_WRITE_INTERVAL_S = 0.05

# TMAP's v2 render worker normally delivers map_main continuously.  When TMAP
# rebuilds its route/camera after a destination change, the worker can remain
# connected but stop producing frames for a while.  Ask the app to resync
# instead of leaving the HUD frozen on the last route for tens of seconds.
MAP_STALE_S = 3.0
MAP_RESYNC_INTERVAL_S = 2.0
MAP_STALE_CLEAR_S = 5.0


def map_render_fps(params):
  try:
    raw = params.get(PARAM_MAP_FPS)
    value = int(raw) if raw is not None else MAP_RENDER_FPS_DEFAULT
  except (TypeError, ValueError):
    value = MAP_RENDER_FPS_DEFAULT
  return max(MAP_RENDER_FPS_MIN, min(MAP_RENDER_FPS_MAX, value))


class NaviState(object):
  def __init__(self):
    self.params = Params()
    self.lock = threading.Lock()
    self.condition = threading.Condition(self.lock)
    self.values = {}
    self.updated_at = {}
    self.dirty = False
    self.last_write = 0.0
    self.last_map_write = 0.0
    self.last_map_rx = 0.0
    self.last_map_sequence = -1
    self.last_map_digest = None
    self.map_seen = False
    self.map_stale_cleared = False
    self.last_resync_request = 0.0
    self.route_signature = None
    self.route_change_pending = False
    self.route_change_started = 0.0
    self.route_change_baseline_digest = None
    self.control_conn = None
    self.control_send_lock = threading.Lock()
    self.session_id = uuid.uuid4().hex
    self.manifest_revision = 1
    threading.Thread(target=self._writer_loop, name="carrot-navi-state", daemon=True).start()
    threading.Thread(target=self._map_watchdog_loop, name="carrot-navi-map-watchdog", daemon=True).start()

  @staticmethod
  def _write_state(output):
    tmp = STATE_FILE + ".tmp"
    try:
      with open(tmp, "w") as f:
        json.dump(output, f, ensure_ascii=False, separators=(",", ":"))
      os.rename(tmp, STATE_FILE)
    except IOError:
      pass

  def _writer_loop(self):
    while True:
      with self.condition:
        while not self.dirty:
          self.condition.wait()
        delay = self.last_write + STATE_WRITE_INTERVAL_S - time.monotonic()
        if delay > 0.0:
          self.condition.wait(delay)
          continue
        output = dict(self.values)
        output["updated_at_ms"] = max(self.updated_at.values()) if self.updated_at else 0
        output["stream_updated_at_ms"] = dict(self.updated_at)
        self.dirty = False
        self.last_write = time.monotonic()
      self._write_state(output)

  @staticmethod
  def _route_signature(value):
    if not isinstance(value, dict):
      return None
    polyline = value.get("polyline")
    destination = None
    if isinstance(polyline, list) and polyline:
      point = polyline[-1]
      if isinstance(point, dict):
        try:
          destination = (round(float(point.get("lat")), 5),
                         round(float(point.get("lon")), 5))
        except (TypeError, ValueError):
          destination = None
    try:
      total_distance = float(value.get("total_distance_m", 0.0) or 0.0)
    except (TypeError, ValueError):
      total_distance = 0.0
    distance_bucket = int(round(total_distance / 100.0)) if total_distance > 0.0 else 0
    if destination is None and distance_bucket <= 0:
      return None
    return destination, distance_bucket

  def update(self, name, value):
    if name not in JSON_NAMES:
      return

    route_changed = False
    with self.lock:
      if name == "route":
        signature = self._route_signature(value)
        if signature is not None:
          if self.route_signature is not None and signature != self.route_signature:
            route_changed = True
            self.route_change_pending = True
            self.route_change_started = time.monotonic()
            self.route_change_baseline_digest = self.last_map_digest
          self.route_signature = signature

      self.values[name] = value
      now_ms = int(time.time() * 1000)
      self.updated_at[name] = now_ms
      self.dirty = True
      self.condition.notify()

    # A new route/destination should not wait for TMAP's long autonomous
    # recovery path.  Request map_main resync immediately; the watchdog retries
    # until a genuinely new map image arrives.
    if route_changed:
      self.request_map_resync()

  @staticmethod
  def _binary_payload(packet):
    """Return (message_type, format_or_reason, sequence, image_payload).

    Current TMAP v2 binary messages carry a fixed 40-byte CNV2 header.  Keep a
    tolerant fallback for the older raw-image form so this receiver does not
    regress if an older patched TMAP is used.
    """
    if len(packet) >= BINARY_HEADER.size and packet[:4] == b"CNV2":
      try:
        (magic, protocol_version, message_type, format_or_reason, _flags,
         _stream_handle, _revision, sequence, _source_timestamp_ms,
         payload_length, width, height) = BINARY_HEADER.unpack_from(packet)
      except struct.error:
        return None
      body = packet[BINARY_HEADER.size:]
      if magic != b"CNV2" or protocol_version != PROTOCOL_VERSION:
        return None
      if payload_length != len(body):
        return None
      if message_type == 4:
        if body or width != 0 or height != 0:
          return None
        return message_type, format_or_reason, sequence, b""
      if message_type != 1:
        return None
      return message_type, format_or_reason, sequence, body
    return 1, 0, -1, packet

  def update_map(self, payload):
    if not payload or len(payload) > MAX_MAP_FRAME_BYTES:
      return

    parsed = self._binary_payload(payload)
    if parsed is None:
      return
    message_type, format_or_reason, sequence, image_payload = parsed

    now = time.monotonic()
    if message_type == 4:
      # A valid stream-clear packet is activity and must immediately remove the
      # old route from both EON storage and the S9 map server.
      with self.lock:
        self.last_map_rx = now
        self.map_seen = True
        self.map_stale_cleared = True
        if sequence >= 0:
          self.last_map_sequence = sequence
      self.clear_map()
      return

    # map_main in this branch explicitly requests JPEG.  For CNV2, format 2 is
    # JPEG; legacy/raw packets use format 0 and are validated by signatures.
    if format_or_reason not in (0, 2):
      return

    start = image_payload.find(b"\xff\xd8")
    end = image_payload.rfind(b"\xff\xd9")
    if start < 0 or end < start:
      return
    image = image_payload[start:end + 2]

    # Only a validated JPEG counts as map activity. Otherwise malformed packets
    # could keep an old destination alive forever by refreshing last_map_rx.
    with self.lock:
      self.last_map_rx = now
      self.map_seen = True
      self.map_stale_cleared = False
      if sequence >= 0:
        self.last_map_sequence = sequence

    # A valid rate-limited frame still refreshed last_map_rx above.
    if now < self.last_map_write + 1.0 / map_render_fps(self.params):
      return
    image_digest = hashlib.sha1(image).digest()
    with self.lock:
      self.last_map_digest = image_digest
      if self.route_change_pending:
        baseline = self.route_change_baseline_digest
        # The first route has no baseline.  For a destination/re-route change,
        # require different map pixels before considering the HUD refreshed.
        if baseline is None or image_digest != baseline:
          self.route_change_pending = False
          self.route_change_started = 0.0
          self.route_change_baseline_digest = None

    tmp = MAP_FILE + ".tmp"
    try:
      with open(tmp, "wb") as f:
        f.write(image)
      os.rename(tmp, MAP_FILE)
      self.last_map_write = now
    except IOError:
      try:
        os.unlink(tmp)
      except OSError:
        pass

  @staticmethod
  def clear_map():
    try:
      os.unlink(MAP_FILE)
    except OSError:
      pass

  def update_overlay(self, name, payload):
    target = OVERLAY_FILES.get(name)
    if target is None or not payload or len(payload) > MAX_LANE_FRAME_BYTES:
      return

    parsed = self._binary_payload(payload)
    if parsed is None:
      return
    message_type, _format_or_reason, _sequence, image_payload = parsed
    if message_type == 4:
      self.clear_overlay(name)
      return

    png_start = image_payload.find(b"\x89PNG\r\n\x1a\n")
    jpg_start = image_payload.find(b"\xff\xd8")
    if png_start >= 0:
      image = image_payload[png_start:]
    elif jpg_start >= 0:
      jpg_end = image_payload.rfind(b"\xff\xd9")
      if jpg_end < jpg_start:
        return
      image = image_payload[jpg_start:jpg_end + 2]
    else:
      return
    tmp = target + ".tmp"
    try:
      with open(tmp, "wb") as f:
        f.write(image)
      os.rename(tmp, target)
    except IOError:
      try:
        os.unlink(tmp)
      except OSError:
        pass

  def clear_overlay(self, name):
    target = OVERLAY_FILES.get(name)
    if target is None:
      return
    try:
      os.unlink(target)
    except OSError:
      pass

  def update_lane(self, payload):
    self.update_overlay("lane_bottom", payload)

  def clear_lane(self):
    self.clear_overlay("lane_bottom")

  def control_connected(self, conn):
    with self.lock:
      self.control_conn = conn
      # A new control socket represents a new negotiation lifetime.  Reuse one
      # session id for repeated requirements_query messages on this socket so
      # TMAP does not see a different session every second.
      self.session_id = uuid.uuid4().hex
      self.manifest_revision = 1
      self.last_resync_request = 0.0

  def control_disconnected(self, conn):
    with self.lock:
      if self.control_conn is conn:
        self.control_conn = None

  def get_manifest(self):
    with self.lock:
      session_id = self.session_id
      revision = self.manifest_revision
    return manifest(self.params, session_id, revision)

  def send_control(self, message):
    with self.lock:
      conn = self.control_conn
    if conn is None:
      return False
    payload = json.dumps(message, separators=(",", ":"))
    try:
      with self.control_send_lock:
        send_frame(conn, payload)
      return True
    except (IOError, OSError, socket.error):
      return False

  def request_map_resync(self):
    now = time.monotonic()
    with self.lock:
      if self.control_conn is None or now < self.last_resync_request + MAP_RESYNC_INTERVAL_S:
        return False
      self.last_resync_request = now
    request = {
      "type": "resync_request",
      "protocol_version": PROTOCOL_VERSION,
      "timestamp_ms": int(time.time() * 1000),
      "request_id": uuid.uuid4().hex,
      "name": "map_main",
    }
    return self.send_control(request)

  def _map_watchdog_loop(self):
    while True:
      time.sleep(0.5)
      now = time.monotonic()
      with self.lock:
        map_seen = self.map_seen
        last_map_rx = self.last_map_rx
        control_present = self.control_conn is not None
        already_cleared = self.map_stale_cleared
        route_change_pending = self.route_change_pending
        route_change_started = self.route_change_started

      # A route/destination change can leave map_main sending the exact same
      # old JPEG repeatedly. Packet timestamps alone would look healthy, so
      # keep asking for resync until the actual map pixels change. If the
      # control socket is gone, clearing still proceeds after the same timeout.
      if route_change_pending:
        if control_present:
          self.request_map_resync()
        route_age = now - route_change_started if route_change_started > 0.0 else 0.0
        if route_age >= MAP_STALE_CLEAR_S and not already_cleared:
          self.clear_map()
          with self.lock:
            self.map_stale_cleared = True
        continue

      if not map_seen or last_map_rx <= 0.0:
        continue

      age = now - last_map_rx
      if age >= MAP_STALE_S and control_present:
        self.request_map_resync()

      # Clear stale pixels even when TMAP's control socket disconnected. The
      # next valid frame recreates the file and MapFrameServer forwards it.
      if age >= MAP_STALE_CLEAR_S and not already_cleared:
        self.clear_map()
        with self.lock:
          self.map_stale_cleared = True


def local_ip():
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  try:
    sock.connect(("8.8.8.8", 80))
    return sock.getsockname()[0]
  except socket.error:
    return "127.0.0.1"
  finally:
    sock.close()


def recv_exact(sock, length):
  data = b""
  while len(data) < length:
    part = sock.recv(length - len(data))
    if not part:
      raise EOFError()
    data += part
  return data


def recv_frame(sock):
  header = recv_exact(sock, 2)
  first, second = struct.unpack("!BB", header)
  opcode = first & 0x0f
  length = second & 0x7f
  if length == 126:
    length = struct.unpack("!H", recv_exact(sock, 2))[0]
  elif length == 127:
    length = struct.unpack("!Q", recv_exact(sock, 8))[0]
  mask = recv_exact(sock, 4) if second & 0x80 else None
  payload = recv_exact(sock, length)
  if mask:
    payload = bytes(bytearray(v ^ mask[i % 4] for i, v in enumerate(bytearray(payload))))
  return opcode, payload


def send_frame(sock, payload, opcode=1):
  if isinstance(payload, str):
    payload = payload.encode("utf-8")
  length = len(payload)
  header = struct.pack("!B", 0x80 | opcode)
  if length < 126:
    header += struct.pack("!B", length)
  elif length < 65536:
    header += struct.pack("!BH", 126, length)
  else:
    header += struct.pack("!BQ", 127, length)
  sock.sendall(header + payload)


def manifest(params, session_id, revision):
  map_fps = map_render_fps(params)
  streams = []
  handle = 1
  for kind, names in (("json", JSON_NAMES), ("image", IMAGE_NAMES), ("render", RENDER_NAMES)):
    for name in names:
      enabled = ((kind == "json" and name in ENABLED) or
                 (kind == "image" and name in OVERLAY_FILES) or
                 (kind == "render" and name == "map_main"))
      stream_params = {}
      if kind == "json":
        stream_params = {"delivery_mode": "on_change_with_heartbeat", "interval_ms": 500,
                         "stale_timeout_ms": 30000}
      elif kind == "image":
        # Native guidance bitmaps change slowly; 2 FPS keeps the S9 display
        # faithful without creating meaningful work on the EON.
        stream_params = {"format": "png", "max_fps": 2}
      else:
        stream_params = {"composition": "map_route_vehicle",
                         "width": MAP_RENDER_WIDTH, "height": MAP_RENDER_HEIGHT, "dpi": 160,
                         "fps": map_fps, "codec": "jpeg", "jpeg_quality": 65,
                         "camera_mode": "app_sync", "map_theme": "light",
                         "stale_timeout_ms": int(MAP_STALE_S * 1000)}
      streams.append({"kind": kind, "name": name, "schema_version": 1,
                      "stream_handle": handle, "enabled": enabled, "params": stream_params})
      handle += 1
  return {"type": "subscription_manifest", "protocol_version": PROTOCOL_VERSION,
          "session_id": session_id, "revision": revision,
          "metrics_enabled": False, "streams": streams}


def websocket_handshake(conn):
  request = b""
  while b"\r\n\r\n" not in request and len(request) < 16384:
    part = conn.recv(4096)
    if not part:
      raise EOFError()
    request += part
  lines = request.decode("iso-8859-1").split("\r\n")
  path = lines[0].split(" ")[1]
  headers = {}
  for line in lines[1:]:
    if ":" in line:
      key, value = line.split(":", 1)
      headers[key.strip().lower()] = value.strip()
  key = headers.get("sec-websocket-key", "")
  if not key:
    raise ValueError("missing websocket key")
  accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode("ascii")).digest()).decode("ascii")
  response = ("HTTP/1.1 101 Switching Protocols\r\n"
              "Upgrade: websocket\r\nConnection: Upgrade\r\n"
              "Sec-WebSocket-Accept: %s\r\n\r\n") % accept
  conn.sendall(response.encode("ascii"))
  return path


def client_loop(conn, state):
  path = websocket_handshake(conn)
  is_control = "/control/" in path
  stream_name = path.rstrip("/").split("/")[-1] if not is_control else None
  if is_control:
    state.control_connected(conn)
  try:
    while True:
      opcode, payload = recv_frame(conn)
      if opcode == 8:
        if is_control:
          with state.control_send_lock:
            send_frame(conn, b"", 8)
        else:
          send_frame(conn, b"", 8)
        return
      if opcode == 9:
        if is_control:
          with state.control_send_lock:
            send_frame(conn, payload, 10)
        else:
          send_frame(conn, payload, 10)
        continue
      if opcode == 2:
        if not is_control and stream_name == "map_main":
          state.update_map(payload)
        elif not is_control and stream_name in OVERLAY_FILES:
          state.update_overlay(stream_name, payload)
        continue
      if opcode != 1:
        continue
      try:
        message = json.loads(payload.decode("utf-8"))
      except (ValueError, UnicodeDecodeError):
        continue
      if is_control and message.get("type") == "requirements_query":
        state.send_control(state.get_manifest())
      elif is_control and message.get("type") == "catalog_query":
        # The app normally sends catalog_query only as a recovery path; reply
        # with the same stable manifest rather than creating a new session.
        state.send_control(state.get_manifest())
      elif not is_control and message.get("type") == "item_update":
        name = message.get("name", stream_name)
        value = message.get("value") if message.get("present", True) else None
        if name == "map_main" or name in OVERLAY_FILES:
          if value is None:
            if name == "map_main":
              state.clear_map()
            else:
              state.clear_overlay(name)
            continue
          encoded = value.get("data") if isinstance(value, dict) else value
          if isinstance(encoded, str):
            if "," in encoded and encoded.startswith("data:"):
              encoded = encoded.split(",", 1)[1]
            try:
              image = base64.b64decode(encoded)
              if name == "map_main":
                state.update_map(image)
              else:
                state.update_overlay(name, image)
            except (TypeError, ValueError):
              pass
        else:
          state.update(name, value)
  finally:
    if is_control:
      state.control_disconnected(conn)


def handle_client(conn, state):
  try:
    conn.settimeout(45)
    client_loop(conn, state)
  except (EOFError, IOError, ValueError, socket.error):
    pass
  finally:
    state.control_disconnected(conn)
    try:
      conn.close()
    except socket.error:
      pass


def discovery_loop():
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
  payload_ip = None
  while True:
    ip = local_ip()
    if ip != "127.0.0.1":
      payload_ip = ip
    if payload_ip:
      payload = json.dumps({"ip": payload_ip, "navi_debug": 0},
                           separators=(",", ":")).encode("utf-8")
      try:
        sock.sendto(payload, ("255.255.255.255", DISCOVERY_PORT))
      except socket.error as e:
        if e.errno not in (errno.ENETUNREACH, errno.EADDRNOTAVAIL):
          pass
    time.sleep(1)


def main():
  state = NaviState()
  threading.Thread(target=discovery_loop, name="carrot-navi-discovery", daemon=True).start()
  server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  server.bind(("0.0.0.0", PORT))
  server.listen(16)
  while True:
    conn, _ = server.accept()
    threading.Thread(target=handle_client, args=(conn, state), daemon=True).start()


if __name__ == "__main__":
  main()

import io
import json
import math
import os
import time

from PIL import Image, ImageDraw, ImageFont


NAVI_STATE = "/dev/shm/carrot_navi_route.json"
NAVI_MAP = "/dev/shm/carrot_navi_map.jpg"
NAVI_LANE = "/dev/shm/carrot_navi_lane_bottom.png"
NAVI_MAX_AGE_MS = 35000
_FONT_CACHE = {}
_IMAGE_CACHE = {}
_FIT_CACHE = {}


def _font(size, bold=False):
  key = (int(size), bool(bold))
  cached = _FONT_CACHE.get(key)
  if cached is not None:
    return cached
  candidates = [
    "/system/fonts/NotoSansCJK-Bold.ttc" if bold else "/system/fonts/NotoSansCJK-Regular.ttc",
    "/system/fonts/Roboto-Bold.ttf" if bold else "/system/fonts/Roboto-Regular.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
  ]
  for path in candidates:
    try:
      font = ImageFont.truetype(path, size)
      _FONT_CACHE[key] = font
      return font
    except IOError:
      pass
  font = ImageFont.load_default()
  _FONT_CACHE[key] = font
  return font


def read_navi_state(path=NAVI_STATE):
  try:
    with open(path, "r") as f:
      state = json.load(f)
  except (IOError, ValueError):
    return {}
  updated_at = int(state.get("updated_at_ms", 0) or 0)
  if updated_at <= 0 or abs(int(time.time() * 1000) - updated_at) > NAVI_MAX_AGE_MS:
    return {}
  return state


def _safe_image(path):
  try:
    stat = os.stat(path)
    signature = (stat.st_mtime, stat.st_size)
    cached = _IMAGE_CACHE.get(path)
    if cached is not None and cached[0] == signature:
      return cached[1]
    with Image.open(path) as source:
      image = source.convert("RGB")
    _IMAGE_CACHE[path] = (signature, image)
    return image
  except (IOError, OSError, ValueError):
    _IMAGE_CACHE.pop(path, None)
    return None


def _fit_cover(image, size):
  target_w, target_h = size
  scale = max(float(target_w) / image.width, float(target_h) / image.height)
  resized = image.resize((max(1, int(image.width * scale)), max(1, int(image.height * scale))), Image.BILINEAR)
  left = max(0, (resized.width - target_w) // 2)
  top = max(0, (resized.height - target_h) // 2)
  return resized.crop((left, top, left + target_w, top + target_h))


def _safe_fitted_image(path, size):
  image = _safe_image(path)
  if image is None:
    return None
  signature = _IMAGE_CACHE[path][0]
  key = (path, signature, int(size[0]), int(size[1]))
  cached = _FIT_CACHE.get(key)
  if cached is not None:
    return cached
  for old_key in list(_FIT_CACHE):
    if old_key[0] == path:
      del _FIT_CACHE[old_key]
  fitted = _fit_cover(image, size)
  _FIT_CACHE[key] = fitted
  return fitted


def _clamp(value, low, high):
  return max(low, min(high, value))


class HudRenderer(object):
  DRIVE_RATIO = 0.60
  MAX_DISTANCE_M = 120.0

  def __init__(self, width, height, jpeg_quality=58):
    self.width = int(width)
    self.height = int(height)
    self.jpeg_quality = max(35, min(75, int(jpeg_quality)))

  def _project(self, panel, longitudinal, lateral):
    left, top, right, bottom = panel
    distance = _clamp(float(longitudinal), 0.0, self.MAX_DISTANCE_M)
    depth = distance / self.MAX_DISTANCE_M
    perspective = math.pow(max(0.0, 1.0 - depth), 1.35)
    horizon = top + int((bottom - top) * 0.10)
    screen_y = horizon + (bottom - horizon) * perspective
    pixels_per_meter = 4.0 + 66.0 * perspective
    center_x = (left + right) * 0.5
    screen_x = center_x - float(lateral) * pixels_per_meter
    return int(round(screen_x)), int(round(screen_y))

  def _project_line(self, panel, points):
    projected = []
    for longitudinal, lateral in points:
      if 0.0 <= longitudinal <= self.MAX_DISTANCE_M:
        point = self._project(panel, longitudinal, lateral)
        if not projected or point != projected[-1]:
          projected.append(point)
    return projected

  def _fallback_lanes(self):
    distances = (0.0, 8.0, 18.0, 32.0, 52.0, 78.0, 110.0)
    return [
      {"points": [(distance, -1.8) for distance in distances], "probability": 0.65},
      {"points": [(distance, 1.8) for distance in distances], "probability": 0.65},
    ]

  def _draw_polyline(self, draw, panel, points, fill, width):
    projected = self._project_line(panel, points)
    if len(projected) >= 2:
      draw.line(projected, fill=fill, width=max(1, int(width)))

  def _draw_path(self, draw, panel, points):
    if len(points) < 2:
      points = [(0.0, 0.0), (12.0, 0.0), (30.0, 0.0), (60.0, 0.0), (100.0, 0.0)]
    left_edge = []
    right_edge = []
    for longitudinal, lateral in points:
      if 0.0 <= longitudinal <= self.MAX_DISTANCE_M:
        half_width = 0.78 + 0.18 * min(1.0, longitudinal / 60.0)
        left_edge.append(self._project(panel, longitudinal, lateral + half_width))
        right_edge.append(self._project(panel, longitudinal, lateral - half_width))
    polygon = left_edge + list(reversed(right_edge))
    if len(polygon) >= 4:
      draw.polygon(polygon, fill=(25, 104, 205))
      center = self._project_line(panel, points)
      if len(center) >= 2:
        draw.line(center, fill=(90, 181, 255), width=max(3, self.height // 80))

  def _draw_lead(self, draw, panel, lead, primary):
    distance = float(lead.get("distance", 0.0) or 0.0)
    lateral = float(lead.get("lateral", 0.0) or 0.0)
    if distance <= 0.0 or distance > self.MAX_DISTANCE_M:
      return
    cx, cy = self._project(panel, distance, lateral)
    scale = math.pow(max(0.10, 1.0 - distance / self.MAX_DISTANCE_M), 1.15)
    car_w = int(24 + 100 * scale)
    car_h = int(16 + 62 * scale)
    color = (255, 169, 45) if primary else (105, 177, 255)
    box = (cx - car_w // 2, cy - car_h, cx + car_w // 2, cy)
    draw.rounded_rectangle(box, radius=max(4, car_w // 10), fill=(28, 35, 43), outline=color,
                           width=max(2, self.height // 115))
    relative_kph = float(lead.get("relative_speed", 0.0) or 0.0) * 3.6
    label = "%dm  %+.0f" % (int(round(distance)), relative_kph)
    draw.text((cx, cy - car_h - 8), label, font=_font(max(14, self.height // 25), True),
              fill=color, anchor="ms")

  def _draw_speed_limit(self, draw, x, y, limit):
    if limit <= 0:
      return
    radius = max(29, self.height // 10)
    draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=(250, 250, 250),
                 outline=(220, 45, 45), width=max(6, radius // 6))
    draw.text((x, y), str(limit), font=_font(max(24, radius), True), fill=(20, 20, 20), anchor="mm")

  def _draw_driving_mode(self, draw, box, mode):
    modes = {
      1: ("ECO", (40, 210, 125)),
      2: ("SAFE", (255, 169, 45)),
      3: ("NORM", (235, 240, 245)),
      4: ("FAST", (235, 70, 70)),
    }
    if mode not in modes:
      return
    label, color = modes[mode]
    left, top, right, _ = box
    x = (left + right) // 2
    draw.rounded_rectangle((x - 58, top + 15, x + 58, top + 58), radius=12,
                           fill=(18, 25, 33), outline=color, width=2)
    draw.text((x, top + 36), label, font=_font(max(17, self.height // 23), True),
              fill=color, anchor="mm")

  def _draw_tpms(self, draw, box, tpms):
    if not tpms:
      return
    values = [tpms.get(key) for key in ("fl", "fr", "rl", "rr")]
    valid = [value for value in values if value is not None and 5.0 <= float(value) <= 60.0]
    if not valid:
      return
    _, _, right, bottom = box
    center_x = right - max(88, self.width // 25)
    center_y = bottom - max(78, self.height // 6)
    car_w = max(28, self.height // 12)
    car_h = max(64, self.height // 5)
    draw.rounded_rectangle((center_x - car_w // 2, center_y - car_h // 2,
                            center_x + car_w // 2, center_y + car_h // 2),
                           radius=8, fill=(28, 35, 43), outline=(90, 105, 118), width=2)
    offsets = ((-55, -32), (55, -32), (-55, 32), (55, 32))
    for value, (dx, dy) in zip(values, offsets):
      is_valid = value is not None and 5.0 <= float(value) <= 60.0
      text = str(int(round(float(value)))) if is_valid else "--"
      color = (235, 70, 70) if is_valid and float(value) < 31.0 else (220, 228, 234)
      draw.text((center_x + dx, center_y + dy), text,
                font=_font(max(16, self.height // 24), True), fill=color, anchor="mm")

  def _draw_alert(self, draw, alert):
    if not alert or not alert.get("text1"):
      return
    status = str(alert.get("status", "")).lower()
    critical = status in ("critical", "2") or "critical" in status
    color = (225, 55, 55) if critical else (255, 169, 45)
    height = max(105, int(self.height * 0.31))
    top = (self.height - height) // 2
    margin = max(34, self.width // 32)
    draw.rounded_rectangle((margin, top, self.width - margin, top + height), radius=22,
                           fill=(10, 14, 19), outline=color, width=max(4, self.height // 80))
    text1 = str(alert.get("text1", ""))
    text2 = str(alert.get("text2", ""))
    draw.text((self.width // 2, top + int(height * 0.38)), text1,
              font=_font(max(30, self.height // 10), True), fill=(250, 250, 250), anchor="mm")
    if text2:
      draw.text((self.width // 2, top + int(height * 0.73)), text2,
                font=_font(max(20, self.height // 16), True), fill=(205, 215, 222), anchor="mm")

  def _draw_driving_panel(self, image, draw, box, speed_kph, cruise_kph, enabled, limit, scene):
    left, top, right, bottom = box
    draw.rectangle(box, fill=(6, 10, 16))
    horizon = top + int((bottom - top) * 0.10)
    draw.polygon(((left + int((right - left) * 0.13), bottom),
                  (left + int((right - left) * 0.43), horizon),
                  (left + int((right - left) * 0.57), horizon),
                  (right - int((right - left) * 0.13), bottom)), fill=(16, 22, 29))

    scene = scene or {}
    for edge in scene.get("edges", []):
      probability = float(edge.get("probability", 0.5) or 0.5)
      color = (int(85 + 100 * probability), int(42 + 35 * probability), int(42 + 35 * probability))
      self._draw_polyline(draw, box, edge.get("points", []), color, max(2, self.height // 105))

    lanes = scene.get("lanes", []) or self._fallback_lanes()
    for index, lane in enumerate(lanes):
      probability = float(lane.get("probability", 0.5) or 0.5)
      intensity = int(95 + 150 * _clamp(probability, 0.0, 1.0))
      color = (intensity, intensity, int(115 + 140 * _clamp(probability, 0.0, 1.0)))
      self._draw_polyline(draw, box, lane.get("points", []), color,
                          max(2, int(self.height * (0.006 + 0.005 * probability))))

    self._draw_path(draw, box, scene.get("path", []))
    for index, lead in enumerate(scene.get("leads", [])[:2]):
      self._draw_lead(draw, box, lead, index == 0)

    status_color = (40, 210, 125) if enabled else (115, 125, 135)
    speed_y = bottom - int((bottom - top) * 0.08)
    draw.text((left + 34, speed_y), str(max(0, int(round(speed_kph)))),
              font=_font(max(58, int(self.height * 0.25)), True), fill=(245, 248, 250), anchor="ls")
    draw.text((left + int((right - left) * 0.24), speed_y - 4), "km/h",
              font=_font(max(17, self.height // 22)), fill=(145, 158, 168), anchor="ls")
    cruise = "--" if cruise_kph <= 0 or cruise_kph >= 255 else str(int(round(cruise_kph)))
    draw.text((left + int((right - left) * 0.24), speed_y - max(34, self.height // 9)), "SET " + cruise,
              font=_font(max(22, self.height // 13), True), fill=status_color, anchor="ls")
    self._draw_speed_limit(draw, left + 70, top + 72, limit)
    self._draw_driving_mode(draw, box, int(scene.get("driving_mode", 0) or 0))
    self._draw_tpms(draw, box, scene.get("tpms"))
    draw.text((right - 22, top + 24), "LIGHT 3D",
              font=_font(max(14, self.height // 28)), fill=(115, 132, 145), anchor="ra")

  def _draw_navi_panel(self, image, draw, box, navi):
    left, top, right, bottom = box
    panel_w = right - left
    map_image = _safe_fitted_image(NAVI_MAP, (panel_w, bottom - top))
    if map_image is not None and navi:
      image.paste(map_image, (left, top))
    else:
      draw.rectangle(box, fill=(10, 17, 24))
      draw.text(((left + right) // 2, (top + bottom) // 2), "TMAP WAIT",
                font=_font(max(20, self.height // 15), True), fill=(120, 135, 145), anchor="mm")

    if not navi:
      return
    guide = navi.get("guidance_current") or {}
    route = navi.get("route") or {}
    instruction = guide.get("main_text") or guide.get("road_name") or "안내 없음"
    distance = int(guide.get("distance_m", -1) or -1)
    card = (left + 18, top + 18, right - 18, top + int(self.height * 0.31))
    draw.rounded_rectangle(card, radius=18, fill=(12, 18, 25), outline=(52, 151, 108), width=3)
    if len(instruction) > 17:
      instruction = instruction[:16] + "…"
    draw.text((card[0] + 20, card[1] + 18), instruction, font=_font(max(24, self.height // 12), True),
              fill=(240, 242, 244), anchor="la")
    if distance >= 0:
      distance_text = ("%.1f km" % (distance / 1000.0)) if distance >= 1000 else ("%d m" % distance)
      draw.text((card[0] + 20, card[3] - 18), distance_text,
                font=_font(max(23, self.height // 11), True), fill=(64, 181, 255), anchor="ls")

    lane_w, lane_h = int(panel_w * 0.58), int(self.height * 0.18)
    lane = _safe_fitted_image(NAVI_LANE, (lane_w, lane_h))
    if lane is not None:
      lane_x = right - lane_w - 18
      lane_y = bottom - lane_h - 18
      image.paste(lane, (lane_x, lane_y))

    remain_distance = int(route.get("remain_distance_m", -1) or -1)
    if remain_distance >= 0:
      remain = ("남은 거리 %.1f km" % (remain_distance / 1000.0)) if remain_distance >= 1000 else ("남은 거리 %d m" % remain_distance)
      draw.rounded_rectangle((left + 18, bottom - 62, left + min(panel_w - 18, 330), bottom - 18),
                             radius=12, fill=(12, 18, 25))
      draw.text((left + 32, bottom - 39), remain, font=_font(max(16, self.height // 20), True),
                fill=(205, 215, 222), anchor="lm")

  def _draw_trip_report(self, draw, box, report):
    left, top, right, bottom = box
    draw.rectangle(box, fill=(7, 12, 18))
    title_size = max(24, self.height // 12)
    body_size = max(19, self.height // 18)
    draw.text(((left + right) // 2, top + 42), "DRIVING REPORT",
              font=_font(title_size, True), fill=(235, 240, 245), anchor="mm")
    duration_s = max(0.0, float(report.get("duration_s", 0.0) or 0.0))
    distance_km = max(0.0, float(report.get("distance_m", 0.0) or 0.0)) / 1000.0
    rows = (
      ("TIME", "%02d:%02d" % (int(duration_s) // 3600, (int(duration_s) // 60) % 60)),
      ("DIST", "%.1f km" % distance_km),
      ("AVG", "%.0f km/h" % float(report.get("average_speed_kph", 0.0) or 0.0)),
      ("MAX", "%.0f km/h" % float(report.get("max_speed_kph", 0.0) or 0.0)),
    )
    card_left, card_right = left + 24, right - 24
    row_h = max(58, (bottom - top - 92) // len(rows))
    for index, (label, value) in enumerate(rows):
      row_top = top + 72 + index * row_h
      draw.rounded_rectangle((card_left, row_top, card_right, row_top + row_h - 8), radius=12,
                             fill=(16, 23, 32), outline=(55, 68, 80), width=2)
      draw.text((card_left + 18, row_top + (row_h - 8) // 2), label,
                font=_font(body_size, True), fill=(145, 158, 168), anchor="lm")
      draw.text((card_right - 18, row_top + (row_h - 8) // 2), value,
                font=_font(body_size, True), fill=(235, 240, 245), anchor="rm")

  def render(self, speed_kph, cruise_kph, enabled, navi=None, scene=None):
    navi = navi or {}
    scene = scene or {}
    image = Image.new("RGB", (self.width, self.height), (5, 8, 12))
    draw = ImageDraw.Draw(image)
    divider = int(self.width * self.DRIVE_RATIO)
    speed_state = navi.get("speed") or {}
    limit = int(speed_state.get("road_limit_kph", 0) or 0)
    panel_layout = int(scene.get("panel_layout", 0) or 0)
    driving_box = (0, 0, divider - 3, self.height)
    info_box = (divider + 3, 0, self.width, self.height)
    if panel_layout == 1:
      info_width = self.width - divider
      info_box = (0, 0, info_width - 3, self.height)
      driving_box = (info_width + 3, 0, self.width, self.height)
    self._draw_driving_panel(image, draw, driving_box,
                             speed_kph, cruise_kph, enabled, limit, scene)
    if panel_layout == 1:
      split = self.width - divider
      draw.rectangle((split - 3, 0, split + 3, self.height), fill=(34, 42, 50))
    else:
      draw.rectangle((divider - 3, 0, divider + 3, self.height), fill=(34, 42, 50))
    if scene.get("parked") and scene.get("trip_report"):
      self._draw_trip_report(draw, info_box, scene["trip_report"])
    else:
      self._draw_navi_panel(image, draw, info_box, navi)
    self._draw_alert(draw, scene.get("alert"))
    return image

  def encode_portrait_jpeg(self, image):
    portrait = image.transpose(Image.ROTATE_90)
    output = io.BytesIO()
    portrait.save(output, format="JPEG", quality=self.jpeg_quality, optimize=False,
                  progressive=False, subsampling=2)
    return output.getvalue()

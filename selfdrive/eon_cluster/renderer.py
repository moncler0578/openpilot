import io
import json
import os
import time

from PIL import Image, ImageDraw, ImageFont


NAVI_STATE = "/dev/shm/carrot_navi_route.json"
NAVI_MAP = "/dev/shm/carrot_navi_map.jpg"
NAVI_LANE = "/dev/shm/carrot_navi_lane_bottom.png"
NAVI_MAX_AGE_MS = 35000


def _font(size, bold=False):
  candidates = [
    "/system/fonts/NotoSansCJK-Bold.ttc" if bold else "/system/fonts/NotoSansCJK-Regular.ttc",
    "/system/fonts/Roboto-Bold.ttf" if bold else "/system/fonts/Roboto-Regular.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
  ]
  for path in candidates:
    try:
      return ImageFont.truetype(path, size)
    except IOError:
      pass
  return ImageFont.load_default()


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
    if not os.path.exists(path):
      return None
    with Image.open(path) as source:
      return source.convert("RGB")
  except (IOError, ValueError):
    return None


def _fit_cover(image, size):
  target_w, target_h = size
  scale = max(float(target_w) / image.width, float(target_h) / image.height)
  resized = image.resize((max(1, int(image.width * scale)), max(1, int(image.height * scale))), Image.BILINEAR)
  left = max(0, (resized.width - target_w) // 2)
  top = max(0, (resized.height - target_h) // 2)
  return resized.crop((left, top, left + target_w, top + target_h))


class HudRenderer(object):
  def __init__(self, width, height, jpeg_quality=58):
    self.width = int(width)
    self.height = int(height)
    self.jpeg_quality = max(35, min(75, int(jpeg_quality)))

  def render(self, speed_kph, cruise_kph, enabled, navi=None):
    navi = navi or {}
    image = Image.new("RGB", (self.width, self.height), (5, 8, 12))
    draw = ImageDraw.Draw(image)
    map_width = int(self.width * 0.52)
    map_image = _safe_image(NAVI_MAP)
    if map_image is not None and navi:
      image.paste(_fit_cover(map_image, (map_width, self.height)), (0, 0))
      draw.rectangle((map_width - 4, 0, map_width, self.height), fill=(34, 42, 50))
    else:
      draw.rectangle((0, 0, map_width, self.height), fill=(10, 17, 24))
      draw.text((map_width // 2, self.height // 2), "TMAP WAIT", font=_font(max(20, self.height // 15), True),
                fill=(120, 135, 145), anchor="mm")

    panel_x = map_width + 28
    status_color = (40, 210, 125) if enabled else (115, 125, 135)
    draw.rounded_rectangle((panel_x, 20, self.width - 20, self.height - 20), radius=24,
                           fill=(12, 18, 25), outline=status_color, width=5)
    speed_font = _font(max(72, int(self.height * 0.34)), True)
    draw.text((panel_x + 35, int(self.height * 0.42)), str(max(0, int(round(speed_kph)))),
              font=speed_font, fill=(245, 248, 250), anchor="lm")
    draw.text((self.width - 55, int(self.height * 0.25)), "km/h", font=_font(max(20, self.height // 13)),
              fill=(145, 158, 168), anchor="ra")
    cruise = "--" if cruise_kph <= 0 or cruise_kph >= 255 else str(int(round(cruise_kph)))
    draw.text((self.width - 55, int(self.height * 0.43)), "SET " + cruise,
              font=_font(max(24, self.height // 10), True), fill=status_color, anchor="ra")

    guide = navi.get("guidance_current") or {}
    route = navi.get("route") or {}
    speed_state = navi.get("speed") or {}
    instruction = guide.get("main_text") or guide.get("road_name") or "안내 없음"
    distance = int(guide.get("distance_m", -1) or -1)
    limit = int(speed_state.get("road_limit_kph", 0) or 0)
    guide_y = int(self.height * 0.62)
    draw.text((panel_x + 36, guide_y), instruction, font=_font(max(24, self.height // 11), True),
              fill=(240, 242, 244), anchor="la")
    if distance >= 0:
      distance_text = ("%.1f km" % (distance / 1000.0)) if distance >= 1000 else ("%d m" % distance)
      draw.text((panel_x + 36, guide_y + max(35, self.height // 9)), distance_text,
                font=_font(max(22, self.height // 12), True), fill=(64, 181, 255), anchor="la")
    if limit > 0:
      radius = max(35, self.height // 11)
      cx, cy = self.width - radius - 52, int(self.height * 0.70)
      draw.ellipse((cx - radius, cy - radius, cx + radius, cy + radius), fill=(250, 250, 250),
                   outline=(220, 45, 45), width=max(7, radius // 6))
      draw.text((cx, cy), str(limit), font=_font(max(25, radius), True), fill=(20, 20, 20), anchor="mm")

    lane = _safe_image(NAVI_LANE)
    if lane is not None and navi:
      lane_w, lane_h = int(self.width * 0.18), int(self.height * 0.18)
      image.paste(_fit_cover(lane, (lane_w, lane_h)), (map_width - lane_w - 18, self.height - lane_h - 18))

    remain_distance = int(route.get("remain_distance_m", -1) or -1)
    if remain_distance >= 0:
      remain = ("남은 거리 %.1f km" % (remain_distance / 1000.0)) if remain_distance >= 1000 else ("남은 거리 %d m" % remain_distance)
      draw.text((panel_x + 36, self.height - 54), remain, font=_font(max(18, self.height // 18)),
                fill=(150, 164, 174), anchor="ls")
    return image

  def encode_portrait_jpeg(self, image):
    portrait = image.transpose(Image.ROTATE_90)
    output = io.BytesIO()
    portrait.save(output, format="JPEG", quality=self.jpeg_quality, optimize=False,
                  progressive=False, subsampling=2)
    return output.getvalue()

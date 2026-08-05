static QJsonValue carrotFirstJsonValue(const QJsonObject &object,
                                       std::initializer_list<const char *> keys) {
  for (const char *key : keys) {
    if (object.contains(key)) return object.value(key);
  }
  return QJsonValue();
}

static bool carrotJsonBool(const QJsonObject &object,
                           std::initializer_list<const char *> keys) {
  const QJsonValue value = carrotFirstJsonValue(object, keys);
  if (value.isBool()) return value.toBool();
  if (value.isDouble()) return value.toInt() != 0;
  const QString text = value.toString().toLower();
  return text == "true" || text == "yes" || text == "active" || text == "recommended";
}

// Normalize lane directions from nMirror/Tmap. Lane direction values can be
// numbers, strings, or arrays depending on the APK version.
static int carrotLaneTypeValue(const QJsonValue &value) {
  if (value.isDouble()) return value.toInt();

  if (value.isArray()) {
    bool straight = false;
    bool left = false;
    bool right = false;
    bool uturn = false;
    int fallback = 0;
    bool has_value = false;
    for (const QJsonValue &item : value.toArray()) {
      const int type = carrotLaneTypeValue(item);
      has_value = true;
      fallback = type;
      straight |= type == 0 || type == 20 || type == 21;
      left |= type == 12 || type == 16 || type == 20;
      right |= type == 13 || type == 18 || type == 21;
      uturn |= type == 14;
    }
    if (uturn) return 14;
    if (straight && left) return 20;
    if (straight && right) return 21;
    if (left && right) return 22;
    return has_value ? fallback : 0;
  }

  const QString text = value.toString().trimmed().toLower();
  if (text.isEmpty()) return 0;

  bool numeric = false;
  const int numeric_type = text.toInt(&numeric);
  if (numeric) return numeric_type;

  const bool uturn = text.contains("uturn") || text.contains("u-turn") ||
                     text.contains(QString::fromUtf8("유턴"));
  const bool straight = text.contains("straight") || text.contains("through") ||
                        text.contains("forward") || text.contains(QString::fromUtf8("직진")) ||
                        text.contains(QString::fromUtf8("직좌")) ||
                        text.contains(QString::fromUtf8("좌직")) ||
                        text.contains(QString::fromUtf8("직우")) ||
                        text.contains(QString::fromUtf8("우직"));
  const bool left = text.contains("left") || text.contains(QString::fromUtf8("좌"));
  const bool right = text.contains("right") || text.contains(QString::fromUtf8("우"));

  if (uturn) return 14;
  if (straight && left) return 20;   // straight + left
  if (straight && right) return 21;  // straight + right
  if (left && right) return 22;      // left + right
  if (text.contains("slight_left") || text.contains("slight left")) return 16;
  if (text.contains("slight_right") || text.contains("slight right")) return 18;
  if (left) return 12;
  if (right) return 13;
  return 0;
}

// The Tmap bridge has used both array and object lane payloads.  Normalize the
// common spellings here so old and new APK builds can share the same UI.
static void parseCarrotLanes(const QJsonValue &value, QVector<int> &types,
                             QVector<int> &active) {
  types.clear();
  active.clear();
  if (value.isNull() || value.isUndefined()) return;

  QJsonArray lanes;
  QJsonObject wrapper;
  if (value.isArray()) {
    lanes = value.toArray();
  } else if (value.isObject()) {
    wrapper = value.toObject();
    const QJsonValue lane_value = carrotFirstJsonValue(
      wrapper, {"lanes", "lane_info", "laneInfo", "items", "lane_items", "laneItems", "data"});
    if (lane_value.isArray()) lanes = lane_value.toArray();
  }

  if (!lanes.isEmpty()) {
    const int count = std::min(8, lanes.size());
    for (int i = 0; i < count; ++i) {
      const QJsonValue lane = lanes.at(i);
      int type = 0;
      bool recommended = false;
      if (lane.isObject()) {
        const QJsonObject item = lane.toObject();
        type = carrotLaneTypeValue(carrotFirstJsonValue(
          item, {"turn_type", "turnType", "lane_type", "laneType",
                 "direction", "directions", "type"}));
        recommended = carrotJsonBool(item, {"recommended", "is_recommended",
                                             "isRecommended", "active", "is_active",
                                             "isActive", "selected", "usable"});
      } else if (lane.isBool()) {
        recommended = lane.toBool();
      } else if (lane.isDouble() || lane.isString() || lane.isArray()) {
        type = carrotLaneTypeValue(lane);
      }
      types.push_back(type);
      active.push_back(recommended ? 1 : 0);
    }
  } else if (!wrapper.isEmpty()) {
    const QJsonArray direction_values = carrotFirstJsonValue(
      wrapper, {"directions", "lane_types", "laneTypes", "lane_type"}).toArray();
    int count = carrotFirstJsonValue(
      wrapper, {"lane_count", "laneCount", "count", "total"}).toInt();
    if (count <= 0) count = direction_values.size();
    count = std::max(0, std::min(8, count));
    for (int i = 0; i < count; ++i) {
      types.push_back(i < direction_values.size() ? carrotLaneTypeValue(direction_values.at(i)) : 0);
      active.push_back(0);
    }
  }

  if (types.isEmpty()) return;
  const QJsonArray active_values = carrotFirstJsonValue(
    wrapper, {"recommended_lanes", "recommendedLanes", "active_lanes", "activeLanes",
              "lane_available", "laneAvailable", "available", "recommended"}).toArray();
  if (active_values.size() == types.size()) {
    for (int i = 0; i < active_values.size(); ++i) {
      const QJsonValue flag = active_values.at(i);
      active[i] = flag.isBool() ? flag.toBool() : flag.toInt() != 0;
    }
  } else {
    for (const QJsonValue &index_value : active_values) {
      int index = index_value.toInt(-1);
      if (index >= 1 && index <= types.size()) --index;  // accept one-based indexes
      if (index >= 0 && index < active.size()) active[index] = 1;
    }
  }
}

void NvgWindow::updateCarrotNavi(bool load_images) {
  const uint64_t now = millis_since_boot();
  if (now - carrot_navi_last_read < 500) return;
  carrot_navi_last_read = now;

  QFile file("/dev/shm/carrot_navi_route.json");
  if (!file.open(QIODevice::ReadOnly)) return;
  QJsonParseError error;
  const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &error);
  if (error.error != QJsonParseError::NoError || !doc.isObject()) return;

  const QJsonObject root = doc.object();

  // nMirror가 제공하는 실제 Tmap 하단 차선 안내 이미지를 2Hz로 갱신한다.
  // JSON 방향값은 APK 버전마다 달라 이미지가 있으면 이를 우선 사용한다.
  if (load_images) {
    const QString lane_image_path = "/dev/shm/carrot_navi_lane_bottom.png";
    const QFileInfo lane_image_info(lane_image_path);
    const qint64 lane_now = QDateTime::currentMSecsSinceEpoch();
    if (lane_image_info.exists()) {
      const qint64 lane_mtime = lane_image_info.lastModified().toMSecsSinceEpoch();
      if (lane_now - lane_mtime >= -5000 && lane_now - lane_mtime <= 5000) {
        if (lane_mtime != carrot_navi_lane_image_mtime || carrot_navi_lane_image.isNull()) {
          QPixmap next_lane_image;
          if (next_lane_image.load(lane_image_path)) {
            carrot_navi_lane_image = next_lane_image;
            carrot_navi_lane_scaled_image = QPixmap();
            carrot_navi_lane_scaled_width = 0;
            carrot_navi_lane_scaled_height = 0;
            carrot_navi_lane_image_mtime = lane_mtime;
          }
        }
      } else {
        carrot_navi_lane_image = QPixmap();
        carrot_navi_lane_scaled_image = QPixmap();
        carrot_navi_lane_scaled_width = 0;
        carrot_navi_lane_scaled_height = 0;
        carrot_navi_lane_image_mtime = 0;
      }
    } else {
      carrot_navi_lane_image = QPixmap();
      carrot_navi_lane_scaled_image = QPixmap();
      carrot_navi_lane_scaled_width = 0;
      carrot_navi_lane_scaled_height = 0;
      carrot_navi_lane_image_mtime = 0;
    }
  } else if (!carrot_navi_lane_image.isNull() || !carrot_navi_lane_scaled_image.isNull()) {
    carrot_navi_lane_image = QPixmap();
    carrot_navi_lane_scaled_image = QPixmap();
    carrot_navi_lane_scaled_width = 0;
    carrot_navi_lane_scaled_height = 0;
    carrot_navi_lane_image_mtime = 0;
  }

  carrot_navi_updated_at = root.value("updated_at_ms").toVariant().toULongLong();
  const QJsonObject stream_times = root.value("stream_updated_at_ms").toObject();
  carrot_navi_guidance_updated_at = stream_times.contains("guidance_current")
    ? stream_times.value("guidance_current").toVariant().toULongLong()
    : carrot_navi_updated_at;
  const QJsonObject vehicle = root.value("vehicle").toObject();
  carrot_navi_lat = vehicle.value("lat").toDouble();
  carrot_navi_lon = vehicle.value("lon").toDouble();
  carrot_navi_road = vehicle.value("road_name").toString();

  const QJsonObject guide = root.value("guidance_current").toObject();
  carrot_navi_instruction = guide.value("main_text").toString();
  if (carrot_navi_instruction.isEmpty()) carrot_navi_instruction = guide.value("road_name").toString();
  carrot_navi_distance = guide.contains("distance_m") ? guide.value("distance_m").toInt() : -1;
  carrot_navi_turn_type = guide.value("turn_type").toInt();

  const QJsonObject next_guide = root.value("guidance_next").toObject();
  carrot_navi_next_instruction = next_guide.value("main_text").toString();
  if (carrot_navi_next_instruction.isEmpty()) carrot_navi_next_instruction = next_guide.value("road_name").toString();
  carrot_navi_next_distance = next_guide.contains("distance_m") ? next_guide.value("distance_m").toInt() : -1;
  carrot_navi_next_turn_type = next_guide.value("turn_type").toInt();

  QVector<int> current_types, current_active, ahead_types, ahead_active;
  parseCarrotLanes(root.value("lane_current"), current_types, current_active);
  parseCarrotLanes(root.value("lane_ahead"), ahead_types, ahead_active);
  carrot_navi_lanes_ahead = current_types.isEmpty() && !ahead_types.isEmpty();
  carrot_navi_lane_types = carrot_navi_lanes_ahead ? ahead_types : current_types;
  carrot_navi_lane_active = carrot_navi_lanes_ahead ? ahead_active : current_active;

  const QJsonObject route = root.value("route").toObject();
  carrot_navi_remain_distance = route.contains("remain_distance_m") ? route.value("remain_distance_m").toInt() : -1;
  carrot_navi_remain_time = route.contains("remain_time_sec") ? route.value("remain_time_sec").toInt() : -1;
  const QJsonArray points = route.value("polyline").toArray();
  if (root.contains("route") && points.isEmpty()) {
    carrot_navi_route.clear();
  } else if (!points.isEmpty()) {
    QVector<QPointF> next;
    const int stride = std::max(1, (points.size() + 179) / 180);
    next.reserve(std::min(180, points.size()));
    for (int i = 0; i < points.size(); i += stride) {
      const QJsonValue value = points.at(i);
      const QJsonObject point = value.toObject();
      const double lat = point.value("lat").toDouble();
      const double lon = point.value("lon").toDouble();
      if (std::abs(lat) > 0.000001 && std::abs(lon) > 0.000001) next.push_back(QPointF(lon, lat));
    }
    if (!points.isEmpty() && (points.size() - 1) % stride != 0) {
      const QJsonObject point = points.last().toObject();
      const double lat = point.value("lat").toDouble();
      const double lon = point.value("lon").toDouble();
      if (std::abs(lat) > 0.000001 && std::abs(lon) > 0.000001) next.push_back(QPointF(lon, lat));
    }
    carrot_navi_route = next;
  }

  // ---- speed 스트림: 도로 제한속도 (내비 중 LIMIT 폴백용) ----
  carrot_navi_speed_limit = 0;
  const QJsonObject speed = root.value("speed").toObject();
  if (!speed.isEmpty()) {
    // 서버 키 형식(snake/camel) 불확실 → 후보 키 순차 시도 (도로 제한속도 우선)
    static const char *limit_keys[] = {
      "road_limit_kph", "limit_speed", "roadLimitKph",
      "section_speed_limit_kph", "sectionSpeedLimitKph",
      "sdi_speed_limit_kph", "sdiSpeedLimitKph"
    };
    for (const char *k : limit_keys) {
      if (speed.contains(k)) {
        const int v = speed.value(k).toInt();
        if (v > 0 && v <= 150) { carrot_navi_speed_limit = v; break; }
      }
    }
  }
}

static QString carrotDistanceText(int meters) {
  if (meters < 0) return QString();
  if (meters < 1000) return QString("%1 m").arg(meters);
  return QString("%1 km").arg(meters / 1000.0, 0, 'f', meters < 10000 ? 1 : 0);
}

enum class CarrotTurnDirection {
  STRAIGHT, LEFT, RIGHT, STRAIGHT_LEFT, STRAIGHT_RIGHT, LEFT_RIGHT,
  UTURN, SLIGHT_LEFT, SLIGHT_RIGHT, ARRIVE
};

enum class CarrotAtcKind { NONE, TURN, FORK, UTURN, ROTARY };

static bool carrotAtcTypeIn(int type, std::initializer_list<int> types) {
  return std::find(types.begin(), types.end(), type) != types.end();
}

static CarrotAtcKind carrotAtcKind(int type, const QString &text, int *direction) {
  *direction = 0;
  if (carrotAtcTypeIn(type, {12, 16})) {
    *direction = -1;
    return CarrotAtcKind::TURN;
  }
  if (carrotAtcTypeIn(type, {13, 19})) {
    *direction = 1;
    return CarrotAtcKind::TURN;
  }
  if (carrotAtcTypeIn(type, {7, 17, 44, 75, 76, 102, 105, 112, 115, 118})) {
    *direction = -1;
    return CarrotAtcKind::FORK;
  }
  if (carrotAtcTypeIn(type, {6, 43, 73, 74, 101, 104, 111, 114, 117, 123, 124})) {
    *direction = 1;
    return CarrotAtcKind::FORK;
  }
  if (type == 14) {
    *direction = -1;
    return CarrotAtcKind::UTURN;
  }
  if (type >= 131 && type <= 142) return CarrotAtcKind::ROTARY;

  const QString lower = text.toLower();
  if (lower.contains(QString::fromUtf8("유턴")) || lower.contains("u-turn") || lower.contains("uturn")) {
    *direction = -1;
    return CarrotAtcKind::UTURN;
  }
  const bool fork = lower.contains(QString::fromUtf8("분기")) ||
                    lower.contains(QString::fromUtf8("진출")) || lower.contains("fork");
  if (lower.contains(QString::fromUtf8("좌회전")) || lower.contains(QString::fromUtf8("왼쪽")) || lower.contains("left")) {
    *direction = -1;
    return fork ? CarrotAtcKind::FORK : CarrotAtcKind::TURN;
  }
  if (lower.contains(QString::fromUtf8("우회전")) || lower.contains(QString::fromUtf8("오른쪽")) || lower.contains("right")) {
    *direction = 1;
    return fork ? CarrotAtcKind::FORK : CarrotAtcKind::TURN;
  }
  return CarrotAtcKind::NONE;
}

static CarrotTurnDirection carrotTurnDirection(int type, const QString &text) {
  const QString value = text.toLower();
  if (type == 20) return CarrotTurnDirection::STRAIGHT_LEFT;
  if (type == 21) return CarrotTurnDirection::STRAIGHT_RIGHT;
  if (type == 22) return CarrotTurnDirection::LEFT_RIGHT;
  if (value.contains(QString::fromUtf8("유턴")) || value.contains("u-turn") || type == 14) return CarrotTurnDirection::UTURN;
  if (value.contains(QString::fromUtf8("목적지")) || value.contains(QString::fromUtf8("도착")) || type == 2) return CarrotTurnDirection::ARRIVE;
  if (value.contains(QString::fromUtf8("왼쪽 방향")) || value.contains(QString::fromUtf8("좌측 방향")) ||
      value.contains(QString::fromUtf8("비스듬히 왼쪽")) || type == 16 || type == 17) return CarrotTurnDirection::SLIGHT_LEFT;
  if (value.contains(QString::fromUtf8("오른쪽 방향")) || value.contains(QString::fromUtf8("우측 방향")) ||
      value.contains(QString::fromUtf8("비스듬히 오른쪽")) || type == 18 || type == 19) return CarrotTurnDirection::SLIGHT_RIGHT;
  if (value.contains(QString::fromUtf8("좌회전")) || value.contains(QString::fromUtf8("왼쪽")) || type == 12) return CarrotTurnDirection::LEFT;
  if (value.contains(QString::fromUtf8("우회전")) || value.contains(QString::fromUtf8("오른쪽")) || type == 13) return CarrotTurnDirection::RIGHT;
  return CarrotTurnDirection::STRAIGHT;
}

static void drawCarrotTurnArrow(QPainter &p, const QRect &box, int type, const QString &text,
                                const QColor &color, int width) {
  const CarrotTurnDirection direction = carrotTurnDirection(type, text);
  const QPointF c = box.center();
  const qreal s = std::min(box.width(), box.height()) * 0.34;
  QPainterPath path;
  if (direction == CarrotTurnDirection::ARRIVE) {
    p.setPen(QPen(color, width));
    p.setBrush(color);
    p.drawEllipse(c, s * 0.45, s * 0.45);
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(c, s * 0.85, s * 0.85);
    return;
  } else if (direction == CarrotTurnDirection::STRAIGHT_LEFT ||
             direction == CarrotTurnDirection::STRAIGHT_RIGHT ||
             direction == CarrotTurnDirection::LEFT_RIGHT) {
    const bool draw_straight = direction != CarrotTurnDirection::LEFT_RIGHT;
    const bool draw_left = direction == CarrotTurnDirection::STRAIGHT_LEFT ||
                           direction == CarrotTurnDirection::LEFT_RIGHT;
    const bool draw_right = direction == CarrotTurnDirection::STRAIGHT_RIGHT ||
                            direction == CarrotTurnDirection::LEFT_RIGHT;

    if (draw_straight) {
      path.moveTo(c.x(), c.y() + s);
      path.lineTo(c.x(), c.y() - s);
      path.moveTo(c.x() - s * 0.35, c.y() - s * 0.6);
      path.lineTo(c.x(), c.y() - s);
      path.lineTo(c.x() + s * 0.35, c.y() - s * 0.6);
    }
    if (draw_left) {
      path.moveTo(c.x(), c.y() + s * 0.25);
      path.lineTo(c.x(), c.y() - s * 0.15);
      path.lineTo(c.x() - s, c.y() - s * 0.15);
      path.moveTo(c.x() - s * 0.55, c.y() - s * 0.5);
      path.lineTo(c.x() - s, c.y() - s * 0.15);
      path.lineTo(c.x() - s * 0.55, c.y() + s * 0.2);
    }
    if (draw_right) {
      path.moveTo(c.x(), c.y() + s * 0.25);
      path.lineTo(c.x(), c.y() - s * 0.15);
      path.lineTo(c.x() + s, c.y() - s * 0.15);
      path.moveTo(c.x() + s * 0.55, c.y() - s * 0.5);
      path.lineTo(c.x() + s, c.y() - s * 0.15);
      path.lineTo(c.x() + s * 0.55, c.y() + s * 0.2);
    }
  } else if (direction == CarrotTurnDirection::UTURN) {
    path.moveTo(c.x() + s * 0.55, c.y() + s);
    path.lineTo(c.x() + s * 0.55, c.y() - s * 0.25);
    path.cubicTo(c.x() + s * 0.55, c.y() - s, c.x() - s * 0.65, c.y() - s, c.x() - s * 0.65, c.y() - s * 0.2);
    path.moveTo(c.x() - s, c.y() - s * 0.35);
    path.lineTo(c.x() - s * 0.65, c.y() + s * 0.05);
    path.lineTo(c.x() - s * 0.3, c.y() - s * 0.35);
  } else {
    qreal dx = 0.0;
    if (direction == CarrotTurnDirection::LEFT) dx = -s;
    else if (direction == CarrotTurnDirection::RIGHT) dx = s;
    else if (direction == CarrotTurnDirection::SLIGHT_LEFT) dx = -s * 0.7;
    else if (direction == CarrotTurnDirection::SLIGHT_RIGHT) dx = s * 0.7;
    path.moveTo(c.x(), c.y() + s);
    if (direction == CarrotTurnDirection::LEFT || direction == CarrotTurnDirection::RIGHT) {
      path.lineTo(c.x(), c.y() - s * 0.2);
      path.lineTo(c.x() + dx, c.y() - s * 0.2);
    } else {
      path.lineTo(c.x() + dx, c.y() - s);
    }
    const QPointF tip = path.currentPosition();
    const qreal side = dx < 0 ? 1.0 : (dx > 0 ? -1.0 : 0.0);
    if (dx == 0.0) {
      path.moveTo(tip.x() - s * 0.35, tip.y() + s * 0.4);
      path.lineTo(tip);
      path.lineTo(tip.x() + s * 0.35, tip.y() + s * 0.4);
    } else {
      path.moveTo(tip.x() + side * s * 0.05, tip.y() + s * 0.45);
      path.lineTo(tip);
      path.lineTo(tip.x() + side * s * 0.45, tip.y() + s * 0.05);
    }
  }
  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(color, width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.drawPath(path);
}

void NvgWindow::drawCarrotNavi(QPainter &p) {
  const qint64 wall_now = QDateTime::currentMSecsSinceEpoch();
  if (carrot_navi_updated_at == 0 || wall_now - static_cast<qint64>(carrot_navi_updated_at) > 35000) return;
  const bool route_active = carrot_navi_route.size() >= 2 &&
                            (carrot_navi_remain_distance > 0 || carrot_navi_remain_time > 0);
  if (!route_active) return;

  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  p.setRenderHint(QPainter::TextAntialiasing);
  // 우측 내비 패널
  const int panel_y = 78;                  // 상단 정보줄 아래 18px 간격
  const int panel_bottom = height() - 5;  // 좌측 HUD 하단과 동일, 상태바 위
  const int panel_h = panel_bottom - panel_y;

  // 내부 레이아웃
  const int panel_header_h = 118;
  const int next_h = 54;
  const int panel_footer_h = 64;  // 흰 배경 큰 글씨(남은거리/도착)가 여유있게 들어가도록 확장

  const int gap1 = 10;
  const int gap2 = 8;
  const int bottom_margin = 14;
  // 차선안내는 항상 티맵이 실제로 보낸 lane_bottom 이미지가 있을 때만 표시한다.
  // (예전 turn_type 배열로 그리던 자체 벡터 아이콘 폴백은 완전히 제거)
  const bool has_lane_image = !carrot_navi_lane_image.isNull() &&
                              wall_now - carrot_navi_lane_image_mtime >= -5000 &&
                              wall_now - carrot_navi_lane_image_mtime <= 5000;
  const bool has_lane_guidance = has_lane_image;

  // Lane guidance no longer reserves its own row — like the real Tmap, it's
  // a small overlay box drawn on top of the map (see lane_row below), so the
  // map always gets the full height down to the footer regardless of
  // has_lane_guidance.
  const int map_to_footer_h = gap2;
  const int map_h =
      panel_h
      - panel_header_h
      - next_h
      - panel_footer_h
      - gap1
      - map_to_footer_h
      - bottom_margin;

  const QRect panel(width() - 475, panel_y, 440, panel_h);

  const QRect header(
      panel.x(),
      panel.y(),
      panel.width(),
      panel_header_h);

  const QRect next_row(
      panel.x(),
      header.bottom() + 1,
      panel.width(),
      next_h);

  const QRect map_rect(
      panel.x() + 14,
      next_row.bottom() + gap1,
      panel.width() - 28,
      map_h);

  // Small black box overlaid inside the map, bottom-anchored with a margin
  // on all sides so it floats over the map surface the way Tmap's own lane
  // guidance does — it never touches or reserves space from the footer.
  const int lane_overlay_margin = 10;
  const int lane_overlay_h = 56;
  const QRect lane_row(
      map_rect.x() + lane_overlay_margin,
      map_rect.bottom() - lane_overlay_h - lane_overlay_margin,
      map_rect.width() - lane_overlay_margin * 2,
      lane_overlay_h);

  const QRect footer(
      panel.x() + 14,
      map_rect.bottom() + gap2,
      panel.width() - 28,
      panel_footer_h);
  p.setPen(QPen(QColor(255, 255, 255, 120), 2));
  p.setBrush(QColor(8, 14, 18, 225));
  p.drawRoundedRect(panel, 24, 24);

  // Tmap-style primary guidance: green surface and high-contrast white content.
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(20, 126, 78));
  p.drawRoundedRect(header, 24, 24);
  p.drawRect(QRect(header.left(), header.bottom() - 24, header.width(), 25));

  QString title = carrot_navi_instruction;
  if (title.isEmpty()) title = carrot_navi_road;
  if (title.isEmpty()) title = QString::fromUtf8("경로 안내");
  const QRect current_icon(header.x() + 14, header.y() + 10, 94, 108);
  drawCarrotTurnArrow(p, current_icon, carrot_navi_turn_type, title, Qt::white, 11);
  const QString distance_text = carrotDistanceText(carrot_navi_distance);
  configFont(p, "Open Sans", 53, "Bold");
  p.setPen(Qt::white);
  p.drawText(QRect(header.x() + 118, header.y() + 6, header.width() - 132, 66),
             Qt::AlignLeft | Qt::AlignVCenter,
             distance_text.isEmpty() ? QString::fromUtf8("안내 중") : distance_text);
  configFont(p, "Open Sans", 31, "Bold");
  const QString road_text = QFontMetrics(p.font()).elidedText(title, Qt::ElideRight, header.width() - 142);
  p.drawText(QRect(header.x() + 118, header.y() + 70, header.width() - 132, 45),
             Qt::AlignLeft | Qt::AlignVCenter, road_text);

  // Secondary maneuver row uses a darker green and a smaller white arrow.
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(10, 82, 50));
  p.drawRect(next_row);
  drawCarrotTurnArrow(p, QRect(next_row.x() + 14, next_row.y() + 7, 44, 44),
                      carrot_navi_next_turn_type, carrot_navi_next_instruction,
                      Qt::white, 5);
  QString next_text = carrotDistanceText(carrot_navi_next_distance);
  if (!carrot_navi_next_instruction.isEmpty()) {
    next_text += (next_text.isEmpty() ? QString() : QString("  ·  ")) + carrot_navi_next_instruction;
  }
  if (next_text.isEmpty()) next_text = QString::fromUtf8("다음 안내 대기 중");
  configFont(p, "Open Sans", 27, "Bold");
  p.setPen(Qt::white);
  p.drawText(QRect(next_row.x() + 68, next_row.y(), next_row.width() - 80, next_row.height()),
             Qt::AlignLeft | Qt::AlignVCenter,
             QFontMetrics(p.font()).elidedText(next_text, Qt::ElideRight, next_row.width() - 88));

  // Prefer the Tmap-rendered map stream while it is fresh. Polling is capped at
  // 5 Hz; the existing vector map below remains the complete fallback.
  static const QString map_main_path = "/dev/shm/carrot_navi_map.jpg";
  static QPixmap map_main_pixmap;
  static uint64_t map_main_checked_at = 0;
  static qint64 map_main_modified_at = 0;
  const uint64_t map_main_now = millis_since_boot();
  if (map_main_checked_at == 0 || map_main_now - map_main_checked_at >= 200) {
    map_main_checked_at = map_main_now;
    const QFileInfo map_main_info(map_main_path);
    if (!map_main_info.exists() || !map_main_info.isFile() || map_main_info.size() <= 0) {
      map_main_pixmap = QPixmap();
      map_main_modified_at = 0;
    } else {
      const qint64 modified_at = map_main_info.lastModified().toMSecsSinceEpoch();
      if (map_main_pixmap.isNull() || modified_at != map_main_modified_at) {
        QPixmap candidate;
        if (candidate.load(map_main_path, "JPEG")) {
          map_main_pixmap = candidate;
          map_main_modified_at = modified_at;
        } else {
          map_main_pixmap = QPixmap();
          map_main_modified_at = 0;
        }
      }
    }
  }
  const qint64 map_main_age = wall_now - map_main_modified_at;
  const bool map_main_fresh = !map_main_pixmap.isNull() &&
                              map_main_age >= 0 && map_main_age <= 2500;

  // CPU: pre-crop + pre-scale the JPEG once per source frame (≤5Hz) or once
  // per map_rect resize, instead of re-running SmoothPixmapTransform on every
  // UI paint (previously ~20Hz regardless of whether the source had changed).
  static QPixmap map_main_scaled;
  static qint64 map_main_scaled_src_modified_at = -1;
  static int map_main_scaled_w = 0, map_main_scaled_h = 0;
  if (map_main_fresh &&
      (map_main_scaled_src_modified_at != map_main_modified_at ||
       map_main_scaled_w != map_rect.width() || map_main_scaled_h != map_rect.height())) {
    // map_main is rendered at 160 dpi. Crop the proportional Android/Tmap
    // status bar, then use a centered 1.18x cover crop so the map keeps its
    // aspect ratio instead of being stretched to the panel dimensions.
    const qreal status_bar_h = std::min<qreal>(map_main_pixmap.height() / 8.0,
                                               map_main_pixmap.width() * 0.05);
    const QRectF map_content(0.0, status_bar_h, map_main_pixmap.width(),
                             map_main_pixmap.height() - status_bar_h);
    const qreal target_aspect = static_cast<qreal>(map_rect.width()) / map_rect.height();
    qreal source_w = map_content.width();
    qreal source_h = map_content.height();
    if (source_w / source_h > target_aspect) {
      source_w = source_h * target_aspect;
    } else {
      source_h = source_w / target_aspect;
    }
    constexpr qreal map_zoom = 1.18;
    source_w /= map_zoom;
    source_h /= map_zoom;
    const QRectF map_source(map_content.center().x() - source_w / 2.0,
                            map_content.center().y() - source_h / 2.0,
                            source_w, source_h);
    QPixmap cropped = map_main_pixmap.copy(map_source.toRect())
                          .scaled(map_rect.size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    // Bake the rounded-corner mask in here, once per source frame, instead of
    // clipping with a QPainterPath on every single UI paint. Use a QImage
    // with an explicit alpha format — a bare QPixmap isn't guaranteed to
    // carry an alpha channel, so the transparent corners could come out
    // opaque/garbage on some backends.
    QImage rounded(map_rect.size(), QImage::Format_ARGB32_Premultiplied);
    rounded.fill(Qt::transparent);
    QPainter rp(&rounded);
    rp.setRenderHint(QPainter::Antialiasing);
    QPainterPath mask;
    mask.addRoundedRect(QRectF(QPointF(0, 0), map_rect.size()), 14, 14);
    rp.setClipPath(mask);
    rp.drawPixmap(0, 0, cropped);
    rp.end();
    map_main_scaled = QPixmap::fromImage(rounded);
    map_main_scaled_src_modified_at = map_main_modified_at;
    map_main_scaled_w = map_rect.width();
    map_main_scaled_h = map_rect.height();
  }

  if (map_main_fresh) {
    // The real Tmap frame fully covers map_rect — the vector fallback below
    // would be entirely hidden, so skip drawing it and save the CPU. Corner
    // rounding is already baked into map_main_scaled (see above).
    p.drawPixmap(map_rect.topLeft(), map_main_scaled);
  } else {
  // Lightweight schematic map background: a few fixed vector shapes only.
  QPainterPath map_clip;
  map_clip.addRoundedRect(map_rect, 14, 14);
  p.save();
  p.setClipPath(map_clip);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(34, 44, 51));
  p.drawRect(map_rect);
  p.setBrush(QColor(35, 72, 58));
  p.drawRoundedRect(QRect(map_rect.x() + 20, map_rect.y() + 18, 105, 67), 12, 12);
  p.drawRoundedRect(QRect(map_rect.right() - 115, map_rect.bottom() - 78, 96, 59), 12, 12);
  QPainterPath water;
  water.moveTo(map_rect.left() - 10, map_rect.y() + 205);
  water.cubicTo(map_rect.x() + 100, map_rect.y() + 150, map_rect.x() + 250, map_rect.y() + 245,
                map_rect.right() + 10, map_rect.y() + 185);
  water.lineTo(map_rect.right() + 10, map_rect.bottom() + 20);
  water.lineTo(map_rect.left() - 10, map_rect.bottom() + 20);
  water.closeSubpath();
  p.setBrush(QColor(38, 75, 96));
  p.drawPath(water);
  p.setBrush(QColor(55, 62, 68));
  for (int i = 0; i < 6; ++i) {
    const int bx = map_rect.x() + 24 + (i % 3) * 113;
    const int by = map_rect.y() + 99 + (i / 3) * 58;
    p.drawRoundedRect(QRect(bx, by, 66 + (i % 2) * 18, 30), 4, 4);
  }
  QPainterPath road_a, road_b, road_c;
  road_a.moveTo(map_rect.left() - 10, map_rect.y() + 85);
  road_a.cubicTo(map_rect.x() + 115, map_rect.y() + 135, map_rect.x() + 260, map_rect.y() + 30,
                 map_rect.right() + 10, map_rect.y() + 74);
  road_b.moveTo(map_rect.x() + 82, map_rect.top() - 10);
  road_b.cubicTo(map_rect.x() + 105, map_rect.y() + 105, map_rect.x() + 42, map_rect.y() + 205,
                 map_rect.x() + 142, map_rect.bottom() + 10);
  road_c.moveTo(map_rect.x() + 274, map_rect.top() - 10);
  road_c.cubicTo(map_rect.x() + 232, map_rect.y() + 78, map_rect.x() + 350, map_rect.y() + 155,
                 map_rect.x() + 306, map_rect.bottom() + 10);
  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(QColor(18, 23, 27, 130), 13, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.drawPath(road_a); p.drawPath(road_b); p.drawPath(road_c);
  p.setPen(QPen(QColor(104, 110, 114), 7, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.drawPath(road_a); p.drawPath(road_b); p.drawPath(road_c);

  if (carrot_navi_route.size() >= 2) {
    double min_lon = carrot_navi_route[0].x(), max_lon = min_lon;
    double min_lat = carrot_navi_route[0].y(), max_lat = min_lat;
    for (const QPointF &point : carrot_navi_route) {
      min_lon = std::min(min_lon, point.x()); max_lon = std::max(max_lon, point.x());
      min_lat = std::min(min_lat, point.y()); max_lat = std::max(max_lat, point.y());
    }
    const bool has_car = std::abs(carrot_navi_lat) > 0.000001 && std::abs(carrot_navi_lon) > 0.000001;
    if (has_car && carrot_navi_lon > min_lon - 0.02 && carrot_navi_lon < max_lon + 0.02 &&
        carrot_navi_lat > min_lat - 0.02 && carrot_navi_lat < max_lat + 0.02) {
      min_lon = std::min(min_lon, carrot_navi_lon); max_lon = std::max(max_lon, carrot_navi_lon);
      min_lat = std::min(min_lat, carrot_navi_lat); max_lat = std::max(max_lat, carrot_navi_lat);
    }
    const double lon_span = std::max(0.00001, max_lon - min_lon);
    const double lat_span = std::max(0.00001, max_lat - min_lat);
    const double scale = std::min((map_rect.width() - 28) / lon_span, (map_rect.height() - 28) / lat_span);
    const double left = map_rect.center().x() - lon_span * scale / 2.0;
    const double top = map_rect.center().y() - lat_span * scale / 2.0;
    const auto project = [&](double lon, double lat) {
      return QPointF(left + (lon - min_lon) * scale, top + (max_lat - lat) * scale);
    };

    int car_idx = 0;
    if (has_car) {
      double best = 1e18;
      for (int i = 0; i < carrot_navi_route.size(); ++i) {
        const double dlon = carrot_navi_route[i].x() - carrot_navi_lon;
        const double dlat = carrot_navi_route[i].y() - carrot_navi_lat;
        const double d = dlon * dlon + dlat * dlat;
        if (d < best) { best = d; car_idx = i; }
      }
    }
    const auto smooth_path = [&](int first, int last) {
      QPainterPath path;
      if (first < 0 || last < first || last >= carrot_navi_route.size()) return path;
      path.moveTo(project(carrot_navi_route[first].x(), carrot_navi_route[first].y()));
      for (int i = first + 1; i < last; ++i) {
        const QPointF point = project(carrot_navi_route[i].x(), carrot_navi_route[i].y());
        const QPointF next = project(carrot_navi_route[i + 1].x(), carrot_navi_route[i + 1].y());
        path.quadTo(point, (point + next) / 2.0);
      }
      if (last > first) path.lineTo(project(carrot_navi_route[last].x(), carrot_navi_route[last].y()));
      return path;
    };

    const QPainterPath full_path = smooth_path(0, carrot_navi_route.size() - 1);
    const QPainterPath remain_path = smooth_path(car_idx, carrot_navi_route.size() - 1);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(QColor(0, 0, 0, 185), 22, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPath(full_path);
    if (car_idx > 0) {
      // Tmap day 테마의 unselected 경로 색(#8C9299)과 동일하게 지나온 구간 표시
      p.setPen(QPen(QColor(0x8C, 0x92, 0x99), 10, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
      p.drawPath(smooth_path(0, car_idx));
    }
    // Tmap theme_navi_day.json 의 routeLine 색(strokeColor #254ea1 / fillColor #3673ee, width 17)과 동일하게 맞춤
    p.setPen(QPen(QColor(0x25, 0x4e, 0xa1), 17, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPath(remain_path);
    p.setPen(QPen(QColor(0x36, 0x73, 0xee), 12, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPath(remain_path);
    p.setPen(QPen(QColor(112, 207, 255), 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPath(remain_path);

    const QPointF destination = project(carrot_navi_route.last().x(), carrot_navi_route.last().y());
    p.setPen(QPen(Qt::white, 3)); p.setBrush(QColor(238, 74, 72));
    p.drawEllipse(destination, 11, 11);
    if (has_car) {
      const QPointF car = project(carrot_navi_lon, carrot_navi_lat);
      const int next_index = std::min(car_idx + 1, carrot_navi_route.size() - 1);
      const QPointF ahead = project(carrot_navi_route[next_index].x(), carrot_navi_route[next_index].y());
      p.save();
      p.translate(car);
      p.rotate(atan2(ahead.y() - car.y(), ahead.x() - car.x()) * 57.29577951);
      QPainterPath car_arrow;
      car_arrow.moveTo(17, 0); car_arrow.lineTo(-10, -12); car_arrow.lineTo(-4, 0);
      car_arrow.lineTo(-10, 12); car_arrow.closeSubpath();
      p.setPen(QPen(Qt::white, 2)); p.setBrush(QColor(26, 190, 104));
      p.drawPath(car_arrow);
      p.restore();
    }
  }
  p.restore();
  }

  // Lane guidance: draw Tmap's own lane_bottom capture as-is, as a small
  // overlay box floating inside the map (lane_row is now inset from
  // map_rect's bottom, not a separate row — see layout above). There is no
  // placeholder/vector fallback any more — has_lane_guidance is exactly
  // has_lane_image, so this whole block is skipped unless Tmap is actually
  // sending a fresh lane message right now.
  if (has_lane_guidance) {
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(18, 26, 32, 235));
    p.drawRoundedRect(lane_row, 12, 12);

    QPainterPath lane_clip;
    lane_clip.addRoundedRect(lane_row, 12, 12);
    p.save();
    p.setClipPath(lane_clip);
    // Smooth scaling is expensive on EON. Rebuild the cache only for a new
    // source image (cache cleared in updateCarrotNavi) or a new target size.
    if (carrot_navi_lane_scaled_image.isNull() ||
        carrot_navi_lane_scaled_width != lane_row.width() ||
        carrot_navi_lane_scaled_height != lane_row.height()) {
      carrot_navi_lane_scaled_image = carrot_navi_lane_image.scaled(
          lane_row.size(), Qt::KeepAspectRatioByExpanding, Qt::SmoothTransformation);
      carrot_navi_lane_scaled_width = lane_row.width();
      carrot_navi_lane_scaled_height = lane_row.height();
    }
    const int source_x = std::max(0, (carrot_navi_lane_scaled_image.width() - lane_row.width()) / 2);
    const int source_y = std::max(0, (carrot_navi_lane_scaled_image.height() - lane_row.height()) / 2);
    p.drawPixmap(lane_row, carrot_navi_lane_scaled_image,
                 QRect(source_x, source_y, lane_row.width(), lane_row.height()));
    p.restore();
  }

  // Remaining distance and ETA: match Tmap's own bottom bar — white surface,
  // black bold text, each side sized as large as it can be without the two
  // halves (or the divider) overlapping.
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(255, 255, 255, 250));
  p.drawRoundedRect(footer, 12, 12);

  const QString remain_text = carrotDistanceText(carrot_navi_remain_distance);
  const QString eta_text = carrot_navi_remain_time > 0
    ? QDateTime::currentDateTime().addSecs(carrot_navi_remain_time).toString("HH:mm") : QString("--:--");
  const QString left_text = QString::fromUtf8("남은 거리 %1").arg(remain_text.isEmpty() ? QString("--") : remain_text);
  const QString right_text = QString::fromUtf8("도착 %1").arg(eta_text);

  const int pad = 16;
  const int half_w = footer.width() / 2 - pad - 6;
  // CPU: the text (and therefore the fitted size) only actually changes when
  // updateCarrotNavi() refreshes (≤2Hz) or the ETA clock ticks over a minute —
  // not on every ~20Hz UI paint. Cache per side so the shrink-to-fit loop only
  // reruns when its own text or the box width actually changed.
  static QString left_cache_text; static int left_cache_w = -1; static int left_cache_size = 34;
  static QString right_cache_text; static int right_cache_w = -1; static int right_cache_size = 34;
  const auto fitFontSize = [&](const QString &text, int max_w,
                               QString &cache_text, int &cache_w, int &cache_size) {
    if (cache_text != text || cache_w != max_w) {
      int size = 34;
      for (; size > 18; --size) {
        configFont(p, "Open Sans", size, "Bold");
        if (QFontMetrics(p.font()).horizontalAdvance(text) <= max_w) break;
      }
      cache_text = text;
      cache_w = max_w;
      cache_size = size;
    }
    configFont(p, "Open Sans", cache_size, "Bold");
  };

  p.setPen(QColor(20, 20, 20));
  fitFontSize(left_text, half_w, left_cache_text, left_cache_w, left_cache_size);
  p.drawText(QRect(footer.x() + pad, footer.y(), half_w, footer.height()),
             Qt::AlignLeft | Qt::AlignVCenter, left_text);

  fitFontSize(right_text, half_w, right_cache_text, right_cache_w, right_cache_size);
  p.drawText(QRect(footer.center().x() + 6, footer.y(), half_w, footer.height()),
             Qt::AlignRight | Qt::AlignVCenter, right_text);

  p.setPen(QPen(QColor(210, 210, 210), 2));
  p.drawLine(footer.center().x(), footer.y() + 10, footer.center().x(), footer.bottom() - 10);
  p.restore();
}

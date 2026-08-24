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
  const QString lane_image_path = "/dev/shm/carrot_navi_lane_bottom.png";
  const QFileInfo lane_image_info(lane_image_path);
  const qint64 lane_now = QDateTime::currentMSecsSinceEpoch();
  if (load_images) {
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
  } else {
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
  uint64_t route_updated_at = carrot_navi_updated_at;
  static const char *route_time_keys[] = {"route", "route_geometry", "polyline"};
  for (const char *key : route_time_keys) {
    if (stream_times.contains(key)) {
      route_updated_at = stream_times.value(key).toVariant().toULongLong();
      break;
    }
  }
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
  if (route_updated_at == 0 || route_updated_at != carrot_navi_route_updated_at) {
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
      if ((points.size() - 1) % stride != 0) {
        const QJsonObject point = points.last().toObject();
        const double lat = point.value("lat").toDouble();
        const double lon = point.value("lon").toDouble();
        if (std::abs(lat) > 0.000001 && std::abs(lon) > 0.000001) next.push_back(QPointF(lon, lat));
      }
      carrot_navi_route = next;
    }
    carrot_navi_route_updated_at = route_updated_at;
  }

  // ---- speed 스트림: 기준 브랜치의 제한속도와 SDI 안전안내 ----
  carrot_navi_speed_limit = 0;
  carrot_navi_safety_description.clear();
  const QJsonObject speed = root.value("speed").toObject();
  if (!speed.isEmpty()) {
    // nMirror/Tmap 버전별 snake_case/camelCase 키를 모두 허용한다.
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
    static const char *description_keys[] = {
      "sdi_description", "sdiDescription", "sdi_descr", "sdiDescr",
      "safety_description", "safetyDescription",
      "camera_description", "cameraDescription", "description"
    };
    for (const char *k : description_keys) {
      if (speed.contains(k)) {
        const QString value = speed.value(k).toString().trimmed();
        if (!value.isEmpty()) { carrot_navi_safety_description = value; break; }
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

enum class NavigationKind { NONE, TURN, FORK, UTURN, ROTARY };

static bool navigationTypeIn(int type, std::initializer_list<int> types) {
  return std::find(types.begin(), types.end(), type) != types.end();
}

[[maybe_unused]] static NavigationKind navigationKind(int type, const QString &text, int *direction) {
  *direction = 0;
  if (navigationTypeIn(type, {12, 16})) {
    *direction = -1;
    return NavigationKind::TURN;
  }
  if (navigationTypeIn(type, {13, 19})) {
    *direction = 1;
    return NavigationKind::TURN;
  }
  if (navigationTypeIn(type, {7, 17, 44, 75, 76, 102, 105, 112, 115, 118})) {
    *direction = -1;
    return NavigationKind::FORK;
  }
  if (navigationTypeIn(type, {6, 43, 73, 74, 101, 104, 111, 114, 117, 123, 124})) {
    *direction = 1;
    return NavigationKind::FORK;
  }
  if (type == 14) {
    *direction = -1;
    return NavigationKind::UTURN;
  }
  if (type >= 131 && type <= 142) return NavigationKind::ROTARY;

  const QString lower = text.toLower();
  if (lower.contains(QString::fromUtf8("유턴")) || lower.contains("u-turn") || lower.contains("uturn")) {
    *direction = -1;
    return NavigationKind::UTURN;
  }
  const bool fork = lower.contains(QString::fromUtf8("분기")) ||
                    lower.contains(QString::fromUtf8("진출")) || lower.contains("fork");
  if (lower.contains(QString::fromUtf8("좌회전")) || lower.contains(QString::fromUtf8("왼쪽")) || lower.contains("left")) {
    *direction = -1;
    return fork ? NavigationKind::FORK : NavigationKind::TURN;
  }
  if (lower.contains(QString::fromUtf8("우회전")) || lower.contains(QString::fromUtf8("오른쪽")) || lower.contains("right")) {
    *direction = 1;
    return fork ? NavigationKind::FORK : NavigationKind::TURN;
  }
  return NavigationKind::NONE;
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
  const bool navi_fresh = carrot_navi_updated_at != 0 &&
                          wall_now - static_cast<qint64>(carrot_navi_updated_at) >= -5000 &&
                          wall_now - static_cast<qint64>(carrot_navi_updated_at) <= 35000;
  const bool route_active = carrot_navi_remain_distance > 0 &&
                            carrot_navi_remain_time > 0;

  // c3-wip DISPLAY state equivalent for EON. The target branch has no
  // carrotMan DISPLAY message, so preserve the same 0.2 s MAP entry and
  // 5 s ROAD return timing with the available NOO guidance state.
  static bool carrot_map_mode = false;
  static qint64 carrot_map_enter_since = 0;
  static qint64 carrot_map_exit_since = 0;
  if (!navi_fresh || !route_active) {
    carrot_map_mode = false;
    carrot_map_enter_since = 0;
    carrot_map_exit_since = 0;
    return;
  }

  p.save();
  p.setRenderHint(QPainter::Antialiasing);
  p.setRenderHint(QPainter::TextAntialiasing);

  // Keep the guidance box tight to the bottom-right edge and narrow enough
  // to leave the centered lane-width/debug strip unobstructed.
  const qreal ui_scale = std::min(width() / 1920.0, height() / 1080.0);
  const int right_margin = qRound(10 * ui_scale);
  const int bottom_margin = qRound(5 * ui_scale);
  const int panel_w = qRound(500 * ui_scale);
  const int compact_h = qRound(320 * ui_scale);
  const QRect compact_box(width() - right_margin - panel_w,
                          height() - bottom_margin - compact_h,
                          panel_w, compact_h);

  int navigation_direction = 0;
  const NavigationKind navigation_kind = navigationKind(carrot_navi_turn_type,
                                               carrot_navi_instruction,
                                               &navigation_direction);
  const qint64 guidance_age = wall_now -
      static_cast<qint64>(carrot_navi_guidance_updated_at);
  const bool navigation_fresh = carrot_navi_guidance_updated_at != 0 &&
                         guidance_age >= -5000 && guidance_age <= 3000;
  const bool navigation_available = noo_enabled &&
                             navigation_fresh && carrot_navi_distance >= 0 &&
                             navigation_kind != NavigationKind::NONE;

  // Match c3-wip's AutoTurnMapChange thresholds. Normal turns and forks use
  // a road-speed-dependent 160-350 m start distance; U-turn uses 1000 m.
  const qreal map_speed = std::clamp<qreal>(
      carrot_navi_speed_limit > 0 ? carrot_navi_speed_limit : 50, 30.0, 100.0);
  qreal navigation_map_start_distance = map_speed <= 50.0
      ? 160.0 + (map_speed - 30.0) * 2.0
      : 200.0 + (map_speed - 50.0) * 3.0;
  if (navigation_kind == NavigationKind::UTURN) navigation_map_start_distance = 1000.0;

  const bool navigation_map_request = navigation_available &&
                               carrot_navi_distance <= navigation_map_start_distance;
  const bool map_request = show_map_image &&
                           (show_route_map_always || navigation_map_request);
  if (!show_map_image) {
    carrot_map_mode = false;
    carrot_map_enter_since = 0;
    carrot_map_exit_since = 0;
  } else if (map_request) {
    carrot_map_exit_since = 0;
    if (carrot_map_enter_since == 0) carrot_map_enter_since = wall_now;
    if (wall_now - carrot_map_enter_since >= 200) carrot_map_mode = true;
  } else {
    carrot_map_enter_since = 0;
    if (carrot_map_mode) {
      if (carrot_map_exit_since == 0) carrot_map_exit_since = wall_now;
      if (wall_now - carrot_map_exit_since >= 5000) {
        carrot_map_mode = false;
        carrot_map_exit_since = 0;
      }
    } else {
      carrot_map_exit_since = 0;
    }
  }

  // Do not draw a separate navigation/NOO lane arrow in the road center.
  // Guidance remains contained in the bottom-right information box.

  // The compact c3-wip information box remains visible for the whole route.
  // MAP mode only adds the equally wide map panel above this box.
  p.setPen(QPen(QColor(255, 255, 255, 220), std::max(1, qRound(2 * ui_scale))));
  p.setBrush(QColor(0, 0, 0, 150));
  p.drawRoundedRect(compact_box, qRound(30 * ui_scale), qRound(30 * ui_scale));

  QString title = carrot_navi_instruction;
  if (title.isEmpty()) title = carrot_navi_road;
  if (title.isEmpty()) title = QString::fromUtf8("경로 안내");
  configFont(p, "Open Sans", std::max(18, qRound(30 * ui_scale)), "Bold");
  p.setPen(Qt::white);
  p.drawText(QRect(compact_box.x() + qRound(14 * ui_scale),
                   compact_box.y() + qRound(4 * ui_scale),
                   compact_box.width() - qRound(28 * ui_scale),
                   qRound(52 * ui_scale)),
             Qt::AlignLeft | Qt::AlignVCenter,
             QFontMetrics(p.font()).elidedText(
                 title, Qt::ElideRight, compact_box.width() - qRound(28 * ui_scale)));

  // ---- NOO 상태 배지 (제목줄 오른쪽) ----
  //      회전조향 중 : 파랑  TURN ◀ / TURN ▶
  //      차선변경 중 : 초록  2▶3
  //      변경 계획됨 : 청록  2>3
  //      계획 없음   : 회색  c4/m3  (카메라/티맵 차로수 불일치를 그대로 표시)
  {
    const auto lateral_plan = (*(uiState()->sm))["lateralPlan"].getLateralPlan();
    const int noo_turn = lateral_plan.getNooTurnDirection();
    const int noo_cur = lateral_plan.getNooCurrentLane();
    const int noo_tgt = lateral_plan.getNooTargetLane();
    const int noo_lc = lateral_plan.getNooLaneChangeDirection();
    const int noo_cam = lateral_plan.getNooCameraLaneCount();
    const int noo_map = lateral_plan.getNooRouteLaneCount();

    QString noo_str;
    QColor noo_color;
    if (noo_turn != 0) {
      noo_str = noo_turn < 0 ? QString::fromUtf8("TURN ◀") : QString::fromUtf8("TURN ▶");
      noo_color = QColor(0, 90, 220, 220);
    } else if (noo_lc != 0 && noo_cur > 0 && noo_tgt > 0) {
      noo_str = QString("%1%2%3").arg(noo_cur)
                    .arg(noo_lc < 0 ? QString::fromUtf8("◀") : QString::fromUtf8("▶")).arg(noo_tgt);
      noo_color = QColor(0, 170, 60, 220);
    } else if (noo_cur > 0 && noo_tgt > 0) {
      noo_str = QString("%1>%2").arg(noo_cur).arg(noo_tgt);
      noo_color = QColor(0, 150, 160, 220);
    } else if (noo_cam > 0 || noo_map > 0) {
      noo_str = QString("c%1/m%2").arg(noo_cam).arg(noo_map);
      noo_color = QColor(96, 102, 110, 200);
    }

    if (!noo_str.isEmpty()) {
      configFont(p, "Open Sans", std::max(14, qRound(24 * ui_scale)), "Bold");
      const int pad = qRound(12 * ui_scale);
      const int badge_w = QFontMetrics(p.font()).horizontalAdvance(noo_str) + pad * 2;
      const int badge_h = qRound(38 * ui_scale);
      const QRect badge(compact_box.right() - qRound(14 * ui_scale) - badge_w,
                        compact_box.y() + qRound(10 * ui_scale), badge_w, badge_h);
      p.setPen(Qt::NoPen);
      p.setBrush(noo_color);
      p.drawRoundedRect(badge, qRound(10 * ui_scale), qRound(10 * ui_scale));
      p.setPen(Qt::white);
      p.drawText(badge, Qt::AlignCenter, noo_str);
    }
  }

  const int turn_center_x = compact_box.x() + qRound(70 * ui_scale);
  const int turn_center_y = compact_box.y() + qRound(166 * ui_scale);
  const QRect turn_card(turn_center_x - qRound(56 * ui_scale),
                        turn_center_y - qRound(102 * ui_scale),
                        qRound(112 * ui_scale), qRound(245 * ui_scale));
  const bool navigation_prepare = navigation_available && carrot_navi_distance > 350;
  const bool navigation_apply = navigation_available && !navigation_prepare;
  const bool navigation_blink_on = !navigation_apply || blink_timer <= 8;
  const QColor turn_fill(55, 55, 55, 170);
  p.setPen(QPen(QColor(255, 255, 255, 230), std::max(2, qRound(2 * ui_scale))));
  p.setBrush(turn_fill);
  p.drawRoundedRect(turn_card, qRound(12 * ui_scale), qRound(12 * ui_scale));

  const int turn_icon_w = qRound(106 * ui_scale);
  const int turn_icon_h = qRound(158 * ui_scale);
  const QRect turn_icon(turn_center_x - turn_icon_w / 2,
                        turn_center_y - qRound(84 * ui_scale),
                        turn_icon_w, turn_icon_h);
  // Match c3-wip: use its original 256x256 PNG artwork for turns,
  // forks/lane changes, and U-turns. Keep vector rendering only as a fallback
  // if an asset cannot be loaded on the device.
  static const QPixmap turn_left_pixmap("../assets/images/turn_l.png");
  static const QPixmap turn_right_pixmap("../assets/images/turn_r.png");
  static const QPixmap turn_uturn_pixmap("../assets/images/turn_u.png");
  static const QPixmap fork_left_pixmap("../assets/images/lane_change_l.png");
  static const QPixmap fork_right_pixmap("../assets/images/lane_change_r.png");
  const QPixmap *turn_pixmap = nullptr;
  if (navigation_kind == NavigationKind::TURN) {
    turn_pixmap = navigation_direction < 0 ? &turn_left_pixmap : &turn_right_pixmap;
  } else if (navigation_kind == NavigationKind::FORK) {
    turn_pixmap = navigation_direction < 0 ? &fork_left_pixmap : &fork_right_pixmap;
  } else if (navigation_kind == NavigationKind::UTURN) {
    turn_pixmap = &turn_uturn_pixmap;
  }

  if (turn_pixmap != nullptr && !turn_pixmap->isNull()) {
    // Keep the original asset colors in inactive and prepare states. During
    // actual NOO application, blink the unmodified image without per-frame
    // QPixmap copies or tint painting.
    const QRect turn_source(48, 48, 160, 160);
    if (navigation_blink_on) p.drawPixmap(turn_icon, *turn_pixmap, turn_source);
  } else if (navigation_kind == NavigationKind::TURN ||
             navigation_kind == NavigationKind::FORK ||
             navigation_kind == NavigationKind::UTURN) {
    if (navigation_blink_on) {
      drawCarrotTurnArrow(p, turn_icon, carrot_navi_turn_type,
                          carrot_navi_instruction, Qt::white,
                          std::max(6, qRound(13 * ui_scale)));
    }
  } else {
    configFont(p, "Open Sans", std::max(18, qRound(32 * ui_scale)), "Bold");
    p.setPen(Qt::white);
    QString fallback_text = QString::fromUtf8("안내");
    if (navigation_kind == NavigationKind::ROTARY) {
      // c3-wip maps rotary guidance to xTurnInfo 5; its default HUD case shows "감속:5".
      fallback_text = QString::fromUtf8("감속:5");
    } else if (carrot_navi_turn_type > 0) {
      // Match c3-wip's default switch case for positive, icon-less turn information.
      fallback_text = QString::fromUtf8("감속:%1").arg(carrot_navi_turn_type);
    }
    p.drawText(turn_icon, Qt::AlignCenter, fallback_text);
  }

  configFont(p, "Open Sans", std::max(18, qRound(30 * ui_scale)), "Bold");
  p.setPen(Qt::white);
  const QString turn_distance = carrotDistanceText(carrot_navi_distance);
  p.drawText(QRect(turn_card.x(), turn_card.bottom() - qRound(52 * ui_scale),
                   turn_card.width(), qRound(46 * ui_scale)),
             Qt::AlignCenter, turn_distance.isEmpty() ? QString("--") : turn_distance);

  const int info_x = compact_box.x() + qRound(136 * ui_scale);
  const int info_w = compact_box.right() - info_x - qRound(14 * ui_scale);
  const QDateTime eta = QDateTime::currentDateTime().addSecs(carrot_navi_remain_time);
  const int eta_hour = eta.time().hour();
  const QString ampm_text = eta_hour < 12 ? QString::fromUtf8("오전") : QString::fromUtf8("오후");
  const QString time_text = QString("%1:%2")
      .arg(eta_hour % 12 == 0 ? 12 : eta_hour % 12)
      .arg(eta.time().minute(), 2, 10, QChar('0'));
  const bool distance_in_km = carrot_navi_remain_distance >= 1000;
  const QString distance_text = carrot_navi_remain_distance < 0
      ? QString("--")
      : (distance_in_km
          ? QString::number(carrot_navi_remain_distance / 1000.0, 'f', 1)
          : QString::number(carrot_navi_remain_distance));
  const QString distance_unit = carrot_navi_remain_distance < 0
      ? QString()
      : (distance_in_km ? QString("km") : QString("m"));

  // Draw the ETA and remaining distance on one baseline with c3-style
  // emphasis: time/distance are full size, while their labels and unit are smaller.
  const int full_size = std::max(26, qRound(42 * ui_scale));
  const int half_size = std::max(24, qRound(25 * ui_scale));
  const int quarter_size = std::max(18, qRound(18 * ui_scale));
  const int eta_baseline = compact_box.y() + qRound(126 * ui_scale);
  int eta_x = info_x;
  const auto draw_eta_part = [&](const QString &text, int font_size, int gap) {
    configFont(p, "Open Sans", font_size, "Bold");
    p.setPen(Qt::white);
    p.drawText(eta_x, eta_baseline, text);
    eta_x += QFontMetrics(p.font()).horizontalAdvance(text) + gap;
  };
  draw_eta_part(ampm_text, half_size, qRound(5 * ui_scale));
  draw_eta_part(time_text, full_size, qRound(4 * ui_scale));
  draw_eta_part(QString::fromUtf8("예정"), quarter_size, qRound(15 * ui_scale));
  draw_eta_part(distance_text, full_size, qRound(3 * ui_scale));
  draw_eta_part(distance_unit, half_size, 0);

  QString road_text = carrot_navi_safety_description;
  const bool has_safety_description = !road_text.isEmpty();
  if (road_text.isEmpty()) road_text = carrot_navi_road;
  if (road_text.isEmpty()) road_text = carrot_navi_next_instruction;
  configFont(p, "Open Sans", std::max(24, qRound(38 * ui_scale)), "Bold");
  const QRect road_box(info_x, compact_box.y() + qRound(230 * ui_scale),
                       info_w, qRound(55 * ui_scale));
  const QString visible_road_text =
      QFontMetrics(p.font()).elidedText(road_text, Qt::ElideRight, info_w - qRound(13 * ui_scale));
  if (has_safety_description && !visible_road_text.isEmpty()) {
    const int pill_w = std::min(
        info_w, QFontMetrics(p.font()).horizontalAdvance(visible_road_text) +
                    qRound(16 * ui_scale));
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 120, 55, 230));
    p.drawRoundedRect(QRect(road_box.x(), road_box.y() + qRound(4 * ui_scale),
                            pill_w, road_box.height() - qRound(8 * ui_scale)),
                      qRound(10 * ui_scale), qRound(10 * ui_scale));
  }
  p.setPen(Qt::white);
  p.drawText(road_box.adjusted(qRound(6 * ui_scale), 0, 0, 0),
             Qt::AlignLeft | Qt::AlignVCenter, visible_road_text);

  // Poll and decode the EON Tmap JPEG at 5 Hz only while MAP mode is visible.
  // The phone stream and /dev/shm file updates continue while hidden, but the
  // EON skips QFileInfo/QPixmap JPEG work. Resetting the poll timestamp while
  // hidden forces an immediate decode of the newest frame on MAP entry.
  // No header, footer, or lane-image layer is drawn.
  static const QString map_path = "/dev/shm/carrot_navi_map.jpg";
  static QPixmap map_pixmap;
  static uint64_t map_checked_at = 0;
  static qint64 map_modified_at = 0;
  const uint64_t map_now = millis_since_boot();
  if (!carrot_map_mode) map_checked_at = 0;
  if (carrot_map_mode &&
      (map_checked_at == 0 || map_now - map_checked_at >= 200)) {
    map_checked_at = map_now;
    const QFileInfo map_info(map_path);
    if (!map_info.exists() || !map_info.isFile() || map_info.size() <= 0) {
      map_pixmap = QPixmap();
      map_modified_at = 0;
    } else {
      const qint64 modified_at = map_info.lastModified().toMSecsSinceEpoch();
      if (map_pixmap.isNull() || modified_at != map_modified_at) {
        QPixmap candidate;
        if (candidate.load(map_path, "JPEG")) {
          map_pixmap = candidate;
          map_modified_at = modified_at;
        } else {
          map_pixmap = QPixmap();
          map_modified_at = 0;
        }
      }
    }
  }

  const qint64 map_age = wall_now - map_modified_at;
  const bool map_fresh = !map_pixmap.isNull() && map_age >= 0 && map_age <= 2500;
  if (carrot_map_mode && map_fresh) {
    // The top status line occupies y=12..60. Start the map below it with a
    // visible gap so it never covers the live torque/steer-ratio values.
    const int map_top = qRound(80 * ui_scale);
    const int map_gap = qRound(10 * ui_scale);
    const int map_h = compact_box.top() - map_gap - map_top;
    const QRect map_panel(compact_box.x(), map_top, compact_box.width(),
                          std::max(1, map_h));

    // Rebuild the crop only for a new Tmap frame or a geometry change. This
    // keeps map-only mode inexpensive on EON/C3 while paintEvent runs at UI Hz.
    static QImage map_scaled;
    static qint64 map_scaled_modified_at = -1;
    static QSize map_scaled_size;
    if (map_scaled_modified_at != map_modified_at ||
        map_scaled_size != map_panel.size()) {
      const qreal status_bar_h = std::min<qreal>(
          map_pixmap.height() / 8.0, map_pixmap.width() * 0.05);
      const QRectF content(0.0, status_bar_h, map_pixmap.width(),
                           map_pixmap.height() - status_bar_h);
      const qreal target_aspect =
          static_cast<qreal>(map_panel.width()) / map_panel.height();
      qreal source_w = content.width();
      qreal source_h = content.height();
      if (source_w / source_h > target_aspect) {
        source_w = source_h * target_aspect;
      } else {
        source_h = source_w / target_aspect;
      }
      const QRectF source_rect(content.center().x() - source_w / 2.0,
                               content.center().y() - source_h / 2.0,
                               source_w, source_h);
      const QPixmap scaled = map_pixmap.copy(source_rect.toRect()).scaled(
          map_panel.size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);

      QImage rounded(map_panel.size(), QImage::Format_ARGB32_Premultiplied);
      rounded.fill(Qt::transparent);
      QPainter map_painter(&rounded);
      map_painter.setRenderHint(QPainter::Antialiasing);
      QPainterPath clip;
      clip.addRoundedRect(QRectF(QPointF(0, 0), map_panel.size()),
                          qRound(30 * ui_scale), qRound(30 * ui_scale));
      map_painter.setClipPath(clip);
      map_painter.drawPixmap(0, 0, scaled);
      map_painter.end();

      map_scaled = rounded;
      map_scaled_modified_at = map_modified_at;
      map_scaled_size = map_panel.size();
    }

    p.setPen(QPen(QColor(255, 255, 255, 220),
                  std::max(1, qRound(2 * ui_scale))));
    p.setBrush(QColor(0, 0, 0, 150));
    p.drawRoundedRect(map_panel, qRound(30 * ui_scale), qRound(30 * ui_scale));
    p.drawImage(map_panel.topLeft(), map_scaled);
  }

  p.restore();
}

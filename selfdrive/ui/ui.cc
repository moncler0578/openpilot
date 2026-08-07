#include "selfdrive/ui/ui.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>

#include <QtConcurrent>

#include "common/transformations/orientation.hpp"
#include "selfdrive/common/params.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/util.h"
#include "selfdrive/common/watchdog.h"
#include "selfdrive/hardware/hw.h"

#define BACKLIGHT_DT 0.05
#define BACKLIGHT_TS 10.00
#define BACKLIGHT_OFFROAD 50

static bool calib_frame_to_full_frame(const UIState *s, float in_x, float in_y, float in_z, QPointF *out) {
  const float margin = 1000.0f;
  const QRectF clip_region{-margin, -margin, s->fb_w + 2 * margin, s->fb_h + 2 * margin};

  const vec3 pt = (vec3){{in_x, in_y, in_z}};
  const vec3 Ep = matvecmul3(s->scene.view_from_calib, pt);
  const vec3 KEp = matvecmul3(s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix, Ep);

  QPointF point = s->car_space_transform.map(QPointF{KEp.v[0] / KEp.v[2], KEp.v[1] / KEp.v[2]});
  if (clip_region.contains(point)) {
    *out = point;
    return true;
  }
  return false;
}

static int get_path_length_idx(const cereal::ModelDataV2::XYZTData::Reader &line, const float path_height) {
  const auto line_x = line.getX();
  int max_idx = 0;
  for (int i = 1; i < TRAJECTORY_SIZE && line_x[i] <= path_height; ++i) {
    max_idx = i;
  }
  return max_idx;
}

static void update_leads(UIState *s, const cereal::RadarState::Reader &radar_state, const cereal::ModelDataV2::XYZTData::Reader &line) {
  for (int i = 0; i < 2; ++i) {
    auto lead_data = (i == 0) ? radar_state.getLeadOne() : radar_state.getLeadTwo();
    if (lead_data.getStatus()) {
      float z = line.getZ()[get_path_length_idx(line, lead_data.getDRel())];
      calib_frame_to_full_frame(s, lead_data.getDRel(), -lead_data.getYRel(), z + 1.22, &s->scene.lead_vertices[i]);
      s->scene.lead_radar[i] = lead_data.getRadar();

      // carrot 스타일: 리드 뒷면 좌/우 끝 (폭 2.4m 가정)
      s->scene.lead_status[i] = true;
      calib_frame_to_full_frame(s, lead_data.getDRel(), -lead_data.getYRel() - 1.2, z + 1.22, &s->scene.lead_left[i]);
      calib_frame_to_full_frame(s, lead_data.getDRel(), -lead_data.getYRel() + 1.2, z + 1.22, &s->scene.lead_right[i]);
      if (i == 0) s->scene.lead_radar_dist = lead_data.getRadar() ? lead_data.getDRel() : 0.0f;
    }
    else {
      s->scene.lead_radar[i] = false;
      s->scene.lead_status[i] = false;
      if (i == 0) s->scene.lead_radar_dist = 0.0f;
    }
  }
}

// t_follow 목표선 : 경로 위 desiredDistance 지점을 가로지르는 선
static void update_tfollow_line(UIState *s, const cereal::ModelDataV2::XYZTData::Reader &line) {
  const auto plan = (*s->sm)["longitudinalPlan"].getLongitudinalPlan();
  float d = plan.getDesiredDistance();
  s->scene.tf_distance = d;
  s->scene.t_follow = plan.getTFollow();
  s->scene.tf_valid = false;
  if (d <= 1.0f) return;

  int idx = get_path_length_idx(line, d);
  float y = line.getY()[idx];
  float z = line.getZ()[idx];
  if (calib_frame_to_full_frame(s, d, y - 1.0, z + 1.22, &s->scene.tf_left) &&
      calib_frame_to_full_frame(s, d, y + 1.0, z + 1.22, &s->scene.tf_right)) {
    s->scene.tf_valid = true;
  }
}

// apilot(c3-master) update_line_data 방식으로 교체.
//  - 위/아래(z_off_left/right) 두 점이 "둘 다" 투영 성공했을 때만 쌍으로 넣는다.
//    기존 코드는 앞/뒤 패스를 따로 돌아서, 클리핑이 비대칭으로 일어나면
//    v[i] <-> v[n-1-i] 짝이 어긋나 drawBlindSpot 울타리 사각형이 전부
//    찌그러진 슬리버(면적 0)가 되어 화면에 안 보였다.
//  - allow_invert=false 면 화면 y 가 역전되는(지평선 근처 뒤집힘) 점은 버린다.
//    기존 코드는 이 파라미터를 받기만 하고 실제로는 무시했다.
static void update_line_data(const UIState *s, const cereal::ModelDataV2::XYZTData::Reader &line,
                             float y_off, float z_off_left, float z_off_right, line_vertices_data *pvd, int max_idx, bool allow_invert=true,
                             float y_shift=0.0) {
  const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
  QPointF left_points[TRAJECTORY_SIZE], right_points[TRAJECTORY_SIZE];
  int cnt = 0;
  for (int i = 0; i <= max_idx; i++) {
    if (line_x[i] < 0) continue;
    QPointF left, right;
    bool l = calib_frame_to_full_frame(s, line_x[i], line_y[i] + y_shift - y_off, line_z[i] + z_off_left, &left);
    bool r = calib_frame_to_full_frame(s, line_x[i], line_y[i] + y_shift + y_off, line_z[i] + z_off_right, &right);
    if (l && r) {
      if (!allow_invert && cnt > 0 && left.y() > left_points[cnt - 1].y()) continue;
      left_points[cnt] = left;
      right_points[cnt] = right;
      cnt++;
    }
  }
  QPointF *v = &pvd->v[0];
  for (int i = 0; i < cnt; i++) *v++ = left_points[i];
  for (int i = cnt - 1; i >= 0; i--) *v++ = right_points[i];
  pvd->cnt = v - pvd->v;
  assert(pvd->cnt <= std::size(pvd->v));
}

static void update_model(UIState *s, const cereal::ModelDataV2::Reader &model) {
  UIScene &scene = s->scene;
  auto model_position = model.getPosition();
  float max_distance = std::clamp(model_position.getX()[TRAJECTORY_SIZE - 1],
                                  MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

  // update lane lines
  const auto lane_lines = model.getLaneLines();
  const auto lane_line_probs = model.getLaneLineProbs();
  int max_idx = get_path_length_idx(lane_lines[0], max_distance);
  for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
    scene.lane_line_probs[i] = lane_line_probs[i];
    update_line_data(s, lane_lines[i], 0.025 * scene.lane_line_probs[i], 0, 0, &scene.lane_line_vertices[i], max_idx);
  }

  // lane barriers for blind spot (carrot 방식)
  //   차선이 아니라 주행 경로를 좌우 1.7m 평행이동시켜 벽을 세운다.
  //   차선 인식이 끊겨도 항상 그려진다.
  int max_distance_barrier = 40;
  int max_idx_barrier = get_path_length_idx(model_position, max_distance_barrier);
  //   z 는 경로(model_position) 기준이라 노면까지 1.22 를 더해야 한다.
  //   (차선 기준이던 이전 코드의 -0.05 / -0.6 을 그대로 쓰면 1.22m 아래에 그려진다)
  update_line_data(s, model_position, 0, 1.22 - 0.05, 1.22 - 0.6, &scene.lane_barrier_vertices[0], max_idx_barrier, false, -1.7);
  update_line_data(s, model_position, 0, 1.22 - 0.05, 1.22 - 0.6, &scene.lane_barrier_vertices[1], max_idx_barrier, false, 1.7);

  // update road edges
  const auto road_edges = model.getRoadEdges();
  const auto road_edge_stds = model.getRoadEdgeStds();
  for (int i = 0; i < std::size(scene.road_edge_vertices); i++) {
    scene.road_edge_stds[i] = road_edge_stds[i];
    update_line_data(s, road_edges[i], 0.025, 0, 0, &scene.road_edge_vertices[i], max_idx);
  }

  // update path
  // ShowPathWidth is stored as centimeters (90 = 0.90 m half-width).
  // Refresh at 1 Hz so the UI setting applies while driving without reading
  // persistent storage on every model frame.
  static Params path_params;
  static int path_param_frame = 0;
  static float path_width = 0.9f;
  if (++path_param_frame >= UI_FREQ) {
    path_param_frame = 0;
    const std::string configured_width_value = path_params.get("ShowPathWidth");
    const int configured_width = configured_width_value.empty()
        ? 90 : std::atoi(configured_width_value.c_str());
    path_width = std::clamp(configured_width > 0 ? configured_width / 100.0f : 0.9f,
                            0.3f, 1.5f);
  }
  auto lead_one = (*s->sm)["radarState"].getRadarState().getLeadOne();
  if (lead_one.getStatus()) {
    const float lead_d = lead_one.getDRel() * 2.;
    max_distance = std::clamp((float)(lead_d - fmin(lead_d * 0.35, 10.)), 0.0f, max_distance);
  }
  max_idx = get_path_length_idx(model_position, max_distance);
  update_line_data(s, model_position, path_width, 1.22, 1.22, &scene.track_vertices, max_idx, false);
}

static void update_sockets(UIState *s) {
  s->sm->update(0);
}

static void update_state(UIState *s) {
  SubMaster &sm = *(s->sm);
  UIScene &scene = s->scene;

  if (sm.updated("liveCalibration")) {
    auto rpy_list = sm["liveCalibration"].getLiveCalibration().getRpyCalib();
    Eigen::Vector3d rpy;
    rpy << rpy_list[0], rpy_list[1], rpy_list[2];
    Eigen::Matrix3d device_from_calib = euler2rot(rpy);
    Eigen::Matrix3d view_from_device;
    view_from_device << 0,1,0,
                        0,0,1,
                        1,0,0;
    Eigen::Matrix3d view_from_calib = view_from_device * device_from_calib;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        scene.view_from_calib.v[i*3 + j] = view_from_calib(i,j);
      }
    }
  }
  if (s->worldObjectsVisible()) {
    if (sm.updated("modelV2")) {
      update_model(s, sm["modelV2"].getModelV2());
    }
    if (sm.updated("radarState") && sm.rcv_frame("modelV2") > s->scene.started_frame) {
      update_leads(s, sm["radarState"].getRadarState(), sm["modelV2"].getModelV2().getPosition());
    }
    if (sm.updated("modelV2")) {
      update_tfollow_line(s, sm["modelV2"].getModelV2().getPosition());
      auto leads_v3 = sm["modelV2"].getModelV2().getLeadsV3();
      s->scene.lead_vision_dist = (leads_v3.size() > 0 && leads_v3[0].getProb() > 0.5)
                                ? leads_v3[0].getX()[0] : 0.0f;
    }
  }
  if (sm.updated("pandaStates")) {
    auto pandaStates = sm["pandaStates"].getPandaStates();
    if (pandaStates.size() > 0) {
      scene.pandaType = pandaStates[0].getPandaType();

      if (scene.pandaType != cereal::PandaState::PandaType::UNKNOWN) {
        scene.ignition = false;
        for (const auto& pandaState : pandaStates) {
          scene.ignition |= pandaState.getIgnitionLine() || pandaState.getIgnitionCan();
        }
      }
    }
  } else if ((s->sm->frame - s->sm->rcv_frame("pandaStates")) > 5*UI_FREQ) {
    scene.pandaType = cereal::PandaState::PandaType::UNKNOWN;
  }
  if (sm.updated("carParams")) {
    scene.longitudinal_control = sm["carParams"].getCarParams().getOpenpilotLongitudinalControl();
  }
  if (!scene.started && sm.updated("sensorEvents")) {
    for (auto sensor : sm["sensorEvents"].getSensorEvents()) {
      if (sensor.which() == cereal::SensorEventData::ACCELERATION) {
        auto accel = sensor.getAcceleration().getV();
        if (accel.totalSize().wordCount) {
          scene.accel_sensor = accel[2];
        }
      } else if (sensor.which() == cereal::SensorEventData::GYRO_UNCALIBRATED) {
        auto gyro = sensor.getGyroUncalibrated().getV();
        if (gyro.totalSize().wordCount) {
          scene.gyro_sensor = gyro[1];
        }
      }
    }
  }
  if (!Hardware::TICI() && sm.updated("roadCameraState")) {
    auto camera_state = sm["roadCameraState"].getRoadCameraState();

    float max_lines = Hardware::EON() ? 5408 : 1904;
    float max_gain = Hardware::EON() ? 1.0: 10.0;
    float max_ev = max_lines * max_gain;

    float ev = camera_state.getGain() * float(camera_state.getIntegLines());

    scene.light_sensor = std::clamp<float>(1.0 - (ev / max_ev), 0.0, 1.0);
  } else if (Hardware::TICI() && sm.updated("wideRoadCameraState")) {
    auto camera_state = sm["wideRoadCameraState"].getWideRoadCameraState();

    float max_lines = 1618;
    float max_gain = 10.0;
    float max_ev = max_lines * max_gain / 6;

    float ev = camera_state.getGain() * float(camera_state.getIntegLines());

    scene.light_sensor = std::clamp<float>(1.0 - (ev / max_ev), 0.0, 1.0);
  }
  scene.started = sm["deviceState"].getDeviceState().getStarted() && scene.ignition;

  if (sm.updated("lateralPlan")) {
    scene.dynamic_lane_profile = sm["lateralPlan"].getLateralPlan().getDynamicLaneProfile();
    scene.dynamic_lane_profile_status = sm["lateralPlan"].getLateralPlan().getDynamicLaneProfileStatus();
  }
}

void ui_update_params(UIState *s) {
  auto params = Params();
  s->scene.is_metric = params.getBool("IsMetric");
  s->scene.map_on_left = params.getBool("NavSettingLeftSide");
  s->scene.experimental_mode = params.getBool("ExperimentalMode");
  s->show_debug = params.getBool("ShowDebugUI");
  s->lat_control = std::string(Params().get("LateralControl"));
}

void UIState::updateStatus() {
  if (scene.started && sm->updated("controlsState")) {
    auto controls_state = (*sm)["controlsState"].getControlsState();
    auto alert_status = controls_state.getAlertStatus();
    auto state = controls_state.getState();
    if (alert_status == cereal::ControlsState::AlertStatus::USER_PROMPT) {
      status = STATUS_WARNING;
    } else if (alert_status == cereal::ControlsState::AlertStatus::CRITICAL) {
      status = STATUS_ALERT;
    } else if (state == cereal::ControlsState::OpenpilotState::PRE_ENABLED || state == cereal::ControlsState::OpenpilotState::OVERRIDING) {
      status = STATUS_OVERRIDE;
    } else {
      status = controls_state.getEnabled() ? STATUS_ENGAGED : STATUS_DISENGAGED;
    }
  }

  // Handle onroad/offroad transition
  if (scene.started != started_prev || sm->frame == 1) {
    if (scene.started) {
      status = STATUS_DISENGAGED;
      scene.started_frame = sm->frame;
      wide_camera = Hardware::TICI() ? Params().getBool("EnableWideCamera") : false;
    }
    started_prev = scene.started;
    emit offroadTransition(!scene.started);
  }
}

UIState::UIState(QObject *parent) : QObject(parent) {
  sm = std::make_unique<SubMaster, const std::initializer_list<const char *>>({
    "modelV2", "controlsState", "liveCalibration", "radarState", "deviceState", "roadCameraState",
    "pandaStates", "carParams", "driverMonitoringState", "sensorEvents", "carState", "liveLocationKalman",
    "wideRoadCameraState",
    "gpsLocationExternal", "carControl", "liveParameters", "roadLimitSpeed", "lateralPlan",
    "longitudinalPlan",
  });

  Params params;
  wide_camera = Hardware::TICI() ? params.getBool("EnableWideCamera") : false;
  prime_type = std::atoi(params.get("PrimeType").c_str());

  // update timer
  timer = new QTimer(this);
  QObject::connect(timer, &QTimer::timeout, this, &UIState::update);
  timer->start(1000 / UI_FREQ);
}

void UIState::update() {
  update_sockets(this);
  update_state(this);
  updateStatus();

  if (sm->frame % UI_FREQ == 0) {
    watchdog_kick();
  }
  emit uiUpdate(*this);
}

Device::Device(QObject *parent) : brightness_filter(BACKLIGHT_OFFROAD, BACKLIGHT_TS, BACKLIGHT_DT), QObject(parent) {
  setAwake(true);
  resetInteractiveTimout();

  QObject::connect(uiState(), &UIState::uiUpdate, this, &Device::update);
}

void Device::update(const UIState &s) {
  updateBrightness(s);
  updateWakefulness(s);

  // TODO: remove from UIState and use signals
  uiState()->awake = awake;
}

void Device::setAwake(bool on) {
  if (on != awake) {
    awake = on;
    Hardware::set_display_power(awake);
    LOGD("setting display power %d", awake);
    emit displayPowerChanged(awake);
  }
}

void Device::resetInteractiveTimout() {
  interactive_timeout = (ignition_on ? 10 : 30) * UI_FREQ;
}

void Device::updateBrightness(const UIState &s) {
  float clipped_brightness = BACKLIGHT_OFFROAD;
  if (s.scene.started) {
    clipped_brightness = 100.0 * s.scene.light_sensor;

    if (clipped_brightness <= 8) {
      clipped_brightness = (clipped_brightness / 903.3);
    } else {
      clipped_brightness = std::pow((clipped_brightness + 16.0) / 116.0, 3.0);
    }

    clipped_brightness = std::clamp(100.0f * clipped_brightness, 10.0f, 100.0f);
  }

  int brightness = brightness_filter.update(clipped_brightness);
  if (!awake) {
    brightness = 0;
  }

  if (brightness != last_brightness) {
    if (!brightness_future.isRunning()) {
      brightness_future = QtConcurrent::run(Hardware::set_brightness, brightness);
      last_brightness = brightness;
    }
  }
}

bool Device::motionTriggered(const UIState &s) {
  static float accel_prev = 0;
  static float gyro_prev = 0;

  bool accel_trigger = abs(s.scene.accel_sensor - accel_prev) > 0.2;
  bool gyro_trigger = abs(s.scene.gyro_sensor - gyro_prev) > 0.15;

  gyro_prev = s.scene.gyro_sensor;
  accel_prev = (accel_prev * (accel_samples - 1) + s.scene.accel_sensor) / accel_samples;

  return (!awake && accel_trigger && gyro_trigger);
}

void Device::updateWakefulness(const UIState &s) {
  bool ignition_just_turned_off = !s.scene.ignition && ignition_on;
  ignition_on = s.scene.ignition;

  if (ignition_just_turned_off || motionTriggered(s)) {
    resetInteractiveTimout();
  } else if (interactive_timeout > 0 && --interactive_timeout == 0) {
    emit interactiveTimout();
  }

  setAwake(s.scene.ignition || interactive_timeout > 0);
}

UIState *uiState() {
  static UIState ui_state;
  return &ui_state;
}

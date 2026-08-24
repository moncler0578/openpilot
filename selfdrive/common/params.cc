#include "selfdrive/common/params.h"

#include <dirent.h>
#include <sys/file.h>

#include <algorithm>
#include <csignal>
#include <unordered_map>

#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"

namespace {

volatile sig_atomic_t params_do_exit = 0;
void params_sig_handler(int signal) {
  params_do_exit = 1;
}

int fsync_dir(const std::string &path) {
  int result = -1;
  int fd = HANDLE_EINTR(open(path.c_str(), O_RDONLY, 0755));
  if (fd >= 0) {
    result = fsync(fd);
    close(fd);
  }
  return result;
}

bool create_params_path(const std::string &param_path, const std::string &key_path) {
  // Make sure params path exists
  if (!util::file_exists(param_path) && !util::create_directories(param_path, 0775)) {
    return false;
  }

  // See if the symlink exists, otherwise create it
  if (!util::file_exists(key_path)) {
    // 1) Create temp folder
    // 2) Symlink it to temp link
    // 3) Move symlink to <params>/d

    std::string tmp_path = param_path + "/.tmp_XXXXXX";
    // this should be OK since mkdtemp just replaces characters in place
    char *tmp_dir = mkdtemp((char *)tmp_path.c_str());
    if (tmp_dir == NULL) {
      return false;
    }

    std::string link_path = std::string(tmp_dir) + ".link";
    if (symlink(tmp_dir, link_path.c_str()) != 0) {
      return false;
    }

    // don't return false if it has been created by other
    if (rename(link_path.c_str(), key_path.c_str()) != 0 && errno != EEXIST) {
      return false;
    }
  }

  return true;
}

std::string ensure_params_path(const std::string &path = {}) {
  std::string params_path = path.empty() ? Path::params() : path;
  if (!create_params_path(params_path, params_path + "/d")) {
    throw std::runtime_error(util::string_format("Failed to ensure params path, errno=%d", errno));
  }
  return params_path;
}

class FileLock {
public:
  FileLock(const std::string &fn) {
    fd_ = HANDLE_EINTR(open(fn.c_str(), O_CREAT, 0775));
    if (fd_ < 0 || HANDLE_EINTR(flock(fd_, LOCK_EX)) < 0) {
      LOGE("Failed to lock file %s, errno=%d", fn.c_str(), errno);
    }
  }
  ~FileLock() { close(fd_); }

private:
  int fd_ = -1;
};

std::unordered_map<std::string, uint32_t> keys = {
    {"AccessToken", CLEAR_ON_MANAGER_START | DONT_LOG},
    {"AthenadPid", PERSISTENT},
    {"AthenadUploadQueue", PERSISTENT},
    {"AutoGasTokSpeed", PERSISTENT},
    {"AutoGasCancelSpeed", PERSISTENT},
    {"SpeedFromPCM", PERSISTENT},
    {"AutoSpeedUptoRoadSpeedLimit", PERSISTENT},
    {"AutoRoadSpeedAdjust", PERSISTENT},
    {"AutoRoadSpeedLimitOffset", PERSISTENT},
    {"AutoNaviSpeedCtrlEnd", PERSISTENT},
    {"AutoNaviSpeedBumpTime", PERSISTENT},
    {"AutoNaviSpeedBumpSpeed", PERSISTENT},
    {"AutoNaviSpeedSafetyFactor", PERSISTENT},
    {"CruiseButtonMode", PERSISTENT},
    {"CruiseSpeedUnit", PERSISTENT},
    {"CruiseSpeedUnitBasic", PERSISTENT},
    {"CruiseButtonLongDelay", PERSISTENT},
    {"CruiseButtonLongDelayC2Migrated", PERSISTENT},
    {"CruiseSpeed1", PERSISTENT},
    {"CruiseSpeed2", PERSISTENT},
    {"CruiseSpeed3", PERSISTENT},
    {"CruiseSpeed4", PERSISTENT},
    {"CruiseSpeed5", PERSISTENT},
    {"AutoGasResumeGuard", PERSISTENT},            // 가속페달 재개 안전조건 (깜빡이/근접 앞차)   // 도로제한속도 대비 자동증속 상한(%), 0=off
    {"AutoResumeFromGas", PERSISTENT},
    {"AutoResumeFromGasSpeedMode", PERSISTENT},
    {"AutoResumeFromBrakeRelease", PERSISTENT},
    {"AutoResumeFromBrakeCarSpeed", PERSISTENT},
    {"AutoResumeFromBrakeReleaseDist", PERSISTENT},
    {"CalibrationParams", PERSISTENT},
    {"CarBatteryCapacity", PERSISTENT},
    {"CarParams", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
    {"CarParamsCache", CLEAR_ON_MANAGER_START},
    {"CarParamsPersistent", PERSISTENT},
    {"CarVin", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
    {"CellularUnmetered", PERSISTENT},
    {"CompletedTrainingVersion", PERSISTENT},
    {"ControlsReady", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
    {"CurrentRoute", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
    {"DisablePowerDown", PERSISTENT},
    {"ExperimentalLongitudinalEnabled", PERSISTENT}, // WARNING: THIS MAY DISABLE AEB
    {"DisableUpdates", PERSISTENT},
    {"DisengageOnAccelerator", PERSISTENT},
    {"DongleId", PERSISTENT},
    {"DoReboot", CLEAR_ON_MANAGER_START},
    {"DoShutdown", CLEAR_ON_MANAGER_START},
    {"DoUninstall", CLEAR_ON_MANAGER_START},
    {"ExperimentalMode", PERSISTENT},
    {"E2EAccMode", PERSISTENT},             // 0: ACC, 1: AUTO, 2: E2E
    {"EnableWideCamera", CLEAR_ON_MANAGER_START},
    {"ForcePowerDown", CLEAR_ON_MANAGER_START},
    {"GitBranch", PERSISTENT},
    {"GitCommit", PERSISTENT},
    {"GitDiff", PERSISTENT},
    {"GithubSshKeys", PERSISTENT},
    {"GithubUsername", PERSISTENT},
    {"GitRemote", PERSISTENT},
    {"GsmApn", PERSISTENT},
    {"GsmRoaming", PERSISTENT},
    {"HardwareSerial", PERSISTENT},
    {"HasAcceptedTerms", PERSISTENT},
    {"IMEI", PERSISTENT},
    {"InstallDate", PERSISTENT},
    {"IsDriverViewEnabled", CLEAR_ON_MANAGER_START},
    {"IsEngaged", PERSISTENT},
    {"IsLdwEnabled", PERSISTENT},
    {"IsMetric", PERSISTENT},
    {"IsOffroad", CLEAR_ON_MANAGER_START},
    {"IsOnroad", PERSISTENT},
    {"IsRHD", PERSISTENT},
    {"IsTakingSnapshot", CLEAR_ON_MANAGER_START},
    {"IsUpdateAvailable", CLEAR_ON_MANAGER_START},
    {"JoystickDebugMode", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_OFF},
    {"LastAthenaPingTime", CLEAR_ON_MANAGER_START},
    {"LastGPSPosition", PERSISTENT},
    {"LastManagerExitReason", CLEAR_ON_MANAGER_START},
    {"LastPeripheralPandaType", PERSISTENT},
    {"LastPowerDropDetected", CLEAR_ON_MANAGER_START},
    {"LastSystemShutdown", CLEAR_ON_MANAGER_START},
    {"LastUpdateException", PERSISTENT},
    {"LastUpdateTime", PERSISTENT},
    {"LiveParameters", PERSISTENT},
    {"MapboxToken", PERSISTENT | DONT_LOG},
    {"NavDestination", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_OFF},
    {"NavSettingTime24h", PERSISTENT},
    {"NavdRender", PERSISTENT},
    {"OpenpilotEnabledToggle", PERSISTENT},
    {"PandaHeartbeatLost", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_OFF},
    {"PandaSignatures", CLEAR_ON_MANAGER_START},
    {"Passive", PERSISTENT},
    {"PrimeRedirected", PERSISTENT},
    {"PrimeType", PERSISTENT},
    {"RecordFront", PERSISTENT},
    {"RecordFrontLock", PERSISTENT},  // for the internal fleet
    {"ReleaseNotes", PERSISTENT},
    {"ShouldDoUpdate", CLEAR_ON_MANAGER_START},
    {"SnoozeUpdate", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_OFF},
    {"SshEnabled", PERSISTENT},
    {"SubscriberInfo", PERSISTENT},
    {"TermsVersion", PERSISTENT},
    {"Timezone", PERSISTENT},
    {"TrainingVersion", PERSISTENT},
    {"TFollowSpeedRatio", PERSISTENT},
    {"UpdateAvailable", CLEAR_ON_MANAGER_START},
    {"UpdateFailedCount", CLEAR_ON_MANAGER_START},
    {"Version", PERSISTENT},
    {"VisionRadarToggle", PERSISTENT},
    {"MixRadarInfo", PERSISTENT},
    {"LanelessOffset", PERSISTENT},
    {"ApiCache_Device", PERSISTENT},
    {"ApiCache_DriveStats", PERSISTENT},
    {"ApiCache_NavDestinations", PERSISTENT},
    {"ApiCache_Owner", PERSISTENT},
    {"Offroad_BadNvme", CLEAR_ON_MANAGER_START},
    {"Offroad_CarUnrecognized", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
    {"Offroad_ChargeDisabled", CLEAR_ON_MANAGER_START },
    {"Offroad_ConnectivityNeeded", CLEAR_ON_MANAGER_START},
    {"Offroad_ConnectivityNeededPrompt", CLEAR_ON_MANAGER_START},
    {"Offroad_InvalidTime", CLEAR_ON_MANAGER_START},
    {"Offroad_IsTakingSnapshot", CLEAR_ON_MANAGER_START},
    {"Offroad_NeosUpdate", CLEAR_ON_MANAGER_START},
    {"Offroad_NoFirmware", CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON},
    {"Offroad_StorageMissing", CLEAR_ON_MANAGER_START},
    {"Offroad_TemperatureTooHigh", CLEAR_ON_MANAGER_START},
    {"Offroad_UnofficialHardware", CLEAR_ON_MANAGER_START},
    {"Offroad_UpdateFailed", CLEAR_ON_MANAGER_START},
    {"OPKRTimeZone", PERSISTENT},

    {"SelectedCar", PERSISTENT},
    {"UseClusterSpeed", PERSISTENT},
    {"LongControlEnabled", PERSISTENT},
    {"MadModeEnabled", PERSISTENT},
    {"CruiseSpeedMin", PERSISTENT},
    {"JerkStartLimit", PERSISTENT},
    {"StoppingDecelRate", PERSISTENT},

    {"DynamicLaneProfile", PERSISTENT},
    {"MpcPathCost", PERSISTENT},
    {"MpcLateralMotionCost", PERSISTENT},
    {"MpcLateralAccelCost", PERSISTENT},
    {"MpcLateralJerkCost", PERSISTENT},
    {"SteeringRateCost", PERSISTENT},
    {"LaneChangeEnabled", PERSISTENT},
    {"AutoLaneChangeEnabled", PERSISTENT},
    {"OffsetTotal", PERSISTENT},
    {"AdjustLaneOffset", PERSISTENT},          // 좌우 여유공간 비대칭 보정 (cm, 0=off)     // 통합 오프셋(offset_total). 전 모드 공통, Auto-Tuner Phase2 학습 대상
    {"SccSmootherState", PERSISTENT},
    {"SccSmootherSyncGasPressed", PERSISTENT},
    {"StockNaviDecelEnabled", PERSISTENT},
    {"NewRadarInterface", PERSISTENT},
    {"WideCameraOnly", PERSISTENT},
    {"AutoLaneChangeTimer", PERSISTENT},
    {"AutoLaneChangeSpeed", PERSISTENT},        // 자동/방향지시등 차선변경 허용 최저 속도 (km/h)
    {"NavigationOnOpenpilot", PERSISTENT},
    {"NooUnifiedMigrated", PERSISTENT},
    {"NooTurnSpeed", PERSISTENT},
    {"NooMode", PERSISTENT},
    {"NooTurnEndTime", PERSISTENT},
    {"StopDistance", PERSISTENT},                // shared ACC/E2E standstill distance (cm), default 600
    {"ApplyLongDynamicCost", PERSISTENT},        // 동적 longitudinal MPC cost 적용
    {"MyDrivingMode", PERSISTENT},             // 주행모드 1:SAFE 2:ECO 3:NORM 4:FAST
    {"InitMyDrivingMode", PERSISTENT},         // 부팅모드 1~4, 5=AUTO
    {"MyEcoModeFactor", PERSISTENT},           // ECO 최대가속 비율 (x100)
    {"MySafeModeFactor", PERSISTENT},           // SAFE 모드 TR 비율 (x100)
    {"PrevCruiseGap", PERSISTENT},              // 마지막 소프트웨어 크루즈 갭 1~4
    {"ShowGearAnimation", PERSISTENT},
    {"ShowCarrotHud", PERSISTENT},              // 1=좌측 carrot HUD 박스 표시, 0=숨김
    {"EonClusterHud", PERSISTENT},
    {"EonClusterHudBrightness", PERSISTENT},
    {"EonClusterHudBsdStyle", PERSISTENT},
    {"EonClusterHudCarStyle", PERSISTENT},
    {"EonClusterHudRoadSigns", PERSISTENT},
    {"CarrotLearnFrictionBase", PERSISTENT},   // friction 학습 기준선(사용자 수동값)   // 노면 표시 0:끔 1:제한속도 2:방지턱 3:둘다
    {"EonClusterHudBuildings", PERSISTENT},
    {"EonClusterHudTmapIcon", PERSISTENT},
    {"EonClusterHudJunction", PERSISTENT},
    {"EonClusterHudRoadZ", PERSISTENT},         // 노면 높낮이 배율 % (-300..300, 100=원본)
    {"EonClusterHudPitchDyn", PERSISTENT},      // 주행 중 pitch 반영 % (0..200)
    {"EonClusterHudOutputMode", PERSISTENT},
    {"EonClusterHudLayoutMode", PERSISTENT},  // 1=3분할, 2=주행+티맵 2분할
    {"EonClusterHudOutputTarget", PERSISTENT}, // 1=외부 USB HUD, 2=S9 화면, 3=동시 출력
    {"EonClusterHudConnected", CLEAR_ON_MANAGER_START},
    {"EonClusterHudHeartbeat", CLEAR_ON_MANAGER_START},
    {"EonClusterHudFps", PERSISTENT},
    {"EonClusterHudMapFps", PERSISTENT},
    {"EonClusterHudJpegQuality", PERSISTENT},
    {"EonClusterHudScreenMode", PERSISTENT},
    {"EonClusterHudTheme", PERSISTENT},
    {"EonClusterHudOrientation", PERSISTENT},
    {"EonClusterHudMirror", PERSISTENT},
    {"EonClusterHudPathFlip", PERSISTENT},       // 진단용: S9 경로/차선 좌우반전 (0=기본, 1=반전)
    {"EonClusterHudLanguage", PERSISTENT},
    {"EonClusterHudRadarInfo", PERSISTENT},
    {"ShowMapboxMap", PERSISTENT},              // 1=Mapbox/NOO Tmap 지도 이미지 표시, 0=모든 지도 이미지 숨김
    {"ShowRouteMapAlways", PERSISTENT},         // 1=목적지 경로 동안 Tmap 지도 이미지 상시 표시
    {"ShowPathWidth", PERSISTENT},               // 경로 반폭 cm (90=0.90m)
    {"ShowPathStatusColor", PERSISTENT},         // 가감속 상태에 따른 경로 색상
    {"ShowPlotMode", PERSISTENT},                // 0=off, 1..8=C3 driving analysis plot
    {"CustomSteerRatio", PERSISTENT},          // 고정 조향비 x100
    {"UseLiveSteerRatio", PERSISTENT},         // 1=liveParameters 학습 조향비 사용
    {"SteerActuatorDelay", PERSISTENT},        // 조향 지연 보상 x100 (초)
    {"LateralTorqueCustom", PERSISTENT},       // 1=아래 토크값 사용, 0=차량 기본값
    {"LateralTorqueAccelFactor", PERSISTENT},  // latAccelFactor x1000
    {"LateralTorqueFriction", PERSISTENT},     // friction x1000
    {"LateralTorqueKpV", PERSISTENT},          // kp x100
    {"LateralTorqueKiV", PERSISTENT},          // ki x100
    {"LateralTorqueKf", PERSISTENT},           // kf x100
    {"LateralTorqueKd", PERSISTENT},           // kd x100
    {"LatAccelFrictionFactor", PERSISTENT},    // friction 입력 횡가속 비율 x100
    {"LatJerkFrictionFactor", PERSISTENT},     // friction 입력 횡저크 비율 x100
    // ── LiveTorque self-learning (backport from ajouatom/openpilot hoya/c3-atune) ──
    {"ShowBlindSpotAlways", PERSISTENT},       // BSD 벽 상시표시 (진단용, 0=감지시만)         // 기어 변경 팝업 애니메이션
    {"KeepSteeringTurnSignals", PERSISTENT},
    {"HapticFeedbackWhenSpeedCamera", PERSISTENT},
    {"TurnVisionControl", PERSISTENT},
    {"AutoCurveSpeedFactor", PERSISTENT},       // vision curvature multiplier, x100
    {"AutoCurveSpeedLowerLimit", PERSISTENT},   // minimum vision/map curve speed, km/h
    {"MapTurnSpeedFactor", PERSISTENT},         // Tmap route curve speed multiplier, x100
    {"AutoNaviSpeedDecelRate", PERSISTENT},     // map curve deceleration, x100 m/s^2
    {"SoftRestartTriggered", CLEAR_ON_MANAGER_START},
    // ── CarrotPilot Auto-Tuner (commit 9dd5e2c port) ──────────────────
    {"LongCoastBand", PERSISTENT},             // 코스팅 데드밴드 (x100 정수, m/s², 기본 0=off; commit 10fa725 Phase9 추천·longcontrol 라이브 반영)
    {"LongTuningKpV", PERSISTENT},
    {"LongTuningKiV", PERSISTENT},
    {"LongTuningKf", PERSISTENT},
    {"LongActuatorDelay", PERSISTENT},
    {"LongitudinalActuatorDelayLowerBound", PERSISTENT},
    {"LongitudinalActuatorDelayUpperBound", PERSISTENT},
    {"StoppingAccel", PERSISTENT},
    // 학습 대상 파라미터 (x100 정수 저장)
    {"TFollowGap1", PERSISTENT},               // GAP1 (default 110 = 1.10s)
    {"TFollowGap2", PERSISTENT},               // GAP2 (default 120 = 1.20s)
    {"TFollowGap3", PERSISTENT},               // GAP3 (default 140 = 1.40s)
    {"TFollowGap4", PERSISTENT},               // GAP4 (default 160 = 1.60s)
    {"ComfortBrake", PERSISTENT},              // 접근 감속 기준 (x100, default 250 = 2.50m/s^2)
    {"XEgoObstacleCost", PERSISTENT},          // 차간거리 추종 강도 (x100, default 600 = 6.0)
    {"EnableSpeedTF", PERSISTENT},
    {"TFollowDecelBoost", PERSISTENT},         // 감속 중 차간시간 추가 비율 (x100, default 30)
    {"RadarReactionFactor", PERSISTENT},       // 레이더 앞차 가속도 지속 예측 비율 (x100, default 70)
    {"NoLeadCruiseAccelFactor", PERSISTENT},  // 앞차 없을 때 CruiseMax 적용 비율 (x100)
    {"NoLeadCruiseJerkLimit", PERSISTENT},    // 앞차 없을 때 가속 상승률 (x100 m/s^3)
    {"StartAccelApply", PERSISTENT},
    {"StopAccelApply", PERSISTENT},
    {"StandstillHoldApply", PERSISTENT},      // 완전정지 유지 제동값 (x100%, default 55)
    {"StandstillHoldRate", PERSISTENT},       // 완전정지 유지 증가율 (x100, default 120)
    {"SoftHoldMode", PERSISTENT},
    {"TrafficStopMode", PERSISTENT},         // 0: off/ACC, 1: conditional, 2: aPilot conditional
    {"TrafficStopAccel", PERSISTENT},        // traffic-signal deceleration factor, percent
    {"TrafficStopDistanceAdjust", PERSISTENT}, // traffic-stop line adjustment (cm), default +400

    {"CruiseMaxVals1", PERSISTENT},
    {"CruiseMaxVals2", PERSISTENT},
    {"CruiseMaxVals3", PERSISTENT},
    {"CruiseMaxVals4", PERSISTENT},
    {"CruiseMaxVals5", PERSISTENT},
    {"CruiseMaxVals6", PERSISTENT},
    {"CruiseMaxVals20", PERSISTENT},          // 20 km/h 최대가속 보간점 (x100)
};

} // namespace

Params::Params(const std::string &path) {
  static std::string default_param_path = ensure_params_path();
  params_path = path.empty() ? default_param_path : ensure_params_path(path);
}

std::vector<std::string> Params::allKeys() const {
  std::vector<std::string> ret;
  for (auto &p : keys) {
    ret.push_back(p.first);
  }
  return ret;
}


bool Params::checkKey(const std::string &key) {
  return keys.find(key) != keys.end();
}

ParamKeyType Params::getKeyType(const std::string &key) {
  return static_cast<ParamKeyType>(keys[key]);
}

int Params::put(const char* key, const char* value, size_t value_size) {
  // Information about safely and atomically writing a file: https://lwn.net/Articles/457667/
  // 1) Create temp file
  // 2) Write data to temp file
  // 3) fsync() the temp file
  // 4) rename the temp file to the real name
  // 5) fsync() the containing directory
  std::string tmp_path = params_path + "/.tmp_value_XXXXXX";
  int tmp_fd = mkstemp((char*)tmp_path.c_str());
  if (tmp_fd < 0) return -1;

  int result = -1;
  do {
    // Write value to temp.
    ssize_t bytes_written = HANDLE_EINTR(write(tmp_fd, value, value_size));
    if (bytes_written < 0 || (size_t)bytes_written != value_size) {
      result = -20;
      break;
    }

    // fsync to force persist the changes.
    if ((result = fsync(tmp_fd)) < 0) break;

    FileLock file_lock(params_path + "/.lock");

    // Move temp into place.
    if ((result = rename(tmp_path.c_str(), getParamPath(key).c_str())) < 0) break;

    // fsync parent directory
    result = fsync_dir(getParamPath());
  } while (false);

  close(tmp_fd);
  ::unlink(tmp_path.c_str());
  return result;
}

int Params::remove(const std::string &key) {
  FileLock file_lock(params_path + "/.lock");
  int result = unlink(getParamPath(key).c_str());
  if (result != 0) {
    return result;
  }
  return fsync_dir(getParamPath());
}

std::string Params::get(const std::string &key, bool block) {
  if (!block) {
    return util::read_file(getParamPath(key));
  } else {
    // blocking read until successful
    params_do_exit = 0;
    void (*prev_handler_sigint)(int) = std::signal(SIGINT, params_sig_handler);
    void (*prev_handler_sigterm)(int) = std::signal(SIGTERM, params_sig_handler);

    std::string value;
    while (!params_do_exit) {
      if (value = util::read_file(getParamPath(key)); !value.empty()) {
        break;
      }
      util::sleep_for(100);  // 0.1 s
    }

    std::signal(SIGINT, prev_handler_sigint);
    std::signal(SIGTERM, prev_handler_sigterm);
    return value;
  }
}

std::map<std::string, std::string> Params::readAll() {
  FileLock file_lock(params_path + "/.lock");
  return util::read_files_in_dir(getParamPath());
}

void Params::clearAll(ParamKeyType key_type) {
  FileLock file_lock(params_path + "/.lock");

  std::string path;
  for (auto &[key, type] : keys) {
    if (type & key_type) {
      unlink(getParamPath(key).c_str());
    }
  }

  fsync_dir(getParamPath());
}

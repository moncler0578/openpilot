"""
CarrotPilot Auto-Tuner 포팅판 (원본 commit 9dd5e2c, selfdrive/carrot/carrot_learning.py)

이 포크(구형 openpilot + neokii SCC 구조)에 맞춰 단순화한 운전자 개입 기반 학습기.
설치 위치: selfdrive/controls/lib/carrot_learning.py

  [Phase 2] OffsetTotal : 직진 주행 편차 보정 (이 포크에 이미 존재하는 파라미터, 단위 m)
            트리거: 직진(|조향각|<5도, 오버라이드 없음) 중 조향각 평균 편차
            발동:  샘플 >= 400개 (0.05s 주기 -> 약 20초)
  [Phase 4] TFollowGap1~4 : cruiseGap 단계별 추종거리
            트리거: 선행차 추종 중 gas(좁히기 의도) / brake(넓히기 의도) 개입
            발동:  gas 누적 >= 15초 / brake 누적 >= 10초
  [Phase 5] latAccelFactor / friction / steerActuatorDelay / steerRatio : 조향 파라미터
            (nTune JSON 직접 수정)
            이 포크의 latcontrol_torque는 Params가 아닌 /data/ntune/lat_torque*.json을
            라이브 로딩하므로, 학습기가 JSON 파일을 직접 수정 → 재시작 없이 즉시 반영.
            트리거: 커브(|조향각|>=5도, 40km/h 이상) 중 조향 개입.
                    방향 판정 = steeringTorque × steeringAngleDeg 부호
                    (같은 방향=조향력 부족→latAccelFactor 감소 / 반대=안쪽 쏠림→증가)
                    직선 미세 개입 비율 높음 → friction 상향
            발동:  커브 샘플 >= 600개(약 30초) / 직선 샘플 >= 400개
            steerRatio : override 휴리스틱이 아니라 liveParameters(paramsd 칼만 추정)을
            '정답'으로 누적해 nTune common.json steerRatio 를 보정. (아래 _SR_* 참고)

저장: Params("CarrotLearningData") JSON
추천: P단 전환 시 Params("CarrotLearningRecommend") 기록 + CarrotLearningPopupReady=1
적용: CarrotLearningAutoApply=1 이면 P단 전환 시 즉시 자동 적용 + History 기록 (최대 50개)
      (이 포크에는 추천 팝업/그래프 UI가 없으므로 AutoApply 사용 또는 SSH로 확인 권장)
초기화: CarrotLearningClear=1 -> 누적 학습 데이터 삭제

원본 대비 제외된 것 (이 포크에 대응 신호/파라미터 없음):
  - Phase 3 JLeadFactor3, Phase 5 DynamicTFollow/TFollowDecelBoost (jLead 신호 없음)
    -> 수동 브레이크 개입은 Phase 4의 '거리 넓히기' 신호로 흡수
    -> 단, 원본 commit 00b70cd의 '선제(교육용) 제동' 학습 의도는 jLead 대신
       종방향 응답성(longKf) 으로 적응 반영 (아래 _LEAD_PROACTIVE_* 참고)
  - 토크 조향 파라미터 학습 (LateralTorque* 파라미터가 이 포크에 없음)
  - 주행 중 타이머/정차 팝업 (UI 없음, parking 트리거만 유지)

추가 (원본 commit 10fa725):
  [Phase 9] 수동주행 기준분포 로거 → LongCoastBand : 비인게이지(사람 직접 운전) 중
            속도밴드별 자연 가감속/코스팅 감속/추종 차간시간을 '정답'으로 누적.
            1차 적용은 무페달 코스팅 자연 감속 → LongCoastBand(코스팅 데드밴드) 추천.
            (LongCoastBand 키를 포크 params 레지스트리에 PERSISTENT INT 기본 "0" 으로
             등록해야 하며, longcontrol.py가 1초마다 라이브 반영)

추가 (원본 commit 1e95637):
  [응답보강 신호 정제] 추종 중 운전자 브레이크 중 '늦은(긴급) 제동'만 별도 누적
            (_tfollow_brake_late). longKf↑/longActuatorDelay↓ 응답보강은 이 늦은
            제동에서만 파생시켜, 플래너 onset jerk 완화로 생긴 '여유 감속'이 다시
            longKf를 끌어올려 제동을 날카롭게 만드는 악순환을 차단한다. 여유 감속은
            Phase 4 TFollowGap '넓히기'(_tfollow_brake_acc)로만 흡수된다.

추가 (원본 commit dff7287):
  [Apply LAT/LONG 토글 ↔ 학습 게이팅 연동] 기존에는 Apply LAT/LONG 토글이 추천 계산
            (_calc_recommendations)에만 반영되어, 토글을 꺼둔 상태에서도 원시 데이터
            누적(update)은 계속 진행됐다. 그 결과 꺼둔 동안 쌓인 데이터가 토글을 다시
            켤 때 한꺼번에 반영되는 문제가 있었다. 이제 update() 진입 시점에 두 토글을
            읽어, 해당 카테고리의 누적 자체를 건너뛴다.
              - apply_lat  : Phase 2(OffsetTotal) / Phase 5(토크 조향) / steerRatio 게이트
              - apply_long : Phase 4(추종거리) / Phase 9(수동주행 로거) 게이트

추가 (원본 commit ffec86e, LateralTorqueKpV/KiV 대상 → latAccelFactor/friction에 적응):
  [Phase 5 진동-우선 판정] 원본은 커브 추종오차의 '부호 패턴'(반전=진동 / 유지=정상상태
            lag)으로 KpV/KiV 방향을 정해, 진동으로 생긴 오차를 '게인 부족'으로 오인해
            무한 상향시키던 발산 버그를 막는다. 이 포크엔 KpV/KiV(Params 기반 피드백
            게인)도 steer_err(추종오차)도 없으므로, 이미 수집 중인 '개입 방향'
            (tq_under=조향력부족 / tq_inner=안쪽쏠림)의 반전 패턴을 진동 신호로 대신
            쓴다: 방향이 자주 뒤집히면 개입 비율(크기)만 보고 latAccelFactor를 계속
            같은 쪽으로 밀어붙이지 않고 토크를 완화하는 쪽으로만 조정한다. 고속 직선
            핑퐁(개입 없이 조향각 자체가 0 근처에서 반복 반전)도 원본과 동일 로직으로
            별도 검출해 같은 완화 경로에 합류시킨다.
            [정상상태 lag → Kp 보조 상향] 진동이 아니라 방향이 꾸준히 한쪽으로 유지되면
            (정상상태 lag), 원본처럼 FF(이 포크에선 friction)가 이미 충분할 때만
            latAccelFactor를 작은 스텝(_LAF_STEADY_STEP)으로 보조 조정한다. friction이
            아직 부족하면 기존 비율(≥30%) 기반 언더/이너 분기가 friction부터 채우도록
            우선순위를 양보한다.

추가 (사용자 요청, selfdrive/controls/lib/vision_turn_controller.py 대상):
  [Phase 6] 비전 커브 감속 학습 → TurnEnteringDecel0~1 / TurnTurningAcc0~4 /
            TurnLeavingAcc : VisionTurnController의 ENTERING(진입 감속)/
            TURNING(커브 중 가감속)/LEAVING(탈출 가속) 상태에서 페달 개입으로
            _ENTERING_SMOOTH_DECEL_V / _TURNING_ACC_V / _LEAVING_ACC 를 학습한다.
            트리거: 각 상태에서 gas(그 구간 가감속이 과함→완화) 또는 brake(부족함
            →강화) 개입을 BP(진입=max_pred_lat_acc, 추종=current_lat_acc) 밴드별로
            누적. VisionTurnController가 read_learned_turn_params()로 5초 주기
            반영(longitudinal_planner의 read_param() 패턴과 동일).
            발동: 밴드당 gas 누적 >= 8초 / brake 누적 >= 6초, 1회 변동 ±0.20 m/s^2.
"""
import os
import json
import datetime
import numpy as np

from common.params import Params
from common.realtime import DT_MDL

_DT = DT_MDL  # longitudinal_planner.update() 호출 주기 (0.05s)

# ── Phase 9 수동주행 로거 속도 대역 (km/h) ──
_BP_KPH = [0., 36., 90., 144.]
_NUM_BANDS = len(_BP_KPH)
_TFOLLOW_MAX_DELTA = 15                   # 추종거리 세션당 변동폭 제한

# ── Phase 2 상수: OffsetTotal (단위 m, float) ──
_STRAIGHT_DEG = 5.0
_LATERAL_MIN_SAMPLES = 400                # 0.05s * 400 = 약 20초 직진
_PATH_OFFSET_DEG_THRESHOLD = 1.5          # 평균 편차 이 이상이면 추천
_PATH_OFFSET_M_PER_DEG = 0.01             # 1도 편차 ≈ 0.01m 보정 (실험값)
_PATH_OFFSET_LIMIT = 0.15                 # ±0.15m 제한

# ── Phase 4 상수: CRUISE_GAP_V x100 ──
_TFOLLOW_KEYS = ["TFollowGap1", "TFollowGap2", "TFollowGap3", "TFollowGap4"]
_TFOLLOW_DEFAULTS = [110, 120, 140, 160]  # CRUISE_GAP_V = [1.1, 1.2, 1.4, 1.6]
_TFOLLOW_GAS_THRESHOLD_SEC = 15.0
_TFOLLOW_BRAKE_THRESHOLD_SEC = 10.0
_TFOLLOW_MIN_V_KPH = 40.0
_TFOLLOW_MAX_LEAD_DREL = 150.0
_TFOLLOW_MIN = 90                         # 0.90초 안전 하한
_TFOLLOW_MAX = 220
_TFOLLOW_NARROW_STEP = 5
_TFOLLOW_WIDEN_STEP = 5

# ── Phase 5: 토크 조향 (nTune) ──
_NTUNE_DIR = "/data/ntune"
_TQ_MIN_V_KPH = 40.0
_TQ_CURVE_DEG = 5.0                        # |조향각| 이 이상이면 커브 구간
_TQ_MIN_CURVE_SAMPLES = 600                # 0.05s * 600 = 약 30초 커브 주행
_TQ_OVERRIDE_HI = 0.30                     # 개입 비율 이 이상이면 보정 추천
_TQ_DIR_DOMINANCE = 1.5                    # 한 방향 개입이 반대의 1.5배 이상일 때만
_LAF_STEP = 0.10                           # latAccelFactor 1회 변화량
_LAF_MIN, _LAF_MAX = 1.0, 4.0

# ── 진동-우선 판정 (commit ffec86e 적응, 원본은 LateralTorqueKpV/KiV 대상) ──────
# 이 포크엔 KpV/KiV(Params 기반 피드백 게인)가 없어 latAccelFactor/friction(nTune)에
# 대신 적용한다. 원본의 '추종오차 부호패턴'은 이 포크에 없으므로(steer_err 미보유),
# 대신 이미 수집 중인 '개입 방향(tq_under=조향력부족 / tq_inner=안쪽쏠림)'의 반전
# 패턴을 진동 신호로 쓴다: 방향이 자주 뒤집히면 = latAccelFactor가 실제로는 맞는데
# 진동으로 잘못 개입되는 것 → 크기 신호(override_ratio)만 보고 계속 밀어붙이면
# (구버전 KpV처럼) 발산한다. 고속 직선 핑퐁(개입 없이 조향각 자체가 흔들림)도
# 원본과 동일 로직으로 별도 검출한다.
_TQ_OSC_MIN = 6                # 커브 개입방향 반전(진동) 판정 최소 누적
_TQ_STEADY_MIN = 15             # 커브 개입방향 유지(정상상태 편향) 판정 최소 누적
_TQ_PINGPONG_MIN_KPH = 70.0     # 고속 직선 핑퐁 검출 최소 속도
_TQ_PINGPONG_DEG = 0.4          # 핑퐁 반전 판정 조향각 데드밴드 (도)
_TQ_PINGPONG_MIN_COUNT = 15     # 직선 핑퐁 판정 최소 반전 횟수
_LAF_OSC_STEP = 0.10            # 진동 감지 시 latAccelFactor 완화(토크↓) 스텝
_FRICTION_SUFFICIENT = 0.12     # friction 이 이 이상이면 'FF(마찰보정) 충분' 판정
_LAF_STEADY_STEP = 0.05         # 정상상태 lag 보조 상향/완화 스텝 (기존 _LAF_STEP의 절반)
# ────────────────────────────────────────────────────────────────────────
_TQ_MIN_STR_SAMPLES = 400                  # 직선 약 20초
_STR_OVERRIDE_HI = 0.35                    # friction 상향 기준
_STR_OVERRIDE_LO = 0.05                    # friction 하향 수렴 기준
_FRICTION_STEP = 0.01
_FRICTION_MIN, _FRICTION_MAX = 0.0, 0.20

# ── Phase 6: 비전 커브 감속 학습 (selfdrive/controls/lib/vision_turn_controller.py) ──
# ENTERING(진입 감속) / TURNING(커브 중 가감속) / LEAVING(탈출 가속) 상태에서 운전자
# 페달 개입 방향으로 vision_turn_controller.py의 _ENTERING_SMOOTH_DECEL_V /
# _TURNING_ACC_V / _LEAVING_ACC 를 학습한다.
#   - gas 개입   = 그 구간 감속이 과함 / 탈출가속이 부족함 → 완화(가속도 값↑, 0 방향)
#   - brake 개입 = 그 구간 감속이 부족함 / 탈출가속이 과함 → 강화(가속도 값↓)
# BP(구간 경계)는 vision_turn_controller.py 원본 그대로 고정하고, V(가속도) 값만
# TFollowGap과 동일한 패턴(Params, x100 정수)으로 학습·대체한다.
_TURN_ENTERING_BP = [1.3, 3.0]             # _ENTERING_SMOOTH_DECEL_BP 와 동일 (max_pred_lat_acc)
_TURN_ENTERING_KEYS = ["TurnEnteringDecel0", "TurnEnteringDecel1"]
_TURN_ENTERING_DEFAULTS = [-10, -30]       # _ENTERING_SMOOTH_DECEL_V x100
_TURN_ENTERING_MIN, _TURN_ENTERING_MAX = -100, 0     # -1.00 ~ 0.00 m/s^2 (x100)

_TURN_TURNING_BP = [1.5, 10.0, 12.0, 14.0, 16.0]     # _TURNING_ACC_BP 와 동일 (current_lat_acc)
_TURN_TURNING_KEYS = ["TurnTurningAcc0", "TurnTurningAcc1", "TurnTurningAcc2",
                       "TurnTurningAcc3", "TurnTurningAcc4"]
_TURN_TURNING_DEFAULTS = [120, 94, 90, 80, -10]      # _TURNING_ACC_V x100
_TURN_TURNING_MIN, _TURN_TURNING_MAX = -100, 200     # -1.00 ~ 2.00 m/s^2 (x100)

_TURN_LEAVING_KEY = "TurnLeavingAcc"
_TURN_LEAVING_DEFAULT = 50                 # _LEAVING_ACC x100
_TURN_LEAVING_MIN, _TURN_LEAVING_MAX = 0, 150        # 0.00 ~ 1.50 m/s^2 (x100)

_TURN_ACC_STEP = 5              # 1회 조정량 (x100 단위 = 0.05 m/s^2)
_TURN_GAS_THRESHOLD_SEC = 8.0   # 밴드당 가속 개입 누적 임계 (감속 과다/탈출가속 부족 신호)
_TURN_BRAKE_THRESHOLD_SEC = 6.0 # 밴드당 브레이크 개입 누적 임계 (감속 부족/탈출가속 과다 신호)
_TURN_MAX_DELTA = 20            # 세션당 변동폭 제한 (0.20 m/s^2)
# ────────────────────────────────────────────────────────────────────────

# ── SteerActuatorDelay (nTune common.json) ──
_NTUNE_COMMON_FILE = "/data/ntune/common.json"
_SAD_STEP = 0.01                           # 1회 변화량 (초)
_SAD_MIN, _SAD_MAX = 0.0, 0.8              # nTune checkValidCommon 범위와 동일
_SAD_OVERRIDE_HI = 0.40                    # 커브 개입 비율 이 이상이면 딜레이 하향(반응 빠르게)
_SAD_OVERRIDE_LO = 0.08                    # 개입 거의 없으면 소폭 상향 수렴(안정)

# ── steerRatio 학습 (nTune common.json, liveParameters 추정 기반) ─────────────
# steerRatio는 openpilot paramsd가 칼만필터로 라이브 추정(liveParameters.steerRatio)한다.
# 그 추정값을 qualifying 구간에서 '정답'으로 누적해 nTune common.json 의 steerRatio 를
# 보정한다. override 방향 휴리스틱으로 유추하지 않는 이유: steerRatio와 latAccelFactor는
# 효과가 겹쳐(degenerate) 같은 개입신호로 둘 다 학습하면 서로 밀어내며 진동한다. 전용
# 추정기 출력을 쓰는 것이 정확하고 안정적이다.
# (참고: common.json useLiveSteerRatio=1 이면 컨트롤러(controlsd)는 라이브 값을 직접 쓰므로
#  이 고정값은 '동결 백업'(추후 useLiveSteerRatio를 끌 때 사용)으로 의미를 가진다.)
_SR_MIN, _SR_MAX = 10.0, 20.0              # nTune checkValidCommon steerRatio 범위와 동일
_SR_DEFAULT = 16.5
_SR_MIN_V_KPH = 30.0                       # 추정 신뢰 구간 (저속 제외)
_SR_MIN_SAMPLES = 600                      # 약 30초 qualifying 주행
_SR_DEADBAND = 0.1                         # 이 이상 차이날 때만 추천 (미세 변동 억제)
_SR_MAX_DELTA = 0.5                        # 세션당 변동 상한 (급변 방지)

# ── 종방향 응답성 (longcontrol.py 라이브 반영, 경로 A 개입 기반) ──
# longcontrol이 Params에서 1초마다 읽어 actuatorDelay/kf 를 라이브 반영.
# 신호: Phase 4 추종 카운터 재활용 (브레이크 개입=반응 느림, 가속 개입=굼뜸)
_LONG_ACT_DELAY_KEY = "CarrotLongActuatorDelay"
_LONG_KF_KEY = "CarrotLongKf"
_LONG_DELAY_DEFAULT = 0.4                   # interface.py longitudinalActuatorDelay 기본
_LONG_KF_DEFAULT = 1.0                      # PID kf 기본 (실제 적용은 longcontrol clip 0.7~1.3)
_LONG_DELAY_STEP = 0.02
_LONG_KF_STEP = 0.03
_LONG_DELAY_MIN, _LONG_DELAY_MAX = 0.20, 0.60   # 보수적 범위 (longcontrol은 0.1~1.0 clip)
_LONG_KF_MIN, _LONG_KF_MAX = 0.70, 1.30
_LONG_BRAKE_THRESHOLD_SEC = 12.0           # 추종 중 '늦은' 브레이크 누적 (반응 느림 판정)
_LONG_GAS_THRESHOLD_SEC = 18.0             # 추종 중 가속 누적 (굼뜸 판정)

# ── 선제(교육용) 선행차 제동 학습 (원본 commit 00b70cd 적응) ──
# 원본은 Phase 3 JLeadFactor3 를 강화하지만 이 포크엔 jLead 파라미터가 없으므로,
# '선행차에 더 일찍 반응하라'는 교육 신호를 종방향 응답성(longKf)으로 매핑한다.
# 위험할 만큼 늦지 않은 TTC(3.5~6.0s)에 유의미한 감속(≥1.0m/s^2)으로 반복 선제 제동 →
# longKf 를 약하게(+0.02) 상향. 정상 여유 제동의 과누적을 막기 위해 보수적 임계 + 작은 step.
_LEAD_PROACTIVE_TTC_LO = 3.5                # 이 TTC 미만은 '위험한 늦은 제동'(별도 처리 대상)
_LEAD_PROACTIVE_TTC_HI = 6.0               # 선제 교육 제동으로 인정할 TTC 상한
_LEAD_PROACTIVE_DECEL = 1.0                # 선제 제동이 '굼뜬 반응' 신호로 인정될 최소 감속도 (m/s^2)
_LEAD_PROACTIVE_MIN_COUNT = 6             # 추천 발동 최소 선제 제동 이벤트 수 (반복성 요구)
_LONG_KF_PROACTIVE_STEP = 0.02            # 선제 제동 시 약한 kf 증가 (강한 brake_total 경로 0.03보다 작게)

# ── 늦은(긴급) 제동 분리 집계 (원본 commit 1e95637 적응) ──────────────────
# 원본은 '자율 급제동' 카운터에 TTC 게이트를 달아, onset jerk 완화로 생긴 '여유
# 감속'이 late braking 으로 오인되어 제동을 다시 날카롭게 만드는 악순환을 막는다.
# 이 포크엔 자율제동 카운터가 없고 응답보강은 운전자 브레이크 누적에서 파생되므로,
# 같은 취지로 '늦은(긴급)' 구간만 _tfollow_brake_late 로 분리해 보강 전용 신호로 쓴다.
_LATE_BRAKE_TTC = 6.0           # 늦은(접근) 제동 인정 TTC 상한 (이상=여유 감속)
_LATE_BRAKE_PANIC_DECEL = -2.5  # TTC 무관, 패닉 감속이면 늦은 제동으로 인정 (m/s^2)

# ── Phase 9 상수 (수동주행 기준분포 로거 → LongCoastBand 추천, 원본 commit 10fa725) ──
# 인게이지 '개입 카운팅'과 달리, 사람이 직접 운전(비인게이지)하는 동안의 자연 주행을
# '정답'으로 통째 누적한다. 1차 적용: 무페달(코스팅) 구간의 자연 감속(회생제동/엔진
# 브레이크)을 측정해 종방향 코스팅 데드밴드(LongCoastBand)를 직접 보정 — 역문제 없음.
# (LongCoastBand 는 longcontrol.py가 1초마다 라이브로 읽어 반영; 포크 params 키 등록 필요)
_MANUAL_MIN_V_KPH = 20.0        # 수집 최소 속도 (정체 stop&go 잡음 배제)
_MANUAL_COAST_MIN_N = 300       # LongCoastBand 추천 발동 최소 코스팅 감속 표본 수
_MANUAL_COAST_MIN_SEC = 60.0    # LongCoastBand 추천 발동 최소 누적 코스팅 시간 (_DT 무관, 주 게이트)
_MANUAL_COAST_GAIN = 0.25       # 측정 코스팅 감속 → 데드밴드 변환 계수 (차의 코스트 권한 일부만 사용)
_LONG_COAST_BAND_MAX = 40       # LongCoastBand 안전 상한 (0.40 m/s², params_keys.h 범위와 동일)


# 추천 키 → 소속 Phase. '적용'된 Phase의 누적치만 선택적으로 리셋하기 위한 매핑.
# (과거 _reset_counters()는 적용 여부와 무관하게 전체 누적을 리셋 → 느리게 쌓이는
#  조향 학습(Phase2 OffsetTotal 약 20초, Phase5 토크 약 30초)이 무관한 longitudinal
#  적용 때마다 초기화되어 문턱에 도달하지 못하던 버그가 있었다.)
# longActuatorDelay/longKf 는 Phase 4 추종 카운터에서 파생되므로 Phase 4 에 귀속.
# latAccelFactor/friction/steerActuatorDelay/steerRatio 는 모두 Phase 5(조향)에 귀속.
_KEY_RESET_PHASE = {
  "OffsetTotal": 2,
  **{k: 4 for k in _TFOLLOW_KEYS},
  _LONG_ACT_DELAY_KEY: 4, _LONG_KF_KEY: 4,
  "latAccelFactor": 5, "friction": 5, "steerActuatorDelay": 5, "steerRatio": 5,
  **{k: 6 for k in _TURN_ENTERING_KEYS},
  **{k: 6 for k in _TURN_TURNING_KEYS},
  _TURN_LEAVING_KEY: 6,
  "LongCoastBand": 9,
}


def _find_torque_file():
  """nTune 토크 파일 자동 탐색 (lat_torque*.json, 버전명 차이 대응)"""
  try:
    for fn in sorted(os.listdir(_NTUNE_DIR)):
      if fn.startswith("lat_torque") and fn.endswith(".json"):
        return os.path.join(_NTUNE_DIR, fn)
  except OSError:
    pass
  return os.path.join(_NTUNE_DIR, "lat_torque_v4.json")


def _ntune_read(path):
  try:
    with open(path) as f:
      return json.load(f)
  except Exception:
    return {}


# ── 토크값 저장소 : nTune JSON → Params (carrot 방식) ────────────────────────
# latcontrol_torque 가 LateralTorqueAccelFactor / LateralTorqueFriction 를
# 0.1초마다 다시 읽는다. 키 이름(latAccelFactor/friction)은 학습기 내부 식별자로
# 그대로 두고, 저장/조회만 Params 로 바꾼다.
_TORQUE_PARAM_KEY = {
  "latAccelFactor": ("LateralTorqueAccelFactor", 1000.0, 2.7),
  "friction":       ("LateralTorqueFriction",    1000.0, 0.08),
}


_COMMON_PARAM_KEY = {
  "steerActuatorDelay": ("SteerActuatorDelay", 100.0, 0.10),
  "steerRatio":         ("CustomSteerRatio",   100.0, 16.50),
}


def _common_read():
  from common.params import Params as _P
  out = {}
  pr = _P()
  for k, (pkey, scale, dflt) in _COMMON_PARAM_KEY.items():
    try:
      v = pr.get(pkey, encoding="utf8")
      out[k] = float(v) / scale if v not in (None, "") else dflt
    except (TypeError, ValueError):
      out[k] = dflt
  return out


def _common_write(key, value):
  from common.params import Params as _P
  if key not in _COMMON_PARAM_KEY:
    return
  pkey, scale, _ = _COMMON_PARAM_KEY[key]
  _P().put(pkey, str(int(round(float(value) * scale))))


def _torque_read():
  from common.params import Params as _P
  out = {}
  pr = _P()
  for k, (pkey, scale, dflt) in _TORQUE_PARAM_KEY.items():
    try:
      v = pr.get(pkey, encoding="utf8")
      out[k] = float(v) / scale if v not in (None, "") else dflt
    except (TypeError, ValueError):
      out[k] = dflt
  return out


def _torque_write(key, value):
  from common.params import Params as _P
  if key not in _TORQUE_PARAM_KEY:
    return
  pkey, scale, _ = _TORQUE_PARAM_KEY[key]
  _P().put(pkey, str(int(round(float(value) * scale))))
  _P().put("LateralTorqueCustom", "1")   # 학습값을 쓰려면 커스텀 모드여야 한다


def _ntune_write(path, data):
  try:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
      json.dump(data, f, indent=2)
    os.chmod(path, 0o666)  # nTune.write_config와 동일: 타 프로세스 쓰기 권한 유지
  except Exception:
    pass


# ── Params 헬퍼 (이 포크 python Params에 get_int/put_int가 없으므로) ──────
def _get_int(params, key, default):
  raw = params.get(key, encoding='utf8')
  try:
    return int(raw) if raw else default
  except (TypeError, ValueError):
    return default


def _get_float(params, key, default):
  raw = params.get(key, encoding='utf8')
  try:
    return float(raw) if raw else default
  except (TypeError, ValueError):
    return default


def _remove(params, key):
  """python 바인딩 버전에 따라 remove/delete 명칭이 달라 안전하게 처리"""
  for name in ("remove", "delete"):
    fn = getattr(params, name, None)
    if fn is not None:
      try:
        fn(key)
        return
      except Exception:
        pass
  params.put(key, "")


# ── planner / long_mpc 에서 학습값을 읽어가는 공용 함수 ───────────────────
def read_learned_tfollow(params):
  """long_mpc의 CRUISE_GAP_V 대체값 (초 리스트, 길이 4)"""
  return [float(np.clip(_get_int(params, _TFOLLOW_KEYS[i], _TFOLLOW_DEFAULTS[i]),
                        _TFOLLOW_MIN, _TFOLLOW_MAX)) / 100.0
          for i in range(4)]


def _speed_band(v_ego_kph):
  for i in range(_NUM_BANDS - 1, -1, -1):
    if v_ego_kph >= _BP_KPH[i]:
      return i
  return 0


def _band_index(value, bp):
  """bp(오름차순 경계 리스트)에서 value가 속하는 밴드 인덱스.
  _speed_band와 동일한 '이하 마지막 매치' 규칙의 범용 버전 (Phase 6 커브 밴드용)."""
  idx = 0
  for i in range(len(bp) - 1, -1, -1):
    if value >= bp[i]:
      idx = i
      break
  return idx


def read_learned_turn_params(params):
  """vision_turn_controller의 커브 감속/가속 테이블 대체값 (m/s^2 단위).
  반환: (entering_decel_v: [2], turning_acc_v: [5], leaving_acc: float)"""
  entering = [float(np.clip(_get_int(params, _TURN_ENTERING_KEYS[i], _TURN_ENTERING_DEFAULTS[i]),
                            _TURN_ENTERING_MIN, _TURN_ENTERING_MAX)) / 100.0
              for i in range(2)]
  turning = [float(np.clip(_get_int(params, _TURN_TURNING_KEYS[i], _TURN_TURNING_DEFAULTS[i]),
                           _TURN_TURNING_MIN, _TURN_TURNING_MAX)) / 100.0
             for i in range(5)]
  leaving = float(np.clip(_get_int(params, _TURN_LEAVING_KEY, _TURN_LEAVING_DEFAULT),
                          _TURN_LEAVING_MIN, _TURN_LEAVING_MAX)) / 100.0
  return entering, turning, leaving


class CarrotLearner:
  """longitudinal_planner.update()에서 매 프레임(0.05s) 호출."""

  def __init__(self):
    self._params = Params()
    # Phase 2
    self._steer_acc = 0.0
    self._steer_count = 0
    # Phase 4
    self._tfollow_gas_acc = [0.0] * 4
    self._tfollow_brake_acc = [0.0] * 4
    self._tfollow_brake_late = [0.0] * 4   # 추종 중 '늦은(긴급)' 브레이크 누적 (longKf 보강 전용, commit 1e95637)
    self._tfollow_min_gap = [999.0] * 4
    self._lead_proactive_count = 0   # 선제(교육용) 제동 이벤트 수 (commit 00b70cd 적응)
    self._current_gap = 4
    # Phase 5 (토크 조향)
    self._tq_curve_samples = 0
    self._tq_curve_overrides = 0
    self._tq_under = 0          # 더 꺾는 개입 (조향력 부족)
    self._tq_inner = 0          # 풀어주는 개입 (안쪽 쏠림)
    self._tq_str_samples = 0
    self._tq_str_overrides = 0
    self._tq_prev_pressed = False
    # 진동-우선 판정용 (commit ffec86e 적응)
    self._tq_osc_count = 0          # 커브 개입방향 반전(진동) 누적
    self._tq_steady_count = 0       # 커브 개입방향 유지(정상상태 편향) 누적
    self._tq_straight_reversal = 0  # 고속 직선 조향각 반전(핑퐁) 누적
    self._prev_tq_dir_sign = 0      # 직전 개입방향 부호 (+1 under / -1 inner)
    self._prev_tq_straight_sign = 0 # 직전 직선 조향각 부호 (핑퐁 검출)
    # steerRatio 학습 (liveParameters 추정 누적, Phase 5 조향 그룹)
    self._sr_sum = 0.0
    self._sr_n = 0
    # Phase 6 (비전 커브 감속: entering/turning/leaving 개입 밴드별 누적)
    self._turn_entering_gas_acc = [0.0] * 2
    self._turn_entering_brake_acc = [0.0] * 2
    self._turn_turning_gas_acc = [0.0] * 5
    self._turn_turning_brake_acc = [0.0] * 5
    self._turn_leaving_gas_acc = 0.0
    self._turn_leaving_brake_acc = 0.0
    # Phase 9 (수동주행 기준분포 로거 → LongCoastBand). 모두 밴드별 누적.
    self._manual_coast_sec = [0.0] * _NUM_BANDS        # 무페달(코스팅) 누적 시간
    self._manual_coast_decel_sum = [0.0] * _NUM_BANDS  # 코스팅 중 자연 감속 크기 합 (m/s², 양수)
    self._manual_coast_decel_n = [0] * _NUM_BANDS      # 코스팅 중 감속 표본 수
    self._manual_gas_accel_sum = [0.0] * _NUM_BANDS    # 가속페달 시 사람의 가속도 합 (m/s²)
    self._manual_gas_n = [0] * _NUM_BANDS              # 가속페달 표본 수
    self._manual_brake_decel_sum = [0.0] * _NUM_BANDS  # 브레이크 시 사람의 감속 크기 합 (m/s², 양수)
    self._manual_brake_n = [0] * _NUM_BANDS            # 브레이크 표본 수
    self._manual_gap_sum = 0.0       # 수동 추종 차간시간 합 (s)
    self._manual_gap_n = 0           # 수동 추종 표본 수
    # 공통
    self._prev_brake = False
    self._prev_gear_park = True
    self._has_driven = False
    self._frame = 0
    self._load()

  # ── 공개 API ─────────────────────────────────────────────────────────
  def is_active(self):
    return self._params.get_bool("CarrotLearningActive")

  def set_current_gap(self, gap):
    """현재 cruiseGap 단계 (1~4). planner에서 매 프레임 전달."""
    self._current_gap = max(1, min(4, int(gap)))

  def update(self, v_ego_kph, gas_pressed, engaged, gear_park,
             steer_deg=0.0, steer_pressed=False, brake_pressed=False,
             lead_drel=0.0, lead_v_kph=0.0, a_ego=0.0, v_cruise_kph=0.0,
             gas_val=0.0, blinker=False, steer_torque=0.0, steer_deg_corr=None,
             steer_ratio_live=0.0, steer_ratio_valid=False,
             tvc_entering=False, tvc_turning=False, tvc_leaving=False,
             tvc_current_lat_acc=0.0, tvc_max_pred_lat_acc=0.0):
    if not self.is_active():
      if self._params.get_bool("CarrotLearningPopupReady"):
        self._params.put_bool("CarrotLearningPopupReady", False)
      return

    # UI/SSH로부터 초기화 신호
    if self._params.get_bool("CarrotLearningClear"):
      self._clear()
      self._params.put_bool("CarrotLearningClear", False)

    # Factory Reset 신호 (commit e06a7dd): UI가 Params 기본값을 이미 기록했으므로
    # 여기서는 onroad 인스턴스의 메모리상 누적 카운터만 재동기화 + 플래그 해제
    if self._params.get_bool("CarrotTunerFactoryReset"):
      self._clear()
      self._params.put_bool("CarrotTunerFactoryReset", False)

    if engaged:
      self._has_driven = True

    # Apply 토글(LAT/LONG)이 꺼져 있으면 해당 카테고리의 학습(데이터 누적)도 함께
    # 멈춘다. (commit dff7287) 과거에는 토글이 추천 계산(_calc_recommendations)에만
    # 반영되어, 꺼둔 동안에도 누적은 계속되다가 다시 켰을 때 한꺼번에 반영되는
    # 문제가 있었다. 이제 update() 진입 시점에 읽어 누적 자체를 게이팅한다.
    #   - apply_lat  : Phase 2(OffsetTotal) / Phase 5(토크 조향) / steerRatio 게이트
    #   - apply_long : Phase 4(추종거리) / Phase 9(수동주행 로거) 게이트
    raw_lat = self._params.get("CarrotTunerApplyLat", encoding='utf8')
    raw_long = self._params.get("CarrotTunerApplyLong", encoding='utf8')
    apply_lat = True if not raw_lat else raw_lat.strip() == "1"
    apply_long = True if not raw_long else raw_long.strip() == "1"

    # 학습 제외 조건 (깜빡이 / 극단적 가속)
    exclude = blinker or (a_ego > 2.2) or (gas_val > 0.7)

    # ── Phase 2: 직진 편차 (OffsetTotal) ──
    # 센서 영점 오프셋(angleOffsetDeg) 오학습 방지:
    # controlsState.angleSteers(보정값)가 전달되면 그것을 누적, 없으면 raw 사용
    if apply_lat and engaged and v_ego_kph >= 20.0 and abs(steer_deg) < _STRAIGHT_DEG \
       and not steer_pressed and not blinker:
      dev_deg = steer_deg_corr if steer_deg_corr is not None else steer_deg
      self._steer_acc += dev_deg
      self._steer_count += 1

    # ── Phase 4: 선행차 추종 중 페달 개입 ──
    if apply_long and engaged and v_ego_kph >= _TFOLLOW_MIN_V_KPH \
       and 0.0 < lead_drel < _TFOLLOW_MAX_LEAD_DREL:
      gi = self._current_gap - 1
      if gas_pressed and not exclude:
        self._tfollow_gas_acc[gi] += _DT
        v_ms = v_ego_kph / 3.6
        if v_ms > 1.0:
          self._tfollow_min_gap[gi] = min(self._tfollow_min_gap[gi], lead_drel / v_ms)
      elif brake_pressed and not blinker:
        self._tfollow_brake_acc[gi] += _DT
        # 접근(closing) TTC — '늦은(긴급) 제동' 분리 집계와 선제 제동 검출 공용
        v_ms = v_ego_kph / 3.6
        closing = v_ms - lead_v_kph / 3.6
        ttc_b = lead_drel / closing if closing > 0.1 else 999.0
        # ── (commit 1e95637 적응) 늦은(긴급) 제동만 longKf 응답보강 경로로 분리 누적 ──
        # TTC<6s(접근) 또는 패닉 감속(-2.5↓)일 때만. 여유 감속(높은 TTC)은 TFollowGap
        # '넓히기'로만 흡수 → onset jerk 완화로 생긴 정상 감속이 다시 longKf를 끌어올려
        # 제동을 날카롭게 만드는 악순환을 차단한다.
        if ttc_b < _LATE_BRAKE_TTC or a_ego < _LATE_BRAKE_PANIC_DECEL:
          self._tfollow_brake_late[gi] += _DT
        # ── 선제(교육용) 제동 이벤트 검출 (rising edge 1회, commit 00b70cd) ──
        # 위험할 만큼 늦지 않은 TTC(3.5~6.0s) + 유의미한 감속(≥1.0m/s^2) = '더 일찍
        # 반응하라' 교육 신호. (vLead = lead_v_kph 가 planner에서 전달돼야 함; 미전달 시
        # lead_v_kph=0 → 카운트 보류 = 보수적)
        if not self._prev_brake and lead_v_kph > 0.0 and closing > 0.1:
          if _LEAD_PROACTIVE_TTC_LO <= ttc_b < _LEAD_PROACTIVE_TTC_HI \
             and (-a_ego) >= _LEAD_PROACTIVE_DECEL:
            self._lead_proactive_count += 1
    self._prev_brake = brake_pressed

    # ── Phase 5: 토크 조향 파라미터 (nTune) ──
    if apply_lat and engaged and not blinker:
      if v_ego_kph >= _TQ_MIN_V_KPH and abs(steer_deg) >= _TQ_CURVE_DEG:
        # 커브 구간: 개입 비율 + 방향(운전자 토크 부호) 수집
        self._tq_curve_samples += 1
        if steer_pressed:
          self._tq_curve_overrides += 1
          if not self._tq_prev_pressed:  # 개입 이벤트당 1회만 방향 판정
            dir_sign = 0
            if steer_torque * steer_deg > 0:
              self._tq_under += 1        # 같은 방향으로 더 꺾음 → 조향력 부족
              dir_sign = 1
            elif steer_torque * steer_deg < 0:
              self._tq_inner += 1        # 반대 방향으로 풀어줌 → 안쪽 쏠림
              dir_sign = -1
            # ── 진동-우선 판정 (commit ffec86e 적응) ──────────────────────
            # 개입 방향의 '크기'가 아니라 '부호 반전 패턴'을 본다.
            # 방향 반전(zero-crossing) = 진동(latAccelFactor가 실제로는
            # 맞는데 진동으로 잘못 개입됨) / 방향 유지 = 정상상태 편향(실제 부족/과다).
            # (구버전은 override_ratio(크기)만 보고 계속 밀어붙여 발산 위험)
            if dir_sign != 0:
              if self._prev_tq_dir_sign != 0:
                if dir_sign != self._prev_tq_dir_sign:
                  self._tq_osc_count += 1     # 방향 반전 → 진동
                else:
                  self._tq_steady_count += 1  # 방향 유지 → 정상상태 편향
              self._prev_tq_dir_sign = dir_sign
      elif v_ego_kph >= 30.0 and abs(steer_deg) < _TQ_CURVE_DEG:
        # 완만/직선 구간: friction 학습용 미세 개입 비율
        self._tq_str_samples += 1
        self._prev_tq_dir_sign = 0  # 커브 종료 → 진동 부호 추적 초기화(구간 간 오검출 방지)
        if steer_pressed:
          self._tq_str_overrides += 1
        # 고속 직선 핑퐁(latAccelFactor 과다 개입) 검출: 운전자 개입 없이
        # 조향각이 0 근처에서 반복 반전 → 순수 토크 과다 신호 (commit ffec86e).
        elif v_ego_kph >= _TQ_PINGPONG_MIN_KPH:
          s_sign = 0
          if steer_deg > _TQ_PINGPONG_DEG:
            s_sign = 1
          elif steer_deg < -_TQ_PINGPONG_DEG:
            s_sign = -1
          if s_sign != 0:
            if self._prev_tq_straight_sign != 0 and s_sign != self._prev_tq_straight_sign:
              self._tq_straight_reversal += 1
            self._prev_tq_straight_sign = s_sign
    self._tq_prev_pressed = steer_pressed

    # ── steerRatio 수집: liveParameters(paramsd 칼만 추정)을 qualifying 구간에서 누적 ──
    # 전용 추정기 출력을 '정답'으로 모은다. (override 휴리스틱으로 유추하면 latAccelFactor와
    #  효과가 겹쳐 서로 밀어내며 진동하므로 사용하지 않음). 인게이지·일정속도·유효추정·
    #  비깜빡이 구간만 신뢰.
    if apply_lat and engaged and not blinker and steer_ratio_valid \
       and v_ego_kph >= _SR_MIN_V_KPH and _SR_MIN <= steer_ratio_live <= _SR_MAX:
      self._sr_sum += steer_ratio_live
      self._sr_n += 1

    # ── Phase 6: 비전 커브 감속 (VisionTurnController) ────────────────────
    # ENTERING(진입 감속)/TURNING(커브 중 가감속)/LEAVING(탈출 가속) 상태에서 페달
    # 개입을 밴드별로 누적한다. gas=그 구간 가감속이 과함(완화 신호) / brake=부족함
    # (강화 신호). exclude(깜빡이/극단가속)는 다른 종방향 Phase와 동일 기준 재사용.
    if apply_long and engaged and not exclude:
      if tvc_entering:
        ei = _band_index(tvc_max_pred_lat_acc, _TURN_ENTERING_BP)
        if gas_pressed:
          self._turn_entering_gas_acc[ei] += _DT
        elif brake_pressed:
          self._turn_entering_brake_acc[ei] += _DT
      elif tvc_turning:
        ti = _band_index(tvc_current_lat_acc, _TURN_TURNING_BP)
        if gas_pressed:
          self._turn_turning_gas_acc[ti] += _DT
        elif brake_pressed:
          self._turn_turning_brake_acc[ti] += _DT
      elif tvc_leaving:
        if gas_pressed:
          self._turn_leaving_gas_acc += _DT
        elif brake_pressed:
          self._turn_leaving_brake_acc += _DT

    # ── Phase 9: 수동주행 기준분포 로거 (원본 commit 10fa725) ──────────
    # 비인게이지(=사람이 직접 운전) 주행 중, 속도밴드별 사람의 가속/브레이크감속/무페달
    # 코스팅 자연감속/추종 차간시간을 통째 누적한다. 핵심은 무페달 코스팅 구간의 자연
    # 감속을 측정해 LongCoastBand(코스팅 데드밴드)를 직접 식별하는 것(역문제 없음).
    # (학습기는 CarrotLearningActive=1 일 때만 동작하므로, planner가 비인게이지 중에도
    #  매 프레임 update()를 호출해 주어야 한다.)
    if apply_long and (not engaged) and not gear_park and v_ego_kph >= _MANUAL_MIN_V_KPH:
      band = _speed_band(v_ego_kph)
      if gas_pressed:
        # 사람이 선택한 가속 (과도 가속은 제외; gas_val 기반 exclude는 수동 가속을
        #  통째로 날리므로 쓰지 않고 a_ego 임계만 사용)
        if a_ego <= 2.2:
          self._manual_gas_accel_sum[band] += a_ego
          self._manual_gas_n[band] += 1
      elif brake_pressed:
        # 사람이 선택한 감속 크기 (양수로 저장)
        self._manual_brake_decel_sum[band] += max(-a_ego, 0.0)
        self._manual_brake_n[band] += 1
      else:
        # 무페달 = 코스팅(자연 회생제동/엔진브레이크). 이 구간의 감속률을 측정.
        self._manual_coast_sec[band] += _DT
        if a_ego < 0.0:
          self._manual_coast_decel_sum[band] += -a_ego
          self._manual_coast_decel_n[band] += 1
      # 수동 추종 차간시간(time gap) 기준 분포 (선행차 존재 시)
      if 0.0 < lead_drel < _TFOLLOW_MAX_LEAD_DREL and v_ego_kph >= _TFOLLOW_MIN_V_KPH:
        v_ms = v_ego_kph / 3.6
        if v_ms > 1.0:
          self._manual_gap_sum += lead_drel / v_ms
          self._manual_gap_n += 1

    # 10초마다 주기 저장 (전원 차단 대비)
    self._frame += 1
    if self._frame % 200 == 0:
      self._save()

    # ── 주차(P단) 전환 트리거 ──
    if gear_park and not self._prev_gear_park and self._has_driven:
      self._on_parking()
      self._has_driven = False
    self._prev_gear_park = gear_park

  def apply_recommendations(self):
    """SSH 등에서 수동 호출용: 저장된 추천을 적용."""
    raw = self._params.get("CarrotLearningRecommend", encoding='utf8')
    if not raw:
      return
    try:
      recs = json.loads(raw)
    except Exception:
      return
    self._apply(recs)
    _remove(self._params, "CarrotLearningRecommend")
    self._params.put_bool("CarrotLearningPopupReady", False)

  # ── 내부 메서드 ──────────────────────────────────────────────────────
  def _on_parking(self):
    self._save()
    recs = self._calc_recommendations()
    if not recs:
      return
    self._params.put("CarrotLearningRecommend", json.dumps(recs, ensure_ascii=False))
    self._params.put("CarrotLearningPopupSource", "parking")
    self._params.put_bool("CarrotLearningPopupReady", True)
    if self._params.get_bool("CarrotLearningAutoApply"):
      self._apply(recs)
      _remove(self._params, "CarrotLearningRecommend")
      self._params.put_bool("CarrotLearningPopupReady", False)

  def _calc_recommendations(self):
    # ApplyLat/Long 토글: 미설정 시 기본 True
    raw_lat = self._params.get("CarrotTunerApplyLat", encoding='utf8')
    raw_long = self._params.get("CarrotTunerApplyLong", encoding='utf8')
    apply_lat = True if not raw_lat else raw_lat.strip() == "1"
    apply_long = True if not raw_long else raw_long.strip() == "1"

    result = {
      "가속 (Acceleration)": {},
      "조향 (Steering)": {},
      "곡선 (Curve)": {},
      "거리 (Following Distance)": {},
      "주행 (Driving)": {},
    }

    # ── Phase 2: OffsetTotal ──
    if apply_lat and self._steer_count >= _LATERAL_MIN_SAMPLES:
      avg_deg = self._steer_acc / self._steer_count
      if abs(avg_deg) >= _PATH_OFFSET_DEG_THRESHOLD:
        cur = _get_float(self._params, "OffsetTotal", 0.0)
        rec = float(np.clip(cur + avg_deg * _PATH_OFFSET_M_PER_DEG,
                            -_PATH_OFFSET_LIMIT, _PATH_OFFSET_LIMIT))
        if abs(rec - cur) >= 0.005:
          result["조향 (Steering)"]["OffsetTotal"] = {
            "current": round(cur, 3), "recommended": round(rec, 3),
            "band_kph": f"직진 편차 보정 (avg {avg_deg:.2f}deg)",
            "is_float": True,
          }

    # ── Phase 5: latAccelFactor / friction (nTune, 토크 제어 차량만) ──
    if apply_lat and self._is_torque_control():
      tq = _torque_read()

      # (a) latAccelFactor + friction 동시 조정 (commit e06a7dd Phase 0)
      #     latAccelFactor는 분모이므로 값↓ = 토크↑.
      #     언더스티어(조향력 부족) → factor↓ + friction↑ (응답 보강)
      #     안쪽 쏠림            → factor↑ + friction↓ (응답 완화)
      #
      #     ── 진동-우선 판정 (commit ffec86e 적응) ──────────────────────
      #     핵심 개선: 개입 '비율(크기)'만 보고 방향으로 계속 밀어붙이면, 진동이
      #     만든 개입도 '방향성 편향'으로 오인해 latAccelFactor를 계속 같은
      #     방향으로 밀어 발산할 수 있다(원본 KpV 무한상향 버그와 동일 패턴).
      #     개입 방향의 부호가 자주 반전(진동)되거나 고속 직선에서 조향각 자체가
      #     핑퐁이면, 방향성 판단(언더/이너) 대신 토크를 완화하는 쪽으로만 조정한다.
      cur_fr_curve = float(tq.get("friction", 0.08))
      fr_from_curve = None
      curve_osc_dominant = (self._tq_osc_count >= _TQ_OSC_MIN
                            and self._tq_osc_count > self._tq_steady_count)
      straight_pingpong = self._tq_straight_reversal >= _TQ_PINGPONG_MIN_COUNT
      if straight_pingpong or curve_osc_dominant:
        cur_laf = float(tq.get("latAccelFactor", 2.7))
        rec_laf = min(_LAF_MAX, round(cur_laf + _LAF_OSC_STEP, 3))
        fr_from_curve = max(_FRICTION_MIN, round(cur_fr_curve - _FRICTION_STEP, 3))
        reason = (f"고속 핑퐁/진동 감지 → 조향력 완화 "
                 f"(방향반전 {self._tq_osc_count}회, 직선반전 {self._tq_straight_reversal}회)")
        if rec_laf != cur_laf:
          result["조향 (Steering)"]["latAccelFactor"] = {
            "current": round(cur_laf, 3), "recommended": rec_laf,
            "band_kph": reason, "is_float": True, "ntune": "torque",
          }
      elif (self._tq_steady_count >= _TQ_STEADY_MIN and self._tq_osc_count < _TQ_OSC_MIN
            and self._tq_curve_samples >= _TQ_MIN_CURVE_SAMPLES
            and (self._tq_curve_overrides / self._tq_curve_samples) < _TQ_OVERRIDE_HI):
        # 정상상태 lag(방향 반전 없이 한쪽으로 꾸준히 개입): 원본은 이 경우 FF(Kf)
        # 강화를 우선시하고, FF가 이미 충분할 때만 Kp를 보조로 소폭 상향한다.
        # 이 포크의 FF 대응 파라미터는 friction 이므로, friction이 이미 '충분'
        # (_FRICTION_SUFFICIENT 이상)할 때만 latAccelFactor를 작은 스텝으로 보조
        # 조정한다. friction이 아직 부족하면 아래 비율 기반(≥30%) 분기가 언더/이너
        # 판정으로 latAccelFactor+friction을 함께 채우도록 그대로 둔다(우선순위 양보).
        if cur_fr_curve >= _FRICTION_SUFFICIENT and self._tq_under != self._tq_inner:
          cur_laf = float(tq.get("latAccelFactor", 2.7))
          if self._tq_under > self._tq_inner:
            rec_laf = max(_LAF_MIN, round(cur_laf - _LAF_STEADY_STEP, 3))
            reason = f"정상상태 조향력 부족 지속 (FF 충분, 보조 상향 {self._tq_under}회)"
          else:
            rec_laf = min(_LAF_MAX, round(cur_laf + _LAF_STEADY_STEP, 3))
            reason = f"정상상태 안쪽쏠림 지속 (FF 충분, 보조 완화 {self._tq_inner}회)"
          if rec_laf != cur_laf:
            result["조향 (Steering)"]["latAccelFactor"] = {
              "current": round(cur_laf, 3), "recommended": rec_laf,
              "band_kph": reason, "is_float": True, "ntune": "torque",
            }
      elif self._tq_curve_samples >= _TQ_MIN_CURVE_SAMPLES and self._tq_curve_overrides > 0:
        ratio = self._tq_curve_overrides / self._tq_curve_samples
        cur_laf = float(tq.get("latAccelFactor", 2.7))
        rec_laf, reason = cur_laf, ""
        if ratio >= _TQ_OVERRIDE_HI:
          if self._tq_under >= self._tq_inner * _TQ_DIR_DOMINANCE:
            # 낮을수록 같은 횡가속에 더 큰 토크 → 조향 강화
            rec_laf = max(_LAF_MIN, round(cur_laf - _LAF_STEP, 3))
            fr_from_curve = min(_FRICTION_MAX, round(cur_fr_curve + _FRICTION_STEP, 3))
            reason = f"커브 조향력 부족 (개입 {ratio*100:.0f}%, 더꺾음 {self._tq_under}회)"
          elif self._tq_inner >= self._tq_under * _TQ_DIR_DOMINANCE:
            rec_laf = min(_LAF_MAX, round(cur_laf + _LAF_STEP, 3))
            fr_from_curve = max(_FRICTION_MIN, round(cur_fr_curve - _FRICTION_STEP, 3))
            reason = f"커브 안쪽 쏠림 (개입 {ratio*100:.0f}%, 풀어줌 {self._tq_inner}회)"
        if rec_laf != cur_laf:
          result["조향 (Steering)"]["latAccelFactor"] = {
            "current": round(cur_laf, 3), "recommended": rec_laf,
            "band_kph": reason, "is_float": True, "ntune": "torque",
          }

      # (b) friction: 커브 동조정이 있으면 우선, 없으면 직선 미세 개입으로 학습
      if fr_from_curve is not None and fr_from_curve != cur_fr_curve:
        result["조향 (Steering)"]["friction"] = {
          "current": round(cur_fr_curve, 3), "recommended": fr_from_curve,
          "band_kph": "커브 방향 보정 동조정 (latAccelFactor 연동)",
          "is_float": True, "ntune": "torque",
        }
      elif fr_from_curve is None and self._tq_str_samples >= _TQ_MIN_STR_SAMPLES:
        r = self._tq_str_overrides / self._tq_str_samples
        cur_fr = float(tq.get("friction", 0.08))
        rec_fr, reason = cur_fr, ""
        if r >= _STR_OVERRIDE_HI:
          rec_fr = min(_FRICTION_MAX, round(cur_fr + _FRICTION_STEP, 3))
          reason = f"직선 미세 불감대 해소 (개입 {r*100:.0f}%)"
        elif r < _STR_OVERRIDE_LO and cur_fr > 0.02:
          rec_fr = max(_FRICTION_MIN, round(cur_fr - 0.005, 3))
          reason = "마찰보상 안정화 감쇄"
        if rec_fr != cur_fr:
          result["조향 (Steering)"]["friction"] = {
            "current": round(cur_fr, 3), "recommended": rec_fr,
            "band_kph": reason, "is_float": True, "ntune": "torque",
          }

      # (c) steerActuatorDelay (nTune common.json): 커브 개입 비율 기반
      # 개입이 잦음 = 조향 타이밍이 안 맞음 → 딜레이 낮춰 더 빠르게 반응
      if self._tq_curve_samples >= _TQ_MIN_CURVE_SAMPLES:
        ratio = self._tq_curve_overrides / max(self._tq_curve_samples, 1)
        common = _common_read()
        cur_sad = float(common.get("steerActuatorDelay", 0.1))
        rec_sad, reason = cur_sad, ""
        if ratio >= _SAD_OVERRIDE_HI:
          rec_sad = max(_SAD_MIN, round(cur_sad - _SAD_STEP, 3))
          reason = f"커브 조향 지연 (개입 {ratio*100:.0f}% → 반응 빠르게)"
        elif ratio < _SAD_OVERRIDE_LO:
          rec_sad = min(_SAD_MAX, round(cur_sad + _SAD_STEP, 3))
          reason = f"커브 안정 (개입 {ratio*100:.0f}% → 지연 소폭 상향)"
        if rec_sad != cur_sad:
          result["조향 (Steering)"]["steerActuatorDelay"] = {
            "current": round(cur_sad, 3), "recommended": rec_sad,
            "band_kph": reason, "is_float": True, "ntune": "common",
          }

    # ── steerRatio (nTune common.json): liveParameters 추정 평균으로 보정 ──
    # common.json은 제어방식(torque/indi) 무관이라 torque 게이트 밖에서 처리한다.
    # 전용 추정기(paramsd) 출력의 평균을 nTune 고정값으로 굳힌다. useLiveSteerRatio=1
    # 이면 컨트롤러는 라이브 값을 쓰므로 이 고정값은 '동결 백업'으로 의미.
    if apply_lat and self._sr_n >= _SR_MIN_SAMPLES:
      mean_sr = self._sr_sum / self._sr_n
      common = _common_read()
      cur_sr = float(common.get("steerRatio", _SR_DEFAULT))
      if abs(mean_sr - cur_sr) >= _SR_DEADBAND:
        # 세션당 변동 상한으로 급변 방지 후 nTune 유효범위 클램프
        rec_sr = cur_sr + float(np.clip(mean_sr - cur_sr, -_SR_MAX_DELTA, _SR_MAX_DELTA))
        rec_sr = round(float(np.clip(rec_sr, _SR_MIN, _SR_MAX)), 2)
        if abs(rec_sr - cur_sr) >= 0.01:
          result["조향 (Steering)"]["steerRatio"] = {
            "current": round(cur_sr, 2), "recommended": rec_sr,
            "band_kph": f"라이브 추정 평균 {mean_sr:.2f} (n={self._sr_n}) → steerRatio 보정",
            "is_float": True, "ntune": "common",
          }

    # ── Phase 4: TFollowGap ──
    if apply_long:
      # GAP1~4를 각각 하나의 단계별 추종시간으로 학습한다.
      for i in range(4):
        key = _TFOLLOW_KEYS[i]
        cur = _get_int(self._params, key, _TFOLLOW_DEFAULTS[i])
        rec, reason = cur, ""
        if self._tfollow_gas_acc[i] >= _TFOLLOW_GAS_THRESHOLD_SEC:
          target = int(self._tfollow_min_gap[i] * 100)
          diff = cur - target
          step = int(np.clip(diff * 0.5, _TFOLLOW_NARROW_STEP, 25)) if diff > 10 else _TFOLLOW_NARROW_STEP
          rec = max(_TFOLLOW_MIN, cur - step)
          reason = f"too wide ({self._tfollow_gas_acc[i]:.0f}s gas)"
        elif self._tfollow_brake_acc[i] >= _TFOLLOW_BRAKE_THRESHOLD_SEC:
          rec = min(_TFOLLOW_MAX, cur + _TFOLLOW_WIDEN_STEP)
          reason = f"too short ({self._tfollow_brake_acc[i]:.0f}s brake)"
        if rec != cur:
          rec = cur + int(np.clip(rec - cur, -_TFOLLOW_MAX_DELTA, _TFOLLOW_MAX_DELTA))
        if rec != cur:
          result["거리 (Following Distance)"][key] = {
            "current": cur, "recommended": rec,
            "band_kph": f"GAP{i+1} >=40km/h ({reason})",
          }

      # ── 종방향 응답성: longActuatorDelay / longKf (longcontrol 라이브 반영) ──
      # 추종 중 '늦은' 브레이크 누적 많음 = 제동 반응 느림 → 딜레이↓(빠르게) + kf↑(응답 보강)
      # 추종 중 가속 누적 많음           = 가속 굼뜸     → kf↑
      # 선제 교육 제동 반복             = '더 일찍 반응하라' → kf 약하게↑ (commit 00b70cd 적응)
      #
      # (commit 1e95637) 응답 보강(longKf↑/delay↓)은 '늦은(긴급)' 제동만 집계한다.
      # 여유 감속(높은 TTC)은 위 Phase4 TFollowGap '넓히기'에서 _tfollow_brake_acc로
      # 이미 반영되며, 여기 보강 신호로는 쓰지 않아 onset jerk 완화로 생긴 정상 감속이
      # 다시 longKf를 끌어올려 제동을 날카롭게 만드는 악순환을 차단한다.
      brake_total = sum(self._tfollow_brake_late)
      gas_total = sum(self._tfollow_gas_acc)

      cur_delay = _get_float(self._params, _LONG_ACT_DELAY_KEY, _LONG_DELAY_DEFAULT)
      cur_kf = _get_float(self._params, _LONG_KF_KEY, _LONG_KF_DEFAULT)
      rec_delay, rec_kf = cur_delay, cur_kf
      delay_reason, kf_reason = "", ""

      if brake_total >= _LONG_BRAKE_THRESHOLD_SEC:
        rec_delay = round(max(_LONG_DELAY_MIN, cur_delay - _LONG_DELAY_STEP), 3)
        rec_kf = round(min(_LONG_KF_MAX, cur_kf + _LONG_KF_STEP), 3)
        delay_reason = f"늦은 제동 반응 느림 ({brake_total:.0f}s late-brake → 지연 단축)"
        kf_reason = f"늦은 제동 응답 보강 ({brake_total:.0f}s late-brake)"
      elif gas_total >= _LONG_GAS_THRESHOLD_SEC:
        rec_kf = round(min(_LONG_KF_MAX, cur_kf + _LONG_KF_STEP), 3)
        kf_reason = f"가속 응답 보강 ({gas_total:.0f}s gas)"
      elif self._lead_proactive_count >= _LEAD_PROACTIVE_MIN_COUNT:
        # 선제(교육용) 제동: 강한 늦은-제동 경로(brake_total)엔 못 미쳤지만 선행차에
        # 반복적으로 미리 제동 = '더 일찍 반응하라' → kf 약하게 상향(작은 step).
        # delay 는 보수적으로 유지(과도한 빠른 반응 방지). 별도·완화 경로로 분리해
        # 정상 여유 제동의 과도 누적을 막는다(원본 commit의 +5 small-step 설계 대응).
        rec_kf = round(min(_LONG_KF_MAX, cur_kf + _LONG_KF_PROACTIVE_STEP), 3)
        kf_reason = f"선제 교육 제동 ({self._lead_proactive_count}회 → 응답 소폭 보강)"

      if rec_delay != cur_delay:
        result["가속 (Acceleration)"][_LONG_ACT_DELAY_KEY] = {
          "current": round(cur_delay, 3), "recommended": rec_delay,
          "band_kph": delay_reason, "is_float": True, "long_param": True,
        }
      if rec_kf != cur_kf:
        result["가속 (Acceleration)"][_LONG_KF_KEY] = {
          "current": round(cur_kf, 3), "recommended": rec_kf,
          "band_kph": kf_reason, "is_float": True, "long_param": True,
        }

      # ── Phase 9: 수동주행 코스팅 측정 → LongCoastBand 추천 (원본 commit 10fa725) ──
      # 사람이 무페달로 코스팅할 때의 자연 감속(회생제동/엔진브레이크)을 측정하여,
      # 종방향 코스팅 데드밴드를 차의 코스트 권한 일부 범위(0.15~0.40 m/s²) 내에서 보정.
      coast_n = sum(self._manual_coast_decel_n)
      coast_sec = sum(self._manual_coast_sec)
      if coast_n >= _MANUAL_COAST_MIN_N and coast_sec >= _MANUAL_COAST_MIN_SEC:
        mean_coast_decel = sum(self._manual_coast_decel_sum) / coast_n   # m/s² (양수)
        # 데드밴드(m/s²) = 측정 코스팅 감속 × 게인, 안전범위 클램프 후 cm/s²(×100) 정수화
        rec_band = int(np.clip(round(np.clip(mean_coast_decel * _MANUAL_COAST_GAIN, 0.15, 0.40) * 100),
                               0, _LONG_COAST_BAND_MAX))
        cur_band = _get_int(self._params, "LongCoastBand", 0)
        # 5(=0.05 m/s²) 이상 차이날 때만 추천 (미세 변동 잡음 억제)
        if abs(rec_band - cur_band) >= 5:
          result["주행 (Driving)"]["LongCoastBand"] = {
            "current": cur_band, "recommended": rec_band,
            "band_kph": f"수동 코스팅 감속 {mean_coast_decel:.2f}m/s^2 측정 → 코스팅(회생제동) 데드밴드 보정",
          }

    # ── Phase 6: 비전 커브 감속 (Entering/Turning/Leaving) ───────────────
    if apply_long:
      # (a) Entering: 진입 감속 완화(gas)/강화(brake)
      for i in range(2):
        key = _TURN_ENTERING_KEYS[i]
        cur = _get_int(self._params, key, _TURN_ENTERING_DEFAULTS[i])
        rec, reason = cur, ""
        if self._turn_entering_gas_acc[i] >= _TURN_GAS_THRESHOLD_SEC:
          rec = min(_TURN_ENTERING_MAX, cur + _TURN_ACC_STEP)   # 0 방향 = 덜 감속
          reason = f"진입감속 과다 ({self._turn_entering_gas_acc[i]:.0f}s gas)"
        elif self._turn_entering_brake_acc[i] >= _TURN_BRAKE_THRESHOLD_SEC:
          rec = max(_TURN_ENTERING_MIN, cur - _TURN_ACC_STEP)   # 더 감속
          reason = f"진입감속 부족 ({self._turn_entering_brake_acc[i]:.0f}s brake)"
        if rec != cur:
          rec = cur + int(np.clip(rec - cur, -_TURN_MAX_DELTA, _TURN_MAX_DELTA))
        if rec != cur:
          result["곡선 (Curve)"][key] = {
            "current": cur, "recommended": rec,
            "band_kph": f"진입 예측lat{_TURN_ENTERING_BP[i]:.1f}~ ({reason})",
          }

      # (b) Turning: 커브 중 가감속 완화(gas)/강화(brake)
      for i in range(5):
        key = _TURN_TURNING_KEYS[i]
        cur = _get_int(self._params, key, _TURN_TURNING_DEFAULTS[i])
        rec, reason = cur, ""
        if self._turn_turning_gas_acc[i] >= _TURN_GAS_THRESHOLD_SEC:
          rec = min(_TURN_TURNING_MAX, cur + _TURN_ACC_STEP)
          reason = f"커브 감속 과다 ({self._turn_turning_gas_acc[i]:.0f}s gas)"
        elif self._turn_turning_brake_acc[i] >= _TURN_BRAKE_THRESHOLD_SEC:
          rec = max(_TURN_TURNING_MIN, cur - _TURN_ACC_STEP)
          reason = f"커브 감속 부족 ({self._turn_turning_brake_acc[i]:.0f}s brake)"
        if rec != cur:
          rec = cur + int(np.clip(rec - cur, -_TURN_MAX_DELTA, _TURN_MAX_DELTA))
        if rec != cur:
          result["곡선 (Curve)"][key] = {
            "current": cur, "recommended": rec,
            "band_kph": f"현재lat{_TURN_TURNING_BP[i]:.1f}~ ({reason})",
          }

      # (c) Leaving: 탈출가속 강화(gas)/완화(brake)
      cur_lv = _get_int(self._params, _TURN_LEAVING_KEY, _TURN_LEAVING_DEFAULT)
      rec_lv, reason_lv = cur_lv, ""
      if self._turn_leaving_gas_acc >= _TURN_GAS_THRESHOLD_SEC:
        rec_lv = min(_TURN_LEAVING_MAX, cur_lv + _TURN_ACC_STEP)
        reason_lv = f"탈출가속 부족 ({self._turn_leaving_gas_acc:.0f}s gas)"
      elif self._turn_leaving_brake_acc >= _TURN_BRAKE_THRESHOLD_SEC:
        rec_lv = max(_TURN_LEAVING_MIN, cur_lv - _TURN_ACC_STEP)
        reason_lv = f"탈출가속 과다 ({self._turn_leaving_brake_acc:.0f}s brake)"
      if rec_lv != cur_lv:
        rec_lv = cur_lv + int(np.clip(rec_lv - cur_lv, -_TURN_MAX_DELTA, _TURN_MAX_DELTA))
      if rec_lv != cur_lv:
        result["곡선 (Curve)"][_TURN_LEAVING_KEY] = {
          "current": cur_lv, "recommended": rec_lv,
          "band_kph": reason_lv,
        }

    return {k: v for k, v in result.items() if v}

  def _is_torque_control(self):
    """이 포크는 LateralControl 파라미터가 비어있거나 TORQUE일 때 토크 제어 (CommunityPanel 기본값)"""
    raw = self._params.get("LateralControl", encoding='utf8')
    return (not raw) or raw.strip().upper() == "TORQUE"

  def _apply(self, recs):
    applied = {}
    for group, items in recs.items():
      g = {}
      for key, info in items.items():
        if info.get("ntune") == "torque":
          # Params 직접 수정 → latcontrol_torque 가 0.1초마다 라이브 반영
          _torque_write(key, float(info["recommended"]))
        elif info.get("ntune") == "common":
          # Params 직접 수정 → live_tune 이 1초 주기로 라이브 반영
          # (steerActuatorDelay / steerRatio 모두 이 경로)
          _common_write(key, float(info["recommended"]))
        elif info.get("long_param"):
          # 종방향 응답성: Params에 float 저장 → longcontrol.py가 1초마다 라이브 읽기
          self._params.put(key, f"{float(info['recommended']):.3f}")
        elif info.get("is_float"):
          self._params.put(key, f"{float(info['recommended']):.3f}")
        else:
          self._params.put(key, str(int(info["recommended"])))
        g[key] = info
      if g:
        applied[group] = g

    if applied:
      now = datetime.datetime.now()
      entry = {
        "id": now.strftime("%Y%m%d%H%M%S%f"),
        "timestamp": now.strftime("%Y-%m-%d %H:%M:%S"),
        "changes": applied,
      }
      hist_raw = self._params.get("CarrotLearningHistory", encoding='utf8')
      hist = []
      if hist_raw:
        try:
          hist = json.loads(hist_raw)
          if not isinstance(hist, list):
            hist = []
        except Exception:
          hist = []
      hist.insert(0, entry)
      self._params.put("CarrotLearningHistory", json.dumps(hist[:50], ensure_ascii=False))

    # ── 적용된 Phase의 누적치만 선택적으로 리셋 ─────────────────────────
    # 적용되지 않은 Phase(특히 느리게 쌓이는 조향 Phase2/5)는 데이터를 보존해
    # 무관한 longitudinal 적용 때문에 학습 진척이 초기화되지 않도록 한다.
    applied_phases = set()
    for group in applied:
      for key in applied[group]:
        ph = _KEY_RESET_PHASE.get(key)
        if ph is not None:
          applied_phases.add(ph)
    phase_reset = {
      2: self._reset_phase2,
      4: self._reset_phase4,
      5: self._reset_phase5,
      6: self._reset_phase6,
      9: self._reset_phase9,
    }
    for ph in applied_phases:
      phase_reset[ph]()

    # 보존된 Phase 데이터가 재부팅에도 살아남도록 즉시 저장(remove 대신 _save).
    self._save()

  def _reset_phase2(self):
    """직진 편차 (OffsetTotal)"""
    self._steer_acc = 0.0
    self._steer_count = 0

  def _reset_phase4(self):
    """추종거리 (TFollowGap) + 종방향 응답성(longActuatorDelay/longKf 파생 카운터)"""
    self._tfollow_gas_acc = [0.0] * 4
    self._tfollow_brake_acc = [0.0] * 4
    self._tfollow_brake_late = [0.0] * 4   # 늦은(긴급) 제동 누적도 함께 리셋 (longKf 파생, commit 1e95637)
    self._tfollow_min_gap = [999.0] * 4
    self._lead_proactive_count = 0   # 선제 제동 카운터도 함께 리셋 (longKf 파생)
    self._prev_brake = False

  def _reset_phase5(self):
    """조향 (latAccelFactor/friction/steerActuatorDelay/steerRatio) — 방향·추정 카운터 포함"""
    self._tq_curve_samples = 0
    self._tq_curve_overrides = 0
    self._tq_under = 0
    self._tq_inner = 0
    self._tq_str_samples = 0
    self._tq_str_overrides = 0
    self._tq_prev_pressed = False
    # 진동/핑퐁 카운터도 함께 리셋 (commit ffec86e 적응)
    self._tq_osc_count = 0
    self._tq_steady_count = 0
    self._tq_straight_reversal = 0
    self._prev_tq_dir_sign = 0
    self._prev_tq_straight_sign = 0
    # steerRatio 누적도 함께 리셋 (Phase 5 조향 그룹 귀속)
    self._sr_sum = 0.0
    self._sr_n = 0

  def _reset_phase6(self):
    """비전 커브 감속 (TurnEnteringDecel/TurnTurningAcc/TurnLeavingAcc)"""
    self._turn_entering_gas_acc = [0.0] * 2
    self._turn_entering_brake_acc = [0.0] * 2
    self._turn_turning_gas_acc = [0.0] * 5
    self._turn_turning_brake_acc = [0.0] * 5
    self._turn_leaving_gas_acc = 0.0
    self._turn_leaving_brake_acc = 0.0

  def _reset_phase9(self):
    """수동주행 기준분포 로거 (LongCoastBand)"""
    self._manual_coast_sec = [0.0] * _NUM_BANDS
    self._manual_coast_decel_sum = [0.0] * _NUM_BANDS
    self._manual_coast_decel_n = [0] * _NUM_BANDS
    self._manual_gas_accel_sum = [0.0] * _NUM_BANDS
    self._manual_gas_n = [0] * _NUM_BANDS
    self._manual_brake_decel_sum = [0.0] * _NUM_BANDS
    self._manual_brake_n = [0] * _NUM_BANDS
    self._manual_gap_sum = 0.0
    self._manual_gap_n = 0

  def _reset_all_phases(self):
    self._reset_phase2()
    self._reset_phase4()
    self._reset_phase5()
    self._reset_phase6()
    self._reset_phase9()

  def _clear(self):
    self._reset_all_phases()
    _remove(self._params, "CarrotLearningData")
    _remove(self._params, "CarrotLearningRecommend")

  def _load(self):
    raw = self._params.get("CarrotLearningData", encoding='utf8')
    if not raw:
      return
    try:
      d = json.loads(raw)
      self._steer_acc = float(d.get("steer_acc", 0.0))
      self._steer_count = int(d.get("steer_count", 0))
      tg = d.get("tfollow_gas_acc", [])
      if len(tg) == 4:
        self._tfollow_gas_acc = [float(x) for x in tg]
      tb = d.get("tfollow_brake_acc", [])
      if len(tb) == 4:
        self._tfollow_brake_acc = [float(x) for x in tb]
      tbl = d.get("tfollow_brake_late", [])
      if len(tbl) == 4:
        self._tfollow_brake_late = [float(x) for x in tbl]
      tm = d.get("tfollow_min_gap", [])
      if len(tm) == 4:
        self._tfollow_min_gap = [float(x) for x in tm]
      self._lead_proactive_count = int(d.get("lead_proactive_count", 0))
      # Phase 9 (밴드별 리스트는 길이 검증 후 복원)
      p9 = d.get("phase9", {})
      def _band_list(key, cast, default):
        v = p9.get(key)
        if isinstance(v, list) and len(v) == _NUM_BANDS:
          return [cast(x) for x in v]
        return [default] * _NUM_BANDS
      self._manual_coast_sec = _band_list("manual_coast_sec", float, 0.0)
      self._manual_coast_decel_sum = _band_list("manual_coast_decel_sum", float, 0.0)
      self._manual_coast_decel_n = _band_list("manual_coast_decel_n", int, 0)
      self._manual_gas_accel_sum = _band_list("manual_gas_accel_sum", float, 0.0)
      self._manual_gas_n = _band_list("manual_gas_n", int, 0)
      self._manual_brake_decel_sum = _band_list("manual_brake_decel_sum", float, 0.0)
      self._manual_brake_n = _band_list("manual_brake_n", int, 0)
      self._manual_gap_sum = float(p9.get("manual_gap_sum", 0.0))
      self._manual_gap_n = int(p9.get("manual_gap_n", 0))
      tq = d.get("tq", {})
      self._tq_curve_samples = int(tq.get("curve_samples", 0))
      self._tq_curve_overrides = int(tq.get("curve_overrides", 0))
      self._tq_under = int(tq.get("under", 0))
      self._tq_inner = int(tq.get("inner", 0))
      self._tq_str_samples = int(tq.get("str_samples", 0))
      self._tq_str_overrides = int(tq.get("str_overrides", 0))
      self._tq_osc_count = int(tq.get("osc_count", 0))
      self._tq_steady_count = int(tq.get("steady_count", 0))
      self._tq_straight_reversal = int(tq.get("straight_reversal", 0))
      self._sr_sum = float(tq.get("sr_sum", 0.0))
      self._sr_n = int(tq.get("sr_n", 0))
      # Phase 6 (비전 커브 감속, 길이 검증 후 복원)
      p6 = d.get("phase6", {})
      def _turn_list(key, n, default):
        v = p6.get(key)
        if isinstance(v, list) and len(v) == n:
          return [float(x) for x in v]
        return [default] * n
      self._turn_entering_gas_acc = _turn_list("turn_entering_gas_acc", 2, 0.0)
      self._turn_entering_brake_acc = _turn_list("turn_entering_brake_acc", 2, 0.0)
      self._turn_turning_gas_acc = _turn_list("turn_turning_gas_acc", 5, 0.0)
      self._turn_turning_brake_acc = _turn_list("turn_turning_brake_acc", 5, 0.0)
      self._turn_leaving_gas_acc = float(p6.get("turn_leaving_gas_acc", 0.0))
      self._turn_leaving_brake_acc = float(p6.get("turn_leaving_brake_acc", 0.0))
    except Exception:
      pass  # 데이터 손상 시 기본값 유지

  def _save(self):
    data = {
      "steer_acc": self._steer_acc,
      "steer_count": self._steer_count,
      "tfollow_gas_acc": self._tfollow_gas_acc,
      "tfollow_brake_acc": self._tfollow_brake_acc,
      "tfollow_brake_late": self._tfollow_brake_late,
      "tfollow_min_gap": self._tfollow_min_gap,
      "lead_proactive_count": self._lead_proactive_count,
      "phase6": {
        "turn_entering_gas_acc": self._turn_entering_gas_acc,
        "turn_entering_brake_acc": self._turn_entering_brake_acc,
        "turn_turning_gas_acc": self._turn_turning_gas_acc,
        "turn_turning_brake_acc": self._turn_turning_brake_acc,
        "turn_leaving_gas_acc": self._turn_leaving_gas_acc,
        "turn_leaving_brake_acc": self._turn_leaving_brake_acc,
      },
      "phase9": {
        "manual_coast_sec": self._manual_coast_sec,
        "manual_coast_decel_sum": self._manual_coast_decel_sum,
        "manual_coast_decel_n": self._manual_coast_decel_n,
        "manual_gas_accel_sum": self._manual_gas_accel_sum,
        "manual_gas_n": self._manual_gas_n,
        "manual_brake_decel_sum": self._manual_brake_decel_sum,
        "manual_brake_n": self._manual_brake_n,
        "manual_gap_sum": self._manual_gap_sum,
        "manual_gap_n": self._manual_gap_n,
      },
      "tq": {
        "curve_samples": self._tq_curve_samples,
        "curve_overrides": self._tq_curve_overrides,
        "under": self._tq_under,
        "inner": self._tq_inner,
        "str_samples": self._tq_str_samples,
        "str_overrides": self._tq_str_overrides,
        "osc_count": self._tq_osc_count,
        "steady_count": self._tq_steady_count,
        "straight_reversal": self._tq_straight_reversal,
        "sr_sum": self._sr_sum,
        "sr_n": self._sr_n,
      },
    }
    self._params.put("CarrotLearningData", json.dumps(data))

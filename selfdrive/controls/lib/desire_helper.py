import time
import numpy as np
from cereal import log
from common.realtime import DT_MDL
from common.conversions import Conversions as CV
from common.params import Params
from selfdrive.controls.lib.carrot_navi_atc import CarrotNaviAtc, AtcForkLaneChangeController

AUTO_LCA_START_TIME = 1.0

LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 50 * CV.KPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeRight,
  },
}


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.LateralPlan.Desire.none

    self.params = Params()
    self.lane_change_enabled = self.params.get_bool('LaneChangeEnabled')
    self.auto_lane_change_enabled = self.params.get_bool('AutoLaneChangeEnabled')
    self.lane_change_speed_min = LANE_CHANGE_SPEED_MIN
    self.last_params_update = 0.0

    self.auto_lane_change_timer = 0.0
    self.auto_lane_change_timer_setting = 0
    self.prev_torque_applied = False

    # apilot 참고: ATC 회전신호가 순간적으로 끊겨도(steering_request 는 distance/
    # kind 조건에서 벗어나는 즉시 0을 줌) 모델이 아직 회전 중이라고 보면(desireState
    # 의 turnLeft/turnRight 확률) 방향을 래치해두고 부드럽게 페이드아웃한다.
    self.turn_direction_latched = 0
    self.turn_ll_prob = 1.0
    self.turn_disable_count = 0

    # Lane Change Timer (AutoLaneChangeTimer) 관련
    self.lane_change_wait_timer = 0.0
    self.prev_lane_change = False
    self.road_edge = False
    self.carrot_atc = CarrotNaviAtc()
    self.empty_atc_state = self.carrot_atc.empty_state()
    self.atc_fork_controller = AtcForkLaneChangeController()
    self.carrot_atc_mode = 0
    self.atc_state = self.empty_atc_state
    self.atc_turn_direction = 0
    self.atc_driver_cancel = True

  @staticmethod
  def _road_edge_detected(model_data, direction):
    if model_data is None or direction not in (-1, 1):
      return True

    edge_index = 0 if direction < 0 else 1
    current_lane_index = 1 if direction < 0 else 2
    outer_lane_index = 0 if direction < 0 else 3

    try:
      desired_edge = model_data.roadEdges[edge_index]
      current_lane = model_data.laneLines[current_lane_index]
      left_lane = model_data.laneLines[1]
      right_lane = model_data.laneLines[2]
      lines = (desired_edge, current_lane, left_lane, right_lane)
      if any(len(line.x) < 2 or len(line.y) < 2 or len(line.x) != len(line.y) for line in lines):
        return True
    except (AttributeError, IndexError, TypeError):
      return True

    def near_median_gap(line_a, line_b):
      start_x = max(float(line_a.x[0]), float(line_b.x[0]), 0.0)
      end_x = min(float(line_a.x[-1]), float(line_b.x[-1]), 30.0)
      if not np.isfinite(start_x) or not np.isfinite(end_x) or end_x <= start_x:
        return None
      sample_x = np.linspace(start_x, end_x, num=8)
      a_y = np.interp(sample_x, line_a.x, line_a.y)
      b_y = np.interp(sample_x, line_b.x, line_b.y)
      gap = np.abs(a_y - b_y)
      return float(np.median(gap)) if np.all(np.isfinite(gap)) else None

    edge_gap = near_median_gap(desired_edge, current_lane)
    measured_lane_width = near_median_gap(left_lane, right_lane)
    if edge_gap is None or measured_lane_width is None:
      return True
    lane_width = float(np.clip(measured_lane_width, 2.5, 4.5))

    try:
      current_lane_prob = float(model_data.laneLineProbs[current_lane_index])
      outer_lane_prob = float(model_data.laneLineProbs[outer_lane_index])
    except (AttributeError, IndexError, TypeError, ValueError):
      current_lane_prob = 1.0
      outer_lane_prob = 1.0

    try:
      edge_confidence = float(np.clip(1.0 - model_data.roadEdgeStds[edge_index], 0.0, 1.0))
    except (AttributeError, IndexError, TypeError, ValueError):
      edge_confidence = 1.0

    # C2's active detector compares the edge gap with the measured lane width,
    # instead of allowing a change when any single far-path point exceeds 3 m.
    close_physical_edge = edge_gap < lane_width * 0.7
    no_outer_lane = current_lane_prob > 0.2 and outer_lane_prob <= 0.2
    c2_single_lane_edge = no_outer_lane and edge_gap * 1.2 < lane_width

    # On right-driving roads the left boundary can be the centre line even
    # when the model places the physical road edge beyond the oncoming lane.
    # Use a strict probability fallback only on the left to avoid crossing it.
    left_centre_line = (direction < 0 and current_lane_prob > 0.5 and
                        outer_lane_prob < 0.1)
    uncertain_single_lane_edge = edge_confidence < 0.35 and no_outer_lane

    return close_physical_edge or c2_single_lane_edge or left_centre_line or uncertain_single_lane_edge

  def _update_atc_turn_completion(self, model_data, carstate, turn_active):
    """Stop re-requesting a turn after the vehicle passes the turn apex."""
    completed = False
    if turn_active and model_data is not None:
      try:
        orientation_rate = abs(model_data.orientationRate.z[5])
        orientation_rate_future = abs(model_data.orientationRate.z[15])
        completed = abs(carstate.steeringAngleDeg) > 80 and orientation_rate_future < orientation_rate
      except (IndexError, AttributeError, TypeError):
        completed = False

    if completed:
      self.turn_disable_count = int(10.0 / DT_MDL)
    else:
      self.turn_disable_count = max(0, self.turn_disable_count - 1)

  def update(self, carstate, lateral_active, lane_change_prob, model_data=None):
    t = time.monotonic()
    if t - self.last_params_update > 1.0:
      self.lane_change_enabled = self.params.get_bool('LaneChangeEnabled')
      self.auto_lane_change_enabled = self.params.get_bool('AutoLaneChangeEnabled')
      try:
        self.carrot_atc_mode = int(self.params.get('CarrotAutoTurnControl', encoding='utf8') or '0')
      except (TypeError, ValueError):
        self.carrot_atc_mode = 0
      try:
        auto_lc_speed_kph = int(self.params.get('AutoLaneChangeSpeed', encoding='utf8') or '50')
      except (TypeError, ValueError):
        auto_lc_speed_kph = 50
      self.lane_change_speed_min = auto_lc_speed_kph * CV.KPH_TO_MS
      try:
        self.auto_lane_change_timer_setting = int(
          self.params.get("AutoLaneChangeTimer", encoding="utf8") or "0")
      except (TypeError, ValueError):
        self.auto_lane_change_timer_setting = 0
      self.last_params_update = t

    # AutoLaneChangeTimer 파라미터 읽기 및 대기 시간 계산
    lane_change_set_timer = self.auto_lane_change_timer_setting
    lane_change_auto_timer = 0.0 if lane_change_set_timer == 0 else \
                             0.1 if lane_change_set_timer == 1 else \
                             0.5 if lane_change_set_timer == 2 else \
                             1.0 if lane_change_set_timer == 3 else \
                             1.5 if lane_change_set_timer == 4 else 2.0

    v_ego = carstate.vEgo
    atc_steering = self.carrot_atc_mode in (1, 2) and lateral_active and not carstate.brakePressed
    # Avoid parsing the shared navigation JSON in lateral planning when ATC
    # steering is disabled. Longitudinal navigation uses its own reader.
    atc_state = self.carrot_atc.update() if atc_steering else self.empty_atc_state
    atc_direction = atc_state['direction'] if atc_steering else 0
    opposite_torque = carstate.steeringPressed and ((atc_direction < 0 and carstate.steeringTorque < 0) or
                                                    (atc_direction > 0 and carstate.steeringTorque > 0))
    conflicting_blinker = (atc_direction < 0 and carstate.rightBlinker) or \
                          (atc_direction > 0 and carstate.leftBlinker)
    if opposite_torque or conflicting_blinker:
      atc_direction = 0
    # AutoLaneChangeEnabled 의 자동타이머, 그리고 방향이 맞는 핸들토크 둘 다 —
    # ATC가 이미 같은 방향의 실제 회전(교차로 turn/uturn, 분기가 아님)을 보고 있을
    # 때는 차선변경(laneChangeStarting)으로 새치기하지 못하게 막기 위한 플래그.
    # 반대방향 토크(opposite_torque)는 그대로 취소 신호로 살아있다.
    atc_turn_matches_blinker = (atc_steering and atc_state.get('kind') in ('turn', 'uturn') and
                               ((atc_direction < 0 and carstate.leftBlinker) or
                                (atc_direction > 0 and carstate.rightBlinker)))
    # Latest carrot-style exit gating adapted to this fork's simpler model:
    # right exits only, arm at the last lane, then request one change when the
    # exit lane opens. Keep the request alive while BSD blocks it.
    right_lane_open = False
    if atc_steering:
      right_lane_open = not self._road_edge_detected(model_data, 1)
      atc_fork_direction = self.atc_fork_controller.update(
        atc_state, v_ego, right_lane_open,
        driver_cancel=opposite_torque or conflicting_blinker,
        lane_change_started=self.lane_change_state == LaneChangeState.laneChangeStarting,
        lane_change_finished=self.lane_change_state == LaneChangeState.laneChangeFinishing,
      )
    else:
      self.atc_fork_controller.reset()
      atc_fork_direction = 0
    left_blinker = carstate.leftBlinker
    right_blinker = carstate.rightBlinker or atc_fork_direction > 0
    one_blinker = left_blinker != right_blinker
    below_lane_change_speed = v_ego < self.lane_change_speed_min

    # Driver lane changes retain the original road-edge gate. ATC only raises
    # its virtual blinker after the controller has already observed an open lane.
    direction = -1 if left_blinker else 1 if right_blinker else 0
    if direction == 1 and atc_steering:
      self.road_edge = not right_lane_open
    else:
      self.road_edge = self._road_edge_detected(model_data, direction) if direction else False

    if (not lateral_active) or (self.lane_change_timer > LANE_CHANGE_TIME_MAX) or \
       (not one_blinker) or (not self.lane_change_enabled):
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.prev_lane_change = False
    else:
      manual_or_auto_torque = (carstate.steeringPressed and
                       ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                        (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))) or \
                        (self.auto_lane_change_enabled and
                        (AUTO_LCA_START_TIME + 0.25) > self.auto_lane_change_timer > AUTO_LCA_START_TIME)
      # ATC가 같은 방향의 실제 회전을 인식 중이면, 손을 얹어 생기는 미세한 동일방향
      # 토크나 자동타이머 둘 다 "차선변경 시작 의사"로 오인하지 않는다. 반대방향
      # 토크(opposite_torque, atc_direction 계산부에서 이미 처리됨)는 여전히 취소로
      # 작동한다.
      torque_applied = manual_or_auto_torque and not atc_turn_matches_blinker

      blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                            (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

      if self.lane_change_state == LaneChangeState.off and one_blinker and \
         not self.prev_one_blinker and not below_lane_change_speed and not carstate.brakePressed:
        if left_blinker:
          self.lane_change_direction = LaneChangeDirection.left
        elif right_blinker:
          self.lane_change_direction = LaneChangeDirection.right

        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.lane_change_wait_timer = 0.0

      # preLaneChange: road edge 감지 시 차단
      elif self.lane_change_state == LaneChangeState.preLaneChange and self.road_edge:
        self.lane_change_direction = LaneChangeDirection.none

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        self.lane_change_wait_timer += DT_MDL

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.prev_lane_change = False

        elif torque_applied and blindspot_detected and self.auto_lane_change_timer != 10.0:
          self.auto_lane_change_timer = 10.0
          self.prev_lane_change = False

        elif not torque_applied and self.auto_lane_change_timer == 10.0 and not self.prev_torque_applied:
          self.prev_torque_applied = True

        elif (torque_applied and (not blindspot_detected or self.prev_torque_applied)) or \
             (lane_change_auto_timer and
              self.lane_change_wait_timer > lane_change_auto_timer and
              not self.prev_lane_change and
              not blindspot_detected):
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.prev_lane_change = True

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            self.lane_change_state = LaneChangeState.off
            self.prev_lane_change = False

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    if self.lane_change_state == LaneChangeState.off:
      self.auto_lane_change_timer = 0.0
      self.prev_torque_applied = False
    elif self.auto_lane_change_timer < (AUTO_LCA_START_TIME + 0.25):
      self.auto_lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    turn_direction = self.carrot_atc.steering_request(atc_state, v_ego) if atc_steering else 0
    if opposite_torque or conflicting_blinker:
      turn_direction = 0

    # Keep g_autoturn's distance/speed/driver safety gates for starting, then use
    # c3-wip's steering-angle + predicted-yaw falloff to end the turn at its apex.
    turn_active = (atc_state.get('kind') in ('turn', 'uturn') and
                   (turn_direction != 0 or self.turn_direction_latched != 0))
    self._update_atc_turn_completion(model_data, carstate, turn_active)
    if self.turn_disable_count > 0:
      turn_direction = 0
      self.turn_direction_latched = 0
      self.turn_ll_prob = 1.0

    # apilot 방식의 부드러운 회전종료 페이드: steering_request 가 0으로 끊긴 뒤에도
    # 모델이 아직 이 방향의 회전을 인지하고 있으면(desireState 확률 > 2%) 0.5초에
    # 걸쳐 래치된 방향을 유지하다 서서히 끈다. 차선변경 종료(lane_change_ll_prob)와
    # 동일한 감쇠 방식.
    turn_model_prob = 0.0
    if model_data is not None and turn_direction == 0 and self.turn_direction_latched != 0:
      try:
        latched_desire = (log.LateralPlan.Desire.turnLeft if self.turn_direction_latched < 0
                          else log.LateralPlan.Desire.turnRight)
        turn_model_prob = model_data.meta.desireState[latched_desire]
      except (IndexError, AttributeError, TypeError):
        turn_model_prob = 0.0

    if turn_direction != 0:
      self.turn_direction_latched = turn_direction
      self.turn_ll_prob = 1.0
    elif turn_model_prob > 0.02 and self.turn_ll_prob > 0.0:
      self.turn_ll_prob = max(self.turn_ll_prob - 2 * DT_MDL, 0.0)
    else:
      self.turn_direction_latched = 0
      self.turn_ll_prob = 1.0

    effective_turn_direction = turn_direction if turn_direction != 0 else (
      self.turn_direction_latched if self.turn_ll_prob > 0.0 else 0)
    if opposite_torque or conflicting_blinker:
      effective_turn_direction = 0
      self.turn_direction_latched = 0
      self.turn_ll_prob = 1.0

    # preLaneChange 는 지시등을 켠 순간 곧바로 들어가는 대기 상태일 뿐 아직 실제
    # 조향 개입은 없다(preLaneChange 의 desire 는 항상 none). 그런데 일반 도로
    # 우회전에서 습관적으로 지시등을 켜면, 속도가 AutoLaneChangeSpeed 이상일 때
    # 여기서 상태가 off→preLaneChange 로 바뀌어버려서 아래 조건이 막혀 ATC 회전
    # 조향이 무시되는 문제가 있었다. 실제로 진행 중인 차선변경(laneChangeStarting/
    # Finishing)만 보호하고, preLaneChange 는 회전조향이 덮어써도 되게 완화한다.
    if effective_turn_direction and self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.desire = log.LateralPlan.Desire.turnLeft if effective_turn_direction < 0 else log.LateralPlan.Desire.turnRight

    # LateralPlanner uses this already-validated state to blend TMAP route
    # curvature into the model path. Never let map steering survive a driver
    # override, brake press, stale route, or an actual lane change.
    self.atc_state = atc_state
    self.atc_turn_direction = effective_turn_direction
    self.atc_driver_cancel = (opposite_torque or conflicting_blinker or carstate.brakePressed or
                              not lateral_active or not atc_state.get('route_fresh', False) or
                              self.lane_change_state in (LaneChangeState.laneChangeStarting,
                                                         LaneChangeState.laneChangeFinishing))

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.LateralPlan.Desire.keepLeft, log.LateralPlan.Desire.keepRight):
        self.desire = log.LateralPlan.Desire.none

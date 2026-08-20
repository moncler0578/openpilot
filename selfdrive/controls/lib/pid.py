import numpy as np
from numbers import Number
from common.numpy_fast import clip, interp
class PIDController():
  def __init__(self, k_p, k_i, k_f=0., k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100, i_leak_factor=1.0):
    self._k_p = k_p
    self._k_i = k_i
    self._k_d = k_d
    self.k_f = k_f   # feedforward gain
    if isinstance(self._k_p, Number):
      self._k_p = [[0], [self._k_p]]
    if isinstance(self._k_i, Number):
      self._k_i = [[0], [self._k_i]]
    if isinstance(self._k_d, Number):
      self._k_d = [[0], [self._k_d]]
    self.pos_limit = pos_limit
    self.neg_limit = neg_limit
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    # 적분 누설 계수: 정상 추종 중 매 스텝 i 를 이 비율로 감쇠시켜 긴 커브에서의
    # 무한 누적(와인드업)을 막는다. 시간상수 τ ≈ 1/(rate*(1-leak)).
    #   0.999 @100Hz → τ≈1.0s / 0.998 → τ≈0.5s / 0.9995 → τ≈2.0s
    # 기본 1.0 = 누설 없음(원본 동작 완전 보존). longitudinal 등은 기본값을 쓰고,
    # lateral 토크 컨트롤러만 생성 시 0.999 등을 넘겨 적용한다.
    self.i_leak_factor = i_leak_factor
    self.speed = 0.0
    self.reset()
  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])
  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])
  @property
  def k_d(self):
    return interp(self.speed, self._k_d[0], self._k_d[1])
  @property
  def error_integral(self):
    return self.i/self.k_i
  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0
  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
    self.speed = speed
    self.p = float(error) * self.k_p
    self.f = feedforward * self.k_f
    self.d = error_rate * self.k_d
    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      # 적분 누설(leak/decay): 정상 추종 중에도 적분을 매 스텝 0 방향으로 조금씩
      # 흘려보내 긴 커브에서의 무한 누적을 막는다. freeze 시엔 '새 누적'은 멈추되
      # '기존 누적'은 계속 빠지도록 leak 은 freeze 와 무관하게 항상 적용한다.
      # (i_leak_factor=1.0 이면 self.i 불변 → 원본과 완전히 동일하게 동작)
      self.i *= self.i_leak_factor
      i = self.i + error * self.k_i * self.i_rate
      control = self.p + i + self.d + self.f
      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.i = i
    control = self.p + self.i + self.d + self.f
    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control

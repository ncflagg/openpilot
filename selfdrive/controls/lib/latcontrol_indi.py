import math
import numpy as np

from cereal import log
from common.realtime import DT_CTRL
from common.numpy_fast import clip
from selfdrive.car.toyota.carcontroller import SteerLimitParams
from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.controls.lib.drive_helpers import get_steer_max
from selfdrive.virtualZSS import virtualZSS_class
from selfdrive.kegman_conf import kegman_conf
from common.realtime import sec_since_boot


def mean(numbers):
  return float(sum(numbers)) / max(len(numbers), 1)


class LatControlINDI(object):
  def __init__(self, CP):
    self.angle_steers_des = 0.
    self.angle_steers_vzss = 0.

    A = np.matrix([[1.0, DT_CTRL, 0.0],
                   [0.0, 1.0, DT_CTRL],
                   [0.0, 0.0, 1.0]])
    C = np.matrix([[1.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0]])

    # Q = np.matrix([[1e-2, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 10.0]])
    # R = np.matrix([[1e-2, 0.0], [0.0, 1e3]])

    # (x, l, K) = control.dare(np.transpose(A), np.transpose(C), Q, R)
    # K = np.transpose(K)
    K = np.matrix([[7.30262179e-01, 2.07003658e-04],
                   [7.29394177e+00, 1.39159419e-02],
                   [1.71022442e+01, 3.38495381e-02]])

    self.K = K
    self.A_K = A - np.dot(K, C)
    self.x = np.matrix([[0.], [0.], [0.]])

    self.enfore_rate_limit = CP.carName == "toyota"

    self.RC = CP.lateralTuning.indi.timeConstant
    self.G = CP.lateralTuning.indi.actuatorEffectiveness
    self.outer_loop_gain = CP.lateralTuning.indi.outerLoopGain
    self.inner_loop_gain = CP.lateralTuning.indi.innerLoopGain
    self.alpha = 1. - DT_CTRL / (self.RC + DT_CTRL)

    self.ZSS = virtualZSS_class.vZSS()
    self.output_steer = 0
    self.readings = []
    # Live Tuning variable init
    kegman = kegman_conf(CP)
    self.mpc_frame = 0

    self.reset()

  def reset(self):
    self.delayed_output = 0.
    self.output_steer = 0.
    self.counter = 0


  # Live tuning
  def live_tune(self, CP):
    self.mpc_frame += 1
    if self.mpc_frame % 300 == 0:
      # live tuning through /data/openpilot/tune.py overrides interface.py settings
      kegman = kegman_conf()
      if True:
        # Get values
        self.timeConstant = float(kegman.conf['timeConstant'])
        self.actuatorEffectiveness = float(kegman.conf['actuatorEffectiveness'])
        self.outerLoopGain = float(kegman.conf['outerLoopGain'])
        self.innerLoopGain = float(kegman.conf['innerLoopGain'])

        # Set values
        self.RC = self.timeConstant
        self.G = self.actuatorEffectiveness
        self.outer_loop_gain = self.outerLoopGain
        self.inner_loop_gain = self.innerLoopGain
        self.alpha = 1. - DT_CTRL / (self.RC + DT_CTRL)

      self.mpc_frame = 0

  def update(self, active, v_ego, angle_steers, angle_steers_rate, eps_torque, steer_override, CP, VM, path_plan, driver_torque, wheel_speeds):
    #virtualZSS
    stock_angle = float(angle_steers)
    if self.ZSS.can_predict():
      angle_steers = self.ZSS.predict()
      self.angle_steers_vzss = self.ZSS.predict()

    self.ZSS.save_sample([stock_angle, self.output_steer, wheel_speeds.fl, wheel_speeds.fr, wheel_speeds.rl, wheel_speeds.rr])

    # smooth angle
    #max_samples = 20
    #self.readings.append(angle_steers)
    #angle_steers = round(mean(self.readings), 2)
    #if len(self.readings) == max_samples:
    #    self.readings.pop(0)

    # Trigger Live tuning
    self.live_tune(CP)

    # Update Kalman filter
    y = np.matrix([[math.radians(angle_steers)], [math.radians(angle_steers_rate)]])
    self.x = np.dot(self.A_K, self.x) + np.dot(self.K, y)

    indi_log = log.ControlsState.LateralINDIState.new_message()
    indi_log.steerAngle = math.degrees(self.x[0])
    indi_log.steerRate = math.degrees(self.x[1])
    indi_log.steerAccel = math.degrees(self.x[2])

    if v_ego < 0.3 or not active:
      indi_log.active = False
      self.output_steer = 0.0
      self.delayed_output = 0.0
    else:
      self.angle_steers_des = round(path_plan.angleSteers, 2)
      self.rate_steers_des = path_plan.rateSteers

      steers_des = math.radians(self.angle_steers_des)
      rate_des = math.radians(self.rate_steers_des)

      # Expected actuator value
      self.delayed_output = self.delayed_output * self.alpha + self.output_steer * (1. - self.alpha)

      # Compute acceleration error
      rate_sp = self.outer_loop_gain * (steers_des - self.x[0]) + rate_des
      accel_sp = self.inner_loop_gain * (rate_sp - self.x[1])
      accel_error = accel_sp - self.x[2]

      # Compute change in actuator
      g_inv = 1. / self.G
      delta_u = g_inv * accel_error

      # Enforce rate limit
      if self.enfore_rate_limit:
        steer_max = float(SteerLimitParams.STEER_MAX)
        new_output_steer_cmd = steer_max * (self.delayed_output + delta_u)
        prev_output_steer_cmd = steer_max * self.output_steer
        new_output_steer_cmd = apply_toyota_steer_torque_limits(new_output_steer_cmd, prev_output_steer_cmd, prev_output_steer_cmd, SteerLimitParams)
        self.output_steer = new_output_steer_cmd / steer_max
      else:
        self.output_steer = self.delayed_output + delta_u

      steers_max = get_steer_max(CP, v_ego)
      self.output_steer = clip(self.output_steer, -steers_max, steers_max)

      print "seconds:", round(sec_since_boot(), 2), "mph:", int(round(v_ego * 2.237, 1)), "a_des", round(self.angle_steers_des, 2), "TSS1:", round(stock_angle, 2), "vZSS:", round(angle_steers, 2), "o_steer:", self.output_steer
      #"os_smooth", self.output_steer_smoothed


      indi_log.active = True
      indi_log.rateSetPoint = float(rate_sp)
      indi_log.accelSetPoint = float(accel_sp)
      indi_log.accelError = float(accel_error)
      indi_log.delayedOutput = float(self.delayed_output)
      indi_log.delta = float(delta_u)
      indi_log.output = float(self.output_steer)

    self.sat_flag = False
    return float(self.output_steer), float(self.angle_steers_des), float(self.angle_steers_vzss), indi_log

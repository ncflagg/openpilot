import numpy as np
from selfdrive.controls.lib.drive_helpers import get_steer_max
from common.numpy_fast import clip
from cereal import log
from selfdrive.virtualZSS import virtualZSS_wrapper

def interp_fast(x, xp, fp):  # extrapolates above range, np.interp does not
  return (((x - xp[0]) * (fp[1] - fp[0])) / (xp[1] - xp[0])) + fp[0]

class LatControlLQR(object):
  def __init__(self, CP, rate=100):
    self.sat_flag = False
    self.scale = CP.lateralTuning.lqr.scale
    self.ki = CP.lateralTuning.lqr.ki


    self.A = np.array(CP.lateralTuning.lqr.a).reshape((2,2))
    self.B = np.array(CP.lateralTuning.lqr.b).reshape((2,1))
    self.C = np.array(CP.lateralTuning.lqr.c).reshape((1,2))
    self.K = np.array(CP.lateralTuning.lqr.k).reshape((1,2))
    self.L = np.array(CP.lateralTuning.lqr.l).reshape((2,1))
    self.dc_gain = CP.lateralTuning.lqr.dcGain

    self.x_hat = np.array([[0], [0]])
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate

    #virtualZSS
    self.model_wrapper = virtualZSS_wrapper.get_wrapper()
    self.model_wrapper.init_model()
    self.output_steer = 0

    self.reset()
    self.past_data = []
    self.seq_len = 20
    self.scales = {'zorro_sensor': [-31.666841506958008, 39.42588806152344],
                   'stock_sensor': [-31.0, 37.599998474121094], 'steer_command': [-1.0, 1.0]}

  def reset(self):
    self.i_lqr = 0.0
    self.output_steer = 0.0

  def update(self, active, v_ego, angle_steers, angle_steers_rate, eps_torque, steer_override, CP, VM, path_plan, driver_torque):
    #virtualZSS
    self.past_data.append([interp_fast(angle_steers, self.scales['stock_sensor'], [0, 1]), self.output_steer])  # steer command is already 'normalized'
    while len(self.past_data) > self.seq_len:
      del self.past_data[0]

    if len(self.past_data) == self.seq_len:
      angle_steers = interp_fast(float(self.model_wrapper.run_model_time_series([i for x in self.past_data for i in x])), [0.0, 1.0], self.scales['zorro_sensor'])

    lqr_log = log.ControlsState.LateralLQRState.new_message()

    torque_scale = (0.45 + v_ego / 60.0)**2  # Scale actuator model with speed

    # Subtract offset. Zero angle should correspond to zero torque
    self.angle_steers_des = path_plan.angleSteers - path_plan.angleOffset
    angle_steers -= path_plan.angleOffset

    # Update Kalman filter
    angle_steers_k = float(self.C.dot(self.x_hat))
    e = angle_steers - angle_steers_k
    self.x_hat = self.A.dot(self.x_hat) + self.B.dot(eps_torque / torque_scale) + self.L.dot(e)

    if v_ego < 0.3 or not active:
      lqr_log.active = False
      self.reset()
    else:
      lqr_log.active = True

      # LQR
      u_lqr = float(self.angle_steers_des / self.dc_gain - self.K.dot(self.x_hat))

      # Integrator
      if steer_override:
        self.i_lqr -= self.i_unwind_rate * float(np.sign(self.i_lqr))
      else:
        self.i_lqr += self.ki * self.i_rate * (self.angle_steers_des - angle_steers_k)

      lqr_output = torque_scale * u_lqr / self.scale
      self.i_lqr = clip(self.i_lqr, -1.0 - lqr_output, 1.0 - lqr_output) # (LQR + I) has to be between -1 and 1

      self.output_steer = lqr_output + self.i_lqr

      # Clip output
      steers_max = get_steer_max(CP, v_ego)
      self.output_steer = clip(self.output_steer, -steers_max, steers_max)

    lqr_log.steerAngle = angle_steers_k + path_plan.angleOffset
    lqr_log.i = self.i_lqr
    lqr_log.output = self.output_steer
    return self.output_steer, float(self.angle_steers_des), lqr_log

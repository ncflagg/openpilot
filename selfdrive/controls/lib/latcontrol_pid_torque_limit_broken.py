from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from cereal import car
from cereal import log
from selfdrive.virtualZSS import virtualZSS_wrapper
#from selfdrive.kegman_conf import kegman_conf
from common.realtime import sec_since_boot
from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.toyota.carcontroller import CustomSteerLimitParams

## NO PWD
## NO self.output_steer

def interp_fast(x, xp, fp):  # extrapolates above range, np.interp does not
  return (((x - xp[0]) * (fp[1] - fp[0])) / (xp[1] - xp[0])) + fp[0]

class LatControlPID(object):
  def __init__(self, CP):
    self.pid = PIController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            k_f=CP.lateralTuning.pid.kf, pos_limit=1.0)
    self.angle_steers_des = 0.

    self.output_steer_prev = 0.

    # virtualZSS
    self.model_wrapper = virtualZSS_wrapper.get_wrapper()
    self.model_wrapper.init_model()
    self.output_steer = 0
    self.past_data = []
    self.seq_len = 20 # 40 was WILD
    self.scales = {'zorro_sensor': [-31.666841506958008, 39.42588806152344],
                   'stock_sensor': [-31.0, 37.599998474121094], 'steer_command': [-1.0, 1.0]}


    # Working variables
    self.TSS1 = 0.0                            # Track TSS1 angle for stuck-detection


  def reset(self):
    self.pid.reset()

  def update(self, active, v_ego, angle_steers, angle_steers_rate, eps_torque, steer_override, CP, VM, path_plan, driver_torque):
    self.TSS1 = angle_steers      # Save for stuck-detection
    # virtualZSS
    # Offset vZSS angle in the past history data here. Might fix TAKE CONTROL msgs
    self.past_data.append([interp_fast(angle_steers - 0.5, self.scales['stock_sensor'], [0, 1]), self.output_steer])  # steer command is already 'normalized'
    while len(self.past_data) > self.seq_len:
      del self.past_data[0]

    #print "past_data:", self.past_data
	  
    if len(self.past_data) == self.seq_len:
      # Add manual offset
      angle_steers = interp_fast(float(self.model_wrapper.run_model_time_series([i for x in self.past_data for i in x])), [0.0, 1.0], self.scales['zorro_sensor'])
    #angle_steers -= 0.4  # I've seen some evidence that vZSS is off by +0.5deg
    #print "vZSS:", round(angle_steers, 2)

    #angle_steers += 1.1   # Trying adding my offset here too
    #print "vZSS + offset:", angle_steers

    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steerAngle = float(angle_steers)
    pid_log.steerRate = float(angle_steers_rate)

    if v_ego < 0.3 or not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      self.angle_steers_des = path_plan.angleSteers  # get from MPC/PathPlanner
      steers_max = get_steer_max(CP, v_ego)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      steer_feedforward = self.angle_steers_des   # feedforward desired angle
      if CP.steerControlType == car.CarParams.SteerControlType.torque:
        # TODO: feedforward something based on path_plan.rateSteers
        steer_feedforward -= path_plan.angleOffset   # subtract the offset, since it does not contribute to resistive torque
        steer_feedforward *= v_ego**2  # proportional to realigning tire momentum (~ lateral accel)
      deadzone = 0.01
      output_steer = self.pid.update(self.angle_steers_des, angle_steers, check_saturation=(v_ego > 10), override=steer_override,
                                     feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone)

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = bool(self.pid.saturated)

      print "seconds:", round(sec_since_boot(), 2), "mph:", int(round(v_ego * 2.237, 1)), "a_des", round(self.angle_steers_des, 2), "TSS1:", round(self.TSS1, 2), "vZSS:", round(angle_steers, 2), "o_steer:", round(output_steer, 2)

    # Haven't been feeding vZSS torque commands up til now (10-16-19)
    # May mean it wasn't really working, at least as accurately as it was supposed to
    self.output_steer = output_steer


    #print round(sec_since_boot(), 2), "mph:", int(round(v_ego * 2.237, 1)), "a_des", round(self.angle_steers_des, 2), "a_steers:", round(self.TSS1, 2)


    self.sat_flag = self.pid.saturated
    return output_steer, float(self.angle_steers_des), pid_log


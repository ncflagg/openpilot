import os
import math
import numpy as np

from common.realtime import sec_since_boot
from selfdrive.services import service_list
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LAT
from selfdrive.controls.lib.model_parser import ModelParser
import selfdrive.messaging as messaging

LOG_MPC = os.environ.get('LOG_MPC', False)


def calc_states_after_delay(states, v_ego, steer_angle, curvature_factor, steer_ratio, delay):
  states[0].x = v_ego * delay
  states[0].psi = v_ego * curvature_factor * math.radians(steer_angle) / steer_ratio * delay
  states[0].delta = math.radians(steer_angle) / steer_ratio
  return states


class PathPlanner(object):
  def __init__(self, CP):
    self.MP = ModelParser()

    self.l_poly = [0., 0., 0., 0.]
    self.r_poly = [0., 0., 0., 0.]

    self.last_cloudlog_t = 0

    self.plan = messaging.pub_sock(service_list['pathPlan'].port)
    self.livempc = messaging.pub_sock(service_list['liveMpc'].port)

    self.setup_mpc(CP.steerRateCost)
    self.solution_invalid_cnt = 0

    self.frame = 0
    #self.avg_offset = 0
    self.avg_offset  = -1.1          # Start with my known offset, for now
    self.fast_offset = 0

    self.learning_rate = 10
    self.slow_learning_rate = 2400   # Z was 12000 for slow curvature and said 100Hz


  def setup_mpc(self, steer_rate_cost):
    self.libmpc = libmpc_py.libmpc
    self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, steer_rate_cost)

    self.mpc_solution = libmpc_py.ffi.new("log_t *")
    self.cur_state = libmpc_py.ffi.new("state_t *")
    self.cur_state[0].x = 0.0
    self.cur_state[0].y = 0.0
    self.cur_state[0].psi = 0.0
    self.cur_state[0].delta = 0.0

    self.angle_steers_des = 0.0
    self.angle_steers_des_mpc = 0.0
    self.angle_steers_des_prev = 0.0
    self.angle_steers_des_time = 0.0

    self.l_poly = libmpc_py.ffi.new("double[4]")
    self.r_poly = libmpc_py.ffi.new("double[4]")
    self.p_poly = libmpc_py.ffi.new("double[4]")

  def update(self, sm, CP, VM):
    v_ego = sm['carState'].vEgo
    # Feed vZSS angle here?
    angle_steers = sm['carState'].steeringAngle
    active = sm['controlsState'].active

    #angle_offset_average = sm['liveParameters'].angleOffsetAverage
    #angle_offset_average = -1.15   # Nate's Prius Prime's average

    # Using the learner. Behind by one loop setting it here...
    angle_offset_average = self.avg_offset

    #angle_offset_bias = sm['controlsState'].angleModelBias + angle_offset_average
    angle_offset_bias = angle_offset_average
    angle_offset = sm['liveParameters'].angleOffset




    # Stolen from Zorro's curvature learner
    # Also, make sure we're learning only when cruise is on
    # Make sure to remove manually set offset(s) elsewhere, except maybe in controlsd
    # Only learn when going straight
    # How to avoid right-drive-bias? I'm still having trouble coming up with a decent way to figure out when the car is rolling one way or another.
    # Actually, one way I can think of would be to only learn slow AO 35 - 55mph or something.
    # That way, I think there's less likelihood of driving on constantly-cambered roads.
    # Still want to read some kind of tilt, though

    if active  and  angle_steers > 0.1  and  15.6 < v_ego < 24.6:  # 35-55mph
      if abs(angle_steers) < 2.:
        self.avg_offset -= self.MP.d_poly[3] / self.slow_learning_rate
      elif angle_steers < -0.1:
        if abs(angle_steers) < 2.:
          self.avg_offset += self.MP.d_poly[3] / self.slow_learning_rate
    self.avg_offset = np.clip(self.avg_offset, -2.0, 2.0)

    # How about a fast learner? I think the hard part here will be overshoot. 
    # Rate of change is a bigger concern, but we need it to change quickly.
    # -Might try increments like +/-0.2deg.
    # -But, how do we know when to back it off?
    # -And what if we pull back too much? Pings and pongs

    # Fast offset
    # Angle doesn't matter

    if active  and  v_ego > 15.6:                    # 35mph
      if abs(angle_steers - self.avg_offset) > 0:
        self.fast_offset -= self.MP.d_poly[3] / self.learning_rate
      else:
        self.fast_offset += self.MP.d_poly[3] / self.learning_rate
    else:
      self.fast_offset = 0
    self.fast_offset = np.clip(self.fast_offset, -4.0, 4.0)

    self.frame += 1
    if active  and  self.frame >= 20:   # every second, at 100Hz. If not, need to change learning rate
      print "avg_offset:", self.avg_offset, "fast_offset:", self.fast_offset
      self.frame = 0

    angle_steers = angle_steers - self.avg_offset - self.fast_offset   # Is subtraction correct here ???








    self.MP.update(v_ego, sm['model'])

    # Run MPC
    self.angle_steers_des_prev = self.angle_steers_des_mpc
    # just omit this?
    VM.update_params(sm['liveParameters'].stiffnessFactor, sm['liveParameters'].steerRatio)
    curvature_factor = VM.curvature_factor(v_ego)
    #curvature_factor = 0.29 #0.4 was ok          # Testing setting statically
    #Might have been way off. Try 0.0076, 0.009, 0.014
    #curvature_factor = 0.016  # 0.014 - 0.016 seemed nice is straights and turned too much in corners like Z said
    # I think I've been confusing cfactor with "curvature" a bunch.... :\
    #curvature_factor = 0.45 ..didnt seem enough either  #0.47 not turning enough
    #curvature_factor = 0.34 # 0.42 may have not been turning enough on the freeway
    self.l_poly = list(self.MP.l_poly)
    self.r_poly = list(self.MP.r_poly)
    self.p_poly = list(self.MP.p_poly)

    # account for actuation delay
    self.cur_state = calc_states_after_delay(self.cur_state, v_ego, angle_steers - angle_offset_average, curvature_factor, CP.steerRatio, CP.steerActuatorDelay)

    v_ego_mpc = max(v_ego, 5.0)  # avoid mpc roughness due to low speed
    self.libmpc.run_mpc(self.cur_state, self.mpc_solution,
                        self.l_poly, self.r_poly, self.p_poly,
                        self.MP.l_prob, self.MP.r_prob, self.MP.p_prob, curvature_factor, v_ego_mpc, self.MP.lane_width)

    # reset to current steer angle if not active or overriding
    if active:
      delta_desired = self.mpc_solution[0].delta[1]
      rate_desired = math.degrees(self.mpc_solution[0].rate[0] * CP.steerRatio)
    else:
      delta_desired = math.radians(angle_steers - angle_offset_bias) / CP.steerRatio
      rate_desired = 0.0

    #self.cur_state[0].delta = delta_desired

    self.angle_steers_des_mpc = float(math.degrees(delta_desired * CP.steerRatio) + angle_offset_bias)

    #  Check for infeasable MPC solution
    mpc_nans = np.any(np.isnan(list(self.mpc_solution[0].delta)))
    t = sec_since_boot()
    if mpc_nans:
      self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, CP.steerRateCost)
      self.cur_state[0].delta = math.radians(angle_steers - angle_offset_bias) / CP.steerRatio

      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.mpc_solution[0].cost > 20000. or mpc_nans:   # TODO: find a better way to detect when MPC did not converge
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0
    plan_solution_valid = self.solution_invalid_cnt < 2

    plan_send = messaging.new_message()
    plan_send.init('pathPlan')
    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState', 'liveParameters', 'model'])
    plan_send.pathPlan.laneWidth = float(self.MP.lane_width)
    plan_send.pathPlan.dPoly = [float(x) for x in self.MP.d_poly]
    plan_send.pathPlan.cPoly = [float(x) for x in self.MP.c_poly]
    plan_send.pathPlan.cProb = float(self.MP.c_prob)
    plan_send.pathPlan.lPoly = [float(x) for x in self.l_poly]
    plan_send.pathPlan.lProb = float(self.MP.l_prob)
    plan_send.pathPlan.rPoly = [float(x) for x in self.r_poly]
    plan_send.pathPlan.rProb = float(self.MP.r_prob)
    plan_send.pathPlan.angleSteers = float(self.angle_steers_des_mpc)
    plan_send.pathPlan.rateSteers = float(rate_desired)
    plan_send.pathPlan.angleOffset = float(angle_offset_average)
    plan_send.pathPlan.mpcSolutionValid = bool(plan_solution_valid)
    plan_send.pathPlan.paramsValid = bool(sm['liveParameters'].valid)
    plan_send.pathPlan.sensorValid = bool(sm['liveParameters'].sensorValid)
    plan_send.pathPlan.posenetValid = bool(sm['liveParameters'].posenetValid)

    self.plan.send(plan_send.to_bytes())

    if LOG_MPC:
      dat = messaging.new_message()
      dat.init('liveMpc')
      dat.liveMpc.x = list(self.mpc_solution[0].x)
      dat.liveMpc.y = list(self.mpc_solution[0].y)
      dat.liveMpc.psi = list(self.mpc_solution[0].psi)
      dat.liveMpc.delta = list(self.mpc_solution[0].delta)
      dat.liveMpc.cost = self.mpc_solution[0].cost
      self.livempc.send(dat.to_bytes())

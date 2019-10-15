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
from selfdrive.controls.lib.offset_learner import OffsetLearner

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

    self.learning_rate = 60.
    self.fast_learning_rate = 1. / 600.    # 80, Learn at max rate of 1 deg every 4 sec at 20Hz
    self.slow_learning_rate = 1. / 72000.  # 72k=1hr. 1200 (20 x 60) means learn at max rate of 1 deg in 1 minute at 20Hz
    self.avg_offset = 0.0                 # Start near my offset. -now in carstate-
    self.fast_offset = 0.
    self.frame_print = 0


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
    angle_steers = sm['carState'].steeringAngle
    active = sm['controlsState'].active

    #angle_offset_average = sm['liveParameters'].angleOffsetAverage
    #angle_offset_average = -1.15   # Nate's Prius Prime's average
    #angle_offset_bias = sm['controlsState'].angleModelBias + angle_offset_average
    #angle_offset_bias = angle_offset_average
    angle_offset = sm['liveParameters'].angleOffset   # from params_learner. Not currently used here



    # Seems like I could do  abs(angle_steers - self.avg_offset - fast)  here also
    # When going straight, and a faster learner is working, we're going 0, even though there's camber
    # Does that effect path planning? I don't think OP knows bout tilt at all

    if active  and  8 < v_ego:
      if self.MP.d_poly[3] < 0:                                           # Riding left of center (d_poly[3] will be negative)..
        #self.fast_offset += self.fast_learning_rate # works, but needs to recover faster when centered to avoid overshoot
        # Mod fO to the right (neg) at a faster rate than the avg
        # Though, I'm starting to think it might be better to have a fast learner, learning only when the AVG learner is not
        #self.fast_offset -= self.MP.d_poly[3] / self.learning_rate        # Mod fastO to the right (angle_steers neg)
        if abs(angle_steers - self.avg_offset) < 3:                       # ..and near center
          self.avg_offset -= self.slow_learning_rate                      # Mod avg to the right (angle mod will be neg, opposite of d_poly behavior)

        #elif abs(angle_steers - self.avg_offset - self.fast_offset) > 3: # and NOT near center
        #  self.fast_offset += self.MP.d_poly[3] / self.learning_rate     # Mod fO to the right (neg)

      elif 0 < self.MP.d_poly[3]:                                         # Riding to the right of center (d_poly[3] will be positive)..
        #self.fast_offset -= self.fast_learning_rate # works               # Mod fO to the left (pos) at a faster rate than the avg
        #self.fast_offset -= self.MP.d_poly[3] / self.learning_rate
        if abs(angle_steers - self.avg_offset) < 3:                       # ..and near center
          self.avg_offset += self.slow_learning_rate                      # Mod avg to the left (angle mod will be pos, opposite of d_poly behavior)

        #elif abs(angle_steers - self.avg_offset - self.fast_offset) > 3:  # and NOT near center
        #  self.fast_offset -= self.MP.d_poly[3] / self.slow_learning_rate # Mod fO to the left (pos)

    if not active:
      self.fast_offset = 0.                                               # Kill the fast offset

    if v_ego < 8:
      self.fast_offset *= 0.99                                            # Kill, slowly

    self.avg_offset  = np.clip(self.avg_offset,  -2.0, 2.0)
    self.fast_offset = np.clip(self.fast_offset, -0.5, 0.5)               # Limit fast learning for now, or AVG won't learn as much. Actually, that might be IMPORTANT# Will avg work if fast O is running correctly?


    if active  and  self.frame_print >= 5  and  8 < v_ego:
      #print round (sec_since_boot(), 2,), "steer", round(angle_steers, 2), "avg_offset:", round(self.avg_offset, 3), "fast_offset:", round(self.fast_offset, 3), "dpoly3", round(self.MP.d_poly[3], 3)
      self.frame_print = 0
    self.frame_print += 1

    angle_offset_average = self.avg_offset
    angle_offset_bias = angle_offset_average - self.fast_offset   # Not sure about this yet. Might need to be some combination of angle_offset_average and angle_offset_bias if Gernby set angle_offset_bias to avg
    #angle_offset_bias = angle_offset_average



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
    #curvature_factor = 0.41 # 0.42 may have not been turning enough on the freeway
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

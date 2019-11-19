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

    self.aDelay = 0.0
    self.delay_switch_time = 0.0

  def update(self, sm, CP, VM):
    v_ego = sm['carState'].vEgo
    angle_steers = sm['carState'].steeringAngle
    active = sm['controlsState'].active

    #angle_offset_average = sm['liveParameters'].angleOffsetAverage
    angle_offset_average = 0.0
    #angle_offset_average = -1.15   # Nate's Prius Prime's average
    #angle_offset_bias = sm['controlsState'].angleModelBias + angle_offset_average
    #angle_offset_bias = angle_offset_average
    angle_offset = sm['liveParameters'].angleOffset   # from params_learner
    #angle_offset = 0.0
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


    steerRatio = 13.4

    # Interp between two points I think are good
    #curvature_factor = np.interp(v_ego, [0., 31.5], [0.5495, 0.1])
    #curvature_factor = np.interp(v_ego, [12.07, 28.16], [0.375, 0.175])
    curvature_factor = np.interp(v_ego, [12.07, 29.95], [0.39, 0.144])
    #  12.07/26mph - 0.37726
    #  28.16/66mph - 0.1476

    corner = abs(self.MP.l_poly[1]) > 0.00003  or  abs(self.MP.r_poly[1]) > 0.00003
      #or  abs(self.MP.d_poly[3]) > 0.09  # Shouldn't need to track lane center, if lower sR = MORE steering already

    if corner:
      # Tune for turns
      steerRatio = 13.4
      self.aDelay = 0.26
      self.delay_switch_time = 0.26 + sec_since_boot()
    else:
      # Tune for straights
      steerRatio = 13.4
      # OR higher actuator delay
      if self.delay_switch_time < sec_since_boot():  # Don't switch if it's recently changed
        self.aDelay = CP.steerActuatorDelay #test if 0.75 contributes to 'lateness'  #0.75  #1.0 made it hug to the RIGHT a little  # maybe with  steerRatio = 18.0
        #curvature_factor = 0.5
        # Or, decrease cfactor by 2.5%, increasing steering while "going straight"
        curvature_factor *= 1.0
        #curvature_factor = 0.37

    if self.aDelay == 0.0:
      self.aDelay = CP.steerActuatorDelay

    #corner_delay = 0.25

    #if 1.9 <  abs(self.angle_steers_des - angle_offset): # angle checked can be smaller ASSUMING avg offset is being processed already on angle_steers. Otherwise, need -avg_offset too

    #if 2.5 < abs(self.angle_steers_des):
      # Ramp delay
      #self.aDelay -= 0.05  # +/- 0.05 (20Hz) seems ideal since longitudinal should never go backwards
      # -= CP.steerActuatorDelay - corner_delay
      #self.aDelay = 0.12  #0.25
    #  self.aDelay = CP.steerActuatorDelay
    #else:
      #self.aDelay += 0.05   # Should be able to move up faster since it's in the future?
    #  self.aDelay = CP.steerActuatorDelay
    # If ramping, clip
    #self.aDelay = np.clip(self.aDelay, corner_delay, CP.steerActuatorDelay)

    # account for actuation delay
    self.cur_state = calc_states_after_delay(self.cur_state, v_ego, angle_steers - angle_offset_average - angle_offset, curvature_factor, steerRatio, self.aDelay)

    #print "cur_state:", self.cur_state

    print "seconds:", round(sec_since_boot(), 2)
    print "f_offset:", angle_offset, "d_poly3:", self.MP.d_poly[3], "c_poly3:", self.MP.c_poly[3], "l_poly:", self.MP.l_poly, "r_poly:", self.MP.r_poly, "    p_poly:", self.MP.p_poly, "curv:", curvature_factor
    print "Corner?:", corner

    v_ego_mpc = max(v_ego, 5.0)  # avoid mpc roughness due to low speed
    self.libmpc.run_mpc(self.cur_state, self.mpc_solution,
                        self.l_poly, self.r_poly, self.p_poly,
                        self.MP.l_prob, self.MP.r_prob, self.MP.p_prob, curvature_factor, v_ego_mpc, self.MP.lane_width)

    # reset to current steer angle if not active or overriding
    if active:
      delta_desired = self.mpc_solution[0].delta[1]
      rate_desired = math.degrees(self.mpc_solution[0].rate[0] * steerRatio)
    else:
      delta_desired = math.radians(angle_steers - angle_offset) / steerRatio
      rate_desired = 0.0

    #self.cur_state[0].delta = delta_desired

    self.angle_steers_des_mpc = float(math.degrees(delta_desired * steerRatio) + angle_offset)

    #  Check for infeasable MPC solution
    mpc_nans = np.any(np.isnan(list(self.mpc_solution[0].delta)))
    t = sec_since_boot()
    if mpc_nans:
      self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, CP.steerRateCost)
      self.cur_state[0].delta = math.radians(angle_steers - angle_offset) / steerRatio

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

import os
import math
import numpy as np

# from common.numpy_fast import clip
from common.realtime import sec_since_boot
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LAT
from selfdrive.controls.lib.lane_planner import LanePlanner
import selfdrive.messaging as messaging
from selfdrive.controls.lib.curvature_learner import CurvatureLearner

LOG_MPC = os.environ.get('LOG_MPC', False)


def calc_states_after_delay(states, v_ego, steer_angle, curvature_factor, steer_ratio, delay):
  states[0].x = v_ego * delay
  states[0].psi = v_ego * curvature_factor * math.radians(steer_angle) / steer_ratio * delay
  return states


class PathPlanner(object):
  def __init__(self, CP):
    self.LP = LanePlanner()

    self.last_cloudlog_t = 0

    self.setup_mpc(CP.steerRateCost)
    self.solution_invalid_cnt = 0
    #self.path_offset_i = 0.0

    self.aDelay = 0.0
    self.delay_switch_time = 0.0

    self.frame = 0
    self.curvature_offset = CurvatureLearner(debug=False)


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

  def update(self, sm, pm, CP, VM):
    v_ego = sm['carState'].vEgo
    angle_steers = sm['carState'].steeringAngle
    active = sm['controlsState'].active

    angle_offset = sm['liveParameters'].angleOffset

    # Gernby: prevent over-inflation of desired angle
    # https://github.com/commaai/openpilot/pull/700/files
    # Doesn't look like it would do anything in 0.6.4 though; needs review
    if abs(self.angle_steers_des_mpc - self.angle_steers_des_prev) > abs(angle_steers - self.angle_steers_des_prev):
      self.cur_state[0].delta = math.radians(angle_steers - angle_offset) / VM.sR

    self.LP.update(v_ego, sm['model'])

    # Run MPC
    self.angle_steers_des_prev = self.angle_steers_des_mpc
    VM.update_params(sm['liveParameters'].stiffnessFactor, sm['liveParameters'].steerRatio)

    curvature_factor = VM.curvature_factor(v_ego)


    steerRatio = 13.4
    if self.aDelay == 0.0:
      self.aDelay = CP.steerActuatorDelay

    # Interp between two points I think are good
    #curvature_factor = np.interp(v_ego, [0., 31.5], [0.5495, 0.1])
    #curvature_factor = np.interp(v_ego, [12.07, 28.16], [0.375, 0.175])
    #curvature_factor = np.interp(v_ego, [12.07, 29.95], [0.39, 0.144])
    #  12.07/26mph - 0.37726
    #  28.16/66mph - 0.1476

    curve = abs(self.LP.l_poly[1]) > 0.00003  or  abs(self.LP.r_poly[1]) > 0.00003
      #or  abs(self.LP.d_poly[3]) > 0.09  # If too far out of line

    if curve:
      # Tune for curves
      self.aDelay = 0.26
      self.delay_switch_time = 0.26 + sec_since_boot()
    else:
      # Tune for straights
      #steerRatio = 9.0
      if self.delay_switch_time < sec_since_boot():  # Don't switch if it's recently changed
        self.aDelay = 0.5 #CP.steerActuatorDelay    # 1.0 made it hug to the RIGHT a little
        #curvature_factor = 0.5


    # TODO: Check for active, override, and saturation
    # if active:
    #   self.path_offset_i += self.LP.d_poly[3] / (60.0 * 20.0)
    #   self.path_offset_i = clip(self.path_offset_i, -0.5,  0.5)
    #   self.LP.d_poly[3] += self.path_offset_i
    # else:
    #   self.path_offset_i = 0.0

    #if active:
    #  curvfac = self.curvature_offset.update(angle_steers - angle_offset, self.LP.d_poly, v_ego)
    #  print "angle_steers:", angle_steers, "ao:", angle_offset, "dpoly:", self.LP.d_poly, "dp3:", self.LP.d_poly[3], "speed:", v_ego
    #else:
    #  curvfac = 0.
    #curvature_factor = VM.curvature_factor(v_ego) + curvfac

    # account for actuation delay
    #self.cur_state = calc_states_after_delay(self.cur_state, v_ego, angle_steers - angle_offset, curvature_factor, VM.sR, CP.steerActuatorDelay)
    self.cur_state = calc_states_after_delay(self.cur_state, v_ego, angle_steers - angle_offset, curvature_factor, VM.sR, self.aDelay)

    v_ego_mpc = max(v_ego, 5.0)  # avoid mpc roughness due to low speed

    #print "cur_state:", self.cur_state
    print "seconds:", round(sec_since_boot(), 2)
    print "angle", angle_steers, "f_offset:", angle_offset, "curv:", curvature_factor, "tS:", sm['liveParameters'].stiffnessFactor
    print "d_poly3:", self.LP.d_poly[3], "lProb:", self.LP.l_prob, "rProb:", self.LP.r_prob, "veMpc:", v_ego_mpc, "lWidth:", self.LP.lane_width
    print "aDesMpc:", self.angle_steers_des_mpc, "aD", self.aDelay
    # "l_poly:", self.LP.l_poly, "r_poly:", self.LP.r_poly, "p_poly:"    , self.LP.p_poly,
    #print "Corner?:", corner
    #"c_poly3:", self.LP.c_poly[3]

    self.libmpc.run_mpc(self.cur_state, self.mpc_solution,
                        list(self.LP.l_poly), list(self.LP.r_poly), list(self.LP.d_poly),
                        self.LP.l_prob, self.LP.r_prob, curvature_factor, v_ego_mpc, self.LP.lane_width)

    # reset to current steer angle if not active or overriding
    if active:
      delta_desired = self.mpc_solution[0].delta[1]
      rate_desired = math.degrees(self.mpc_solution[0].rate[0] * VM.sR)
    else:
      delta_desired = math.radians(angle_steers - angle_offset) / VM.sR
      rate_desired = 0.0

    self.cur_state[0].delta = delta_desired

    self.angle_steers_des_mpc = float(math.degrees(delta_desired * VM.sR) + angle_offset)

    #  Check for infeasable MPC solution
    mpc_nans = np.any(np.isnan(list(self.mpc_solution[0].delta)))
    t = sec_since_boot()
    if mpc_nans:
      self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, CP.steerRateCost)
      self.cur_state[0].delta = math.radians(angle_steers - angle_offset) / VM.sR

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
    plan_send.pathPlan.laneWidth = float(self.LP.lane_width)
    plan_send.pathPlan.dPoly = [float(x) for x in self.LP.d_poly]
    plan_send.pathPlan.lPoly = [float(x) for x in self.LP.l_poly]
    plan_send.pathPlan.lProb = float(self.LP.l_prob)
    plan_send.pathPlan.rPoly = [float(x) for x in self.LP.r_poly]
    plan_send.pathPlan.rProb = float(self.LP.r_prob)

    plan_send.pathPlan.angleSteers = float(self.angle_steers_des_mpc)
    plan_send.pathPlan.rateSteers = float(rate_desired)
    plan_send.pathPlan.angleOffset = float(sm['liveParameters'].angleOffsetAverage)
    plan_send.pathPlan.mpcSolutionValid = bool(plan_solution_valid)
    plan_send.pathPlan.paramsValid = bool(sm['liveParameters'].valid)
    plan_send.pathPlan.sensorValid = bool(sm['liveParameters'].sensorValid)
    plan_send.pathPlan.posenetValid = bool(sm['liveParameters'].posenetValid)

    pm.send('pathPlan', plan_send)

    if LOG_MPC:
      dat = messaging.new_message()
      dat.init('liveMpc')
      dat.liveMpc.x = list(self.mpc_solution[0].x)
      dat.liveMpc.y = list(self.mpc_solution[0].y)
      dat.liveMpc.psi = list(self.mpc_solution[0].psi)
      dat.liveMpc.delta = list(self.mpc_solution[0].delta)
      dat.liveMpc.cost = self.mpc_solution[0].cost
      pm.send('liveMpc', dat)

from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from cereal import car
from cereal import log
from selfdrive.virtualZSS import virtualZSS_wrapper
#from selfdrive.kegman_conf import kegman_conf
from common.realtime import sec_since_boot

def interp_fast(x, xp, fp):  # extrapolates above range, np.interp does not
  return (((x - xp[0]) * (fp[1] - fp[0])) / (xp[1] - xp[0])) + fp[0]

class LatControlPID(object):
  def __init__(self, CP):
    self.pid = PIController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            k_f=CP.lateralTuning.pid.kf, pos_limit=1.0)
    self.angle_steers_des = 0.

    # virtualZSS
    self.model_wrapper = virtualZSS_wrapper.get_wrapper()
    self.model_wrapper.init_model()
    self.output_steer = 0
    self.past_data = []
    self.seq_len = 20 # 40 was WILD
    self.scales = {'zorro_sensor': [-31.666841506958008, 39.42588806152344],
                   'stock_sensor': [-31.0, 37.599998474121094], 'steer_command': [-1.0, 1.0]}


    ###################################################################################################
    # Toyota Pulse Width Modulation (PWM) I.E. Prius Prime
    ###################################################################################################
    # Tune-ables
    self.stuck_ms = 100                        # Declare steering column / TSS1 sensor stuck after x milliseconds
                                               # I'd say 100ms on the low-end, up to maybe 200ms
    #self.sensor_delay = 0.35                   # Prius Prime. Seems like it can be somewhat variable given stiff steering
    #self.steer_delay = 0.1                     # Same as EPS delay (about 0.1)
                                               # Valids are 0.1 -> (len(torque_history) * 2 - 1)

    # Working variables
    self.TSS1 = 0.0                            # Track TSS1 angle for stuck-detection
    self.stuck_debug = True                    # Print debug messages?
    self.stuck_check1 = 1500.0                 # Save the previously recorded angle_steers (start with impossible value)
    #self.stuck_check2 = 2000.0                 # Check a second, adjacent angle to detect rapid oscillations seen in cabana data for STEER_FRACTION
    self.angle_steers_same = False             # Save a total count of near-identical angle_steers values. May not need to be global
    self.stuck_start_time = 9000000000.0       # Track time since angle_steers has shown the same value (start with far future date)
    #self.stuck_torque = 3000
    #self.torque_history = []

    # For pulse width modulator
    self.pulse_start = -1.                      # Start with false value
    self.pulsing = False
    self.pulse_start_first = True
    self.straight_limit = 0.1
    
    
  def reset(self):
    self.pid.reset()

  def update(self, active, v_ego, angle_steers, angle_steers_rate, eps_torque, steer_override, CP, VM, path_plan, driver_torque):
    self.TSS1 = angle_steers      # Save for stuck-detection
    #print "TSS1:", self.TSS1
    #print "v_ego:", v_ego, "a_rate:", angle_steers_rate, "eps:", eps_torque, "over:", steer_override, "d_torq:", driver_torque
    #"CP:", CP, "VM:", VM, "pplan:", path_plan,
    #print "TSS1 Angle?:", angle_steers
    #print "self TSS1 Angle?:", self.angle_steers
    #print "self_o_steer:", self.output_steer

    # virtualZSS
    self.past_data.append([interp_fast(angle_steers, self.scales['stock_sensor'], [0, 1]), self.output_steer])  # steer command is already 'normalized'
    while len(self.past_data) > self.seq_len:
      del self.past_data[0]

    #print "past_data:", self.past_data
	  
    if len(self.past_data) == self.seq_len:
      angle_steers = interp_fast(float(self.model_wrapper.run_model_time_series([i for x in self.past_data for i in x])), [0.0, 1.0], self.scales['zorro_sensor'])

    #print "vZSS:", angle_steers

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






    # retain self.control over time and send out modulated pulses here
    # make this a function?
    pulse_trigger = 0.05   #0.1       # minimum requested torque (factor) to trigger pulse width logic
    pulse_height = 0.15 #0.3=450 (w max 1500) # torque value to start with to overcome friction
    pulse_length = 0.05   #0.1       # length of time (seconds) to max-out to overcome friction
    pulse_window = 0.102                # total time in sec before another pulse_length is allowed


    # Tests to try:
    # -Limit control value (max_torque) just to watch behavior
    #   o car/toyota/carcontroller.py:  STEER_MAX = 1500
    # -Create STEER_MIN?
    #   o The idea here, is that having torque anywhere under 300 is essentially useless (barring angle-faking) because
    #      the sensor remains stuck until 400-450
    # -Need to eventually pass stuck=True/False in
    # -Maybe add

    #      jog control up and down between 400-500
    # -Check whether torque is moving towards zero. Currently, I'm just checking left/right but what if it's
    #   dropping to zero quickly, yet still within the trigger window?
    # -


    # Check for stagnant TSS1 steering sensor
    # Currently check if self.TSS1 is the same for x milliseconds
    if abs((self.TSS1 + 1000) - (self.stuck_check1 + 1000)) < 0.149:   # 0.099  # Within about 0.1 of the saved angle. More precisely, should check if oscillating between two 0.x
      self.angle_steers_same = True
    else:
      self.angle_steers_same = False
      self.stuck_check1 = self.TSS1
      self.stuck_start_time = sec_since_boot()

    # Moving left, or right?
    # could do this with control[x] or angle_des[x] also
    # Compare des against TSS1
    #moving_right = ((self.angle_steers_des + 2000) - (self.TSS1 + 2000)) < -0.15
    # Compare des against vZSS
    moving_right = ((self.angle_steers_des + 2000) - (angle_steers + 2000)) < -0.15

    # If stuck
    ##abs(angle_steers) < 5. and \
    if active and \
              self.angle_steers_same and \
              sec_since_boot() - self.stuck_start_time >= self.stuck_ms * 0.001 and \
              pulse_trigger < abs(output_steer) < pulse_height :

      if self.pulse_start_first:
        self.pulse_start = sec_since_boot()
        self.pulse_start_first = False
      if self.pulsing:
        #print "o_steer:", output_steer
        if moving_right:
          # Pulse less if moving towards zero?
          #if (self.angle_steers_des > 0 and self.TSS1 > 0):
          #  output_steer = 0.9 * -pulse_height
          #if self.angle_steers_des < -3:
          #  output_steer = 1.1 * -pulse_height
          #else:

          # Mod torque +/- to the set -pulse_height (cuz right)
          output_steer = 1.0 * -pulse_height
          #print "Going Right:", output_steer
        # If not much change, do nothing
        elif 0.15 > ((self.angle_steers_des + 2000) - (angle_steers + 2000)) > -0.15:
          # Do nadda
          #print "No movement"
          abcd = 1
        else:  # Left!
          #if (self.angle_steers_des < 0 and self.TSS1 < 0):
          #  output_steer = 0.9 * pulse_height
          #if self.angle_steers_des > 3:
          #  output_steer = 1.1 * pulse_height
          #else:

          output_steer = 1.3 * pulse_height
          #print "Going Left:", output_steer

    else: # cancel pulsing
      self.pulse_start = -1.
      self.pulsing = False
      self.pulse_start_first = True

    # if 'going straight', limit total torque
    #if abs(output_steer) < 1.0  and  abs(self.angle_steers_des) < 2.0:
    #  if output_steer > self.straight_limit:
    #    output_steer = self.straight_limit
    #  elif output_steer < -self.straight_limit:
    #    output_steer = -self.straight_limit

    #if abs(output_steer) != pulse_height:
    #  if output_steer > self.straight_limit:
    #    output_steer = self.straight_limit
    #  elif output_steer < -self.straight_limit:
    #    output_steer = -self.straight_limit

      #elif  0.075 < output_steer < 0.15:
      #   output_steer = 0.15
      #elif  -0.075 < output_steer < -0.15:
      #   output_steer = -0.15

    #if abs (self.angle_steers_des) >= 5.5:
    #  output_steer *= 1.4


    # Need to think about cancelling the pulse if angle_des is < 1.5 now


    # Trigger pulse if desired torque falls within our window  AND stuck AND  (pulse_start=False  OR  withing pulse window)
    #if self.pulsing:
      #if not self.pulsing:
      # Pulse started at this time


    # pulse window status
    if self.pulse_start + pulse_length > sec_since_boot():
      self.pulsing = True
    else:
      self.pulsing = False

    # total window is over, so allow future pulse
    if self.pulse_start + pulse_window < sec_since_boot():
      #self.pulse_start = -1.
      self.pulse_start_first = True


    if active  and  self.stuck_debug:
      #print round(sec_since_boot(), 2), "mph:", int(round(v_ego * 2.237, 1)), "a_des", round(self.angle_steers_des, 2), "a_steers:", round(self.TSS1, 2), "s_time:", round(sec_since_boot() - self.stuck_start_time, 2), "o_steer:", output_steer
      #print "s_torque", self.stuck_torque, "t_hist:", self.torque_history



      #self.output_steer = output_steer
      #print "s_o_steer:", output_steer
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = bool(self.pid.saturated)

    #print round(sec_since_boot(), 2), "mph:", int(round(v_ego * 2.237, 1)), "a_des", round(self.angle_steers_des, 2), "a_steers:", round(self.TSS1, 2)


    self.sat_flag = self.pid.saturated
    return output_steer, float(self.angle_steers_des), pid_log


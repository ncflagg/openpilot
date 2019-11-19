from numpy import clip
import pickle
import csv
import os

# HOW TO
# import this module to where you want to use it, such as:
#   from selfdrive.controls.lib.offset_learner import OffsetLearner
# create the object:
#   self.avg_offset = OffsetLearner(debug=False)
# call the update method:
#   self.avg_offset.update(angle_steers - average_offset, self.LP.d_poly)
# The learned, slow offset saves and loads automatically
#
# ncflagg -=adapted from Zorrobyte code=-


class OffsetLearner:
    def __init__(self, debug=True):

        # Tunables
        self.learning_rate = 400  #400 way too high   #   10 @20Hz. Could've been a hair faster, maybe
        self.slow_learning_rate = 1. / 60000.  # 3600 means learn at max rate of 1 deg in 1 minute at 20Hz
        #200 #2400  #@20Hz. Z was 12000 for slow curvature and said 100Hz

        self.avg_offset = 0.
        self.fast_offset = 0.
        self.frame = 0
        self.frame_print = 0
        self.debug = debug

        try:
            self.learned_offset = pickle.load(open("/data/offset_learned.p", "rb"))
        except (OSError, IOError):
            self.learned_offset = {
                "average": 0.,
            }
            pickle.dump(self.learned_offset, open("/data/offset_learned.p", "wb"))
            os.chmod("/data/offset_learned.p", 0o777)

    # Pass in override too so fast AB can get reset
    # Probably should pass in steer_req too
    def update(self, angle_steers=0., d_poly=None, v_ego=0.):

        # Slowly-learned angle offset
        # Make sure to remove manually set offset(s) elsewhere, except maybe in controlsd
        # Only learn when going straight
        # How to avoid right-drive-bias? I'm still having trouble coming up with a decent way to figure out when the car is rolling +/-
        # Actually, one way I can think of would be to only learn slow AO 35 - 55mph
        # That way, I think there's less likelihood of driving on constantly-cambered roads (at least in my area)
        # Still want to try and read some kind of tilt, though
        # Speed is not a calculation factor yet


        # Trying flipping +/- d_poly around. Seemed to work in my other test, but seems backwards now

        
        if 10.0 < v_ego:  # 15.6 for 35 #35-55mph  24.6 for 55
            if 0.149 < angle_steers:  # Left, and not at center, so correct to the right
                # Line below didn't work for some reason
                #self.learned_offset["average"] = -1.2               # Remove this later
                self.fast_offset += d_poly[3] / self.learning_rate  # d_poly sign is opposite of steering angle
                #if 0 < angle_steers < 2:         # Learn avg going straight
                #    self.learned_offset["average"] += self.slow_learning_rate
            elif angle_steers < -0.149:                             # Right, and not at center, so correct to the left
                self.fast_offset -= d_poly[3] / self.learning_rate  # d_poly sign is opposite of steering angle
                #if -2 < angle_steers < 0:       # Learn avg going straight
                #    self.learned_offset["average"] -= self.slow_learning_rate

            # New AVG idea
            if abs(angle_steers) < 2:  # Going straight
                if 0 < d_poly[3]:      # We're to the right of center (d_poly is opposite sign)
                    self.learned_offset["average"] -= self.slow_learning_rate
                elif d_poly[3] < 0:    # We're to the left of center
                    self.learned_offset["average"] += self.slow_learning_rate

        else:
            self.fast_offset /= 1.01  #1.001  # Taper-off at 100Hz. Needs more at 20Hz
            # Maybe trigger taper if overshoot detected. I.E. track d_poly in a list mebbe
        self.learned_offset["average"] = clip(self.learned_offset["average"], -2.0, 2.0)
        self.avg_offset = self.learned_offset["average"]
        self.fast_offset = clip(self.fast_offset, -0.5, 0.5)

        # New AVG idea
        if -2 < angle_steers < 2:  # Going straight
            if 0 < d_poly[3]:      # We're to the right of center (d_poly is opposite sign)
                self.learned_offset["average"] -= self.slow_learning_rate
            elif d_poly[3] < 0:    # We're to the left of center
                self.learned_offset["average"] += self.slow_learning_rate


        # Fast offset
        # I think the hard part here will be overshoot
        # Rate of change is a bigger concern, but we need it to change quickly
        #  -Might try increments like +/-0.2deg
        #  -But, how do we know when to back it off?
        #  -And what if we pull back too much? Pings and pongs
        # Angle doesn't matter

        #if v_ego > 12.0:  #28mph      # 15.6  # 35mph
        #    if 0.149 < angle_steers:     # Left, and not at center
        #    #if abs(angle_steers - self.avg_offset) > 0:
        #        self.fast_offset -= d_poly[3] / self.learning_rate
        #    elif angle_steers < -0.149:  # Right, and not at center
        #        self.fast_offset += d_poly[3] / self.learning_rate
        #else:
        #    self.fast_offset /= 1.001  # Taper-off at 100Hz. Needs more at 20Hz
        #self.fast_offset = clip(self.fast_offset, -4.0, 4.0)


        self.frame_print += 1
        if self.frame_print >= 5:  # 20 is every second, at 20Hz
            print "avg_offset:", self.avg_offset, "fast_offset:", self.fast_offset
            self.frame_print = 0

        self.frame += 1
        if self.frame >= 200:  #10s @20Hz   #10s @100        # 12000  # every 2 mins
            pickle.dump(self.learned_offset, open("/data/offset_learned.p", "wb"))
            self.frame = 0
        if self.debug:
            with open('/data/offsetdebug.csv', 'a') as csv_file:
                csv_file_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                csv_file_writer.writerow([self.learned_offset, v_ego])

        self.fast_offset = 0. # Disable fast learner
        return self.avg_offset, self.fast_offset

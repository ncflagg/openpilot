from numpy import clip
import pickle
import csv
import os

# HOW TO
# import this module to where you want to use it, such as:
#   from selfdrive.controls.lib.offset_learner import OffsetLearner
# create the object:
#   self.average_offset = OffsetLearner(debug=False)
# call the update method:
#   self.average_offset.update(angle_steers - average_offset, self.LP.d_poly)
# The learned, slow offset saves and loads automatically
#
# ncflagg -=adapted from Zorrobyte code=-


class OffsetLearner:
    def __init__(self, debug=False):

        # Tunables
        self.learning_rate = 8  #400 way too high   #   10 @20Hz. Could've been a hair faster, maybe
        self.slow_learning_rate = 40 #2400  #@20Hz. Z was 12000 for slow curvature and said 100Hz

        self.average_offset = 0.
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
    def update(self, angle_steers=0., d_poly=None, v_ego=0.):

        # Slowly-learned angle offset
        # Make sure to remove manually set offset(s) elsewhere, except maybe in controlsd
        # Only learn when going straight
        # How to avoid right-drive-bias? I'm still having trouble coming up with a decent way to figure out when the car is rolling +/-
        # Actually, one way I can think of would be to only learn slow AO 35 - 55mph
        # That way, I think there's less likelihood of driving on constantly-cambered roads (at least in my area)
        # Still want to try and read some kind of tilt, though
        # Speed is not a calculation factor yet
        
        if 12.4 < v_ego:  # 15.6 for 35 #35-55mph  24.6 for 55
            if angle_steers > 0.1:
                # Line below doens't work for some reason
                #self.learned_offset["average"] = -1.2    # Remove this later
                if abs(angle_steers) < 2.:
                    self.learned_offset["average"] -= d_poly[3] / self.slow_learning_rate
            elif angle_steers < -0.1:
                if abs(angle_steers) < 2.:
                    self.learned_offset["average"] += d_poly[3] / self.slow_learning_rate
        # Why clip it here? Clip is before it's stored?
        self.average_offset = clip(self.learned_offset["average"], -2.0, 2.0)


        # Fast offset
        # I think the hard part here will be overshoot
        # Rate of change is a bigger concern, but we need it to change quickly
        #  -Might try increments like +/-0.2deg
        #  -But, how do we know when to back it off?
        #  -And what if we pull back too much? Pings and pongs
        # Angle doesn't matter

        if v_ego > 12.4:  #28mph      # 15.6  # 35mph
            if abs(angle_steers - self.average_offset) > 0:
                self.fast_offset -= d_poly[3] / self.learning_rate
            else:
                self.fast_offset += d_poly[3] / self.learning_rate
        else:
            self.fast_offset /= 1.001  # Taper-off
        self.fast_offset = clip(self.fast_offset, -4.0, 4.0)

        self.frame_print += 1
        if self.frame_print >= 20:  # every second, at 100Hz
            print "avg_offset:", self.average_offset, "fast_offset:", self.fast_offset
            self.frame_print = 0

        self.frame += 1
        if self.frame >= 200:  #10s @20Hz   #10s @100        # 12000  # every 2 mins
            pickle.dump(self.learned_offset, open("/data/offset_learned.p", "wb"))
            self.frame = 0
        if self.debug:
            with open('/data/offsetdebug.csv', 'a') as csv_file:
                csv_file_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                csv_file_writer.writerow([self.learned_offset, v_ego])

        return self.average_offset, self.fast_offset

from __future__ import division
from selfdrive.virtualZSS import virtualZSS_wrapper
import numpy as np


def interp_fast(x, xp, fp=[0, 1], ext=False):  # extrapolates above range when ext is True
  interped = (((x - xp[0]) * (fp[1] - fp[0])) / (xp[1] - xp[0])) + fp[0]
  return interped if ext else min(max(min(fp), interped), max(fp))

class vZSS():
  def __init__(self):
    self.model_wrapper = virtualZSS_wrapper.get_wrapper()
    self.model_wrapper.init_model()
    self.past_data = []
    self.seq_len = 20
    self.scales = {'angle_steers': [-30.60400009155273, 31.55599975585937],
                   'output_steer': [-1.0, 1.0],
                   'shitty_angle': [-30.70000076293945, 31.20000076293945],
                   'wheel_speeds.fl': [10.594444274902344, 19.36111068725586],
                   'wheel_speeds.fr': [10.63611125946045, 19.299999237060547],
                   'wheel_speeds.rl': [10.586111068725586, 19.344444274902344],
                   'wheel_speeds.rr': [10.61388874053955, 19.288888931274414],
                   'zss': [-22.32206916809082, 41.83000183105469]}

    self.inputs = ['shitty_angle', 'output_steer', 'wheel_speeds.fl', 'wheel_speeds.fr', 'wheel_speeds.rl', 'wheel_speeds.rr']

  def can_predict(self):
    return len(self.past_data) >= self.seq_len

  def save_sample(self, data):  # save sample after prediction because the model was trained to predict next sample's angle, not current angle
    data_normalized = self.normalize(data)
    self.past_data.append(data_normalized)

    while len(self.past_data) > self.seq_len:
      del self.past_data[0]

  def predict(self):
    return interp_fast(float(self.model_wrapper.run_model_time_series(np.array(self.past_data).flatten().tolist())), [0.0, 1.0], self.scales['angle_steers'])

  def normalize(self, data):
    return [interp_fast(d, self.scales[name]) for name, d in zip(self.inputs, data)]

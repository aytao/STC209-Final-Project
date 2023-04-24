import numpy as np

smoothing_factor = 3

weights = [0.2, 0.3, 0.5]

class BufferSmoother:
      
  def __init__(self):
    self.buffer = []
    self.idx = 0

  def add(self, val):
    if len(self.buffer) >= smoothing_factor:
      self.buffer.pop(0)

    self.buffer.append(val)
    
    return

  def avg(self):
    if (len(self.buffer) < smoothing_factor):
      return np.average(self.buffer)

    return np.sum([self.buffer[i] * weights[i] for i in range(smoothing_factor)])

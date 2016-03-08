#!/usr/bin/python

import time
import math
import numpy

class LowPassFilter:
    def __init__(self, dim = 1, frequency = 20):
        self.frequency = frequency
        self.dim = dim
        self.tau = 1.0 / (2.0 * math.pi * frequency)
        self.last_time = time.time()
        self.last_value = numpy.zeros(dim)

    def output(self, value):
        delta_time = time.time() - self.last_time
        a = delta_time / (self.tau + delta_time)
        y = (1 - a) * self.last_value + a * value
        self.last_value = y
        self.last_time += delta_time
        return y

    def reset(self):
        self.frequency = 0.0
        self.dim = 1
        self.tau = 0.0
        self.last_time = time.time()
        self.last_value = numpy.zeros(dim)
        
    def set_frequency(frequency):
        self.frequency = frequency
        self.tau = 1.0 / (2.0 * math.pi * frequency)

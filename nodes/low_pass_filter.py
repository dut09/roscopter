#!/usr/bin/python

import time
import math

class LowPassFilter:
    def __init__(self, frequency = 20):
        self.frequency = frequency
        self.tau = 1.0 / (2.0 * math.pi * frequency)
        self.last_time = time.time()
        self.last_value = 0.0

    def output(self, value):
        delta_time = time.time() - self.last_time
        a = delta_time / (self.tau + delta_time)
        y = (1 - a) * self.last_value + a * value
        self.last_value = y
        self.last_time += delta_time
        return y

    def reset(self):
        self.frequency = 0.0
        self.tau = 0.0
        self.last_time = time.time()
        self.last_value = 0.0
        
    def set_frequency(frequency):
        self.frequency = frequency
        self.tau = 1.0 / (2.0 * math.pi * frequency)

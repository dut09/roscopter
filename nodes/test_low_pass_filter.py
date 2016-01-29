#!/usr/bin/python

from low_pass_filter import *
import matplotlib.pyplot as plt
import numpy as np

f = LowPassFilter(10.0)

# Arrays before and after filtering.
unfiltered = []
filtered = []
timeArray = []

nSamples = 2000
for i in range(nSamples):
    t = time.time()
    a = np.sin(2 * np.pi * t) + 0.1 * np.sin(2 * np.pi * 25 * t + np.pi / 3) 
    b = f.output(a)
    unfiltered.append(a)
    filtered.append(b)
    timeArray.append(t)
    time.sleep(0.001)

t0 = timeArray[0]
timeArray[:] = [t - t0 for t in timeArray]

# Plot the results
plt.plot(timeArray, unfiltered, 'r')
plt.plot(timeArray, filtered, 'g')
plt.xlabel('time')
plt.axis('tight')
plt.show()

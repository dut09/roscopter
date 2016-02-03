#!/usr/bin/python

import matplotlib.pyplot as plt
import numpy as np

output_file = open('signals.txt', 'r')

desired_roll = []
desired_pitch = []
z = []
actual_roll = []
actual_pitch = []
actual_yaw = []
for line in output_file.readlines():
    data = line.split()
    desired_roll.append(float(data[0]))
    desired_pitch.append(float(data[1]))
    z.append(float(data[2]))
    actual_roll.append(float(data[3])) 
    actual_pitch.append(float(data[4]))
    actual_yaw.append(float(data[5]))

actual_roll_rate = []
actual_roll_rate.append(0.0)
for i in range(len(actual_roll) - 1):
    actual_roll_rate.append((actual_roll[i + 1] - actual_roll[i]) * 50.0)

plt.figure()
plt.plot(actual_roll)
plt.xlabel('time')
plt.ylabel('actual roll')
plt.axis('tight')
plt.show()

plt.figure()
plt.plot(actual_roll_rate)
plt.xlabel('time')
plt.ylabel('actual roll rate')
plt.axis('tight')
plt.show()

#!/usr/bin/python

from PID import *
import matplotlib.pyplot as plt
import numpy as np

pid_controller = PID(10.0, 0.0, 1.0, 1.0)

set_position = 1.0
actual_position = []
actual_velocity = []
force = []
clock = []

sample_num = 500
dt = 0.02
pid_controller.SetPoint = set_position

# Intentionally sleep for 2 seconds to test whether we handle the initial time
# correctly in I and D term.
time.sleep(2.0)
for i in range(sample_num):
    t = time.time()
    position = 0.0 if i == 0 else actual_position[i - 1]
    pid_controller.update(position)
    f = pid_controller.output
    velocity = 0.0 if i == 0 else actual_velocity[i - 1]
    velocity += f * dt
    position += velocity * dt

    # Update.
    actual_velocity.append(velocity)
    actual_position.append(position)
    force.append(f)
    clock.append(t)

    time.sleep(dt)

t0 = clock[0]
clock[:] = [t - t0 for t in clock]

plt.plot(clock, actual_position)
plt.xlabel('time')
plt.show()

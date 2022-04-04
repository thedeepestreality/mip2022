import pybullet as p
import time
from scipy.integrate import odeint
import math
import copy
import numpy as np
from scipy.optimize import minimize
from random import random

IS_GUI = False
# physical params
dt = 1/240

q0 = math.pi-0.1
pos_d = math.pi
maxTime = 10
t = 0
# joint index
jIdx = 1
g = 10
K = np.array([[-136.,   -19.1]])

if (IS_GUI):
    physicsClient = p.connect(p.GUI)
else:
    physicsClient = p.connect(p.DIRECT)

p.setGravity(0, 0, -g)
bodyId = p.loadURDF("./pendulum.urdf")

# turn off link damping
p.changeDynamics(bodyUniqueId=bodyId,
                linkIndex=jIdx,
                linearDamping=0)

# go to initial pose
p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = jIdx,
                        targetPosition = q0,
                        controlMode = p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# compare initial state
q0_fact = p.getJointState(bodyId, jIdx)[0]
print(f'q0 fact: {q0_fact}')
print(f'q0 error: {q0 - q0_fact}')
pos0 = [q0_fact, 0]

# containters for logging and plots
log_time = [t]
log_pos = [q0_fact]

# turn off control torque for free fall
p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = jIdx,
                        controlMode = p.VELOCITY_CONTROL,
                        targetVelocity = 0,
                        force = 0)
e_int = 0
while t <= maxTime:
    p.stepSimulation()
    pos = p.getJointState(bodyId, jIdx)[0]
    vel = p.getJointState(bodyId, jIdx)[1]
    t += dt
    kp = -K[0,0]
    kv = -K[0,1]
    ki = 80
    e = pos - pos_d
    e_int += e*dt
    # u = -kp*e - kv*vel - ki*e_int
    u = -kp*e -kv*vel
    p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = jIdx,
                        controlMode = p.TORQUE_CONTROL,
                        force = u)

    # TODO switch to preallocated indexing
    # log_pos[idx] = pos
    log_pos.append(pos)
    log_time.append(t)
    if (IS_GUI):
        time.sleep(dt)
p.disconnect()

# show plots
import matplotlib.pyplot as plt
plt.plot(log_time, log_pos, label='sim')
plt.grid(True)
plt.plot([0,maxTime],[pos_d, pos_d], label="reference")
plt.show()
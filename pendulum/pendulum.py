import pybullet as p
import time
from scipy.integrate import odeint
import math
import copy
import numpy as np
from scipy.optimize import minimize
from random import random

IS_GUI = True
# physical params
dt = 1/240
q0 = 0
pos_d = math.pi/2
maxTime = 5
t = 0
# joint index
jIdx = 1
g = 10
L = 0.8
m = 1
kf = 0.1
a = g/L
c = m*L*L
b = kf/c

K = np.array([[200,  30]])
(mx,mv,vx) = (-52.921769241021074, -10.326402285952112, -128.06248474865563)
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

kp = K[0,0]
kv = K[0,1]
ki = 80

# u = K@X
def ctrl_static(pos, vel):
    return -kp*(pos-pos_d) - kv*vel

def feedback_lin(pos, vel,t):
	u_nonlin = (a*math.sin(pos)+b*vel)*c
	u_lin = -kp*(pos-pos_d)-kv*vel
	return u_nonlin + u_lin

glob = {"prev_vel": 0}
def feedback_ast(pos, vel):
    dv = (vel - glob["prev_vel"])/dt
    glob["prev_vel"] = vel
    return mx*vel + mv*dv + vx*(pos-pos_d)

# containters for logging and plots
log_time = [t]
log_pos = [q0_fact]
log_vel= [0]
log_ctrl = []
u = 0

# turn off control torque for free fall
p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = jIdx,
                        controlMode = p.VELOCITY_CONTROL,
                        targetVelocity = 0,
                        force = 0)
e_int = 0
while t <= maxTime:
    pos = p.getJointState(bodyId, jIdx)[0]
    vel = p.getJointState(bodyId, jIdx)[1]
    
	# PID-regulator
    # e = pos - pos_d
    # e_int += e*dt
    # u = -kp*e - kv*vel - ki*e_int

    # u = ctrl_static(pos, vel)
    # u = feedback_lin(pos, vel, t)
    du = feedback_ast(pos, vel)
    u += du*dt
    p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = jIdx,
                        controlMode = p.TORQUE_CONTROL,
                        force = u)
    p.stepSimulation()
    t += dt
    # TODO switch to preallocated indexing
    # log_pos[idx] = pos
    log_pos.append(pos)
    log_vel.append(vel)
    log_ctrl.append(u)
    log_time.append(t)
    if (IS_GUI):
        time.sleep(dt)
p.disconnect()

# show plots
import matplotlib.pyplot as plt

# position plot
plt.subplot(3,1,1)
plt.plot(log_time, log_pos, label='sim_pos')
plt.grid(True)
plt.plot([0,maxTime],[pos_d, pos_d], label="reference")

# velocity plot
plt.subplot(3,1,2)
plt.plot(log_time, log_vel, label='sim_vel')
plt.grid(True)

# control plot
plt.subplot(3,1,3)
plt.plot(log_time[0:-1], log_ctrl, label='control')
plt.grid(True)

plt.show()
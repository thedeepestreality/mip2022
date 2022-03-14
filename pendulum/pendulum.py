import pybullet as p
import time
from scipy.integrate import odeint
import math
import copy
import numpy as np

IS_GUI = False
# physical params
dt = 1/240
g = 10
L = 0.8
m = 1
q0 = 0.78
maxTime = 10
t = 0
# joint index
jIdx = 1

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
print(f'q0 error: {q0 - q0_fact}')

# containters for logging and plots
log_time = [t]
log_pos = [q0_fact]

# turn off control torque for free fall
p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = jIdx,
                        controlMode = p.VELOCITY_CONTROL,
                        targetVelocity = 0,
                        force = 0)
while t <= maxTime:
    p.stepSimulation()
    pos = p.getJointState(bodyId, jIdx)[0]
    t += dt
    # TODO switch to preallocated indexing
    # log_pos[idx] = pos
    log_pos.append(pos)
    log_time.append(t)
    if (IS_GUI):
        time.sleep(dt)
p.disconnect()

# right part of the pendulum DE
def rp(x, t):
    return [x[1], -g/L*math.sin(x[0])]

def symp_euler(fun, x0, TT):
    x1 = copy.copy(x0)
    xx = np.array(x1)
    for i in range(len(TT)-1):
        dt = (TT[i+1] - TT[i])
        x1[1] += rp(x1, 0)[1]*dt
        x1[0] += x1[1]*dt
        xx = np.vstack((xx,x1))
    return xx

# integrate pendulum DE
Y = odeint(rp,[q0, 0], log_time)
log_de = Y[:,0]

log_euler = symp_euler(rp, [q0, 0], log_time)
log_euler = log_euler[:,0]

def cost(q_exp, q_theor):
    l2 = 0
    sz = len(q_exp)
    linf = abs(q_exp[0] - q_theor[0])
    for i in range(sz):
        err = abs(q_exp[i] - q_theor[i])
        if (err > linf):
            linf = err
        l2 += err**2
    l2 = math.sqrt(l2)
    return (l2, linf)

(l2, linf) = cost(log_pos, log_de)
print(f'l2 odeint = {l2}')
print(f'linf odeint = {linf}')

(l2, linf) = cost(log_pos, log_euler)
print(f'l2 euler = {l2}')
print(f'linf euler = {linf}')

# show plots
import matplotlib.pyplot as plt
plt.plot(log_time, log_pos)
plt.grid(True)
plt.plot(log_time, log_euler)
plt.show()
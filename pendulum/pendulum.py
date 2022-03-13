import pybullet as p
import time
from scipy.integrate import odeint
import math

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
# containters for logging and plots
log_time = [t]
log_pos = [q0]

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

# integrate pendulum DE
Y = odeint(rp,[q0, 0], log_time)
log_de = Y[:,0]

# show plots
import matplotlib.pyplot as plt
plt.plot(log_time, log_pos)
plt.grid(True)
plt.plot(log_time, log_de)
plt.show()
from control.matlab import place, lqr 
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

A = np.array([[-0.3176, 0.852, 0],
            [-0.0102, -0.1383, 0],
            [0, 1, 0]])
B = np.vstack((-0.005,
                -0.0217,
                0))

x = np.array([0,0,0])
xd = np.array([0,0,0])
# xd = np.array([0,10*np.pi/180,0])
u = 0

Q = np.diag([1,100,100])
R = 0.01
K, *_ = lqr(A,B,Q,R)

T = 40
dt = 0.01
TT = np.arange(0, T, dt)
int_span = np.array([0,dt])
sz = len(TT)
x_log = np.zeros((sz,3))
u_log = np.zeros(sz)

for idx in range(sz):
    x_log[idx,:] = x
    u = -K.reshape(3,)@(x-xd)
    u = u[0,0]
    u_log[idx] = u
    x = odeint(lambda vv,tt,uu: A@vv+B.reshape(3,)*uu + np.array([0,0.1,0]), x, int_span, args=(u,))[-1,:]

x_log *= 180/np.pi
plt.figure(1)
plt.subplot(3,1,1)
plt.plot(TT,x_log[:,0],label='beta')
plt.legend()
plt.grid()

plt.subplot(3,1,2)
plt.plot(TT,x_log[:,1],label='w')
plt.legend()
plt.grid()

plt.subplot(3,1,3)
plt.plot(TT,x_log[:,2],label='phi')
plt.legend()
plt.grid()

plt.figure(2)
plt.plot(TT,u_log,label='ctrl')
plt.legend()
plt.grid()

plt.show()
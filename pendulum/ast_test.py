from control.matlab import place, lqr 
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

g = 10
L = 0.8
m = 1
kf = 0.1
a = g/L
b = kf/(m*L*L)
dt = 1/240

aa = -a
bb = -b
cc = 1/(m*L*L)
A = np.array([[0, 1, 0],
            [aa, bb, cc],
			[0, 0, 0]])
B = np.array(([0], [0], [1]))

poles = np.array([-3, -2, -1])
K = place(A,B,poles)
print(f"place: {K}")

Q = np.diag([100,20,1])

R = 0.01
K, *_ = lqr(A, B, Q, R)
print(f"lqr: {K}")

poles_fact = np.linalg.eig(A-B@K)
print(poles_fact)

x = np.array([0,0,0])
xd = np.array([0,0,0])
u = 0

T = 20
dt = 0.01
TT = np.arange(0, T, dt)
int_span = np.array([0,dt])
sz = len(TT)
x_log = np.zeros((sz,3))
u_log = np.zeros(sz)
kx = K[0,0]
kv = K[0,1]
ku = K[0,2]
mx = ku*bb/cc-kv
mv = -ku/cc
vx = ku*aa/cc-kx
print(f"{(mx,mv,vx)}")
v_prev = x[1]
for idx in range(sz):
	x_log[idx,:] = x
	
	du = -K@(x-xd)
	dx = x[1]
	dv = (dx-v_prev)/dt
	v_prev = dx
	du = mx*dx + mv*dv + vx*(x[0]-0.1)
	# du = -kx*(x[0]-0.1) -kv*x[1] -ku*u
	u += du*dt
	u_log[idx] = du
	x = odeint(lambda vv,tt,uu: A@vv+B.reshape(3,)*uu + np.array([0,0.1,0]), x, int_span, args=(du,))[-1,:]

plt.figure(1)
plt.plot(TT, x_log[:,0])
plt.grid(True)
plt.show()
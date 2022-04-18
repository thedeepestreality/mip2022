from control.matlab import place, lqr 
import numpy as np

g = 10
L = 0.8
m = 1
kf = 0.1
a = -g/L
b = kf/(m*L*L)

# A = np.array([[0, 1],
#             [-a, -b]])
# B = np.array(([0], [1/(m*L*L)]))
# poles = np.array([-10,-20])
# K = -place(A,B,poles)
# print(f"place: {K}")

# Q = np.diag([100,1])
# R = 0.1
# K, *_ = lqr(A, B, Q, R)
# print(f"lqr: {K}")

# poles_fact = np.linalg.eig(A-B@K)
# print(poles_fact)

A = np.array([[0, 1],
            [0, 0]])
B = np.array(([[0], [1]]))
poles = np.array([-10,-20])
K = place(A,B,poles)
print(f"place: {K}")

poles_fact = np.linalg.eig(A-B@K)
print(poles_fact)
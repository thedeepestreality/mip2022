from control.matlab import place
import numpy as np

g = 10
L = 0.8
m = 1
kf = 0.1
a = -g/L
b = kf/(m*L*L)

A = np.array([[0, 1],
            [-a, -b]])
B = np.array(([0], [1/(m*L*L)]))
poles = np.array([-10,-20])
K = -place(A,B,poles)
print(K)

poles_fact = np.linalg.eig(A+B@K)
print(poles_fact)
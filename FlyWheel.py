import numpy as np
import control
import matplotlib.pyplot as plt
# from sympy import *

inertia_motor = 0.10         #kg*m*m
resistance_motor = 1.1      #N*m*s
stiffness_shaft = 10000.0       #N*m/rad
inertia_wheel = 1.15         #kg*m*m
resistance_wheel = 0.5      #N*m*s

A11 = -resistance_motor / inertia_motor
A12 = -1 / inertia_motor
A21 = stiffness_shaft
A23 = -stiffness_shaft
A32 = 1 / inertia_wheel
A33 = -resistance_wheel / inertia_wheel

A = np.zeros((3,3))
A[0,0] = A11
A[0,1] = A12
A[1,0] = A21
A[1,2] = A23
A[2,1] = A32
A[2,2] = A33


B11 = 1 / inertia_motor

B = np.zeros((3,1))
B[0,0] = B11


C = np.zeros((1,3))
C[0,0] = 1
C = np.identity(3)

D = np.zeros((3,1))
# print(A)
# print(B)
print(C)

veh_dyn_sys = control.StateSpace(A,B,C,D)
print( np.linalg.matrix_rank(control.ctrb(A,B)) )
print( np.linalg.matrix_rank(control.obsv(A,C)) )

# t, yout = control.impulse_response(veh_dyn_sys)

t, yout = control.step_response(veh_dyn_sys)

plt.plot(t,yout[0,0])
plt.plot(t,yout[2,0])
plt.legend(["motor","wheel"])
plt.show()


import numpy as np
import control
import matplotlib.pyplot as plt
# from sympy import *

inertia_motor = 0.10         #kg*m*m
resistance_motor = 1.1      #N*m*s
stiffness_motor_shaft = 100000.0       #N*m/rad
gear_ratio = 0.8           # - 
inertia_gears = 1.15         #N*m*s
resistance_gears = 1.15         #kg*m*m
stiffness_gear_shaft = 100000.0       #N*m/rad
inertia_wheel = 30.5         #kg*m*m
resistance_wheel = 3.50      #n/m*m
stiffness_wheel = 1000.0       #N*m/rad
radius_wheel = 0.2          #meters
mass_vehicle = 2444            #kg (Tesla Model)

n = int(5)   #number of states
m = int(1)   #number of inputs

A11 = -resistance_motor / inertia_motor
A12 = -1 / inertia_motor
A21 = stiffness_motor_shaft
A23 = -stiffness_motor_shaft
A32 = 1 / inertia_gears
A33 = -resistance_gears / inertia_gears
A34 = -1 / (inertia_gears * gear_ratio)
A43 = stiffness_gear_shaft / gear_ratio
A45 = -stiffness_gear_shaft
A54 = 1 / inertia_wheel
A55 = -resistance_wheel / inertia_wheel

A = np.zeros((n,n))
A[0,0] = A11
A[0,1] = A12
A[1,0] = A21
A[1,2] = A23
A[2,1] = A32
A[2,3] = A34
A[3,2] = A43
A[3,4] = A45
A[4,3] = A54
A[4,4] = A55

B11 = 1 / inertia_motor

B = np.zeros((n,m))
B[0,0] = B11

C = np.zeros((1,n))
C[0,0] = 1
C = np.identity(n)

D = np.zeros((n,m))
# print(A)
# print(B)
print(C)

veh_dyn_sys = control.StateSpace(A,B,C,D)
print( np.linalg.matrix_rank(control.ctrb(A,B)) )
print( np.linalg.matrix_rank(control.obsv(A,C)) )

t, yout = control.step_response(veh_dyn_sys,input=0)


# plt.plot(t,yout[0,0])
# plt.plot(t,yout[2,0])
# plt.plot(t,yout[4,0])
# plt.legend(["motor", "gear", "hub"])
plt.plot(t,yout[1,0])
plt.plot(t,yout[3,0])


plt.show()


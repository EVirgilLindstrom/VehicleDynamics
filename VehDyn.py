import numpy as np
import control
import matplotlib.pyplot as plt
# from sympy import *

inertia_motor = 0.10         #kg*m*m
resistance_motor = 0.01      #N*m*s
stiffness_motor = 100000.0       #N*m/rad
gear_ratio = 1.0            # - 
inertia_gears = 0.15         #kg*m*m
stiffness_gears = 100000.0       #N*m/rad
inertia_wheel = 3.5         #kg*m*m
resistance_wheel = 0.50      #n/m*m
stiffness_wheel = 1000.0       #N*m/rad
radius_wheel = 0.2          #meters
vehicle_mass = 2444            #kg (Tesla Model)

A11 = -resistance_motor / inertia_motor
A12 = -1 / inertia_motor
A21 = stiffness_motor
A23 = -stiffness_motor / gear_ratio
A32 = 1 / (gear_ratio * inertia_gears)
A34 = -1 / inertia_gears
A43 = stiffness_gears
A45 = -stiffness_gears
A54 = 1 / inertia_wheel
A55 = -resistance_wheel / inertia_wheel
A56 = -1 / inertia_wheel
A65 = stiffness_wheel
A67 = -stiffness_wheel / radius_wheel
A76 = -1 / (vehicle_mass * radius_wheel)
A77 = resistance_wheel / (vehicle_mass * radius_wheel * radius_wheel)

A = np.zeros((7,7))
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
A[4,5] = A56
A[5,4] = A65
A[5,6] = A67
A[6,5] = A76
A[6,6] = A77

B11 = 1 / inertia_motor
B12 = 1 / inertia_motor
B73 = -1 / vehicle_mass

B = np.zeros((7,3))
B[0,0] = B11
B[0,1] = B12
B[6,2] = B73

C = np.zeros((1,7))
C[0,0] = 1
C = np.identity(7)

D = np.zeros((7,3))
# print(A)
# print(B)
print(C)

veh_dyn_sys = control.StateSpace(A,B,C,D)
print( np.linalg.matrix_rank(control.ctrb(A,B)) )
print( np.linalg.matrix_rank(control.obsv(A,C)) )

t, yout = control.impulse_response(veh_dyn_sys)

plt.scatter(t,yout[3,0])
plt.show()


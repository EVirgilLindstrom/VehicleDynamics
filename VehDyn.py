import numpy as np
import control
import matplotlib.pyplot as plt
# from sympy import *

inertia_motor = 0.10         #kg*m*m
resistance_motor = 1.1      #N*m*s
stiffness_motor_shaft = 100000.0       #N*m/rad
gear_ratio = 1.0            # - 
inertia_gears = 1.15         #N*m*s
resistance_gears = 1.15         #kg*m*m
stiffness_gear_shaft = 100000.0       #N*m/rad
inertia_wheel = 30.5         #kg*m*m
resistance_wheel = 3.50      #n/m*m
stiffness_wheel = 1000.0       #N*m/rad
radius_wheel = 0.2          #meters
mass_vehicle = 2444            #kg (Tesla Model)

n = int(7)   #number of states
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
A56 = -1 / inertia_wheel
A65 = stiffness_wheel
A66 = -stiffness_wheel / resistance_wheel
A67 = -stiffness_wheel / radius_wheel
A76 = 1 / (mass_vehicle * radius_wheel)

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
A[4,5] = A56
A[5,4] = A65
A[5,6] = A67
A[6,5] = A76

B11 = 1 / inertia_motor
B12 = 1 / inertia_motor
B73 = -1 / mass_vehicle

B = np.zeros((7,1))
B[0,0] = B11

C = np.zeros((1,7))
C[0,0] = 1
# C = np.identity(7)

D = np.zeros((1,1))
# print(A)
# print(B)
# print(C)

veh_dyn_sys = control.StateSpace(A,B,C,D)

K = np.array([[0, 0, 0, 0, 0, 0, 0]])
closed_loop_sys = control.StateSpace(A - B @ K, B, C, D)
# print( np.linalg.matrix_rank(control.ctrb(A,B)) )
# print( np.linalg.matrix_rank(control.obsv(A,C)) )

time = np.linspace(0, 5, 1000)

# Step response of the closed-loop system
time, response = control.step_response(closed_loop_sys, time)

# Plot the step response
plt.plot(time, response)
plt.xlabel('Time')
plt.ylabel('System Response')
plt.title('Full-State Feedback Controller Step Response')
plt.grid(True)
plt.show()

t, yout = control.step_response(veh_dyn_sys,input=0)

# veh_dyn_sys.

# plt.plot(t,yout[0,0])
# plt.plot(t,yout[2,0])
# plt.plot(t,yout[4,0])
# plt.plot(t,yout[1,0])
# plt.plot(t,yout[3,0])
# plt.plot(t,yout[5,0])
# plt.plot(t,yout[0,0])

# plt.show()


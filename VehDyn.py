import numpy as np
import control
import matplotlib.pyplot as plt
# from sympy import *

def loop(ss, x0, t0, u, dt=None):
    if dt:
        t, x1 = control.forced_response(ss,U=u,T=[t0,dt],X0=x0)
    else:
        t, x1 = control.forced_response(ss,U=[u,u],T=[t0,t0+0.01],X0=x0)
    return x1


inertia_motor = 0.10                #kg*m*m
resistance_motor = 1.1              #N*m*s
stiffness_motor_shaft = 100000.0    #N*m/rad
gear_ratio = 1.0                    # - 
inertia_gears = 1.15                #N*m*s
resistance_gears = 1.15             #kg*m*m
stiffness_gear_shaft = 100000.0     #N*m/rad
inertia_wheel = 30.5                #kg*m*m
resistance_wheel = 3.50             #n/m*m
stiffness_wheel = 1000.0            #N*m/rad
radius_wheel = 0.2                  #meters
mass_vehicle = 2444                 #kg (Tesla Model)
drag_coeficient = 0.219         

n = int(7)   #number of states
m = int(2)   #number of inputs

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

B = np.zeros((n,m))
B[0,0] = B11
B[6,1] = B73

C = np.zeros((1,n))
C[0,6] = 1
C = np.identity(n)

D = np.zeros((n,m))
# print(A)
# print(B)
# print(C)

# Define the desired closed-loop poles
desired_poles = np.array([-10, -20, -30, -40 ,-50, -60])

# Compute the state feedback gain matrix using the Ackermann formula
# K = control.acker(A, B, desired_poles)

veh_dyn_sys = control.StateSpace(A,B,C,D)
# t, yout = control.step_response(veh_dyn_sys,input=0)
# plt.plot(t,yout[0,0])
# plt.show()

time = np.linspace(0, 200, 20001)
x0 = np.zeros(n)
x_states = [0]
u = 1000.0 * np.ones((20001,0))
drag_force = 0


for t in time:

    x0 = loop(veh_dyn_sys,x0,t,[1000,drag_force])
    x0 =x0[:,1]
    x_states.append(x0[6])
    drag_force = drag_coeficient * x0[6] * x0[6]

# t, xout = control.forced_response(veh_dyn_sys, T=time,U=u)
plt.plot(x_states)
plt.show()

# print(veh_dyn_sys.poles)
K = np.array([[10, 10, 0.1, 0.1, 0.1, 0.1, 0.1]])
closed_loop_sys = control.StateSpace(A - B @ K, B, C, D)
# print( np.linalg.matrix_rank(control.ctrb(A,B)) )
# print( np.linalg.matrix_rank(control.obsv(A,C)) )

time = np.linspace(0, 10, 1001)
u = 1.0 * np.ones(1001)
# Step response of the closed-loop system
# time, response = control.step_response(closed_loop_sys, time)


T, xout = control.forced_response(veh_dyn_sys, T=time,U=u)

# Plot the step response
plt.plot(T, xout)
plt.xlabel('Time')
plt.ylabel('System Response')
plt.title('Full-State Feedback Controller Step Response')
plt.grid(True)
plt.show()

t, yout = control.step_response(veh_dyn_sys,input=0)

# veh_dyn_sys.



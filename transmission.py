#!/usr/bin/env python3

from sympy import *

# masses of robot arms
m1, m2 = symbols("m1 m2")

# lengths of robot arms
l1, l2 = symbols("l1 l2")

# acceleration due to gravity
g = symbols("g")

# angles at robot joints
q1, q2 = symbols("q1 q2")

# 1st derivatives of angles
dq1, dq2 = symbols("dq1 dq2")

# 2nd derivatives of angles
ddq1, ddq2 = symbols("ddq1 ddq2")
ddq = Matrix([ddq1,
              ddq2])

# control input torque
u1, u2 = symbols("u1 u2")
U = Matrix([u1, u2])

# matrix coefficients for intertia matrix M(q)
M11 = (m1+m2)*l1**2 + m2*l2**2 + 2*m2*l1*l2*cos(q2)
M12 = m2*l2**2 + m2*l1*l2*cos(q2)
M21 = m2*l2**2 + m2*l1*l2*cos(q2)
M22 = m2*l2**2
M = Matrix([[M11, M12], [M21, M22]])

# matrix coefficients for coriolis matrix C(q, dq)
C11 = -m2*l1*l2*(2*dq1*dq2 + dq2**2)*sin(q2)
C21 = m2*l1*l2*dq1**2*sin(q2)
C = Matrix([C11, C21])

# matrix coefficients for gravity vector G(q)
G11 = (m1 + m2)*g*l1*cos(q1) + m2*g*l2*cos(q1+q2)
G21 = m2*g*l2*cos(q1+q2)
G = Matrix([G11, G21])

# dynamic equations of robot
eq = Eq(U, M*ddq + C + G)

sol = solve(eq, (ddq1, ddq2))

print("ddq1 = {} \n\nddq2 = {}".format(sol[ddq1], sol[ddq2]))
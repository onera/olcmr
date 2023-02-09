#!/usr/bin/python3
from math import pi
import cs_tools as cs

# Conversion from ENU to NED
qz = cs.Quaternion(pi/2, [0, 0, 1])     # Rot(pi/2, z)
qx = cs.Quaternion(pi, [1, 0, 0])       # Rot(pi, x)
Q = qz*qx
print("ENU->NED:\n{}".format(Q))

# Test
print("Heading in ENU = 20 deg")
q_enu = cs.Quaternion(cs.Euler(cs.radian(20), 0, 0))
q_ned = Q*q_enu
print("Heading in NED = {:.2f} deg".format(cs.degree(q_ned.euler().yaw)))

angles_deg = cs.Euler(70, 10, 20)
print("Euler in ENU =\n{}".format(angles_deg))
q_enu = cs.Quaternion(cs.radian(angles_deg))
q_ned = Q*q_enu
angles = q_ned.euler();
print("Euler in NED =\n{}".format(cs.degree(angles)))

if angles.roll > pi/2:
    angles.roll -= pi
elif angles.roll < -pi/2:
    angles.roll += pi

if angles.pitch > pi/2:
    angles.pitch -= pi
elif angles.pitch < -pi/2:
    angles.pitch += pi

print("Euler in NED (up)=\n{}".format(cs.degree(angles)))

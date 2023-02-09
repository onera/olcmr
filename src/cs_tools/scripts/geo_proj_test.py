#!/usr/bin/python3
import cs_tools
import math

# caylsu camp
origin = cs_tools.GeoPoint(44.2743, 1.7285, 361.0)
print("Origin:\n{}".format(origin))

print("===================== ECEF ==========================")
proj = cs_tools.GeoProj(cs_tools.WGS84, origin)

o_ecef = proj.spherical_to_ecef(origin)
print("Origin in ECEF:\n{}".format(o_ecef))

o_lla = proj.ecef_to_spherical(o_ecef)
print("Origin in LLA:\n{}".format(o_lla))

p_ecef = o_ecef
p_ecef.x +=10
p_ecef.y +=10
p_ecef.z +=10
print("add offset (10.0, 10.0, 10.0):\n{}".format(p_ecef))

p_lla = proj.ecef_to_spherical(p_ecef)
print("Offset in LLA:\n{}".format(p_lla))

print("===================== LPT ==========================")
p = cs_tools.Point(10.0, 10.0, 2.0)
print("Local position:\n{}".format(p))

p_ecef = proj.local_to_ecef(p, cs_tools.ENU)
print("Local position in ecef:\n{}".format(p_ecef))

p_lla = proj.ecef_to_spherical(p_ecef)
print("Local position in LLA:\n{}".format(p_lla))

# Direct
p_lla = proj.local_to_spherical(p, cs_tools.ENU)
print("Local position in LLA (direct):\n{}".format(p_lla))

p_local = proj.spherical_to_local(p_lla, cs_tools.ENU)
print("Back to local position (direct):\n{}".format(p_local))

print("===================== distance =====================")
G1 = cs_tools.GeoPoint(44.2743, 1.7285, 361.0)
P1 = proj.spherical_to_local(G1, cs_tools.ENU)
P2 = cs_tools.Point()
P2.x = P1.x + 10
P2.y = P1.y + 10
P2.z = P1.z + 5
G2 = proj.local_to_spherical(P2,cs_tools.ENU)
dist_1 = math.sqrt(10*10+10*10+5*5)
dist_2 = cs_tools.distance(P1, P2)
dist_3 = cs_tools.distance(G1, G2)
print("Distance - expected : {}".format(dist_1))
print("Distance - local    : {}".format(dist_2))
print("Distance - spherical: {}".format(dist_3))


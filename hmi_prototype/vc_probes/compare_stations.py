"""Station 1 vs station 2 frame at the robot side, both at chuck 1.838 deg (same robot pose)."""
import json
import numpy as np
from probe_math import station_frame, rot_angle_axis

a = json.load(open("vc_probes/stn1_T2.json"))
b = json.load(open("vc_probes/stn2_T2.json"))
F1 = station_frame(a["W0"], a["S0"])
F2 = station_frame(b["W0"], b["S0"])
D = np.linalg.inv(F1) @ F2
ang, ax = rot_angle_axis(D[:3, :3])
np.set_printoptions(precision=3, suppress=True)
print("STN1 origin", F1[:3, 3], " STN2 origin", F2[:3, 3])
print("origin offset (world, mm):", F2[:3, 3] - F1[:3, 3], " |d| = %.3f mm" % np.linalg.norm(F2[:3, 3] - F1[:3, 3]))
print("orientation difference: %.4f deg about STN1 axis %s" % (ang, ax))
print("same station-relative point, world gap: %.3f mm" % np.linalg.norm((F2 @ np.r_[a["S0"][0], 1])[:3] - (F1 @ np.r_[a["S0"][0], 1])[:3]))

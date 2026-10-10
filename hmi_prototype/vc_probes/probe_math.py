"""Station frame in world from a probe pair: T_world_station = T_world_tcp * inv(T_station_tcp)."""
import json
import sys

import numpy as np


def quat_to_R(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def T(rt):
    M = np.eye(4)
    M[:3, :3] = quat_to_R(np.array(rt[1], float))
    M[:3, 3] = rt[0]
    return M


def station_frame(world, stn):
    return T(world) @ np.linalg.inv(T(stn))


def rot_angle_axis(R):
    ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return ang, ax / (np.linalg.norm(ax) or 1)


if __name__ == "__main__":
    d = json.load(open(sys.argv[1]))
    F0 = station_frame(d["W0"], d["S0"])
    F1 = station_frame(d["W1"], d["S1"])
    ang, ax = rot_angle_axis(F1[:3, :3] @ F0[:3, :3].T)
    np.set_printoptions(precision=3, suppress=True)
    print("station-relative TCP change  : %.3f mm" % np.linalg.norm(np.subtract(d["S1"][0], d["S0"][0])))
    print("world TCP change             : %.3f mm" % np.linalg.norm(np.subtract(d["W1"][0], d["W0"][0])))
    print("station frame rotated by     : %.3f deg about world axis %s (chuck moved %.3f deg)"
          % (ang, ax, d["c1"] - d["c0"]))
    print("station frame origin at c0   :", F0[:3, 3], " z-axis:", F0[:3, 2])
    print("station frame origin at c1   :", F1[:3, 3], " z-axis:", F1[:3, 2])

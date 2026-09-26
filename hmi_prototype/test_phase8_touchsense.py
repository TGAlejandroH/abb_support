"""Touch-sense P1: the auto touch-up wire (ids 16/18/19) and the HMI's offset math.

Run:  python -m unittest discover -s hmi_prototype -v

Two layers, as for the other phases:
  1. TestAutoTouchupOffset - abb_server.auto_touchup_offset, the prototype's
     mirror of the HMI's AutoTouchUpsOffset (WeldLibrary.cpp): hand-computed
     1-D/2-D/3-D cases, a rotated frame cross-checked against an independent
     rotation matrix, and the HMI's own quirks (per-axis, last touch wins).
  2. FakeTouchWireRobot - the executable spec of TGS/TD05TsWire.mod run by
     TG_Main (handshake, file transfer, pass check, the TSPWeld2_full block with
     its searches pretended, the PWeld2 weld frame, the end request), served by
     the real AbbTgsHmi. Design: docs/abb_touch_sense_port_v1.md sections 3 and 6.
"""

import os
import re
import socket
import threading
import unittest

import abb_server
from abb_server import (
    ACK,
    CORRUPT_FRAME_PAYLOAD,
    TOUCH_REAL_DEMO,
    TOUCH_WIRE_DEMO,
    AbbTgsHmi,
    auto_touchup_offset,
    pose_literal_to_xyzwpr,
    rotate_by_wpr,
    touch_axis_index,
    translate_frame,
    xyzwpr_to_pose_literal,
)
from fake_rapid import FakeHandshake
from test_phase1 import _mat_from_wpr

IDENTITY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def point(nominal, axis):
    return {"nominal": nominal, "snapped_axis": axis}


class TestAutoTouchupOffset(unittest.TestCase):

    def assert_vec(self, got, want, places=9):
        self.assertEqual(len(got), len(want))
        for g, w in zip(got, want):
            self.assertAlmostEqual(g, w, places=places)

    def test_one_touch_is_a_one_axis_shift(self):
        # Search[+X] onto x = 100; measured 2.5 further, with junk in y and z.
        delta = auto_touchup_offset(IDENTITY, [point((100, 0, 20), (-1, 0, 0))], [(102.5, 7.0, -9.0)])
        self.assert_vec(delta, (2.5, 0.0, 0.0))

    def test_the_wire_demo_is_x_plus_3_z_minus_2(self):
        delta = auto_touchup_offset(IDENTITY, TOUCH_WIRE_DEMO["touch_points"], TOUCH_WIRE_DEMO["measured"])
        self.assert_vec(delta, TOUCH_WIRE_DEMO["delta_cad"])

    def test_three_touches_on_three_axes(self):
        pts = [point((10, 0, 0), (1, 0, 0)), point((0, 20, 0), (0, -1, 0)), point((0, 0, 30), (0, 0, 1))]
        delta = auto_touchup_offset(IDENTITY, pts, [(11, 5, 5), (6, 18.5, 6), (7, 7, 30.25)])
        self.assert_vec(delta, (1.0, -1.5, 0.25))

    def test_the_offset_is_rotated_into_the_robot_base(self):
        # A part frame turned 90 deg about Z: its X is the base's Y.
        frame = [900.0, 80.0, 350.0, 0.0, 0.0, 90.0]
        delta = auto_touchup_offset(frame, [point((100, 0, 0), (-1, 0, 0))], [(103, 0, 0)])
        self.assert_vec(delta, (0.0, 3.0, 0.0))

    def test_a_general_frame_matches_an_independent_rotation_matrix(self):
        frame = [900.0, 80.0, 350.0, -2.5, 3.5, 90.0]      # the prototype's default weld frame
        delta = auto_touchup_offset(frame, TOUCH_WIRE_DEMO["touch_points"], TOUCH_WIRE_DEMO["measured"])
        m = _mat_from_wpr(-2.5, 3.5, 90.0)
        d = TOUCH_WIRE_DEMO["delta_cad"]
        self.assert_vec(delta, [sum(m[i][k] * d[k] for k in range(3)) for i in range(3)])

    def test_a_repeated_axis_keeps_the_later_touch(self):
        # The HMI overwrites, it does not average (DoMultipleAutoTouchUpPointsSearchInTheSameAxis
        # exists but is never called).
        pts = [point((100, 0, 0), (-1, 0, 0)), point((100, 50, 0), (-1, 0, 0))]
        delta = auto_touchup_offset(IDENTITY, pts, [(101, 0, 0), (104, 50, 0)])
        self.assert_vec(delta, (4.0, 0.0, 0.0))

    def test_only_the_axis_of_a_snapped_axis_matters(self):
        self.assertEqual(touch_axis_index((-1, 0, 0)), touch_axis_index((1, 0, 0)))
        self.assertEqual(touch_axis_index((-0.9, 0.1, 0.2)), 0)   # dominant, as the .tgs loader snaps
        self.assertEqual(touch_axis_index((0, 0, -1)), 2)
        with self.assertRaises(ValueError):
            touch_axis_index((0, 0, 0))

    def test_a_point_count_other_than_the_welds_is_refused(self):
        with self.assertRaises(ValueError):
            auto_touchup_offset(IDENTITY, TOUCH_WIRE_DEMO["touch_points"], TOUCH_WIRE_DEMO["measured"][:1])

    def test_the_correction_translates_and_keeps_the_rotation(self):
        frame = [900.0, 80.0, 350.0, -2.5, 3.5, 90.0]
        moved = translate_frame(frame, (1.0, -2.0, 0.5))
        self.assert_vec(moved, [901.0, 78.0, 350.5, -2.5, 3.5, 90.0])

    def test_rotate_by_wpr_is_the_fanuc_convention(self):
        m = _mat_from_wpr(30.0, -20.0, 10.0)
        v = (1.0, 2.0, 3.0)
        self.assert_vec(rotate_by_wpr(30.0, -20.0, 10.0, v), [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)])


# ---------------------------------------------------------------------------
# Fake robot: executable spec of TG_Main + TGS/TD05TsWire.mod on the wire
# ---------------------------------------------------------------------------

ROBOT_POSE_XYZWPR = [1500.0, -200.0, 1400.0, 10.0, -20.0, 30.0]
#: The pretended contact points of TD05TsWire.mod (rtTsHit1 / rtTsHit2), orientation [0,0,1,0].
TD05TSWIRE_HITS = [(103.0, 0.7, 20.4), (59.6, 25.3, -2.0)]
HIT_ROT_WPR = abb_server.quat_to_euler_wpr(0.0, 0.0, 1.0, 0.0)


class FakeTouchWireRobot(threading.Thread):
    """The RAPID side of one TD05TsWire cycle, message by message.

    TG_Main.tgMainCycle: handshake, accept, program selection, file transfer, then
    TD05TsWire: pass check;
        TSPWeld2_full R_W_F (frame -> oframe);
        R_TS_D; IF flag = 1: two R_TS_P (pretended hits), R_TS_END (frame -> oframe);
        PWeld2 R_W_F (frame -> oframe); abort_end: R_E.
    A malformed id-19 frame -> tgTouchAbort: the connection drops, no R_E.
    """

    def __init__(self, hits=None, prog="TD05TsWire"):
        super().__init__(daemon=True)
        self.hits = list(hits if hits is not None else TD05TSWIRE_HITS)
        self.prog = prog
        self.errors = []
        self.acks = []
        self.sent_ids = []
        self.point_payloads = []
        self.frames = {}          # what each frame-serving request delivered, raw
        self.ts_flag = None
        self.oframe = None        # wobjTG_Weld.oframe, as parsed (xyzwpr)
        self.aborted = False
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]
        self.handshake = FakeHandshake()

    def _send_ack(self, conn, payload):                      # tgSendAck
        conn.sendall(payload.encode("utf-8"))
        self.acks.append(conn.recv(16))

    def _send_id(self, conn, req_id):
        self.sent_ids.append(req_id)
        self._send_ack(conn, req_id)

    def _prompt(self, conn, text):                           # tgPromptRecv
        conn.sendall(text.encode("utf-8"))
        return conn.recv(1024).decode("utf-8")

    def _send_pose(self, conn):                              # tgSendPose
        self._send_ack(conn, xyzwpr_to_pose_literal(ROBOT_POSE_XYZWPR))

    @staticmethod
    def _try_pose(literal):                                  # tgTryStrToPose
        try:
            return pose_literal_to_xyzwpr(literal)
        except (ValueError, IndexError):
            return None

    def run(self):
        try:
            if self.handshake.serve() != "1":
                return
            conn, _ = self.listener.accept()
            with conn:
                self._cycle(conn)
        except Exception as exc:
            self.errors.append(exc)
        finally:
            self.listener.close()
            self.handshake.close()

    def _cycle(self, conn):
        if self._prompt(conn, "Give me the program ID") != "1":
            return
        self._send_id(conn, "10")                            # TG_ReqFileTransfer
        self._send_ack(conn, "99999999")
        ftp = self._prompt(conn, "Give me FTP status")
        self._prompt(conn, "Give me prog name")
        if ftp == "1":
            self._program(conn)

    def _weld_frame(self, conn, sub):                        # TG_ReqWeldFrame
        self._send_id(conn, "4")
        self._send_pose(conn)
        self._send_ack(conn, sub)
        raw = self._prompt(conn, "Give me the frame")
        self.frames[sub] = raw
        frame = self._try_pose(raw)
        status = self._prompt(conn, "Give me weld status")
        for axis in "xyz":
            self._prompt(conn, "Give me touchup " + axis)
        if frame is None:
            return 2
        self.oframe = frame
        return int(status[:1] or "0")

    def _program(self, conn):                                # TD05TsWire
        self._send_id(conn, "5")                             # TG_ReqPassCheck
        self._send_pose(conn)
        self._send_ack(conn, "none")
        self._send_ack(conn, self.prog)
        if self._prompt(conn, "Give me the status")[:1] == "0":
            return
        # ---- TouchSense for Weld2
        self._weld_frame(conn, "TSPWeld2_full")
        self._send_id(conn, "16")                            # TG_ReqTouchSenseDo
        reply = self._prompt(conn, "Give me TS status")
        self.ts_flag = 1 if reply[:1] == "1" else 0
        if self.ts_flag == 1:
            for hit in self.hits:                            # TG_TouchSearch's hits
                self._send_id(conn, "18")                    # TG_ReqTouchSensePoint
                payload = xyzwpr_to_pose_literal(list(hit) + list(HIT_ROT_WPR))
                self.point_payloads.append(payload)
                self._send_ack(conn, payload)
            self._send_id(conn, "19")                        # TG_ReqTouchSenseEnd
            raw = self._prompt(conn, "Give me the frame")
            self.frames["touch_end"] = raw
            frame = self._try_pose(raw)
            if frame is None:
                self.aborted = True                          # tgTouchAbort: sockets closed, ExitCycle
                return
            self.oframe = frame
        # ---- Start of Weld2
        status = self._weld_frame(conn, "PWeld2")
        # (status 2 -> abort_end; either way the end request follows)
        self._send_id(conn, "100")                           # TG_ReqEnd
        self._send_pose(conn)
        self._send_ack(conn, "none")


def run_touch_cycle(hmi_setup=None):
    robot = FakeTouchWireRobot()
    robot.start()
    hmi = AbbTgsHmi(host="127.0.0.1", port=robot.port, handshake_port=robot.handshake.port, verbose=False)
    hmi.prog_name = "TD05TsWire"
    hmi.touch_points = [dict(p) for p in TOUCH_WIRE_DEMO["touch_points"]]
    hmi.auto_touchups = True
    if hmi_setup:
        hmi_setup(hmi)
    hmi.serve_cycle()
    robot.join(timeout=10)
    if robot.is_alive():
        raise AssertionError("fake robot did not finish")
    if robot.errors:
        raise robot.errors[0]
    return robot, hmi


class TestTouchWire(unittest.TestCase):

    def assert_frame(self, got, want, mm=0.006, deg=1e-4):
        for g, w in zip(got[:3], want[:3]):
            self.assertAlmostEqual(g, w, delta=mm)
        for g, w in zip(got[3:], want[3:]):
            self.assertAlmostEqual(g, w, delta=deg)

    def expected_corrected_frame(self, hmi):
        loc = hmi.weld_frame_xyzwpr
        return translate_frame(loc, rotate_by_wpr(loc[3], loc[4], loc[5], TOUCH_WIRE_DEMO["delta_cad"]))

    def test_request_order_is_the_tsp_block(self):
        robot, hmi = run_touch_cycle()
        self.assertEqual(hmi.request_log, ["10", "5", "4", "16", "18", "18", "19", "4", "100"])
        self.assertEqual(robot.sent_ids, hmi.request_log)

    def test_request_17_is_never_sent_and_not_served(self):
        robot, hmi = run_touch_cycle()
        self.assertNotIn("17", hmi.request_log)
        self.assertNotIn("17", hmi.handlers)         # plan D2: a robot sending 17 gets "no handler"

    def test_every_robot_message_is_acked(self):
        robot, _ = run_touch_cycle()
        self.assertTrue(robot.acks)
        self.assertEqual(set(robot.acks), {ACK})

    def test_the_hmi_says_run_the_block(self):
        robot, _ = run_touch_cycle()
        self.assertEqual(robot.ts_flag, 1)

    def test_the_tsp_frame_is_the_localization_alone(self):
        robot, hmi = run_touch_cycle(lambda h: setattr(h, "touch_offset_base", (50.0, 50.0, 50.0)))
        # A stored touch-up from an earlier run is cleared by the TSP R_W_F.
        self.assert_frame(pose_literal_to_xyzwpr(robot.frames["TSPWeld2_full"]), hmi.weld_frame_xyzwpr)

    def test_the_points_arrive_as_pose_literals_in_rapid_limits(self):
        robot, hmi = run_touch_cycle()
        self.assertEqual(len(robot.point_payloads), 2)
        for payload in robot.point_payloads:
            self.assertLessEqual(len(payload), 80)
        # the HMI stored x, y, z of each - and consumed them at id 19
        self.assertEqual(hmi.touch_measured_xyz, [])

    def test_the_touch_end_frame_is_the_localization_moved_by_r_delta(self):
        robot, hmi = run_touch_cycle()
        want = self.expected_corrected_frame(hmi)
        self.assert_frame(pose_literal_to_xyzwpr(robot.frames["touch_end"]), want)
        self.assert_frame(hmi.last_touch_frame_xyzwpr, want, mm=1e-9, deg=1e-9)

    def test_the_weld_frame_after_the_block_is_the_corrected_frame(self):
        robot, hmi = run_touch_cycle()
        self.assert_frame(pose_literal_to_xyzwpr(robot.frames["PWeld2"]),
                          pose_literal_to_xyzwpr(robot.frames["touch_end"]), mm=1e-9, deg=1e-9)
        self.assert_frame(robot.oframe, self.expected_corrected_frame(hmi))

    def test_touch_ups_off_skip_the_block(self):
        robot, hmi = run_touch_cycle(lambda h: setattr(h, "auto_touchups", False))
        self.assertEqual(robot.ts_flag, 0)
        self.assertEqual(hmi.request_log, ["10", "5", "4", "16", "4", "100"])
        self.assert_frame(robot.oframe, hmi.weld_frame_xyzwpr)

    def test_a_weld_without_touch_points_answers_zero_too(self):
        robot, hmi = run_touch_cycle(lambda h: setattr(h, "touch_points", []))
        self.assertEqual(robot.ts_flag, 0)
        self.assertNotIn("18", hmi.request_log)

    def test_a_corrupt_touch_end_frame_abandons_the_cycle(self):
        robot, hmi = run_touch_cycle(lambda h: setattr(h, "corrupt_touch_frame", True))
        self.assertTrue(robot.aborted)
        self.assertEqual(robot.frames["touch_end"], CORRUPT_FRAME_PAYLOAD)
        self.assertEqual(hmi.request_log[-1], "19")          # no weld frame, no end request
        # The work object still holds the TSP frame: nothing the HMI did not mean was applied.
        self.assert_frame(robot.oframe, hmi.weld_frame_xyzwpr)

    def test_more_reports_than_touch_points_are_refused(self):
        with self.assertRaises(RuntimeError):
            run_touch_cycle(lambda h: setattr(h, "touch_points", h.touch_points[:1]))


# ---------------------------------------------------------------------------
# P3: TD05Touch.mod, TG_TrRig.mod and TOUCH_REAL_DEMO must describe ONE part
# ---------------------------------------------------------------------------

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
TD05TOUCH = os.path.join(REPO, "abb", "rapid", "TGS", "TD05Touch.mod")
TRRIG = os.path.join(REPO, "hmi_prototype", "vc_probes", "TG_TrRig.mod")
_RT = re.compile(r"LOCAL CONST robtarget (\w+):=\[\[([^\]]*)\],\[([^\]]*)\]")
_POS = re.compile(r"LOCAL CONST pos (\w+):=\[([^\]]*)\]")


def _floats(text):
    return tuple(float(v) for v in text.split(","))


def rapid_robtargets(path):
    with open(path, "rb") as fh:
        text = fh.read().decode("ascii")
    return {m.group(1): (_floats(m.group(2)), _floats(m.group(3))) for m in _RT.finditer(text)}


def rapid_positions(path):
    with open(path, "rb") as fh:
        text = fh.read().decode("ascii")
    return {m.group(1): _floats(m.group(2)) for m in _POS.finditer(text)}


def _qmul(a, b):
    a1, b1, c1, d1 = a
    a2, b2, c2, d2 = b
    return (a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2, a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2,
            a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2, a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2)


class TestTouchRealContract(unittest.TestCase):

    def setUp(self):
        self.rts = rapid_robtargets(TD05TOUCH)
        self.lo, self.hi = TOUCH_REAL_DEMO["block"]

    def test_the_module_searches_from_the_dicts_approaches_to_its_nominals(self):
        d = TOUCH_REAL_DEMO["standoff"]
        for i, point in enumerate(TOUCH_REAL_DEMO["touch_points"], start=1):
            contact = self.rts["rtTs%dContact" % i][0]
            approach = self.rts["rtTs%dApproach" % i][0]
            self.assertEqual(contact, tuple(point["nominal"]))
            want = tuple(c + d * a for c, a in zip(point["nominal"], point["snapped_axis"]))
            self.assertEqual(approach, want)

    def test_each_nominal_is_on_the_face_its_axis_points_out_of(self):
        for point in TOUCH_REAL_DEMO["touch_points"]:
            axis = touch_axis_index(point["snapped_axis"])
            face = self.lo[axis] if point["snapped_axis"][axis] < 0 else self.hi[axis]
            self.assertEqual(point["nominal"][axis], face)
            for other in {0, 1, 2} - {axis}:
                self.assertTrue(self.lo[other] < point["nominal"][other] < self.hi[other])

    def test_every_approach_and_air_move_is_outside_the_block(self):
        for name in ("rtTsVia", "rtTs1Approach", "rtTs2Approach", "rtW2Approach", "rtW2Start",
                     "rtW2End", "rtW2Depart"):
            p = self.rts[name][0]
            inside = all(self.lo[k] <= p[k] <= self.hi[k] for k in range(3))
            self.assertFalse(inside, name)

    def test_the_orientation_is_the_home_tool_orientation_in_world(self):
        q_loc = abb_server.euler_wpr_to_quat(*TOUCH_REAL_DEMO["weld_frame_xyzwpr"][3:6])
        for name, (_, q_part) in self.rts.items():
            q_world = _qmul(q_loc, q_part)
            for got, want in zip(q_world, (0.5, 0.0, 0.866025, 0.0)):
                self.assertAlmostEqual(got, want, places=5, msg=name)

    def test_the_rig_box_is_the_block_mapped_to_world(self):
        frame = TOUCH_REAL_DEMO["weld_frame_xyzwpr"]
        corners = [(x, y, z) for x in (self.lo[0], self.hi[0]) for y in (self.lo[1], self.hi[1])
                   for z in (self.lo[2], self.hi[2])]
        world = [tuple(a + b for a, b in zip(rotate_by_wpr(frame[3], frame[4], frame[5], c), frame[:3]))
                 for c in corners]
        rig = rapid_positions(TRRIG)
        for k in range(3):
            self.assertAlmostEqual(min(w[k] for w in world), rig["TR_LO"][k], places=6)
            self.assertAlmostEqual(max(w[k] for w in world), rig["TR_HI"][k], places=6)

    def test_a_block_moved_3_along_x_and_minus_2_along_z_moves_the_weld_frame_by_r_delta(self):
        shift = (3.0, 0.0, -2.0)
        hits = []
        for point in TOUCH_REAL_DEMO["touch_points"]:
            axis = touch_axis_index(point["snapped_axis"])
            hit = list(point["nominal"])
            hit[axis] += shift[axis]
            hits.append(tuple(hit))

        def setup(h):
            h.prog_name = TOUCH_REAL_DEMO["prog_name"]
            h.weld_frame_xyzwpr = list(TOUCH_REAL_DEMO["weld_frame_xyzwpr"])
            h.touch_points = [dict(p) for p in TOUCH_REAL_DEMO["touch_points"]]

        robot = FakeTouchWireRobot(hits=hits, prog="TD05Touch")
        robot.start()
        hmi = AbbTgsHmi(host="127.0.0.1", port=robot.port, handshake_port=robot.handshake.port, verbose=False)
        hmi.auto_touchups = True
        setup(hmi)
        hmi.serve_cycle()
        robot.join(timeout=10)
        self.assertEqual(robot.errors, [])
        frame = TOUCH_REAL_DEMO["weld_frame_xyzwpr"]
        want = translate_frame(frame, rotate_by_wpr(frame[3], frame[4], frame[5], shift))
        # part X is world +Y here: the shift lands as (0, +3, -2) in the base
        self.assertAlmostEqual(want[1] - frame[1], 3.0, places=9)
        got = pose_literal_to_xyzwpr(robot.frames["PWeld2"])
        for g, w in zip(got[:3], want[:3]):
            self.assertAlmostEqual(g, w, delta=0.006)


if __name__ == "__main__":
    unittest.main(verbosity=2)

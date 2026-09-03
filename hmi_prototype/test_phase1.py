"""Phase 1 tests for the ABB HMI prototype.

Run:  python -m unittest discover -s hmi_prototype -v
 or:  python hmi_prototype/test_phase1.py

Two layers:
  1. Pose codec unit tests - Euler(FANUC W,P,R) <-> quaternion, cross-checked
     against rotation matrices built independently.
  2. Choreography test - a fake robot thread that emulates, message by
     message, exactly what abb/rapid/TG_Comms.sys + TG_Main.mod do in
     Phase 1 (accept -> prog-sel prompt -> request 100 -> disconnect),
     with the real AbbTgsHmi client served against it, twice (the Phase 1
     exit criterion is two consecutive cycles).
"""

import math
import socket
import threading
import unittest

from abb_server import (
    ACK,
    AbbTgsHmi,
    euler_wpr_to_quat,
    fmt_real,
    pose_literal_to_xyzwpr,
    quat_to_euler_wpr,
    xyzwpr_to_pose_literal,
)


# ---------------------------------------------------------------------------
# Independent rotation-matrix helpers (pure python, no numpy)
# ---------------------------------------------------------------------------

def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _mat_from_wpr(w_deg, p_deg, r_deg):
    """FANUC convention: R = Rz(r) * Ry(p) * Rx(w), fixed axes."""
    return _mat_mul(_rot_z(math.radians(r_deg)),
                    _mat_mul(_rot_y(math.radians(p_deg)),
                             _rot_x(math.radians(w_deg))))


def _mat_from_quat(q1, q2, q3, q4):
    w, x, y, z = q1, q2, q3, q4
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


class TestPoseCodec(unittest.TestCase):

    def assert_mats_close(self, a, b, tol=1e-9):
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(a[i][j], b[i][j], delta=tol)

    def test_identity(self):
        self.assertEqual(euler_wpr_to_quat(0, 0, 0), (1.0, 0.0, 0.0, 0.0))

    def test_known_single_axis_rotations(self):
        h = math.sqrt(0.5)
        for angles, expected in [
            ((90, 0, 0), (h, h, 0.0, 0.0)),   # +90 about X -> q2
            ((0, 90, 0), (h, 0.0, h, 0.0)),   # +90 about Y -> q3
            ((0, 0, 90), (h, 0.0, 0.0, h)),   # +90 about Z -> q4
        ]:
            q = euler_wpr_to_quat(*angles)
            for got, want in zip(q, expected):
                self.assertAlmostEqual(got, want, places=9, msg=str(angles))

    def test_quat_is_normalized(self):
        q = euler_wpr_to_quat(159.464, -40.967, 29.743)  # from TD05tRJYQd P[52]
        self.assertAlmostEqual(sum(c * c for c in q), 1.0, places=12)

    def test_matrix_cross_check(self):
        """euler->quat is right iff both produce the same rotation matrix."""
        cases = [(0, 0, 0), (30, 0, 0), (0, 45, 0), (0, 0, 60),
                 (159.464, -40.967, 29.743), (-120, 35, -170), (10, -85, 100)]
        for w, p, r in cases:
            m_euler = _mat_from_wpr(w, p, r)
            m_quat = _mat_from_quat(*euler_wpr_to_quat(w, p, r))
            self.assert_mats_close(m_euler, m_quat)

    def test_round_trip(self):
        """quat->euler inverts euler->quat away from the p = +-90 poles."""
        cases = [(0, 0, 0), (30, 40, 50), (159.464, -40.967, 29.743),
                 (-90, 10, 170), (5, -80, -5), (179, 0, -179)]
        for angles in cases:
            back = quat_to_euler_wpr(*euler_wpr_to_quat(*angles))
            m1 = _mat_from_wpr(*angles)
            m2 = _mat_from_wpr(*back)
            # compare via matrices: euler triples are only unique mod the
            # representation, the rotation itself must match exactly
            self.assert_mats_close(m1, m2, tol=1e-9)

    def test_pose_literal_round_trip(self):
        frame = [81.125, -129.068, 28.281, 159.464, -40.967, 29.743]  # TD05 P[52]
        literal = xyzwpr_to_pose_literal(frame)
        self.assertLessEqual(len(literal), 80)
        back = pose_literal_to_xyzwpr(literal)
        for got, want in zip(back[:3], frame[:3]):
            self.assertAlmostEqual(got, want, places=2)
        m1 = _mat_from_wpr(*frame[3:])
        m2 = _mat_from_wpr(*back[3:])
        self.assert_mats_close(m1, m2, tol=1e-4)  # quat limited to 6 decimals

    def test_pose_literal_worst_case_length(self):
        literal = xyzwpr_to_pose_literal([-9999.99, -9999.99, -9999.99, 45, 45, 45])
        self.assertLessEqual(len(literal), 80)

    def test_pose_literal_rejects_out_of_range(self):
        with self.assertRaises(ValueError):
            xyzwpr_to_pose_literal([10000.0, 0, 0, 0, 0, 0])

    def test_parses_rapid_style_literal(self):
        """Accept RAPID NumToStr output, which may drop trailing zeros."""
        frame = pose_literal_to_xyzwpr("[[600,500,225.3],[1,0,0,0]]")
        self.assertEqual(frame, [600.0, 500.0, 225.3, 0.0, 0.0, 0.0])

    def test_fmt_real_matches_fanuc_examples(self):
        self.assertEqual(fmt_real(905.216), "+0905.216")   # FANUCRobot.cpp comment
        self.assertEqual(fmt_real(-905.216), "-0905.216")
        self.assertEqual(fmt_real(-645.0), "-0645.000")
        self.assertEqual(fmt_real(0), "+0000.000")
        self.assertEqual(len(fmt_real(-1234.5)), 9)


# ---------------------------------------------------------------------------
# Fake robot: executable spec of the RAPID Phase 1 behavior
# ---------------------------------------------------------------------------

# The pose the fake robot reports (mm / FANUC W,P,R deg), sent as the same
# literal TG_Comms.tgPoseToStr would produce.
ROBOT_POSE_XYZWPR = [600.0, -150.5, 1225.3, 159.464, -40.967, 29.743]


class FakeRapidRobot(threading.Thread):
    """Emulates TG_Main + TG_Comms Phase 1, message by message:

    per cycle:  accept                          (TG_SocketCom)
                send "Give me the program ID",
                recv program id                 (TG_ReqProgSel / tgPromptRecv)
                send "100", recv ack            (TG_ReqEnd / tgSendAck)
                send pose literal, recv ack     (tgSendPose)
                send "none", recv ack           (tgSendAck stTG_SubName)
                close connection                (TG_SocketDisc)
    """

    def __init__(self, cycles):
        super().__init__(daemon=True)
        self.cycles = cycles
        self.received_prog_ids = []
        self.received_acks = []
        self.errors = []
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0))  # ephemeral port
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]

    def _send_ack_msg(self, conn, payload):  # tgSendAck
        conn.sendall(payload.encode("utf-8"))
        self.received_acks.append(conn.recv(16))

    def run(self):
        try:
            for _ in range(self.cycles):
                conn, _addr = self.listener.accept()
                with conn:
                    # TG_ReqProgSel
                    conn.sendall(b"Give me the program ID")
                    self.received_prog_ids.append(conn.recv(16).decode("utf-8"))
                    # TG_ReqEnd
                    self._send_ack_msg(conn, "100")
                    self._send_ack_msg(conn, xyzwpr_to_pose_literal(ROBOT_POSE_XYZWPR))
                    self._send_ack_msg(conn, "none")
                # connection closed = TG_SocketDisc
        except Exception as exc:  # surfaced by the test
            self.errors.append(exc)
        finally:
            self.listener.close()


class TestPhase1Choreography(unittest.TestCase):

    def test_two_full_cycles(self):
        """Phase 1 exit criterion: connect/prog-sel/end/disconnect twice."""
        robot = FakeRapidRobot(cycles=2)
        robot.start()
        hmi = AbbTgsHmi(host="127.0.0.1", port=robot.port, verbose=False)
        for _ in range(2):
            hmi.serve_cycle()
        robot.join(timeout=10)
        self.assertFalse(robot.is_alive(), "fake robot did not finish")
        self.assertEqual(robot.errors, [])

        # robot got the program selection both cycles
        self.assertEqual(robot.received_prog_ids, ["1", "1"])
        # every robot->HMI message was acked with the FANUC-compatible b"0"
        self.assertEqual(robot.received_acks, [ACK] * 6)
        # the HMI decoded the end request
        self.assertEqual(hmi.last_sub_name, "none")
        for got, want in zip(hmi.last_pose_xyzwpr[:3], ROBOT_POSE_XYZWPR[:3]):
            self.assertAlmostEqual(got, want, places=2)
        for got, want in zip(hmi.last_pose_xyzwpr[3:], ROBOT_POSE_XYZWPR[3:]):
            self.assertAlmostEqual(got, want, places=3)


if __name__ == "__main__":
    unittest.main(verbosity=2)

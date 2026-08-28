"""Phase 2 tests: all priority requests served end-to-end.

Run:  python -m unittest discover -s hmi_prototype -v
 or:  python hmi_prototype/test_phase2.py

`FakeTgsRobot` emulates, message by message, what the RAPID side does in a
full cycle - TG_Main.tgMainCycle plus the TD05Test .tgs program - including
every branch (ftp failed, wrong password, capture skipped/failed, weld
skipped/aborted, predefined vs user-defined weld parameters). It is the
executable spec of abb/rapid/TG_Comms.sys + TG_Main.mod + TGS/TD05Test.mod.
"""

import socket
import threading
import unittest

from abb_server import (
    ACK,
    AbbTgsHmi,
    pose_literal_to_xyzwpr,
    xyzwpr_to_pose_literal,
)

ROBOT_POSE_XYZWPR = [1500.0, -200.0, 1400.0, 10.0, -20.0, 30.0]


class FakeTgsRobot(threading.Thread):
    """Executable spec of the RAPID Phase 2 cycle (TG_Main + TD05Test)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.received = {}     # everything the robot side received, by key
        self.acks = []
        self.errors = []
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]

    # -- primitives, mirroring TG_Comms helpers ------------------------------

    def _send_ack(self, conn, payload):                      # tgSendAck
        conn.sendall(payload.encode("utf-8"))
        self.acks.append(conn.recv(16))

    def _prompt(self, conn, text):                           # tgPromptRecv
        conn.sendall(text.encode("utf-8"))
        return conn.recv(1024).decode("utf-8")

    def _send_pose(self, conn):                              # tgSendPose
        self._send_ack(conn, xyzwpr_to_pose_literal(ROBOT_POSE_XYZWPR))

    # -- the cycle, mirroring the RAPID control flow --------------------------

    def run(self):
        try:
            conn, _ = self.listener.accept()
            with conn:
                self._run_cycle(conn)
        except Exception as exc:
            self.errors.append(exc)
        finally:
            self.listener.close()

    def _run_cycle(self, conn):                              # tgMainCycle
        self.received["prog_sel"] = self._prompt(conn, "Give me the program ID")
        if self.received["prog_sel"] != "1":
            return
        # TG_ReqFileTransfer
        self._send_ack(conn, "10")
        self._send_ack(conn, "99999999")
        self.received["ftp_status"] = self._prompt(conn, "Give me FTP status")
        self.received["prog_name"] = self._prompt(conn, "Give me prog name")
        if self.received["ftp_status"] == "1":
            self._run_tgs_program(conn)
        # connection closed on return = TG_SocketDisc

    def _run_tgs_program(self, conn):                        # TD05Test PROC
        # TG_ReqPassCheck (password = program name, set by the .tgs program)
        self._send_ack(conn, "5")
        self._send_pose(conn)
        self._send_ack(conn, "none")
        self._send_ack(conn, "TD05Test")
        status = self._prompt(conn, "Give me the status")
        self.received["pass_status"] = status
        if status[0] == "0":
            return  # FANUC 'END': terminate WITHOUT the end request
        # capture set: two captures
        for i, sub in enumerate(("C1PGlobal_m45_3", "C2PGlobal_m45_3")):
            # TG_ReqCamFrame
            self._send_ack(conn, "1")
            self._send_pose(conn)
            self._send_ack(conn, sub)
            self.received[f"cam_frame_{i}"] = self._prompt(conn, "Give me the frame")
            do_cap = self._prompt(conn, "Give me capture status")
            self.received[f"do_capture_{i}"] = do_cap
            if do_cap == "1":
                # TG_ReqCapture
                self._send_ack(conn, "2")
                self._send_pose(conn)
                self._send_ack(conn, sub)
                cap_ok = self._prompt(conn, "Give me capture status")
                self.received[f"capture_ok_{i}"] = cap_ok
                if cap_ok == "0":
                    self._end_req(conn)  # GOTO abort_end
                    return
        # TG_ReqGlobalCapDone
        self._send_ack(conn, "11")
        self.received["global_ok"] = self._prompt(conn, "Give me global loc status")
        # TG_ReqWeldFrame
        self._send_ack(conn, "4")
        self._send_pose(conn)
        self._send_ack(conn, "PWeld2")
        self.received["weld_frame"] = self._prompt(conn, "Give me the frame")
        weld_status = self._prompt(conn, "Give me weld status")
        self.received["weld_status"] = weld_status
        if weld_status == "2":
            self._end_req(conn)  # GOTO abort_end
            return
        if weld_status == "1":
            # TG_ReqWeldParams
            self._send_ack(conn, "14")
            self.received["udwp"] = self._prompt(conn, "Give me UDWP flag")
            self.received["travel_raw"] = self._prompt(conn, "Give me travel speed")
            if self.received["udwp"] == "1":
                self.received["welder_type_raw"] = self._prompt(conn, "Give me welder type")
                self.received["proc_raw"] = self._prompt(conn, "Give me proc")
                self.received["wirefeed_raw"] = self._prompt(conn, "Give me wire feed speed")
                self.received["arclen_raw"] = self._prompt(conn, "Give me arc length")
                self.received["arcctl_raw"] = self._prompt(conn, "Give me arc control")
        self._end_req(conn)

    def _end_req(self, conn):                                # TG_ReqEnd
        self._send_ack(conn, "100")
        self._send_pose(conn)
        self._send_ack(conn, "none")


def run_one_cycle(hmi_config=None):
    """Run the fake robot against a fresh AbbTgsHmi; return (robot, hmi)."""
    robot = FakeTgsRobot()
    robot.start()
    hmi = AbbTgsHmi(host="127.0.0.1", port=robot.port, verbose=False)
    for key, value in (hmi_config or {}).items():
        setattr(hmi, key, value)
    hmi.serve_cycle()
    robot.join(timeout=10)
    if robot.is_alive():
        raise AssertionError("fake robot did not finish")
    if robot.errors:
        raise robot.errors[0]
    return robot, hmi


class TestFullCycle(unittest.TestCase):
    """Happy path with default config: every priority request is exercised."""

    @classmethod
    def setUpClass(cls):
        cls.robot, cls.hmi = run_one_cycle()

    def test_request_order_matches_tgs_program(self):
        self.assertEqual(
            self.hmi.request_log,
            ["10", "5", "1", "2", "1", "2", "11", "4", "14", "100"])

    def test_all_acks_fanuc_compatible(self):
        self.assertEqual(self.robot.acks, [ACK] * len(self.robot.acks))
        self.assertGreater(len(self.robot.acks), 0)

    def test_file_transfer_exchange(self):
        self.assertEqual(self.hmi.last_free_bytes, 99999999)
        self.assertEqual(self.robot.received["ftp_status"], "1")
        self.assertEqual(self.robot.received["prog_name"], "TD05Test")
        self.assertLessEqual(len(self.robot.received["prog_name"]), 10)

    def test_pass_check_exchange(self):
        self.assertEqual(self.hmi.last_password, "TD05Test")
        self.assertEqual(self.robot.received["pass_status"], "10")  # ok, no dry-run
        self.assertEqual(len(self.robot.received["pass_status"]), 2)

    def test_frames_round_trip(self):
        for key, expected in (("cam_frame_0", self.hmi.cam_frame_xyzwpr),
                              ("cam_frame_1", self.hmi.cam_frame_xyzwpr),
                              ("weld_frame", self.hmi.weld_frame_xyzwpr)):
            literal = self.robot.received[key]
            self.assertLessEqual(len(literal), 80)
            got = pose_literal_to_xyzwpr(literal)
            for g, w in zip(got[:3], expected[:3]):
                self.assertAlmostEqual(g, w, places=2, msg=key)
            for g, w in zip(got[3:], expected[3:]):
                self.assertAlmostEqual(g, w, places=3, msg=key)

    def test_robot_pose_decoded_by_hmi(self):
        for g, w in zip(self.hmi.last_pose_xyzwpr, ROBOT_POSE_XYZWPR):
            self.assertAlmostEqual(g, w, places=2)

    def test_weld_params_formats(self):
        r = self.robot.received
        self.assertEqual(r["udwp"], "1")
        self.assertEqual(r["travel_raw"], "+0017.500")
        self.assertEqual(r["welder_type_raw"], "01")
        self.assertEqual(r["proc_raw"], "05")
        self.assertEqual(r["wirefeed_raw"], "+0250.000")
        self.assertEqual(r["arclen_raw"], "+0002.500")
        self.assertEqual(r["arcctl_raw"], "+0000.000")
        for key in ("travel_raw", "wirefeed_raw", "arclen_raw", "arcctl_raw"):
            self.assertEqual(len(r[key]), 9, key)


class TestBranchScenarios(unittest.TestCase):
    """The robot-side branches react to the served values exactly like RAPID."""

    def test_ftp_failure_skips_program(self):
        robot, hmi = run_one_cycle({"ftp_status": 0})
        self.assertEqual(hmi.request_log, ["10"])

    def test_wrong_password_terminates_without_end_request(self):
        robot, hmi = run_one_cycle({"pass_ok": 0})
        self.assertEqual(hmi.request_log, ["10", "5"])  # FANUC 'END' semantics
        self.assertEqual(robot.received["pass_status"], "00")

    def test_capture_failure_aborts_to_end(self):
        robot, hmi = run_one_cycle({"capture_ok": 0})
        self.assertEqual(hmi.request_log, ["10", "5", "1", "2", "100"])

    def test_captures_skipped_when_flag_zero(self):
        robot, hmi = run_one_cycle({"do_capture": 0})
        self.assertEqual(hmi.request_log,
                         ["10", "5", "1", "1", "11", "4", "14", "100"])

    def test_weld_abort_skips_params(self):
        robot, hmi = run_one_cycle({"weld_status": 2})
        self.assertEqual(hmi.request_log,
                         ["10", "5", "1", "2", "1", "2", "11", "4", "100"])

    def test_predefined_schedule_sends_only_flag_and_speed(self):
        robot, hmi = run_one_cycle({"udwp_flag": 0})
        self.assertEqual(hmi.request_log,
                         ["10", "5", "1", "2", "1", "2", "11", "4", "14", "100"])
        self.assertEqual(robot.received["udwp"], "0")
        self.assertEqual(robot.received["travel_raw"], "+0017.500")
        self.assertNotIn("proc_raw", robot.received)


if __name__ == "__main__":
    unittest.main(verbosity=2)

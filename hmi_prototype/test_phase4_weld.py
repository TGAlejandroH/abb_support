"""Phase 4 tests: the two-weld arc program (TD05Weld) and weld-data mapping.

Run:  python -m unittest discover -s hmi_prototype -v
 or:  python hmi_prototype/test_phase4_weld.py

`FakeWeldRobot` is the executable spec of abb/rapid/TGS/TD05Weld.mod: two
welds, each doing R_W_F -> (status branch) -> R_W_P -> apply -> ArcLStart/
ArcLEnd, then R_E. It models the WIRE only, so it stays valid whatever the
RAPID side does with the values afterwards.

The conversion expectations mirror what TG_Weld.sys TG_ApplyWeldParams must
produce, and are cross-checked against the VC measurement that established
welddata.weld_speed is mm/s (300.0 mm at weld_speed 8.89 took 33.722 s ->
8.896 mm/s, a 0.07 % match; docs/abb_weld_motion_and_data_design_v1.md 2.5).
"""

import socket
import threading
import unittest

import abb_server
from fake_rapid import FakeHandshake
from abb_server import (
    ACK,
    IPM_TO_MM_S,
    WELD_DEMO_EXPECTED_MM_S,
    WELD_DEMO_SEQUENCE,
    AbbTgsHmi,
    fmt_real,
    xyzwpr_to_pose_literal,
)

ROBOT_POSE_XYZWPR = [1500.0, -200.0, 1400.0, 10.0, -20.0, 30.0]

# The dummy (dist_mm, arc_on_s) TD05Weld.mod reports per weld via R_W_S.
# Both seams are 200 mm; the times are that length divided by the weld speed
# each weld was served (21 IPM = 8.89 mm/s, then 30 IPM = 12.7 mm/s), so the
# two payloads differ and a repeated payload cannot pass as two servings.
TD05WELD_WELD_STATS = ((200.0, 22.5), (200.0, 15.75))


class FakeWeldRobot(threading.Thread):
    """Executable spec of TD05Weld.mod (two welds) on the wire."""

    def __init__(self):
        super().__init__(daemon=True)
        self.weld_params = []      # one dict per R_W_P round, as received
        self.weld_stats_sent = []  # one payload per R_W_S round, as sent
        self.touchups = []         # one (x, y, z) inches per R_W_F round
        self.sub_names = []
        self.pass_status = ""
        self.errors = []
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]
        # The start handshake every cycle opens with (fake_rapid.py), on its own
        # ephemeral port - never the real 2001.
        self.handshake = FakeHandshake()

    # -- primitives, mirroring the TG_Comms helpers -------------------------

    def _send_ack(self, conn, payload):                      # tgSendAck
        conn.sendall(payload.encode("utf-8"))
        conn.recv(16)

    def _prompt(self, conn, text):                           # tgPromptRecv
        conn.sendall(text.encode("utf-8"))
        return conn.recv(1024).decode("utf-8")

    def _send_pose(self, conn):                              # tgSendPose
        self._send_ack(conn, xyzwpr_to_pose_literal(ROBOT_POSE_XYZWPR))

    # -- the requests TD05Weld issues --------------------------------------

    def _req_weld_frame(self, conn, sub_name):               # TG_ReqWeldFrame
        self._send_ack(conn, "4")
        self._send_pose(conn)
        self._send_ack(conn, sub_name)
        self.sub_names.append(sub_name)
        self._prompt(conn, "Give me the frame")
        status = int(self._prompt(conn, "Give me weld status"))
        # Phase 7: touch-up push - served on EVERY reply mode, received
        # before branching (RAPID stores it in posTG_Touchup, nothing more).
        self.touchups.append(tuple(
            float(self._prompt(conn, f"Give me touchup {axis}"))
            for axis in "xyz"))
        return status

    def _req_weld_params(self, conn):                        # TG_ReqWeldParams
        # WS3/WS4: UNCONDITIONAL. The `if udwp == 1` that used to wrap
        # everything below the travel speed is gone -- ABB has no controller
        # schedule to fall back on, so a preset-bound weld that is not served
        # its values has none. `udwp` is provenance now, not a branch.
        self._send_ack(conn, "14")
        got = {
            "udwp": int(self._prompt(conn, "Give me UDWP flag")),
            "travel_speed": float(self._prompt(conn, "Give me travel speed")),
            "welder_type": int(self._prompt(conn, "Give me welder type")),
            "proc": int(self._prompt(conn, "Give me proc")),
            "wire_feed": float(self._prompt(conn, "Give me wire feed speed")),
            "arc_length": float(self._prompt(conn, "Give me arc length")),
            "arc_control": float(self._prompt(conn, "Give me arc control")),
            "weld_sched": int(self._prompt(conn, "Give me weld schedule")),
        }
        # The nine seam phases arrive as one bracketed message.
        raw = self._prompt(conn, "Give me seam phases")
        got["seam_phases_raw"] = raw
        got["seam_phases"] = [float(v) for v in raw.strip("[]").split(",")]
        self.weld_params.append(got)
        return got

    def _req_weld_stats(self, conn, dist_mm, arc_on_sec):    # TG_ReqWeldStats
        # R_W_S: id, then ONE csv message. succ_ae mirrors the module's
        # nTG_SuccArcEnd := 1 - nTG_DryRun (dry run -> no arc -> the HMI
        # must not record the weld).
        succ_ae = 1 - int(self.pass_status[1])
        payload = ",".join(fmt_real(v) for v in (dist_mm, arc_on_sec, succ_ae))
        self._send_ack(conn, "13")
        self._send_ack(conn, payload)
        self.weld_stats_sent.append(payload)

    def run(self):
        try:
            if self.handshake.serve() != "1":
                return                      # refused: the part ends, no run
            conn, _ = self.listener.accept()
            with conn:
                # program selection + file transfer
                self._prompt(conn, "Give me the program ID")
                self._send_ack(conn, "10")
                self._send_ack(conn, "99999999")
                self._prompt(conn, "Give me FTP status")
                self._prompt(conn, "Give me prog name")

                # pass check
                self._send_ack(conn, "5")
                self._send_pose(conn)
                self._send_ack(conn, "none")
                self._send_ack(conn, "TD05Weld")
                self.pass_status = self._prompt(conn, "Give me the status")

                # global captures done (TD05Weld skips the capture set)
                self._send_ack(conn, "11")
                self._prompt(conn, "Give me global loc status")

                # ---- the two welds ----
                for i, sub in enumerate(("PWeld2", "PWeld3")):
                    status = self._req_weld_frame(conn, sub)
                    if status == 2:
                        break                       # abort to LBL[101]
                    if status == 1:
                        self._req_weld_params(conn)  # then apply + ArcL*
                        self._req_weld_stats(conn, *TD05WELD_WELD_STATS[i])

                # end request
                self._send_ack(conn, "100")
                self._send_pose(conn)
                self._send_ack(conn, "none")
        except Exception as exc:                     # surfaced by the test
            self.errors.append(exc)
        finally:
            self.listener.close()
            self.handshake.close()


def run_cycle(robot, hmi_setup=None):
    robot.start()
    hmi = AbbTgsHmi(host="127.0.0.1", port=robot.port,
                    handshake_port=robot.handshake.port, verbose=False)
    hmi.prog_name = "TD05Weld"
    hmi.weld_param_sequence = WELD_DEMO_SEQUENCE
    if hmi_setup:
        hmi_setup(hmi)
    hmi.serve_cycle()
    robot.join(timeout=5)
    if robot.errors:
        raise robot.errors[0]
    return hmi


class TestTwoWeldCycle(unittest.TestCase):
    """The full TD05Weld choreography: two welds, two R_W_P rounds."""

    def test_two_welds_served_in_one_cycle(self):
        robot = FakeWeldRobot()
        run_cycle(robot)
        self.assertEqual(robot.sub_names, ["PWeld2", "PWeld3"])
        self.assertEqual(len(robot.weld_params), 2,
                         "each weld must get its own R_W_P round")

    def test_first_weld_operator_overrides_second_preset_bound(self):
        """One run covers both provenances -- but the flag no longer gates
        the wire, so both must arrive COMPLETE."""
        robot = FakeWeldRobot()
        run_cycle(robot)
        first, second = robot.weld_params
        self.assertEqual(first["udwp"], 1)
        self.assertEqual(second["udwp"], 0)

    def test_a_preset_bound_weld_is_served_every_value(self):
        """The regression this whole change exists for.

        Under the pre-WS3 wire, UDWP=0 sent travel speed and nothing else, so
        TG_ApplyWeldParams fell back to the placeholder wdTG_Lib entry and the
        weld ran at weld_speed 0 -- the abb_coordinated_v6.tgs Weld5 defect.
        A preset-bound weld must now receive the same complete set as an
        operator-overridden one.
        """
        robot = FakeWeldRobot()
        run_cycle(robot)
        second = robot.weld_params[1]
        self.assertEqual(second["udwp"], 0, "this is the preset-bound weld")
        for key in ("welder_type", "proc", "wire_feed", "arc_length",
                    "arc_control", "weld_sched", "seam_phases"):
            self.assertIn(key, second, f"a preset-bound weld must be served {key}")
        # ...and with REAL values, not zeros: a wire that sends the fields but
        # leaves them empty would pass the check above and still weld at zero.
        self.assertAlmostEqual(second["travel_speed"], 30.0, places=3)
        self.assertAlmostEqual(second["wire_feed"], 400.0, places=3)
        self.assertGreater(second["weld_sched"], 0)

    def test_user_defined_fields_match_the_hmi_defaults(self):
        robot = FakeWeldRobot()
        run_cycle(robot)
        first = robot.weld_params[0]
        self.assertEqual(first["welder_type"], 2)      # FRONIUSTPSi
        self.assertEqual(first["proc"], 1)
        self.assertAlmostEqual(first["travel_speed"], 21.0, places=3)
        self.assertAlmostEqual(first["wire_feed"], 520.0, places=3)
        self.assertAlmostEqual(first["arc_length"], 49.0, places=3)
        self.assertAlmostEqual(first["arc_control"], 0.0, places=3)
        self.assertEqual(first["weld_sched"], 4)

    def test_seam_phases_arrive_as_one_message_in_the_declared_order(self):
        """Nine values, one message, and the order the planner/RAPID/server
        all three have to agree on -- a shift here lands a wire feed in a
        purge time with nothing failing."""
        robot = FakeWeldRobot()
        run_cycle(robot)
        first = robot.weld_params[0]
        self.assertEqual(len(first["seam_phases"]), len(abb_server.SEAM_PHASE_ORDER))
        self.assertTrue(first["seam_phases_raw"].startswith("["))
        self.assertTrue(first["seam_phases_raw"].endswith("]"))
        by_name = dict(zip(abb_server.SEAM_PHASE_ORDER, first["seam_phases"]))
        self.assertAlmostEqual(by_name["purge_time_s"], 0.5, places=3)
        self.assertAlmostEqual(by_name["preflow_time_s"], 0.2, places=3)
        self.assertAlmostEqual(by_name["postflow_time_s"], 0.5, places=3)
        self.assertAlmostEqual(by_name["craterfill_time_s"], 0.25, places=3)
        self.assertAlmostEqual(by_name["craterfill_wire_feed_speed"], 350.0, places=3)
        # The preset with no seam phases authored serves the all-zero row --
        # which is what CLEARS the previous weld's phases from the PERS.
        self.assertEqual(robot.weld_params[1]["seam_phases"],
                         list(abb_server.DEFAULT_SEAM_PHASES))

    def test_the_seam_phase_formatter_refuses_a_wrong_count(self):
        """Padding a short row would shift every later value into the wrong
        seamdata component, silently."""
        with self.assertRaises(ValueError):
            abb_server.fmt_seam_phases([0.0] * 8)
        with self.assertRaises(ValueError):
            abb_server.fmt_seam_phases([0.0] * 10)

    def test_the_seam_phase_payload_is_byte_identical_to_the_production_hmi(self):
        """The literal is PINNED, and pinned in both languages.

        The tests above parse the payload back into floats, so they pass whatever
        the formatting -- which is exactly how this prototype and the C++ HMI came
        to encode the same row differently (2026-09-20). This side sent
        "[0.500,...]"; WeldParameterWire::FormatSeamPhases sent
        "[+0000.500,...]", 91 characters, past RAPID's 80-char string limit. Every
        green test here was validating a payload the robot never receives.

        Change this string only together with the matching CHECK in
        TGuideWeldingHMI/tests/unit/weld_parameter_wire_tests.cpp.
        """
        row = [0.5, 0.2, 0.5, 0.08, 0.25, 350.0, 3.0, 300.0, 2.0]
        self.assertEqual(abb_server.fmt_seam_phases(row),
                         "[0.50,0.20,0.50,0.08,0.25,350.00,3.00,300.00,2.00]")
        self.assertEqual(abb_server.fmt_seam_phases([0.0] * 9),
                         "[0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00]")

    def test_the_seam_phase_payload_fits_the_rapid_string_limit(self):
        """SocketReceive \\Str does not error past 80 chars: it truncates and
        leaves the tail in the TCP buffer, where it becomes the reply to the NEXT
        prompt and desyncs the rest of the session."""
        worst = [-123456.0] * 9          # clamps to -999.99, the widest field
        payload = abb_server.fmt_seam_phases(worst)
        self.assertEqual(payload, "[-999.99,-999.99,-999.99,-999.99,-999.99,"
                                  "-999.99,-999.99,-999.99,-999.99]")
        self.assertEqual(len(payload), 73)

        for value in (0.0, -0.001, 800.0, -999.99, 999.99, 1e9, -1e9):
            with self.subTest(value=value):
                self.assertLessEqual(len(abb_server.fmt_seam_phases([value] * 9)),
                                     abb_server.RAPID_STRING_MAX)

    def test_seam_phase_values_are_rapid_num_literals(self):
        """No leading '+'. The standalone scalar replies go through tgParseReal,
        which strips the FANUC plus sign; this payload reaches StrToVal directly
        and nothing strips it, so it must never be emitted. Negatives keep their
        sign -- a Fronius arc-length correction is signed."""
        self.assertEqual(abb_server.fmt_seam_phase_value(0.5), "0.50")
        self.assertEqual(abb_server.fmt_seam_phase_value(-3.5), "-3.50")
        self.assertNotIn("+", abb_server.fmt_seam_phases([-3.5] + [0.0] * 8))

    def test_touchup_pushed_once_per_weld_frame(self):
        """Phase 7: every R_W_F reply carries the stored touch-up offset."""
        robot = FakeWeldRobot()
        hmi = run_cycle(robot)
        self.assertEqual(len(robot.touchups), 2)
        for got in robot.touchups:
            for g, w in zip(got, hmi.touchup_offsets_in):
                self.assertAlmostEqual(g, w, places=3)

    def test_weld_abort_skips_the_second_weld(self):
        """nTG_WeldStatus=2 -> GOTO abort_end, so no further R_W_P."""
        robot = FakeWeldRobot()

        def one_abort(hmi):
            hmi.weld_status = 2

        run_cycle(robot, hmi_setup=one_abort)
        self.assertEqual(len(robot.weld_params), 0,
                         "an aborted weld must not request parameters")

    def test_weld_skip_requests_no_parameters(self):
        """nTG_WeldStatus=0 -> the whole IF body is skipped."""
        robot = FakeWeldRobot()

        def skip(hmi):
            hmi.weld_status = 0

        run_cycle(robot, hmi_setup=skip)
        self.assertEqual(len(robot.weld_params), 0)
        self.assertEqual(robot.sub_names, ["PWeld2", "PWeld3"],
                         "a skipped weld is still announced and frame-checked")


class TestWeldDataMapping(unittest.TestCase):
    """The conversions TG_Weld.sys TG_ApplyWeldParams must perform.

    These assert the arithmetic the RAPID side is expected to produce, so a
    mismatch between this file and a VC transcript localises the bug.
    """

    def test_ipm_to_mm_s_constant_matches_rapid(self):
        # TG_Weld.sys: LOCAL CONST num nTG_IpmToMmS := 0.4233333;
        self.assertAlmostEqual(IPM_TO_MM_S, 0.4233333, places=6)

    def test_travel_speed_conversion(self):
        # 21 IPM is the HMI's native-.tgs default travel speed.
        self.assertAlmostEqual(21.0 * IPM_TO_MM_S, 8.890, places=3)
        self.assertAlmostEqual(30.0 * IPM_TO_MM_S, 12.700, places=3)

    def test_wire_feed_conversion(self):
        # 520 IPM is the HMI's native-.tgs default wire feed speed.
        self.assertAlmostEqual(520.0 * IPM_TO_MM_S, 220.133, places=3)

    def test_expected_table_agrees_with_the_sequence(self):
        for entry, expected in zip(WELD_DEMO_SEQUENCE,
                                   WELD_DEMO_EXPECTED_MM_S):
            self.assertAlmostEqual(
                entry["travel_speed"] * IPM_TO_MM_S,
                expected["weld_speed"], places=6)

    def test_arc_length_default_is_out_of_fronius_range(self):
        """Why TG_ApplyWeldParams must clamp: the HMI's own DEFAULT arc
        length (49.0) exceeds the Fronius correction range (about +/-10),
        so an unclamped port would raise 'correction outside specified
        range' on the very first weld."""
        self.assertGreater(WELD_DEMO_SEQUENCE[0]["arc_length"], 10.0)
        self.assertEqual(WELD_DEMO_EXPECTED_MM_S[0]["arc_length_clamped"], 10.0)

    def test_vc_measurement_confirms_mm_s_interpretation(self):
        """Guard on the measurement that settled the units question."""
        measured_seconds = 34.429 - (141.421 / 200.0)   # minus the approach
        implied = 300.0 / measured_seconds
        self.assertAlmostEqual(implied, 21.0 * IPM_TO_MM_S, delta=0.02)


if __name__ == "__main__":
    unittest.main(verbosity=2)

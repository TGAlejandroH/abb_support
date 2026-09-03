"""Phase 6 tests: R_W_S (id 13, welding stats).

Run:  python -m unittest discover -s hmi_prototype -v
 or:  python hmi_prototype/test_phase6_weldstats.py

The request is the simplest in the family - id, then ONE csv message
"dist_mm,arc_on_s,succ_ae", no pose and no sub token - so these tests focus
on the two things that can actually go wrong in a port:

1. the NUMBER FORMAT, since two different producers exist (FANUC KAREL
   CNV_REAL_STR blank-pads, RAPID tgFmtReal signs and zero-pads) and one
   consumer, std::stringstream >> double, has to accept both; and
2. the ANALYTICS GATE - the real HMI records a weld only when succ_ae == 1
   (RobotCell.cpp case RequestWeldingStats), which is what makes a dry run
   leave no trace in the database.

The values asserted here are the dummies the RAPID modules currently send
(TD05Test.mod, TD05Weld.mod). When the real sensing lands (see
docs/abb_weld_stats_port_v1.md section 6) the payload VALUES change and
these expectations move with them - the wire shape does not.
"""

import contextlib
import io
import socket
import threading
import unittest

from abb_server import AbbTgsHmi, fmt_real
from test_phase2 import TD05TEST_WELD_STATS, FakeTgsRobot, run_one_cycle
from test_phase4_weld import TD05WELD_WELD_STATS, FakeWeldRobot, run_cycle


class OneRequestRobot(threading.Thread):
    """Minimal robot: connect, serve program selection, send ONE R_W_S.

    Lets a single stats payload be tested without dragging a whole cycle
    behind it - including payloads no current RAPID module produces, such as
    FANUC's blank-padded CNV_REAL_STR form.
    """

    def __init__(self, payload):
        super().__init__(daemon=True)
        self.payload = payload
        self.errors = []
        self.acks = []
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]

    def run(self):
        try:
            conn, _ = self.listener.accept()
            with conn:
                conn.sendall(b"Give me the program ID")
                conn.recv(1024)
                for message in ("13", self.payload):
                    conn.sendall(message.encode("utf-8"))
                    self.acks.append(conn.recv(16))
        except Exception as exc:
            self.errors.append(exc)
        finally:
            self.listener.close()


def serve_one_stats_payload(payload, verbose=False):
    """Run OneRequestRobot against a fresh HMI; return the HMI."""
    robot = OneRequestRobot(payload)
    robot.start()
    hmi = AbbTgsHmi(host="127.0.0.1", port=robot.port, verbose=verbose)
    hmi.serve_cycle()
    robot.join(timeout=5)
    if robot.errors:
        raise robot.errors[0]
    return hmi


class TestStatsPayloadParsing(unittest.TestCase):
    """One payload in, the HMI's three derived facts out."""

    def test_rapid_format_parses(self):
        hmi = serve_one_stats_payload("+0123.456,+0007.890,+0001.000")
        self.assertEqual(hmi.last_weld_stats, (123.456, 7.89, 1.0))

    def test_fanuc_cnv_real_str_format_parses(self):
        """FANUC's own producer blank-pads to a minimum width with at least
        one leading blank and no '+' (KAREL CNV_REAL_STR(v,8,3)). The C++
        consumer skips whitespace, so the same handler must accept it - this
        is what keeps ONE HMI implementation valid for both robot brands."""
        hmi = serve_one_stats_payload(" 123.456,   7.890,   1.000")
        self.assertEqual(hmi.last_weld_stats, (123.456, 7.89, 1.0))

    def test_distance_is_converted_to_inches_for_analytics(self):
        """The robot sends mm; AnalyticsManager::InsertWeldEntry takes
        (length_in, arc_on_sec), so the /25.4 happens HMI-side."""
        hmi = serve_one_stats_payload(fmt_real(254.0) + ",+0010.000,+0001.000")
        self.assertEqual(hmi.weld_stats_entries, [(10.0, 10.0)])

    def test_succ_ae_zero_is_received_but_not_recorded(self):
        """The dry-run / no-arc path: the HMI reads the stats and drops them
        (RobotCell.cpp only inserts when succ_ae is true)."""
        hmi = serve_one_stats_payload("+0123.456,+0007.890,+0000.000")
        self.assertEqual(hmi.last_weld_stats, (123.456, 7.89, 0.0))
        self.assertEqual(hmi.weld_stats_entries, [])

    def test_malformed_payload_is_rejected_loudly(self):
        """A truncated stream must not silently record a bogus weld."""
        with self.assertRaises(ValueError):
            serve_one_stats_payload("+0123.456,+0007.8")

    def test_both_messages_are_acked(self):
        """Two robot->HMI messages, so two acks - id and payload alike."""
        robot = OneRequestRobot("+0123.456,+0007.890,+0001.000")
        robot.start()
        hmi = AbbTgsHmi(host="127.0.0.1", port=robot.port, verbose=False)
        hmi.serve_cycle()
        robot.join(timeout=5)
        self.assertEqual(robot.acks, [b"0", b"0"])


class TestVerboseTranscript(unittest.TestCase):
    """The operator-facing log line is what the VC checks are read from, and
    every other test runs quiet - so exercise it explicitly."""

    def _serve_verbose(self, payload):
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            serve_one_stats_payload(payload, verbose=True)
        return buffer.getvalue()

    def test_recorded_weld_line(self):
        out = self._serve_verbose("+0123.456,+0007.890,+0001.000")
        self.assertIn("123.456 mm (4.860 in)", out)
        self.assertIn("arc on 7.890 s", out)
        self.assertIn("-> recorded", out)

    def test_dropped_weld_line_says_why(self):
        out = self._serve_verbose("+0123.456,+0007.890,+0000.000")
        self.assertIn("NOT recorded (succ_ae != 1)", out)


class TestStatsInTheTD05TestCycle(unittest.TestCase):
    """R_W_S inside the full non-Arc regression cycle (TD05Test.mod)."""

    def test_served_once_after_weld_params(self):
        robot, hmi = run_one_cycle()
        self.assertEqual(hmi.request_log.count("13"), 1)
        # FANUC order: R_W_P, then WELD END, then R_W_S, then R_E.
        self.assertEqual(hmi.request_log[-3:], ["14", "13", "100"])

    def test_module_dummy_values_arrive_intact(self):
        robot, hmi = run_one_cycle()
        dist_mm, arc_on_sec, succ_ae = hmi.last_weld_stats
        self.assertAlmostEqual(dist_mm, TD05TEST_WELD_STATS[0], places=3)
        self.assertAlmostEqual(arc_on_sec, TD05TEST_WELD_STATS[1], places=3)
        self.assertEqual(succ_ae, 1.0)
        self.assertEqual(len(hmi.weld_stats_entries), 1)
        length_in, _ = hmi.weld_stats_entries[0]
        self.assertAlmostEqual(length_in, 123.456 / 25.4, places=4)

    def test_payload_is_the_fixed_width_wire_format(self):
        robot, hmi = run_one_cycle()
        self.assertEqual(robot.received["weld_stats_raw"],
                         "+0123.456,+0007.890,+0001.000")
        # 3 signed 9-char fields + 2 commas, well inside RAPID's 80-char
        # string limit (the payload is built by concatenation in RAPID).
        self.assertEqual(len(robot.received["weld_stats_raw"]), 29)

    def test_dry_run_reports_no_arc_and_records_nothing(self):
        """A dry run still SERVES the request (FANUC calls R_W_S
        unconditionally inside the weld branch) but must leave the analytics
        untouched - nTG_SuccArcEnd := 1 - nTG_DryRun."""
        robot, hmi = run_one_cycle({"dry_run": 1})
        self.assertEqual(hmi.request_log.count("13"), 1)
        self.assertEqual(robot.received["weld_stats_raw"],
                         "+0123.456,+0007.890,+0000.000")
        self.assertEqual(hmi.weld_stats_entries, [])

    def test_skipped_weld_serves_no_stats(self):
        """nTG_WeldStatus=0 skips the whole weld body, R_W_S included."""
        robot, hmi = run_one_cycle({"weld_status": 0})
        self.assertNotIn("13", hmi.request_log)
        self.assertIsNone(hmi.last_weld_stats)

    def test_aborted_weld_serves_no_stats(self):
        robot, hmi = run_one_cycle({"weld_status": 2})
        self.assertNotIn("13", hmi.request_log)


class TestStatsInTheTwoWeldCycle(unittest.TestCase):
    """R_W_S once per weld in the Arc program (TD05Weld.mod)."""

    def test_two_independent_servings(self):
        robot = FakeWeldRobot()
        hmi = run_cycle(robot)
        self.assertEqual(hmi.request_log.count("13"), 2)
        self.assertEqual(len(hmi.weld_stats_entries), 2)

    def test_per_weld_values_differ_and_match_the_module(self):
        robot = FakeWeldRobot()
        hmi = run_cycle(robot)
        expected = [",".join(fmt_real(v) for v in stats + (1,))
                    for stats in TD05WELD_WELD_STATS]
        self.assertEqual(robot.weld_stats_sent, expected)
        self.assertNotEqual(robot.weld_stats_sent[0], robot.weld_stats_sent[1])
        # last_weld_stats holds the SECOND weld once the cycle is over
        self.assertAlmostEqual(hmi.last_weld_stats[1],
                               TD05WELD_WELD_STATS[1][1], places=3)

    def test_arc_on_times_agree_with_the_served_weld_speeds(self):
        """The dummies are not arbitrary: each is the 200 mm seam divided by
        the weld speed that weld was actually served, so a transcript can be
        sanity-checked against the run itself."""
        from abb_server import IPM_TO_MM_S, WELD_DEMO_SEQUENCE
        for (dist_mm, arc_on_sec), served in zip(TD05WELD_WELD_STATS,
                                                 WELD_DEMO_SEQUENCE):
            mm_per_s = served["travel_speed"] * IPM_TO_MM_S
            self.assertAlmostEqual(dist_mm / mm_per_s, arc_on_sec, places=2)

    def test_aborted_second_weld_leaves_one_serving(self):
        robot = FakeWeldRobot()

        def abort(hmi):
            hmi.weld_status = 2

        hmi = run_cycle(robot, hmi_setup=abort)
        self.assertNotIn("13", hmi.request_log)
        self.assertEqual(hmi.weld_stats_entries, [])


class TestPerCycleReset(unittest.TestCase):

    def test_entries_do_not_leak_between_cycles(self):
        """The real HMI opens one weld session per run; the prototype's
        entry list must start empty on every cycle."""
        _, hmi = run_one_cycle()
        self.assertEqual(len(hmi.weld_stats_entries), 1)
        robot = FakeTgsRobot()
        robot.start()
        hmi.host, hmi.port = "127.0.0.1", robot.port
        hmi.serve_cycle()
        robot.join(timeout=10)
        self.assertEqual(len(hmi.weld_stats_entries), 1,
                         "second cycle must not accumulate the first's rows")


if __name__ == "__main__":
    unittest.main(verbosity=2)

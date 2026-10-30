"""Offline tests of the run recorder's analysis and checks.

The data files are real recordings from VC A (MonarchRS 4600-803651, 2026-10-28), trimmed to the
weld module +- 1 s. Each known-bad run must turn specific checks red, and the good runs must stay
green, so a change to the analysis cannot quietly stop catching what it was built to catch:

  vc_a_base_fixed        the planner's fixed export (E59 fix): one 20 mm weld at 10 IPM
  vc_a_e59_site_module   the site's pre-fix module: ArcLStart and ArcLEnd on the same target
  vc_a_zero_weld_speed   the H17 values (travel 0, wire feed 0): the robot stands on the seam
  vc_a_two_presets       two welds, two presets (10 IPM / sched 1, 25 IPM / sched 2)
  vc_a_weave             the fixed weld with a 4 mm x 3 mm zig-zag weave
  vc_a_coord_turning     a coordinated weld on station 2: 66.8 mm at 21 IPM in wobjTG_WeldActStn
                         while the plate turns 6.86 -> -17.26 deg (the site project, re-exported
                         for VC A's station frame)
  vc_a_hmi_three_presets the real HMI driving F3: three indexed welds, preset A / preset B /
                         custom, frames served at plate 0, 45 and tilt 90
  vc_a_hmi_refused_zone  the real HMI refusing F8: work zone 1 while station 2 is presented
  vc_a_hmi_weave_50501   the real HMI driving F2: weld 1 weaves; RobotWare warns 50501 at the
                         zero-length entry into weld 3 (a planner export finding)

Run: python -m unittest discover -s tools/run_recorder/tests
"""
import json
import os
import sqlite3
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from analysis import analyse, parse_welddata  # noqa: E402
from checks import (check, expected_welddata, frame_error, inv4, matrix_pose, mul4,  # noqa: E402
                    planted_frame, planted_station_frame, pose_matrix)
from hmi_case import part_in_station  # noqa: E402
from rapid_module import ModuleInfo  # noqa: E402
from sampler import _unit_inactive, channel_key  # noqa: E402


def load(name):
    with open(os.path.join(HERE, "data", name + ".json")) as fh:
        recording = json.load(fh)
    recording["analysis"] = analyse(recording)
    return recording


def failing(rows):
    return [(rid, text) for rid, ok, text in rows if ok is False]


class ModuleParsing(unittest.TestCase):
    TEXT = "\n".join([
        "MODULE T1_Mod",
        "    LOCAL CONST robtarget pA := [[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];",
        "    LOCAL CONST robtarget pB := [[30,40,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];",
        "    LOCAL CONST robtarget pC := [[30,40,10],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];",
        "    LOCAL PERS weavedata wvTG001 := [1,0,4,3,0,0,0,0,0,0,0,0,3,0,0];",
        "    PROC T1()",
        "        MoveL pA, v100, fine, tTG_Weld \\WObj:=wobjTG_Weld;",
        "        ArcLStart pA, v100, sd, wd \\Weave:=wvTG001, fine, tTG_Weld \\WObj:=wobjTG_Weld;",
        "        ArcL pB, v100, sd, wd \\Weave:=wvTG001, z1, tTG_Weld \\WObj:=wobjTG_Weld;",
        "        ArcLEnd pC, v100, sd, wd \\Weave:=wvTG001, fine, tTG_Weld \\WObj:=wobjTG_Weld;",
        "        ArcLStart pC, v100, sd, wd, fine, tTG_Weld \\WObj:=wobjTG_WeldActStn;",
        "        ArcLEnd pC, v100, sd, wd, fine, tTG_Weld \\WObj:=wobjTG_WeldActStn;",
        "    ENDPROC",
        "ENDMODULE"])

    def test_spans_lines_targets_and_work_object(self):
        info = ModuleInfo(self.TEXT)
        spans = info.arc_spans()
        self.assertEqual(info.module_name, "T1_Mod")
        self.assertEqual([s["lines"] for s in spans], [[8, 9, 10], [11, 12]])
        self.assertEqual(spans[0]["targets"], ["pA", "pB", "pC"])
        self.assertEqual(spans[0]["weave"], "wvTG001")
        self.assertEqual(spans[1]["wobj"], "wobjTG_WeldActStn")

    def test_planned_length_follows_the_targets(self):
        info = ModuleInfo(self.TEXT)
        spans = info.arc_spans()
        self.assertAlmostEqual(info.planned_length(spans[0]), 50.0 + 10.0)
        self.assertEqual(info.planned_length(spans[1]), 0.0)     # the E59 shape

    def test_declarations_are_read(self):
        self.assertEqual(ModuleInfo(self.TEXT).declarations["wvTG001"][1], "weavedata")

    def test_robtarget_external_axes_are_read_without_9e9(self):
        info = ModuleInfo("LOCAL CONST robtarget pX := [[1,2,3],[1,0,0,0],[0,-1,1,0],"
                          "[0.000,0.000,-17.256,9E9,9E9,9E9]];")
        self.assertEqual(info.extax["pX"], {"eax_a": 0.0, "eax_b": 0.0, "eax_c": -17.256})


class Sampling(unittest.TestCase):
    def test_moving_work_object_is_read_only_while_its_unit_is_active(self):
        """A robtarget read in the work object of a deactivated unit logs 50066 (VC A)."""
        channel = ("tcp", "tTG_Weld", "wobjTG_WeldActStn", "STN2")
        self.assertTrue(_unit_inactive(channel, {"joints:STN2": {}}))
        self.assertTrue(_unit_inactive(channel, {}))
        self.assertFalse(_unit_inactive(channel, {"joints:STN2": {"rax_1": "0", "rax_2": "6.9"}}))
        self.assertFalse(_unit_inactive(("tcp", "tTG_Weld", "wobj0"), {}))
        self.assertEqual(channel_key(channel), "tcp:tTG_Weld:wobjTG_WeldActStn")


class Welddata(unittest.TestCase):
    def test_parse(self):
        wd = parse_welddata("[4.23333,4.23333,[1,0,0,220.133,0,0,0,0,0],[1,0,0,220.133,0,0,0,0,0]]")
        self.assertEqual((wd["weld_speed"], wd["sched"], wd["wirefeed"]), (4.23333, 1.0, 220.133))

    def test_expected_values_follow_tg_weld_sys(self):
        served = {"travel_speed": 10.0, "wire_feed_speed": 520.0, "weld_sched": 1,
                  "arc_length": 49.0, "arc_control": 0.0, "welder_type": 2}
        want = expected_welddata(served)
        self.assertAlmostEqual(want["weld_speed"], 4.233333, places=5)
        self.assertEqual(want["voltage"], 10.0)                  # Fronius path clamps 49 to +10
        served["welder_type"] = 1
        self.assertEqual(expected_welddata(served)["voltage"], 49.0)   # Miller passes it through


class GoodRuns(unittest.TestCase):
    def test_fixed_export_passes_everything(self):
        rec = load("vc_a_base_fixed")
        self.assertEqual(failing(check(rec, rec["sent"], rec["expect"])), [])
        sp = rec["analysis"]["spans"][0]
        self.assertAlmostEqual(sp["travel_seconds"], 20.0 / 4.23333, delta=0.25)
        self.assertAlmostEqual(sp["weld_on"]["seconds"], 20.0 / 4.23333, delta=0.25)

    def test_two_presets_each_weld_gets_its_own_values(self):
        rec = load("vc_a_two_presets")
        self.assertEqual(failing(check(rec, rec["sent"], rec["expect"])), [])
        a, b = rec["analysis"]["spans"]
        self.assertEqual((a["welddata"]["sched"], b["welddata"]["sched"]), (1.0, 2.0))
        self.assertAlmostEqual(b["welddata"]["weld_speed"], 25 * 0.4233333, places=3)
        self.assertAlmostEqual(b["travel_seconds"], 20.0 / (25 * 0.4233333), delta=0.2)   # a 1.9 s weld

    def test_coordinated_weld_with_the_positioner_turning(self):
        rec = load("vc_a_coord_turning")
        self.assertEqual(failing(check(rec, rec["sent"], rec["expect"])), [])
        sp = rec["analysis"]["spans"][0]
        self.assertEqual(sp["wobj"], "wobjTG_WeldActStn")
        plate = sp["positioner"]["axes"]["eax_c"]
        self.assertEqual(plate["rax"], "rax_2")
        self.assertAlmostEqual(plate["observed"][0], 6.857, delta=0.05)
        self.assertAlmostEqual(plate["observed"][1], -17.256, delta=0.05)
        # the speed relative to the turning part, not the TCP's speed in space
        self.assertAlmostEqual(sp["speed_mm_s"], 21 * 0.4233333, delta=0.45)
        self.assertNotIn("eax_a", sp["positioner"]["axes"])   # exported for a rail the cell lacks

    def test_weave_is_read_and_not_counted_as_travel(self):
        rec = load("vc_a_weave")
        self.assertEqual(failing(check(rec, rec["sent"], rec["expect"])), [])
        sp = rec["analysis"]["spans"][0]
        self.assertGreater(sp["path_mm"], 30.0)                  # the zig-zag is in the TCP path
        self.assertAlmostEqual(sp["speed_mm_s"], 4.23, delta=0.45)   # but not in the seam speed


class KnownBadRuns(unittest.TestCase):
    def test_e59_site_module_is_caught(self):
        rec = load("vc_a_e59_site_module")
        bad = failing(check(rec, rec["sent"], rec["expect"]))
        texts = " | ".join(t for _, t in bad)
        self.assertIn("welds nothing (the E59 shape)", texts)
        self.assertIn("weld-on output high", texts)
        self.assertLess(rec["analysis"]["spans"][0]["weld_on"]["seconds"], 0.5)

    def test_e59_is_caught_by_the_span_check_alone(self):
        """Without the planner's seam length the module's own length is 0 - still red."""
        rec = load("vc_a_e59_site_module")
        self.assertIn("A1", [rid for rid, _ in failing(check(rec, rec["sent"], None))])

    def test_zero_weld_speed_is_caught_as_a_hang(self):
        rec = load("vc_a_zero_weld_speed")
        bad = failing(check(rec, rec["sent"], rec["expect"]))
        self.assertIn("A1", [rid for rid, _ in bad])
        self.assertTrue(any(rid == "A5" and "program pointer on the span" in t for rid, t in bad))
        self.assertGreater(rec["analysis"]["spans"][0]["pointer_seconds"], 60)

    def test_weld_data_served_wrong_is_caught(self):
        rec = load("vc_a_base_fixed")
        sent = json.loads(json.dumps(rec["sent"]))
        sent["weld_params"][0]["travel_speed"] = 21.0
        self.assertTrue(any(rid == "A2" and "differs" in t for rid, t in failing(check(rec, sent, rec["expect"]))))

    def test_positioner_that_did_not_turn_is_caught(self):
        rec = load("vc_a_coord_turning")
        for s in rec["samples"]:
            if s.get("joints:STN2"):
                s["joints:STN2"]["rax_2"] = "6.857"
        rec["analysis"] = analyse(rec)
        bad = failing(check(rec, rec["sent"], rec["expect"]))
        self.assertTrue(any(rid == "A1" and "positioner STN2 eax_c" in t for rid, t in bad))

    def test_positioner_read_at_the_fine_stop_not_on_the_move(self):
        """The seam start is fitted from the TCP, so the last reading before it can already be
        moving (a fast-turning plate). The reading compared is the one at the fine stop."""
        from analysis import _standstill
        before = [253.757, 253.757, 253.757, 244.66]     # standing at ArcLStart, then turning
        after = [-91.4, -91.457, -91.457]                # arriving at ArcLEnd, then standing
        self.assertEqual(_standstill(before, last=True), 253.757)
        self.assertEqual(_standstill(after, last=False), -91.457)
        self.assertEqual(_standstill([1.0, 2.0, 3.0], last=True), 3.0)   # never still: nearest
        self.assertIsNone(_standstill([], last=True))

    def test_a_weld_never_reached_is_caught(self):
        rec = load("vc_a_base_fixed")
        rec["samples"] = [s for s in rec["samples"]
                          if not (s.get("pointers") and s["pointers"][0] and s["pointers"][0][2] in (69, 70))]
        rec["analysis"] = analyse(rec)
        self.assertTrue(any("never reached" in t for _, t in failing(check(rec, rec["sent"], rec["expect"]))))


class HmiInTheLoop(unittest.TestCase):
    """Recordings of runs the real HMI drove (hmi_case.ps1), checked against the fixture's
    expected values: what the project says the HMI must send, and the frames it must serve."""

    def test_three_presets_each_weld_as_the_project_says(self):
        rec = load("vc_a_hmi_three_presets")
        self.assertEqual(failing(check(rec, rec["sent"], rec["expect"])), [])
        scheds = [sp["welddata"]["sched"] for sp in rec["analysis"]["spans"]]
        self.assertEqual(scheds, [1.0, 3.0, 1.0])               # preset A, preset B, custom
        a6 = [ok for rid, ok, _ in check(rec, rec["sent"], rec["expect"]) if rid == "A6"]
        self.assertEqual(a6, [True, True, True])                # frames at plate 0, 45, tilt 90

    def test_a_weld_inheriting_the_previous_values_is_caught(self):
        """P9 / E54: weld 2 must not run with weld 1's values."""
        rec = load("vc_a_hmi_three_presets")
        expect = json.loads(json.dumps(rec["expect"]))
        expect["welds"][1]["weld_params"] = dict(expect["welds"][0]["weld_params"])
        bad = failing(check(rec, rec["sent"], expect))
        self.assertTrue(any(rid == "A2" and t.startswith("weld 2: weld data differs") for rid, t in bad))

    def test_a_frame_served_wrong_is_caught(self):
        rec = load("vc_a_hmi_three_presets")
        expect = json.loads(json.dumps(rec["expect"]))
        expect["welds"][2]["frame"][0] += 0.5
        bad = failing(check(rec, rec["sent"], expect))
        self.assertTrue(any(rid == "A6" and t.startswith("weld 3:") for rid, t in bad))

    def test_a_refused_part_passes_as_refused_and_fails_as_a_run(self):
        rec = load("vc_a_hmi_refused_zone")
        self.assertEqual(failing(check(rec, rec["sent"], rec["expect"])), [])
        self.assertIsNone(rec["module"]["module"])
        self.assertTrue(failing(check(rec, rec["sent"], {"welds": [{"seam_length_mm": 20.0}]})))

    def test_weave_as_set_on_the_weld_and_the_short_movement_warning(self):
        """F2 on VC A, 2026-10-28: the weave is right; the transition into Weld3 ends in two
        identical jointtargets plus the zero-length chain-entry MoveL, and RobotWare warns."""
        rec = load("vc_a_hmi_weave_50501")
        bad = failing(check(rec, rec["sent"], rec["expect"]))
        self.assertEqual([rid for rid, _ in bad], ["A3"])
        self.assertIn("50501", bad[0][1])
        weave = [t for rid, ok, t in check(rec, rec["sent"], rec["expect"]) if "weave in effect" in t]
        self.assertTrue(weave[0].startswith("weld 1: weave in effect [1.0, 0.0, 4.0, 3.0"))
        expect = json.loads(json.dumps(rec["expect"]))
        expect["welds"][0]["weave"] = None                       # the weld does not weave
        self.assertTrue(any("weave in effect" in t for _, t in failing(check(rec, rec["sent"], expect))))


class CameraTestMode(unittest.TestCase):
    """HMI_SIMULATION_PART_OFFSET: the served frames must move with the planted part (A6)."""

    OFFSET = [5.0, -3.0, 2.0, 2.0, -1.0, 0.5]
    # VC A's station 2 frame and F4's module seed (wobjTG_WeldActStn.oframe), mm.
    STATION = [1370.14, 797.92, 416.57, 0.536457, 0.537943, -0.460038, 0.459561]
    SEED = [-621.543, 1290.355, 797.596, 0.536457, -0.537943, 0.460038, -0.459561]

    def test_a_seed_that_is_the_part_frame_moves_as_a_base_frame(self):
        mm, deg = frame_error(planted_station_frame(self.SEED, self.OFFSET, self.SEED),
                              planted_frame(self.SEED, self.OFFSET))
        self.assertLess(mm, 1e-6)
        self.assertLess(deg, 1e-4)

    def test_f4_on_vc_a_the_served_frame_is_the_part_moved_in_the_station_frame(self):
        """PA_F4_stn2, 2026-10-30: the frame the robot ran with, against the expectation."""
        part = [-4.139, -0.083, -86.814, 0.003692, -0.000820, 0.000556, -0.999993]   # part_in_station(F4)
        served = [-656.300, 1279.220, 798.800, 0.537991, -0.552279, 0.453050, -0.447548]
        mm, deg = frame_error(served, planted_station_frame(self.SEED, self.OFFSET, part))
        self.assertLess(mm, 0.05)
        self.assertLess(deg, 0.01)
        mm, _ = frame_error(served, planted_frame(self.SEED, self.OFFSET))   # the seed is not the part
        self.assertGreater(mm, 30.0)

    def test_part_pose_in_the_station_frame_is_read_from_the_project(self):
        part_world_at_capture = [1369.5, 884.7, 412.6, 0.462235, 0.461325, 0.536162, -0.534835]
        index = pose_matrix([10.0, -20.0, 5.0, 0.991445, 0.0, 0.0, 0.130526])   # 15 deg about Z
        local = [[0.0, 0.0, 0.0], [120.0, 5.0, -3.0], [10.0, 80.0, 2.0], [-40.0, 30.0, 60.0]]
        f = pose_matrix(part_world_at_capture)
        world = [[(sum(f[i][k] * p[k] for k in range(3)) + f[i][3]) / 1000.0 for i in range(3)] for p in local]
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "p.tgs")
            con = sqlite3.connect(path)
            con.execute("create table external_axis_groups (metadata_json text)")
            con.execute("create table weld_global_capture_sets (capture_set_id text, order_index int, payload_json text)")
            con.execute("create table weld_global_capture_poses (capture_set_id text, order_index int, "
                        "geometry_payload_json text)")
            pose = {"xyz_m": [v / 1000.0 for v in self.STATION[:3]], "quat_wxyz": self.STATION[3:]}
            con.execute("insert into external_axis_groups values (?)", (json.dumps(
                {"export_v1": {"motion_role": "extended_axis", "brands": {"abb": {"station_frame_pose": pose}}}}),))
            index_m = [[v / 1000.0 if j == 3 and i < 3 else v for j, v in enumerate(r)] for i, r in enumerate(index)]
            con.execute("insert into weld_global_capture_sets values ('s1', 0, ?)",
                        (json.dumps({"positioner_part_transform_base": index_m}),))
            con.execute("insert into weld_global_capture_poses values ('s1', 0, ?)", (json.dumps(
                {"point_cloud_cad_local_mm": local, "point_cloud_world_m": world}),))
            con.commit()
            con.close()
            got = part_in_station(path)
        want = matrix_pose(mul4(inv4(pose_matrix(self.STATION)), mul4(inv4(index), f)))
        mm, deg = frame_error(got, want)
        self.assertLess(mm, 1e-6)
        self.assertLess(deg, 1e-4)

    def test_a_coordinated_weld_is_not_passed_without_the_part_pose(self):
        rec = load("vc_a_coord_turning")
        expect = json.loads(json.dumps(rec["expect"]))
        expect["planted_part_offset"] = self.OFFSET
        a6 = [(ok, t) for rid, ok, t in check(rec, rec["sent"], expect) if rid == "A6"]
        self.assertEqual([ok for ok, _ in a6], [None])
        self.assertIn("cannot be checked", a6[0][1])


if __name__ == "__main__":
    unittest.main()

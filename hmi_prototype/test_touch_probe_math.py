"""Touch-sense P0: the judge of TG_TouchProbe.mod results (vc_probes/touch_probe_math.py).

These pin what each verdict row accepts and, as importantly, what it REJECTS. The two
failure modes the probe exists to catch must each turn a specific row red:
- SearchL running the full stroke on a VC (curobo E17);
- SearchPoint coming back in the wrong frame (the F-2 class).

Run: python -m unittest discover -s hmi_prototype
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "vc_probes"))
import touch_probe_math as tpm  # noqa: E402

FACE = 1000.0
START = (1500.0, 20.0, 1050.0)
TO = (1500.0, 20.0, 900.0)
Q_TOOL = (0.5, 0.0, 0.8660254, 0.0)


def rt(trans, rot=Q_TOOL):
    return {"trans": tuple(trans), "rot": tuple(rot)}


def failed(rows):
    return [name for name, ok, _ in rows if not ok]


class Parsing(unittest.TestCase):
    def test_numbers_reads_rws_exponent_forms(self):
        self.assertEqual(tpm.numbers("[9E+09,-1.5e-3,.5,7]"), [9e9, -0.0015, 0.5, 7.0])

    def test_robtarget_literal(self):
        r = tpm.parse_robtarget("[[1812.5,0,1398.25],[0.5,0,0.866025,0],[0,0,0,0],"
                                "[9E+09,9E+09,1.839,9E+09,9E+09,9E+09]]")
        self.assertEqual(r["trans"], (1812.5, 0.0, 1398.25))
        self.assertEqual(r["rot"], (0.5, 0.0, 0.866025, 0.0))

    def test_wobj_literal_skips_bools_and_empty_name(self):
        w = tpm.parse_wobj('[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[10,20,30],[0.7071068,0,0,0.7071068]]]')
        self.assertEqual(w["uframe"], ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)))
        self.assertEqual(w["oframe"], ((10.0, 20.0, 30.0), (0.7071068, 0.0, 0.0, 0.7071068)))

    def test_short_value_is_rejected(self):
        with self.assertRaises(ValueError):
            tpm.parse_robtarget("?")


class Geometry(unittest.TestCase):
    def test_quat_zyx_matches_a_plain_z_rotation(self):
        q = tpm.quat_zyx(90, 0, 0)
        for a, b in zip(tpm.quat_rotate(q, (1, 0, 0)), (0, 1, 0)):
            self.assertAlmostEqual(a, b, places=9)

    def test_quat_zyx_order_is_z_then_y_then_x(self):
        # Rz(90)*Ry(90): the x axis goes to -z under Ry, and Rz leaves -z alone.
        q = tpm.quat_zyx(90, 90, 0)
        for a, b in zip(tpm.quat_rotate(q, (1, 0, 0)), (0, 0, -1)):
            self.assertAlmostEqual(a, b, places=9)

    def test_pose_inverse_undoes_pose_apply(self):
        pose = ((100.0, -50.0, 300.0), tpm.quat_zyx(35, 10, -20))
        p = (12.0, -7.0, 41.0)
        back = tpm.pose_inverse_apply(pose, tpm.pose_apply(pose, p))
        for a, b in zip(back, p):
            self.assertAlmostEqual(a, b, places=9)

    def test_off_line(self):
        self.assertAlmostEqual(tpm.off_line((3, 4, 0), (0, 0, 0), (0, 0, 10)), 5.0)


def x1_result(hits=(999.80, 999.75, 999.82), stops=(999.40, 999.35, 999.45), step="X1 done", errno=0):
    return {"step": step, "errno": errno, "face_z": FACE, "to": rt(TO),
            "hit_z": list(hits), "stop_z": list(stops)}


class X1(unittest.TestCase):
    def test_a_real_search_passes_every_row(self):
        self.assertEqual(failed(tpm.x1_verdict(x1_result())), [])

    def test_e17_full_stroke_turns_the_stop_row_red(self):
        # SearchL behaving as a plain MoveL: the robot ends at the ToPoint.
        rows = tpm.x1_verdict(x1_result(stops=(900.0, 900.0, 900.0)))
        self.assertIn("X1 SearchL STOPPED on the DI (did not run the stroke to the ToPoint)", failed(rows))

    def test_searchpoint_equal_to_stop_point_is_caught(self):
        # If SearchPoint were the stop position, it would sit the overshoot deep, not at the face.
        rows = tpm.x1_verdict(x1_result(hits=(998.2, 998.1, 998.3), stops=(998.2, 998.1, 998.3)))
        self.assertIn("X1 SearchPoint AT the face (detection point, not stop point)", failed(rows))

    def test_the_measured_trigger_lead_passes_but_a_large_one_does_not(self):
        # VC 2026-09-25: the World Zone DO fired ~0.10 mm before the face at 15 mm/s.
        self.assertEqual(failed(tpm.x1_verdict(x1_result(hits=(1000.10, 1000.10, 1000.10),
                                                         stops=(999.15, 999.15, 999.15)))), [])
        rows = tpm.x1_verdict(x1_result(hits=(1001.0, 1001.0, 1001.0), stops=(999.9, 999.9, 999.9)))
        self.assertIn("X1 SearchPoint AT the face (detection point, not stop point)", failed(rows))

    def test_scatter_beyond_the_manual_is_caught(self):
        rows = tpm.x1_verdict(x1_result(hits=(999.9, 999.3, 999.8)))
        self.assertIn("X1 repeatable (TRM: 0.1-0.3 mm)", failed(rows))

    def test_an_error_is_not_a_pass(self):
        rows = tpm.x1_verdict(x1_result(step="X1 search 1 | ERRNO 1110", errno=1110))
        self.assertIn("X1 ran to completion", failed(rows))


def x2_result(part_frame_is_world=False):
    oframe = ((1580.0, -40.0, 930.0), tpm.quat_zyx(35, 10, -20))
    to_part = lambda w: tpm.pose_inverse_apply(oframe, w)
    hit_w = (START[0], START[1], FACE - 0.2)
    stop_w = (START[0], START[1], FACE - 0.6)
    q_part = (0.3, 0.1, 0.9, 0.3)
    hit_part = hit_w if part_frame_is_world else to_part(hit_w)
    return {"step": "X2 done", "errno": 0, "face_z": FACE,
            "wobj": {"uframe": ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)), "oframe": oframe},
            "start_part": rt(to_part(START), q_part), "to_part": rt(to_part(TO), q_part),
            "hit_part": rt(hit_part, q_part), "stop_part": rt(to_part(stop_w), q_part),
            "stop_w": rt(stop_w), "frame_err": 0.0, "off_line": 0.0}


class X2(unittest.TestCase):
    def test_searchpoint_in_the_work_object_passes(self):
        self.assertEqual(failed(tpm.x2_verdict(x2_result(), FACE - 0.25)), [])

    def test_searchpoint_in_world_coordinates_is_caught(self):
        # The F-2 class: numbers that look plausible but are in the wrong frame.
        rows = failed(tpm.x2_verdict(x2_result(part_frame_is_world=True), FACE - 0.25))
        self.assertIn("X2 oframe * SearchPoint lands AT the world face", rows)
        self.assertIn("X2 (diagnostic) SearchPoint is NOT world coordinates", rows)
        self.assertIn("X2 hit ON the part-frame search line", rows)

    def test_a_non_identity_uframe_is_flagged(self):
        r = x2_result()
        r["wobj"]["uframe"] = ((5.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
        self.assertIn("X2 uframe identity (THE RULE)", failed(tpm.x2_verdict(r, FACE - 0.25)))


class X3(unittest.TestCase):
    def test_miss_at_the_topoint_after_the_full_stroke(self):
        r = {"step": "X3a ERR_WHLSEARCH, back at S", "errno": 1110, "err_pos": rt(TO), "to": rt(TO),
             "search_s": 10.4, "stop": rt(START), "start": rt(START)}
        self.assertEqual(failed(tpm.x3a_verdict(r)), [])
        r["search_s"] = 0.8                       # an error raised at once is not a full stroke
        self.assertIn("X3a full 150 mm stroke at 15 mm/s", failed(tpm.x3a_verdict(r)))

    def test_supervision_error_at_the_start(self):
        r = {"step": "X3b ERR_SIGSUPSEARCH, back at S", "errno": 1111, "di_at_start": 1.0,
             "err_pos": rt((START[0], START[1], START[2] - 0.4)), "start": rt(START)}
        self.assertEqual(failed(tpm.x3b_verdict(r)), [])

    def test_pause_and_retry_needs_the_pause_to_have_been_seen(self):
        r = {"step": "X3c hit after 1 retry | caller resumed", "retries": 1.0, "err_pos": rt(TO),
             "to": rt(TO), "face_z": FACE, "hit": rt((START[0], START[1], FACE - 0.3))}
        self.assertEqual(failed(tpm.x3c_verdict(r, True)), [])
        self.assertIn("X3c paused on the miss (Stop inside the ERROR handler)", failed(tpm.x3c_verdict(r, False)))


class X6(unittest.TestCase):
    def test_overshoot_must_grow_with_speed(self):
        r = {"step": "X6 done", "errno": 0, "face_z": FACE, "hit_z": [999.5], "stop_z": [998.0]}
        self.assertEqual(failed(tpm.x6_verdict(r, 0.4)), [])
        self.assertIn("X6 overshoot larger than at 15 mm/s", failed(tpm.x6_verdict(r, 2.0)))

    def test_the_window_is_time_so_it_widens_with_speed(self):
        # VC 2026-09-25 at 50 mm/s: hit 0.43 mm above the face, i.e. the same ~9 ms trigger lead.
        r = {"step": "X6 done", "errno": 0, "face_z": FACE, "hit_z": [1000.43], "stop_z": [997.29]}
        self.assertEqual(failed(tpm.x6_verdict(r, 0.95)), [])
        lo, hi = tpm.hit_window(50.0)
        self.assertAlmostEqual(lo, -50.0 * tpm.LEAD_S)
        self.assertAlmostEqual(hi, 3.35)

    def test_the_lead_allowance_covers_the_rigs_measured_quantum(self):
        # X7 (2026-09-26): hits quantized at 0.36 mm at 15 mm/s, up to one quantum early.
        self.assertGreaterEqual(tpm.LEAD_S, tpm.RIG_PERIOD_S)
        self.assertAlmostEqual(15.0 * tpm.RIG_PERIOD_S, 0.36, places=6)
        self.assertLessEqual(tpm.hit_window(15.0)[0], -0.336)   # X7's deepest early hit


if __name__ == "__main__":
    unittest.main()


def p2_result(after=None, sensor_after=0.0, live_after=False, pauses=1.0, last_pause=3.0, step="S2 done"):
    """A TG_TsProbe scenario in an identity work object: start 50 mm above the face at z = 1000."""
    hit = (START[0], START[1], FACE + 0.1)
    return {"step": step, "hit_ok": True, "face_z": FACE,
            "wobj": {"uframe": ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)), "oframe": ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))},
            "start": rt(START), "contact": rt((START[0], START[1], FACE)), "hit": rt(hit),
            "after": rt(after if after is not None else (hit[0], hit[1], hit[2] - 0.95)),
            "sensor_after": sensor_after, "live_after": live_after, "pauses": pauses, "last_pause": last_pause}


class P2(unittest.TestCase):
    def test_a_retried_hit_that_stays_put_passes(self):
        self.assertEqual(failed(tpm.p2_verdict("S2 miss", p2_result(), 1, 3)), [])

    def test_a_primitive_that_returns_to_the_start_breaks_d12(self):
        rows = failed(tpm.p2_verdict("S2 miss", p2_result(after=START), 1, 3))
        self.assertEqual(rows, ["S2 miss NO return after the hit (D12)"])

    def test_the_sense_voltage_left_on_is_caught(self):
        rows = failed(tpm.p2_verdict("S2 miss", p2_result(sensor_after=1.0), 1, 3))
        self.assertEqual(rows, ["S2 miss sense voltage off after the search (D5)"])

    def test_the_wrong_pause_reason_is_caught(self):
        rows = failed(tpm.p2_verdict("S2 miss", p2_result(last_pause=4.0), 1, 3))
        self.assertEqual(rows, ["S2 miss paused 1 x for 'no contact'"])

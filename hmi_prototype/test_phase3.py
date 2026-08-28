"""Phase 3 tests: the file-transfer request actually delivers the module.

Run:  python -m unittest discover -s hmi_prototype -v

Reuses the FakeTgsRobot executable spec from test_phase2 (the wire protocol
is unchanged in Phase 3 - only what happens DURING request 10 differs:
the HMI copies <prog_name>.mod into the controller's HOME:/TGS/ folder,
standing in for the FTP upload).
"""

import os
import tempfile
import unittest

from test_phase2 import run_one_cycle


class TestModuleTransfer(unittest.TestCase):

    def test_module_copied_into_vc_home(self):
        with tempfile.TemporaryDirectory() as home:
            robot, hmi = run_one_cycle({"vc_home_dir": home})
            dst = os.path.join(home, "TGS", "TD05Test.mod")
            self.assertTrue(os.path.isfile(dst), "module not delivered")
            src = os.path.join(hmi.tgs_source_dir, "TD05Test.mod")
            with open(src, "rb") as f_src, open(dst, "rb") as f_dst:
                self.assertEqual(f_src.read(), f_dst.read())
            # transfer succeeded -> the whole program ran
            self.assertEqual(robot.received["ftp_status"], "1")
            self.assertEqual(hmi.request_log[-1], "100")

    def test_failed_transfer_reports_ftp_zero(self):
        """A missing source module -> ftp status 0 -> robot skips the program
        (FANUC 'Problem when copying/loading the robot program' path)."""
        with tempfile.TemporaryDirectory() as home:
            robot, hmi = run_one_cycle({"vc_home_dir": home,
                                        "prog_name": "NoSuchProg"})
            self.assertEqual(robot.received["ftp_status"], "0")
            self.assertEqual(hmi.request_log, ["10"])
            self.assertFalse(
                os.path.exists(os.path.join(home, "TGS", "NoSuchProg.mod")))

    def test_no_transfer_without_vc_home_dir(self):
        robot, hmi = run_one_cycle()
        self.assertIsNone(hmi.vc_home_dir)
        self.assertEqual(robot.received["ftp_status"], "1")
        self.assertEqual(hmi.request_log[-1], "100")


if __name__ == "__main__":
    unittest.main(verbosity=2)

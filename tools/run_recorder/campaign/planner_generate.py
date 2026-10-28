"""Drive the Weld Planner (a curobo_suite source tree) through Open -> Generate (with Plan Path)
-> Save on one .tgs, with no clicks: the dialogs are answered in code and their text is printed.

    python planner_generate.py <curobo_root> <project.tgs> <result.json>

Generate's own mode prompt answers "regenerate_all"; the playback after Plan Path is skipped.
TG_E2E_NAME_AFTER_PASSWORD=1 names the main program after the project password, as the site
laptop's planner config does. The process may crash on exit (Qt teardown) after the save; the
result file says whether the save happened.
"""
import json
import os
import sys
import time
import traceback

ROOT, PROJECT, RESULT = sys.argv[1], os.path.abspath(sys.argv[2]), sys.argv[3]
sys.path.insert(0, ROOT)
os.chdir(ROOT)
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")

from PySide2.QtCore import QTimer  # noqa: E402
from PySide2.QtWidgets import QApplication, QMessageBox  # noqa: E402

import tguide_lib.app_main as app_main  # noqa: E402

events = []


def note(kind, text):
    line = f"[{time.strftime('%H:%M:%S')}] {kind}: {text}"
    events.append(line)
    print(line, flush=True)


def _static(kind, answer):
    def box(parent, title, text, *args, **kwargs):
        note(kind, f"{title} | {text}")
        return answer
    return staticmethod(box)


QMessageBox.question = _static("question", QMessageBox.Yes)
QMessageBox.information = _static("information", QMessageBox.Ok)
QMessageBox.warning = _static("warning", QMessageBox.Ok)
QMessageBox.critical = _static("critical", QMessageBox.Ok)


def _exec(self):
    note("messagebox", f"{self.windowTitle()} | {self.text()} | {self.informativeText()}")
    return QMessageBox.Ok


QMessageBox.exec_ = _exec


def main():
    app_main.enable_high_dpi_scaling()
    app = QApplication(sys.argv)
    from tguide_lib import app_instance_lock
    lock = app_instance_lock.acquire()
    if lock.verdict is not app_instance_lock.Verdict.PROCEED:
        note("lock", lock.describe_for_log())
        return 3
    app_main._load_application_modules(app, None)
    from tguide_lib.workflows import weld_workflow_execution as wwe

    svc = wwe.WeldWorkflowExecutionService
    svc._prompt_generate_mode = lambda self: "regenerate_all"
    svc._prompt_plan_path_mode = lambda self: "all"
    svc.simulate_workflow_package = lambda self: True
    # A weld edited by the fixture (e.g. made coordinated) loses its teach-pendant points, as
    # the planner's own "Discard taught points and regenerate" button does.
    from tguide_lib import WeldWorkflowController as wwc

    def _discard(self, conflicted):
        note("question", "Taught Points on a Changed Seam | discard and regenerate: %s"
             % [getattr(w, "name", "") for w in conflicted])
        return True
    wwc.WeldWorkflowController._ask_discard_taught_points_for_seam_conflicts = _discard
    # The site laptop's planner config: the main program is named after the project password.
    if os.environ.get("TG_E2E_NAME_AFTER_PASSWORD") == "1":
        from tguide_lib.workflows import weld_workflow_package_generation as pkg
        _orig_value = pkg.weld_planner_config_value

        def _value(main_window, key, default=None):
            if key == "FANUC_REPLACE_ROBOT_PROGRAM_NAME_WITH_PROJECT_PASSWORD":
                return True
            return _orig_value(main_window, key, default)
        pkg.weld_planner_config_value = _value
        note("config", "main program named after the project password")

    def _report(self, report):
        note("report", f"{report.title} | {report.summary} | " + " || ".join(str(l) for l in report.lines))
    svc._show_report = _report

    window = app_main.MainWindow()
    window.show_startup_dialog = lambda *a, **k: None
    window.show()
    result = {"project": PROJECT, "opened": False, "generated": None, "saved": False}

    def finish():
        result["events"] = events
        with open(RESULT, "w", encoding="utf-8") as fh:
            json.dump(result, fh, indent=1)
        app.quit()

    def prepare():
        """TG_E2E_PREPARE = JSON list of edits made in the planner before Generate, as a user
        would: {"op": "auto_captures", "weld": "<name>"} runs the weld's Auto-add global
        captures (it reuses or adds the capture set at the weld's positioner angles);
        "next_stop" moves the weld to its next best positioner stop; "clear_capture_links"
        removes the weld's capture associations (before re-adding them for a changed weld)."""
        steps = json.loads(os.environ.get("TG_E2E_PREPARE", "[]") or "[]")
        controller = getattr(window, "weld_workflow_controller", None)
        for step in steps:
            project = controller.get_project()
            weld = next((w for w in getattr(project, "weld_items", []) if w.name == step.get("weld")), None)
            if step.get("op") == "generate":
                # A coordinated weld's capture angle is resolved by generation, so its captures
                # can only be auto-added after a first Generate.
                note("prepare", "first generate: %s" % bool(svc(window).generate_workflow_programs()))
            elif step.get("op") == "next_stop" and weld is not None:
                # The weld editor's "next best positioner stop" (another reachable stop, locked).
                moved = controller.move_weld_to_next_best_positioner_stop(weld.weld_id)
                weld = next(w for w in controller.get_project().weld_items if w.name == step["weld"])
                note("prepare", "next stop for %s: %s, now at %s" % (
                    weld.name, moved, (getattr(weld, "tool_path_parameters", {}) or {}).get("positioner_angles")))
            elif step.get("op") == "clear_capture_links" and weld is not None:
                ids = controller._associated_global_capture_pose_ids(weld)
                for pose_id in ids:
                    controller.remove_global_capture_pose_association_from_weld(weld.weld_id, pose_id)
                note("prepare", "capture links removed from %s: %s" % (weld.name, ids))
            elif step.get("op") == "auto_captures" and weld is not None:
                before = len([p for s in project.global_capture_sets for p in s.capture_poses])
                controller.auto_add_global_captures_for_weld(weld.weld_id, show_selection_progress=False,
                                                             show_status_message=False)
                project = controller.get_project()
                after = len([p for s in project.global_capture_sets for p in s.capture_poses])
                note("prepare", "auto captures for %s: %d -> %d capture pose(s), %d set(s)"
                     % (weld.name, before, after, len(project.global_capture_sets)))
            else:
                note("prepare", "step not applied: %s" % step)
        result["prepared"] = steps

    def step_generate():
        try:
            prepare()
            t0 = time.time()
            ok = bool(svc(window).generate_workflow_programs())
            result["generated"] = ok
            result["generate_s"] = round(time.time() - t0, 1)
            note("generate", f"returned {ok} after {result['generate_s']} s")
            if ok:
                window.project_session.save()
                result["saved"] = True
                note("save", "project saved")
        except Exception:
            note("exception", traceback.format_exc())
        QTimer.singleShot(3000, finish)

    def wait_loaded(started=[None]):
        if started[0] is None:
            started[0] = time.time()
        if app_main._project_is_loading(window) and time.time() - started[0] < 600:
            QTimer.singleShot(500, wait_loaded)
            return
        result["opened"] = True
        note("open", f"loaded after {time.time() - started[0]:.1f} s")
        QTimer.singleShot(2000, step_generate)

    def step_open():
        note("open", PROJECT)
        window.on_file_open_project(requested_path=PROJECT)
        QTimer.singleShot(1000, wait_loaded)

    QTimer.singleShot(3000, step_open)
    QTimer.singleShot(40 * 60 * 1000, lambda: (note("timeout", "40 min"), finish()))
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())

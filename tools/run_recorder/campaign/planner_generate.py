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

    # A project the HMI retrieved robot edits into opens with the planner's forced review
    # (ProjectSessionManager._show_teach_pendant_touchup_report). Answered "Use" for every weld
    # the scan lists as touched, through the planner's own adopt lane; the lines are printed.
    from tguide_lib import ProjectSessionManager as psm

    def _review_on_open(self, scan):
        for w in scan.welds:
            note("open-review", w.summary_line())
            for d in w.deltas:
                note("open-review", "  %s %s %.2f mm %.2f deg" % (d.target_id, d.classification, d.delta_mm, d.delta_deg))
        controller = getattr(self.main_window, "weld_workflow_controller", None)
        choices = {str(w.weld_id): {"adopt": True, "remove_touch_points": False} for w in scan.touched_welds}
        _project, adopted, failures = controller.adopt_teach_pendant_touchups(scan.artifact_path, choices)
        note("open-review", "adopted %s, failures %s" % (adopted, failures))
        return True
    psm.ProjectSessionManager._show_teach_pendant_touchup_report = _review_on_open

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

    def import_robot_program(controller, path):
        """Tools > Import Robot-Edited Program... on ``path``, the planner's own flow
        (WeldWorkflowController.import_robot_edited_program) with its dialogs answered as an
        operator would: go ahead, this file, the first main, compare anyway, and in the review
        "Use" every weld the scan lists as touched, keeping touch points. The scan's per-weld
        lines are printed as the review shows them."""
        from tguide_lib.workflows import weld_touchup_review_dialog as dlg
        ui = dlg.TeachPendantImportUi
        ui.confirm_heads_up = lambda self: True
        ui.confirm_program_name_mismatch = lambda self, text: (note("question", "program name: %s" % text), True)[1]
        ui.confirm_older_version = lambda self: (note("question", "older version suspected"), True)[1]
        ui.choose_file = lambda self: path
        ui.choose_main = lambda self, candidates: (note("question", "mains: %s" % candidates), candidates[0])[1]
        ui.show_message = lambda self, title, text, **kw: note("message", "%s | %s" % (title, text))

        def review(self, scan, file_name, name_note, has_touch_points, turn_fixes):
            for w in scan.welds:
                note("review", w.summary_line())
                for d in w.deltas:
                    note("review", "  %s %s %.2f mm %.2f deg" % (d.target_id, d.classification, d.delta_mm, d.delta_deg))
            return {str(w.weld_id): {"adopt": True, "remove_touch_points": False} for w in scan.touched_welds}
        ui.review = review
        return controller.import_robot_edited_program()

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
            elif step.get("op") == "bind_preset" and weld is not None:
                # The weld editor's library combo (_commit_library_selection): a snapshot of the
                # planner's own weld-library row, "Set manually" for custom values.
                from tguide_lib.workflows.weld_library_qtsql import (
                    weld_library_db_path, weld_library_row_by_id, weld_library_rows, weld_library_snapshot_from_row)
                note("prepare", "weld library %s: %d row(s)" % (weld_library_db_path(window), len(weld_library_rows(window))))
                row = weld_library_row_by_id(window, step["library_id"])
                snapshot = weld_library_snapshot_from_row(row)
                current = dict((getattr(weld, "payload", {}) or {}).get("weld_library", {}) or {})
                manual = dict(current.get("manual_parameters", {}) or {})
                manual.update(step.get("manual", {}))
                binding = {"binding_mode": "manual" if step.get("custom") else "snapshot",
                           "library_id": int(step["library_id"]),
                           "display_name": str(snapshot.get("name", "") or ""),
                           "parameters": snapshot, "manual_parameters": manual}
                controller.update_weld_library_binding(weld.weld_id, binding)
                note("prepare", "preset for %s: library row %s -> %s" % (weld.name, step["library_id"], snapshot))
            elif step.get("op") == "import_robot_program":
                note("prepare", "import robot-edited program: %s" % import_robot_program(controller, step["file"]))
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

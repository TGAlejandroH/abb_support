"""TetraGen HMI prototype for ABB - serves robot-initiated TGS requests over TCP.

The robot (RAPID ``TG_Main``/``TG_Comms`` in ``abb/rapid/``) is the TCP *server*;
this program is the TCP *client* that connects to it - mirroring how
``FANUCRobot`` in TGuideWeldingHMI connects to the FANUC controller. At the
application level, however, THIS side serves the requests the robot initiates.

Wire protocol (docs/abb_port_plan_v1.md sections 1.4 and 4.5):
  * every robot->HMI message is answered with a 1-byte ack b"0"
  * every HMI->robot value is pulled by a prompt string sent by the robot
  * scalars keep the FANUC fixed-width format (e.g. "+0905.216")
  * frames/poses travel as RAPID pose literals "[[x,y,z],[q1,q2,q3,q4]]"
    with a NORMALIZED quaternion (q1=w, q2=x, q3=y, q4=z). Conversion to and
    from the FANUC x,y,z,w,p,r Euler convention (w=Rx, p=Ry, r=Rz, fixed
    axes, R = Rz*Ry*Rx) happens HERE, on the PC side - the future C++
    ABBRobot class will carry the same codec.

Phase 2 scope: program selection + all priority requests
(1 R_C_F, 2 R_C, 4 R_W_F, 5 R_P_C, 10 R_F_T, 11 R_G_C_D, 14 R_W_P, 100 R_E).
Dummy, configurable answers everywhere; no real HMI/camera logic.

Usage:
    python abb_server.py [host] [port] [cycles] [transfer]
    defaults: 127.0.0.1 2000 2 (no module transfer)

    transfer: how request 10 delivers abb/rapid/TGS/<prog_name>.mod to the
    controller. Two mechanisms (Phase 5 decision - BOTH are kept, the copy
    fallback must not be removed):
      * a directory path = the virtual controller's HOME folder (e.g.
        "<solution>\\Virtual Controllers\\Controller1\\HOME"): direct file
        copy into <dir>/TGS/ - the original prototype mechanism.
      * an http(s) URL = the controller's RWS base (e.g.
        "http://127.0.0.1:80"): upload via Robot Web Services
        PUT /fileservice/$home/TGS/<prog_name>.mod - what the real HMI
        will do (docs/abb_program_touchup_and_retrieval_v1.md section 4).
    Retrieval of operator-edited programs is the separate tg_retrieve.py.
"""

import math
import os
import shutil
import socket
import sys
import time

from rws_client import RwsClient, RwsError

ACK = b"0"
RECV_MAX = 1024  # same buffer size the C++ HMI uses


class ConnectionClosedError(Exception):
    """The robot closed the connection (normal at the end of a cycle)."""


# ---------------------------------------------------------------------------
# Pose codec: FANUC-convention Euler <-> ABB normalized quaternion
# ---------------------------------------------------------------------------

def euler_wpr_to_quat(w_deg, p_deg, r_deg):
    """FANUC W,P,R (deg; rotations about fixed X, Y, Z; R = Rz*Ry*Rx)
    -> ABB quaternion (q1, q2, q3, q4) = (w, x, y, z), normalized."""
    rx = math.radians(w_deg)
    ry = math.radians(p_deg)
    rz = math.radians(r_deg)
    cx, sx = math.cos(rx / 2), math.sin(rx / 2)
    cy, sy = math.cos(ry / 2), math.sin(ry / 2)
    cz, sz = math.cos(rz / 2), math.sin(rz / 2)
    q = (
        cx * cy * cz + sx * sy * sz,  # q1 = w
        sx * cy * cz - cx * sy * sz,  # q2 = x
        cx * sy * cz + sx * cy * sz,  # q3 = y
        cx * cy * sz - sx * sy * cz,  # q4 = z
    )
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def quat_to_euler_wpr(q1, q2, q3, q4):
    """ABB quaternion (w, x, y, z) -> FANUC W,P,R in degrees (see above).

    The generic ZYX formulas carry cos(P) as a common factor in BOTH atan2 terms
    of W (R32 = cos(P) sin(W), R33 = cos(P) cos(W)) and of R (R21, R11) - it
    cancels inside atan2, so they are exact for every cos(P) != 0.  At
    cos(P) == 0 exactly they degenerate to atan2(0, 0) and return a triple whose
    W - R (at P = +90) or W + R (at P = -90) is zero whatever the input was.
    Only that combination is determined at the pole, so the result is a
    DIFFERENT ROTATION, not one of several valid splits: measured against truth,
    (W, P, R) = (13.932, 90, 109.244) came back rotated by 95.312 deg, silently.
    Recorded as L3 in TG_RoboCal/docs/lessons_learned.md, and aimed at exactly
    the poses an overhead capture uses (P = +/-90 is tool X vertical).

    The pole branch states the convention the degeneracy leaves free - W := 0,
    the whole determined combination reported as R - which IS rotation
    preserving.  Kept numerically identical to
    tg_robocal.robots.abb.quat_to_euler_zyx and to the HMI's
    AbbPoseCodec::FanucWprFromQuat (same 1e-12 threshold, same wrap) so the
    repos carrying this function cannot drift.  The wrap is needed because
    nothing here canonicalizes the incoming quaternion sign, and -q shifts
    atan2(x, w) by pi, which would report R as -330 deg where +q reports 30:
    the same rotation, but outside the range the generic branch returns.

    Away from the pole the arithmetic is untouched, so ordinary poses are
    bit-identical to the previous version.
    """
    n = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
    w, x, y, z = q1 / n, q2 / n, q3 / n, q4 / n
    sp = 2.0 * (w * y - z * x)
    sp = max(-1.0, min(1.0, sp))  # clamp against rounding at the gimbal poles
    if abs(sp) >= 1.0 - 1e-12:
        ry = math.copysign(math.pi / 2.0, sp)
        rz = (-2.0 if sp > 0.0 else 2.0) * math.atan2(x, w)
        rx = 0.0
        # atan2 spans (-pi, pi], so rz spans [-2pi, 2pi) before this; one pass
        # is enough to land it in (-pi, pi].
        if rz > math.pi:
            rz -= 2.0 * math.pi
        elif rz <= -math.pi:
            rz += 2.0 * math.pi
    else:
        rx = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        ry = math.asin(sp)
        rz = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(rx), math.degrees(ry), math.degrees(rz)


def xyzwpr_to_pose_literal(frame):
    """[x, y, z, w, p, r] (mm/deg, FANUC convention) -> RAPID pose literal.

    Format matches TG_Comms.tgPoseToStr: translations 2 decimals, quaternions
    6 decimals -> always < 80 chars (RAPID string limit) for |xyz| < 10000.
    """
    x, y, z, w, p, r = (float(v) for v in frame)
    if any(abs(v) >= 10000.0 for v in (x, y, z)):
        raise ValueError("frame translation out of range (|v| must be < 10000 mm)")
    q = euler_wpr_to_quat(w, p, r)
    literal = "[[{:.2f},{:.2f},{:.2f}],[{:.6f},{:.6f},{:.6f},{:.6f}]]".format(
        x, y, z, *q)
    assert len(literal) <= 80, "pose literal exceeds the RAPID string limit"
    return literal


def pose_literal_to_xyzwpr(literal):
    """RAPID pose literal "[[x,y,z],[q1,q2,q3,q4]]" -> [x, y, z, w, p, r]."""
    values = [float(tok) for tok in
              literal.replace("[", " ").replace("]", " ").replace(",", " ").split()]
    if len(values) != 7:
        raise ValueError("expected 7 values in pose literal, got %r" % literal)
    x, y, z, q1, q2, q3, q4 = values
    w, p, r = quat_to_euler_wpr(q1, q2, q3, q4)
    return [x, y, z, w, p, r]


def fmt_real(value):
    """FANUC fixed-width scalar: sign + 8 chars, 3 decimals ("+0905.216")."""
    return f"{float(value):+09.3f}"


#: The nine seam-phase values, in the order the wire carries them. Matches
#: `WELD_PROCESS_PARAMETER_KEYS` in the planner (weld_library_qtsql.py) and the
#: fan-out in `TG_ReqWeldParams`; all three must agree or the values land in
#: the wrong components silently. Times in SECONDS, wire feeds in IPM, volts
#: absolute -- the same units the rest of this wire uses.
SEAM_PHASE_ORDER = (
    "purge_time_s",
    "preflow_time_s",
    "postflow_time_s",
    "burnback_time_s",
    "craterfill_time_s",
    "craterfill_wire_feed_speed",
    "craterfill_volts",
    "ignition_wire_feed_speed",
    "ignition_volts",
)

#: "no seam phases" -- what a preset that never authored them serves.
DEFAULT_SEAM_PHASES = (0.0,) * len(SEAM_PHASE_ORDER)


#: RAPID's hard limit on a `string`, and so on any one `SocketReceive ... \\Str` message.
#: See abb_port_plan_v1.md ("RAPID string max length is 80 chars").
RAPID_STRING_MAX = 80

#: Seam phases are clamped tighter than the standalone scalars (fmt_real's +/-9999.999)
#: so that nine of them plus separators cannot exceed RAPID_STRING_MAX: with two decimals
#: the worst case is "-999.99" (7) x 9 + 8 commas + 2 brackets = 73. Two decimals costs
#: these fields nothing -- the times are hundredths of a second (MONARCH's own seamdata
#: uses 0.2, 0.25, 0.08), the wire feeds are whole IPM (its largest is 800), and the
#: volts are a correction or an absolute to 0.01 V.
SEAM_PHASE_MAX = 999.99


def fmt_seam_phase_value(value):
    """One seam phase as a RAPID num LITERAL -- NOT the fixed-width scalar `fmt_real`.

    No leading '+'. `fmt_real`'s "+0000.500" is right for a standalone reply, which
    TG_Comms reads with `tgParseReal` -- a helper that exists precisely to strip the
    FANUC plus sign, because "an explicit plus sign is not part of a RAPID num literal,
    so StrToVal may reject it". This payload reaches `StrToVal` DIRECTLY, with no such
    stripping, so the sign must never be emitted in the first place.
    """
    clamped = max(-SEAM_PHASE_MAX, min(SEAM_PHASE_MAX, float(value)))
    return f"{clamped:.2f}"


def fmt_seam_phases(values):
    """The nine seam phases as ONE bracketed message RAPID `StrToVal` parses
    straight into a `num{9}` -- the batching "Give me the frame" established.

    MUST stay byte-identical to `WeldParameterWire::FormatSeamPhases` in the production
    HMI. The two diverged once (2026-09-20): this side emitted 59 characters while the
    C++ emitted 91 with FANUC plus signs. Every green test here was therefore validating
    a payload the real HMI never sends, and the one it did send was over RAPID's 80-char
    string limit -- the divergence is what hid the defect.

    Raises on the wrong count rather than padding: a short list would silently
    shift every later value into the wrong seamdata component.
    """
    row = [float(v) for v in values]
    if len(row) != len(SEAM_PHASE_ORDER):
        raise ValueError(
            f"seam phases must have {len(SEAM_PHASE_ORDER)} values "
            f"({', '.join(SEAM_PHASE_ORDER)}), got {len(row)}"
        )
    payload = "[" + ",".join(fmt_seam_phase_value(v) for v in row) + "]"
    # By construction, but asserted: an oversize message does NOT fail cleanly. RAPID's
    # SocketReceive truncates at 80 and the remainder stays in the TCP buffer, becoming
    # the reply to the NEXT prompt and desyncing the rest of the session.
    assert len(payload) <= RAPID_STRING_MAX, (
        f"seam phase payload is {len(payload)} chars, over RAPID's "
        f"{RAPID_STRING_MAX}-char string limit: {payload}"
    )
    return payload


# ---------------------------------------------------------------------------
# The HMI prototype
# ---------------------------------------------------------------------------

# A realistically broken frame payload: truncated mid-number, the way a cut
# TCP stream would look. RAPID StrToVal and pose_literal_to_xyzwpr both
# reject it.
CORRUPT_FRAME_PAYLOAD = "[[850.00,-120.00,4"


# --- weld-demo script (see main(), mode "weld-demo") ----------------------
#
# Values are the HMI's own native-.tgs seed defaults (WeldLibrary.cpp):
# proc 1, wire feed 520 IPM, travel 21 IPM, arc length 49.0, arc control 0.0
# - except arc length, which is deliberately left at 49.0 to prove
# TG_ApplyWeldParams CLAMPS it (Fronius corrections are about +/-10 steps).
#
# Weld 1 = user-defined (UDWP 1), weld 2 = predefined (UDWP 0), so a single
# run covers both branches of TG_ApplyWeldParams.
WELD_DEMO_SEQUENCE = [
    {
        "udwp_flag": 1,
        "welder_type": 2,        # FRONIUSTPSi, matching the cell's config.json
        "weld_proc": 1,
        "travel_speed": 21.0,    # IPM -> expect weld_speed 8.890 mm/s in RAPID
        "wire_feed_speed": 520.0,  # IPM -> expect wirefeed 220.133 mm/s
        "arc_length": 49.0,      # out of range -> expect CLAMP to 10
        "arc_control": 0.0,      # HMI hides this field and always sends 0.0
        "weld_sched": 4,         # MONARCH uses 1, 2 and 4
        # Seam phases in SEAM_PHASE_ORDER. Shaped after MONARCH's own
        # sm3_16_ft_tack: a short purge/preflow, a real crater fill.
        "seam_phases": [0.5, 0.2, 0.5, 0.08, 0.25, 350.0, 3.0, 300.0, 2.0],
    },
    {
        # The weld that used to break: bound to a PRESET, not to operator
        # overrides. Under the pre-WS3 wire it received travel speed and
        # nothing else and welded at weld_speed 0. It must now be served the
        # full set exactly like the weld above -- different values, same
        # completeness -- which is the whole point of the change.
        "udwp_flag": 0,
        "welder_type": 2,
        "weld_proc": 2,
        "travel_speed": 30.0,    # IPM -> expect weld_speed 12.700 mm/s
        "wire_feed_speed": 400.0,  # IPM -> expect wirefeed 169.333 mm/s
        "arc_length": 3.0,       # in range -> NOT clamped
        "arc_control": 0.0,
        "weld_sched": 1,
        "seam_phases": list(DEFAULT_SEAM_PHASES),  # a preset with none authored
    },
]

# The mm/s values the RAPID side should report for the sequence above, so the
# expectation lives next to the input rather than only in the docs.
IPM_TO_MM_S = 25.4 / 60.0
WELD_DEMO_EXPECTED_MM_S = [
    {"weld_speed": 21.0 * IPM_TO_MM_S, "wirefeed": 520.0 * IPM_TO_MM_S,
     "arc_length_clamped": 10.0, "arc_control": 0.0, "sched": 4,
     "fill_time": 0.25, "fill_wirefeed": 350.0 * IPM_TO_MM_S,
     "preflow_time": 0.2, "postflow_time": 0.5},
    # The preset-bound weld: a real speed and a real wire feed, where the
    # pre-WS3 wire gave it weld_speed 0 and left the previous weld's crater
    # fill standing in the PERS seamdata.
    {"weld_speed": 30.0 * IPM_TO_MM_S, "wirefeed": 400.0 * IPM_TO_MM_S,
     "arc_length_clamped": 3.0, "arc_control": 0.0, "sched": 1,
     "fill_time": 0.0, "fill_wirefeed": 0.0,
     "preflow_time": 0.0, "postflow_time": 0.0},
]


class AbbTgsHmi:
    """Application-level request server / transport-level TCP client."""

    def __init__(self, host="127.0.0.1", port=2000, program_selection=1,
                 verbose=True, vc_home_dir=None, rws=None, handshake_port=2001):
        self.host = host
        self.port = port
        # Socket-start handshake port (socket plan S14, 2026-09-24): every cycle begins
        # with a short connection here (START, STNFRAME, STNAXES, verdict) before the
        # robot listens on `port` for the run.
        self.handshake_port = handshake_port
        self.last_start = None
        self.program_selection = program_selection
        self.verbose = verbose
        self.sock = None

        # Module transfer: where the .tgs module sources are in this repo,
        # and one of two delivery mechanisms (Phase 5: both kept).
        #   vc_home_dir - the controller's HOME: on disk (VC only): copy.
        #   rws         - an RwsClient (or base-URL string): RWS upload.
        self.vc_home_dir = vc_home_dir
        if isinstance(rws, str):
            rws = RwsClient(rws)
        self.rws = rws
        self.tgs_source_dir = os.path.normpath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "abb", "rapid", "TGS"))

        # ---- canned answers (dummy values; tweak to exercise robot branches)
        self.ftp_status = 1                # R_F_T: 1 = transfer succeeded
        self.prog_name = "TD05Test"        # R_F_T: .tgs program name (<= 10 chars,
                                           #        doubles as project password)
        self.pass_ok = 1                   # R_P_C
        self.dry_run = 0                   # R_P_C
        self.cam_frame_xyzwpr = [850.0, -120.0, 400.0, 5.0, -10.0, 45.0]
        self.do_capture = 1                # R_C_F: 1 = perform the capture
        # Fault injection (error matrix I4 checks): send a malformed frame
        # payload instead of the pose literal. The robot must force the
        # skip (cam) / abort (weld) path and keep the choreography intact.
        self.corrupt_cam_frame = False
        self.corrupt_weld_frame = False
        self.capture_ok = 1                # R_C: 1 = capture succeeded
        self.global_ok = 1                 # R_G_C_D
        self.weld_frame_xyzwpr = [900.0, 80.0, 350.0, -2.5, 3.5, 90.0]
        self.weld_status = 1               # R_W_F: 0=skip, 1=weld, 2=abort
        # R_W_F touch-up push (Phase 7, FANUC r_w_f.kl 2026-09-04): the HMI's
        # stored per-weld touch-up offset X/Y/Z in INCHES, sent on EVERY
        # weld-frame reply - weld/skip/abort alike. Unconditional for ABB
        # (ROBOT_PUSH_TOUCHUP_OFFSETS_TO_PENDANT assumed True from day one).
        # Non-zero defaults so a VC run proves the values land in
        # posTG_Touchup rather than matching its [0,0,0] initializer.
        self.touchup_offsets_in = [0.045, -0.12, 0.005]
        self.udwp_flag = 1                 # R_W_P: 1 = operator overrides, 0 = preset
        self.travel_speed = 17.5           # R_W_P
        self.welder_type = 1               # R_W_P: 1=Miller, 2=FroniusTPSi
        self.weld_proc = 5                 # R_W_P
        self.wire_feed_speed = 250.0       # R_W_P
        self.arc_length = 2.5              # R_W_P
        self.arc_control = 0.0             # R_W_P
        # WS3/WS4: the power source's program / characteristic number
        # (-> welddata main_arc.sched) and the seam phases, both served on
        # every weld. See handle_weld_params_req.
        self.weld_sched = 0                # R_W_P
        self.seam_phases = list(DEFAULT_SEAM_PHASES)  # R_W_P, 9 values

        # Optional per-call script for R_W_P, so ONE run can exercise both
        # branches of TG_ApplyWeldParams (user-defined, then predefined).
        # Each entry is a dict of any of the R_W_P fields above and is
        # applied before that call is served. None = the single fixed set
        # above, which is what phases 1-3 and their tests rely on.
        self.weld_param_sequence = None
        self._weld_param_calls = 0

        # ---- last-received data, for tests / future HMI logic
        self.last_pose_xyzwpr = None
        self.last_sub_name = None
        self.last_password = None
        self.last_free_bytes = None
        self.request_log = []
        # R_W_S: raw (mm, s, flag) of the last stats message, and the rows the
        # real HMI would have written to the analytics DB this cycle - see
        # handle_weld_stats_req.
        self.last_weld_stats = None
        self.weld_stats_entries = []

        self.handlers = {
            "1": self.handle_cam_frame_req,             # R_C_F
            "2": self.handle_capture_req,               # R_C
            "4": self.handle_weld_frame_req,            # R_W_F
            "5": self.handle_pass_check_req,            # R_P_C
            "10": self.handle_file_transfer_req,        # R_F_T
            "11": self.handle_global_captures_done_req, # R_G_C_D
            "13": self.handle_weld_stats_req,           # R_W_S
            "14": self.handle_weld_params_req,          # R_W_P
            "100": self.handle_end_req,                 # R_E
        }

    # -- transport primitives (mirror FANUCRobot::do_receive / do_send) -----

    def connect(self, retry_seconds=30.0):
        """Connect to the robot, retrying while it is (re)binding its port."""
        deadline = time.monotonic() + retry_seconds
        while True:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((self.host, self.port))
                self._log(f"connected to robot at {self.host}:{self.port}")
                return
            except OSError:
                self.sock.close()
                self.sock = None
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.5)

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None
            self._log("connection closed")

    # -- socket-start handshake (socket plan S14-S17, 2026-09-24) ------------

    HANDSHAKE_TIMEOUT = 30.0

    def handshake_connect(self, retry_seconds=30.0):
        """Connect to the handshake port, retrying while the robot is not listening."""
        deadline = time.monotonic() + retry_seconds
        while True:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                s.settimeout(self.HANDSHAKE_TIMEOUT)
                s.connect((self.host, self.handshake_port))
                self.sock = s
                self._log(f"handshake: connected to {self.host}:{self.handshake_port}")
                return
            except OSError:
                s.close()
                self.sock = None
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.5)

    def decide_start(self, event):
        """Verdict for this part: 1 = run, anything else = refuse. The base client always
        runs; SocketStartRun maps the part id and checks the station (plan S15)."""
        return 1

    def handshake(self):
        """One handshake connection: START, STNFRAME + pose, STNAXES, then the verdict on
        the robot's prompt, then wait for the robot to close (listener first, then the
        client - plan S14) before anything reconnects. Returns the StartEvent dict."""
        self.handshake_connect()
        event = {"verdict": 0}
        try:
            start = self.do_receive()
            toks = start.split()
            if not toks or toks[0] != "START":
                raise RuntimeError(f"handshake: expected START, got {start!r}")
            event["seq"] = int(toks[1])
            event["part"] = toks[2] if len(toks) > 2 else "0"
            event["station"] = int(toks[3]) if len(toks) > 3 and toks[3].lstrip("-").isdigit() else 0
            header = self.do_receive().strip()
            if header != "STNFRAME":
                raise RuntimeError(f"handshake: expected STNFRAME, got {header!r}")
            event["station_frame"] = self.do_receive().strip()
            event["station_frame_xyzwpr"] = pose_literal_to_xyzwpr(event["station_frame"])
            axes = self.do_receive().split()
            if not axes or axes[0] != "STNAXES":
                raise RuntimeError(f"handshake: expected STNAXES, got {axes!r}")
            event["tilt_deg"] = float(axes[1]) if len(axes) > 1 else None
            event["chuck_deg"] = float(axes[2]) if len(axes) > 2 else None
            event["verdict"] = int(self.decide_start(event))
            prompt = self.do_send(str(event["verdict"]))
            if "verdict" not in prompt:
                self._log(f"WARNING: unexpected verdict prompt: {prompt!r}")
            # The robot closes its listener, then this client; read until end-of-file so
            # a reconnect (to any port) never races the close.
            try:
                trailing = self.sock.recv(RECV_MAX)
                if trailing:
                    self._log(f"handshake: unexpected trailing data {trailing!r}")
                else:
                    self._log("handshake: robot closed the connection")
            except socket.timeout:
                self._log("handshake: robot did not close within the timeout")
            return event
        finally:
            self.close()

    def _recv(self):
        data = self.sock.recv(RECV_MAX)
        if not data:
            raise ConnectionClosedError
        return data.decode("utf-8")

    def do_receive(self):
        """Robot->HMI message: receive payload, answer the 1-byte ack."""
        data = self._recv()
        self._log(f"  robot -> {data!r}")
        self.sock.sendall(ACK)
        return data

    def do_send(self, payload):
        """HMI->robot value: wait for the robot's prompt, send the payload."""
        prompt = self._recv()
        self._log(f"  robot prompts {prompt!r}")
        self.sock.sendall(str(payload).encode("utf-8"))
        self._log(f"  hmi   -> {payload!r}")
        return prompt

    # -- request handlers ----------------------------------------------------

    def serve_program_selection(self):
        """First exchange after connecting (FANUC REQ_PROG_SEL)."""
        prompt = self.do_send(str(self.program_selection))
        if "program ID" not in prompt:
            self._log(f"WARNING: unexpected program-selection prompt: {prompt!r}")

    def _recv_pose_and_sub(self):
        """Common prefix of the frame-ish requests: current pose + sub token."""
        self.last_pose_xyzwpr = pose_literal_to_xyzwpr(self.do_receive())
        self.last_sub_name = self.do_receive()
        self._log(f"  pose(xyzwpr)={['%.3f' % v for v in self.last_pose_xyzwpr]} "
                  f"sub={self.last_sub_name!r}")

    def handle_end_req(self):
        """FANUC R_E (id 100): receive current pose + sub-routine token."""
        self._recv_pose_and_sub()

    def handle_cam_frame_req(self):
        """FANUC R_C_F (id 1): pose + sub in; camera frame + capture flag out.

        A real HMI would run registration here and send the resulting frame;
        the prototype sends the canned ``cam_frame_xyzwpr``.
        """
        self._recv_pose_and_sub()
        if self.corrupt_cam_frame:
            self.do_send(CORRUPT_FRAME_PAYLOAD)
        else:
            self.do_send(xyzwpr_to_pose_literal(self.cam_frame_xyzwpr))
        self.do_send(str(self.do_capture))

    def handle_capture_req(self):
        """FANUC R_C (id 2): pose + sub in; capture-success flag out."""
        self._recv_pose_and_sub()
        self.do_send(str(self.capture_ok))

    def handle_weld_frame_req(self):
        """FANUC R_W_F (id 4): pose + sub in; weld frame + weld status +
        touch-up offset (x/y/z, inches) out.

        The touch-up push always completes, corrupt-frame injection included:
        the RAPID side receives the full sequence on every reply mode.
        """
        self._recv_pose_and_sub()
        if self.corrupt_weld_frame:
            self.do_send(CORRUPT_FRAME_PAYLOAD)
        else:
            self.do_send(xyzwpr_to_pose_literal(self.weld_frame_xyzwpr))
        self.do_send(str(self.weld_status))
        for value in self.touchup_offsets_in:
            self.do_send(fmt_real(value))

    def handle_pass_check_req(self):
        """FANUC R_P_C (id 5): pose + sub + password in; 2-char status out
        (char 1 = password correct, char 2 = dry run)."""
        self._recv_pose_and_sub()
        self.last_password = self.do_receive()
        self.do_send(f"{self.pass_ok:d}{self.dry_run:d}")

    def handle_file_transfer_req(self):
        """FANUC R_F_T (id 10): free memory in; transfer status + name out.

        This is the point where the real HMI uploads the .tgs module to the
        controller (FTP). The prototype copies the module file into the
        virtual controller's HOME:/TGS/ folder when ``vc_home_dir`` is set;
        a failed copy is reported to the robot as ftp status 0, which makes
        TG_Main skip the program - same as the FANUC error path.
        """
        self.last_free_bytes = int(self.do_receive())
        status = self.ftp_status
        if status == 1 and (self.vc_home_dir or self.rws):
            try:
                self._transfer_tgs_module()
            except (OSError, RwsError) as exc:
                self._log(f"ERROR: module transfer failed: {exc}")
                status = 0
        self.do_send(str(status))
        self.do_send(self.prog_name)  # on FANUC this is the project password
                                      # (== program name); <= 10 chars

    def _transfer_tgs_module(self):
        """Deliver abb/rapid/TGS/<prog>.mod to the controller's HOME:/TGS/.

        Two mechanisms, per the Phase 5 decision (touch-up doc section 9):
        RWS upload when an RwsClient is configured, else the original
        direct-copy fallback into the VC's HOME folder - kept on purpose.
        """
        src = os.path.join(self.tgs_source_dir, f"{self.prog_name}.mod")
        if self.rws is not None:
            with open(src, "rb") as f:
                data = f.read()
            dst = f"$home/TGS/{self.prog_name}.mod"
            self.rws.put_file(dst, data)
            self._log(f"  transferred {src} -> RWS {dst}")
            return
        dst_dir = os.path.join(self.vc_home_dir, "TGS")
        os.makedirs(dst_dir, exist_ok=True)
        dst = os.path.join(dst_dir, f"{self.prog_name}.mod")
        shutil.copyfile(src, dst)
        self._log(f"  transferred {src} -> {dst}")

    def handle_global_captures_done_req(self):
        """FANUC R_G_C_D (id 11): global localization status out."""
        self.do_send(str(self.global_ok))

    def handle_weld_stats_req(self):
        """FANUC R_W_S (id 13): one CSV message in - no pose, no sub token.

        Payload is "dist_mm,arc_on_s,succ_ae". Mirrors the real HMI
        (FANUCRobot::ReceiveWeldingStats + RobotCell.cpp case
        RequestWeldingStats): parse three reals, convert the distance to
        inches, and record an analytics row ONLY when succ_ae == 1 - that
        flag is what tells the HMI the arc actually ran, so a dry run (or a
        weld that never ignited) is received and then deliberately dropped.

        `weld_stats_entries` stands in for AnalyticsManager::InsertWeldEntry,
        whose signature is (double length_in, double arc_on_sec) - inches and
        seconds, hence the /25.4 here and not on the robot side.

        The C++ parse is std::stringstream >> double, which tolerates the
        blank padding FANUC's CNV_REAL_STR emits AND the leading "+" of the
        RAPID tgFmtReal form; float() tolerates both the same way, so this
        handler is valid against either robot brand.
        """
        payload = self.do_receive()
        fields = payload.split(",")
        if len(fields) != 3:
            raise ValueError(
                f"R_W_S needs 3 comma-separated reals, got {payload!r}")
        dist_mm, arc_on_sec, succ_ae = (float(f) for f in fields)
        self.last_weld_stats = (dist_mm, arc_on_sec, succ_ae)
        length_in = dist_mm / 25.4
        if succ_ae == 1.0:
            self.weld_stats_entries.append((length_in, arc_on_sec))
            recorded = "recorded"
        else:
            recorded = "NOT recorded (succ_ae != 1)"
        self._log(f"  weld stats: {dist_mm:.3f} mm ({length_in:.3f} in), "
                  f"arc on {arc_on_sec:.3f} s, succ_ae={succ_ae:g} -> "
                  f"{recorded}")

    def handle_weld_params_req(self):
        """R_W_P (id 14): every weld parameter, on every weld.

        ⚠ **This is where the ABB wire departs from FANUC's (WS3/WS4,
        2026-09-20).** It used to send welder type, proc and the schedule
        values only `if self.udwp_flag == 1`, because on FANUC the values
        live in an ArcTool schedule file on the controller and the HMI only
        had to speak up when the operator overrode them.

        ABB has no such file - welddata/seamdata are ordinary RAPID data - so
        whatever is not sent, nothing supplies. Under the old shape a weld
        bound to a preset got travel speed and nothing else, and welded at
        weld_speed 0. Now the flag is provenance only: it tells the robot
        (and the pendant log) whether these numbers came from the operator's
        overrides or from the weld's preset, and the values arrive either way.

        Two additions beyond the old set: `weld_sched` (the power source's
        program / characteristic number, -> main_arc.sched) and the nine
        `seam_phases`, which go out as ONE bracketed message rather than nine
        round trips - the same batching "Give me the frame" already uses.

        `weld_param_sequence`: when set, entry N is applied before the Nth
        call of this cycle, which lets a two-weld program be served with
        different parameters per weld.
        """
        if self.weld_param_sequence:
            idx = min(self._weld_param_calls, len(self.weld_param_sequence) - 1)
            for key, value in self.weld_param_sequence[idx].items():
                if not hasattr(self, key):
                    raise AttributeError(
                        f"weld_param_sequence[{idx}] has unknown field {key!r}")
                setattr(self, key, value)
        self._weld_param_calls += 1

        self.do_send(str(self.udwp_flag))
        self.do_send(fmt_real(self.travel_speed))
        self.do_send(f"{self.welder_type:02d}")
        self.do_send(f"{self.weld_proc:02d}")
        self.do_send(fmt_real(self.wire_feed_speed))
        self.do_send(fmt_real(self.arc_length))
        self.do_send(fmt_real(self.arc_control))
        self.do_send(f"{int(self.weld_sched):02d}")
        self.do_send(fmt_seam_phases(self.seam_phases))

    # -- main loop -----------------------------------------------------------

    def serve_cycle(self):
        """One robot cycle: the start handshake, then connect, program selection, serve
        requests until the robot disconnects (which it does right after request 100)."""
        self.request_log = []
        self.last_start = self.handshake()
        if self.last_start["verdict"] != 1:
            self._log("handshake: refused - the robot ends the part, no run this cycle")
            return
        self.connect()
        self._weld_param_calls = 0
        # One analytics "session" per cycle, like the real HMI (it opens a
        # weld session per run and inserts a row per successful weld).
        self.weld_stats_entries = []
        try:
            self.serve_program_selection()
            while True:
                try:
                    req_id = self.do_receive().strip()
                except ConnectionClosedError:
                    self._log("robot disconnected (end of cycle)")
                    return
                handler = self.handlers.get(req_id)
                if handler is None:
                    raise RuntimeError(f"no handler for request id {req_id!r}")
                self._log(f"serving request {req_id}")
                self.request_log.append(req_id)
                handler()
        finally:
            self.close()

    def _log(self, msg):
        if self.verbose:
            print(msg, flush=True)


def main(argv):
    host = argv[1] if len(argv) > 1 else "127.0.0.1"
    port = int(argv[2]) if len(argv) > 2 else 2000
    cycles = int(argv[3]) if len(argv) > 3 else 2
    transfer = argv[4] if len(argv) > 4 else None
    mode = argv[5] if len(argv) > 5 else None
    # An http(s) URL selects the RWS transfer; anything else is the VC HOME
    # folder for the kept copy fallback (see module docstring).
    vc_home_dir = None
    rws = None
    if transfer and transfer.lower().startswith(("http://", "https://")):
        rws = transfer
    else:
        vc_home_dir = transfer
    hmi = AbbTgsHmi(host=host, port=port, vc_home_dir=vc_home_dir, rws=rws)
    if mode == "corrupt-cam":
        hmi.corrupt_cam_frame = True
    elif mode == "corrupt-weld":
        hmi.corrupt_weld_frame = True
    elif mode == "weld-demo":
        # Serve the two-weld arc program instead of the comms regression
        # program, with DIFFERENT parameters per weld so one run exercises
        # both branches of TG_ApplyWeldParams.
        hmi.prog_name = "TD05Weld"
        hmi.weld_param_sequence = WELD_DEMO_SEQUENCE
    elif mode == "dry-run":
        # Welding inhibited. The robot still serves every request, R_W_S
        # included, but reports succ_ae = 0 (nTG_SuccArcEnd := 1-nTG_DryRun),
        # so no weld may be recorded - see handle_weld_stats_req.
        hmi.dry_run = 1
    elif mode is not None:
        raise SystemExit(f"unknown mode {mode!r} (use corrupt-cam, "
                         "corrupt-weld, weld-demo or dry-run)")
    for i in range(cycles):
        print(f"--- cycle {i + 1}/{cycles} ---", flush=True)
        hmi.serve_cycle()
        # The weld rows the real HMI would have written to its analytics DB
        # this cycle (R_W_S with succ_ae = 1). Printed per cycle because the
        # list is reset per cycle, like the HMI's per-run weld session.
        print(f"    weld rows recorded: "
              f"{[(round(l, 3), round(t, 3)) for l, t in hmi.weld_stats_entries]}",
              flush=True)
    print("all cycles complete", flush=True)


if __name__ == "__main__":
    main(sys.argv)

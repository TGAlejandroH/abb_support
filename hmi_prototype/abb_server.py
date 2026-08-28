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

Phase 1 scope: program-selection exchange + request 100 (R_E, program end).
Dummy values everywhere; no HMI/camera logic.

Usage:
    python abb_server.py [host] [port] [cycles]
    defaults: 127.0.0.1 2000 2
"""

import math
import socket
import sys
import time

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
    """ABB quaternion (w, x, y, z) -> FANUC W,P,R in degrees (see above)."""
    n = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
    w, x, y, z = q1 / n, q2 / n, q3 / n, q4 / n
    rx = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sp = 2.0 * (w * y - z * x)
    sp = max(-1.0, min(1.0, sp))  # clamp against rounding at the gimbal poles
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


# ---------------------------------------------------------------------------
# The HMI prototype
# ---------------------------------------------------------------------------

class AbbTgsHmi:
    """Application-level request server / transport-level TCP client."""

    def __init__(self, host="127.0.0.1", port=2000, program_selection=1,
                 verbose=True):
        self.host = host
        self.port = port
        self.program_selection = program_selection
        self.verbose = verbose
        self.sock = None
        # last-received data, for tests / future HMI logic
        self.last_pose_xyzwpr = None
        self.last_sub_name = None
        self.handlers = {
            "100": self.handle_end_req,  # FANUC R_E
            # Phase 2: "1" R_C_F, "2" R_C, "4" R_W_F, "5" R_P_C,
            #          "10" R_F_T, "11" R_G_C_D, "14" R_W_P
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

    def handle_end_req(self):
        """FANUC R_E (id 100): receive current pose + sub-routine token."""
        pose_literal = self.do_receive()
        self.last_pose_xyzwpr = pose_literal_to_xyzwpr(pose_literal)
        self.last_sub_name = self.do_receive()
        self._log(f"  end request: pose(xyzwpr)={['%.3f' % v for v in self.last_pose_xyzwpr]} "
                  f"sub={self.last_sub_name!r}")

    # -- main loop -----------------------------------------------------------

    def serve_cycle(self):
        """One robot cycle: connect, program selection, serve requests until
        the robot disconnects (which it does right after request 100)."""
        self.connect()
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
    hmi = AbbTgsHmi(host=host, port=port)
    for i in range(cycles):
        print(f"--- cycle {i + 1}/{cycles} ---", flush=True)
        hmi.serve_cycle()
    print("all cycles complete", flush=True)


if __name__ == "__main__":
    main(sys.argv)

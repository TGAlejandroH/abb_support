"""Shared RAPID-side pieces for the fake-robot executable specs in test_*.py.

Every cycle of TG_Main.tgMainCycle starts with the two-port start handshake
(socket plan S14-S17, commit e3d9f4a, 2026-09-24). Only then does the robot
listen on the run port, which carries the cycle the phase 1-7 specs model. The
prototype HMI's serve_cycle always plays the handshake first. A fake robot that
does not answer it leaves the HMI retrying the handshake port for 30 s and then
failing: that is how every phase test broke on 2026-09-24.

WORSE, the default handshake port is 2001, the REAL one. If a VC or a cell were
listening there, a test would have talked to it. So every fake robot owns a
FakeHandshake on an ephemeral port, and the tests hand that port to the HMI.
"""

import socket

from abb_server import xyzwpr_to_pose_literal

#: What TG_SendStnFrame sends for a cell with no station frame worth the name:
#: identity, as the pose literal tgPoseToStr formats it.
IDENTITY_FRAME_LITERAL = xyzwpr_to_pose_literal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


class FakeHandshake:
    """Executable spec of the handshake half of TG_Main.tgMainCycle.

    TG_HandshakeCom     accept on the handshake port
    TG_SendStart        "START <seq> <part> <station>"                   (tgSendAck)
    TG_SendStnFrame     "STNFRAME", the pose literal, "STNAXES <b> <c>"  (tgSendAck each)
    TG_ReqVerdict       prompt "Give me the verdict" -> the HMI's verdict
    TG_HandshakeDisc    close the connection

    One instance serves every cycle of its fake robot. The listener stays open
    between cycles, where RAPID re-opens it per cycle; the HMI cannot tell the
    difference. The counter bumps per served handshake, as nTG_CycleSeq does.
    """

    def __init__(self, part=0, station=0, seq=0, station_frame=IDENTITY_FRAME_LITERAL,
                 axes=("0.000", "0.000")):
        self.part = part
        self.station = station
        self.seq = seq
        self.station_frame = station_frame
        self.axes = axes
        self.verdicts = []
        self.acks = []
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]

    def _send_ack(self, conn, payload):                      # tgSendAck
        conn.sendall(payload.encode("utf-8"))
        self.acks.append(conn.recv(16))

    def serve(self):
        """One handshake. Returns the verdict string ("1" = run the cycle)."""
        conn, _ = self.listener.accept()
        with conn:
            self.seq += 1
            self._send_ack(conn, "START %d %d %d" % (self.seq, self.part, self.station))
            self._send_ack(conn, "STNFRAME")
            self._send_ack(conn, self.station_frame)
            self._send_ack(conn, "STNAXES %s %s" % self.axes)
            conn.sendall(b"Give me the verdict")
            verdict = conn.recv(1024).decode("utf-8")
        self.verdicts.append(verdict)
        return verdict

    def close(self):
        self.listener.close()

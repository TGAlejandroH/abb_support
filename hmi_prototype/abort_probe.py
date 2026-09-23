"""R-5 probe: what does the controller do when the HMI dies mid-cycle?

Plan: TGuideWeldingHMI/docs/socket_start_trigger_hmi_plan_v1.md, acceptance
item 6. This is a listed GATE, not a nicety - `tgCycleAbort` does `ExitCycle`,
and under Production Manager the PP then lands in `gapMain.main`, ExecEngine
restarts, and `EE_ABORT` may fire `GoSafeEEv:MoveAbort` (robot moves to the
safe position). That chain has never been observed.

Method: connect-loop exactly like the real stand-in HMI, but on receiving
START, acknowledge it and then **abruptly reset the connection** - SO_LINGER
with a zero timeout sends a TCP RST rather than a graceful FIN, which is what
a crashed HMI or a yanked network cable looks like to the controller. The
robot is left blocked in `tgPromptRecv`'s `SocketReceive ... WAIT_MAX`.

Exits after one abort so it cannot re-arm the cell while the aftermath is
being inspected.

Usage: python abort_probe.py [host] [port]
"""

import socket
import struct
import sys
import time

CONNECT_TIMEOUT = 2.0
READ_TIMEOUT = 20.0
BACKOFF = 0.5
ACK = b"0"


def log(msg):
    print("[ABORT %s.%03d] %s" % (time.strftime("%H:%M:%S"),
                                  int((time.time() % 1) * 1000), msg), flush=True)


def main(argv):
    host = argv[1] if len(argv) > 1 else "127.0.0.1"
    port = int(argv[2]) if len(argv) > 2 else 2000
    refused = 0

    log("waiting for a dispatch; will RST the connection right after START")
    while True:
        try:
            sock = socket.create_connection((host, port), timeout=CONNECT_TIMEOUT)
        except (OSError, socket.timeout):
            refused += 1
            if refused % 20 == 1:
                log("  (not listening - %d refusals)" % refused)
            time.sleep(BACKOFF)
            continue

        log("CONNECTED")
        sock.settimeout(READ_TIMEOUT)
        try:
            first = sock.recv(1024).decode("utf-8", "replace")
            log("  robot -> %r" % first)
            if not first.strip().startswith("START"):
                log("  not a START message; ignoring this connection")
                sock.close()
                time.sleep(BACKOFF)
                continue
            sock.sendall(ACK)
            log("  acked START - now simulating an HMI crash")

            # SO_LINGER {on=1, timeout=0} makes close() emit a TCP RST instead
            # of a FIN. A FIN would look like an orderly shutdown; a crash does
            # not do that, and the distinction matters for what the controller's
            # SocketReceive reports.
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                            struct.pack("ii", 1, 0))
            sock.close()
            log("  connection RESET. Robot should now error in SocketReceive,")
            log("  run tgCycleAbort -> ExitCycle, and land back in gapMain.main.")
            return 0
        except Exception as exc:
            log("  unexpected: %r" % (exc,))
            try:
                sock.close()
            except OSError:
                pass
            return 1


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv))
    except KeyboardInterrupt:
        print()
        log("stopped")

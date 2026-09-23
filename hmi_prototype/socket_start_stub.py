"""Stand-in HMI for the socket-based start trigger — VC in the loop.

Design: TGuideWeldingHMI/docs/socket_start_trigger_hmi_plan_v1.md, phase R5.

This replaces the green START button with a connect loop. It is deliberately
NOT a welding client: it proves arming -> dispatch -> START -> part id ->
clean return to Production Manager, and nothing else (owner's scope choice,
"dispatch proof only, no welding").

What it exercises, and why each part is here:

  * The connect loop (S1). Under PM the robot's listener exists only while a
    TG part is running, so between parts this SHOULD see connection-refused.
    Refusals are counted and printed - seeing them is acceptance item 4, not
    a problem to hide.

  * The START lookahead (S12). The first message after connect is read RAW
    and inspected before anything is written back. If it is a START line it
    is acknowledged like any other robot->HMI payload; if it is not, it IS
    the "Give me the program ID" prompt and gets a payload reply instead.
    Acking a prompt, or replying to a payload, desyncs the whole wire - this
    is the Q-14 mechanism, tested here for real rather than on paper.

  * The S9 comparison. seq >  last -> fire. seq == last -> ignore (a replay
    of the cycle just serviced). seq <  last -> the controller's counter was
    reset; adopt, fire, and say so loudly. last_seq is persisted so an HMI
    restart cannot re-service a cycle, but it must NEVER block a real start.

Usage:
    python socket_start_stub.py [host] [port] [prog_id]
    defaults: 127.0.0.1 2000 2

    prog_id 2 = ends the cycle through TG_ReqEnd, so the id-100 end exchange
                is exercised too (recommended).
    prog_id 9 = unknown id; the robot ends the cycle immediately. Minimal,
                cannot desync, useful if something upstream is misbehaving.

Stdlib only. Ctrl-C to stop.
"""

import json
import os
import socket
import sys
import time

CONNECT_TIMEOUT = 2.0
READ_TIMEOUT = 15.0        # generous: PM may be indexing the station
BACKOFF = 0.5
ACK = b"0"

HERE = os.path.dirname(os.path.abspath(__file__))
STATE_FILE = os.path.join(HERE, ".socket_start_stub_state.json")

# Stand-in for the HMI's work-selection map (plan S11 / D47). Here it only
# proves the id arrives and resolves; the real map is a schema-validated file.
PART_MAP = {
    "9011": "VC test part A  (would load frame_a.tgs)",
    "9012": "VC test part B  (would load frame_b.tgs)",
}


def now():
    return time.strftime("%H:%M:%S") + ".%03d" % int((time.time() % 1) * 1000)


def log(msg):
    print("[HMI %s] %s" % (now(), msg), flush=True)


def load_last_seq():
    try:
        with open(STATE_FILE) as fh:
            return int(json.load(fh)["last_seq"])
    except Exception:
        return 0


def save_last_seq(seq):
    try:
        with open(STATE_FILE, "w") as fh:
            json.dump({"last_seq": seq}, fh)
    except OSError as exc:
        log("WARNING: could not persist last_seq (%s)" % exc)


def recv_text(sock):
    data = sock.recv(1024)
    if not data:
        raise ConnectionError("peer closed")
    return data.decode("utf-8", errors="replace")


def parse_start(line):
    """-> (seq, part_no) if this is a START message, else None.

    Tolerant on purpose: a malformed START must not kill the loop (P-3).
    """
    toks = line.strip().split()
    if not toks or toks[0] != "START":
        return None
    try:
        seq = int(toks[1])
    except (IndexError, ValueError):
        log("MALFORMED START (%r) -> ignoring, staying in the loop" % line.strip())
        return None
    part = toks[2] if len(toks) > 2 else "0"
    return seq, part


def probe_station():
    """Q-10 measurement, taken at the instant START arrives.

    At this moment Production Manager has finished whatever choreography it
    intends to do and has called our routine, so the state here is what PM
    set up for THIS part.

    Three things are read, and the FIRST is the decisive one:
      * mechunit mode  - if STN1 is still "Deactivated" then PM never
        activated the station, which means it never positioned it either,
        and the .tgs must own positioning after all (this would overturn
        plan S10).
      * ROB_1 jointtarget extax - where an ACTIVE station's axes appear to
        the robot task.
      * STN1 jointtarget - only meaningful once the unit is active.
    """
    try:
        import re as _re
        from rws_client import RwsClient
        c = RwsClient("http://127.0.0.1:80")
        g = lambda path: c._request("GET", path, query={"json": "1"}).decode("utf-8", "replace")
        out = []
        try:
            b = g("/rw/motionsystem/mechunits")
            # The mechunit list nests _links braces between _title and mode,
            # so a "[^}]*?" bridge fails. Pull both lists and zip - RWS emits
            # exactly one mode per unit, in order.
            names = _re.findall(r'"_title"\s*:\s*"([^"]+)"', b)
            modes = _re.findall(r'"mode"\s*:\s*"([^"]+)"', b)
            out.append("modes=" + ",".join("%s:%s" % (n, m) for n, m in zip(names, modes)))
        except Exception as e:
            out.append("modes unreadable (%s)" % e)
        for mu in ("ROB_1", "STN1"):
            try:
                b = g("/rw/motionsystem/mechunits/%s/jointtarget" % mu)
                # Dump EVERY axis field, rax and eax alike. A positioner mech
                # unit may surface its own axes in rax_1/rax_2 rather than in
                # extax, and filtering to eax_* would hide exactly the number
                # this measurement exists to capture.
                v = _re.findall(r'"(rax_\d|eax_[a-f])"\s*:\s*"?([-0-9.eE+]+)"?', b)
                out.append("%s{%s}" % (mu, " ".join("%s=%s" % kv for kv in v)))
            except Exception as e:
                out.append("%s unreadable (%s)" % (mu, e))
        return " | ".join(out)
    except Exception as e:
        return "probe failed: %r" % (e,)


def serve_cycle(sock, prog_id, last_seq):
    """One accepted connection. Returns the new last_seq."""
    first = recv_text(sock)
    log("  robot -> %r" % first)

    start = parse_start(first)
    if start is None:
        # Not a START line, so it IS the program-id prompt: reply, do not ack.
        # This is the legacy path a controller without the START send takes,
        # and it must stay byte-identical to today.
        log("  no START message -> legacy cell; treating as the prog-id prompt")
        sock.sendall(str(prog_id).encode())
        drain(sock)
        return last_seq

    sock.sendall(ACK)                      # START is an ordinary payload
    seq, part = start

    # ---- S9: the three-way comparison -------------------------------------
    if seq == last_seq:
        log("REPLAY: cycle %d already serviced -> IGNORING, no false start" % seq)
        return last_seq
    if seq < last_seq:
        log("COUNTER RESET: robot sent %d, we last saw %d. Controller restore "
            "or P-start. Adopting %d and STARTING - a rollback must never "
            "deadlock the cell." % (seq, last_seq, seq))
    else:
        log(">>> REAL START: cycle %d, part %s <<<" % (seq, part))

    log("    part resolves to: %s" % PART_MAP.get(part, "*** UNKNOWN PART ID ***"))
    log("    STATION AT START: %s" % probe_station())
    if part not in PART_MAP:
        # Plan S5: refuse this cycle, message the operator, do NOT latch.
        log("    refusing this cycle (unknown id). Next START will retry.")

    save_last_seq(seq)

    # ---- serve just enough of the wire for a clean cycle end --------------
    prompt = recv_text(sock)
    log("  robot -> %r" % prompt)
    sock.sendall(str(prog_id).encode())
    log("  HMI   -> %r  (program id)" % str(prog_id))
    drain(sock)
    return seq


def drain(sock):
    """Ack whatever the robot sends until it closes (TG_ReqEnd, then disc)."""
    while True:
        try:
            msg = recv_text(sock)
        except (ConnectionError, OSError):
            log("  robot closed the connection - cycle complete")
            return
        log("  robot -> %r" % msg)
        try:
            sock.sendall(ACK)
        except OSError:
            log("  robot closed the connection - cycle complete")
            return


def main(argv):
    host = argv[1] if len(argv) > 1 else "127.0.0.1"
    port = int(argv[2]) if len(argv) > 2 else 2000
    prog_id = int(argv[3]) if len(argv) > 3 else 2

    last_seq = load_last_seq()
    refused = 0
    cycles = 0

    log("connect loop started against %s:%d (this REPLACES the green START "
        "button). Persisted last_seq = %d" % (host, port, last_seq))
    log("between parts, connection-refused is the EXPECTED state.")

    while True:
        try:
            sock = socket.create_connection((host, port), timeout=CONNECT_TIMEOUT)
        except (OSError, socket.timeout):
            refused += 1
            if refused % 20 == 1:
                log("  (robot not listening - %d refusals so far; this is the "
                    "armed-only-during-a-cycle behaviour we want)" % refused)
            time.sleep(BACKOFF)
            continue

        log("CONNECTED - the robot's listener is up, so PM dispatched a TG part")
        sock.settimeout(READ_TIMEOUT)
        try:
            new_seq = serve_cycle(sock, prog_id, last_seq)
            if new_seq != last_seq:
                cycles += 1
            last_seq = new_seq
        except socket.timeout:
            log("  read timed out -> abandoning this connection, back to the loop")
        except (ConnectionError, OSError) as exc:
            log("  connection lost (%s) -> back to the loop" % exc)
        finally:
            try:
                sock.close()
            except OSError:
                pass

        log("SUMMARY so far: cycles serviced=%d, last_seq=%d, refusals=%d"
            % (cycles, last_seq, refused))
        time.sleep(0.05)      # fast loop-back: exactly what provokes the race


if __name__ == "__main__":
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print()
        log("stopped by operator")

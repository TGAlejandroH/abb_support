import sys, time, re, urllib.parse
from rws_client import RwsClient, RwsError
c = RwsClient("http://127.0.0.1:80")

def post(path, query, fields):
    body = urllib.parse.urlencode(fields).encode("ascii")
    return c._request("POST", path, data=body,
                      headers={"Content-Type": "application/x-www-form-urlencoded"},
                      query=query)

def get(path, q=None):
    return c._request("GET", path, query=q or {"json": "1"}).decode("utf-8", "replace")

def execstate():
    m = re.search(r'"ctrlexecstate":\s*"([^"]+)"', get("/rw/rapid/execution"))
    return m.group(1) if m else "?"

cmd = sys.argv[1] if len(sys.argv) > 1 else "status"

if cmd == "start":
    print("before:", execstate())
    try:
        c.request_mastership()
    except RwsError as e:
        print("mastership:", e)
    try:
        post("/rw/rapid/execution", {"action": "start"},
             {"regain": "continue", "execmode": "continue", "cycle": "forever",
              "condition": "none", "stopatbp": "disabled", "alltaskbytsp": "false"})
        print("start issued")
    except RwsError as e:
        print("start FAILED:", e)
    try:
        c.release_mastership()
    except RwsError:
        pass
    time.sleep(2)
    print("after :", execstate())

elif cmd == "pulse":
    sig = "/rw/iosystem/signals/Local/B_GAP_SIM/siGap_Run_Part_R1"
    for val in ("1", "0"):
        ok = False
        for q, f in (({"action": "set"}, {"lvalue": val}),
                     ({"action": "set"}, {"lvalue": val, "mastership": "implicit"})):
            try:
                post(sig, q, f); ok = True; break
            except RwsError as e:
                last = e
        print("  set %s -> %s" % (val, "OK" if ok else "FAILED: %s" % last))
        if val == "1":
            time.sleep(1.0)
    print("  signal now:", re.search(r'"lvalue":\s*"?([^",]+)"?', get(sig)).group(1))

elif cmd == "status":
    print("exec  :", execstate())
    print("pcp   :", re.search(r'"modulemame":\s*"([^"]*)".*?"routinename":\s*"([^"]*)"',
                               get("/rw/rapid/tasks/T_ROB1/pcp"), re.S).groups())

elif cmd == "elog":
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 25
    b = get("/rw/elog/0", {"json": "1", "lang": "en"})
    hrefs = re.findall(r'"href":\s*"(\d+)\?', b)[:n]
    for h in hrefs:
        try:
            d = get("/rw/elog/0/%s" % h, {"json": "1", "lang": "en"})
            t = re.search(r'"tstamp":\s*"([^"]*)"', d)
            ti = re.search(r'"title":\s*"([^"]*)"', d)
            ds = re.search(r'"desc":\s*"([^"]*)"', d)
            print("  [%s] %s | %s" % (t.group(1) if t else "?",
                                      ti.group(1) if ti else "?",
                                      (ds.group(1) if ds else "")[:150]))
        except Exception as e:
            print("  elog %s: %s" % (h, e))

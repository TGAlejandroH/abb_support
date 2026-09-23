"""Check what the VC really holds for the wobjTG_WeldActStn change.

Reads the live wobjTG_WeldActStn value from task memory, then the TG_Cell.sys / TG_Comms.sys
files the controller loaded from ($home/TGS, where _deploy_modules.py uploads them).
"""
import re
import sys

sys.path.insert(0, r"C:\Users\TG_Laptop08\PycharmProjects\abb_support\hmi_prototype")
from rws_session import RwsSession

c = RwsSession("http://127.0.0.1:80")
b = c._request("GET", "/rw/rapid/symbol/data/RAPID/T_ROB1/TG_Comms/wobjTG_WeldActStn",
               query={"json": "1"}).decode("utf-8", "replace")
m = re.search(r'"value":\s*"((?:\\.|[^"\\])*)"', b)
print("controller wobjTG_WeldActStn =", m.group(1).replace('\\"', '"') if m else b[:200])
for name in ("TG_Cell.sys", "TG_Comms.sys"):
    text = c._request("GET", "/fileservice/$home/TGS/" + name).decode("utf-8", "replace")
    print("%-13s binds: %d  declares: %d" % (
        name, text.count("wobjTG_WeldActStn.ufmec:="), text.count("PERS wobjdata wobjTG_WeldActStn")))

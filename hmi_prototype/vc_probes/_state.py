import re, sys
sys.path.insert(0, r"C:\Users\TG_Laptop08\PycharmProjects\abb_support\hmi_prototype")
from rws_session import RwsSession
c = RwsSession("http://127.0.0.1:80")
g = lambda p: c._request("GET", p, query={"json": "1"}).decode("utf-8", "replace")
print("exec:", re.search(r'"ctrlexecstate":\s*"([^"]+)"', g("/rw/rapid/execution")).group(1))
print("pcp :", re.findall(r'"(?:modulemame|routinename)":\s*"([^"]*)"', g("/rw/rapid/tasks/T_ROB1/pcp")))
try:
    print("uiinstr:", g("/rw/rapid/uiinstr/active")[:600])
except Exception as e:
    print("uiinstr:", e)
print("joints:", re.findall(r'"rax_\d":"([^"]*)"', g("/rw/motionsystem/mechunits/ROB_1/jointtarget")))

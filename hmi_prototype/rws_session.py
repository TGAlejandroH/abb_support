"""RwsClient that keeps ONE authenticated RWS session alive.

Finding V-3 (socket_start_trigger_hmi_plan_v1.md, VC 2026-09-22): the base
``RwsClient`` builds its opener without a cookie jar, so every request opens a
brand-new RWS session and re-runs the digest handshake. RWS 1.0 caps
concurrent sessions; a burst produced ``503 Service Unavailable`` and then a
blanket ``401 digest auth failed`` lockout until the stale sessions expired.

RWS identifies a session by the ``-http-session-`` and ``ABBCX`` cookies it
sets on the first authenticated response. Sending them back reuses that
session, so the digest handshake happens once and the session count stays at
one. This is the external-start plan's L-3 "cached handle" rule applied to
HTTP, and it is what the real HMI's RWS layer must do.

Usage is identical to RwsClient:
    from rws_session import RwsSession
    c = RwsSession("http://127.0.0.1:80")
"""

import http.cookiejar
import urllib.request

from rws_client import DEFAULT_PASSWORD, DEFAULT_USERNAME, RwsClient


class RwsSession(RwsClient):
    def __init__(self, base_url, username=DEFAULT_USERNAME,
                 password=DEFAULT_PASSWORD, timeout=10.0):
        super().__init__(base_url, username, password, timeout)
        password_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()
        password_mgr.add_password(None, self.base_url, username, password)
        self.cookies = http.cookiejar.CookieJar()
        self._opener = urllib.request.build_opener(
            urllib.request.HTTPCookieProcessor(self.cookies),
            urllib.request.HTTPDigestAuthHandler(password_mgr))

    def session_cookie_names(self):
        return sorted(c.name for c in self.cookies)

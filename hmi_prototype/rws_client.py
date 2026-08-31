"""Minimal Robot Web Services (RWS 1.0, RobotWare 6) client - stdlib only.

Phase 5 (docs/abb_program_touchup_and_retrieval_v1.md section 4): the HMI's
transport to and from the controller's file system. RWS is base RobotWare-OS
(no controller option), REST over HTTP with digest authentication - which
``urllib`` speaks natively, keeping this prototype dependency-free like the
rest of ``hmi_prototype``.

Endpoints used (cross-checked against ABB Developer Center and
ros-industrial/abb_librws):

  GET    /fileservice/<path>                        download a file
  PUT    /fileservice/<path>                        upload/overwrite a file
  DELETE /fileservice/<path>                        remove a file
  POST   /rw/rapid/modules/<mod>?action=save        save module memory->disk
  POST   /rw/rapid/symbol/data/<url>?action=set     write a PERS value
  GET    /rw/rapid/symbol/data/<url>?json=1         read a PERS value
  POST   /rw/mastership?action=request|release      RAPID mastership (edits)

Paths use the controller's environment-variable aliases, e.g.
``$home/TGS/TD05Test.mod``. Against a RobotStudio virtual controller the base
URL is ``http://127.0.0.1:80`` (or the port configured for the VC); default
UAS credentials are ``Default User`` / ``robotics``.
"""

import json
import urllib.error
import urllib.parse
import urllib.request

DEFAULT_USERNAME = "Default User"
DEFAULT_PASSWORD = "robotics"
DEFAULT_TASK = "T_ROB1"


class RwsError(Exception):
    """An RWS request failed (HTTP error, auth failure, connection refused)."""


class RwsFileNotFoundError(RwsError):
    """GET/DELETE on a file the controller does not have (HTTP 404)."""


class RwsClient:
    """One controller connection: base URL + digest credentials."""

    def __init__(self, base_url, username=DEFAULT_USERNAME,
                 password=DEFAULT_PASSWORD, timeout=10.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        password_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()
        password_mgr.add_password(None, self.base_url, username, password)
        self._opener = urllib.request.build_opener(
            urllib.request.HTTPDigestAuthHandler(password_mgr))

    # -- plumbing ------------------------------------------------------------

    def _request(self, method, path, data=None, headers=None, query=None):
        url = self.base_url + urllib.parse.quote(path, safe="/$")
        if query:
            url += "?" + urllib.parse.urlencode(query)
        request = urllib.request.Request(url, data=data, method=method,
                                         headers=headers or {})
        try:
            with self._opener.open(request, timeout=self.timeout) as response:
                return response.read()
        except urllib.error.HTTPError as exc:
            if exc.code == 404:
                raise RwsFileNotFoundError(f"{method} {path}: not found") from exc
            raise RwsError(f"{method} {path}: HTTP {exc.code} {exc.reason}") from exc
        except urllib.error.URLError as exc:
            raise RwsError(f"{method} {path}: {exc.reason}") from exc

    def _post_form(self, path, query, fields):
        body = urllib.parse.urlencode(fields).encode("ascii")
        headers = {"Content-Type": "application/x-www-form-urlencoded"}
        return self._request("POST", path, data=body, headers=headers,
                             query=query)

    # -- file service ----------------------------------------------------------

    def get_file(self, path):
        """Download ``path`` (e.g. "$home/TGS/edited/TD05Test.mod") -> bytes.

        Raises RwsFileNotFoundError if the controller does not have the file.
        """
        return self._request("GET", "/fileservice/" + path.lstrip("/"))

    def put_file(self, path, data):
        """Upload ``data`` (bytes) to ``path``, overwriting an existing file."""
        self._request("PUT", "/fileservice/" + path.lstrip("/"), data=data,
                      headers={"Content-Type": "application/octet-stream"})

    def delete_file(self, path):
        """Delete the file at ``path``."""
        self._request("DELETE", "/fileservice/" + path.lstrip("/"))

    # -- RAPID domain ----------------------------------------------------------

    def request_mastership(self):
        self._post_form("/rw/mastership", {"action": "request"}, {})

    def release_mastership(self):
        self._post_form("/rw/mastership", {"action": "release"}, {})

    def save_module(self, module, save_dir, filename, task=DEFAULT_TASK,
                    with_mastership=True):
        """Serialize ``module`` from program memory to ``save_dir/filename``.

        POST /rw/rapid/modules/<module>?action=save with the documented
        ``path``/``name`` parameters. Edit operations need RAPID mastership;
        it is requested/released around the call unless the caller manages it
        (VC check T2 in robotstudio_setup.md decides whether this works while
        the FlexPendant is in manual mode - trigger A in the touch-up doc does
        not depend on it).
        """
        if with_mastership:
            self.request_mastership()
        try:
            self._post_form(f"/rw/rapid/modules/{module}",
                            {"action": "save", "task": task},
                            {"path": save_dir, "name": filename})
        finally:
            if with_mastership:
                try:
                    self.release_mastership()
                except RwsError:
                    pass  # releasing is best-effort; the session drop releases too

    def symbol_path(self, symbol, module="TG_Comms", task=DEFAULT_TASK):
        """RWS symbol-data resource for a task-global PERS in ``module``."""
        return f"/rw/rapid/symbol/data/RAPID/{task}/{module}/{symbol}"

    def set_symbol(self, symbol, value, module="TG_Comms", task=DEFAULT_TASK):
        """Write a PERS value (RW6 takes mastership internally for this)."""
        self._post_form(self.symbol_path(symbol, module, task),
                        {"action": "set"}, {"value": str(value)})

    def get_symbol(self, symbol, module="TG_Comms", task=DEFAULT_TASK):
        """Read a PERS value; returns the RWS "value" string (e.g. "1")."""
        raw = self._request("GET", self.symbol_path(symbol, module, task),
                            query={"json": "1"})
        payload = json.loads(raw.decode("utf-8"))
        # RWS 1.0 JSON shape: {"_embedded": {"_state": [{"value": ...}]}}
        state = payload.get("_embedded", {}).get("_state", [])
        if state and "value" in state[0]:
            return state[0]["value"]
        raise RwsError(f"unexpected symbol payload for {symbol}: {payload!r}")

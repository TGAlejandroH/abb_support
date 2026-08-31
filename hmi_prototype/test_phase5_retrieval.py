"""Phase 5 tests: RWS transport + touch-up program retrieval.

Run:  python -m unittest discover -s hmi_prototype -v

Three layers:
  1. FakeRwsServer - an executable spec of the slice of Robot Web Services
     this phase uses (digest auth, /fileservice GET/PUT/DELETE, module save,
     symbol get/set, mastership), so the client is tested against the real
     wire behavior (401 challenge -> digest response) and not a mock.
  2. rws_client tests against it.
  3. tg_retrieve tests (validation, backup+adopt, cleanup, both sources) and
     the abb_server RWS transfer next to the KEPT copy fallback (Phase 5
     decision: the copy mechanism must survive).

The RAPID half of the phase (UnLoad \\ErrIfChanged staging) can only be
verified on the VC - robotstudio_setup.md section 16.
"""

import hashlib
import json
import os
import re
import tempfile
import threading
import unittest
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from rws_client import (
    DEFAULT_PASSWORD,
    DEFAULT_USERNAME,
    RwsClient,
    RwsError,
    RwsFileNotFoundError,
)
from test_phase2 import run_one_cycle
from tg_retrieve import (
    RetrieveError,
    RwsSource,
    VcHomeSource,
    retrieve_program,
    validate_tgs_module,
)

REALM = "fake-rws"
NONCE = "0123456789abcdef"


def _md5(text):
    return hashlib.md5(text.encode("utf-8")).hexdigest()


def _parse_auth_header(header):
    fields = {}
    for match in re.finditer(r'(\w+)=(?:"([^"]*)"|([^\s,]+))', header):
        fields[match.group(1)] = (match.group(2) if match.group(2) is not None
                                  else match.group(3))
    return fields


class _FakeRwsHandler(BaseHTTPRequestHandler):
    """The RWS slice used by Phase 5, with real HTTP digest authentication."""

    def log_message(self, fmt, *args):  # silence test output
        pass

    # -- digest auth ---------------------------------------------------------

    def _authorized(self):
        header = self.headers.get("Authorization", "")
        if not header.startswith("Digest "):
            return False
        fields = _parse_auth_header(header[len("Digest "):])
        server = self.server.owner
        if fields.get("username") != server.username:
            return False
        ha1 = _md5(f"{server.username}:{REALM}:{server.password}")
        ha2 = _md5(f"{self.command}:{fields.get('uri', '')}")
        if fields.get("qop") == "auth":
            expected = _md5(":".join([ha1, fields.get("nonce", ""),
                                      fields.get("nc", ""),
                                      fields.get("cnonce", ""),
                                      "auth", ha2]))
        else:
            expected = _md5(f"{ha1}:{fields.get('nonce', '')}:{ha2}")
        return fields.get("response") == expected

    def _challenge(self):
        self.send_response(401)
        self.send_header(
            "WWW-Authenticate",
            f'Digest realm="{REALM}", nonce="{NONCE}", qop="auth", '
            f'algorithm=MD5')
        self.send_header("Content-Length", "0")
        self.end_headers()

    # -- plumbing --------------------------------------------------------------

    def _reply(self, code, body=b""):
        self.send_response(code)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if body:
            self.wfile.write(body)

    def _read_body(self):
        length = int(self.headers.get("Content-Length", "0"))
        return self.rfile.read(length) if length else b""

    def _route(self):
        if not self._authorized():
            self._read_body()  # drain so the connection stays sane
            self._challenge()
            return None, None
        parsed = urllib.parse.urlparse(self.path)
        path = urllib.parse.unquote(parsed.path)
        query = urllib.parse.parse_qs(parsed.query)
        return path, query

    def _fs_key(self, path):
        return path[len("/fileservice/"):]

    # -- methods -----------------------------------------------------------------

    def do_GET(self):
        path, query = self._route()
        if path is None:
            return
        server = self.server.owner
        if path.startswith("/fileservice/"):
            key = self._fs_key(path)
            with server.lock:
                data = server.files.get(key)
            if data is None:
                self._reply(404)
            else:
                self._reply(200, data)
            return
        if path.startswith("/rw/rapid/symbol/data/"):
            symbol = path.rsplit("/", 1)[-1]
            with server.lock:
                value = server.symbols.get(symbol)
            if value is None:
                self._reply(404)
                return
            body = json.dumps(
                {"_embedded": {"_state": [{"value": value}]}}).encode("utf-8")
            self._reply(200, body)
            return
        self._reply(404)

    def do_PUT(self):
        path, query = self._route()
        if path is None:
            return
        server = self.server.owner
        body = self._read_body()
        if path.startswith("/fileservice/"):
            if server.fail_uploads:
                self._reply(500)
                return
            with server.lock:
                server.files[self._fs_key(path)] = body
            self._reply(201)
            return
        self._reply(404)

    def do_DELETE(self):
        path, query = self._route()
        if path is None:
            return
        server = self.server.owner
        if path.startswith("/fileservice/"):
            key = self._fs_key(path)
            with server.lock:
                existed = server.files.pop(key, None) is not None
            self._reply(204 if existed else 404)
            return
        self._reply(404)

    def do_POST(self):
        path, query = self._route()
        if path is None:
            return
        server = self.server.owner
        body = self._read_body().decode("utf-8")
        fields = {k: v[0] for k, v in urllib.parse.parse_qs(body).items()}
        action = (query.get("action") or [""])[0]

        if path == "/rw/mastership":
            with server.lock:
                server.mastership_log.append(action)
            self._reply(204)
            return

        if path.startswith("/rw/rapid/modules/") and action == "save":
            module = path.rsplit("/", 1)[-1]
            with server.lock:
                server.module_saves.append(
                    {"module": module, "task": (query.get("task") or [""])[0],
                     **fields})
                content = server.module_memory.get(module)
                if content is None:
                    self._reply(400)
                    return
                # The controller serializes program memory to path/name and
                # APPENDS ".mod" to the name (VC-observed 2026-08-31 - a name
                # of "T4a.mod" produced "T4a.mod.mod" on the real RW6).
                key = fields["path"].rstrip("/") + "/" + fields["name"] + ".mod"
                # RWS paths use $home aliases in fileservice keys.
                server.files[key] = content
            self._reply(200)
            return

        if path.startswith("/fileservice/") and fields.get("fs-action") == "create":
            # directory creation - modeled as a no-op (the store is flat)
            self._reply(201)
            return

        if path.startswith("/rw/rapid/symbol/data/") and action == "set":
            symbol = path.rsplit("/", 1)[-1]
            with server.lock:
                server.symbols[symbol] = fields.get("value", "")
            self._reply(204)
            return

        self._reply(404)


class FakeRwsServer:
    """Owns the HTTP server thread and the in-memory controller state."""

    def __init__(self, username=DEFAULT_USERNAME, password=DEFAULT_PASSWORD):
        self.username = username
        self.password = password
        self.lock = threading.Lock()
        self.files = {}          # fileservice key ("$home/...") -> bytes
        self.symbols = {}        # symbol name -> value string
        self.module_memory = {}  # module name -> bytes "in program memory"
        self.module_saves = []   # recorded action=save calls
        self.mastership_log = []
        self.fail_uploads = False
        self._httpd = ThreadingHTTPServer(("127.0.0.1", 0), _FakeRwsHandler)
        self._httpd.owner = self
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        daemon=True)

    @property
    def base_url(self):
        host, port = self._httpd.server_address
        return f"http://{host}:{port}"

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._httpd.shutdown()
        self._httpd.server_close()
        self._thread.join(timeout=10)


# ---------------------------------------------------------------------------
# Test data
# ---------------------------------------------------------------------------

def real_module_bytes():
    """The actual TD05Test.mod from the repo - retrieval must accept it."""
    path = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "abb", "rapid", "TGS", "TD05Test.mod"))
    with open(path, "rb") as f:
        return f.read()


def edited_module_bytes():
    """The repo module with one 'touched-up' target, as the controller's
    Save would serialize it (different literal, same structure)."""
    return real_module_bytes().replace(
        b"[[15,10,-10,0,40,0]", b"[[15.02,10.11,-9.87,0,40,0]")


# ---------------------------------------------------------------------------
# rws_client against the fake server
# ---------------------------------------------------------------------------

class TestRwsClient(unittest.TestCase):

    def test_put_get_delete_roundtrip(self):
        with FakeRwsServer() as server:
            client = RwsClient(server.base_url)
            payload = b"MODULE X_Mod\nENDMODULE\n"
            client.put_file("$home/TGS/X.mod", payload)
            self.assertEqual(client.get_file("$home/TGS/X.mod"), payload)
            client.delete_file("$home/TGS/X.mod")
            with self.assertRaises(RwsFileNotFoundError):
                client.get_file("$home/TGS/X.mod")

    def test_missing_file_raises_not_found(self):
        with FakeRwsServer() as server:
            client = RwsClient(server.base_url)
            with self.assertRaises(RwsFileNotFoundError):
                client.get_file("$home/TGS/edited/Nope.mod")

    def test_wrong_password_raises_rws_error(self):
        with FakeRwsServer() as server:
            client = RwsClient(server.base_url, password="wrong")
            with self.assertRaises(RwsError):
                client.get_file("$home/TGS/X.mod")

    def test_digest_actually_challenged(self):
        """The client must succeed via the 401->digest handshake, i.e. the
        fake really authenticates (guards against a fake that accepts all)."""
        with FakeRwsServer() as server:
            client = RwsClient(server.base_url)
            client.put_file("$home/a.txt", b"1")
            self.assertEqual(client.get_file("$home/a.txt"), b"1")

    def test_save_module_writes_file_no_explicit_mastership(self):
        """Default: no explicit mastership (RW6 takes it internally in AUTO,
        VC-validated), and a ".mod" passed in the basename is stripped so the
        controller's own append does not produce "X.mod.mod"."""
        with FakeRwsServer() as server:
            server.module_memory["TD05Test_Mod"] = edited_module_bytes()
            client = RwsClient(server.base_url)
            client.save_module("TD05Test_Mod", "$home/TGS/edited",
                               "TD05Test.mod")
            self.assertEqual(server.files["$home/TGS/edited/TD05Test.mod"],
                             edited_module_bytes())
            self.assertEqual(server.module_saves[0]["module"], "TD05Test_Mod")
            self.assertEqual(server.module_saves[0]["task"], "T_ROB1")
            self.assertEqual(server.module_saves[0]["name"], "TD05Test")
            self.assertEqual(server.mastership_log, [])

    def test_save_module_with_explicit_mastership(self):
        with FakeRwsServer() as server:
            server.module_memory["TD05Test_Mod"] = edited_module_bytes()
            client = RwsClient(server.base_url)
            client.save_module("TD05Test_Mod", "$home/TGS/edited", "TD05Test",
                               with_mastership=True)
            self.assertEqual(server.mastership_log, ["request", "release"])

    def test_create_directory(self):
        with FakeRwsServer() as server:
            RwsClient(server.base_url).create_directory("$home/TGS", "edited")

    def test_symbol_set_and_get(self):
        with FakeRwsServer() as server:
            client = RwsClient(server.base_url)
            client.set_symbol("nTG_ProgEdited", 0)
            self.assertEqual(server.symbols["nTG_ProgEdited"], "0")
            self.assertEqual(client.get_symbol("nTG_ProgEdited"), "0")


# ---------------------------------------------------------------------------
# abb_server: RWS transfer next to the kept copy fallback
# ---------------------------------------------------------------------------

class TestAbbServerRwsTransfer(unittest.TestCase):

    def test_module_uploaded_via_rws(self):
        with FakeRwsServer() as server:
            robot, hmi = run_one_cycle({"rws": RwsClient(server.base_url)})
            self.assertEqual(robot.received["ftp_status"], "1")
            self.assertEqual(server.files["$home/TGS/TD05Test.mod"],
                             real_module_bytes())
            self.assertEqual(hmi.request_log[-1], "100")

    def test_rws_failure_reports_ftp_zero(self):
        """Upload refused (HTTP 500) -> ftp status 0 -> the robot skips the
        program, exactly like the copy fallback's error path."""
        with FakeRwsServer() as server:
            server.fail_uploads = True
            robot, hmi = run_one_cycle({"rws": RwsClient(server.base_url)})
            self.assertEqual(robot.received["ftp_status"], "0")
            self.assertEqual(hmi.request_log, ["10"])

    def test_copy_fallback_untouched(self):
        """Phase 5 decision: the direct-copy mechanism must keep working
        exactly as before when no RWS client is configured."""
        with tempfile.TemporaryDirectory() as home:
            robot, hmi = run_one_cycle({"vc_home_dir": home})
            self.assertIsNone(hmi.rws)
            dst = os.path.join(home, "TGS", "TD05Test.mod")
            self.assertTrue(os.path.isfile(dst))
            self.assertEqual(robot.received["ftp_status"], "1")


# ---------------------------------------------------------------------------
# Validation (touch-up doc section 6.3)
# ---------------------------------------------------------------------------

class TestValidation(unittest.TestCase):

    def test_real_repo_module_passes(self):
        validate_tgs_module(real_module_bytes(), "TD05Test")

    def test_edited_module_passes(self):
        validate_tgs_module(edited_module_bytes(), "TD05Test")

    def test_leading_comments_allowed(self):
        data = (b"! saved by controller\n\n"
                b"MODULE P1_Mod\n PROC P1()\n ENDPROC\nENDMODULE\n")
        validate_tgs_module(data, "P1")

    def test_wrong_module_name_rejected(self):
        with self.assertRaises(RetrieveError):
            validate_tgs_module(real_module_bytes(), "TD06Test")

    def test_missing_proc_rejected(self):
        data = b"MODULE P1_Mod\n PROC other()\n ENDPROC\nENDMODULE\n"
        with self.assertRaises(RetrieveError):
            validate_tgs_module(data, "P1")

    def test_empty_rejected(self):
        with self.assertRaises(RetrieveError):
            validate_tgs_module(b"", "P1")


# ---------------------------------------------------------------------------
# retrieve_program: the Retrieve-button behavior
# ---------------------------------------------------------------------------

def _quiet(_msg):
    pass


class TestRetrieveViaRws(unittest.TestCase):

    def test_full_retrieve_adopt_backup_cleanup(self):
        with FakeRwsServer() as server, \
                tempfile.TemporaryDirectory() as dest:
            server.files["$home/TGS/edited/TD05Test.mod"] = edited_module_bytes()
            server.symbols["nTG_ProgEdited"] = "1"
            master = os.path.join(dest, "TD05Test.mod")
            with open(master, "wb") as f:
                f.write(real_module_bytes())

            source = RwsSource(RwsClient(server.base_url))
            result = retrieve_program("TD05Test", source, dest, log=_quiet)

            self.assertEqual(result, master)
            with open(master, "rb") as f:
                self.assertEqual(f.read(), edited_module_bytes())
            backups = os.listdir(os.path.join(dest, "retrieved_backups"))
            self.assertEqual(len(backups), 1)
            self.assertTrue(backups[0].startswith("TD05Test_"))
            with open(os.path.join(dest, "retrieved_backups", backups[0]),
                      "rb") as f:
                self.assertEqual(f.read(), real_module_bytes())
            # cleanup: staged file gone, flag cleared
            self.assertNotIn("$home/TGS/edited/TD05Test.mod", server.files)
            self.assertEqual(server.symbols["nTG_ProgEdited"], "0")

    def test_nothing_staged_returns_none_and_master_untouched(self):
        with FakeRwsServer() as server, \
                tempfile.TemporaryDirectory() as dest:
            master = os.path.join(dest, "TD05Test.mod")
            with open(master, "wb") as f:
                f.write(real_module_bytes())
            source = RwsSource(RwsClient(server.base_url))
            result = retrieve_program("TD05Test", source, dest, log=_quiet)
            self.assertIsNone(result)
            with open(master, "rb") as f:
                self.assertEqual(f.read(), real_module_bytes())
            self.assertFalse(
                os.path.isdir(os.path.join(dest, "retrieved_backups")))

    def test_invalid_staged_rejected_master_untouched(self):
        with FakeRwsServer() as server, \
                tempfile.TemporaryDirectory() as dest:
            server.files["$home/TGS/edited/TD05Test.mod"] = \
                b"MODULE Wrong_Mod\nPROC Wrong()\nENDPROC\nENDMODULE\n"
            master = os.path.join(dest, "TD05Test.mod")
            with open(master, "wb") as f:
                f.write(real_module_bytes())
            source = RwsSource(RwsClient(server.base_url))
            with self.assertRaises(RetrieveError):
                retrieve_program("TD05Test", source, dest, log=_quiet)
            with open(master, "rb") as f:
                self.assertEqual(f.read(), real_module_bytes())
            # a rejected file is NOT cleaned up - it is evidence
            self.assertIn("$home/TGS/edited/TD05Test.mod", server.files)

    def test_no_cleanup_keeps_staged_and_flag(self):
        with FakeRwsServer() as server, \
                tempfile.TemporaryDirectory() as dest:
            server.files["$home/TGS/edited/TD05Test.mod"] = edited_module_bytes()
            server.symbols["nTG_ProgEdited"] = "1"
            source = RwsSource(RwsClient(server.base_url))
            retrieve_program("TD05Test", source, dest, cleanup=False,
                             log=_quiet)
            self.assertIn("$home/TGS/edited/TD05Test.mod", server.files)
            self.assertEqual(server.symbols["nTG_ProgEdited"], "1")

    def test_staged_by_module_save_can_be_retrieved(self):
        """End-to-end against the fake: 'controller memory' -> action=save
        (what tgSaveEditedModule does on the robot, here driven over RWS as
        trigger B) -> retrieve adopts the serialized module."""
        with FakeRwsServer() as server, \
                tempfile.TemporaryDirectory() as dest:
            server.module_memory["TD05Test_Mod"] = edited_module_bytes()
            client = RwsClient(server.base_url)
            client.save_module("TD05Test_Mod", "$home/TGS/edited", "TD05Test")
            result = retrieve_program("TD05Test", RwsSource(client), dest,
                                      log=_quiet)
            with open(result, "rb") as f:
                self.assertEqual(f.read(), edited_module_bytes())


class TestRetrieveViaVcHome(unittest.TestCase):

    def test_retrieve_from_vc_home_folder(self):
        with tempfile.TemporaryDirectory() as home, \
                tempfile.TemporaryDirectory() as dest:
            staged_dir = os.path.join(home, "TGS", "edited")
            os.makedirs(staged_dir)
            staged = os.path.join(staged_dir, "TD05Test.mod")
            with open(staged, "wb") as f:
                f.write(edited_module_bytes())
            source = VcHomeSource(home)
            result = retrieve_program("TD05Test", source, dest, log=_quiet)
            with open(result, "rb") as f:
                self.assertEqual(f.read(), edited_module_bytes())
            self.assertFalse(os.path.exists(staged), "staged file not cleaned")

    def test_nothing_staged_in_vc_home(self):
        with tempfile.TemporaryDirectory() as home, \
                tempfile.TemporaryDirectory() as dest:
            source = VcHomeSource(home)
            self.assertIsNone(
                retrieve_program("TD05Test", source, dest, log=_quiet))


if __name__ == "__main__":
    unittest.main(verbosity=2)

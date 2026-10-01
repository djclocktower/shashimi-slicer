#!/usr/bin/env python3
"""Tests for tools/orca_mcp_bridge.py against a fake application endpoint (stdlib only).

Run from the repo root:  python3 -m unittest discover -s tools/tests -v
POSIX only: the fake app listens on a Unix-domain socket.
"""

import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest

BRIDGE = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "orca_mcp_bridge.py"))

DESCRIBE = {"tools": [
    {"name": "app_info", "summary": "state", "params": []},
    {"name": "object_get", "summary": "one object", "params": [
        {"name": "object", "type": "integer", "description": "index"},
        {"name": "rotation", "type": "array", "description": "", "default": None},
        {"name": "value", "type": "any", "description": "anything", "default": ""}]},
    {"name": "screenshot", "summary": "image", "params": []},
]}


class FakeApp:
    """Answers the app's line-delimited JSON-RPC on a Unix socket, recording requests."""

    def __init__(self, path):
        self.path = path
        self.requests = []
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(path)
        self.sock.listen(4)
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                conn, _ = self.sock.accept()
            except OSError:
                return
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn):
        with conn, conn.makefile("rb") as f:
            for line in f:
                req = json.loads(line)
                self.requests.append(req)
                m = req["method"]
                if m == "describe_tools":
                    out = {"result": DESCRIBE}
                elif m == "screenshot":
                    out = {"result": {"png_base64": "iVBORw0KGgo=", "mime": "image/png", "width": 1, "height": 1}}
                elif m == "app_info":
                    out = {"result": {"version": "test", "params": req.get("params")}}
                else:
                    out = {"error": {"code": -32601, "message": "Unknown method: " + m}}
                out.update(jsonrpc="2.0", id=req["id"])
                conn.sendall((json.dumps(out) + "\n").encode())

    def close(self):
        self.sock.close()


def run_bridge(endpoint, requests, delay_before_last=0.0, before_last=None):
    """Feed requests to the bridge's stdin; return its parsed stdout messages."""
    proc = subprocess.Popen([sys.executable, BRIDGE, endpoint], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    for i, r in enumerate(requests):
        if i == len(requests) - 1 and before_last:
            time.sleep(delay_before_last)
            before_last()
        proc.stdin.write((json.dumps(r) + "\n").encode())
        proc.stdin.flush()
    out, _ = proc.communicate(timeout=30)
    return [json.loads(l) for l in out.decode().splitlines() if l.strip()]


def call(rid, name, arguments=None):
    return {"jsonrpc": "2.0", "id": rid, "method": "tools/call", "params": {"name": name, "arguments": arguments or {}}}


@unittest.skipUnless(hasattr(socket, "AF_UNIX"), "needs Unix-domain sockets")
class BridgeTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.endpoint = os.path.join(self.tmp, "app.sock")
        self.app = None

    def tearDown(self):
        if self.app:
            self.app.close()

    def test_tool_list_is_built_from_describe_tools(self):
        self.app = FakeApp(self.endpoint)
        msgs = run_bridge(self.endpoint, [{"jsonrpc": "2.0", "id": 1, "method": "tools/list"}])
        tools = {t["name"]: t for t in msgs[0]["result"]["tools"]}
        self.assertIn("orca_call", tools)
        schema = tools["object_get"]["inputSchema"]
        self.assertEqual(schema["required"], ["object"])
        self.assertEqual(schema["properties"]["rotation"], {"type": "array", "items": {}})
        self.assertNotIn("type", schema["properties"]["value"])   # "any" is untyped

    def test_calls_results_images_and_errors(self):
        self.app = FakeApp(self.endpoint)
        msgs = run_bridge(self.endpoint, [
            call(1, "screenshot"),
            call(2, "orca_call", {"method": "app_info", "params": {"a": 1}}),
            call(3, "missing"),
        ])
        image = msgs[0]["result"]["content"]
        self.assertEqual(image[0], {"type": "image", "data": "iVBORw0KGgo=", "mimeType": "image/png"})
        self.assertEqual(json.loads(image[1]["text"])["width"], 1)
        self.assertEqual(json.loads(msgs[1]["result"]["content"][0]["text"])["params"], {"a": 1})
        self.assertTrue(msgs[2]["result"]["isError"])
        self.assertIn("-32601", msgs[2]["result"]["content"][0]["text"])

    def test_app_down_then_up_notifies_list_changed(self):
        def start():
            self.app = FakeApp(self.endpoint)
        msgs = run_bridge(self.endpoint, [{"jsonrpc": "2.0", "id": 1, "method": "tools/list"},
                                          call(2, "app_info"), call(3, "app_info")],
                          delay_before_last=0.1, before_last=start)
        self.assertIn("describe_tools", [t["name"] for t in msgs[0]["result"]["tools"]])   # fallback list
        self.assertTrue(msgs[1]["result"]["isError"])                                       # app still down
        self.assertIn("unreachable", msgs[1]["result"]["content"][0]["text"])
        self.assertFalse(msgs[2]["result"].get("isError", False))
        self.assertEqual(msgs[3], {"jsonrpc": "2.0", "method": "notifications/tools/list_changed"})


if __name__ == "__main__":
    unittest.main()

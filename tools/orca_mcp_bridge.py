#!/usr/bin/env python3
"""Zero-dependency stdio MCP server bridging to the slicer's control endpoint.

Speaks MCP (JSON-RPC 2.0 over newline-delimited stdio) to an MCP client (Claude Code...)
and forwards each tool call to the running application's control endpoint, opened when the
app is launched with ORCA_MCP set (see docs/HLSD/mcp-control.md):

    Linux / macOS   a Unix-domain socket, default /tmp/orca-mcp.sock
    Windows         a named pipe, default \\\\.\\pipe\\orca-mcp

The tool list is built live from the app's own `describe_tools` reply, so tools added to the
app surface here without touching this file. `orca_call` forwards any method by name, which
also covers the case where the client listed tools before the app was running.

Usage:  orca_mcp_bridge.py [ENDPOINT]
        ENDPOINT defaults to $ORCA_MCP (or legacy $ORCA_CAD_MCP) when it names a path,
        else the platform default above; the legacy /tmp/orca-cad-mcp.sock is tried too.
Example Claude Code registration:
        claude mcp add orca -- python3 /path/to/tools/orca_mcp_bridge.py
"""
import itertools
import json
import os
import socket
import sys
import time

SERVER_INFO = {"name": "orca", "version": "1.0"}
IS_WINDOWS = os.name == "nt"
# The app answers within its own limits (60 s, 600 s for long tools, `timeout` for waits);
# this only guards against a dead peer.
IO_TIMEOUT = 1800

_app_id = itertools.count(1)


def _candidates():
    if len(sys.argv) > 1:
        return [sys.argv[1]]
    out = []
    for var in ("ORCA_MCP", "ORCA_CAD_MCP"):
        v = os.environ.get(var, "")
        if v and v != "1":
            out.append(v if not IS_WINDOWS or v.startswith("\\\\.\\pipe\\") else "\\\\.\\pipe\\" + v)
    if IS_WINDOWS:
        out += [r"\\.\pipe\orca-mcp", r"\\.\pipe\orca-cad-mcp"]
    else:
        out += ["/tmp/orca-mcp.sock", "/tmp/orca-cad-mcp.sock"]
    return out


def _endpoint():
    cands = _candidates()
    if not IS_WINDOWS:
        for c in cands:
            if os.path.exists(c):
                return c
    return cands[0]


# --- app control round-trip ----------------------------------------------------------
def _round_trip(line):
    ep = _endpoint()
    if IS_WINDOWS:
        last = None
        for ep in _candidates():
            for _attempt in range(40):
                try:
                    with open(ep, "r+b", buffering=0) as pipe:
                        pipe.write(line)
                        buf = b""
                        while not buf.endswith(b"\n"):
                            chunk = pipe.read(65536)
                            if not chunk:
                                break
                            buf += chunk
                        return buf
                except OSError as e:
                    last = e
                    if getattr(e, "winerror", None) != 231:   # ERROR_PIPE_BUSY: every instance taken, retry
                        break
                    time.sleep(0.05)
        raise last
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(IO_TIMEOUT)
    try:
        s.connect(ep)
        s.sendall(line)
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
    finally:
        s.close()
    return buf


def app_call(method, params=None):
    """One request to the app. Raises on transport failure."""
    req = {"jsonrpc": "2.0", "id": next(_app_id), "method": method}
    if params is not None:
        req["params"] = params
    return json.loads(_round_trip((json.dumps(req) + "\n").encode()).decode())


# --- describe_tools -> MCP tool schemas -----------------------------------------------
_TYPES = {"number", "integer", "string", "boolean", "array", "object"}


def _param_schema(p):
    t = p.get("type", "string")
    sch = {}
    if t in _TYPES:
        sch["type"] = t
    if t == "array":
        sch["items"] = {}
    desc = p.get("description", "")
    if "unit" in p:
        desc = (desc + " " if desc else "") + f"in {p['unit']}"
    if desc:
        sch["description"] = desc
    for src, dst in (("enum", "enum"), ("min", "minimum"), ("max", "maximum")):
        if src in p:
            sch[dst] = p[src]
    if p.get("default") is not None:
        sch["default"] = p["default"]
    return sch


_ORCA_CALL = {
    "name": "orca_call",
    "description": "Call any application method by name with a params object (see describe_tools). "
                   "Use it for tools that are not listed, e.g. when the app started after this list was read.",
    "inputSchema": {"type": "object",
                    "properties": {"method": {"type": "string"}, "params": {"type": "object"}},
                    "required": ["method"]},
}

_FALLBACK = [
    {"name": "describe_tools", "summary": "List every callable tool (the app must be running with ORCA_MCP set).", "params": []},
    {"name": "app_info", "summary": "Application state.", "params": []},
]

_served_fallback = False


def list_tools():
    global _served_fallback
    try:
        desc = app_call("describe_tools").get("result", {})
        tools = desc["tools"]
        _served_fallback = False
    except Exception:
        tools = _FALLBACK
        _served_fallback = True
    out = []
    for t in tools:
        params = t.get("params", [])
        out.append({
            "name": t["name"],
            "description": t.get("summary", ""),
            "inputSchema": {"type": "object",
                            "properties": {p["name"]: _param_schema(p) for p in params},
                            "required": [p["name"] for p in params if "default" not in p]},
        })
    out.append(_ORCA_CALL)
    return out


# --- results -> MCP content ------------------------------------------------------------
def _content(result):
    """PNG screenshots become image content; everything else is JSON text."""
    if isinstance(result, dict) and "png_base64" in result:
        meta = {k: v for k, v in result.items() if k != "png_base64"}
        return [{"type": "image", "data": result["png_base64"], "mimeType": result.get("mime", "image/png")},
                {"type": "text", "text": json.dumps(meta)}]
    return [{"type": "text", "text": json.dumps(result, indent=2)}]


def _error(rid, text):
    return {"jsonrpc": "2.0", "id": rid, "result": {"content": [{"type": "text", "text": text}], "isError": True}}


# --- MCP method handlers ---------------------------------------------------------------
def handle(req):
    m = req.get("method")
    rid = req.get("id")
    if m == "initialize":
        ver = (req.get("params") or {}).get("protocolVersion", "2024-11-05")
        return {"jsonrpc": "2.0", "id": rid, "result": {
            "protocolVersion": ver,
            "capabilities": {"tools": {"listChanged": True}},
            "serverInfo": SERVER_INFO}}
    if m == "ping":
        return {"jsonrpc": "2.0", "id": rid, "result": {}}
    if m == "tools/list":
        return {"jsonrpc": "2.0", "id": rid, "result": {"tools": list_tools()}}
    if m == "tools/call":
        p = req.get("params") or {}
        name = p.get("name")
        args = p.get("arguments") or {}
        if name == "orca_call":
            name, args = args.get("method", ""), args.get("params") or {}
        try:
            reply = app_call(name, args)
        except Exception as e:
            return _error(rid, f"control endpoint unreachable ({_endpoint()}): {e}. "
                               "Start the app with ORCA_MCP=1 (see docs/HLSD/mcp-control.md).")
        if "error" in reply:
            return _error(rid, json.dumps(reply["error"]))
        return {"jsonrpc": "2.0", "id": rid, "result": {"content": _content(reply.get("result"))}}
    if rid is not None:  # unknown *request*
        return {"jsonrpc": "2.0", "id": rid, "error": {"code": -32601, "message": f"method not found: {m}"}}
    return None  # notification (e.g. notifications/initialized) -> no reply


def _send(msg):
    sys.stdout.write(json.dumps(msg) + "\n")
    sys.stdout.flush()


def main():
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except Exception:
            continue
        if not isinstance(req, dict):   # batches are not used by MCP clients
            _send({"jsonrpc": "2.0", "id": None, "error": {"code": -32600, "message": "expected a single JSON-RPC object"}})
            continue
        was_fallback = _served_fallback
        resp = handle(req)
        if resp is not None:
            _send(resp)
        # The client listed tools while the app was down and a call now got through:
        # tell it to list them again.
        if was_fallback and req.get("method") == "tools/call" and not resp["result"].get("isError"):
            _send({"jsonrpc": "2.0", "method": "notifications/tools/list_changed"})
            globals()["_served_fallback"] = False


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Former name of tools/orca_mcp_bridge.py, kept so existing MCP client registrations work."""
import os
import runpy
import sys

sys.argv[0] = os.path.join(os.path.dirname(os.path.abspath(__file__)), "orca_mcp_bridge.py")
runpy.run_path(sys.argv[0], run_name="__main__")

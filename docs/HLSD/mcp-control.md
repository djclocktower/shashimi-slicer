# MCP control surface

The application can be driven by an AI agent through the Model Context Protocol. A local
JSON-RPC 2.0 endpoint inside the running app exposes tools for everything a user does: projects,
objects, plates, presets, every print/filament/printer setting, slicing, G-code, network printers,
calibration, the Design and CAM tabs, and -- through generic UI tools -- any window, dialog or
menu. A zero-dependency stdio bridge, `tools/orca_mcp_bridge.py`, turns that endpoint into an MCP
server for clients such as Claude Code.

## Enabling it

The endpoint is off unless an environment variable names it when the app starts:

| Variable | Linux / macOS | Windows |
|---|---|---|
| `ORCA_MCP=1` | Unix socket `/tmp/orca-mcp.sock` | named pipe `\\.\pipe\orca-mcp` |
| `ORCA_MCP=<path>` | that socket path | that pipe name (`\\.\pipe\` is prepended if missing) |
| `ORCA_CAD_MCP=...` | legacy spelling; `1` means `/tmp/orca-cad-mcp.sock` | `\\.\pipe\orca-cad-mcp` |

Register the bridge with the MCP client, e.g.
`claude mcp add orca -- python3 /path/to/tools/orca_mcp_bridge.py`. The bridge finds the endpoint
from its argument, then `$ORCA_MCP`, then the defaults above. `tools/orca_cad_mcp_bridge.py` is the
former name and forwards to it. `tools/tests/test_orca_mcp_bridge.py` exercises the bridge
against a fake app endpoint (`python3 -m unittest discover -s tools/tests`).

**Access control.** The endpoint can read and write files and start prints, so it is reachable by
the user who started the app and nobody else. The socket is created mode 0600 (umask around
`bind()`, then `chmod`, refusing to listen if that fails). The pipe keeps the default DACL, which
grants write access only to its creator, LocalSystem and administrators; it rejects remote clients
and claims its name with `FILE_FLAG_FIRST_PIPE_INSTANCE`. Preference values that look like
credentials are redacted from `app_preferences_get`; printer API keys are never returned.

## Structure

```
MCP client ── stdio ── orca_mcp_bridge.py ── socket / pipe ── McpServer ── UI thread ── tools
                                                                  │
                                    McpAppTools, McpSlicerTools, McpUiTools (registered tools)
                                    CAD/McpControl (every method not registered: Design & CAM)
```

- `src/slic3r/GUI/Mcp/McpServer.*` -- transport, request dispatch, the tool registry and
  `describe_tools`. Started at the end of `GUI_App::on_init_inner`.
- `McpAppTools.cpp` -- application state, tabs, mode, preferences, undo history, camera,
  screenshots, and the busy state used for waiting.
- `McpSlicerTools.cpp` -- project, objects and parts, per-object settings and layer ranges,
  plates, presets, settings, slicing, G-code, network printers, calibration.
- `McpUiTools.cpp` -- top-level windows, widget trees, clicking, setting control values, closing
  dialogs, main and context menus, synthesized OS mouse/keyboard input.
- `CAD/McpControl.cpp` -- the Design tab's tools (`describe_scene`, `sketch_*`, `extrude`...) and
  `cam_*`; the server hands it every method it has not registered.

`describe_tools` returns every tool with a parameter descriptor (`name`, `type`, `description`,
`default`, `enum`); a parameter without a default is required. The bridge builds the MCP tool list
from that reply, so a tool registered in the app needs no change to the bridge. Tools are
registered explicitly from `start_server_if_enabled()`, not by static initialisers, because the
GUI is a static library and the linker would drop unreferenced registrars.

## Threading

Every tool runs on the wx main thread, through the same entry points the GUI's own controls call
(`Plater`, `Tab`, `ObjectList`, `Selection`...), so the 3D view, object list, undo history and dirty
flags follow along exactly as for a user action. The server thread accepts connections (one thread
per connection), parses a request and posts it with `CallAfter`, then waits for the answer.
Nothing may escape the posted lambda: the wx event loop does not catch, so an escaping exception
would terminate the app.

### Modal dialogs

Many user actions open modal dialogs (unsaved changes, preset transfer, warnings). A modal dialog
runs a nested event loop on the main thread, and the call that opened it stays on the stack until
it closes. The server handles this in three ways:

- **Early answer.** While waiting, the server thread asks the UI thread every 0.8 s whether a modal
  dialog is open. If the call's handler has started and a dialog is up, the call is answered with
  error `-32004` naming the dialog, instead of hanging until a timeout. The operation itself
  continues once the dialog is answered.
- **Reaching the dialog.** Posted calls are still processed inside the dialog's nested event loop,
  and each connection has its own thread, so the agent can use `ui_windows`, `ui_tree`, `ui_click`,
  `ui_set_value` and `ui_close` to read and answer it, then `wait_idle`.
- **Re-entrancy guard.** While a modal dialog is open, while another tool is still on the UI
  thread's stack, or whenever a call arrives through any nested event loop (including the
  `wxYield` of progress bars and popup menus), only tools flagged `ModalSafe` run: perception
  (`app_info`, `screenshot`, `slicing_status`...) and the `ui_*` tools. Anything else is refused
  with `-32003`, because it would run in the middle of the operation that opened that loop and
  could change the model beneath it. With dialogs stacked, the innermost one -- the newest -- is
  the one addressed and reported.

`ui_click`, `ui_set_value` change events and `ui_menu_invoke` are queued rather than processed
inline, for the same reason: a handler that opens another dialog must not run on the tool's stack.

### Waiting for background work

Slicing, export, arrange and orient run in the background. `wait_idle` polls (off the UI thread)
until slicing, export, UI jobs and project loading are idle twice in a row -- "slice all" passes
through an idle instant between plates -- or a modal dialog needs answering, or the timeout passes.
Tools flagged `Waitable` accept `wait: true` and then return the waited state plus a status reply
(`slice` returns `slicing_status`, `gcode_export` the written file).

Time limits: 60 s for a call on the UI thread, 600 s for tools flagged `Long` (file loading,
exports); past that the call is answered with `-32005` and the work continues.

## Tools

| Area | Tools |
|---|---|
| Application | `app_info`, `app_select_tab`, `app_set_mode`, `app_preferences_get/set`, `history`, `undo`, `redo`, `view_camera`, `screenshot`, `wait_idle` |
| Project | `project_new`, `project_open`, `project_save`, `project_import`, `project_export_3mf`, `model_export`, `gcode_preview_file` |
| Objects | `objects_list`, `object_get`, `objects_select`, `objects_delete`, `object_transform`, `object_set`, `object_instances`, `object_mirror`, `object_split`, `object_cut`, `object_add_shape`, `volume_set`, `volume_delete`, `object_settings_get/set`, `arrange`, `orient` |
| Plates | `plates_list`, `plate_select`, `plate_add`, `plate_delete`, `plate_duplicate`, `plate_set`, `object_move_to_plate` |
| Presets | `presets_list`, `preset_select`, `preset_save`, `preset_discard`, `preset_diff`, `filament_add`, `filament_remove` |
| Settings | `config_describe`, `config_get`, `config_set` |
| Slicing / G-code | `slice`, `slicing_status`, `slice_cancel`, `gcode_stats`, `gcode_export`, `gcode_read` |
| Printers | `printers_list`, `printer_upload`, `calibrate` |
| Any UI | `ui_windows`, `ui_tree`, `ui_find`, `ui_click`, `ui_set_value`, `ui_close`, `ui_menu_list`, `ui_menu_invoke`, `ui_input` |
| Design / CAM | see `describe_tools` (`describe_scene`, `sketch_*`, `extrude`, `fillet`... `cam_*`) |

Conventions: indices (object, part, instance, plate, filament slot) are 0-based and describe the
current state, so they are re-read after anything that adds or removes items; filament numbers in
object/part settings are 1-based as in the object list (0 = inherit). Lengths are mm, angles
degrees, setting values use the config's own serialisation (`config_get` shows them).

Settings go through the same path as typing in a settings tab: `config_set` loads the values into
the owning tab (marking the preset modified and re-running the option toggles) and then into the
plater; the preset stays modified until `preset_save` or `preset_discard`, exactly as for a user.
Filament settings apply to the filament preset shown in the Filament tab.

The `screenshot` of the live view renders a frame and reads it back before the buffers are
swapped (`GLCanvas3D::render_and_capture`), since the front buffer is undefined wherever another
window covers it.

Two `Plater` additions exist for this surface: `export_gcode_to()` (the G-code export without its
file dialog) and `last_slicing_error()` (how the last run of each plate ended, which the GUI only
shows as a notification).

### What the dedicated tools do not cover

The ui_* tools keep everything else in reach. Painting (supports, seams, colour, fuzzy skin),
text/SVG emboss, measurement and the other gizmos live in the 3D view's ImGui overlay; they are
reached with `ui_input` (real OS mouse/keyboard events aimed at the canvas) and observed with
`screenshot`. `ui_input` uses `wxUIActionSimulator`, which needs a display server that accepts
synthetic input (X11, Windows, macOS with accessibility permission; not Wayland). Bambu-network
device control (the Device tab) has no dedicated tools.

## Adding a tool

Write `json fn(const json& params)` in the module of its area and register it in that module's
`register_*_tools()` with a summary and parameter descriptors (`param`, `param_enum`). Read
parameters with `arg<T>` / `req<T>`, which turn a missing or mistyped parameter into a `-32602`
error naming it; throw `ToolError` for anything the caller should see. Call the same entry point
the GUI's control calls, so the UI and undo history stay consistent. Flag it `ModalSafe` only if
it merely reads state or drives the UI, `Waitable` if it starts background work, `Long` if it can
legitimately take more than a minute.

# CAM Front End (CAM tab): High Level Design

## Why it exists

The CAM tab turns the solids of the CAD workspace into G-code for a CNC mill or router, with the
Fusion 360 workflow made simpler for beginners: one Setup (machine, material, stock, work origin),
then operations, then Simulate, then Post Process. It is the third tab of the CAD workspace
(Sketch, Modeling, CAM) and reuses its parts: the same `DesignPanel`, viewport, camera, CommandManager
ribbon, left pane and PropertyManager. It adds no machining logic of its own. Every toolpath, feed,
stock simulation and G-code line comes from the `Slic3r::CAM` kernel (`src/libslic3r/CAM/`, see its
README.md); the front end edits the `CamDocument`, builds the kernel's `CamModel` input from the CAD
bodies, and draws what the kernel returns.

## Layout

```
+--------------------------------------------------------------------------------------------+
| [New/Undo/Redo] | CAM ribbon page (scrolls)                                  | Post Process |
+----------------------+---------------------------------------------------------------------+
| [tree] [PM] icons    | Simulation bar (only while simulating): ▶ ⏮ speed [slider] time info |
| wxSimplebook:        |                                                                     |
|   CAM tree           |       DesignCanvas: bodies, stock, WCS triad, toolpaths, tool       |
|   | CAM PropertyManager (Setup page or Operation page)                                     |
+----------------------+---------------------------------------------------------------------+
| status line                                                                                |
+--------------------------------------------------------------------------------------------+
```

## The parts

| File | Role |
|---|---|
| `src/slic3r/GUI/CAD/CamController.hpp/.cpp` | Everything CAM in the GUI: ribbon page, tree, PropertyManager pages, dialogs, viewport layer, simulation, MCP verbs |
| `src/slic3r/GUI/CAD/DesignPanel.*` | Owns `CAM::CamDocument m_cam` next to `m_doc`, and the `CamController`; routes tab, picks, keys and recipe sync to it |
| `src/slic3r/GUI/CAD/CadTabPage.*`, `MainFrame.*` | `WorkspaceTab::Cam`, the top-bar tab `TAB_ID_CAM "cam"` after Modeling |
| `src/slic3r/GUI/CAD/CadRibbon.*` | A third ribbon page, `CadRibbon::Page::Cam` |
| `src/slic3r/GUI/CAD/CadPropertyManager.*` | `is_expanded()` and `set_on_group_toggled()`, so a page can show only some rows of a group |
| `src/slic3r/GUI/CAD/DesignSketchTool.*` | `on_render_overlay` / `overlay_on`: the hook the CAM layer draws through |
| `resources/images/sw_cam_*.svg`, `tab_cam_active.svg` | Ribbon, tree and tab icons |

`CamController` is a pimpl whose `Impl` is a friend of `DesignPanel`, so it reads the CAD document,
the viewport, the status line and the left book directly instead of widening `DesignPanel`'s API.

## Document ownership and persistence

`DesignPanel::m_cam` is the project's CAM recipe. It is saved as `Model::cam_recipe`
(`Metadata/shashimi_cam.bin` in the 3MF) exactly the way `cad_recipe` is: `Impl::sync_recipe()` runs
after every CAM edit, writes `CamDocument::serialize()` into the Model (an empty document clears it)
and calls `put_other_changes()` so the project reads as modified. `on_tab_shown()` loads the blob
when the CAM document is empty, and `clear_document()` (New/Open Project) clears it.

`CamDocument::paths` (the generated toolpaths) is a cache and is never saved: a loaded project shows
every operation as needing regeneration. `Impl::stale` is a GUI-side flag per operation, kept parallel
to `operations` by every add, remove, move and duplicate the front end performs.

### The CAM model and staleness

The generators read a `CamModel`, not the CAD document. `Impl::ensure_model()` builds it lazily with
`build_cam_model()` from the bodies' B-reps (0.02 mm tessellation), adds every enabled sketch with
`add_cam_sketch()`, and fills the setup frames with `update_setup_frames()`. It is rebuilt when
`CadDocument::topo_generation` changes. `DesignPanel::sync_recipe_to_model()` is the one place every
CAD change passes through (recompute, tree edits, MCP), so it calls `CamController::on_cad_changed()`,
which marks the model dirty and every operation stale (the tree shows "⚠ regenerate"). Picked face and
edge ids are only valid for the topology they were picked on, which is why a CAD change makes
toolpaths stale instead of silently regenerating them.

## Tab switching

`set_workspace_tab(Cam)` switches the ribbon to the CAM page, closes a live sketch or CAD command, and
calls `on_tab_entered()`: the overlay is switched on, the model rebuilt if needed, the left pane shows
the CAM tree, and the status line says what to do next. Once the page is on screen the bodies are fed
to the viewport again, because bodies fed while the workspace was hidden (a project load, scripted
modelling) can still show an older tessellation. Leaving closes the simulation and any open CAM
page and switches the overlay off.

`set_ui_mode()` asks MainFrame for another tab only when that mode belongs to one: entering a sketch
asks for Sketch, and leaving a sketch asks for Modeling only when the current tab is Sketch. Many paths
re-assert Feature mode (entering CAM among them), and none of them may pull the user off the CAM tab.

## CAM ribbon

| Button | Action |
|---|---|
| Setup (large) | `open_setup(-1)`: a new setup's PropertyManager |
| Face, Adaptive, Pocket, Contour, Slot, Drill, Bore, Chamfer | `open_op(-1, type)` |
| Engrave ▾ (Engrave, Trace) | `open_op(-1, Engrave / Trace)` |
| 3D Adaptive, 3D Parallel, 3D Contour | `open_op(-1, Adaptive3D / Parallel3D / Contour3D)` |
| 4-Axis ▾ (Rotary Wrap, Rotary Finish) | `open_op(-1, RotaryWrap / RotaryFinish)` |
| Tool Library | `tool_library_dialog()` |
| Regenerate All | `regenerate_all()` |
| Simulate | `sim_open()` |
| Post Process (large, tail) | `post_dialog()` |

Operation and Setup buttons are greyed until the CAD document has a body; Regenerate, Simulate and
Post until there is an operation (`refresh_ribbon_gates()`).

## CAM tree

```
CAM
  Setup1 (Generic 3-axis · Aluminum)   ⏱ 4:30
    [T3] Adaptive1   ⏱ 2:15             op icon + status dot: green ok, amber stale, red error, grey suppressed
    [T5] Contour1    ⚠ regenerate
    Tools
      T3 — 6 mm flat end mill
      T5 — 3 mm flat end mill
```

A plain `wxTreeCtrl` on page 2 of the left book, rebuilt from the document by `rebuild_tree()`. The op
icon is the ribbon icon with a status dot painted into its corner. Selecting an operation shows its
time, lengths and first warning (or its error) on the status line and draws its toolpath in full colour
with the others faded. Double-click edits (operation, setup, or the Tool Library from a tool row). The
context menu opens on the right-button release, as in the FeatureManager:

| Row | Menu |
|---|---|
| Operation | Edit, Suppress/Unsuppress, Regenerate, Duplicate, Move Up/Down (within its setup), Simulate, Post Process (selected), Delete |
| Setup | Edit Setup, Regenerate Setup, Simulate, Post Process, Delete Setup (asks; removes its operations) |
| CAM | New Setup, Regenerate All, Post Process All |
| Tools | Tool Library |

Keys while the CAM tab has focus: Esc closes the simulation, else the open page, else clears the
selection; Delete deletes the selected operation; Space plays/pauses the simulation. Every other key
goes to the focused control (the CAD shortcuts do not apply here).

## PropertyManager pages

A second `CadPropertyManager` on page 3 of the left book, with two pages built once: Setup and
Operation. Rows are small panels (`make_row`: label + controls). Each row has a visibility rule; after
any change and after a group opens (`set_on_group_toggled`) the page shows exactly the rows whose rule
holds and whose group is open, because opening a group shows everything in it. ✓ confirms, ✗ cancels,
and the Message box repeats the status line (errors in red).

### Setup

| Group | Controls |
|---|---|
| Machine and Material | Machine (`load_machines(resources_dir())`), Material (13 materials) |
| Stock | Shape: Box around the model (six side offsets), Cylinder along X (radius, length, 0 = from the model; extra radius), Custom box (min/max) |
| Work Coordinate System | Origin on stock/model box; the nine box points as a 3×3 grid of radio buttons seen from above, plus Top/Bottom; or Custom point with X/Y/Z and a Pick toggle (a face centroid or a round edge's centre); work offset G54–G59 |
| Model | All bodies, or a checklist of bodies |
| 4th Axis | A index angle, shown only for a machine with an A axis |

✓ stores the setup, refreshes the setup frames and marks that setup's operations stale.

### Operation

One page for every `OpType`; the rows shown depend on the type.

| Group | Controls |
|---|---|
| Tool | Tool (the document's tools, then the library's; "T3  6 mm flat end mill"), Library…; Feeds & speeds from the material (auto) with RPM / cutting / plunge / ramp feed, editable when auto is off; a line with the material, chip load and flutes. Auto values come from `effective_feeds()` and follow tool changes. For a new operation, choosing a tool rescales the operation's defaults with `default_operation(type, tool)` |
| Geometry | What to pick for this type; Pick in viewport (on by default) and Clear; the list of picked faces, edges, sketch and points (Delete removes the highlighted one); Whole model (3D, rotary finish); Holes: auto-detect with a diameter range (Drill, Bore) |
| Heights | Clearance, Retract, Top, Bottom: each a reference (stock/model/selection top/bottom, absolute) plus an offset |
| Passes | stepover, stepdown, stock to leave (walls, floors), climb, side, finishing passes and stepover, drill cycle / peck / dwell / break-through, chamfer width and tip offset, pass direction and boundary (parallel), wrapped operation and wrap radius, A stepover and spiral (rotary), tolerance |
| Linking | entry (helix, ramp, plunge), ramp angle, helix diameter, lead-in radius |
| Advanced (collapsed) | rest machining, ordering, optimal load, lift height, minimum stepdown, helix angle |

While Pick is on, a face click adds the face and an edge click adds the edge; clicking the same one
again removes it (the viewport's re-pick escalation to a whole body is switched off meanwhile). A click
on a committed sketch sets or clears the sketch. The drill diameter filter is resolved on ✓: the holes
`holes_for_op()` finds in range become the selection's faces.

✓ copies the chosen tool into the document if it is not there, stores the operation (a new one goes in
with `add_operation`) and generates its toolpath with `generate_toolpath()` on a worker thread. An error
keeps the page open with the kernel's sentence in red ("Tool 6 mm cannot enter this 5 mm slot"); success
closes it and reports the time and cutting length, plus the first warning.

### Beginner defaults

The first operation created with no setup makes Setup1 from the model: the `CamSetup` defaults, which
are a box stock 1 mm larger on the sides and top and flush below, and the WCS at the top front left
corner of the stock, on the first machine of the list. The new operation takes the first tool that
suits its type (a drill for Drill, a chamfer mill for Chamfer, a V-bit for Engrave, a ball mill for 3D
Parallel and Rotary Finish, else a flat end mill), `default_operation()` tuned for it, material feeds on,
and the status line says what to pick.

## Tool Library

A modal dialog: the list of tools on the left with Add, Duplicate, Delete and Restore defaults; the
fields of the selected tool on the right (T number, name, type, tool material, diameter, corner radius,
tip angle, tip diameter, flutes, flute length, overall length, shank diameter, thread pitch). OK saves
with `save_tool_library()` to `data_dir()/cam_tools.json`. The library is loaded when the panel is
built, with `default_tools()` when the file is missing or unreadable. Tools used by operations are
copied into the document, so a project keeps working on a machine with a different library.

## Viewport layer

`DesignSketchTool::render()` calls `on_render_overlay` while `overlay_on` is set (the CAM tab), inside
the canvas's own frame. Everything is drawn in the setup frame with the model matrix
`CamSetupFrame::to_setup.inverse()`:

- the current setup's stock as translucent faces (depth-tested, no depth write) and edges, and the
  WCS triad (X red, Y green, Z blue) at the setup origin;
- the current setup's toolpaths (the setup of the selected row, else the first) as `GLModel` line
  batches, one per move class and operation, rebuilt only when paths change: rapid and retract yellow, cutting blue, lead-in/out green, plunge and ramp red. Arcs and A
  moves are sampled; a move at A is drawn at `Rx(-A)·p`, where it touches the unrotated part. The
  selected operation is drawn in full colour, the others at 40 % alpha;
- gouge and other positioned warnings (`Toolpath::warnings` with a move index) as red crosses;
- while simulating, the simulated stock mesh and the tool (flute length in orange, ball end as a
  sphere, shank in grey) at the current position.

Toolpaths are drawn without depth test, like the sketch overlay, so a path under a surface stays
visible.

## Simulation

Simulate plays the selected operation, or every enabled operation of the current setup. Stale
operations are regenerated first. The moves become one timeline: each move lasts its length over its
feed (rapids at the machine's rapid feed). The bar over the viewport has Play/Pause, back to start,
speed (1× to 100× real time), a scrub slider, the time, and the current move (operation, tool, feed or
"rapid", X Y Z and A on a 4-axis machine). A 33 ms timer advances the time; about ten times a second
the moves up to the current one are cut into a `StockSim` (box, or cylinder for cylinder stock) and
its `to_mesh()` replaces the stock drawing. Scrubbing backwards restarts the stock from fresh. When
the stock mesh is shown the CAD bodies are hidden (the stock is the part now); closing the simulation
shows them again. If the stock simulation fails the tool still animates over the paths.

## Post Process

A modal dialog: machine and post dialect (defaults from the machine), program name and number,
units, arcs as G2/G3 or lines, inverse-time A feed (enabled for LinuxCNC and Fanuc), which operations
(selected operation, selected setup, all), and the output file with a Browse… that offers the dialect's
extension (`.nc`, `.ngc`, `.gcode`). Show G-code fills a read-only preview; Post writes the file.
Both regenerate stale operations first, call `post_process()`, and report the operation count, tools,
lines and estimated time, which the status line repeats.

## Scripted control

`McpControl` forwards every `cam_*` method to `CamController::mcp()`, which drives the same handlers
the ribbon and the tree do: `cam_tab`, `cam_describe`, `cam_new_setup`, `cam_edit_setup`,
`cam_add_op` (type, tool or tool_diameter, faces, edges, sketch, whole_model, stepover, stepdown,
confirm), `cam_pm_ok`, `cam_pm_cancel`, `cam_select`, `cam_regenerate`, `cam_simulate` (t, play,
close), `cam_post` (dialect, path, scope; opens the dialog after the reply), `cam_tool_library`.

## Constraints

- The front end never computes machining: no toolpath, feed or G-code logic outside the kernel.
- The CAD workspace behaves as before on the Sketch and Modeling tabs; the CAM layer draws only while
  the CAM tab is shown.
- Like the rest of the workspace, the CAM tab is English-only (`_L` is a UTF-8 literal there).

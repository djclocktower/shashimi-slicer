# Laser Front End (Laser tab): High Level Design

## Why it exists

The Laser tab gives desktop diode and CO2 lasers the LightBurn workflow inside the slicer: a 2D
workspace showing the laser bed from above, vector shapes, text and images placed on 30 colour
layers that carry the cut settings, a preview with a time estimate, and a device connection that
frames and streams the job to a GRBL-family controller. All geometry, planning and G-code come from
the `Slic3r::Laser` kernel (`src/libslic3r/Laser/`, see its README.md). The front end edits a
`LaserDocument`, draws it, and hands the kernel's `LaserJob` / G-code to the streamer.

## Layout

```
+-------------------------------------------------------------------------------------------+
| Import Open Save | Undo Redo | Zoom | Preview Frame Start Pause Stop Home | X Y W H Rot Lock |
+----+--------------------------------------------------------------+-----------------------+
|tool| Start-here hint bar (empty workspace / not connected)         | Cuts/Layers | Laser | |
|strip| ruler                                                       | Move | Console |      |
|    | ruler  LaserCanvas (bed, grid, shapes, images, selection)     | Shape Properties |    |
|    |                                                               | Library               |
|    | colour strip: 30 layer swatches                               |                       |
+----+--------------------------------------------------------------+-----------------------+
| status line (cursor position, last action, errors)                                          |
+-------------------------------------------------------------------------------------------+
```

## The parts

- **Hosting.** `MainFrame` inserts a `LazyPage<LaserPanel>` (`TAB_ID_LASER`) right after CAM. The
  panel is built the first time the tab is shown and never prebuilt. It is independent of the CAD
  workspace: it owns its document, canvas and device connection.
- **`LaserPanel`** (`src/slic3r/GUI/Laser/LaserPanel.*`) owns the `LaserDocument`, the selection
  (top-level shape indices; a group stands for its members), the current layer, undo, the device and
  material libraries, and the `GrblStreamer`. Every edit ends in `commit()`: an undo snapshot, the
  `Model::laser_recipe` sync (`put_other_changes()` marks the project dirty, the 3MF carries it as
  `Metadata/shashimi_laser.bin`), and a refresh of the canvas, Cuts table, toolbar fields and
  properties. On tab activation the panel compares `Model::laser_recipe` with the blob it last
  wrote or read, and reloads when another part of the app (project load, new project) changed it.
- **`LaserCanvas`** (`LaserCanvas.*`) is a `wxGLCanvas` created with
  `OpenGLManager::create_wxglcanvas` on the app's shared context. It has its own orthographic 2D
  renderer (not `GLCanvas3D`): `GLModel` line and triangle batches drawn with the `flat` shader,
  images as `GLTexture` quads with `flat_texture`. The rulers are two plain wx windows painted with
  a DC from the same view transform, so no text rendering is needed in GL. The canvas caches
  `LaserDocument::flatten()` per shape (hit tests, selection box) and one line batch per layer, and
  rebuilds them after every document change. Fill layers get a light 45° preview hatch from
  `hatch_fill`; images show their adjusted grayscale.
- **Tools.** Select (click, Shift/Ctrl add, rubber band: left-to-right selects enclosed shapes,
  right-to-left touching ones; drag to move with snapping to grid, bed edges and other shapes;
  8 scale handles, Shift keeps the aspect; rotate handle, Shift snaps 15°), Node Edit (Path shapes:
  drag vertices, double-click an edge to insert, Delete to remove), Line/Polyline, Rectangle,
  Ellipse, Polygon, Text. Drags transform from the xforms saved at mouse-down, so a drag never
  accumulates rounding. Wheel zooms to the cursor, middle/right drag pans, right-click without a
  drag opens the context menu.
- **Commands** (left strip, context menu, keys) go straight to the kernel: `offset_paths`,
  `boolean_op`, `weld`, grid and circular arrays (`LaserDocument::duplicate` +
  `transform`), align, group/ungroup, mirror, rotate 90°, convert to path, and Trace Image
  (`trace_image`, with a live preview on a downscaled copy).
- **Right dock.** *Cuts / Layers* lists the layers in use (colour, name, mode, speed/power, Output,
  Show); double-click opens the Cut Settings Editor (Common / Advanced / Image, with a live dither
  thumbnail from `dither()`). *Laser* holds the device, port and connection, state and position,
  progress, Start/Pause/Stop, Frame, Home, Start From, the 9-point job origin, rotary and the
  selection options. *Move* jogs (directions mapped through `to_machine()` for the device's origin
  corner), sets the origin, goes to a point and fires the laser at low power. *Console* shows the
  streamer's log and sends lines and four user macros. *Shape Properties* binds form rows directly
  to the selected shape's fields; edits redraw at once and are committed as one undo step once
  typing pauses (600 ms). *Library* shows the material library (material → thickness → entries)
  and assigns an entry to the current layer.
- **Dialogs** (`LaserDialogs.*`) are built with `LaserForm`, a label/control grid whose controls
  write their bound field on every change. Preview rasterises the planned job into an RGB buffer
  (cuts in layer colour, scans darker with power, travel dotted red) so the playback slider can
  redraw any prefix of a large raster job quickly.

## Jobs

Preview, Save G-code, Frame and Start all run `Laser::plan()` on a worker thread (a progress dialog
with Cancel appears when planning takes longer than 300 ms). Save writes `gcode()`. Frame streams
`frame_gcode()` when a laser is connected and otherwise shows it. Start asks for confirmation with
the device, the burning layers, the estimated time and plain-language warnings (job past the bed,
100 % fills on a diode, controller not Idle), then streams the G-code lines with the job's time
estimate for the ETA.

## Device streaming

`GrblStreamer` (`src/slic3r/Utils/GrblStreamer.*`, no wx) owns a worker thread and an
`ITransport` (`SerialTransport` over `Utils::Serial`, or `GrblSimulator`). The worker:

- waits for the controller's banner (soft reset if none arrives in 3 s), then asks `$I`;
- streams by character counting: lines go out while the bytes of unacknowledged lines stay within
  127 (GRBL's 128-byte RX buffer); each `ok` / `error:N` retires the oldest line. Marlin and
  Smoothie get one line at a time;
- polls `?` every 200 ms (Marlin: `M114` every second when idle) and parses the `<...>` reports;
- turns `error:N` during a job into a feed hold and a paused job (resume skips the line), and
  `ALARM:N` into a failed job; both reach the GUI decoded to plain language;
- stops with feed hold, a soft reset once the hold has stopped the motion (so the position is
  kept), then `$X`.

Callbacks run on the worker thread; `LaserPanel` marshals each with `CallAfter`. The console keeps
the last 2000 lines; job lines and status polls are not logged. The port list offers
"Simulator (no hardware)", a fake GRBL 1.1 controller behind `ITransport`, so the whole connect /
frame / start / console flow works without a laser. The same simulator drives the streamer tests
(`tests/slic3rutils/test_grbl_streamer.cpp`).

## Persistence

- The document: `Model::laser_recipe` in the 3MF, and `.slaser` files (Open / Save laser project).
- Device profiles and the material library: `laser_devices.json` and `laser_materials.json` in the
  data dir through the kernel's Materials API.
- UI preferences (selected device, last port, default font, console macros): `laser_ui.json` in
  the data dir.

## Constraints

- Undo keeps whole-document snapshots (100 steps, 256 MB cap), which is simple and exact but grows
  with embedded photos.
- Text fonts are stored as the family name picked from the system font list, so a project moves
  between machines; the kernel's `resolve_font_path()` finds the file (and a real bold / italic
  face when there is one) and falls back to the bundled font. The outlines are cached in the
  document, so a project still shows its text on a machine without the font.
- Network transports (grblHAL telnet, FluidNC/ESP3D WebSocket) fit behind `ITransport` but are not
  implemented.

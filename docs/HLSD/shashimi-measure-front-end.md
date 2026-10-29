# Measure Front End (Measure tab): High Level Design

## Why it exists

A Bambu printer already carries a small measuring system: a chamber camera that sees the plate,
and a nozzle whose contact the bed's force sensing detects. The Measure tab turns the two into a
multisensor CMM in the style of an image-dimension measuring system (Keyence IM and similar):
camera inspection with OK/NG judgement, calibration on a printed dot mat, outline tracing, and
reverse engineering of 2.5D parts (outline from the camera, heights from nozzle touches) into a
printable model. All measurement comes from the `Slic3r::Metrology` kernel
(`src/libslic3r/Metrology/`, see its README.md); the tab drives it and draws its results.

The printer half is unverified: the tab writes G-code and reads saved camera frames, and a
built-in simulator stands in for the camera and the probe, so every step can be tried and tested
without hardware.

## Layout

```
+------------------------------------------------------------------------------------------+
| MEASURE  [Inspect][Calibrate][Trace][Reverse Engineer][Probe]  Source Scene  Capture ... | OK |
+--------+-------------------------------------------------------------+-------------------+
| Select |                                                             | mode page         |
| Line   |   MetrologyView: the frame on black, zoom/pan, overlays,    | (settings, lists, |
| Circle |   scale bar                                                 |  actions)         |
| Width  |                                                             |                   |
| Box    |                                                             |                   |
| 2-Point|                                                             |                   |
+--------+-------------------------------------------------------------+-------------------+
| cursor px / mm / grey | [calibration state] | last message                               |
+------------------------------------------------------------------------------------------+
```

## The parts

- **Hosting.** `MainFrame` adds a `LazyPage<MetrologyPanel>` (`TAB_ID_MEASURE`) after Calibration,
  never prebuilt. It does not depend on the printer type or the CAD workspace.
- **`MetrologyView`** paints the frame with `wxGraphicsContext` (no OpenGL): nearest-neighbour
  pixels when zoomed in, a scale bar from the calibration, and a drag preview for the active tool.
  Wheel zooms at the cursor, right or middle drag pans, double-click fits. The panel draws every
  overlay through `on_overlay` in window coordinates.
- **`MetrologyPanel`** holds the frame, the camera model, the inspection program, the trace and the
  reverse-engineering session. Plate-frame geometry is drawn through `CameraModel::to_px(mm, z)`,
  so outlines and levels sit on the image at their own height.
- **Sources.** *Simulator*: `sim_render()` of a preset scene (bracket, test coupon, mat only) with a
  P1P-like 720p camera, and `sim_touch()` for probe results. *Image files*: saved camera frames.
  With *Light pair* on, a light-off frame is subtracted to remove room light.
- **Modes.**
  - *Inspect*: Line, Circle and Width tools create `Feature`s from a drag; Distance and Angle combine
    two selected features. The `Program` re-runs on each frame; tolerances give OK/NG per row and
    the overall judgement in the top bar. Results export as CSV. Uncalibrated, results are in pixels.
  - *Calibrate*: mat detection (`calibrate_mat`), raised-bed frames and `calibrate_camera_center`
    for parallax and the probe readout, two-point scale as a mat-less fallback. Saved in
    `metrology.json` in the data dir.
  - *Trace*: the Box tool bounds `trace_silhouette()`; the outline exports as SVG or extrudes onto
    the plate.
  - *Reverse Engineer*: a `ReSession` over the traced silhouette. Each batch of touches runs in the
    simulator, or is saved as G-code (with a CSV of the touches) and its heights come back from a
    CSV or from a folder of camera frames (one hold per touch, the first on the mat). *Build model*
    gives levels; *Send to plate* adds `model_mesh()` (one closed stepped shell) through
    `ObjectList::load_mesh_object`, and STL/SVG export is available.
  - *Probe*: the mat's placement on the printer, touch timing, the repeatability-test G-code, frame
    analysis (`find_stills`, `plane_offset`, `repeat_stats`) and the error budget.

## Constraints

- The camera feed is not captured live: Bambu's stream is reached only through the closed
  networking plugin, so frames come from files for now.
- Probing relies on `G380` stopping the bed at contact and the bed holding there during the dwell;
  both are unverified on hardware and every generated file says so.
- Reverse engineering is 2.5D: vertical walls and flat levels. Undercuts, side-wall features and
  freeform surfaces are out of reach of both sensors.
- Rendering in the simulator and the reverse-engineering build run on the UI thread with a busy
  cursor; both take about a second at the default resolutions.

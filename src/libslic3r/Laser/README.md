# Shashimi Laser kernel (`Slic3r::Laser`)

Headless: no wx, no GUI, no OCCT. Built only with `SLIC3R_CAD` (the same switch as CAD and CAM).
Design decisions live in the Laser brief. The headers are the contract between the three owners
below. If a signature has to change, change the header in its own edit and tell the other two owners.

## Ownership

| Owner | Files |
|---|---|
| A | LaserDocument.cpp, VectorOps.cpp, LaserPlan.cpp, LaserGCode.cpp, Import.cpp, Materials.cpp, TextLayout.cpp, Laser.cpp |
| B | ImageEngrave.cpp, Trace.cpp |
| C (GUI) | `src/slic3r/Utils/GrblStreamer.hpp/.cpp` and everything in `src/slic3r/GUI/Laser/` |

`LaserTypes.hpp` is shared (header-only), so changes to it need agreement from all three owners.

## Conventions

- **Units.** The API uses mm, mm/s for speeds (LightBurn convention; the G-code writer emits F in
  mm/min), power in % (0 to 100, `S = power * s_max / 100`), degrees and seconds. `Polyline`,
  `Polygon` and `ExPolygon` coordinates are libslic3r *scaled* mm (`scale_()` / `unscale()`), so
  ClipperUtils works on them directly. Every `double` and `Vec2d` is plain mm.
- **Frames.**
  - Workspace: the bed seen from above, X right, Y up (towards the rear), origin at the front-left
    corner, as in LightBurn. Every document coordinate and every `LaserJob` coordinate uses it
    (shifted to the job-origin anchor for UserOrigin / CurrentPosition jobs).
  - Machine: `LaserDevice::origin_corner` says where the controller's zero is. Only the G-code
    writer converts (`to_machine()` / `from_machine()` in LaserGCode.hpp); the GUI uses the same
    helpers for jog readouts.
  - Shape local frame: `LaserShape::xform` maps it to the workspace and is absolute, not relative
    to the parent group. `LaserDocument::transform()` moves a group by transforming every member.
- **Layers.** 30 colour layers, index = colour (C00 to C29, `layer_color()`). Image shapes are
  always rastered with the layer's `image_*` settings; `LaserLayer::mode` applies to vectors.
- **Groups.** A member stores its group's index in `LaserShape::parent` (-1 = top level); groups
  keep no child list. Only the `LaserDocument` helpers add or remove shapes, so indices stay valid.
- **Errors.** Plain-language sentences for the user in `LaserJob::error` / `warnings`,
  `ImportResult::error` / `warnings`, or the `std::string* error` out-parameter. The kernel never
  throws out of its API.
- **Progress.** `ProgressFn` gets the fraction done; returning true cancels, and the result then
  carries the error "Cancelled".
- **Serialization.** cereal, append-only.
  - `LaserShape`, `LaserLayer` and `JobSettings` are each length-framed in the document blob, the
    same way as `sketch_dimensions_encode`. To add a field, append it to the end of the record's
    `serialize()` list. Never reorder fields.
  - `LaserPath` and the transform's 6 coefficients are serialized inline and are frozen.
    `JobSettings::Optimize` fields are listed in `JobSettings::serialize()`, so new ones go at the
    end of that list.
  - Enums are serialized as integers: only append values.
  - `LaserDocument::warnings`, `LaserJob` and everything generated are never serialized.
- **Device side.** `GrblStreamer` has no wx. Its callbacks run on the streamer's own thread and
  the GUI marshals them with `CallAfter`. Device profiles and the material library are JSON files
  in the app data dir (Materials.hpp).

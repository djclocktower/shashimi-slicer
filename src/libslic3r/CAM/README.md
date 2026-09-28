# Shashimi CAM kernel (`Slic3r::CAM`)

Headless: no wx, no GUI. Built only with `SLIC3R_CAD`. Design decisions live in the CAM brief. The headers are
the contract between the three owners below. If a signature has to change, change the header in its own
edit and tell the other two owners.

## Ownership

| Owner | Files |
|---|---|
| A | CamDocument.cpp, CamGeometry.cpp, ToolLibrary.cpp, FeedsSpeeds.cpp, Machines.cpp, Op2D.cpp, Drill.cpp, Post.cpp, Toolpath.cpp, CAM.cpp |
| B | Adaptive.cpp (port of FreeCAD `libarea/Adaptive.cpp`) |
| C | Op3D.cpp, Rotary.cpp, StockSim.cpp |

`CamTypes.hpp` is shared, so changes to it need agreement from all three owners.

## Conventions

- **Units.** The API uses mm, mm/min, RPM, degrees and seconds. `ExPolygons`, `Polylines` and `Point` use
  libslic3r *scaled* coordinates (`scale_()` / `unscale()`). Every `double` and `Vec3d` is plain mm.
- **Frames.**
  - CAD world: `CamModel` bodies, meshes and sketches.
  - Part frame: world rotated by `CamSetup::a_index_deg` about the index axis. Stock is defined in this frame.
  - Setup frame (X right, Y back, Z up, origin at the WCS): every toolpath, `Region2D`, `HeightMap`,
    `StockSim` and `HoleFeature` from `holes_for_op`.
  - `CamModel::setups[i].to_setup` maps world to setup and already includes the index rotation.
- **A axis.** A rotates about +X (right-hand rule). In a rotary setup the A axis is the setup-frame X axis
  (Y = Z = 0), and a Cylinder stock's WCS is always on that axis. `Move::a_deg` is absolute.
- **Toolpaths.**
  - A `Move` goes from the end of the previous move to `to`.
  - Generators start with a Rapid at clearance, link with `append_link_move`, and finish with
    `append_retract`.
  - Rapids never pass through stock.
  - Arcs lie in the XY plane. Test for an arc with `is_arc()` / `arc_dir()`, not with `Kind`, because lead
    and ramp moves can also be arcs.
- **Errors.** Errors and warnings are plain-language sentences for the user, stored in `Toolpath::error` /
  `Toolpath::warnings`. Generators never throw. OCCT's `Standard_Failure` is caught inside CamGeometry.cpp.
- **Serialization.** cereal, append-only.
  - `CamTool`, `CamSetup` and `CamOperation` are each length-framed in the document blob, the same way as
    `sketch_dimensions_encode`. To add a field, append it to the end of the record's `serialize()` list.
    Never reorder fields.
  - Nested value structs (`FeedsSpeeds`, `Stock`, `WcsOrigin`, `FaceRef`, `EdgeRef`, `GeometrySelection`,
    `Height`, `Heights`) are frozen. A new field for one of them goes at the end of the record that owns it.
  - Enums are serialized as integers: only append values.
  - `CamDocument::paths` and `warnings` are never serialized.
- **OCCT.** Only `CamGeometry.hpp` may include OCCT headers. Other headers see `CamBodyShape` as opaque.

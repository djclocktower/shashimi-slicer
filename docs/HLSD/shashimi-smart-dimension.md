# Smart Dimension: High Level Design

## Why it exists

A parametric sketch is only as useful as the numbers that drive it. The Design tab's sketcher
solves entity constraints with the vendored SolveSpace solver; Smart Dimension is the one tool
that turns "this line should be 40 mm" into such a constraint the way SolidWorks users expect:
pick geometry, watch the dimension follow the cursor, click to place it, type the value. The
dimension stays on the sketch as an annotation, is saved with the sketch feature, and can be
moved, edited, made driven or deleted later.

## The parts

| Layer | Files | Role |
|---|---|---|
| Kernel (headless, unit-tested) | `src/libslic3r/CAD/SketchDimension.hpp/.cpp` | Data model, pick resolver, measurement, constraint generation, render layout, text, re-indexing |
| Kernel | `src/libslic3r/CAD/SketchSolver.hpp/.cpp` | Solver mapping; per-entity "fully defined" analysis |
| Kernel | `src/libslic3r/CAD/CadDocument.hpp/.cpp` | `CadFeature::dimensions` persistence |
| GUI | `src/slic3r/GUI/CAD/DesignSketchTool.*` | `Mode::Dimension` interaction, rendering, colours |
| GUI | `src/slic3r/GUI/CAD/DesignCanvas.*` | Session hand-over to and from the document |
| GUI | `src/slic3r/GUI/CAD/SketchInlineEditor.*` | The Modify box (in-canvas value field) |

## Data model

`SketchDimension` (kernel) references geometry the way `SketchEntityConstraintDef` does — entity
index plus `SketchPointRole`, with the negative implicit references `kSketchRefOrigin`,
`kSketchRefAxisX`, `kSketchRefAxisY` — so it follows every solve.

| Field | Meaning |
|---|---|
| `kind` | `Length` (aligned, one line), `Horizontal`, `Vertical`, `Diameter`, `Radius`, `Angle`, `PointPoint` (aligned), `PointLine`, `LineLine` (parallel distance). Serialized as int, append-only. |
| `ea, ra, eb, rb` | References; per kind: a line (`Length`), two points (`Horizontal`/`Vertical`/`PointPoint`), a circle or arc (`Diameter`/`Radius`), two lines (`Angle`/`LineLine`), a point and a line (`PointLine`). |
| `text_pos` | Text centre in sketch-plane coordinates. The whole layout is derived from it. |
| `driven` | Reference dimension: measures, owns no constraint. |
| `constraint` | Index of the driving constraint in the sketch's constraint list, -1 when driven. |
| `value` | Display units: mm, degrees for angles. For a driving dimension the target, for a driven one the last measurement. |
| `sector` | Angle: which of the four angles between two crossing lines (bit0 reverses the first line's direction, bit1 the second's). `PointPoint` / `PointLine`: bit0 / bit1 measure from the edge of the first / second reference's circle or arc instead of its centre. |

`CadFeature::dimensions` is the last field of the cereal field list, written as ONE opaque byte
block (a cereal string, so it is length-framed inside the feature):

```
u32 block version | u32 item count | { u32 item byte length | item payload } * count
```

The item payload is `SketchDimension::serialize`, append-only. `sketch_dimensions_decode` consumes
exactly each item's byte length, so a newer build's appended fields are skipped by an older reader,
and an older build's shorter item keeps its read fields and defaults the rest. No dimensions encode
as an empty block. A project written before the field existed ends early and loads with no
dimensions (recipes are length-framed per feature from v5; no version bump). The unframed v4
layout is read inside `CadRecipeV4Scope`, which stops `CadFeature::load` at the fields v4 had —
anything appended after the framing is read only from framed blobs. An Import feature's solid is
restored before the appended fields are read, so a blob ending early still restores it.

### Keeping references valid

The constraint index and the entity references are re-indexed by the same rules the sketch
applies to its constraint list: `sketch_dimensions_remap_entities` (deleting entities drops the
dimensions on them and shifts the rest), `sketch_dimensions_remap_constraints` /
`sketch_dimensions_erase_constraint` (a dimension whose constraint is dropped becomes driven),
`sketch_dimensions_sanitize` (clamps a loaded set to its sketch). `DesignSketchTool` calls them from
`delete_selected`, `remove_constraint`, `drop_orientation_constraints` and
`drop_constraints_referencing`.

### Angle units

Every stored `Angle` constraint value is in RADIANS, between the two lines' `p1 - p0`
directions, which is what `SketchSolver` converts for `SLVS_C_ANGLE`. A dimension's sector angle is
converted by `sketch_angle_constraint_value`: the same number when the sector uses both directions
or both reversed, its supplement when it reverses one. The constraint-button path converts the
panel's degree prefill the same way (`write_plan_value`).

## Selection rules (`resolve_smart_dimension`)

A pick is a whole entity or a point (`SmartDimPick`); a Point entity and the origin are always
points. With the cursor position the resolver returns the dimension and its measured value:

| Picks | Dimension |
|---|---|
| 1 line | Cursor inside the band perpendicular to the line between its endpoints: aligned `Length`. Outside: `Horizontal` when the cursor's offset from the midpoint is mostly vertical (`|dy| >= |dx|`), else `Vertical`. An axis-aligned line always gives the matching `Horizontal`/`Vertical`. |
| 1 circle / 1 arc | `Diameter` / `Radius` |
| line + line | Parallel (within 1e-4 in the sine): `LineLine`. Otherwise `Angle` of the sector the cursor lies in; its value is that sector's interior angle. |
| point + point | Distance with the same aligned / horizontal / vertical rule as a line |
| point + line | `PointLine` perpendicular distance (point stored first) |
| point + circle/arc, circle/arc + circle/arc | Distance to / between centres, point rule |
| line + circle/arc | `PointLine` from the centre |

A circle or arc picked with `SmartDimPick::edge` (Shift+click) is measured to its edge: the
distance is always aligned and equals the centre distance minus the radius (minus both radii edge to
edge).

A lone point, an ellipse or spline picked whole, or two coincident points resolve to nothing.
Horizontal/vertical references are ordered so the signed `DistanceX`/`DistanceY` constraint is
positive for the current geometry: accepting a dimension never flips it.

## Constraints

`sketch_dimension_constraint` maps a dimension and a value to its driving constraint:
`Length` -> `Distance(P0,P1)`; `Horizontal`/`Vertical` -> `DistanceX`/`DistanceY`;
`PointPoint` -> `Distance` (`Coincident` at 0); `Diameter`/`Radius`; `Angle` (radians, see above);
`PointLine` -> `PointOnLine` with the value; `LineLine` -> `PointOnLine` from the first line's P0 to
the second line. An edge distance drives the centre distance (`value` plus the radii as they are
when the constraint is written; re-typing the value re-captures them). `PointOnLine` stores an
unsigned distance; the solver takes the side from the
current geometry, so a point is held where it is instead of being flipped across the line.

## Layout

`layout_sketch_dimension(entities, dim, style)` returns every stroke: extension lines (starting a
gap away from the geometry, overshooting the dimension line), the dimension line (extended to the
text when the text sits outside), filled arrowheads, and the text anchor. Sizes come from
`SketchDimStyle` in plane units; the GUI derives them from pixels at the current zoom, so the
annotation keeps its screen size.

- Linear: the dimension line runs through `text_pos`, parallel to the measured direction. Heads sit
  inside when the span has room (2.5 arrow lengths); otherwise they sit outside pointing in, and
  text placed inside the span is moved past the nearer arrow.
- An edge reference anchors its extension line at the circle's point facing the other reference
  (the tangent point for a line).
- Diameter: a line through the centre toward the text with heads on both rim points pointing out;
  Radius: a leader from the centre with one head on the arc. An arc dimensioned outside its sweep
  gets the arc continued to the head.
- Angle: an arc about the intersection through the text, over the chosen sector (continued to the
  text if it sits outside), heads on the arms, extension lines along arms shorter than the arc.

Text is formatted by `format_sketch_dimension`: up to two decimals with trailing zeros trimmed,
`Ø12` (U+00D8: U+2300 is not in the UI font atlas), `R5`, `30°`, and `(…)` around a driven value.

## Interaction (`DesignSketchTool`, `Mode::Dimension`)

- Hover pre-selects what a click would take — endpoint, centre, Point, origin, else the edge under
  the cursor (a midpoint picks its line), else the sketch X / Y axis through the origin — and draws
  it in the pre-selection orange (an axis as a line across the sketch). The axes are infinite, so a
  placement click within the pick tolerance of one picks the axis. Shift+click on a circle or arc
  picks its edge (minimum distance) instead of its centre.
- The first click holds that pick. From then on a preview dimension follows the cursor, its kind
  and sector re-resolved on every move.
- A click on another entity or point that the rules can pair with the first switches to the
  two-reference dimension; one they cannot pair starts over from the new pick. A point of the held
  entity counts as the same geometry.
- A click on empty space (or on the held pick, or anything once two are held) places the dimension
  with `text_pos` at the click. Placement tries the driving constraint: an identical constraint the
  sketch already has with no dimension on it is adopted; otherwise the constraint is appended and
  solved. If that solve fails (over-defined or redundant) the dimension is kept as driven and the
  status line reads "Dimension made driven: it would over-define the sketch". A driving dimension
  opens the Modify box pre-filled with the measured value: Enter drives the typed value (a value
  the sketch cannot take is refused and the old one kept), Esc keeps the measured value driving.
- Nothing opens a value field after a shape is drawn (the draw-then-edit queue is off,
  `m_autoedit_enabled`): shapes are dimensioned with Smart Dimension. Arming Smart Dimension
  cancels any open field (keep as drawn) and withdraws a stale tool message; Esc cancels an open
  field before it unwinds anything else; starting, finishing or cancelling a session closes it.
- The picks clear and the tool stays armed. Esc drops a held pick first, then leaves the tool;
  right-click drops a held pick.
- On placed dimensions, in Select and Dimension: click selects the text (selection colour, framed),
  drag moves `text_pos`, double-click opens the Modify box, right-click toggles driven/driving,
  Delete removes the dimension and its constraint.
- The `V` path (type the selection's value) produces the same annotation, at a default position
  clear of the geometry chosen inside the zone that gives the selection's dimension.
- Clicking a live characteristic quote (the non-driving labels shown for a selected shape) promotes
  it to a placed dimension through the same placement path.

## Persistence hand-over

`DesignSketchTool::begin_edit(..., dimensions)` loads a feature's dimensions; `finish()` keeps them
readable through `dimensions()` while `on_commit_entities` runs, index-aligned with the committed
constraints. `DesignCanvas` exposes them as `sketch_dimensions()` / `dimensions()`.
`DesignCanvas::edit_sketch` recognises the document feature being re-edited by the address of the
entity vector the panel hands it, restores that feature's dimensions, and after the panel's commit
handler has run writes the session's dimensions into the feature it appended or replaced (guarded
on the entity and constraint counts, so a refused commit is never given another sketch's
dimensions).

## Committed sketches

A sketch feature that is not being edited is drawn muted gray-blue. The feature picked in the
viewport, or the one the panel names with `DesignCanvas::set_highlighted_sketch(feature)` (a tree
selection), also shows its dimensions, read-only, in the same drawing. `DesignCanvas::
set_display_sketches` attaches each feature's dimensions and full entity list to its
`DisplaySketch`, since the displayed entities may have consumed loops filtered out.

## Colours

Rendered by `DesignSketchTool::render` from the per-entity state:

| State | Light theme | Dark theme |
|---|---|---|
| Under-defined | blue `#1F5FCC` | light blue |
| Fully defined | black | near white |
| Over-defined / conflicting (from the solver's failed constraints) | red | light red |
| Selected (and held Smart Dimension picks) | `#4FC1FF`, heavier stroke | same |
| Pre-selection (Smart Dimension hover) | orange | orange |
| Construction | state colour, dashed | same |
| Dimensions | black | light grey |
| Driven dimensions | grey | grey |

The theme is the canvas's (`GLCanvas3D::get_dark_mode_status`). The defined state comes from
`sketch_entities_defined`: the Slvs C API does not report free parameters, so for each point of an
entity (and a circle's radius) the solver is asked whether an extra equation pinning that
coordinate where it is makes the system redundant — SolveSpace's own free-parameter test through
the public API. The probes run per connected component of the constraint graph, a component that
solves with zero DOF needs none, and a time budget (40 ms) reports whatever is left as
under-defined. The state is recomputed lazily when a change settles (not during a drag, not per
frame).

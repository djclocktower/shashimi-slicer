# CAD Front End (CommandManager, FeatureManager, PropertyManager): High Level Design

## Why it exists

The CAD workspace (the Sketch and Modeling tabs) is one `DesignPanel` holding one `CadDocument`,
one viewport and every command handler. People arriving from SolidWorks expect its screen layout:
a CommandManager ribbon of labelled buttons on top, the FeatureManager design tree on the left,
the PropertyManager replacing the tree while a command is open, and a status bar along the bottom.
The front end provides that layout. It adds no modelling behaviour of its own: every button, tree
row and PropertyManager control calls a handler that already exists in `DesignPanel` and that a
keyboard shortcut or a row of the right-click offer menu also calls. The ribbon, the keys and the
offer are three ways to reach the same handlers, so they cannot drift apart.

## Layout

```
+--------------------------------------------------------------------------------+
| [New/Undo/Redo] | CadRibbon page (Sketch or Features), scrolls | page tail (mode / Send to Plater)
+----------------------+---------------------------------------------------------+
| [FM] [PM] tab icons  | sketch banner (only while sketching)                    |
| wxSimplebook:        |                                                         |
|   CadFeatureTree     |                  DesignCanvas (viewport)                |
|   + Equations        |                                                         |
|   | CadPropertyManager                                                         |
+----------------------+---------------------------------------------------------+
| status line                                           constraint state (DoF)   |
+--------------------------------------------------------------------------------+
```

The left pane has the width of Prepare's sidebar (`sync_sidebar_width`), so the viewport edge does
not move when you switch between Prepare and the CAD tabs.

## The parts

| File | Role |
|---|---|
| `src/slic3r/GUI/CAD/CadRibbon.hpp/.cpp` | `CadTheme` colours, `CadCommand`, `CadToolButton`, `CadRibbon` |
| `src/slic3r/GUI/CAD/CadFeatureTree.hpp/.cpp` | The FeatureManager tree view of a `CadDocument` |
| `src/slic3r/GUI/CAD/CadPropertyManager.hpp/.cpp` | PropertyManager header, Message box, collapsible group boxes |
| `src/slic3r/GUI/CAD/DesignPanel.cpp` | Builds all three (`build_ribbon`, the cards, the tree callbacks) and routes them |
| `resources/images/sw_*.svg` | Ribbon, tree and PropertyManager icons |

### CadRibbon: the CommandManager

A single row of `CadToolButton`s: a 24 px icon (32 px for the large ones) over a label of up to two
lines, grouped by thin separators. It has two pages, chosen by `DesignPanel::WorkspaceTab` in
`on_workspace_tab_changed()`, a small corner group on the left (New, Undo, Redo) that both pages
share, and a tail on the right for each page that never scrolls away (the Sketch page's mode label,
the Features page's Send to Plater). The page between them scrolls horizontally, so on a narrow
window every command can still be reached. The pages hold nothing that can take focus, so showing a
page never scrolls it to a focused control or takes the keyboard from the viewport.

`CadToolButton` is drawn by the front end rather than being a `wxButton`, so hover, pressed,
checked and disabled look the same on every platform. It never takes keyboard focus: the
workspace's shortcuts are a `wxEVT_CHAR_HOOK` on the panel, and a focused button outside it would
block them. A flyout button shows ▾ after its label. Clicking the icon runs the button's current
command, and clicking the label row opens a `wxMenu` of variants. The variant you pick runs and
becomes the button's face. A menu-only button (Add Relation, Reference Geometry, Surfaces, ...)
opens the menu from anywhere on the button. Menu actions run after the menu has closed, so an
action can open a dialog or another menu.

The design_* glyphs are grey line art. Commands that reuse them are drawn in the sketch-geometry
blue (`CadCommand::sketch_glyph`), so one copy of each drawing is enough. The sw_* icons use a
flat SolidWorks palette: solids in yellow/tan with grey edges, sketch geometry in blue, cuts in
red, OK in green.

### Ribbon → handler map

Sketch page. With no sketch open, a sketch-entity tool first asks for a plane (see "Sketching from
the ribbon").

| Button | Handler |
|---|---|
| Sketch / Exit Sketch (corner, large) | Outside a sketch: `start_sketch_command()`. Inside: `tool_confirm()` |
| Cancel | `tool_cancel()` (asks before discarding drawn work) |
| Smart Dimension | `ribbon_sketch_tool(Mode::Dimension)`, the same mode the D key arms |
| Line ▾ (Line, Centerline, Polyline) | `Mode::Line`; Centerline is `Mode::Line` with construction on; `Mode::Polyline` |
| Rectangle ▾ (Corner, Center, 3 Point, Rounded) | `CornerRect`, `CenterRect`, `ObliqueRect`, `RoundedRect` |
| Circle ▾ (Circle, Perimeter, 3 Point) | `CenterCircle`, `TwoPointCircle`, `ThreePointCircle` |
| Arc ▾ (Centerpoint, Tangent, 3 Point) | `CenterArc`, `TangentArc`, `ThreePointArc` |
| Slot ▾ (Straight, Arc) | `Slot`, `ArcSlot` |
| Polygon ▾ (sides 3–12, inscribed/circumscribed) | `push_polygon_params()` + `Mode::Polygon` |
| Ellipse ▾ (Ellipse, Partial Ellipse) | `Ellipse`, `EllipseArc` |
| Spline, Point | `BSpline`, `Point` |
| Trim, Extend, Offset, Mirror, Fillet, Chamfer | `Trim`, `Extend`, `Offset`, `Mirror`, `Fillet`, `Chamfer` |
| Linear Pat., Circ. Pat., Move, Rotate, Scale | `Array`, `PolarArray`, `Move`, `Rotate`, `Scale` |
| Add Relation ▾ (20 relation types) | `apply_constraint(type)` |
| Relations (Display/Delete Relations) | Inside a sketch: `rebuild_constraint_list()` and open the Relations group. Outside: offer verb `btn:constrain` |
| More ▾ (Sketch Text, Import SVG, Normal To, For Construction) | offer verbs `btn:text`, `btn:svg`; sketch key `N`; For Construction is Q's behaviour: converts the selection in Select mode, otherwise `set_construction()` arms construction for what is drawn next |
| mode label (tail) | "Sketch" outside a sketch, the sketch's name inside one (with "· Construction" while construction is armed), "Relations" in Constrain (`update_ribbon_state`) |

Features page. If a sketch is open, each command that changes the model first leaves the sketch
through `tool_confirm()`, which keeps it (`run_feature_command`). The view toggles, Evaluate and
Appearance do not.

| Button | Handler |
|---|---|
| Extruded Boss/Base (large) | `fly:material#0` with `ModePreset::Boss` |
| Revolved Boss/Base | `fly:material#1` with `ModePreset::Boss` (Revolve operation reset to New) |
| Swept / Lofted Boss/Base | `fly:material#2`, `fly:material#3` |
| Rib ▾ (Rib, Thicken) | `fly:material#5`, `fly:material#4` |
| Extruded Cut / Revolved Cut | `fly:material#0` / `#1` with `ModePreset::Cut` (operation preselected to Cut) |
| Hole Wizard ▾ (Hole Wizard, Thread) | `fly:hole#0`, `fly:hole#1` |
| Fillet, Chamfer | `btn:dress#0`, `btn:dress#1` |
| Shell, Draft, Delete Face | `fly:dressup#2`, `#1`, `#3` |
| Linear / Circular Pattern | `btn:pat#0`, `btn:pat#1` |
| Mirror | `fly:placement#1` |
| Reference Geometry ▾ (Plane, Axis, Coordinate System, Helix/Spiral, Project Edges) | `fly:plane#0..4` |
| Combine ▾ (Combine, Subtract, Common, Split) | `btn:bool#0..2`; Split = feature key Shift+X |
| Surfaces ▾ (Extruded, Revolved, Lofted, Filled, Offset, Thicken) | `fly:surface#0..5` |
| Move/Copy Bodies ▾ (Move/Copy Body, Move Body (drag), Place on Face, Mate, Appearance) | `fly:placement#0`, `on_move_body()`, `place_on_face()`, `fly:placement#2`, `btn:colour` |
| Sketch | `start_sketch_command()` (asks for a plane, then switches to the Sketch tab) |
| Import ▾ (STEP, Mesh) | `on_import_step()` / `on_import_mesh()` |
| Section View (toggle) / Flip Section | `toggle_section_view()` / `flip_section_view()` |
| Evaluate ▾ (Mass Properties, Interference Detection, Export STEP) | `btn:mass`, `on_check_interference()`, `on_export_step()` |
| Show Bed (toggle) | feature key Ctrl+Shift+B |
| Send to Plater (large, tail) | `on_commit()` (see "Send to Plater") |

Both pages are sized to fit a 1366 px window without scrolling: short labels, tight button
sides, and the rarely used commands under More.

Commands that need a body are greyed, with a tooltip saying why, until the document has one
(`m_body_gates`, re-checked in `feed_bodies`).

### Sketching from the ribbon

SolidWorks asks for the sketch plane when a sketch starts. `ribbon_sketch_tool()` first moves to
the sketch environment: it cancels an open feature command, leaves Constrain, and calls
`set_ui_mode(Sketch)`, which also shows the three reference planes. If no sketch is open and there
is no fresh target, it stores the tool in `m_pending_sketch_*` and writes "Select a plane or planar
face to sketch on" to the status line. A fresh target is a planar face under the last pick, or a
plane picked since the last sketch started (`m_plane_fresh`). Clicking a reference plane (in the
viewport, or Front/Top/Right Plane or a datum plane in the tree) goes through
`pick_reference_plane()`. Clicking a planar face goes through the solid-pick handler. Both then
call `fire_pending_sketch_tool()`, which arms the stored tool and so starts the sketch on that
plane. While the tool waits, the left pane shows the FeatureManager so its planes can be picked,
and the sketch's PropertyManager comes back when the sketch starts. While it waits, the banner says "Select a plane or planar face for Sketch1" and the mode
label "Select a plane". Esc cancels the prompt. The Sketch command is the same path with
`Mode::Select`. Every new sketch, and every sketch opened for editing, turns the view normal to
its plane once (`select_sketch_tool`, `on_edit_feature`); switching tabs does not.

The keyboard and the offer menu keep their behaviour: they reuse the last chosen plane
(`m_plane_picked`) without asking.

### CadFeatureTree: the FeatureManager

A `wxTreeCtrl` rebuilt from the document by `refresh_tree()`:

```
Part (Shashimi)
  Solid Bodies(n)           every body with its colour swatch; hidden bodies greyed, hollow swatch
  Origin
  Front Plane, Top Plane, Right Plane      (XZ, XY, YZ)
  Sketch1                   a sketch no feature consumes stays at the top level
  Boss-Extrude1
    Sketch2                 a consumed sketch nests under the first feature that uses it
  Fillet1 ...
```

A feature consumes a sketch through `sketch_ref`, `sweep_path_ref`, `loft_profile_refs` or
`rib_sketch_ref`. Suppressed features are dimmed, and features with a mate conflict are red. The
rebuild keeps the selected row and the expanded rows, and it fires no selection callback.

The tree only reports what the user points at. The panel decides what that means:

- Selecting a sketch shows its dimensions (`set_highlighted_sketch`); any other row clears
  them. Selecting another feature highlights the solid. Selecting a body selects it in the viewport as the
  target of the next command. Selecting Front/Top/Right Plane or a datum plane picks it as the
  sketch plane.
- Double-clicking a feature edits it (`on_edit_feature`).
- Right-clicking a feature offers Edit Feature, Edit Sketch (the sketch it consumes), Sketch on
  this plane (datum planes), Suppress/Unsuppress (`CadFeature::enabled`), Rename, Scale artwork,
  Move Up/Down and Delete. Right-clicking a body offers Hide/Show, Rename, Appearance,
  Move/Rotate, Combine, Delete Body and More commands (the offer menu). Right-clicking a reference
  plane offers Sketch.
- Renaming a feature changes `CadFeature::name`. Renaming a body changes `CadBody::user_name`.
  The rebuild waits for the next event-loop turn, because wx is still inside the label-edit
  dispatch.

Document variables are listed under the tree as "Equations".

### Default feature names

New features get SolidWorks names from `next_feature_name(type, mode, variant)`: Sketch1,
Boss-Extrude1, Cut-Extrude1, Revolve1, Cut-Revolve1, Sweep1, Loft1, Fillet1, Chamfer1, Shell1,
Draft1, Hole1, LPattern1, CirPattern1, Mirror1, Plane1, Axis1, Coordinate System1, Combine1,
Split1, Import1, and so on. The number is one more than the highest number already used with that
prefix, so names stay unique after deletes and in projects loaded from disk.

### CadPropertyManager

When a command opens, the left pane shows the PropertyManager instead of the tree, as SolidWorks
does. The two tab icons over the pane switch back to the tree while the command stays open. The
PropertyManager has:

- a header with the command's icon and name (the name the feature will get, or its real name when
  editing), then ✓ and ✗, which call `tool_confirm()` and `tool_cancel()`. While the candidate is
  invalid, `refresh_preview()` greys ✓ through `m_confirm_btns`;
- a Message box that shows the status text;
- collapsible group boxes that hold the command's controls.

The per-tool controls are still built once in the `DesignPanel` constructor as cards (one `m_box_*`
sizer per tool, all parented to `m_cards`), and `open_tool()`/`close_tool()` still show exactly one
card. `m_cards` now lives in the PropertyManager's scrolled body. Each card is wrapped in group
boxes: `wrap_in_group()` moves an existing card's items into a group, and `make_group()` builds
the hand-laid ones. A card's header label (`m_hdr_*`) is hidden, and the PropertyManager header
shows its text instead. `update_cards_frame()` switches the pane: any visible card means a command
is open. It also re-collapses collapsed groups, because showing a card re-shows its whole subtree.

Extrude follows SolidWorks. It has From (Sketch Plane); Direction 1 (end condition Blind, Through
All, Up To Vertex, Up To Surface or Mid Plane, then Depth, Reverse Direction, Draft on/off and
angle); Direction 2 (on means Two-sided, with its own depth); Result (Boss, Cut or Intersect, plus
Merge result for a boss); and Selected Contours. `extrude_end_from_card()`,
`extrude_mode_from_card()`, `extrude_taper_from_card()` and `load_extrude_card()` map the groups
onto `ExtrudeEnd`, `BooleanMode` and `taper_deg`. Fillet/Chamfer has Type, Items To Fillet (the
picked edge, or an edge group) and Parameters. The sketch environment's PropertyManager shows the
Sketch Plane group and the Relations group (the constraint list). Constrain mode shows the
Display/Delete Relations PropertyManager.

### Send to Plater

`on_commit()` ships each visible body as its own plate object, named after the body (its
`user_name`, else "Part1 - Body N"). A body that was sent before updates its object in place,
so sending again never duplicates: the object's mesh is swapped the way the Simplify gizmo does
it (`set_mesh`, new volume id, `Plater::changed_mesh`), and its placement on the bed, its
settings and its instances stay. The new mesh is centred on its own box like any new volume, and the
volume moves by the change of centre (`source.mesh_offset`), so geometry that did not change
in CAD does not move on the plate. Painting is cleared, since it is per-triangle. The panel
remembers body index → `ObjectID` in `m_sent_objects` for the session. A design loaded from a
project finds its objects again by name, because object ids are not saved; a New Design forgets
both, so it never overwrites the previous design's objects. Objects whose body no longer exists
are left alone.

### Status bar

`set_status()` stays the only writer of the status line. The text and colour are kept in the
hidden `m_status`. The status bar shows the text on one line, ellipsized, with the full text as
tooltip, and the PropertyManager's Message box shows it too. The sketch constraint state
(`m_dof_status`) sits at the right of the status bar.

## Constraints

- The front end changes no command logic. Commands are reached through `m_verb_actions`,
  `m_keys_sketch`/`m_keys_feature` and the existing handlers. The offer-table join check at the
  end of the constructor still verifies every offer verb.
- Keyboard focus stays in the viewport. Ribbon and PropertyManager buttons never take focus, and
  when a command closes with focus on one of its hidden card controls, `update_action_bar()` gives
  focus back to the viewport.
- The workspace is English-only, like the rest of `DesignPanel` (`_L` is a UTF-8 literal there).
- `McpControl` uses only the `mcp_*` API of `DesignPanel`, which the front end does not change.

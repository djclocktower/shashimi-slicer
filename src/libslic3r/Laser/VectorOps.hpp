#pragma once

// Vector geometry for laser work: offsets, booleans, fills, kerf, tabs, lead-ins, cut ordering.
// Everything is workspace-frame, scaled (LaserDocument::flatten output); distances are mm.
// Built on ClipperUtils (offset_ex, union_ex, diff_ex, intersection_ex).

#include "libslic3r/Laser/LaserTypes.hpp"

namespace Slic3r::Laser {

enum class OffsetDir { Outward, Inward, Both };
enum class CornerStyle { Round, Miter, Bevel };
enum class BooleanOp { Union, Subtract, Intersect };

// Closed paths -> regions, even-odd (a path inside another is a hole). Open paths are ignored.
ExPolygons to_expolygons(const LaserPaths& paths);
// Regions -> closed paths (contours and holes).
LaserPaths to_paths(const ExPolygons& regions);

// Offset Shapes tool: closed results. Both = the outward and the inward result together.
LaserPaths offset_paths(const LaserPaths& paths, double distance_mm, OffsetDir dir, CornerStyle corners);
// Boolean tool: `a` op `b`, both read as regions (to_expolygons).
LaserPaths boolean_op(const LaserPaths& a, const LaserPaths& b, BooleanOp op);

// Scan fill. Lines at `angle_deg` from +X, `interval_mm` apart, clipped to the regions; crosshatch
// adds the set at angle + 90. Ordered for little travel (bidirectional: alternate directions,
// otherwise every line runs the same way).
Polylines  hatch_fill(const ExPolygons& regions, double interval_mm, double angle_deg, bool crosshatch, bool bidirectional);
// Concentric inward offsets `interval_mm` apart, innermost ring first.
LaserPaths offset_fill(const ExPolygons& regions, double interval_mm);

// Grows the closed parts by kerf_mm (outer contours move out, holes move in); negative shrinks.
// Open paths pass through unchanged.
LaserPaths kerf_offset(const LaserPaths& paths, double kerf_mm);
// Cuts `tab_size_mm` gaps into closed paths: `count` evenly spaced per path, or one every
// `spacing_mm` when that is > 0. Tabbed paths come back open; open paths unchanged.
LaserPaths insert_tabs(const LaserPaths& paths, int count, double spacing_mm, double tab_size_mm);
// Closed paths get a straight approach of `length_mm` from outside the part ending at the path
// start (they come back open, lead-in first). Open paths unchanged.
LaserPaths add_lead_in(const LaserPaths& paths, double length_mm);

// Reorders `paths` in place for cutting from `start` (mm): inner paths before the paths enclosing
// them (cut_inner_first), nearest-neighbour chaining with reversed open paths and rotated closed
// start points (reduce_travel), preferring the previous direction (reduce_direction_changes).
// opts.order_by is the planner's business and is ignored here. Returns the end position (mm).
Vec2d      optimize_order(LaserPaths& paths, const Vec2d& start, const JobSettings::Optimize& opts);

} // namespace Slic3r::Laser

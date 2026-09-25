#pragma once

// Smart Dimension: the SolidWorks-style dimension annotations of an entity sketch.
//
// A dimension is an annotation that NAMES a measurable quantity of the sketch (a length, a
// distance, a radius, an angle) and remembers where its text sits. A DRIVING dimension also owns
// one SketchEntityConstraintDef in the sketch's constraint list, which is what makes the solver
// enforce the value; a DRIVEN (reference) dimension only measures. The annotation references
// geometry the same way constraints do — (entity index, point role), with the negative implicit
// references for the sketch origin and axes — so it follows the solve and is re-indexed by the
// same rules when entities are deleted.
//
// Everything here is pure and headless: the GUI picks, the kernel decides what the picks mean
// (resolve_smart_dimension), what they measure (measure_sketch_dimension), which constraint drives
// them (sketch_dimension_constraint) and where every stroke of the annotation goes
// (layout_sketch_dimension). The renderer only draws what the layout returns.

#include "libslic3r/CAD/SketchEngine.hpp"

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r {

// Serialized as int: APPEND-ONLY, never reorder.
enum class SketchDimKind {
    Length,      // ea: a line; aligned distance p0-p1
    Horizontal,  // points (ea,ra) (eb,rb): |dx|
    Vertical,    // points (ea,ra) (eb,rb): |dy|
    Diameter,    // ea: a circle (or arc)
    Radius,      // ea: an arc (or circle)
    Angle,       // ea, eb: lines / axes; `sector` picks which of the four angles
    PointPoint,  // points (ea,ra) (eb,rb): aligned distance
    PointLine,   // point (ea,ra), line eb: perpendicular distance
    LineLine     // parallel lines ea, eb: distance between them
};

struct SketchDimension {
    SketchDimKind   kind{SketchDimKind::Length};
    int             ea{-1};
    int             eb{-1};
    SketchPointRole ra{SketchPointRole::P0};
    SketchPointRole rb{SketchPointRole::P0};
    Vec2d           text_pos{0, 0};   // sketch-plane coordinates of the text centre
    bool            driven{false};    // reference dimension: measured, owns no constraint
    // Index of the driving constraint in the sketch's constraint list (CadFeature::
    // entity_constraints / the live sketch's constraints); -1 when driven. Re-indexed together
    // with the constraint list (see sketch_dimensions_remap_constraints).
    int             constraint{-1};
    double          value{0.0};       // display units: mm, degrees for Angle
    // Angle only. The two lines cross at X and cut the plane into four sectors; the dimensioned
    // one is spanned by the rays X + sa*dir(ea) and X + sb*dir(eb), where dir(e) = p1 - p0
    // (sketch_dim_line_ends) and bit0 set means sa = -1, bit1 set means sb = -1.
    int             sector{0};

    template<class Archive> void serialize(Archive& ar)
    {
        // Append-only.
        ar(kind, ea, eb, ra, rb, text_pos, driven, constraint, value, sector);
    }
};

// One Smart Dimension pick: a whole entity, or a point of one. `entity` may be an entity index
// or kSketchRefOrigin / kSketchRefAxisX / kSketchRefAxisY. A Point entity and the origin are
// points however they are picked.
struct SmartDimPick {
    int             entity{-1};
    bool            point{false};
    SketchPointRole role{SketchPointRole::P0};

    static SmartDimPick whole(int e) { SmartDimPick p; p.entity = e; return p; }
    static SmartDimPick at(int e, SketchPointRole r) { SmartDimPick p; p.entity = e; p.point = true; p.role = r; return p; }
    bool operator==(const SmartDimPick& o) const
    {
        return entity == o.entity && point == o.point && (!point || role == o.role);
    }
    bool operator!=(const SmartDimPick& o) const { return !(*this == o); }
};

struct SmartDimResolution {
    bool            ok{false};
    SketchDimension dim;          // kind, references, sector; text_pos = cursor; value = measured
};

// SolidWorks Smart Dimension selection rules.
//   1 line            -> cursor inside the band perpendicular to the line between its endpoints:
//                        aligned Length; outside it: Horizontal when |offset from midpoint|.y >=
//                        |.x|, else Vertical. An axis-aligned line always gives the matching H/V.
//   1 circle / 1 arc  -> Diameter / Radius.
//   line + line       -> parallel: LineLine distance; otherwise Angle of the sector the cursor
//                        lies in (interior angle of that sector).
//   point + point     -> distance, with the line's aligned / Horizontal / Vertical cursor rule.
//   point + line      -> PointLine distance.
//   point + circle/arc, circle + circle/arc -> distance to / between the centres (point rule).
//   line + circle/arc -> PointLine distance from the centre.
// ok == false for anything else (a single point, an ellipse or spline picked whole, a pair that
// coincides). A second pick equal to the first is treated as absent.
SmartDimResolution resolve_smart_dimension(const std::vector<SketchEntity>& entities,
                                           const SmartDimPick& first,
                                           const std::optional<SmartDimPick>& second,
                                           const Vec2d& cursor);

// Current value of `dim` in display units (mm / degrees). False when a reference is missing or
// degenerate.
bool measure_sketch_dimension(const std::vector<SketchEntity>& entities,
                              const SketchDimension& dim, double& value);

// The constraint that drives `dim` to `value` (display units). Angle values are converted to the
// solver convention: RADIANS, measured between the lines' p0->p1 directions (so a sector that
// uses one reversed arm stores the supplement). Horizontal/Vertical order their references so the
// signed DistanceX/Y matches the current geometry. nullopt for a value the kind cannot take.
std::optional<SketchEntityConstraintDef> sketch_dimension_constraint(
    const std::vector<SketchEntity>& entities, const SketchDimension& dim, double value);

// Angle constraint value (radians) for a typed sector angle in degrees. Shared with the panel's
// constraint path so both store the solver's unit.
double sketch_angle_constraint_value(double sector_deg, int sector);

// The two defining points of a line reference in the solver's order: a line's p0 and p1, the
// axis references (1,0)->(0,0) and (0,1)->(0,0) (the solver builds them head-first). Angle
// sectors use the direction b - a, which is what keeps them consistent with SLVS_C_ANGLE.
bool sketch_dim_line_ends(const std::vector<SketchEntity>& entities, int e, Vec2d& a, Vec2d& b);

// Display text: up to 2 decimals, trailing zeros trimmed, "Ø" / "R" prefix, "°" suffix, and
// parentheses around a driven dimension.
std::string format_sketch_dimension(const SketchDimension& dim, double value);

// Sizes in sketch-plane units (the caller scales them from pixels at the current zoom).
struct SketchDimStyle {
    double arrow_len{2.0};
    double arrow_width{0.8};
    double ext_gap{0.6};        // gap between the geometry and an extension line
    double ext_overshoot{1.0};  // extension line past the dimension line
    double text_width{6.0};     // approximate label extent along the dimension line
};

struct SketchDimLayout {
    bool ok{false};
    std::vector<std::pair<Vec2d, Vec2d>> ext_lines;  // extension lines
    std::vector<std::pair<Vec2d, Vec2d>> dim_lines;  // dimension line, leader, tessellated arc
    std::vector<std::array<Vec2d, 3>>    arrows;     // filled heads: tip, base, base
    Vec2d text{0, 0};                                // label centre
    bool  arrows_inside{true};                       // false: heads outside, pointing in
};

// Every stroke of the annotation, from the solved geometry and dim.text_pos.
SketchDimLayout layout_sketch_dimension(const std::vector<SketchEntity>& entities,
                                        const SketchDimension& dim, const SketchDimStyle& style);

// ---- Re-indexing, mirroring what the sketch does to its constraint list ----------------------
// Entities: old index -> new index, or -1 when deleted. A dimension referencing a deleted
// entity is dropped. Negative references (origin/axes) are kept as they are.
void sketch_dimensions_remap_entities(std::vector<SketchDimension>& dims, const std::vector<int>& entity_remap);
// Constraints: old index -> new index, or -1 when dropped. A dimension whose constraint was
// dropped becomes driven.
void sketch_dimensions_remap_constraints(std::vector<SketchDimension>& dims, const std::vector<int>& constraint_remap);
// Constraint `idx` was erased from the list: shift the later indices down.
void sketch_dimensions_erase_constraint(std::vector<SketchDimension>& dims, int idx);
// Clamp a loaded set against its sketch: out-of-range entity references drop the dimension,
// an out-of-range constraint index makes it driven.
void sketch_dimensions_sanitize(std::vector<SketchDimension>& dims, int n_entities, int n_constraints);

} // namespace Slic3r

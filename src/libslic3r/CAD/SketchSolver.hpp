#ifndef slic3r_SketchSolver_hpp_
#define slic3r_SketchSolver_hpp_

// Bridge from the Design tab's SketchEntity / SketchEntityConstraintDef model onto the
// vendored SolveSpace constraint solver (src/libslic3r/slvs, libslvs). Replaces the
// hand-rolled SketchConstraints: full constraint set, real DoF counting, and
// over-constrained (bad-constraint) detection. Solves on a fixed 2D XY workplane.

#include "libslic3r/CAD/SketchEngine.hpp"
#include <vector>

namespace Slic3r {

struct SketchSolveResult {
    bool             ok{false};   // solver converged & consistent
    int              dof{-1};     // remaining degrees of freedom (>0 under-constrained)
    int              result{0};   // raw SLVS_RESULT_* code
    std::vector<int> bad;         // indices (into `constraints`) of conflicting constraints
};

// Solve `constraints` over `entities` in place (writes solved coordinates back into the
// entities; arc angles are reflowed preserving sweep direction). No-op success when
// `constraints` is empty.
SketchSolveResult sketch_solve(std::vector<SketchEntity>& entities,
                               const std::vector<SketchEntityConstraintDef>& constraints);

// Drag-aware solve: pins the (dragged_ei, dragged_role) point's parameters via the
// solver's `dragged[]` priority list so the solver keeps that point where the cursor
// placed it (caller must have moved it first) and moves the OTHER free geometry to
// re-satisfy the constraints. dragged_ei < 0 behaves identically to sketch_solve.
SketchSolveResult sketch_solve_drag(std::vector<SketchEntity>& entities,
                                    const std::vector<SketchEntityConstraintDef>& constraints,
                                    int dragged_ei, SketchPointRole dragged_role);

// Per-entity "fully defined" state, the colour SolidWorks paints a sketch entity with: defined[i]
// is 1 when every solver parameter of entity i (its points, and a circle's radius) is fixed by
// `constraints`, 0 when some part of it can still move. Costs up to two extra solves of the
// entity's connected component per point; a component solving with zero DOF costs none. Past
// `max_ms` the remaining under-defined components are reported whole as 0 (conservative).
// Returns false — and all zeros — when the sketch does not solve (over-defined / conflicting).
bool sketch_entities_defined(const std::vector<SketchEntity>& entities,
                             const std::vector<SketchEntityConstraintDef>& constraints,
                             std::vector<char>& defined, double max_ms = 40.0);

} // namespace Slic3r

#endif

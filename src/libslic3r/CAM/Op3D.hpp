#pragma once

// 3D strategies: Adaptive3D (roughing), Parallel3D and Contour3D (finishing).

#include "libslic3r/CAM/CamDocument.hpp"

#include <vector>

namespace Slic3r::CAM {

// Tool-tip Z on a regular XY grid so that the tool touches the mesh without penetrating it.
// Node (i, j) is at (x0 + i * resolution, y0 + j * resolution); z[j * nx + i].
struct HeightMap {
    double             x0{0}, y0{0};
    double             resolution{0.1};
    int                nx{0}, ny{0};
    std::vector<float> z;
    float at(int i, int j) const { return z[size_t(j) * size_t(nx) + size_t(i)]; }
};

// Drop-cutter over `mesh` (same frame as `box`, normally the setup frame) for flat/ball/bull
// cutters (vertex, edge and facet tests, AABB-accelerated). Covers box's XY; nodes the tool does
// not touch get box.min.z().
HeightMap drop_cutter(const TriangleMesh& mesh, const CamTool& tool, const BoundingBox3Base<Vec3d>& box,
                      double resolution);

Toolpath generate_adaptive3d(const CamDocument& doc, const CamOperation& op, const CamModel& model,
                             const ProgressFn& progress = {});
Toolpath generate_parallel3d(const CamDocument& doc, const CamOperation& op, const CamModel& model,
                             const ProgressFn& progress = {});
Toolpath generate_contour3d(const CamDocument& doc, const CamOperation& op, const CamModel& model,
                             const ProgressFn& progress = {});

} // namespace Slic3r::CAM

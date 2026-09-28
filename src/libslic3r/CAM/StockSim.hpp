#pragma once

// Material-removal simulation: dexel height map z(x, y) for box stock, polar height map r(x, theta)
// for cylinder stock (4-axis). Setup frame.

#include "libslic3r/CAM/CamTypes.hpp"

#include <vector>

namespace Slic3r::CAM {

class StockSim {
public:
    // resolution <= 0: automatic from the stock size (never finer than ~0.2 mm).
    void init_box(const BoundingBoxf3& stock, double resolution = 0);
    // Cylinder along X from x0 to x1 (axis at Y = Z = 0); resolution along X, and arc length at
    // the radius around it.
    void init_cylinder(double x0, double x1, double radius, double resolution = 0);

    // Sweeps `tool` from from.to/from.a_deg to to.to/to.a_deg (arcs per to's arc fields).
    // Flat, ball, bull, V/chamfer and drill shapes.
    void cut(const CamTool& tool, const Move& from, const Move& to);

    TriangleMesh to_mesh() const;
    bool         rotary() const { return m_rotary; }

private:
    // Owned by StockSim.cpp: change freely.
    bool               m_rotary{false};
    double             m_x0{0}, m_y0{0}, m_res{0.2};
    int                m_nx{0}, m_ny{0};
    double             m_base{0};   // box: stock bottom Z; cylinder: unused
    std::vector<float> m_h;         // box: top Z per cell; cylinder: radius per (x, theta) cell
};

// Feed (non-Rapid) moves whose tool bottom goes below `model` (ray test against the mesh,
// setup frame), one Warning each ("Gouge: tool goes 0.3 mm into the part").
std::vector<Warning> gouge_check(const Toolpath& tp, const CamTool& tool, const TriangleMesh& model);

} // namespace Slic3r::CAM

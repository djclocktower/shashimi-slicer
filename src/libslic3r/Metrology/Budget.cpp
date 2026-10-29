#include "Budget.hpp"

#include "MetrologyTypes.hpp"

#include <algorithm>
#include <cmath>

namespace Slic3r::Metrology {

BudgetResult budget(const BudgetInputs& in)
{
    BudgetResult r;
    r.readout = in.mm_per_px * in.centroid_floor / std::max(std::sin(in.view_angle_deg * kPi / 180.0), 0.1);
    r.height  = std::sqrt(r.readout * r.readout + in.trigger * in.trigger + in.tip * in.tip + in.thermal * in.thermal);
    const double res = in.bracket_mm / std::pow(2.0, std::max(1, in.touches_per_edge)) / std::sqrt(12.0);
    r.edge        = std::sqrt(in.motion * in.motion + res * res + in.size_reference * in.size_reference);
    r.camera_edge = in.mm_per_px * in.edge_floor;
    return r;
}

double fuse(double a, double sa, double b, double sb, double* sigma)
{
    if (sa <= 0 || sb <= 0) {
        if (sigma) *sigma = std::max(sa, sb);
        return sa <= 0 ? a : b;
    }
    const double wa = 1 / (sa * sa), wb = 1 / (sb * sb);
    if (sigma) *sigma = 1 / std::sqrt(wa + wb);
    return (a * wa + b * wb) / (wa + wb);
}

} // namespace Slic3r::Metrology

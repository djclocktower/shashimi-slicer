#include "libslic3r/CAM/StockSim.hpp"
#include "libslic3r/CAM/CamInternal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace Slic3r::CAM {

namespace {

constexpr double kMaxCells = 1.5e6;

// Calls fn(tip, a_deg) at points along from -> to no more than `step` apart (arcs followed in XY,
// Z and A interpolated). `a_radius` turns A degrees into surface length for the spacing.
template<class Fn> void sample_move(const Move& from, const Move& to, double step, double a_radius, Fn&& fn)
{
    const Vec3d a = from.to, b = to.to;
    const double aa = from.a_deg, ab = to.a_deg;
    if (is_arc(to)) {
        const Vec2d  c  = to.center.head<2>();
        const double r  = (a.head<2>() - c).norm();
        const double t0 = std::atan2(a.y() - c.y(), a.x() - c.x());
        double       sw = std::atan2(b.y() - c.y(), b.x() - c.x()) - t0;
        if (arc_dir(to) == ArcDir::CCW) {
            while (sw <= 1e-9) sw += 2 * M_PI;
        } else {
            while (sw >= -1e-9) sw -= 2 * M_PI;
        }
        const int n = std::max(1, int(std::ceil(std::abs(sw) * r / step)));
        for (int k = 0; k <= n; ++k) {
            const double t = double(k) / n, ang = t0 + t * sw;
            fn(Vec3d(c.x() + r * std::cos(ang), c.y() + r * std::sin(ang), a.z() + t * (b.z() - a.z())), aa + t * (ab - aa));
        }
        return;
    }
    const double len = std::max((b - a).head<2>().norm(), std::abs(ab - aa) * M_PI / 180. * a_radius);
    const int    n   = std::max(1, int(std::ceil(len / step)));
    for (int k = 0; k <= n; ++k) {
        const double t = double(k) / n;
        fn(Vec3d(a + t * (b - a)), aa + t * (ab - aa));
    }
}

} // namespace

void StockSim::init_box(const BoundingBoxf3& stock, double resolution)
{
    const Vec3d s = stock.size();
    m_rotary      = false;
    m_res         = std::max(resolution > 0 ? resolution : 0.2, std::sqrt(std::max(s.x() * s.y(), 1e-6) / kMaxCells));
    m_nx          = std::max(2, int(std::ceil(s.x() / m_res)));
    m_ny          = std::max(2, int(std::ceil(s.y() / m_res)));
    // grid centred on the stock
    m_x0   = stock.center().x() - m_nx * m_res / 2;
    m_y0   = stock.center().y() - m_ny * m_res / 2;
    m_base = stock.min.z();
    m_h.assign(size_t(m_nx) * m_ny, float(stock.max.z()));
    m_h0   = m_h;
    m_done = 0;
}

void StockSim::init_cylinder(double x0, double x1, double radius, double resolution)
{
    const double len = std::max(x1 - x0, 1e-3), circ = 2 * M_PI * std::max(radius, 1e-3);
    m_rotary = true;
    m_res    = std::max(resolution > 0 ? resolution : 0.2, std::sqrt(len * circ / kMaxCells));
    m_nx     = std::max(2, int(std::ceil(len / m_res)));
    m_ny     = std::max(8, int(std::ceil(circ / m_res)));
    m_x0     = (x0 + x1) / 2 - m_nx * m_res / 2;
    m_y0     = 0;
    m_base   = radius;
    m_h.assign(size_t(m_nx) * m_ny, float(radius));
    m_h0   = m_h;
    m_done = 0;
}

void StockSim::cut(const CamTool& tool, const Move& from, const Move& to)
{
    if (m_h.empty())
        return;
    const internal::Cutter c  = internal::make_cutter(tool);
    const double           R  = c.R, R2 = R * R;
    const double           dt = 2 * M_PI / m_ny;

    const auto stamp_box = [&](const Vec3d& p) {
        const int i0 = std::max(0, int(std::floor((p.x() - R - m_x0) / m_res)));
        const int i1 = std::min(m_nx - 1, int(std::floor((p.x() + R - m_x0) / m_res)));
        const int j0 = std::max(0, int(std::floor((p.y() - R - m_y0) / m_res)));
        const int j1 = std::min(m_ny - 1, int(std::floor((p.y() + R - m_y0) / m_res)));
        for (int j = j0; j <= j1; ++j) {
            const double dy  = m_y0 + (j + 0.5) * m_res - p.y();
            const double dy2 = dy * dy;
            if (dy2 > R2)
                continue;
            float* row = m_h.data() + size_t(j) * m_nx;
            for (int i = i0; i <= i1; ++i) {
                if (p.z() >= row[i])
                    continue;
                const double dx = m_x0 + (i + 0.5) * m_res - p.x();
                const double d2 = dx * dx + dy2;
                if (d2 > R2)
                    continue;
                const float z = float(std::max(m_base, p.z() + c.h(std::sqrt(d2))));
                if (z < row[i])
                    row[i] = z;
            }
        }
    };

    // Polar map: cell (x, theta) holds the stock radius along the ray at angle theta (from +Y towards
    // +Z, part frame). At A = a the ray points at phi = theta + a in the machine frame, under the
    // vertical tool. ponytail: only rays whose current surface point is inside the tool are cut
    // (a ray entering the tool only below its surface point is missed), and the new radius comes
    // from 4 fixed-point steps on the tool's bottom profile. Good for display, not for gouge checks.
    const auto stamp_cyl = [&](const Vec3d& p, double a_deg) {
        const double beta = std::asin(std::min(1., (R + std::abs(p.y())) / std::max(m_base, 1e-6)));
        const double a    = a_deg * M_PI / 180.;
        const int    j0   = int(std::floor((M_PI / 2 - beta - a) / dt - 0.5));
        const int    j1   = int(std::ceil((M_PI / 2 + beta - a) / dt - 0.5));
        const int    i0   = std::max(0, int(std::floor((p.x() - R - m_x0) / m_res)));
        const int    i1   = std::min(m_nx - 1, int(std::floor((p.x() + R - m_x0) / m_res)));
        for (int i = i0; i <= i1; ++i) {
            const double dx = m_x0 + (i + 0.5) * m_res - p.x();
            if (dx * dx > R2)
                continue;
            for (int jr = j0; jr <= j1; ++jr) {
                const int    j   = ((jr % m_ny) + m_ny) % m_ny;
                const double phi = (jr + 0.5) * dt + a;
                const double s = std::sin(phi), co = std::cos(phi);
                if (s < 0.05)
                    continue;
                float&       cell = m_h[size_t(j) * m_nx + i];
                const double r0   = cell;
                if (r0 * s <= p.z())
                    continue;
                const double dy = r0 * co - p.y(), d2 = dx * dx + dy * dy;
                if (d2 > R2 || r0 * s <= p.z() + c.h(std::sqrt(d2)))
                    continue;
                double r = r0;
                for (int it = 0; it < 4; ++it) {
                    const double ey = r * co - p.y();
                    r = std::clamp((p.z() + c.h(std::min(R, std::sqrt(dx * dx + ey * ey)))) / s, 0., r0);
                }
                cell = float(r);
            }
        }
    };

    sample_move(from, to, m_res / 2, m_base, [&](const Vec3d& p, double a_deg) {
        if (m_rotary)
            stamp_cyl(p, a_deg);
        else
            stamp_box(p);
    });
}

void StockSim::cut_upto(const CamTool& tool, const Toolpath& tp, int move_index)
{
    if (move_index < m_done) {
        m_h    = m_h0;
        m_done = 0;
    }
    const int last = std::min(move_index, int(tp.moves.size()) - 1);
    for (int k = std::max(1, m_done + 1); k <= last; ++k)
        cut(tool, tp.moves[k - 1], tp.moves[k]);
    m_done = std::max(m_done, last);
}

double StockSim::top_at(double x, double y) const
{
    const int i = int(std::floor((x - m_x0) / m_res)), j = int(std::floor((y - m_y0) / m_res));
    if (m_rotary || i < 0 || j < 0 || i >= m_nx || j >= m_ny)
        return std::numeric_limits<double>::quiet_NaN();
    return m_h[size_t(j) * m_nx + i];
}

double StockSim::removed_volume() const
{
    double v = 0;
    if (m_rotary) {
        const double dt = 2 * M_PI / m_ny;
        for (size_t k = 0; k < m_h.size(); ++k)
            v += 0.5 * (double(m_h0[k]) * m_h0[k] - double(m_h[k]) * m_h[k]) * dt * m_res;
    } else
        for (size_t k = 0; k < m_h.size(); ++k)
            v += (double(m_h0[k]) - m_h[k]) * m_res * m_res;
    return v;
}

TriangleMesh StockSim::to_mesh() const
{
    indexed_triangle_set its;
    if (m_h.empty())
        return TriangleMesh(std::move(its));
    auto&      V   = its.vertices;
    auto&      F   = its.indices;
    const auto idx = [&](int i, int j) { return int(j * m_nx + i); };
    // node x/y: cell centres, the outermost pulled out to the grid edge so the volume matches
    const auto nx_ = [&](int i) { return float(i == 0 ? m_x0 : i == m_nx - 1 ? m_x0 + m_nx * m_res : m_x0 + (i + 0.5) * m_res); };

    if (m_rotary) {
        const double dt = 2 * M_PI / m_ny;
        for (int j = 0; j < m_ny; ++j)
            for (int i = 0; i < m_nx; ++i) {
                const double r = m_h[size_t(j) * m_nx + i], t = (j + 0.5) * dt;
                V.emplace_back(nx_(i), float(r * std::cos(t)), float(r * std::sin(t)));
            }
        for (int j = 0; j < m_ny; ++j) {
            const int jn = (j + 1) % m_ny;
            for (int i = 0; i + 1 < m_nx; ++i) {
                F.emplace_back(idx(i, j), idx(i, jn), idx(i + 1, jn));
                F.emplace_back(idx(i, j), idx(i + 1, jn), idx(i + 1, j));
            }
        }
        const int c0 = int(V.size());
        V.emplace_back(nx_(0), 0.f, 0.f);
        V.emplace_back(nx_(m_nx - 1), 0.f, 0.f);
        for (int j = 0; j < m_ny; ++j) {
            const int jn = (j + 1) % m_ny;
            F.emplace_back(c0, idx(0, jn), idx(0, j));
            F.emplace_back(c0 + 1, idx(m_nx - 1, j), idx(m_nx - 1, jn));
        }
        return TriangleMesh(std::move(its));
    }

    const auto ny_ = [&](int j) { return float(j == 0 ? m_y0 : j == m_ny - 1 ? m_y0 + m_ny * m_res : m_y0 + (j + 0.5) * m_res); };
    for (int j = 0; j < m_ny; ++j)
        for (int i = 0; i < m_nx; ++i)
            V.emplace_back(nx_(i), ny_(j), m_h[size_t(j) * m_nx + i]);
    // Top. Runs of untouched quads merge into one quad (T-junctions: watertight-ish, volume exact).
    const auto untouched = [&](int i, int j) { return m_h[size_t(j) * m_nx + i] == m_h0[size_t(j) * m_nx + i]; };
    for (int j = 0; j + 1 < m_ny; ++j)
        for (int i = 0; i + 1 < m_nx;) {
            int i1 = i;
            while (i1 + 1 < m_nx && untouched(i1, j) && untouched(i1 + 1, j) && untouched(i1, j + 1) && untouched(i1 + 1, j + 1))
                ++i1;
            if (i1 == i)
                i1 = i + 1; // a single (touched) quad
            F.emplace_back(idx(i, j), idx(i1, j), idx(i1, j + 1));
            F.emplace_back(idx(i, j), idx(i1, j + 1), idx(i, j + 1));
            i = i1;
        }
    // Walls and bottom along the perimeter (CCW seen from above).
    std::vector<int> ring;
    for (int i = 0; i < m_nx - 1; ++i) ring.push_back(idx(i, 0));
    for (int j = 0; j < m_ny - 1; ++j) ring.push_back(idx(m_nx - 1, j));
    for (int i = m_nx - 1; i > 0; --i) ring.push_back(idx(i, m_ny - 1));
    for (int j = m_ny - 1; j > 0; --j) ring.push_back(idx(0, j));
    const int b0 = int(V.size());
    V.reserve(V.size() + ring.size() + 1);
    for (int t : ring)
        V.emplace_back(V[t].x(), V[t].y(), float(m_base));
    const int cb = int(V.size());
    V.emplace_back(float(m_x0 + m_nx * m_res / 2), float(m_y0 + m_ny * m_res / 2), float(m_base));
    const int n = int(ring.size());
    for (int k = 0; k < n; ++k) {
        const int kn = (k + 1) % n;
        F.emplace_back(ring[k], b0 + k, b0 + kn);
        F.emplace_back(ring[k], b0 + kn, ring[kn]);
        F.emplace_back(cb, b0 + kn, b0 + k);
    }
    return TriangleMesh(std::move(its));
}

std::vector<Warning> gouge_check(const Toolpath& tp, const CamTool& tool, const TriangleMesh& model)
{
    std::vector<Warning> out;
    if (model.its.indices.empty())
        return out;
    // ponytail: 0.02 mm tolerance (radially: a tool shrunk by it, so tangent contact is not a
    // gouge; axially: the reported depth) for tessellation / sampling noise. 4-axis (A != 0) moves
    // are not checked.
    const double               tol = 0.02;
    const internal::DropCutter dc(model.its, internal::make_cutter(tool, -tol));
    const double               lowest = -std::numeric_limits<double>::max();
    for (size_t k = 1; k < tp.moves.size(); ++k) {
        const Move& m = tp.moves[k];
        const Move& p = tp.moves[k - 1];
        if (m.kind == Move::Kind::Rapid || m.kind == Move::Kind::Retract || m.a_deg != 0 || p.a_deg != 0)
            continue;
        double worst = tol;
        Vec3d  where = Vec3d::Zero();
        sample_move(p, m, 0.25, 0, [&](const Vec3d& tip, double) {
            const double depth = dc.at(tip.x(), tip.y(), lowest) - tip.z();
            if (depth > worst) {
                worst = depth;
                where = tip;
            }
        });
        if (worst > tol) {
            char buf[96];
            snprintf(buf, sizeof(buf), "Gouge: tool goes %.2f mm into the part", worst);
            out.push_back(Warning{int(k), where, buf});
        }
    }
    return out;
}

} // namespace Slic3r::CAM

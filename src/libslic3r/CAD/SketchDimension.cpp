#include "libslic3r/CAD/SketchDimension.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace Slic3r {

namespace {

using R  = SketchPointRole;
using ET = SketchEntity::Type;
using K  = SketchDimKind;

constexpr double kEps      = 1e-9;
// A pair whose |dy| is below this fraction of its length IS horizontal (and |dx| vertical): the
// aligned, horizontal and vertical dimensions of it are the same number, so only H/V is offered.
constexpr double kAxisTol  = 1e-6;
// Two lines whose unit directions have |cross| below this are parallel: a distance, not an angle.
constexpr double kParallel = 1e-4;

inline double cross2(const Vec2d& a, const Vec2d& b) { return a.x() * b.y() - a.y() * b.x(); }
inline Vec2d  perp(const Vec2d& v) { return Vec2d(-v.y(), v.x()); }

bool valid(const std::vector<SketchEntity>& ents, int e) { return e >= 0 && e < int(ents.size()); }
bool is_axis(int e) { return e == kSketchRefAxisX || e == kSketchRefAxisY; }

// Position of the point reference (e, r). The origin and the axes all pass through (0,0).
bool point_of(const std::vector<SketchEntity>& ents, int e, R r, Vec2d& out)
{
    if (e == kSketchRefOrigin || is_axis(e)) { out = Vec2d::Zero(); return true; }
    if (!valid(ents, e)) return false;
    const SketchEntity& s = ents[e];
    switch (s.type) {
    case ET::Circle:
    case ET::Ellipse:    out = s.center; return true;
    case ET::Point:      out = s.p0;     return true;
    case ET::Line:
    case ET::BSpline:
        if (r == R::P0) { out = s.p0; return true; }
        if (r == R::P1) { out = s.p1; return true; }
        return false;
    case ET::Arc:
    case ET::EllipseArc:
        out = (r == R::P0) ? s.p0 : (r == R::P1) ? s.p1 : s.center;
        return true;
    }
    return false;
}

bool round_of(const std::vector<SketchEntity>& ents, int e, Vec2d& c, double& r)
{
    if (!valid(ents, e)) return false;
    const SketchEntity& s = ents[e];
    if (s.type != ET::Circle && s.type != ET::Arc) return false;
    c = s.center;
    r = s.radius;
    return r > kEps;
}

// What a pick is, geometrically.
enum class G { None, Point, Line, Round };
struct Geo {
    G    g{G::None};
    int  e{-1};
    R    role{R::P0};
    bool edge{false};   // Round measured to its edge
};

// Radius of the circle/arc reference (e, Center), or 0 when it is not one.
double round_radius(const std::vector<SketchEntity>& ents, int e, R r)
{
    Vec2d c; double rad = 0.0;
    return (r == R::Center && round_of(ents, e, c, rad)) ? rad : 0.0;
}

// How much of a centre distance the edge flags of `d` take off (PointPoint / PointLine only).
double edge_offset(const std::vector<SketchEntity>& ents, const SketchDimension& d)
{
    if (d.kind != SketchDimKind::PointPoint && d.kind != SketchDimKind::PointLine) return 0.0;
    double off = 0.0;
    if (d.sector & 1) off += round_radius(ents, d.ea, d.ra);
    if ((d.sector & 2) && d.kind == SketchDimKind::PointPoint) off += round_radius(ents, d.eb, d.rb);
    return off;
}

Geo classify(const std::vector<SketchEntity>& ents, const SmartDimPick& p)
{
    Geo g;
    g.e = p.entity;
    if (p.entity == kSketchRefOrigin) { g.g = G::Point; return g; }
    if (is_axis(p.entity))            { g.g = G::Line;  return g; }
    if (!valid(ents, p.entity))       return g;
    const SketchEntity& s = ents[p.entity];
    if (s.type == ET::Point) { g.g = G::Point; g.role = R::P0; return g; }
    if (p.point) {
        R r = p.role;
        if (s.type == ET::Circle || s.type == ET::Ellipse) r = R::Center;
        Vec2d tmp;
        if (!point_of(ents, p.entity, r, tmp)) return g;
        g.g = G::Point;
        g.role = r;
        return g;
    }
    switch (s.type) {
    case ET::Line:   g.g = G::Line;  break;
    case ET::Circle:
    case ET::Arc:    g.g = G::Round; g.edge = p.edge; break;
    default:         break;   // an ellipse or a spline picked whole has no single quantity
    }
    return g;
}

void swap_refs(SketchDimension& d)
{
    std::swap(d.ea, d.eb);
    std::swap(d.ra, d.rb);
}

// The two-point rule shared by a lone line and every point pair: aligned inside the band
// perpendicular to P-Q between them, Horizontal/Vertical outside it by the dominant cursor
// offset. H/V references are ordered so the measured delta is positive.
bool resolve_linear(const Vec2d& P, const Vec2d& Q, const Vec2d& cursor, bool own_line, SketchDimension& d)
{
    const Vec2d  dv = Q - P;
    const double L  = dv.norm();
    if (L < kEps) return false;
    const bool  horiz   = std::abs(dv.y()) <= kAxisTol * L;
    const bool  vert    = std::abs(dv.x()) <= kAxisTol * L;
    const Vec2d v       = cursor - 0.5 * (P + Q);
    const bool  in_band = std::abs(v.dot(dv / L)) <= 0.5 * L;
    if (horiz)        d.kind = K::Horizontal;
    else if (vert)    d.kind = K::Vertical;
    else if (in_band) d.kind = own_line ? K::Length : K::PointPoint;
    else              d.kind = (std::abs(v.y()) >= std::abs(v.x())) ? K::Horizontal : K::Vertical;
    if ((d.kind == K::Horizontal && dv.x() < 0.0) || (d.kind == K::Vertical && dv.y() < 0.0))
        swap_refs(d);
    return true;
}

bool intersect(const Vec2d& a0, const Vec2d& a1, const Vec2d& b0, const Vec2d& b1, Vec2d& X)
{
    const Vec2d  dA  = a1 - a0, dB = b1 - b0;
    const double den = cross2(dA, dB);
    if (std::abs(den) <= kParallel * dA.norm() * dB.norm() || std::abs(den) < kEps) return false;
    X = a0 + dA * (cross2(b0 - a0, dB) / den);
    return true;
}

double wrap_pi(double a)
{
    while (a > M_PI)   a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

std::string fmt_num(double v)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.2f", v);
    std::string s(buf);
    // wx sets LC_NUMERIC to the user locale, so %f may write a decimal comma.
    for (char& c : s)
        if (c == ',') c = '.';
    if (s.find('.') != std::string::npos) {
        while (!s.empty() && s.back() == '0') s.pop_back();
        if (!s.empty() && s.back() == '.') s.pop_back();
    }
    if (s == "-0") s = "0";
    return s;
}

// ---- layout helpers ---------------------------------------------------------------------------

void add_arrow(SketchDimLayout& L, const Vec2d& tip, const Vec2d& to_body, const SketchDimStyle& st)
{
    const Vec2d n    = perp(to_body) * (0.5 * st.arrow_width);
    const Vec2d base = tip + to_body * st.arrow_len;
    L.arrows.push_back({tip, base + n, base - n});
}

void ext_from_point(SketchDimLayout& L, const Vec2d& P, const Vec2d& to, const SketchDimStyle& st)
{
    Vec2d e = to - P;
    const double len = e.norm();
    if (len <= st.ext_gap) return;
    e /= len;
    L.ext_lines.emplace_back(P + e * st.ext_gap, to + e * st.ext_overshoot);
}

// A line anchor needs an extension only where the foot lies beyond its segment; the axes are
// infinite and never do.
void ext_from_segment(SketchDimLayout& L, int e, const Vec2d& a, const Vec2d& b, const Vec2d& to,
                      const SketchDimStyle& st)
{
    if (is_axis(e)) return;
    const Vec2d  d  = b - a;
    const double n2 = d.squaredNorm();
    if (n2 < kEps) return;
    const double t = (to - a).dot(d) / n2;
    if (t >= 0.0 && t <= 1.0) return;
    ext_from_point(L, t < 0.0 ? a : b, to, st);
}

void tessellate_arc(std::vector<std::pair<Vec2d, Vec2d>>& out, const Vec2d& c, double r,
                    double a0, double a1)
{
    const double sweep = a1 - a0;
    const int    n     = std::max(8, int(std::ceil(std::abs(sweep) / (5.0 * M_PI / 180.0))));
    Vec2d prev = c + r * Vec2d(std::cos(a0), std::sin(a0));
    for (int i = 1; i <= n; ++i) {
        const double a   = a0 + sweep * double(i) / double(n);
        const Vec2d  cur = c + r * Vec2d(std::cos(a), std::sin(a));
        out.emplace_back(prev, cur);
        prev = cur;
    }
}

// Dimension line between the two arrow tips A and B, extended to the text; arrows inside when
// the span has room for them, otherwise outside pointing in, with the text moved out of the way.
void linear_layout(SketchDimLayout& L, const Vec2d& A, const Vec2d& B, const Vec2d& T,
                   const SketchDimStyle& st)
{
    L.ok   = true;
    L.text = T;
    const Vec2d  d   = B - A;
    const double len = d.norm();
    if (len < kEps) return;   // zero distance: extension lines and text only
    const Vec2d u = d / len;
    L.arrows_inside = len >= 2.5 * st.arrow_len;
    double s = (T - A).dot(u);
    if (!L.arrows_inside && s > -0.5 * st.text_width && s < len + 0.5 * st.text_width) {
        const double out = 2.5 * st.arrow_len + 0.5 * st.text_width;
        const Vec2d  off = (T - A) - u * s;               // T's offset across the line (0 here)
        s      = (s < 0.5 * len) ? -out : len + out;
        L.text = A + u * s + off;
    }
    double lo = L.arrows_inside ? 0.0 : -2.0 * st.arrow_len;
    double hi = L.arrows_inside ? len : len + 2.0 * st.arrow_len;
    lo = std::min(lo, s);
    hi = std::max(hi, s);
    L.dim_lines.emplace_back(A + u * lo, A + u * hi);
    if (L.arrows_inside) { add_arrow(L, A, u, st);  add_arrow(L, B, -u, st); }
    else                 { add_arrow(L, A, -u, st); add_arrow(L, B, u, st);  }
}

} // namespace

bool sketch_dim_line_ends(const std::vector<SketchEntity>& ents, int e, Vec2d& a, Vec2d& b)
{
    if (e == kSketchRefAxisX) { a = Vec2d(1.0, 0.0); b = Vec2d::Zero(); return true; }
    if (e == kSketchRefAxisY) { a = Vec2d(0.0, 1.0); b = Vec2d::Zero(); return true; }
    if (!valid(ents, e) || ents[e].type != ET::Line) return false;
    a = ents[e].p0;
    b = ents[e].p1;
    return (b - a).squaredNorm() > kEps * kEps;
}

SmartDimResolution resolve_smart_dimension(const std::vector<SketchEntity>& ents,
                                           const SmartDimPick& first,
                                           const std::optional<SmartDimPick>& second,
                                           const Vec2d& cursor)
{
    SmartDimResolution res;
    SketchDimension& d = res.dim;
    d.text_pos = cursor;
    const Geo g1 = classify(ents, first);
    if (g1.g == G::None) return res;

    auto finish = [&]() {
        double v = 0.0;
        res.ok  = measure_sketch_dimension(ents, d, v);
        d.value = v;
        return res;
    };

    if (!second || *second == first) {
        if (g1.g == G::Line && !is_axis(g1.e)) {
            Vec2d a, b;
            if (!sketch_dim_line_ends(ents, g1.e, a, b)) return res;
            d.ea = d.eb = g1.e;
            d.ra = R::P0;
            d.rb = R::P1;
            if (!resolve_linear(a, b, cursor, true, d)) return res;
            return finish();
        }
        if (g1.g == G::Round) {
            d.ea   = g1.e;
            d.kind = (ents[g1.e].type == ET::Circle) ? K::Diameter : K::Radius;
            return finish();
        }
        return res;   // a lone point / axis measures nothing
    }

    Geo g2 = classify(ents, *second);
    if (g2.g == G::None) return res;
    Geo ga = g1, gb = g2;

    // Point-like views: a point, or the centre of a circle/arc.
    auto as_point = [](const Geo& g, int& e, R& r) {
        if (g.g == G::Point) { e = g.e; r = g.role; return true; }
        if (g.g == G::Round) { e = g.e; r = R::Center; return true; }
        return false;
    };

    if (ga.g == G::Line && gb.g == G::Line) {
        Vec2d a0, a1, b0, b1;
        if (!sketch_dim_line_ends(ents, ga.e, a0, a1) || !sketch_dim_line_ends(ents, gb.e, b0, b1))
            return res;
        if (ga.e == gb.e) return res;
        const Vec2d uA = (a1 - a0).normalized(), uB = (b1 - b0).normalized();
        d.ea = ga.e;
        d.eb = gb.e;
        if (std::abs(cross2(uA, uB)) < kParallel) {
            d.kind = K::LineLine;
            return finish();
        }
        Vec2d X;
        if (!intersect(a0, a1, b0, b1, X)) return res;
        const Vec2d  dA = a1 - a0, dB = b1 - b0;
        const double den = cross2(dA, dB);
        const Vec2d  v = cursor - X;
        const double alpha = cross2(v, dB) / den;   // v = alpha*dA + beta*dB
        const double beta  = cross2(dA, v) / den;
        d.kind   = K::Angle;
        d.sector = (alpha < 0.0 ? 1 : 0) | (beta < 0.0 ? 2 : 0);
        return finish();
    }

    // A line and something point-like: perpendicular distance, point first.
    if (ga.g == G::Line || gb.g == G::Line) {
        const Geo& gl = (ga.g == G::Line) ? ga : gb;
        const Geo& gp = (ga.g == G::Line) ? gb : ga;
        int pe; R pr;
        if (!as_point(gp, pe, pr)) return res;
        Vec2d a, b;
        if (!sketch_dim_line_ends(ents, gl.e, a, b)) return res;
        d.kind = K::PointLine;
        d.ea = pe; d.ra = pr;
        d.eb = gl.e;
        if (gp.g == G::Round && gp.edge) d.sector = 1;
        return finish();
    }

    int e1, e2; R r1, r2;
    if (!as_point(ga, e1, r1) || !as_point(gb, e2, r2)) return res;
    if (e1 == e2 && r1 == r2) return res;
    Vec2d P, Q;
    if (!point_of(ents, e1, r1, P) || !point_of(ents, e2, r2, Q)) return res;
    d.ea = e1; d.ra = r1;
    d.eb = e2; d.rb = r2;
    const bool edge1 = ga.g == G::Round && ga.edge, edge2 = gb.g == G::Round && gb.edge;
    if (edge1 || edge2) {
        // To a circle's edge: always the aligned (minimum) distance.
        if ((Q - P).norm() < kEps) return res;
        d.kind   = K::PointPoint;
        d.sector = (edge1 ? 1 : 0) | (edge2 ? 2 : 0);
        return finish();
    }
    if (!resolve_linear(P, Q, cursor, false, d)) return res;
    return finish();
}

bool measure_sketch_dimension(const std::vector<SketchEntity>& ents, const SketchDimension& d, double& value)
{
    value = 0.0;
    switch (d.kind) {
    case K::Length: {
        Vec2d a, b;
        if (!sketch_dim_line_ends(ents, d.ea, a, b) || is_axis(d.ea)) return false;
        value = (b - a).norm();
        return true;
    }
    case K::Horizontal:
    case K::Vertical:
    case K::PointPoint: {
        Vec2d P, Q;
        if (!point_of(ents, d.ea, d.ra, P) || !point_of(ents, d.eb, d.rb, Q)) return false;
        value = (d.kind == K::Horizontal) ? std::abs(Q.x() - P.x())
              : (d.kind == K::Vertical)   ? std::abs(Q.y() - P.y())
                                          : (Q - P).norm() - edge_offset(ents, d);
        return true;
    }
    case K::Diameter:
    case K::Radius: {
        Vec2d c; double r;
        if (!round_of(ents, d.ea, c, r)) return false;
        value = (d.kind == K::Diameter) ? 2.0 * r : r;
        return true;
    }
    case K::Angle: {
        Vec2d a0, a1, b0, b1;
        if (!sketch_dim_line_ends(ents, d.ea, a0, a1) || !sketch_dim_line_ends(ents, d.eb, b0, b1))
            return false;
        const double sa = (d.sector & 1) ? -1.0 : 1.0;
        const double sb = (d.sector & 2) ? -1.0 : 1.0;
        const Vec2d  uA = sa * (a1 - a0).normalized(), uB = sb * (b1 - b0).normalized();
        value = std::acos(std::clamp(uA.dot(uB), -1.0, 1.0)) * 180.0 / M_PI;
        return true;
    }
    case K::PointLine: {
        Vec2d P, a, b;
        if (!point_of(ents, d.ea, d.ra, P) || !sketch_dim_line_ends(ents, d.eb, a, b)) return false;
        value = std::abs(cross2((b - a).normalized(), P - a)) - edge_offset(ents, d);
        return true;
    }
    case K::LineLine: {
        Vec2d a0, a1, b0, b1;
        if (!sketch_dim_line_ends(ents, d.ea, a0, a1) || !sketch_dim_line_ends(ents, d.eb, b0, b1))
            return false;
        value = std::abs(cross2((b1 - b0).normalized(), a0 - b0));
        return true;
    }
    }
    return false;
}

double sketch_angle_constraint_value(double sector_deg, int sector)
{
    const bool   same = ((sector & 1) != 0) == ((sector & 2) != 0);
    const double rad  = sector_deg * M_PI / 180.0;
    return same ? rad : M_PI - rad;
}

std::optional<SketchEntityConstraintDef> sketch_dimension_constraint(
    const std::vector<SketchEntity>& ents, const SketchDimension& d, double value)
{
    using CT = SketchConstraintType;
    if (!std::isfinite(value) || value < 0.0) return std::nullopt;
    SketchEntityConstraintDef c;
    c.value = value;
    switch (d.kind) {
    case K::Length:
        if (value <= kEps || !valid(ents, d.ea)) return std::nullopt;
        c.type = CT::Distance;
        c.ea = d.ea; c.ra = R::P0;
        c.eb = d.ea; c.rb = R::P1;
        return c;
    case K::Horizontal:
    case K::Vertical: {
        Vec2d P, Q;
        if (!point_of(ents, d.ea, d.ra, P) || !point_of(ents, d.eb, d.rb, Q)) return std::nullopt;
        c.type = (d.kind == K::Horizontal) ? CT::DistanceX : CT::DistanceY;
        c.ea = d.ea; c.ra = d.ra;
        c.eb = d.eb; c.rb = d.rb;
        // DistanceX/Y are SIGNED: order the references so the typed (positive) value keeps the
        // points on the side they are on now. Accepting a dimension must never flip geometry.
        const double delta = (d.kind == K::Horizontal) ? Q.x() - P.x() : Q.y() - P.y();
        if (delta < 0.0) { std::swap(c.ea, c.eb); std::swap(c.ra, c.rb); }
        return c;
    }
    case K::PointPoint: {
        Vec2d P, Q;
        if (!point_of(ents, d.ea, d.ra, P) || !point_of(ents, d.eb, d.rb, Q)) return std::nullopt;
        // An edge distance drives the centre distance: value + the radii as they are now.
        // ponytail: the radius is captured, not tied; after a radius change, re-typing the value
        // re-captures it (a tied version needs an auxiliary point on the circle).
        c.value = value + edge_offset(ents, d);
        c.type  = (c.value < kEps) ? CT::Coincident : CT::Distance;
        if (c.value < kEps) c.value = 0.0;
        c.ea = d.ea; c.ra = d.ra;
        c.eb = d.eb; c.rb = d.rb;
        return c;
    }
    case K::Diameter:
    case K::Radius: {
        Vec2d cc; double r;
        if (value <= kEps || !round_of(ents, d.ea, cc, r)) return std::nullopt;
        c.type = (d.kind == K::Diameter) ? CT::Diameter : CT::Radius;
        c.ea = d.ea;
        return c;
    }
    case K::Angle: {
        Vec2d a0, a1, b0, b1;
        if (!(value > kEps && value < 180.0 - kEps)) return std::nullopt;
        if (!sketch_dim_line_ends(ents, d.ea, a0, a1) || !sketch_dim_line_ends(ents, d.eb, b0, b1))
            return std::nullopt;
        c.type  = CT::Angle;
        c.ea    = d.ea;
        c.eb    = d.eb;
        c.value = sketch_angle_constraint_value(value, d.sector);   // radians, solver convention
        return c;
    }
    case K::PointLine: {
        Vec2d P, a, b;
        if (!point_of(ents, d.ea, d.ra, P) || !sketch_dim_line_ends(ents, d.eb, a, b)) return std::nullopt;
        c.type  = CT::PointOnLine;
        c.value = value + edge_offset(ents, d);   // an edge distance holds the centre off by +r
        c.ea = d.ea; c.ra = d.ra;
        c.eb = d.eb;
        return c;
    }
    case K::LineLine: {
        Vec2d a0, a1, b0, b1;
        if (!sketch_dim_line_ends(ents, d.ea, a0, a1) || !sketch_dim_line_ends(ents, d.eb, b0, b1))
            return std::nullopt;
        // Distance between parallel lines = one line's point held off the other line.
        c.type = CT::PointOnLine;
        c.ea = d.ea; c.ra = R::P0;
        c.eb = d.eb;
        return c;
    }
    }
    return std::nullopt;
}

std::string format_sketch_dimension(const SketchDimension& d, double value)
{
    std::string s = fmt_num(value);
    if (d.kind == K::Diameter)    s = "\xC3\x98" + s;   // U+00D8: U+2300 is not in the UI font atlas
    else if (d.kind == K::Radius) s = "R" + s;
    else if (d.kind == K::Angle)  s += "\xC2\xB0";      // degree sign
    if (d.driven) s = "(" + s + ")";
    return s;
}

SketchDimLayout layout_sketch_dimension(const std::vector<SketchEntity>& ents,
                                        const SketchDimension& d, const SketchDimStyle& st)
{
    SketchDimLayout L;
    const Vec2d& T = d.text_pos;
    switch (d.kind) {
    case K::Length:
    case K::Horizontal:
    case K::Vertical:
    case K::PointPoint: {
        Vec2d P, Q;
        if (d.kind == K::Length) {
            if (!sketch_dim_line_ends(ents, d.ea, P, Q) || is_axis(d.ea)) return L;
        } else if (!point_of(ents, d.ea, d.ra, P) || !point_of(ents, d.eb, d.rb, Q)) {
            return L;
        }
        Vec2d u;
        if (d.kind == K::Horizontal)    u = Vec2d(1.0, 0.0);
        else if (d.kind == K::Vertical) u = Vec2d(0.0, 1.0);
        else {
            if ((Q - P).norm() < kEps) return L;
            u = (Q - P).normalized();
        }
        if (d.kind == K::PointPoint) {   // edge references: anchor on the circles' facing points
            if (d.sector & 1) P += u * round_radius(ents, d.ea, d.ra);
            if (d.sector & 2) Q -= u * round_radius(ents, d.eb, d.rb);
        }
        const Vec2d n = perp(u);
        const Vec2d A = P + n * (T - P).dot(n);
        const Vec2d B = Q + n * (T - Q).dot(n);
        ext_from_point(L, P, A, st);
        ext_from_point(L, Q, B, st);
        linear_layout(L, A, B, T, st);
        return L;
    }
    case K::PointLine: {
        Vec2d P, a, b;
        if (!point_of(ents, d.ea, d.ra, P) || !sketch_dim_line_ends(ents, d.eb, a, b)) return L;
        const Vec2d w = (b - a).normalized();
        const Vec2d F = a + w * (P - a).dot(w);            // foot of P on the line
        if ((d.sector & 1) && (P - F).norm() > kEps)       // edge: the circle's point nearest the line
            P -= (P - F).normalized() * round_radius(ents, d.ea, d.ra);
        const Vec2d A = F + w * (T - F).dot(w);            // dimension line runs across the line,
        const Vec2d B = P + w * (T - P).dot(w);            // through the text
        ext_from_segment(L, d.eb, a, b, A, st);
        ext_from_point(L, P, B, st);
        linear_layout(L, A, B, T, st);
        return L;
    }
    case K::LineLine: {
        Vec2d a0, a1, b0, b1;
        if (!sketch_dim_line_ends(ents, d.ea, a0, a1) || !sketch_dim_line_ends(ents, d.eb, b0, b1))
            return L;
        const Vec2d w  = (a1 - a0).normalized();
        const Vec2d n0 = perp(w);
        const Vec2d A  = a0 + w * (T - a0).dot(w);
        const Vec2d B  = A + n0 * (b0 - a0).dot(n0);
        ext_from_segment(L, d.ea, a0, a1, A, st);
        ext_from_segment(L, d.eb, b0, b1, B, st);
        linear_layout(L, A, B, T, st);
        return L;
    }
    case K::Diameter:
    case K::Radius: {
        Vec2d c; double r;
        if (!round_of(ents, d.ea, c, r)) return L;
        const Vec2d  v   = T - c;
        const double dl  = v.norm();
        const Vec2d  dir = dl > kEps ? Vec2d(v / dl) : Vec2d(1.0, 0.0);
        const Vec2d  R2  = c + dir * r;
        const Vec2d  far = dl > r ? T : R2;                 // the leader runs on to the text
        L.text = T;
        if (d.kind == K::Diameter) {
            const Vec2d R1 = c - dir * r;
            L.dim_lines.emplace_back(R1, far);
            add_arrow(L, R1, dir, st);                      // heads on the rim, pointing out
            add_arrow(L, R2, -dir, st);
        } else {
            L.dim_lines.emplace_back(c, far);
            add_arrow(L, R2, -dir, st);                     // head on the arc, from the centre
        }
        // An arc dimensioned outside its sweep gets the arc continued to the arrow.
        const SketchEntity& e = ents[d.ea];
        if (e.type == ET::Arc) {
            const double sweep = e.end_angle - e.start_angle;
            const double phi   = std::atan2(dir.y(), dir.x());
            double rel = sweep >= 0.0 ? phi - e.start_angle : e.start_angle - phi;
            rel = std::fmod(rel, 2.0 * M_PI);
            if (rel < 0.0) rel += 2.0 * M_PI;
            if (rel > std::abs(sweep) + 1e-9) {
                const double to_s = wrap_pi(phi - e.start_angle);
                const double to_e = wrap_pi(phi - e.end_angle);
                const double from = (std::abs(to_s) <= std::abs(to_e)) ? e.start_angle : e.end_angle;
                const double gap  = (std::abs(to_s) <= std::abs(to_e)) ? to_s : to_e;
                tessellate_arc(L.ext_lines, c, r, from, from + gap);
            }
        }
        L.ok = true;
        return L;
    }
    case K::Angle: {
        Vec2d a0, a1, b0, b1, X;
        if (!sketch_dim_line_ends(ents, d.ea, a0, a1) || !sketch_dim_line_ends(ents, d.eb, b0, b1))
            return L;
        if (!intersect(a0, a1, b0, b1, X)) return L;
        const double sa = (d.sector & 1) ? -1.0 : 1.0;
        const double sb = (d.sector & 2) ? -1.0 : 1.0;
        const Vec2d  uA = sa * (a1 - a0).normalized();
        const Vec2d  uB = sb * (b1 - b0).normalized();
        double rho = (T - X).norm();
        if (rho < kEps) rho = 4.0 * st.arrow_len;
        const double delta = std::atan2(cross2(uA, uB), uA.dot(uB));   // signed sector, |delta| < pi
        const double sgn   = delta >= 0.0 ? 1.0 : -1.0;
        const double phiA  = std::atan2(uA.y(), uA.x());
        const double width = std::abs(delta);
        // Arc over the sector, continued to the text when the text sits outside it.
        double k = sgn * wrap_pi(std::atan2((T - X).y(), (T - X).x()) - phiA);
        if (k < 0.5 * width - M_PI)  k += 2.0 * M_PI;
        if (k > 0.5 * width + M_PI)  k -= 2.0 * M_PI;
        const double lo = std::min(0.0, k), hi = std::max(width, k);
        tessellate_arc(L.dim_lines, X, rho, phiA + sgn * lo, phiA + sgn * hi);
        // Extension lines along each arm up to the arc (none for the infinite axes).
        auto arm_ext = [&](int e, const Vec2d& p0, const Vec2d& p1, const Vec2d& u) {
            if (is_axis(e)) return;
            const double t0 = (p0 - X).dot(u), t1 = (p1 - X).dot(u);
            const double tmax = std::max(t0, t1), tmin = std::min(t0, t1);
            if (tmax < rho - st.ext_gap) {
                const double from = std::max(tmax, 0.0) + st.ext_gap;
                if (from < rho + st.ext_overshoot)
                    L.ext_lines.emplace_back(X + u * from, X + u * (rho + st.ext_overshoot));
            } else if (tmin > rho + st.ext_gap) {
                L.ext_lines.emplace_back(X + u * (tmin - st.ext_gap), X + u * std::max(0.0, rho - st.ext_overshoot));
            }
        };
        arm_ext(d.ea, a0, a1, uA);
        arm_ext(d.eb, b0, b1, uB);
        const Vec2d tipA = X + uA * rho, tipB = X + uB * rho;
        const Vec2d tA = perp(uA) * sgn;    // along the arc, into the sector
        const Vec2d tB = -perp(uB) * sgn;
        L.arrows_inside = rho * width >= 2.5 * st.arrow_len;
        if (L.arrows_inside) {
            add_arrow(L, tipA, tA, st);
            add_arrow(L, tipB, tB, st);
        } else {
            add_arrow(L, tipA, -tA, st);
            add_arrow(L, tipB, -tB, st);
            L.dim_lines.emplace_back(tipA, tipA - tA * (2.0 * st.arrow_len));
            L.dim_lines.emplace_back(tipB, tipB - tB * (2.0 * st.arrow_len));
        }
        L.text = T;
        L.ok   = true;
        return L;
    }
    }
    return L;
}

void sketch_dimensions_remap_entities(std::vector<SketchDimension>& dims, const std::vector<int>& remap)
{
    std::vector<SketchDimension> kept;
    kept.reserve(dims.size());
    for (SketchDimension d : dims) {
        bool dead = false;
        auto map = [&](int e) {
            if (e < 0) return e;                       // unset, origin or an axis
            if (e >= int(remap.size()) || remap[e] < 0) { dead = true; return -1; }
            return remap[e];
        };
        d.ea = map(d.ea);
        d.eb = map(d.eb);
        if (!dead) kept.push_back(d);
    }
    dims.swap(kept);
}

void sketch_dimensions_remap_constraints(std::vector<SketchDimension>& dims, const std::vector<int>& remap)
{
    for (SketchDimension& d : dims) {
        if (d.constraint < 0) continue;
        const int n = d.constraint < int(remap.size()) ? remap[d.constraint] : -1;
        if (n < 0) { d.driven = true; d.constraint = -1; }
        else       d.constraint = n;
    }
}

void sketch_dimensions_erase_constraint(std::vector<SketchDimension>& dims, int idx)
{
    for (SketchDimension& d : dims) {
        if (d.constraint == idx)     { d.driven = true; d.constraint = -1; }
        else if (d.constraint > idx) --d.constraint;
    }
}

void sketch_dimensions_sanitize(std::vector<SketchDimension>& dims, int n_entities, int n_constraints)
{
    std::vector<SketchDimension> kept;
    kept.reserve(dims.size());
    for (SketchDimension d : dims) {
        if (d.ea >= n_entities || d.eb >= n_entities || int(d.kind) < 0 || int(d.kind) > int(K::LineLine))
            continue;
        if (d.constraint >= n_constraints) d.constraint = -1;
        if (d.constraint < 0) d.driven = true;
        kept.push_back(d);
    }
    dims.swap(kept);
}

} // namespace Slic3r

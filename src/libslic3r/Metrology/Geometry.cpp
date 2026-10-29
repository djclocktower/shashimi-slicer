#include "Geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace Slic3r::Metrology {

bool solve_linear(std::vector<double> A, std::vector<double> b, int n, std::vector<double>& x)
{
    for (int col = 0; col < n; ++col) {
        int    piv  = col;
        double best = std::abs(A[size_t(col * n + col)]);
        for (int r = col + 1; r < n; ++r)
            if (std::abs(A[size_t(r * n + col)]) > best) {
                best = std::abs(A[size_t(r * n + col)]);
                piv  = r;
            }
        if (best < 1e-14) return false;
        if (piv != col) {
            for (int c = 0; c < n; ++c) std::swap(A[size_t(col * n + c)], A[size_t(piv * n + c)]);
            std::swap(b[size_t(col)], b[size_t(piv)]);
        }
        for (int r = col + 1; r < n; ++r) {
            const double f = A[size_t(r * n + col)] / A[size_t(col * n + col)];
            if (f == 0) continue;
            for (int c = col; c < n; ++c) A[size_t(r * n + c)] -= f * A[size_t(col * n + c)];
            b[size_t(r)] -= f * b[size_t(col)];
        }
    }
    x.assign(size_t(n), 0.0);
    for (int r = n - 1; r >= 0; --r) {
        double s = b[size_t(r)];
        for (int c = r + 1; c < n; ++c) s -= A[size_t(r * n + c)] * x[size_t(c)];
        x[size_t(r)] = s / A[size_t(r * n + r)];
    }
    return true;
}

// Hartley normalization: centroid to the origin, mean distance sqrt(2).
static Mat3 normalizer(const std::vector<P2>& pts)
{
    P2 c;
    for (const P2& p : pts) c += p;
    c = c / double(pts.size());
    double d = 0;
    for (const P2& p : pts) d += (p - c).norm();
    d /= double(pts.size());
    const double s = d > 1e-12 ? std::sqrt(2.0) / d : 1.0;
    Mat3         T;
    T.m = {s, 0, -s * c.x, 0, s, -s * c.y, 0, 0, 1};
    return T;
}

bool fit_homography(const std::vector<P2>& src, const std::vector<P2>& dst, Mat3& H)
{
    const size_t n = src.size();
    if (n < 4 || dst.size() != n) return false;
    const Mat3 Ts = normalizer(src), Td = normalizer(dst);
    // Normal equations of the 2n x 8 system with h33 = 1.
    std::vector<double> AtA(64, 0.0), Atb(8, 0.0);
    for (size_t i = 0; i < n; ++i) {
        const P2 s = Ts.apply(src[i]), d = Td.apply(dst[i]);
        const double r1[8] = {s.x, s.y, 1, 0, 0, 0, -d.x * s.x, -d.x * s.y};
        const double r2[8] = {0, 0, 0, s.x, s.y, 1, -d.y * s.x, -d.y * s.y};
        for (int a = 0; a < 8; ++a) {
            for (int b = 0; b < 8; ++b) AtA[size_t(a * 8 + b)] += r1[a] * r1[b] + r2[a] * r2[b];
            Atb[size_t(a)] += r1[a] * d.x + r2[a] * d.y;
        }
    }
    std::vector<double> h;
    if (!solve_linear(AtA, Atb, 8, h)) return false;
    Mat3 Hn;
    Hn.m = {h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1.0};
    H = Td.inverse() * Hn * Ts;
    const double s = H.m[8];
    if (std::abs(s) > 1e-15)
        for (double& v : H.m) v /= s;
    return std::isfinite(H.m[0]) && std::abs(H.det()) > 1e-18;
}

LineFit fit_line(const std::vector<P2>& pts)
{
    LineFit f;
    f.n = int(pts.size());
    if (pts.size() < 2) return f;
    P2 c;
    for (const P2& p : pts) c += p;
    c = c / double(pts.size());
    double sxx = 0, syy = 0, sxy = 0;
    for (const P2& p : pts) {
        const P2 q = p - c;
        sxx += q.x * q.x;
        syy += q.y * q.y;
        sxy += q.x * q.y;
    }
    // Major eigenvector of the 2x2 scatter matrix.
    const double theta = 0.5 * std::atan2(2 * sxy, sxx - syy);
    P2           d{std::cos(theta), std::sin(theta)};
    if ((pts.back() - pts.front()).dot(d) < 0) d = d * -1.0;
    f.p = c;
    f.d = d;
    double ss = 0, mx = 0;
    for (const P2& p : pts) {
        const double r = f.distance(p);
        ss += r * r;
        mx = std::max(mx, std::abs(r));
    }
    f.rms     = std::sqrt(ss / double(pts.size()));
    f.max_dev = mx;
    f.ok      = (sxx + syy) > 1e-18;
    return f;
}

CircleFit fit_circle(const std::vector<P2>& pts)
{
    CircleFit f;
    f.n = int(pts.size());
    if (pts.size() < 3) return f;
    // Kasa algebraic fit on centred data: x^2 + y^2 + D x + E y + F = 0.
    P2 m;
    for (const P2& p : pts) m += p;
    m = m / double(pts.size());
    std::vector<double> A(9, 0.0), b(3, 0.0);
    for (const P2& p0 : pts) {
        const P2     p = p0 - m;
        const double z = p.x * p.x + p.y * p.y;
        const double row[3] = {p.x, p.y, 1};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) A[size_t(i * 3 + j)] += row[i] * row[j];
            b[size_t(i)] -= row[i] * z;
        }
    }
    std::vector<double> s;
    if (!solve_linear(A, b, 3, s)) return f;
    P2     c{-s[0] / 2, -s[1] / 2};
    double r2 = c.x * c.x + c.y * c.y - s[2];
    if (r2 <= 0) return f;
    double r = std::sqrt(r2);
    // Gauss-Newton on the geometric distance.
    for (int it = 0; it < 20; ++it) {
        std::vector<double> JtJ(9, 0.0), Jtr(3, 0.0);
        for (const P2& p0 : pts) {
            const P2     p  = p0 - m;
            const P2     dv = p - c;
            const double di = dv.norm();
            if (di < 1e-12) continue;
            const double res    = di - r;
            const double J[3]   = {-dv.x / di, -dv.y / di, -1};
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) JtJ[size_t(i * 3 + j)] += J[i] * J[j];
                Jtr[size_t(i)] -= J[i] * res;
            }
        }
        std::vector<double> delta;
        if (!solve_linear(JtJ, Jtr, 3, delta)) break;
        c.x += delta[0];
        c.y += delta[1];
        r += delta[2];
        if (std::abs(delta[0]) + std::abs(delta[1]) + std::abs(delta[2]) < 1e-12) break;
    }
    f.c = c + m;
    f.r = std::abs(r);
    double ss = 0;
    for (const P2& p : pts) {
        const double res = (p - f.c).norm() - f.r;
        ss += res * res;
    }
    f.rms = std::sqrt(ss / double(pts.size()));
    f.ok  = std::isfinite(f.r) && f.r > 0;
    return f;
}

// Least-median-of-squares start: fits through small samples spread over the points, keeping the
// one whose median residual is smallest, then the points within 2.5 robust sigmas of it.
template<class Fit, class FitFn, class ResFn>
static std::vector<P2> lmeds_inliers(const std::vector<P2>& pts, size_t sample, FitFn fit, ResFn residual)
{
    const size_t n = pts.size();
    if (n < sample * 3) return pts;
    double              best_med = std::numeric_limits<double>::max();
    Fit                 best;
    std::vector<double> res(n);
    const size_t        tries = std::min<size_t>(n, 60);
    for (size_t k = 0; k <= tries; ++k) {
        Fit f;
        if (k == tries) f = fit(pts);
        else {
            std::vector<P2> s;
            for (size_t q = 0; q < sample; ++q) s.push_back(pts[(k * n / tries + q * n / sample) % n]);
            f = fit(s);
        }
        if (!f.ok) continue;
        for (size_t i = 0; i < n; ++i) res[i] = std::abs(residual(f, pts[i]));
        std::vector<double> sorted = res;
        std::nth_element(sorted.begin(), sorted.begin() + n / 2, sorted.end());
        if (sorted[n / 2] < best_med) {
            best_med = sorted[n / 2];
            best     = f;
        }
    }
    if (!best.ok) return pts;
    const double    lim = std::max(2.5 * 1.4826 * best_med, 1e-9);
    std::vector<P2> in;
    for (const P2& p : pts)
        if (std::abs(residual(best, p)) <= lim) in.push_back(p);
    return in.size() >= sample ? in : pts;
}

template<class Fit, class FitFn, class ResFn>
static Fit fit_robust(const std::vector<P2>& pts0, double keep_frac, size_t min_n, FitFn fit, ResFn residual)
{
    const std::vector<P2> pts = lmeds_inliers<Fit>(pts0, min_n, fit, residual);
    std::vector<P2> cur = pts;
    Fit             f   = fit(cur);
    const size_t    floor_n = std::max(min_n, size_t(std::ceil(double(pts.size()) * keep_frac)));
    for (int it = 0; it < 5 && f.ok; ++it) {
        const double lim = std::max(2.5 * f.rms, 1e-9);
        std::vector<std::pair<double, P2>> scored;
        for (const P2& p : cur) scored.emplace_back(std::abs(residual(f, p)), p);
        std::sort(scored.begin(), scored.end(), [](auto& a, auto& b) { return a.first < b.first; });
        std::vector<P2> next;
        for (size_t i = 0; i < scored.size(); ++i)
            if (scored[i].first <= lim || next.size() < floor_n) next.push_back(scored[i].second);
        if (next.size() == cur.size()) break;
        cur = std::move(next);
        f   = fit(cur);
    }
    return f;
}

LineFit fit_line_robust(const std::vector<P2>& pts, double keep_frac)
{
    // Keep the input order so the direction convention (first -> last) survives.
    return fit_robust<LineFit>(pts, keep_frac, 2, [&pts](const std::vector<P2>& v) {
            LineFit f = fit_line(v);
            if (f.ok && pts.size() >= 2 && (pts.back() - pts.front()).dot(f.d) < 0) f.d = f.d * -1.0;
            return f;
        }, [](const LineFit& f, const P2& p) { return f.distance(p); });
}

CircleFit fit_circle_robust(const std::vector<P2>& pts, double keep_frac)
{
    return fit_robust<CircleFit>(pts, keep_frac, 3, [](const std::vector<P2>& v) { return fit_circle(v); },
                                 [](const CircleFit& f, const P2& p) { return (p - f.c).norm() - f.r; });
}

double line_angle_deg(const LineFit& a, const LineFit& b)
{
    double c = std::abs(a.d.dot(b.d));
    c        = std::min(1.0, c);
    return std::acos(c) * 180.0 / kPi;
}

double line_direction_deg(const LineFit& a)
{
    double ang = std::atan2(a.d.y, a.d.x) * 180.0 / kPi;
    if (ang <= -90) ang += 180;
    if (ang > 90) ang -= 180;
    return ang;
}

double signed_area(const Polygon2& poly)
{
    double a = 0;
    for (size_t i = 0, n = poly.size(); i < n; ++i) a += poly[i].cross(poly[(i + 1) % n]);
    return 0.5 * a;
}

bool point_in_polygon(const Polygon2& poly, const P2& p)
{
    bool in = false;
    for (size_t i = 0, n = poly.size(), j = n - 1; i < n; j = i++) {
        const P2& a = poly[i];
        const P2& b = poly[j];
        if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
    }
    return in;
}

bool point_in_region(const Region& r, const P2& p)
{
    if (!point_in_polygon(r.outer, p)) return false;
    for (const Polygon2& h : r.holes)
        if (point_in_polygon(h, p)) return false;
    return true;
}

double segment_distance(const P2& a, const P2& b, const P2& p)
{
    const P2     ab = b - a;
    const double l2 = ab.dot(ab);
    double       t  = l2 > 0 ? (p - a).dot(ab) / l2 : 0;
    t               = std::min(1.0, std::max(0.0, t));
    return (a + ab * t - p).norm();
}

static double loop_distance(const Polygon2& poly, const P2& p)
{
    double d = std::numeric_limits<double>::max();
    for (size_t i = 0, n = poly.size(); i < n; ++i) d = std::min(d, segment_distance(poly[i], poly[(i + 1) % n], p));
    return d;
}

double signed_distance(const std::vector<Region>& regions, const P2& p)
{
    double d      = std::numeric_limits<double>::max();
    bool   inside = false;
    for (const Region& r : regions) {
        d = std::min(d, loop_distance(r.outer, p));
        for (const Polygon2& h : r.holes) d = std::min(d, loop_distance(h, p));
        if (point_in_region(r, p)) inside = true;
    }
    if (d == std::numeric_limits<double>::max()) return -1e9;
    return inside ? d : -d;
}

double signed_distance(const Region& r, const P2& p)
{
    double d = loop_distance(r.outer, p);
    for (const Polygon2& h : r.holes) d = std::min(d, loop_distance(h, p));
    if (r.outer.empty()) return -1e9;
    return point_in_region(r, p) ? d : -d;
}

static void dp_open(const Polygon2& pts, size_t a, size_t b, double tol, std::vector<char>& keep)
{
    if (b <= a + 1) return;
    double best = -1;
    size_t idx  = a;
    for (size_t i = a + 1; i < b; ++i) {
        const double d = segment_distance(pts[a], pts[b], pts[i]);
        if (d > best) {
            best = d;
            idx  = i;
        }
    }
    if (best > tol) {
        keep[idx] = 1;
        dp_open(pts, a, idx, tol, keep);
        dp_open(pts, idx, b, tol, keep);
    }
}

Polygon2 simplify_closed(const Polygon2& poly, double tol)
{
    const size_t n = poly.size();
    if (n < 4 || tol <= 0) return poly;
    // Split at the vertex farthest from vertex 0 so both halves are open chains.
    size_t far = 0;
    double fd  = -1;
    for (size_t i = 1; i < n; ++i) {
        const double d = (poly[i] - poly[0]).norm();
        if (d > fd) {
            fd  = d;
            far = i;
        }
    }
    Polygon2 chain(poly.begin(), poly.end());
    chain.push_back(poly[0]);
    std::vector<char> keep(chain.size(), 0);
    keep[0] = keep[far] = keep[chain.size() - 1] = 1;
    dp_open(chain, 0, far, tol, keep);
    dp_open(chain, far, chain.size() - 1, tol, keep);
    Polygon2 out;
    for (size_t i = 0; i + 1 < chain.size(); ++i)
        if (keep[i]) out.push_back(chain[i]);
    return out.size() >= 3 ? out : poly;
}

static Polygon2 offset_loop(const Polygon2& loop, double d)
{
    Polygon2     out;
    const size_t n = loop.size();
    for (size_t k = 0; k < n; ++k) {
        const P2 d1 = (loop[k] - loop[(k + n - 1) % n]).normalized(), d2 = (loop[(k + 1) % n] - loop[k]).normalized();
        // Right-hand normals point out of the material for both outer loops and holes.
        const P2     n1{d1.y, -d1.x}, n2{d2.y, -d2.x};
        const P2     m = (n1 + n2).normalized();
        const double c = std::max(m.dot(n1), 0.3);
        out.push_back(loop[k] + m * (d / c));
    }
    return out;
}

std::vector<Region> offset_regions(const std::vector<Region>& regions, double d)
{
    std::vector<Region> out;
    for (const Region& r : regions) {
        Region o;
        o.outer = offset_loop(r.outer, d);
        for (const Polygon2& h : r.holes) o.holes.push_back(offset_loop(h, d));
        out.push_back(std::move(o));
    }
    return out;
}

std::vector<P2> convex_hull(std::vector<P2> pts)
{
    std::sort(pts.begin(), pts.end(), [](const P2& a, const P2& b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    if (pts.size() < 3) return pts;
    std::vector<P2> h(2 * pts.size());
    size_t          k = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        while (k >= 2 && (h[k - 1] - h[k - 2]).cross(pts[i] - h[k - 2]) <= 0) --k;
        h[k++] = pts[i];
    }
    for (size_t i = pts.size() - 1, t = k + 1; i > 0; --i) {
        while (k >= t && (h[k - 1] - h[k - 2]).cross(pts[i - 1] - h[k - 2]) <= 0) --k;
        h[k++] = pts[i - 1];
    }
    h.resize(k - 1);
    return h;
}

void bounding_box(const std::vector<Region>& regions, P2& lo, P2& hi)
{
    lo = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    hi = {-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()};
    for (const Region& r : regions)
        for (const P2& p : r.outer) {
            lo.x = std::min(lo.x, p.x);
            lo.y = std::min(lo.y, p.y);
            hi.x = std::max(hi.x, p.x);
            hi.y = std::max(hi.y, p.y);
        }
}

} // namespace Slic3r::Metrology

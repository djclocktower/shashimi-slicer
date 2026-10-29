#include "Edges.hpp"

#include <algorithm>
#include <cmath>

namespace Slic3r::Metrology {

namespace {

constexpr double kStep = 0.5;   // profile sampling step, px

std::vector<double> profile(const GrayImage& img, const P2& a, const P2& b, double avg_width, int& n)
{
    const P2     d   = b - a;
    const double len = d.norm();
    if (!std::isfinite(len) || len > 1e5) {
        n = 0;
        return {};
    }
    n                = std::max(2, int(len / kStep) + 1);
    const P2 u       = d / std::max(len, 1e-9);
    const P2 v       = u.perp();
    const int nw     = std::max(1, int(std::round(avg_width)));
    std::vector<double> prof(size_t(n), 0.0);
    for (int i = 0; i < n; ++i) {
        const P2 c = a + u * (i * kStep);
        double   s = 0;
        for (int k = 0; k < nw; ++k) {
            const double off = (k - (nw - 1) * 0.5);
            const P2     q   = c + v * off;
            s += img.sample(q.x, q.y);
        }
        prof[size_t(i)] = s / nw;
    }
    return prof;
}

std::vector<double> smooth(const std::vector<double>& p, double sigma_px)
{
    const double sigma = sigma_px / kStep;
    if (sigma <= 0.01) return p;
    const int           r = std::max(1, int(std::ceil(3 * sigma)));
    std::vector<double> k(size_t(2 * r + 1));
    double              sum = 0;
    for (int i = -r; i <= r; ++i) sum += k[size_t(i + r)] = std::exp(-0.5 * i * i / (sigma * sigma));
    std::vector<double> out(p.size());
    const int n = int(p.size());
    for (int i = 0; i < n; ++i) {
        double s = 0;
        for (int j = -r; j <= r; ++j) s += k[size_t(j + r)] * p[size_t(std::min(std::max(i + j, 0), n - 1))];
        out[size_t(i)] = s / sum;
    }
    return out;
}

} // namespace

std::vector<EdgeHit> find_edges(const GrayImage& img, const P2& a, const P2& b, const EdgeParams& prm)
{
    std::vector<EdgeHit> hits;
    if (img.empty()) return hits;
    int  n    = 0;
    auto prof = smooth(profile(img, a, b, prm.avg_width, n), prm.smooth_sigma);
    if (n < 5) return hits;
    // Gradient in grey levels per px; a step edge of contrast C integrates to C.
    std::vector<double> g(size_t(n), 0.0);
    for (int i = 1; i + 1 < n; ++i) g[size_t(i)] = (prof[size_t(i + 1)] - prof[size_t(i - 1)]) / (2 * kStep);
    const P2     u   = (b - a).normalized();
    const double sig = std::max(prm.smooth_sigma, 0.5);
    // A blurred step of contrast C peaks at about C / (sqrt(2 pi) sigma).
    const double peak_min = prm.min_contrast / (std::sqrt(2 * kPi) * std::sqrt(sig * sig + 0.25));
    for (int i = 2; i + 2 < n; ++i) {
        const double gi = g[size_t(i)];
        if (std::abs(gi) < peak_min) continue;
        if (prm.polarity == Polarity::DarkToLight && gi <= 0) continue;
        if (prm.polarity == Polarity::LightToDark && gi >= 0) continue;
        const double gl = g[size_t(i - 1)], gr = g[size_t(i + 1)];
        if (!(std::abs(gi) >= std::abs(gl) && std::abs(gi) > std::abs(gr))) continue;
        // Parabola through the three samples.
        const double den = gl - 2 * gi + gr;
        double       off = std::abs(den) > 1e-12 ? 0.5 * (gl - gr) / den : 0.0;
        off              = std::max(-0.5, std::min(0.5, off));
        const double t   = (i + off) * kStep;
        // Contrast across the edge from the smoothed profile a few sigma either side.
        const int    w   = std::max(2, int(std::ceil(2.5 * sig / kStep)));
        const double lo  = prof[size_t(std::max(0, i - w))], hi = prof[size_t(std::min(n - 1, i + w))];
        const double contrast = hi - lo;
        if (std::abs(contrast) < prm.min_contrast) continue;
        hits.push_back({a + u * t, contrast, t, u});
    }
    return hits;
}

bool find_edge(const GrayImage& img, const P2& a, const P2& b, const EdgeParams& prm, EdgeHit& out)
{
    auto hits = find_edges(img, a, b, prm);
    if (hits.empty()) return false;
    switch (prm.pick) {
    case EdgePick::First: out = hits.front(); break;
    case EdgePick::Last: out = hits.back(); break;
    default:
        out = *std::max_element(hits.begin(), hits.end(),
                                [](const EdgeHit& x, const EdgeHit& y) { return std::abs(x.strength) < std::abs(y.strength); });
    }
    return true;
}

std::array<P2, 4> OrientedRect::corners() const
{
    const P2 u = dir() * (length * 0.5), v = across() * (width * 0.5);
    return {center - u - v, center + u - v, center + u + v, center - u + v};
}

std::vector<P2> hit_points(const std::vector<EdgeHit>& hits)
{
    std::vector<P2> pts;
    for (const EdgeHit& h : hits) pts.push_back(h.p);
    return pts;
}

std::vector<EdgeHit> scan_rect(const GrayImage& img, const OrientedRect& rect, int n_lines, const EdgeParams& prm)
{
    std::vector<EdgeHit> pts;
    const P2        u = rect.dir(), v = rect.across();
    n_lines           = std::max(1, n_lines);
    for (int i = 0; i < n_lines; ++i) {
        const double f = n_lines == 1 ? 0.0 : (double(i) / (n_lines - 1) - 0.5) * 0.9;
        const P2     c = rect.center + v * (f * rect.width);
        EdgeHit      h;
        if (find_edge(img, c - u * (rect.length * 0.5), c + u * (rect.length * 0.5), prm, h)) pts.push_back(h);
    }
    return pts;
}

void scan_rect_pair(const GrayImage& img, const OrientedRect& rect, int n_lines, const EdgeParams& prm, std::vector<EdgeHit>& first,
                    std::vector<EdgeHit>& last)
{
    first.clear();
    last.clear();
    const P2 u = rect.dir(), v = rect.across();
    n_lines    = std::max(1, n_lines);
    for (int i = 0; i < n_lines; ++i) {
        const double f    = n_lines == 1 ? 0.0 : (double(i) / (n_lines - 1) - 0.5) * 0.9;
        const P2     c    = rect.center + v * (f * rect.width);
        auto         hits = find_edges(img, c - u * (rect.length * 0.5), c + u * (rect.length * 0.5), prm);
        if (hits.size() < 2) continue;
        first.push_back(hits.front());
        last.push_back(hits.back());
    }
}

std::vector<EdgeHit> scan_annulus(const GrayImage& img, const P2& center, double r_in, double r_out, int n_rays, const EdgeParams& prm)
{
    std::vector<EdgeHit> pts;
    n_rays = std::max(3, n_rays);
    for (int i = 0; i < n_rays; ++i) {
        const double a = 2 * kPi * i / n_rays;
        const P2     d{std::cos(a), std::sin(a)};
        EdgeHit      h;
        if (find_edge(img, center + d * r_in, center + d * r_out, prm, h)) pts.push_back(h);
    }
    return pts;
}

} // namespace Slic3r::Metrology

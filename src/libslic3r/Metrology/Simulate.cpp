#include "Simulate.hpp"

#include "Geometry.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace Slic3r::Metrology {

namespace {

struct Pinhole
{
    P3     C;
    P3     r, d, f;   // camera axes in the plate frame: right, image-down, forward
    double focal, cx, cy;
};

Pinhole pinhole(const SimCamera& cam, double offset)
{
    Pinhole ph;
    // Raising the bed by `offset` is the same as lowering the camera.
    ph.C        = cam.center - P3{0, 0, offset};
    const P3 tg = cam.target - P3{0, 0, offset};
    ph.f        = (tg - ph.C).normalized();
    ph.r        = ph.f.cross({0, 0, 1}).normalized();
    ph.d        = ph.f.cross(ph.r);
    ph.focal    = cam.focal_px;
    ph.cx       = 0.5 * (cam.w - 1);
    ph.cy       = 0.5 * (cam.h - 1);
    return ph;
}

double phi(double x) { return 0.5 * std::erfc(-x / std::sqrt(2.0)); }

double rect_sd(const P2& p, const P2& lo, const P2& hi)
{
    // Positive inside.
    return std::min(std::min(p.x - lo.x, hi.x - p.x), std::min(p.y - lo.y, hi.y - p.y));
}

} // namespace

P2 sim_project(const SimCamera& cam, const P3& p, double offset)
{
    const Pinhole ph = pinhole(cam, offset);
    const P3      q  = p - ph.C;
    const double  z  = q.dot(ph.f);
    return {ph.cx + ph.focal * q.dot(ph.r) / z, ph.cy + ph.focal * q.dot(ph.d) / z};
}

Mat3 sim_plate_homography(const SimCamera& cam, double offset)
{
    const Pinhole ph = pinhole(cam, offset);
    const P3      t  = P3{0, 0, 0} - ph.C;
    // pc = x R e1 + y R e2 + R (0 - C), rows of R are r, d, f.
    const double M[3][3] = {{ph.r.x, ph.r.y, ph.r.dot(t)}, {ph.d.x, ph.d.y, ph.d.dot(t)}, {ph.f.x, ph.f.y, ph.f.dot(t)}};
    Mat3         H;
    for (int c = 0; c < 3; ++c) {
        H(0, c) = ph.focal * M[0][c] + ph.cx * M[2][c];
        H(1, c) = ph.focal * M[1][c] + ph.cy * M[2][c];
        H(2, c) = M[2][c];
    }
    const double s = H(2, 2);
    for (double& v : H.m) v /= s;
    return H;
}

P2 sim_nadir(const SimCamera& cam) { return {cam.center.x, cam.center.y}; }

double sim_height(const SimScene& scene, const P2& p)
{
    double h = 0;
    for (const SimPrism& pr : scene.prisms)
        if (pr.height > h && point_in_region(pr.footprint, p)) h = pr.height;
    return h;
}

double sim_touch(const SimScene& scene, const P2& p, double tip_r)
{
    double h = sim_height(scene, p);
    for (int ring = 1; ring <= 3; ++ring)
        for (int k = 0; k < 24; ++k) {
            const double a = 2 * kPi * k / 24;
            h              = std::max(h, sim_height(scene, p + P2{std::cos(a), std::sin(a)} * (tip_r * ring / 3.0)));
        }
    return h;
}

GrayImage sim_render(const SimScene& scene, const SimCamera& cam, double offset, bool light_on, unsigned seed)
{
    const Pinhole ph = pinhole(cam, offset);
    GrayImage     img(cam.w, cam.h);
    std::mt19937  rng(seed);
    std::normal_distribution<double> nd(0.0, 1.0);
    const std::vector<P2> dots = scene.mat.dots();
    const P2 mlo = scene.mat.mat_min(), mhi = scene.mat.mat_max();
    struct Box { P2 lo, hi; };
    std::vector<Box> boxes;
    for (const SimPrism& pr : scene.prisms) {
        Box b;
        bounding_box({pr.footprint}, b.lo, b.hi);
        boxes.push_back(b);
    }
    std::vector<size_t> order(scene.prisms.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return scene.prisms[a].height < scene.prisms[b].height; });
    for (int v = 0; v < cam.h; ++v)
        for (int u = 0; u < cam.w; ++u) {
            const double xc = (u - ph.cx) / ph.focal, yc = (v - ph.cy) / ph.focal;
            const P3     dir = ph.r * xc + ph.d * yc + ph.f;
            double       val = scene.plate_refl;
            if (dir.z < -1e-9) {
                const double t0 = -ph.C.z / dir.z;
                const P3     P0 = ph.C + dir * t0;
                const P2     p0{P0.x, P0.y};
                // Blur radius on the plate: pixel footprint times the blur.
                const double sig = std::max(1e-3, cam.blur_px * t0 * dir.norm() / ph.focal);
                // Mat and dots.
                const double mat_cov = phi(rect_sd(p0, mlo, mhi) / sig);
                double       dot_cov = 0;
                if (mat_cov > 0) {
                    const double band = 4 + scene.mat.marker_d;
                    if (std::abs(rect_sd(p0, {0, 0}, {scene.mat.ring_w, scene.mat.ring_h})) < band) {
                        for (size_t i = 0; i < dots.size(); ++i) {
                            const double rad = (i == 0 ? scene.mat.marker_d : scene.mat.dot_d) * 0.5;
                            const double dd  = (p0 - dots[i]).norm();
                            if (dd < rad + 4 * sig) dot_cov = std::max(dot_cov, phi((rad - dd) / sig));
                        }
                    }
                }
                const double mat_val = scene.mat_refl * (1 - dot_cov) + scene.dot_refl * dot_cov;
                val                  = scene.plate_refl * (1 - mat_cov) + mat_val * mat_cov;
                // Parts, shortest first so taller ones cover them. A ray inside a prism's top slice
                // sees its top face; inside a lower slice only, it sees a side wall, which the
                // overhead light reaches less.
                for (size_t k : order) {
                    const SimPrism& pr = scene.prisms[k];
                    // Enough slices that the wall's silhouette does not show steps.
                    const int       S  = std::min(80, std::max(10, int(std::ceil(pr.height / (0.4 * sig)))));
                    double          sd_any = -1e9, sd_top = -1e9;
                    for (int s = 0; s <= S; ++s) {
                        const double z = pr.height * s / S;
                        const double t = (z - ph.C.z) / dir.z;
                        const P3     P = ph.C + dir * t;
                        const P2     q{P.x, P.y};
                        const double m = 6 * sig + 1;
                        if (q.x < boxes[k].lo.x - m || q.y < boxes[k].lo.y - m || q.x > boxes[k].hi.x + m || q.y > boxes[k].hi.y + m) continue;
                        const double sd = signed_distance(pr.footprint, q);
                        sd_any          = std::max(sd_any, sd);
                        if (s == S) sd_top = sd;
                    }
                    if (sd_any < -1e8) continue;
                    const double cov = phi((sd_any + cam.edge_bias_mm) / sig);
                    if (cov <= 1e-6) continue;
                    const double top = std::min(1.0, phi((sd_top + cam.edge_bias_mm) / sig) / cov);
                    const double col = pr.reflectance * (top + (1 - top) * scene.wall_factor);
                    val              = val * (1 - cov) + col * cov;
                }
            }
            double I = (light_on ? cam.light * val : 0.0) + cam.ambient * (1 + cam.ambient_tilt * (double(u) / cam.w - 0.5)) +
                       cam.noise * nd(rng);
            img.at(u, v) = float(std::min(255.0, std::max(0.0, I)));
        }
    return img;
}

Polygon2 sim_circle(const P2& c, double r, int segments, bool ccw)
{
    Polygon2 p;
    for (int i = 0; i < segments; ++i) {
        const double a = 2 * kPi * i / segments * (ccw ? 1 : -1);
        p.push_back(c + P2{std::cos(a), std::sin(a)} * r);
    }
    return p;
}

Polygon2 sim_rect(double x0, double y0, double x1, double y1) { return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; }

SimScene sim_preset_bracket()
{
    SimScene s;
    s.name = "Bracket";
    const P2 o{60, 45};
    auto     tr = [&](Polygon2 p) {
        for (P2& q : p) q += o;
        return p;
    };
    SimPrism base;
    base.footprint.outer = tr(sim_rect(0, 0, 64, 44));
    base.footprint.holes.push_back(tr(sim_circle({10, 10}, 3.2, 64, false)));
    base.footprint.holes.push_back(tr(sim_circle({10, 34}, 3.2, 64, false)));
    base.height = 4;
    SimPrism boss;
    boss.footprint.outer = tr(sim_circle({26, 22}, 7, 96));
    boss.height          = 7;
    SimPrism wall;
    wall.footprint.outer = tr(sim_rect(44, 0, 64, 44));
    wall.height          = 10;
    s.prisms             = {base, boss, wall};
    return s;
}

SimScene sim_preset_coupon()
{
    SimScene s;
    s.name = "Coupon NT-01";
    const P2 o{60, 50};
    auto     tr = [&](Polygon2 p) {
        for (P2& q : p) q += o;
        return p;
    };
    SimPrism slab;
    slab.footprint.outer = tr(sim_rect(0, 0, 60, 40));
    slab.footprint.holes.push_back(tr(sim_circle({15, 20}, 5, 96, false)));
    slab.height = 3;
    SimPrism step;
    step.footprint.outer = tr(sim_rect(30, 0, 60, 40));
    step.height          = 8;
    s.prisms             = {slab, step};
    return s;
}

SimScene sim_preset_empty()
{
    SimScene s;
    s.name = "Mat only";
    return s;
}

} // namespace Slic3r::Metrology

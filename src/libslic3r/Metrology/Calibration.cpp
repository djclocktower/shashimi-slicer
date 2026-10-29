#include "Calibration.hpp"

#include "Geometry.hpp"
#include "Image.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Slic3r::Metrology {

std::vector<P2> MatLayout::dots() const
{
    std::vector<P2> d;
    const int nx = std::max(1, int(std::round(ring_w / pitch)));
    const int ny = std::max(1, int(std::round(ring_h / pitch)));
    for (int i = 0; i <= nx; ++i) d.emplace_back(i * ring_w / nx, 0.0);
    for (int j = 1; j <= ny; ++j) d.emplace_back(ring_w, j * ring_h / ny);
    for (int i = nx - 1; i >= 0; --i) d.emplace_back(i * ring_w / nx, ring_h);
    for (int j = ny - 1; j >= 1; --j) d.emplace_back(0.0, j * ring_h / ny);
    return d;
}

std::vector<Blob> find_blobs(const GrayImage& img, bool bright, double min_area_px, double max_area_px)
{
    std::vector<Blob> out;
    if (img.empty()) return out;
    const double      thr = otsu_threshold(img);
    std::vector<int>  label(img.px.size(), 0);
    std::vector<int>  stack;
    auto fg = [&](int x, int y) { return bright ? img.at(x, y) > thr : img.at(x, y) < thr; };
    int next = 0;
    for (int y = 0; y < img.h; ++y)
        for (int x = 0; x < img.w; ++x) {
            if (label[size_t(y) * img.w + x] || !fg(x, y)) continue;
            ++next;
            double sw = 0, sx = 0, sy = 0;
            int    area = 0, x0 = x, x1 = x, y0 = y, y1 = y;
            bool   touches_border = false;
            stack.assign(1, y * img.w + x);
            label[size_t(y) * img.w + x] = next;
            while (!stack.empty()) {
                const int id = stack.back();
                stack.pop_back();
                const int cx = id % img.w, cy = id / img.w;
                const double w = std::abs(img.at(cx, cy) - thr);
                sw += w;
                sx += w * cx;
                sy += w * cy;
                ++area;
                x0 = std::min(x0, cx); x1 = std::max(x1, cx);
                y0 = std::min(y0, cy); y1 = std::max(y1, cy);
                if (cx == 0 || cy == 0 || cx == img.w - 1 || cy == img.h - 1) touches_border = true;
                const int nb[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
                for (auto& o : nb) {
                    const int nx = cx + o[0], ny = cy + o[1];
                    if (nx < 0 || ny < 0 || nx >= img.w || ny >= img.h) continue;
                    int& l = label[size_t(ny) * img.w + nx];
                    if (l || !fg(nx, ny)) continue;
                    l = next;
                    stack.push_back(ny * img.w + nx);
                }
            }
            if (touches_border || area < min_area_px || area > max_area_px || sw <= 0) continue;
            const double bw = x1 - x0 + 1, bh = y1 - y0 + 1;
            const double fill = area / (bw * bh), aspect = bw / bh;
            if (fill < 0.5 || fill > 0.98 || aspect < 0.33 || aspect > 3.0) continue;
            out.push_back({{sx / sw, sy / sw}, double(area)});
        }
    return out;
}

namespace {

double quad_area(const P2& a, const P2& b, const P2& c, const P2& d)
{
    return 0.5 * std::abs((a - c).cross(b - d));
}

struct Match
{
    int    matched = 0;
    double rms     = 1e9;
    Mat3   H;
    std::vector<P2> px, mm;
};

Match match_layout(const std::vector<Blob>& blobs, const std::vector<P2>& layout, Mat3 H)
{
    Match m;
    for (int pass = 0; pass < 3; ++pass) {
        std::vector<P2> px, mm;
        for (size_t i = 0; i < layout.size(); ++i) {
            const P2 pred = H.apply(layout[i]);
            // Search radius: a third of the local pitch in pixels.
            const P2 nb   = H.apply(layout[(i + 1) % layout.size()]);
            const double rad = std::max(2.0, 0.35 * (nb - pred).norm());
            int    best = -1;
            double bd   = rad;
            for (size_t k = 0; k < blobs.size(); ++k) {
                const double d = (blobs[k].c - pred).norm();
                if (d < bd) {
                    bd   = d;
                    best = int(k);
                }
            }
            if (best >= 0) {
                px.push_back(blobs[size_t(best)].c);
                mm.push_back(layout[i]);
            }
        }
        if (px.size() < 4) break;
        Mat3 Hn;
        if (!fit_homography(mm, px, Hn)) break;
        H         = Hn;
        m.matched = int(px.size());
        m.H       = H;
        m.px      = px;
        m.mm      = mm;
    }
    if (m.matched >= 4) {
        const Mat3 inv = m.H.inverse();
        double     ss  = 0;
        for (size_t i = 0; i < m.px.size(); ++i) {
            const P2 r = inv.apply(m.px[i]) - m.mm[i];
            ss += r.dot(r);
        }
        m.rms = std::sqrt(ss / double(m.px.size()));
    }
    return m;
}

} // namespace

PlateCalibration calibrate_mat(const GrayImage& img, const MatLayout& layout)
{
    PlateCalibration cal;
    const std::vector<P2> lay = layout.dots();
    cal.expected = int(lay.size());
    const double max_area = std::max(400.0, 0.004 * img.w * img.h);
    std::vector<Blob> blobs = find_blobs(img, layout.bright_dots, 4, max_area);
    if (blobs.size() < 8) {
        cal.error = "Too few mat dots were found. Check that the whole mat is in view and lit.";
        return cal;
    }
    // Marker: the largest blob that is clearly bigger than the typical dot.
    std::vector<double> areas;
    for (const Blob& b : blobs) areas.push_back(b.area_px);
    std::nth_element(areas.begin(), areas.begin() + areas.size() / 2, areas.end());
    const double median = areas[areas.size() / 2];
    // Drop blobs far larger than a dot or the marker could be (parts, labels).
    const double marker_ratio = (layout.marker_d * layout.marker_d) / (layout.dot_d * layout.dot_d);
    blobs.erase(std::remove_if(blobs.begin(), blobs.end(), [&](const Blob& b) { return b.area_px > median * marker_ratio * 2.5; }),
                blobs.end());
    std::vector<P2> centers;
    for (const Blob& b : blobs) centers.push_back(b.c);
    std::vector<P2> hull = convex_hull(centers);
    if (hull.size() < 4) {
        cal.error = "The mat's dot ring could not be outlined.";
        return cal;
    }
    // The four hull points spanning the largest quadrilateral are the ring's corners.
    std::array<size_t, 4> best{0, 1, 2, 3};
    double                best_area = -1;
    const size_t          h         = hull.size();
    for (size_t a = 0; a < h; ++a)
        for (size_t b = a + 1; b < h; ++b)
            for (size_t c = b + 1; c < h; ++c)
                for (size_t d = c + 1; d < h; ++d) {
                    const double ar = quad_area(hull[a], hull[b], hull[c], hull[d]);
                    if (ar > best_area) {
                        best_area = ar;
                        best      = {a, b, c, d};
                    }
                }
    std::array<P2, 4> q;
    for (int i = 0; i < 4; ++i) q[size_t(i)] = hull[best[size_t(i)]];
    // The marker is the corner whose blob is biggest.
    auto area_at = [&](const P2& p) {
        for (const Blob& b : blobs)
            if ((b.c - p).norm() < 1e-9) return b.area_px;
        return 0.0;
    };
    int mk = 0;
    for (int i = 1; i < 4; ++i)
        if (area_at(q[size_t(i)]) > area_at(q[size_t(mk)])) mk = i;
    if (area_at(q[size_t(mk)]) < median * 1.8) {
        cal.error = "The mat's marker dot was not found at a corner of the ring.";
        return cal;
    }
    const std::array<P2, 4> lay_corners{P2{0, 0}, P2{layout.ring_w, 0}, P2{layout.ring_w, layout.ring_h}, P2{0, layout.ring_h}};
    Match best_match;
    for (int dir = 0; dir < 2; ++dir) {
        std::vector<P2> src, dst;
        for (int i = 0; i < 4; ++i) {
            const int k = dir == 0 ? (mk + i) % 4 : (mk - i + 4) % 4;
            src.push_back(lay_corners[size_t(i)]);
            dst.push_back(q[size_t(k)]);
        }
        Mat3 H;
        if (!fit_homography(src, dst, H)) continue;
        Match m = match_layout(blobs, lay, H);
        if (m.matched > best_match.matched || (m.matched == best_match.matched && m.rms < best_match.rms)) best_match = m;
    }
    if (best_match.matched < int(0.6 * lay.size())) {
        cal.error = "Only " + std::to_string(best_match.matched) + " of " + std::to_string(lay.size()) +
                    " mat dots matched the layout. Check the mat settings and that the mat is flat and fully in view.";
        return cal;
    }
    cal.ok       = true;
    cal.mm_to_px = best_match.H;
    cal.px_to_mm = best_match.H.inverse();
    cal.matched  = best_match.matched;
    cal.rms_mm   = best_match.rms;
    cal.dot_px   = best_match.px;
    cal.dot_mm   = best_match.mm;
    const P2 mid{layout.ring_w / 2, layout.ring_h / 2};
    const P2 u = cal.mm_to_px.apply(mid);
    cal.mm_per_px = 0.5 * ((cal.px_to_mm.apply(u + P2{1, 0}) - cal.px_to_mm.apply(u)).norm() +
                           (cal.px_to_mm.apply(u + P2{0, 1}) - cal.px_to_mm.apply(u)).norm());
    return cal;
}

PlateCalibration calibrate_two_points(const P2& a_px, const P2& b_px, double distance_mm)
{
    PlateCalibration cal;
    const double d = (b_px - a_px).norm();
    if (d < 2 || distance_mm <= 0) {
        cal.error = "Pick two points further apart and enter their distance.";
        return cal;
    }
    const double s = distance_mm / d;
    // x_mm = s (u - a.x), y_mm = -s (v - a.y)
    cal.px_to_mm.m = {s, 0, -s * a_px.x, 0, -s, s * a_px.y, 0, 0, 1};
    cal.mm_to_px   = cal.px_to_mm.inverse();
    cal.mm_per_px  = s;
    cal.ok         = true;
    return cal;
}

P2 CameraModel::to_plane(const P2& px, double z) const
{
    const P2 x0 = px_to_mm.apply(px);
    if (!has_center || cz <= 0) return x0;
    return nadir + (x0 - nadir) * ((cz - z) / cz);
}

P2 CameraModel::to_px(const P2& mm, double z) const
{
    if (!has_center || cz <= z) return mm_to_px.apply(mm);
    return mm_to_px.apply(nadir + (mm - nadir) * (cz / (cz - z)));
}

CameraModel camera_from(const PlateCalibration& cal)
{
    CameraModel cam;
    cam.ok        = cal.ok;
    cam.mm_to_px  = cal.mm_to_px;
    cam.px_to_mm  = cal.px_to_mm;
    cam.mm_per_px = cal.mm_per_px;
    return cam;
}

bool calibrate_camera_center(CameraModel& cam, const std::vector<double>& offsets, const std::vector<PlateCalibration>& frames,
                             std::string* error)
{
    double wsum = 0, nx = 0, ny = 0, czs = 0;
    for (size_t f = 0; f < frames.size() && f < offsets.size(); ++f) {
        const double dz = offsets[f];
        const PlateCalibration& fr = frames[f];
        if (!fr.ok || std::abs(dz) < 2.0 || fr.dot_px.size() < 8) continue;
        // Reference-plane positions of the raised dots: X0 = m X + b.
        std::vector<double> A(9, 0.0), b(3, 0.0);
        for (size_t i = 0; i < fr.dot_px.size(); ++i) {
            const P2 X  = fr.dot_mm[i];
            const P2 X0 = cam.px_to_mm.apply(fr.dot_px[i]);
            const double r1[3] = {X.x, 1, 0}, r2[3] = {X.y, 0, 1};
            for (int p = 0; p < 3; ++p) {
                for (int q = 0; q < 3; ++q) A[size_t(p * 3 + q)] += r1[p] * r1[q] + r2[p] * r2[q];
                b[size_t(p)] += r1[p] * X0.x + r2[p] * X0.y;
            }
        }
        std::vector<double> s;
        if (!solve_linear(A, b, 3, s)) continue;
        const double m = s[0];
        if (std::abs(m - 1) < 1e-6) continue;
        const double c = m * dz / (m - 1);
        if (!(c > 0)) continue;
        const double w = std::abs(dz);
        nx += w * s[1] / (1 - m);
        ny += w * s[2] / (1 - m);
        czs += w * c;
        wsum += w;
    }
    if (wsum <= 0) {
        if (error) *error = "Camera height needs at least one mat frame with the bed raised by 2 mm or more.";
        return false;
    }
    cam.nadir      = {nx / wsum, ny / wsum};
    cam.cz         = czs / wsum;
    cam.has_center = true;
    return true;
}

bool plane_offset(const CameraModel& cam, const PlateCalibration& frame, double& offset)
{
    if (!cam.has_center || frame.dot_px.size() < 4) return false;
    double num = 0, den = 0;
    for (size_t i = 0; i < frame.dot_px.size(); ++i) {
        const P2 X  = frame.dot_mm[i] - cam.nadir;
        const P2 X0 = cam.px_to_mm.apply(frame.dot_px[i]) - cam.nadir;
        num += X0.dot(X);
        den += X.dot(X);
    }
    if (den <= 0) return false;
    const double m = num / den;
    if (m <= 0) return false;
    offset = cam.cz * (1 - 1 / m);
    return true;
}

} // namespace Slic3r::Metrology

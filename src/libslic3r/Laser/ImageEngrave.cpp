#include "libslic3r/Laser/ImageEngrave.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace Slic3r::Laser {

namespace {

inline uint8_t clamp8(double v) { return uint8_t(std::clamp(std::lround(v), 0L, 255L)); }

struct Tap { int dx, dy; float w; };

// Error diffusion (left to right on every row) with the given kernel; weights already divided.
std::vector<uint8_t> diffuse(const std::vector<uint8_t>& gray, int w, int h, const std::vector<Tap>& taps)
{
    std::vector<float> v(gray.begin(), gray.end());
    std::vector<uint8_t> out(gray.size(), 0);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const size_t i   = size_t(y) * w + x;
            const float  o   = v[i] < 128.f ? 0.f : 255.f;
            const float  err = v[i] - o;
            out[i] = o == 0.f ? 255 : 0;
            for (const Tap& t : taps) {
                const int xx = x + t.dx, yy = y + t.dy;
                if (xx >= 0 && xx < w && yy < h)
                    v[size_t(yy) * w + xx] += err * t.w;
            }
        }
    return out;
}

std::vector<Tap> kernel(DitherMode mode)
{
    auto k = [](std::vector<Tap> taps, float div) { for (Tap& t : taps) t.w /= div; return taps; };
    switch (mode) {
    case DitherMode::FloydSteinberg: return k({{1, 0, 7}, {-1, 1, 3}, {0, 1, 5}, {1, 1, 1}}, 16);
    case DitherMode::Jarvis:
        return k({{1, 0, 7}, {2, 0, 5}, {-2, 1, 3}, {-1, 1, 5}, {0, 1, 7}, {1, 1, 5}, {2, 1, 3},
                  {-2, 2, 1}, {-1, 2, 3}, {0, 2, 5}, {1, 2, 3}, {2, 2, 1}}, 48);
    case DitherMode::Stucki:
        return k({{1, 0, 8}, {2, 0, 4}, {-2, 1, 2}, {-1, 1, 4}, {0, 1, 8}, {1, 1, 4}, {2, 1, 2},
                  {-2, 2, 1}, {-1, 2, 2}, {0, 2, 4}, {1, 2, 2}, {2, 2, 1}}, 42);
    default: // Atkinson: diffuses only 6/8 of the error (its signature crisp highlights/shadows)
        return k({{1, 0, 1}, {2, 0, 1}, {-1, 1, 1}, {0, 1, 1}, {1, 1, 1}, {0, 2, 1}}, 8);
    }
}

// Rank (0..1) of the round-dot spot function cos(2 pi s) + cos(2 pi t) over one screen cell, so that
// "burn if rank < darkness" burns exactly `darkness` of the cell (dots grow from the cell centre).
float dot_rank(float s, float t)
{
    static const std::vector<float> sorted = [] {
        constexpr int N = 128;
        std::vector<float> v;
        v.reserve(N * N);
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                v.push_back(float(std::cos(2 * M_PI * (i + 0.5) / N) + std::cos(2 * M_PI * (j + 0.5) / N)));
        std::sort(v.begin(), v.end());
        return v;
    }();
    const float f = float(std::cos(2 * M_PI * s) + std::cos(2 * M_PI * t));
    return float(std::lower_bound(sorted.begin(), sorted.end(), f) - sorted.begin()) / float(sorted.size());
}

inline float frac(float v) { return v - std::floor(v); }

} // namespace

void apply_adjustments(std::vector<uint8_t>& gray, int w, int h, double brightness, double contrast, double gamma, bool invert)
{
    const double c   = std::clamp(contrast, -100., 100.) * 2.55;
    const double cf  = 259. * (c + 255.) / (255. * (259. - c));
    const double g   = gamma > 0 ? gamma : 1.;
    uint8_t lut[256];
    for (int i = 0; i < 256; ++i) {
        double v = i + std::clamp(brightness, -100., 100.) * 2.55;
        v = cf * (v - 128.) + 128.;
        v = 255. * std::pow(std::clamp(v, 0., 255.) / 255., 1. / g);   // gamma > 1 lightens midtones
        lut[i] = invert ? uint8_t(255 - clamp8(v)) : clamp8(v);
    }
    const size_t n = std::min(gray.size(), size_t(std::max(w, 0)) * size_t(std::max(h, 0)));
    for (size_t i = 0; i < n; ++i) gray[i] = lut[gray[i]];
}

std::vector<uint8_t> dither(const std::vector<uint8_t>& gray, int w, int h, DitherMode mode,
                            double dpi, double cells_per_inch, double screen_angle_deg)
{
    if (w <= 0 || h <= 0 || gray.size() < size_t(w) * h) return {};
    std::vector<uint8_t> out(size_t(w) * h, 0);
    switch (mode) {
    case DitherMode::Threshold:
        for (size_t i = 0; i < out.size(); ++i) out[i] = gray[i] < 128 ? 255 : 0;
        return out;
    case DitherMode::Grayscale:
        for (size_t i = 0; i < out.size(); ++i) out[i] = 255 - gray[i];
        return out;
    case DitherMode::Ordered: {
        static const uint8_t bayer[8][8] = {
            {0, 32, 8, 40, 2, 34, 10, 42},  {48, 16, 56, 24, 50, 18, 58, 26}, {12, 44, 4, 36, 14, 46, 6, 38},
            {60, 28, 52, 20, 62, 30, 54, 22}, {3, 35, 11, 43, 1, 33, 9, 41},  {51, 19, 59, 27, 49, 17, 57, 25},
            {15, 47, 7, 39, 13, 45, 5, 37}, {63, 31, 55, 23, 61, 29, 53, 21}};
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x)
                out[size_t(y) * w + x] = gray[size_t(y) * w + x] < (bayer[y & 7][x & 7] + 0.5) * 4. ? 255 : 0;
        return out;
    }
    case DitherMode::Newsprint:
    case DitherMode::Halftone: {
        // Screen period in pixels; below 2 px a screen cannot render, so fall back to error diffusion.
        const double period = cells_per_inch > 0 ? dpi / cells_per_inch : 0;
        if (period < 2) return diffuse(gray, w, h, kernel(DitherMode::Jarvis));
        const double a = screen_angle_deg * M_PI / 180., ca = std::cos(a) / period, sa = std::sin(a) / period;
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                const size_t i    = size_t(y) * w + x;
                const float  dark = (255 - gray[i]) / 255.f;
                const float  px = x + 0.5f, py = y + 0.5f;
                const float  s = frac(float(px * ca + py * sa)), t = frac(float(-px * sa + py * ca));
                const float  rank = mode == DitherMode::Newsprint ? std::abs(2 * t - 1) : dot_rank(s, t);
                out[i] = rank < dark ? 255 : 0;
            }
        return out;
    }
    default: return diffuse(gray, w, h, kernel(mode));
    }
}

Raster prepare_image(const LaserShape& image, const LaserLayer& layer, const ProgressFn& progress)
{
    if (image.type != ShapeType::Image || image.image_w <= 0 || image.image_h <= 0 ||
        image.gray.size() < size_t(image.image_w) * image.image_h || image.width_mm <= 0 || image.height_mm <= 0 ||
        layer.image_dpi <= 0)
        return {};
    auto cancelled = [&](double f) { return progress && progress(f); };
    try {
        const double    W = image.width_mm, H = image.height_mm, pitch = 25.4 / layer.image_dpi;
        const auto&     L = image.xform.linear();
        const DitherMode mode = image.dither_override ? image.dither : layer.image_dither;

        std::vector<uint8_t> g(image.gray.begin(), image.gray.begin() + size_t(image.image_w) * image.image_h);
        if (!layer.image_pass_through)
            apply_adjustments(g, image.image_w, image.image_h, image.brightness, image.contrast, image.gamma, image.invert);
        if (layer.image_negative)
            for (uint8_t& v : g) v = 255 - v;
        if (cancelled(0.1)) return {};

        // Resample to about the target pitch first (INTER_AREA shrinks without aliasing), then one
        // affine warp into the scan-aligned raster.
        const int rw = std::max(1, int(std::lround(W * L.col(0).norm() / pitch)));
        const int rh = std::max(1, int(std::lround(H * L.col(1).norm() / pitch)));
        cv::Mat src(image.image_h, image.image_w, CV_8UC1, g.data()), resized;
        const int interp = layer.image_pass_through ? cv::INTER_NEAREST
                         : (double(rw) * rh < double(image.image_w) * image.image_h ? cv::INTER_AREA : cv::INTER_LINEAR);
        cv::resize(src, resized, cv::Size(rw, rh), 0, 0, interp);
        if (cancelled(0.3)) return {};

        // Raster frame: u along the scan direction d, v along n = d rotated -90 deg (rows advance
        // "down" the scan: angle 0 -> first row is the top of the image, Y decreasing).
        const double a = layer.angle_deg * M_PI / 180.;
        const Vec2d  d(std::cos(a), std::sin(a)), n(std::sin(a), -std::cos(a));
        double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
        for (const Vec2d c : {Vec2d(-W / 2, -H / 2), Vec2d(W / 2, -H / 2), Vec2d(W / 2, H / 2), Vec2d(-W / 2, H / 2)}) {
            const Vec2d p = image.xform * c;
            u0 = std::min(u0, p.dot(d)); u1 = std::max(u1, p.dot(d));
            v0 = std::min(v0, p.dot(n)); v1 = std::max(v1, p.dot(n));
        }
        Raster r;
        r.pixel_mm = r.line_mm = pitch;
        r.w = std::max(1, int(std::ceil((u1 - u0) / pitch - 1e-6)));
        r.h = std::max(1, int(std::ceil((v1 - v0) / pitch - 1e-6)));
        if (double(r.w) * r.h > 4e8) return {};   // > 400 Mpx: refuse rather than exhaust memory
        r.to_workspace.linear().col(0) = d * pitch;
        r.to_workspace.linear().col(1) = n * pitch;
        r.to_workspace.translation()   = (u0 + 0.5 * pitch) * d + (v0 + 0.5 * pitch) * n;

        // Raster pixel -> workspace -> image local -> resized source pixel (centres at integers).
        Transform2d local_to_src = Transform2d::Identity();
        local_to_src.linear() << rw / W, 0, 0, -rh / H;
        local_to_src.translation() = Vec2d(rw / 2. - 0.5, rh / 2. - 0.5);
        const Transform2d m = local_to_src * image.xform.inverse() * r.to_workspace;
        cv::Mat M = (cv::Mat_<double>(2, 3) << m(0, 0), m(0, 1), m(0, 2), m(1, 0), m(1, 1), m(1, 2));

        std::vector<uint8_t> warped(size_t(r.w) * r.h), mask(size_t(r.w) * r.h);
        cv::Mat dst(r.h, r.w, CV_8UC1, warped.data()), dmask(r.h, r.w, CV_8UC1, mask.data());
        cv::warpAffine(resized, dst, M, dst.size(), (layer.image_pass_through ? cv::INTER_NEAREST : cv::INTER_LINEAR) | cv::WARP_INVERSE_MAP,
                       cv::BORDER_REPLICATE);
        cv::warpAffine(cv::Mat(rh, rw, CV_8UC1, cv::Scalar(255)), dmask, M, dmask.size(), cv::INTER_NEAREST | cv::WARP_INVERSE_MAP,
                       cv::BORDER_CONSTANT, cv::Scalar(0));
        if (cancelled(0.6)) return {};

        if (layer.image_pass_through) {
            r.burn.resize(warped.size());
            for (size_t i = 0; i < warped.size(); ++i) r.burn[i] = warped[i] < 128 ? 255 : 0;
        } else
            r.burn = dither(warped, r.w, r.h, mode, layer.image_dpi, layer.image_cells_per_inch, layer.image_screen_angle_deg);
        for (size_t i = 0; i < mask.size(); ++i)
            if (!mask[i]) r.burn[i] = 0;
        if (cancelled(1.0)) return {};
        return r;
    } catch (...) {
        return {};
    }
}

std::vector<ScanLine> raster_to_scanlines(const Raster& raster, const LaserLayer& layer, const LaserDevice& device)
{
    std::vector<ScanLine> out;
    if (raster.w <= 0 || raster.h <= 0 || raster.burn.size() < size_t(raster.w) * raster.h || raster.pixel_mm <= 0)
        return out;
    const double speed    = device.max_speed_mm_s > 0 ? std::min(layer.speed_mm_s, device.max_speed_mm_s) : layer.speed_mm_s;
    const double overscan = layer.overscan_mm > 0 ? layer.overscan_mm : std::max(0., speed * layer.overscan_pct / 100.);
    const double os_px    = overscan / raster.pixel_mm;
    // Burn value -> power, quantised to 1 % steps so near-equal grayscale neighbours merge.
    auto power = [&](uint8_t v) {
        return v == 255 ? float(layer.power_max) : float(std::round(layer.power_min + v / 255. * (layer.power_max - layer.power_min)));
    };

    for (int row = 0; row < raster.h; ++row) {
        const uint8_t* px = raster.burn.data() + size_t(row) * raster.w;
        int first = 0, last = raster.w - 1;
        while (first < raster.w && !px[first]) ++first;
        if (first == raster.w) continue;                 // blank row
        while (!px[last]) --last;

        ScanLine sl;
        const double left = first - 0.5 - os_px, right = last + 0.5 + os_px;   // pixel-centre units
        sl.start = raster.to_workspace * Vec2d(left, row);
        sl.end   = raster.to_workspace * Vec2d(right, row);
        for (int c = first; c <= last;) {
            if (!px[c]) { ++c; continue; }
            const float p = power(px[c]);
            int e = c + 1;
            while (e <= last && px[e] && power(px[e]) == p) ++e;
            sl.runs.push_back({float((c - 0.5 - left) * raster.pixel_mm), float((e - 0.5 - left) * raster.pixel_mm), p});
            c = e;
        }
        if (layer.bidirectional && out.size() % 2 == 1) {
            const float len = float((right - left) * raster.pixel_mm);
            std::swap(sl.start, sl.end);
            std::reverse(sl.runs.begin(), sl.runs.end());
            for (Run& rn : sl.runs) rn = {len - rn.x1, len - rn.x0, rn.power_pct};
        }
        out.push_back(std::move(sl));
    }
    return out;
}

} // namespace Slic3r::Laser

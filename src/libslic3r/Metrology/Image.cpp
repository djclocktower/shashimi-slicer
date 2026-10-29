#include "Image.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace Slic3r::Metrology {

GrayImage gaussian_blur(const GrayImage& img, double sigma)
{
    if (sigma <= 0 || img.empty()) return img;
    const int           r = std::max(1, int(std::ceil(3 * sigma)));
    std::vector<double> k(size_t(2 * r + 1));
    double              sum = 0;
    for (int i = -r; i <= r; ++i) sum += k[size_t(i + r)] = std::exp(-0.5 * i * i / (sigma * sigma));
    for (double& v : k) v /= sum;
    GrayImage tmp(img.w, img.h), out(img.w, img.h);
    for (int y = 0; y < img.h; ++y)
        for (int x = 0; x < img.w; ++x) {
            double s = 0;
            for (int i = -r; i <= r; ++i) s += k[size_t(i + r)] * img.at(std::min(std::max(x + i, 0), img.w - 1), y);
            tmp.at(x, y) = float(s);
        }
    for (int y = 0; y < img.h; ++y)
        for (int x = 0; x < img.w; ++x) {
            double s = 0;
            for (int i = -r; i <= r; ++i) s += k[size_t(i + r)] * tmp.at(x, std::min(std::max(y + i, 0), img.h - 1));
            out.at(x, y) = float(s);
        }
    return out;
}

GrayImage subtract(const GrayImage& a, const GrayImage& b, float offset)
{
    if (a.w != b.w || a.h != b.h) return a;
    GrayImage out(a.w, a.h);
    for (size_t i = 0; i < a.px.size(); ++i) out.px[i] = std::min(255.f, std::max(0.f, a.px[i] - b.px[i] + offset));
    return out;
}

GrayImage average(const std::vector<GrayImage>& frames)
{
    if (frames.empty()) return {};
    GrayImage out(frames[0].w, frames[0].h);
    int       n = 0;
    for (const GrayImage& f : frames) {
        if (f.w != out.w || f.h != out.h) continue;
        for (size_t i = 0; i < out.px.size(); ++i) out.px[i] += f.px[i];
        ++n;
    }
    for (float& v : out.px) v /= float(std::max(n, 1));
    return out;
}

double otsu_threshold(const GrayImage& img, const std::function<bool(int, int)>& mask)
{
    std::array<double, 256> hist{};
    double                  total = 0;
    for (int y = 0; y < img.h; ++y)
        for (int x = 0; x < img.w; ++x) {
            if (mask && !mask(x, y)) continue;
            hist[size_t(std::min(255, std::max(0, int(img.at(x, y) + 0.5f))))] += 1;
            total += 1;
        }
    if (total <= 0) return 128;
    double sum_all = 0;
    for (int i = 0; i < 256; ++i) sum_all += i * hist[size_t(i)];
    double w0 = 0, sum0 = 0, best = -1, thr = 128;
    for (int t = 0; t < 256; ++t) {
        w0 += hist[size_t(t)];
        if (w0 <= 0) continue;
        const double w1 = total - w0;
        if (w1 <= 0) break;
        sum0 += t * hist[size_t(t)];
        const double m0 = sum0 / w0, m1 = (sum_all - sum0) / w1;
        const double between = w0 * w1 * (m0 - m1) * (m0 - m1);
        if (between > best) {
            best = between;
            thr  = t + 0.5;
        }
    }
    return thr;
}

double frame_motion(const GrayImage& a, const GrayImage& b)
{
    if (a.w != b.w || a.h != b.h || a.empty()) return 1e9;
    double s = 0;
    int    n = 0;
    for (int y = 0; y < a.h; y += 4)
        for (int x = 0; x < a.w; x += 4) {
            s += std::abs(a.at(x, y) - b.at(x, y));
            ++n;
        }
    return n ? s / n : 0;
}

GrayImage rectify(const GrayImage& img, const std::function<P2(const P2&)>& mm_to_px, double x0, double y0, int w, int h, double res)
{
    GrayImage out(std::max(w, 0), std::max(h, 0));
    for (int j = 0; j < out.h; ++j)
        for (int i = 0; i < out.w; ++i) {
            const P2 u = mm_to_px({x0 + (i + 0.5) * res, y0 + (j + 0.5) * res});
            out.at(i, j) = img.sample(u.x, u.y);
        }
    return out;
}

} // namespace Slic3r::Metrology

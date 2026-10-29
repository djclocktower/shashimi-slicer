#pragma once

// Shashimi Metrology: the flat data types shared by every Metrology source file.
//
// Conventions (see README.md in this directory):
//  - Units: mm and degrees in the API; image coordinates are pixels with (0, 0) at the centre of
//    the top-left pixel, x right, y down.
//  - Plate frame: the measuring mat seen from above, X right, Y up (towards the rear), origin at the
//    centre of the mat's marker dot, Z up from the mat's top face. Every mm coordinate uses it.
//  - Regions: an outer loop counter-clockwise and holes clockwise (in the Y-up plate frame), so the
//    material is always on the left of a loop's direction.
//  - The kernel is headless and depends on the standard library only.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace Slic3r::Metrology {

constexpr double kPi = 3.14159265358979323846;

struct P2
{
    double x = 0, y = 0;
    P2() = default;
    P2(double x_, double y_) : x(x_), y(y_) {}
    P2  operator+(const P2& o) const { return {x + o.x, y + o.y}; }
    P2  operator-(const P2& o) const { return {x - o.x, y - o.y}; }
    P2  operator*(double s) const { return {x * s, y * s}; }
    P2  operator/(double s) const { return {x / s, y / s}; }
    P2& operator+=(const P2& o) { x += o.x; y += o.y; return *this; }
    double dot(const P2& o) const { return x * o.x + y * o.y; }
    double cross(const P2& o) const { return x * o.y - y * o.x; }
    double norm() const { return std::hypot(x, y); }
    P2     normalized() const { double n = norm(); return n > 0 ? P2{x / n, y / n} : P2{}; }
    P2     perp() const { return {-y, x}; }   // rotated +90 degrees
};

struct P3
{
    double x = 0, y = 0, z = 0;
    P3() = default;
    P3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    P3  operator+(const P3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    P3  operator-(const P3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    P3  operator*(double s) const { return {x * s, y * s, z * s}; }
    double dot(const P3& o) const { return x * o.x + y * o.y + z * o.z; }
    P3     cross(const P3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    double norm() const { return std::sqrt(x * x + y * y + z * z); }
    P3     normalized() const { double n = norm(); return n > 0 ? P3{x / n, y / n, z / n} : P3{}; }
};

using Polygon2 = std::vector<P2>;

// A connected area: outer loop CCW, holes CW (plate frame, Y up).
struct Region
{
    Polygon2              outer;
    std::vector<Polygon2> holes;
};

// Row-major 3x3 matrix, used for plane homographies.
struct Mat3
{
    std::array<double, 9> m{1, 0, 0, 0, 1, 0, 0, 0, 1};

    double  operator()(int r, int c) const { return m[size_t(r * 3 + c)]; }
    double& operator()(int r, int c) { return m[size_t(r * 3 + c)]; }

    P2 apply(const P2& p) const
    {
        const double w = m[6] * p.x + m[7] * p.y + m[8];
        const double s = std::abs(w) > 1e-15 ? 1.0 / w : 0.0;
        return {(m[0] * p.x + m[1] * p.y + m[2]) * s, (m[3] * p.x + m[4] * p.y + m[5]) * s};
    }
    Mat3 operator*(const Mat3& o) const
    {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                double s = 0;
                for (int k = 0; k < 3; ++k) s += (*this)(i, k) * o(k, j);
                r(i, j) = s;
            }
        return r;
    }
    double det() const
    {
        return m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
    }
    // Identity when singular.
    Mat3 inverse() const
    {
        const double d = det();
        Mat3         r;
        if (std::abs(d) < 1e-300) return r;
        const double id = 1.0 / d;
        r.m = {(m[4] * m[8] - m[5] * m[7]) * id, (m[2] * m[7] - m[1] * m[8]) * id, (m[1] * m[5] - m[2] * m[4]) * id,
               (m[5] * m[6] - m[3] * m[8]) * id, (m[0] * m[8] - m[2] * m[6]) * id, (m[2] * m[3] - m[0] * m[5]) * id,
               (m[3] * m[7] - m[4] * m[6]) * id, (m[1] * m[6] - m[0] * m[7]) * id, (m[0] * m[4] - m[1] * m[3]) * id};
        return r;
    }
};

// 8-bit-range grey image stored as float (0..255), row-major, y down.
struct GrayImage
{
    int                w = 0, h = 0;
    std::vector<float> px;

    GrayImage() = default;
    GrayImage(int w_, int h_, float fill = 0.f) : w(w_), h(h_), px(size_t(w_) * size_t(h_), fill) {}

    bool  empty() const { return w <= 0 || h <= 0; }
    float at(int x, int y) const { return px[size_t(y) * size_t(w) + size_t(x)]; }
    float& at(int x, int y) { return px[size_t(y) * size_t(w) + size_t(x)]; }
    bool  inside(double x, double y) const { return x >= 0 && y >= 0 && x <= w - 1 && y <= h - 1; }
    // Bilinear, clamped to the border.
    float sample(double x, double y) const
    {
        if (empty() || !std::isfinite(x) || !std::isfinite(y)) return 0.f;
        x = std::min(std::max(x, 0.0), double(w - 1));
        y = std::min(std::max(y, 0.0), double(h - 1));
        const int    x0 = std::min(int(x), w - 2 < 0 ? 0 : w - 2), y0 = std::min(int(y), h - 2 < 0 ? 0 : h - 2);
        const int    x1 = std::min(x0 + 1, w - 1), y1 = std::min(y0 + 1, h - 1);
        const double fx = x - x0, fy = y - y0;
        const double a = at(x0, y0) * (1 - fx) + at(x1, y0) * fx;
        const double b = at(x0, y1) * (1 - fx) + at(x1, y1) * fx;
        return float(a * (1 - fy) + b * fy);
    }
};

// A pass/fail band around a nominal value.
struct Tolerance
{
    bool   enabled = false;
    double nominal = 0;
    double plus    = 0.1;
    double minus   = 0.1;   // magnitude: the band is [nominal - minus, nominal + plus]
};

enum class Judge { None, OK, NG };

inline Judge judge(const Tolerance& t, double value)
{
    if (!t.enabled) return Judge::None;
    return (value >= t.nominal - t.minus - 1e-12 && value <= t.nominal + t.plus + 1e-12) ? Judge::OK : Judge::NG;
}

} // namespace Slic3r::Metrology

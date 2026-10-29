#pragma once

// Outline tracing: scalar fields on a mm grid, marching-squares contours into regions, polygon
// rasterization and SVG export.

#include "Calibration.hpp"
#include "MetrologyTypes.hpp"

#include <functional>
#include <string>
#include <vector>

namespace Slic3r::Metrology {

// Samples on a mm grid: sample (i, j) sits at (x0 + (i + 0.5) res, y0 + (j + 0.5) res), row 0 is
// the lowest y.
struct Field
{
    int                w = 0, h = 0;
    double             x0 = 0, y0 = 0, res = 0.1;
    std::vector<float> v;

    Field() = default;
    Field(int w_, int h_, double x0_, double y0_, double res_, float fill = 0.f)
        : w(w_), h(h_), x0(x0_), y0(y0_), res(res_), v(size_t(std::max(w_, 0)) * size_t(std::max(h_, 0)), fill) {}
    float  at(int i, int j) const { return v[size_t(j) * size_t(w) + size_t(i)]; }
    float& at(int i, int j) { return v[size_t(j) * size_t(w) + size_t(i)]; }
    P2     pos(int i, int j) const { return {x0 + (i + 0.5) * res, y0 + (j + 0.5) * res}; }
    // Bilinear between samples, 0 outside the grid.
    float sample(const P2& p) const
    {
        const double fx = (p.x - x0) / res - 0.5, fy = (p.y - y0) / res - 0.5;
        if (w <= 0 || h <= 0 || fx < -0.5 || fy < -0.5 || fx > w - 0.5 || fy > h - 0.5) return 0.f;
        const int    i0 = std::min(std::max(int(std::floor(fx)), 0), w - 1), j0 = std::min(std::max(int(std::floor(fy)), 0), h - 1);
        const int    i1 = std::min(i0 + 1, w - 1), j1 = std::min(j0 + 1, h - 1);
        const double ax = std::min(std::max(fx - i0, 0.0), 1.0), ay = std::min(std::max(fy - j0, 0.0), 1.0);
        return float((at(i0, j0) * (1 - ax) + at(i1, j0) * ax) * (1 - ay) + (at(i0, j1) * (1 - ax) + at(i1, j1) * ax) * ay);
    }
};

// Field from a rectified image (see rectify(): row 0 is already the lowest y).
Field field_from_image(const GrayImage& rectified, double x0, double y0, double res);

// Areas where the field is above `iso` (or below, with inside_above = false), sub-sample accurate,
// simplified to `simplify_tol` and without loops smaller than `min_area` mm².
std::vector<Region> contour_regions(const Field& f, float iso, bool inside_above, double simplify_tol, double min_area);

// 1 inside the regions, 0 outside (even-odd over every loop). Boundary samples get their covered
// fraction, so contouring at 0.5 gives the outline back to well under a sample.
Field rasterize(const std::vector<Region>& regions, double x0, double y0, int w, int h, double res);

// Replaces the 0/1 value of every sample next to a boundary with the fraction of its cell for
// which inside() holds (ss x ss sub-samples).
void soften_boundary(Field& f, const std::function<bool(const P2&)>& inside, int ss = 4);

// Standalone SVG in mm (the Y axis flipped to SVG's downward convention).
std::string regions_svg(const std::vector<Region>& regions);

// The outline of a part seen by the camera, traced on the mat plane (z = 0) inside the box lo..hi.
// threshold < 0 picks one with Otsu over the box. The result is the silhouette: for a part with
// height it still needs the parallax correction (the reverse-engineering session does that).
std::vector<Region> trace_silhouette(const GrayImage& img, const CameraModel& cam, const P2& lo, const P2& hi, double res,
                                     double threshold, bool part_bright, double simplify_tol, double min_area);

// Orients loops (outer CCW, holes CW) and nests holes under the outer that contains them.
std::vector<Region> regions_from_loops(std::vector<Polygon2> loops, double min_area);

} // namespace Slic3r::Metrology

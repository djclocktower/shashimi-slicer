#pragma once

// Fitting and 2D geometry for the Metrology kernel: homographies, least-squares lines and circles
// with outlier rejection, and polygon helpers.

#include "MetrologyTypes.hpp"

#include <vector>

namespace Slic3r::Metrology {

// Solves the n x n system A x = b in place (partial pivoting). False when singular.
bool solve_linear(std::vector<double> A, std::vector<double> b, int n, std::vector<double>& x);

// Plane homography mapping src[i] to dst[i] (normalized DLT, least squares for more than 4
// points). False with fewer than 4 points or a degenerate configuration.
bool fit_homography(const std::vector<P2>& src, const std::vector<P2>& dst, Mat3& H);

// Total-least-squares line. `p` is the centroid, `d` the unit direction (pointing from the first
// to the last input point), `rms` the orthogonal residual.
struct LineFit
{
    bool   ok   = false;
    P2     p, d;
    double rms  = 0;
    double max_dev = 0;   // largest |residual|: straightness
    int    n    = 0;
    double distance(const P2& q) const { return (q - p).cross(d); }   // signed, + on the left of d
    P2     project(const P2& q) const { return p + d * (q - p).dot(d); }
};

struct CircleFit
{
    bool   ok  = false;
    P2     c;
    double r   = 0;
    double rms = 0;
    int    n   = 0;
};

LineFit   fit_line(const std::vector<P2>& pts);
CircleFit fit_circle(const std::vector<P2>& pts);
// Iterative 2.5-sigma rejection (at least `keep_frac` of the points stay).
LineFit   fit_line_robust(const std::vector<P2>& pts, double keep_frac = 0.6);
CircleFit fit_circle_robust(const std::vector<P2>& pts, double keep_frac = 0.6);

// Acute angle between two lines, degrees in [0, 90].
double line_angle_deg(const LineFit& a, const LineFit& b);
// Direction angle of a line in degrees (-90, 90] measured from +X.
double line_direction_deg(const LineFit& a);

// ---- Polygons --------------------------------------------------------------------------------

double signed_area(const Polygon2& poly);   // + for CCW (Y up)
bool   point_in_polygon(const Polygon2& poly, const P2& p);   // even-odd
bool   point_in_region(const Region& r, const P2& p);
double segment_distance(const P2& a, const P2& b, const P2& p);
// Distance to the nearest boundary; positive inside the region set, negative outside.
double signed_distance(const std::vector<Region>& regions, const P2& p);
double signed_distance(const Region& region, const P2& p);
// Closed-loop Douglas-Peucker.
Polygon2 simplify_closed(const Polygon2& poly, double tol);
// Moves every loop's vertices along the material's outward normal by d (d > 0 grows the material,
// shrinking holes). Meant for small corrections (a fraction of a mm).
std::vector<Region> offset_regions(const std::vector<Region>& regions, double d);
std::vector<P2> convex_hull(std::vector<P2> pts);   // CCW, no collinear points
void bounding_box(const std::vector<Region>& regions, P2& lo, P2& hi);

} // namespace Slic3r::Metrology

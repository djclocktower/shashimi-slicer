#pragma once

// Caliper edge tools (the Keyence "edge" measurement): grey profiles sampled along scan lines,
// edges found as gradient extrema with sub-pixel parabola refinement. Everything here works in
// image pixels; the caller maps edge points to mm.

#include "MetrologyTypes.hpp"

#include <vector>

namespace Slic3r::Metrology {

enum class Polarity { Any, DarkToLight, LightToDark };   // along the scan direction
enum class EdgePick { Strongest, First, Last };

struct EdgeParams
{
    Polarity polarity     = Polarity::Any;
    EdgePick pick         = EdgePick::Strongest;
    double   min_contrast = 12;    // grey levels: weaker gradient steps are ignored
    double   smooth_sigma = 1.0;   // px, along the scan
    double   avg_width    = 3.0;   // px, averaged across the scan
};

struct EdgeHit
{
    P2     p;
    double strength = 0;   // grey-level step, signed along the scan
    double t        = 0;   // position along the scan, px from its start
    P2     dir;            // unit scan direction (image px)
};

// Every edge along a -> b that passes the contrast and polarity filters, in scan order.
std::vector<EdgeHit> find_edges(const GrayImage& img, const P2& a, const P2& b, const EdgeParams& prm);
// The one edge chosen by prm.pick. False when there is none.
bool find_edge(const GrayImage& img, const P2& a, const P2& b, const EdgeParams& prm, EdgeHit& out);

// A rectangle in the image: scans run along `angle` (radians, image frame), `length` long, and are
// spread across `width`.
struct OrientedRect
{
    P2     center;
    double angle  = 0;
    double length = 40;
    double width  = 40;

    P2 dir() const { return {std::cos(angle), std::sin(angle)}; }
    P2 across() const { return dir().perp(); }
    std::array<P2, 4> corners() const;
};

// One edge per scan line (prm.pick), n_lines scans across the rectangle.
std::vector<EdgeHit> scan_rect(const GrayImage& img, const OrientedRect& rect, int n_lines, const EdgeParams& prm);
// Two edges per scan line: the first and the last that pass the filters (a width gauge).
void scan_rect_pair(const GrayImage& img, const OrientedRect& rect, int n_lines, const EdgeParams& prm, std::vector<EdgeHit>& first,
                    std::vector<EdgeHit>& last);
// Rays from the centre outward between r_in and r_out.
std::vector<EdgeHit> scan_annulus(const GrayImage& img, const P2& center, double r_in, double r_out, int n_rays, const EdgeParams& prm);

std::vector<P2> hit_points(const std::vector<EdgeHit>& hits);

} // namespace Slic3r::Metrology

#pragma once

// A synthetic chamber camera and nozzle probe, so the whole measuring flow runs (and is tested)
// without a printer. Parts are stacks of prisms standing on the mat; the camera is a pinhole with
// blur, noise, an edge bias (bright parts bloom) and room light.

#include "Calibration.hpp"
#include "MetrologyTypes.hpp"

#include <string>
#include <vector>

namespace Slic3r::Metrology {

struct SimPrism
{
    Region footprint;           // plate frame
    double height      = 5;     // above the mat
    double reflectance = 0.75;
};

struct SimScene
{
    std::string           name;
    MatLayout             mat;
    std::vector<SimPrism> prisms;
    double                mat_refl   = 0.07;   // black face
    double                dot_refl   = 0.85;
    double                plate_refl = 0.32;
};

struct SimCamera
{
    int    w = 1280, h = 720;      // P1P 720p
    P3     center{-40, -120, 300}; // plate frame, mm
    P3     target{90, 45, 0};
    double focal_px     = 1200;
    double blur_px      = 0.9;
    double noise        = 1.5;     // grey levels, 1 sigma
    double edge_bias_mm = 0.05;    // parts read this much larger per edge
    double light        = 210;
    double ambient      = 25;
    double ambient_tilt = 0.6;     // room light gradient across the frame
};

// Ground truth: plate point on the plane at height z (bed raised by `offset`) -> image pixel.
P2   sim_project(const SimCamera& cam, const P3& p, double offset = 0);
Mat3 sim_plate_homography(const SimCamera& cam, double offset = 0);
P2   sim_nadir(const SimCamera& cam);

GrayImage sim_render(const SimScene& scene, const SimCamera& cam, double offset, bool light_on, unsigned seed);

double sim_height(const SimScene& scene, const P2& p);
// Contact height of a flat nozzle face of radius tip_r centred at p.
double sim_touch(const SimScene& scene, const P2& p, double tip_r);

Polygon2 sim_circle(const P2& c, double r, int segments, bool ccw = true);
Polygon2 sim_rect(double x0, double y0, double x1, double y1);

SimScene sim_preset_bracket();   // a three-level bracket with two through holes
SimScene sim_preset_coupon();    // the 60 x 40 mm test coupon
SimScene sim_preset_empty();     // the mat alone

} // namespace Slic3r::Metrology

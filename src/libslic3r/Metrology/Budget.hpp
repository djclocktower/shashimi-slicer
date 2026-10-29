#pragma once

// The error budget: independent 1-sigma sources combined as root-sum-square. Every input is an
// estimate until the repeatability test replaces the trigger value with a measured one.

namespace Slic3r::Metrology {

struct BudgetInputs
{
    double mm_per_px       = 0.22;    // P1P 720p with the bed raised
    double centroid_floor  = 0.10;    // px, fiducial readout limit
    double edge_floor      = 0.12;    // px, camera edge-fit limit
    double view_angle_deg  = 40;      // camera axis from vertical
    double trigger         = 0.010;   // mm, 1 sigma
    double tip             = 0.005;   // mm, nozzle cleanliness
    double thermal         = 0.015;   // mm
    double motion          = 0.015;   // mm, XY repeatability
    double size_reference  = 0.01;    // mm, tip-radius / bias reference
    double bracket_mm      = 2.0;     // camera-guided search window per edge
    int    touches_per_edge = 6;
};

struct BudgetResult
{
    double readout = 0;   // 1 sigma, mm
    double height  = 0;   // 1 sigma per touch
    double edge    = 0;   // 1 sigma, probe edge
    double camera_edge = 0;   // 1 sigma, camera edge after fitting
};

BudgetResult budget(const BudgetInputs& in);

// Inverse-variance combination of two independent estimates; `sigma` receives the combined 1 sigma.
double fuse(double a, double sa, double b, double sb, double* sigma = nullptr);

} // namespace Slic3r::Metrology

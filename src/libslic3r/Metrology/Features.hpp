#pragma once

// Inspection features (the Keyence-style measurement program): each feature keeps its region of
// interest in image pixels, so the whole program re-runs on a new frame. Edge points are mapped to
// the plate frame on the plane at the feature's height before fitting, so results are in mm.

#include "Calibration.hpp"
#include "Edges.hpp"
#include "Geometry.hpp"

#include <string>
#include <vector>

namespace Slic3r::Metrology {

enum class FeatureKind
{
    Line,       // edge line in a rectangle: value = direction angle (deg), straightness reported
    Circle,     // edge circle in an annulus: value = diameter
    Width,      // first and last edge across a rectangle: value = mean distance between them
    Distance,   // between two features (circle centres / line): value = mm
    Angle,      // between two line features: value = degrees
};

const char* feature_kind_name(FeatureKind k);

struct Feature
{
    int         id = 0;
    std::string name;
    FeatureKind kind = FeatureKind::Line;

    // Region of interest (image px).
    OrientedRect rect;              // Line, Width
    P2           center;            // Circle
    double       r_in = 10, r_out = 30;
    int          scans   = 16;
    EdgeParams   edge;
    double       plane_z = 0;       // height of the measured edges above the mat, mm
    int          ref_a = -1, ref_b = -1;   // feature ids (Distance, Angle)

    Tolerance tol;

    // Results.
    bool            ok = false;
    std::string     error;
    std::vector<P2> edge_px, edge2_px;   // found edge points (Width: both sides)
    LineFit         line, line2;         // plate frame
    CircleFit       circle;              // plate frame
    double          value = 0;
    double          aux   = 0;           // Line: straightness; Circle: roundness rms; Width: spread
    Judge           verdict = Judge::None;
    const char*     unit() const { return (kind == FeatureKind::Line || kind == FeatureKind::Angle) ? "deg" : "mm"; }
};

struct Program
{
    std::vector<Feature> features;
    int                  next_id     = 1;
    bool                 part_bright = true;   // parts brighter than the background (black mat face)

    Feature& add(Feature f);
    Feature* find(int id);
    const Feature* find(int id) const;
    void     remove(int id);   // also clears references to it

    // Runs every feature on the image; derived features after their sources.
    void  run(const GrayImage& img, const CameraModel& cam);
    Judge overall() const;     // NG if any NG, OK if any OK, else None
    std::string to_csv() const;
};

} // namespace Slic3r::Metrology

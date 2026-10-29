#pragma once

// Measuring-mat calibration and the camera model.
//
// The mat carries a rectangular ring of dots at a fixed pitch with one larger marker dot at a
// corner (the plate frame's origin). Because the ring is longer in X than in Y, the dot count per
// side tells the two possible corner orders apart, so the mat can lie at any rotation, either face up.
//
// CameraModel adds the camera centre (found from frames of the mat with the bed raised by known
// amounts), which lets image points be mapped onto planes above the mat (parallax correction) and
// lets the bed height be read back from the dots (the probe readout).

#include "MetrologyTypes.hpp"

#include <string>
#include <vector>

namespace Slic3r::Metrology {

struct MatLayout
{
    double ring_w      = 180;    // marker to the far X corner, mm
    double ring_h      = 140;    // marker to the far Y corner, mm
    double pitch       = 10;
    double dot_d       = 2.5;
    double marker_d    = 6;
    double margin      = 10;     // mat edge outside the ring
    double thickness   = 3;
    bool   bright_dots = true;   // white dots on the black face; false for the white face

    std::vector<P2> dots() const;   // ring dots in the plate frame, the marker (0, 0) first
    P2 mat_min() const { return {-margin, -margin}; }
    P2 mat_max() const { return {ring_w + margin, ring_h + margin}; }
};

struct Blob
{
    P2     c;
    double area_px = 0;
};

// Round blobs brighter (or darker) than the Otsu threshold.
std::vector<Blob> find_blobs(const GrayImage& img, bool bright, double min_area_px, double max_area_px);

struct PlateCalibration
{
    bool        ok = false;
    std::string error;
    Mat3        mm_to_px, px_to_mm;
    int         expected = 0, matched = 0;
    double      rms_mm    = 0;   // dot residual after the fit
    double      mm_per_px = 0;   // at the ring centre
    std::vector<P2> dot_px, dot_mm;   // matched pairs
};

PlateCalibration calibrate_mat(const GrayImage& img, const MatLayout& layout);

// Calibration from two image points a known distance apart (no mat): a similarity transform with
// Y flipped so the plate frame stays Y up. The origin is image point `a`.
PlateCalibration calibrate_two_points(const P2& a_px, const P2& b_px, double distance_mm);

struct CameraModel
{
    bool ok = false;
    Mat3 mm_to_px, px_to_mm;     // the mat's top face (z = 0) at the reference bed position
    double mm_per_px   = 0;
    bool   has_center  = false;  // nadir and cz are valid
    P2     nadir;                // camera centre projected onto the plate, mm
    double cz          = 0;      // camera height above the mat's top face, mm
    double ref_bed_z   = 0;      // printer bed Z when the reference frame was taken

    // Image point -> plate point on the plane at height z (z ignored without a centre).
    P2 to_plane(const P2& px, double z) const;
    // Plate point at height z -> image point.
    P2 to_px(const P2& mm, double z) const;
};

CameraModel camera_from(const PlateCalibration& cal);

// Mat frames taken with the bed raised by `offsets[i]` mm towards the camera (0 = the reference).
// Needs at least one frame raised by 2 mm or more.
bool calibrate_camera_center(CameraModel& cam, const std::vector<double>& offsets, const std::vector<PlateCalibration>& frames,
                             std::string* error);

// Height of the mat plane above its reference position, from the dots matched in a frame.
bool plane_offset(const CameraModel& cam, const PlateCalibration& frame, double& offset);

} // namespace Slic3r::Metrology

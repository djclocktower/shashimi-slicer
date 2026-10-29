#pragma once

// Nozzle-touch probing on Bambu printers: G-code for a batch of touches and the readout helpers.
//
// A touch is a G380 guarded move (Bambu's G38 equivalent, used by the stock start G-code) that
// raises the bed until the nozzle meets the surface, followed by a dwell so the chamber camera can
// record where the bed stopped. The firmware reports nothing back; the camera reads the bed height
// from the mat's dots (plane_offset() in Calibration.hpp). Untested on hardware: see README.md.

#include "MetrologyTypes.hpp"

#include <string>
#include <vector>

namespace Slic3r::Metrology {

struct ProbeSettings
{
    // Where the mat's plate frame sits in printer coordinates (mm, degrees).
    double mat_x       = 128 - 90;
    double mat_y       = 128 - 70;
    double mat_rot_deg = 0;
    double mat_thickness = 3;
    double clearance   = 4;     // travel this far above the tallest expected surface
    double overtravel  = 2;     // the guarded move may go this far below the lowest expected surface
    double hold_s      = 3;     // dwell at each contact for the camera
    double backoff     = 2;     // relative lift after a contact
    double feed_xy     = 6000;  // mm/min
    double probe_feed  = 300;   // mm/min
};

struct ProbeTouch
{
    P2          xy;      // plate frame
    std::string label;
};

P2 mat_to_printer(const P2& p, const ProbeSettings& s);

// A batch of touches. Heights are relative to the mat's top face; z_min/z_max bound the surfaces
// the batch can meet. The first touch is always on the mat itself (the height reference).
std::string probe_gcode(const std::vector<ProbeTouch>& touches, const ProbeSettings& s, double z_min, double z_max,
                        const P2& mat_ref_xy);

// The repeatability test: n touches on one spot of an empty plate (printer coordinates).
std::string repeatability_gcode(const P2& printer_xy, int n, const ProbeSettings& s);

// Still periods in a frame sequence: `motion[i]` is frame_motion(frame i, frame i+1).
struct StillSegment
{
    int first = 0, last = 0;   // frame indices, inclusive
};
std::vector<StillSegment> find_stills(const std::vector<double>& motion, double threshold, int min_frames);

struct RepeatStats
{
    int    n      = 0;
    double mean   = 0;
    double stddev = 0;
    double range  = 0;
};
RepeatStats repeat_stats(const std::vector<double>& values);

} // namespace Slic3r::Metrology

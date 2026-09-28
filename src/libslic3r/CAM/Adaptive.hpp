#pragma once

// Adaptive (constant-engagement) clearing: a port of FreeCAD's libarea/Adaptive.cpp.
// Purely 2D. Input regions are libslic3r scaled coordinates; Params are mm.

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polyline.hpp"

#include <functional>
#include <string>
#include <vector>

namespace Slic3r::CAM::Adaptive {

enum class MotionType { Cutting, LinkClear, LinkNotClear, LinkClearAtPrevPass };

struct Path {
    MotionType type{MotionType::Cutting};
    Polyline   pts;
};

// A helix entry into one disjoint area: ramp down around `center` (radius Result::helix_radius),
// then move to `start` and continue with paths[path_index].
struct Entry {
    Point  center;
    Point  start;
    size_t path_index{0};
    // How to get from the end of this area's paths back to `center` (e.g. for the next depth):
    // LinkClear = the straight move is through cleared area, LinkNotClear = retract.
    MotionType return_type{MotionType::LinkNotClear};
};

struct Result {
    std::vector<Path>  paths;
    Point              helix_center;     // == entries.front().center
    double             helix_radius{0};  // mm
    std::vector<Entry> entries;          // one per disjoint cleared area, in path order
    bool               ok{false};
    std::string        error;            // plain language when !ok
    std::vector<std::string> warnings;   // plain language; result is still usable
    // Area swept by the tool (incl. `already_cleared`), limited to the stock grown by the tool
    // radius. Feed it as `already_cleared` to a rest-machining pass with a smaller tool.
    ExPolygons         cleared;
};

struct Params {
    double tool_diameter{6};
    double stepover_fraction{0.2};      // 0..1 of the tool diameter: max engagement (optimal load)
    double helix_diameter{0};           // 0 = auto (FreeCAD: from the tool diameter)
    double helix_angle_deg{2};
    double tolerance{0.01};             // mm; adaptive: grid ~tolerance/5, min step ~10x tolerance
    double stock_to_leave{0};           // mm, kept from `keep_out` and from region walls
    bool   climb{true};
    bool   force_inside_out{false};
    // FreeCAD keepToolDownDistRatio: a link shorter than this x tool diameter stays down
    // (LinkClear) when its straight path is clear. A ratio, not a bool.
    double keep_tool_down_ratio{3.0};
    bool   finishing_profile{true};     // final pass along the walls
    // false (pocket, FreeCAD ClearingInside): region's outline is a wall, the tool stays inside.
    // true (3D levels, FreeCAD ClearingOutside): region is stock to remove; everything outside
    // region and keep_out is air the tool may travel through.
    bool   outside_is_air{false};
    // Optional: called periodically with progress 0..1; return true to cancel (Result: !ok,
    // error "Cancelled.").
    std::function<bool(double progress)> cancel;
};

// Clears `region` without entering `keep_out` (grown by stock_to_leave).
// `already_cleared` (rest machining, FreeCAD clearedPaths): area the tool may move through freely
// and that needs no cutting or finishing.
Result clear(const ExPolygons& region, const ExPolygons& keep_out, const Params& params,
             const ExPolygons& already_cleared = {});

} // namespace Slic3r::CAM::Adaptive

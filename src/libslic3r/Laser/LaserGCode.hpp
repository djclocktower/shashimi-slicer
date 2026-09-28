#pragma once

// G-code writer: LaserJob -> text for GRBL / grblHAL / Marlin / Smoothie.

#include "libslic3r/Laser/LaserTypes.hpp"

#include <optional>
#include <string>

namespace Slic3r::Laser {

// Job frame -> machine coordinates for the device's origin corner (FrontLeft: identity; Rear*: Y
// becomes bed_h - y; *Right: X becomes bed_w - x). Relative job frames (UserOrigin /
// CurrentPosition) only flip the axis direction. from_machine() is the inverse.
Vec2d to_machine(const Vec2d& p, const LaserDevice& device, JobSettings::StartFrom frame);
Vec2d from_machine(const Vec2d& p, const LaserDevice& device, JobSettings::StartFrom frame);

// Header (comments; G21 G90; M4 S0 dynamic / M3 S0), start_gcode, then per segment: G0 S0 travel (or
// G1 S0 when !uses_g0_for_travel), G1 X Y S F (F mm/min, S = power * s_max / 100), a Scan line as
// one G1 per run with S changes plus S0 overscan ends, G4 P dwell, Z moves (enable_z), air assist
// on/off at layer changes; M5, end_gcode, M2. Marlin: M3/M4/M5, or M106 S0..255 / M107 with
// marlin_fan_laser (each after M400: Marlin does not sync them with the moves), M3 I with
// marlin_inline. Smoothie: S as 0..1. CurrentPosition jobs are wrapped in G92 X0 Y0 ... G92.1.
// Empty string + `error` for a Ruida device or a failed job.
std::string gcode(const LaserJob& job, const LaserDevice& device, const GCodeOptions& opts = {}, std::string* error = nullptr);

// Framing: trace the job's bounding rectangle (Rect) or convex hull of its lit moves (Outline) at
// travel speed, laser off, or at device.frame_power_pct when that is > 0 (diode lasers), then
// return to `return_to` (job frame: where the head was when framing started, from the status
// report), or to the job-frame origin when that is unknown.
enum class FrameMode { Rect, Outline };
std::string frame_gcode(const LaserJob& job, const LaserDevice& device, FrameMode mode,
                        const std::optional<Vec2d>& return_to = std::nullopt);

} // namespace Slic3r::Laser

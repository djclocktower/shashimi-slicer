#pragma once

// Umbrella header of the Laser kernel (namespace Slic3r::Laser). See README.md.

#include "libslic3r/Laser/LaserTypes.hpp"
#include "libslic3r/Laser/LaserDocument.hpp"
#include "libslic3r/Laser/VectorOps.hpp"
#include "libslic3r/Laser/ImageEngrave.hpp"
#include "libslic3r/Laser/Trace.hpp"
#include "libslic3r/Laser/LaserPlan.hpp"
#include "libslic3r/Laser/LaserGCode.hpp"
#include "libslic3r/Laser/Import.hpp"
#include "libslic3r/Laser/Materials.hpp"
#include "libslic3r/Laser/TextLayout.hpp"

namespace Slic3r::Laser {

// plan() + gcode() in one call, for Save G-code and the CLI. Empty + `error` on failure.
std::string export_gcode(const LaserDocument& doc, const LaserDevice& device, const GCodeOptions& opts = {},
                         std::string* error = nullptr);

} // namespace Slic3r::Laser

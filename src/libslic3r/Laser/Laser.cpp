#include "libslic3r/Laser/Laser.hpp"

namespace Slic3r::Laser {

std::string export_gcode(const LaserDocument& doc, const LaserDevice& device, const GCodeOptions& opts, std::string* error)
{
    const LaserJob job = plan(doc, device);
    if (!job.ok()) {
        if (error) *error = job.error;
        return {};
    }
    return gcode(job, device, opts, error);
}

} // namespace Slic3r::Laser

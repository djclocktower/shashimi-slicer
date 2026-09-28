#pragma once

// Machine profiles: resources/cam/machines.json, with built-ins as the fallback.

#include "libslic3r/CAM/CamTypes.hpp"

#include <string>
#include <vector>

namespace Slic3r::CAM {

// "Generic 3-axis", "Generic 4-axis (A about X)", "GRBL router (Shapeoko/X-Carve class)",
// "3018-class desktop CNC", "LinuxCNC mill", "Mach3/Mach4 mill".
std::vector<MachineProfile> builtin_machines();

// Reads <resources_dir>/cam/machines.json; returns builtin_machines() when it is missing or bad.
std::vector<MachineProfile> load_machines(const std::string& resources_dir);

// Process-wide machine list used by the generators and the post (starts as builtin_machines()).
// ponytail: one global list; pass machines explicitly if two lists ever need to coexist.
void                  set_machines(std::vector<MachineProfile> machines);
// By name; the first machine of the list when not found.
const MachineProfile& find_machine(const std::string& name);

} // namespace Slic3r::CAM

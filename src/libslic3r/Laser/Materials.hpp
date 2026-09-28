#pragma once

// User libraries as JSON in the app data dir: the material library (laser_materials.json) and
// the device profiles (laser_devices.json).

#include "libslic3r/Laser/LaserTypes.hpp"

#include <string>
#include <vector>

namespace Slic3r::Laser {

// One library row (LightBurn: material -> thickness -> entries such as "Cut" / "Engrave").
struct MaterialEntry {
    std::string material;          // "Plywood"
    double      thickness_mm{0};   // 0 = not applicable (engraving)
    std::string description;       // "Cut", "Engrave", "Score"
    LaserLayer  settings;          // name/visible/output/priority are not applied on Assign
    std::string notes;
};

// Starting points for common materials: plywood, MDF, cardboard, leather, acrylic (CO2 only
// cuts), anodized aluminum (engrave), slate, paper; tuned for a ~10 W diode or a ~40 W CO2.
std::vector<MaterialEntry> default_materials(LaserSource source);
// False (+ plain-language error) when the file is missing or unreadable; `out` untouched then.
bool load_materials(const std::string& path, std::vector<MaterialEntry>& out, std::string* error = nullptr);
bool save_materials(const std::string& path, const std::vector<MaterialEntry>& entries, std::string* error = nullptr);

// Built-in profiles: "Generic GRBL diode" (400x400, front-left, S1000, M4), "Ortur/Atomstack/
// Sculpfun-class diode", "Generic GRBL CO2" (600x400), "grblHAL", "Marlin laser" (inline M3 I,
// S255) and "Smoothie/Cohesion3D". JSON keys are the struct member names; missing keys keep defaults.
std::vector<LaserDevice> default_devices();
bool load_devices(const std::string& path, std::vector<LaserDevice>& out, std::string* error = nullptr);
bool save_devices(const std::string& path, const std::vector<LaserDevice>& devices, std::string* error = nullptr);

} // namespace Slic3r::Laser

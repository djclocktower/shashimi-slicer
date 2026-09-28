#include "libslic3r/Laser/Materials.hpp"

#include <boost/filesystem/operations.hpp>
#include <boost/nowide/fstream.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace Slic3r::Laser {

using json = nlohmann::json;

namespace {

// Field lists: JSON keys are the member names; enums are stored as integers; missing keys keep
// the defaults, so older files load.
#define LASER_LAYER_FIELDS(X)                                                                                       \
    X(name) X(visible) X(output) X(mode) X(speed_mm_s) X(power_max) X(power_min) X(passes) X(interval_mm)           \
    X(angle_deg) X(bidirectional) X(crosshatch) X(overscan_pct) X(overscan_mm) X(fill_grouping) X(kerf_offset_mm) \
    X(lead_in_mm) X(tabs) X(tab_count) X(tab_spacing_mm) X(tab_size_mm) X(z_offset_mm) X(z_step_per_pass)          \
    X(dot_mode) X(dot_dwell_ms) X(dot_spacing_mm) X(air_assist) X(priority) X(image_dither) X(image_dpi)           \
    X(image_negative) X(image_pass_through) X(image_cells_per_inch) X(image_screen_angle_deg)

#define LASER_DEVICE_FIELDS(X)                                                                                      \
    X(name) X(type) X(source) X(bed_w) X(bed_h) X(origin_corner) X(s_max) X(laser_mode_dynamic)                    \
    X(uses_g0_for_travel) X(max_speed_mm_s) X(travel_speed_mm_s) X(accel_mm_s2) X(baud) X(enable_z)                \
    X(frame_power_pct) X(marlin_fan_laser) X(marlin_inline) X(start_gcode) X(end_gcode) X(air_on_gcode)            \
    X(air_off_gcode)

#define LASER_ROTARY_FIELDS(X) X(enabled) X(type) X(mm_per_rotation) X(roller_diameter) X(object_diameter)

#define TO_JSON(f) j[#f] = v.f;
#define FROM_JSON(f) if (j.contains(#f)) j.at(#f).get_to(v.f);

json to_json(const LaserLayer& v) { json j; LASER_LAYER_FIELDS(TO_JSON) return j; }
void from_json(const json& j, LaserLayer& v) { LASER_LAYER_FIELDS(FROM_JSON) }
json to_json(const RotarySettings& v) { json j; LASER_ROTARY_FIELDS(TO_JSON) return j; }
void from_json(const json& j, RotarySettings& v) { LASER_ROTARY_FIELDS(FROM_JSON) }
json to_json(const LaserDevice& v)
{
    json j;
    LASER_DEVICE_FIELDS(TO_JSON)
    j["rotary"] = to_json(v.rotary);
    return j;
}
void from_json(const json& j, LaserDevice& v)
{
    LASER_DEVICE_FIELDS(FROM_JSON)
    if (j.contains("rotary")) from_json(j.at("rotary"), v.rotary);
}

#undef TO_JSON
#undef FROM_JSON

bool read_json(const std::string& path, json& j, std::string* error)
{
    boost::nowide::ifstream f(path);
    if (!f) {
        if (error) *error = "The file \"" + path + "\" could not be opened.";
        return false;
    }
    try {
        f >> j;
    } catch (const std::exception&) {
        if (error) *error = "The file \"" + path + "\" is not valid JSON.";
        return false;
    }
    return true;
}

// Written to a temporary file first, so a failed write never destroys the existing library.
bool write_json(const std::string& path, const json& j, std::string* error)
{
    const std::string tmp = path + ".tmp";
    {
        boost::nowide::ofstream f(tmp);
        if (!(f << j.dump(2))) {
            if (error) *error = "The file \"" + path + "\" could not be written.";
            return false;
        }
    }
    boost::system::error_code ec;
    boost::filesystem::rename(tmp, path, ec);
    if (ec) {
        if (error) *error = "The file \"" + path + "\" could not be written: " + ec.message();
        return false;
    }
    return true;
}

MaterialEntry entry(const char* material, double thickness, const char* description, LayerMode mode, double speed,
                    double power, int passes = 1, double interval = 0.1)
{
    MaterialEntry e;
    e.material              = material;
    e.thickness_mm          = thickness;
    e.description           = description;
    e.settings.mode         = mode;
    e.settings.speed_mm_s   = speed;
    e.settings.power_max    = power;
    e.settings.power_min    = std::min(10., power);
    e.settings.passes       = passes;
    e.settings.interval_mm  = interval;
    e.notes                 = "Starting point: run a test grid on a scrap of your material.";
    return e;
}

} // namespace

std::vector<MaterialEntry> default_materials(LaserSource source)
{
    constexpr LayerMode Line = LayerMode::Line, Fill = LayerMode::Fill;
    if (source == LaserSource::Diode)   // ~10 W optical diode
        return {
            entry("Plywood", 3, "Cut", Line, 5, 100, 2),        entry("Plywood", 3, "Engrave", Fill, 100, 40),
            entry("MDF", 3, "Cut", Line, 4, 100, 2),            entry("MDF", 3, "Engrave", Fill, 120, 35),
            entry("Cardboard", 3, "Cut", Line, 15, 100),        entry("Cardboard", 3, "Engrave", Fill, 150, 30),
            entry("Leather", 2, "Cut", Line, 8, 100, 2),        entry("Leather", 2, "Engrave", Fill, 150, 25),
            entry("Paper", 0.2, "Cut", Line, 30, 60),
            entry("Anodized aluminum", 0, "Engrave", Fill, 40, 100, 1, 0.08),
            entry("Slate", 0, "Engrave", Fill, 80, 80),
        };
    return {   // ~40 W CO2
        entry("Plywood", 3, "Cut", Line, 15, 65),        entry("Plywood", 3, "Engrave", Fill, 300, 20),
        entry("MDF", 3, "Cut", Line, 12, 70),            entry("MDF", 3, "Engrave", Fill, 300, 18),
        entry("Acrylic", 3, "Cut", Line, 10, 70),        entry("Acrylic", 3, "Engrave", Fill, 300, 20),
        entry("Cardboard", 3, "Cut", Line, 40, 40),      entry("Cardboard", 3, "Engrave", Fill, 400, 15),
        entry("Leather", 2, "Cut", Line, 20, 45),        entry("Leather", 2, "Engrave", Fill, 300, 15),
        entry("Paper", 0.2, "Cut", Line, 60, 20),
        entry("Anodized aluminum", 0, "Engrave", Fill, 300, 30, 1, 0.08),
        entry("Slate", 0, "Engrave", Fill, 250, 35),
    };
}

bool load_materials(const std::string& path, std::vector<MaterialEntry>& out, std::string* error)
{
    json j;
    if (!read_json(path, j, error)) return false;
    try {
        std::vector<MaterialEntry> v;
        for (const json& e : j.at("materials")) {
            MaterialEntry m;
            m.material     = e.value("material", "");
            m.thickness_mm = e.value("thickness_mm", 0.);
            m.description  = e.value("description", "");
            m.notes        = e.value("notes", "");
            if (e.contains("settings")) from_json(e.at("settings"), m.settings);
            v.push_back(std::move(m));
        }
        out = std::move(v);
    } catch (const std::exception&) {
        if (error) *error = "The material library \"" + path + "\" is damaged.";
        return false;
    }
    return true;
}

bool save_materials(const std::string& path, const std::vector<MaterialEntry>& entries, std::string* error)
{
    json arr = json::array();
    for (const MaterialEntry& m : entries)
        arr.push_back({{"material", m.material}, {"thickness_mm", m.thickness_mm}, {"description", m.description},
                       {"notes", m.notes}, {"settings", to_json(m.settings)}});
    return write_json(path, json{{"version", 1}, {"materials", arr}}, error);
}

std::vector<LaserDevice> default_devices()
{
    std::vector<LaserDevice> v;
    LaserDevice d;   // struct defaults: GRBL diode 400 x 400, front-left, S1000, M4
    d.name = "Generic GRBL diode";
    v.push_back(d);

    d.name = "Ortur/Atomstack/Sculpfun-class diode";
    d.max_speed_mm_s = 166;   // 10000 mm/min
    d.travel_speed_mm_s = 100;
    d.accel_mm_s2 = 2500;
    d.frame_power_pct = 1;
    v.push_back(d);

    d = LaserDevice{};
    d.name = "Generic GRBL CO2";
    d.source = LaserSource::CO2;
    d.bed_w = 600;
    d.bed_h = 400;
    d.origin_corner = OriginCorner::RearLeft;
    d.max_speed_mm_s = 500;
    d.travel_speed_mm_s = 300;
    d.accel_mm_s2 = 3000;
    v.push_back(d);

    d = LaserDevice{};
    d.name = "grblHAL";
    d.type = DeviceType::GrblHal;
    d.max_speed_mm_s = 250;
    d.travel_speed_mm_s = 200;
    d.accel_mm_s2 = 2000;
    v.push_back(d);

    d = LaserDevice{};
    d.name = "Marlin laser";
    d.type = DeviceType::Marlin;
    d.bed_w = d.bed_h = 220;
    d.s_max = 255;
    d.marlin_inline = true;
    d.air_on_gcode.clear();
    d.air_off_gcode.clear();
    d.baud = 250000;
    v.push_back(d);

    d = LaserDevice{};
    d.name = "Smoothie/Cohesion3D";
    d.type = DeviceType::Smoothie;
    d.source = LaserSource::CO2;
    d.bed_w = 300;
    d.bed_h = 200;
    d.origin_corner = OriginCorner::RearLeft;
    d.s_max = 1;
    d.max_speed_mm_s = 400;
    d.travel_speed_mm_s = 300;
    d.air_on_gcode.clear();
    d.air_off_gcode.clear();
    v.push_back(d);
    return v;
}

std::vector<std::string> validate_device(LaserDevice& d)
{
    std::vector<std::string> msgs;
    const LaserDevice def;
    auto fix = [&](const char* field, double& v, double lo, double hi, double fallback) {
        const double old = v;
        v = std::isfinite(v) && v >= lo ? std::min(v, hi) : fallback;   // NaN / too small -> fallback
        if (!(v == old)) {   // NaN compares unequal too
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%g", v);
            msgs.push_back("Device \"" + d.name + "\": " + field + " was out of range; set to " + buf + ".");
        }
    };
    const double inf = std::numeric_limits<double>::infinity();
    fix("bed_w", d.bed_w, 10, 3000, 10);
    fix("bed_h", d.bed_h, 10, 3000, 10);
    fix("s_max", d.s_max, std::numeric_limits<double>::min(), inf, 1000);
    fix("frame_power_pct", d.frame_power_pct, 0, 20, 0);
    fix("max_speed_mm_s", d.max_speed_mm_s, std::numeric_limits<double>::min(), inf, def.max_speed_mm_s);
    fix("travel_speed_mm_s", d.travel_speed_mm_s, std::numeric_limits<double>::min(), inf, def.travel_speed_mm_s);
    fix("accel_mm_s2", d.accel_mm_s2, std::numeric_limits<double>::min(), inf, def.accel_mm_s2);
    return msgs;
}

bool load_devices(const std::string& path, std::vector<LaserDevice>& out, std::string* error, std::vector<std::string>* warnings)
{
    json j;
    if (!read_json(path, j, error)) return false;
    try {
        std::vector<LaserDevice> v;
        for (const json& e : j.at("devices")) {
            LaserDevice d;
            from_json(e, d);
            for (std::string& w : validate_device(d))
                if (warnings) warnings->push_back(std::move(w));
            v.push_back(std::move(d));
        }
        out = std::move(v);
    } catch (const std::exception&) {
        if (error) *error = "The device list \"" + path + "\" is damaged.";
        return false;
    }
    return true;
}

bool save_devices(const std::string& path, const std::vector<LaserDevice>& devices, std::string* error)
{
    json arr = json::array();
    for (const LaserDevice& d : devices) arr.push_back(to_json(d));
    return write_json(path, json{{"version", 1}, {"devices", arr}}, error);
}

} // namespace Slic3r::Laser

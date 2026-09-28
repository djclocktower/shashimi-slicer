#include "libslic3r/CAM/ToolLibrary.hpp"

#include <nlohmann/json.hpp>

#include <boost/nowide/fstream.hpp>

#include <algorithm>
#include <cstdio>

namespace Slic3r::CAM {

namespace {

const char* kTypeKeys[] = {"flat", "ball", "bull", "chamfer", "vbit", "drill", "spot", "tap"};

CamTool tool(ToolType type, double d, int flutes, double flute_len, double tip_angle = 118)
{
    CamTool t;
    t.type           = type;
    t.diameter       = d;
    t.flutes         = flutes;
    t.flute_length   = flute_len;
    t.shank_diameter = d < 3 ? 3.175 : d;
    t.overall_length = std::max(38.0, flute_len + 25.0);
    t.tip_angle_deg  = tip_angle;
    return t;
}

} // namespace

const char* tool_type_name(ToolType t)
{
    switch (t) {
    case ToolType::FlatEndMill: return "Flat end mill";
    case ToolType::BallEndMill: return "Ball end mill";
    case ToolType::BullEndMill: return "Bull nose end mill";
    case ToolType::ChamferMill: return "Chamfer mill";
    case ToolType::VBit: return "V-bit";
    case ToolType::Drill: return "Drill";
    case ToolType::SpotDrill: return "Spot drill";
    case ToolType::Tap: return "Tap";
    }
    return "Tool";
}

std::vector<CamTool> default_tools()
{
    std::vector<CamTool> out;
    auto add = [&](CamTool t, const char* name) {
        t.number = int(out.size()) + 1;
        t.name   = name;
        out.push_back(std::move(t));
    };
    CamTool eighth = tool(ToolType::FlatEndMill, 3.175, 2, 12.7);
    eighth.unit_inch = true;
    add(eighth, "1/8\" flat end mill");
    CamTool quarter = tool(ToolType::FlatEndMill, 6.35, 2, 19.05);
    quarter.unit_inch = true;
    add(quarter, "1/4\" flat end mill");
    add(tool(ToolType::FlatEndMill, 3, 2, 12), "3 mm flat end mill");
    add(tool(ToolType::FlatEndMill, 6, 3, 18), "6 mm flat end mill");
    add(tool(ToolType::FlatEndMill, 8, 3, 22), "8 mm flat end mill");
    add(tool(ToolType::FlatEndMill, 10, 3, 25), "10 mm flat end mill");
    add(tool(ToolType::BallEndMill, 3, 2, 12), "3 mm ball end mill");
    add(tool(ToolType::BallEndMill, 6, 2, 18), "6 mm ball end mill");
    CamTool chamfer = tool(ToolType::ChamferMill, 6, 2, 6, 90);
    add(chamfer, "90\xC2\xB0 chamfer mill");
    CamTool v60 = tool(ToolType::VBit, 6.35, 2, 6, 60);
    v60.shank_diameter = 6.35;
    add(v60, "60\xC2\xB0 V-bit");
    CamTool v90 = tool(ToolType::VBit, 6.35, 2, 4, 90);
    v90.shank_diameter = 6.35;
    add(v90, "90\xC2\xB0 V-bit");
    for (double d : {2., 3., 4., 5., 6., 8., 10.}) {
        CamTool t = tool(ToolType::Drill, d, 2, 10 * d);
        t.material = ToolMaterial::HSS;
        char name[32];
        snprintf(name, sizeof(name), "%g mm drill", d);
        add(t, name);
    }
    return out;
}

bool load_tool_library(const std::string& path, std::vector<CamTool>& tools, std::string* error)
{
    try {
        boost::nowide::ifstream f(path);
        if (!f) {
            if (error) *error = "The tool library file could not be opened: " + path;
            return false;
        }
        const nlohmann::json j = nlohmann::json::parse(f);
        std::vector<CamTool> out;
        for (const nlohmann::json& e : j.at("tools")) {
            CamTool t;
            t.number = e.value("number", int(out.size()) + 1);
            t.name   = e.value("name", std::string());
            const std::string type = e.value("type", std::string("flat"));
            for (int i = 0; i < 8; ++i)
                if (type == kTypeKeys[i])
                    t.type = ToolType(i);
            t.diameter       = e.value("diameter", t.diameter);
            t.corner_radius  = e.value("corner_radius", t.corner_radius);
            t.tip_angle_deg  = e.value("tip_angle_deg", t.tip_angle_deg);
            t.tip_diameter   = e.value("tip_diameter", t.tip_diameter);
            t.flute_length   = e.value("flute_length", t.flute_length);
            t.overall_length = e.value("overall_length", t.overall_length);
            t.shank_diameter = e.value("shank_diameter", t.shank_diameter);
            t.flutes         = e.value("flutes", t.flutes);
            t.material       = e.value("material", std::string("carbide")) == "hss" ? ToolMaterial::HSS : ToolMaterial::Carbide;
            t.coating        = e.value("coating", std::string());
            t.thread_pitch   = e.value("thread_pitch", t.thread_pitch);
            t.unit_inch      = e.value("unit_inch", t.unit_inch);
            t.override_feeds = e.value("override_feeds", t.override_feeds);
            if (e.contains("feeds")) {
                const nlohmann::json& fj = e["feeds"];
                t.feeds.rpm           = fj.value("rpm", t.feeds.rpm);
                t.feeds.feed          = fj.value("feed", t.feeds.feed);
                t.feeds.plunge_feed   = fj.value("plunge_feed", t.feeds.plunge_feed);
                t.feeds.ramp_feed     = fj.value("ramp_feed", t.feeds.ramp_feed);
                t.feeds.chipload      = fj.value("chipload", t.feeds.chipload);
                t.feeds.surface_speed = fj.value("surface_speed", t.feeds.surface_speed);
            }
            out.push_back(std::move(t));
        }
        tools = std::move(out);
        return true;
    } catch (const std::exception& ex) {
        if (error) *error = std::string("The tool library file is damaged: ") + ex.what();
        return false;
    }
}

bool save_tool_library(const std::string& path, const std::vector<CamTool>& tools, std::string* error)
{
    nlohmann::json arr = nlohmann::json::array();
    for (const CamTool& t : tools) {
        const FeedsSpeeds& f = t.feeds;
        arr.push_back({{"number", t.number},
                       {"name", t.name},
                       {"type", kTypeKeys[std::clamp(int(t.type), 0, 7)]},
                       {"diameter", t.diameter},
                       {"corner_radius", t.corner_radius},
                       {"tip_angle_deg", t.tip_angle_deg},
                       {"tip_diameter", t.tip_diameter},
                       {"flute_length", t.flute_length},
                       {"overall_length", t.overall_length},
                       {"shank_diameter", t.shank_diameter},
                       {"flutes", t.flutes},
                       {"material", t.material == ToolMaterial::HSS ? "hss" : "carbide"},
                       {"coating", t.coating},
                       {"thread_pitch", t.thread_pitch},
                       {"unit_inch", t.unit_inch},
                       {"override_feeds", t.override_feeds},
                       {"feeds", {{"rpm", f.rpm}, {"feed", f.feed}, {"plunge_feed", f.plunge_feed}, {"ramp_feed", f.ramp_feed},
                                  {"chipload", f.chipload}, {"surface_speed", f.surface_speed}}}});
    }
    try {
        boost::nowide::ofstream f(path);
        if (f) {
            f << nlohmann::json{{"version", 1}, {"tools", arr}}.dump(2);
            f.close();
        }
        if (!f) {
            if (error) *error = "The tool library could not be saved to " + path;
            return false;
        }
    } catch (const std::exception& ex) {
        if (error) *error = std::string("The tool library could not be saved: ") + ex.what();
        return false;
    }
    return true;
}

} // namespace Slic3r::CAM

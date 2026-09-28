#include "libslic3r/CAM/Machines.hpp"

#include <nlohmann/json.hpp>

#include <boost/nowide/fstream.hpp>

#include <mutex>

namespace Slic3r::CAM {

namespace {

MachineProfile make(const char* name, double min_rpm, double max_rpm, double feed_xy, double feed_z, double rapid, Vec3d travel,
                    PostDialect post, ToolChange tc, bool a_axis = false, bool coolant = false)
{
    MachineProfile m;
    m.name        = name;
    m.min_rpm     = min_rpm;
    m.max_rpm     = max_rpm;
    m.max_feed_xy = feed_xy;
    m.max_feed_z  = feed_z;
    m.rapid_feed  = rapid;
    m.travel      = travel;
    m.post        = post;
    m.tool_change = tc;
    m.has_a_axis  = a_axis;
    m.coolant     = coolant;
    return m;
}

const char* kDialectKeys[] = {"grbl", "linuxcnc", "mach3", "fanuc", "marlin"};

std::mutex                  g_mutex;
std::vector<MachineProfile> g_machines = builtin_machines();

} // namespace

std::vector<MachineProfile> builtin_machines()
{
    using PD = PostDialect;
    using TC = ToolChange;
    return {
        make("Generic 3-axis", 0, 24000, 5000, 1000, 5000, {300, 300, 100}, PD::Grbl, TC::ManualPause),
        make("Generic 4-axis (A about X)", 0, 24000, 3000, 1000, 4000, {300, 200, 150}, PD::LinuxCNC, TC::ManualPause, true),
        make("GRBL router (Shapeoko/X-Carve class)", 10000, 30000, 5000, 1500, 5000, {830, 830, 100}, PD::Grbl, TC::ManualPause),
        make("3018-class desktop CNC", 0, 10000, 1000, 500, 1500, {300, 180, 45}, PD::Grbl, TC::ManualPause),
        make("LinuxCNC mill", 100, 5000, 2500, 1000, 3000, {400, 200, 300}, PD::LinuxCNC, TC::M6, false, true),
        make("Mach3/Mach4 mill", 100, 5000, 2500, 1000, 3000, {400, 200, 300}, PD::Mach3, TC::M6, false, true),
    };
}

std::vector<MachineProfile> load_machines(const std::string& resources_dir)
{
    try {
        boost::nowide::ifstream f(resources_dir + "/cam/machines.json");
        if (!f)
            return builtin_machines();
        const nlohmann::json        j = nlohmann::json::parse(f);
        std::vector<MachineProfile> out;
        for (const nlohmann::json& e : j.at("machines")) {
            MachineProfile m;
            m.name            = e.at("name").get<std::string>();
            m.max_rpm         = e.value("max_rpm", m.max_rpm);
            m.min_rpm         = e.value("min_rpm", m.min_rpm);
            m.max_feed_xy     = e.value("max_feed_xy", m.max_feed_xy);
            m.max_feed_z      = e.value("max_feed_z", m.max_feed_z);
            m.rapid_feed      = e.value("rapid_feed", m.rapid_feed);
            if (e.contains("travel"))
                m.travel = Vec3d(e["travel"].at(0).get<double>(), e["travel"].at(1).get<double>(), e["travel"].at(2).get<double>());
            m.has_a_axis      = e.value("has_a_axis", m.has_a_axis);
            const std::string post = e.value("post", std::string("grbl"));
            for (int i = 0; i < 5; ++i)
                if (post == kDialectKeys[i])
                    m.post = PostDialect(i);
            m.tool_change     = e.value("tool_change", std::string("pause")) == "m6" ? ToolChange::M6 : ToolChange::ManualPause;
            m.spindle_control = e.value("spindle_control", m.spindle_control);
            m.coolant         = e.value("coolant", m.coolant);
            out.push_back(std::move(m));
        }
        return out.empty() ? builtin_machines() : out;
    } catch (...) {
        return builtin_machines();
    }
}

void set_machines(std::vector<MachineProfile> machines)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_machines = machines.empty() ? builtin_machines() : std::move(machines);
}

const MachineProfile& find_machine(const std::string& name)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    for (const MachineProfile& m : g_machines)
        if (m.name == name)
            return m;
    return g_machines.front();
}

} // namespace Slic3r::CAM

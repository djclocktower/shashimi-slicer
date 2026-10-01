#pragma once

// Helpers shared by the MCP tool modules. Everything here runs on the wx main thread.

#include "McpServer.hpp"

#include <string>
#include <vector>

#include <wx/string.h>

#include "libslic3r/Point.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/Model.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/Plater.hpp"

namespace Slic3r { namespace GUI { namespace Mcp {

inline Plater& plater()
{
    Plater* p = wxGetApp().plater();
    if (p == nullptr)
        throw ToolError("the application is not initialised yet", -32001);
    return *p;
}

inline std::string utf8(const wxString& s) { return s.utf8_string(); }
inline wxString    from_utf8(const std::string& s) { return wxString::FromUTF8(s); }

inline json vec3(const Vec3d& v) { return json::array({v.x(), v.y(), v.z()}); }

inline Vec3d to_vec3(const json& params, const char* key)
{
    const json& j = params.at(key);
    if (!j.is_array() || j.size() != 3 || !j[0].is_number() || !j[1].is_number() || !j[2].is_number())
        throw ToolError(std::string("parameter '") + key + "' must be [x, y, z]", -32602);
    return Vec3d(j[0].get<double>(), j[1].get<double>(), j[2].get<double>());
}

inline int object_index(const json& params, const char* key = "object")
{
    const int idx = req<int>(params, key);
    if (idx < 0 || idx >= int(plater().model().objects.size()))
        throw ToolError(std::string("no object ") + std::to_string(idx) + " (there are " +
                            std::to_string(plater().model().objects.size()) + "; see objects_list)", -32602);
    return idx;
}

inline std::vector<int> object_indices(const json& params, const char* key = "objects")
{
    std::vector<int> out;
    const size_t     n = plater().model().objects.size();
    for (int idx : req<std::vector<int>>(params, key)) {
        if (idx < 0 || size_t(idx) >= n)
            throw ToolError("no object " + std::to_string(idx) + " (see objects_list)", -32602);
        out.push_back(idx);
    }
    return out;
}

inline Preset::Type preset_type(const std::string& name)
{
    if (name == "print" || name == "process") return Preset::TYPE_PRINT;
    if (name == "filament")                   return Preset::TYPE_FILAMENT;
    if (name == "printer" || name == "machine") return Preset::TYPE_PRINTER;
    throw ToolError("preset type must be print, filament or printer (got '" + name + "')", -32602);
}

inline std::string preset_type_name(Preset::Type t)
{
    switch (t) {
    case Preset::TYPE_PRINT:    return "print";
    case Preset::TYPE_FILAMENT: return "filament";
    case Preset::TYPE_PRINTER:  return "printer";
    default:                    return "other";
    }
}

// Remove "default" from a descriptor built by param_enum() when the parameter is required.
inline json required(json p)
{
    p.erase("default");
    return p;
}

inline json param_preset_type(const std::string& description = "print | filament | printer")
{
    return required(param_enum("type", json::array({"print", "filament", "printer"}), description, json()));
}

}}} // namespace Slic3r::GUI::Mcp

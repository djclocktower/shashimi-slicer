#pragma once

// Tool library: built-in default set + the user's JSON file (cam_tools.json in the app data dir).

#include "libslic3r/CAM/CamTypes.hpp"

#include <string>
#include <vector>

namespace Slic3r::CAM {

// Plain-language tool type name ("Flat end mill", "V-bit", ...).
const char* tool_type_name(ToolType t);

// 1/8", 1/4", 3, 6, 8, 10 mm flat; 3 and 6 mm ball; 90 deg chamfer; 60/90 deg V-bit; drills
// 2-10 mm. Numbered 1..N, names like "6 mm flat end mill".
std::vector<CamTool> default_tools();

// False (+ error) when the file is missing or unreadable; `tools` untouched then.
bool load_tool_library(const std::string& path, std::vector<CamTool>& tools, std::string* error = nullptr);
bool save_tool_library(const std::string& path, const std::vector<CamTool>& tools, std::string* error = nullptr);

} // namespace Slic3r::CAM

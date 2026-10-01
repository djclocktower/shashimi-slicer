#ifndef slic3r_GUI_McpControl_hpp_
#define slic3r_GUI_McpControl_hpp_

// Design/CAM tab half of the MCP control surface. The transport, the threading and the
// slicer-side tools live in slic3r/GUI/Mcp/McpServer; every method the server does not
// register itself is handed to cad_mcp_handle_on_main(). All CAD work runs through the SAME
// CadDocument kernel the GUI uses (no parallel engine).

#include <string>
#include <nlohmann/json.hpp>

namespace Slic3r { namespace GUI {

// The Design tab's tool descriptors: {"tools": [...], "id_lifetime": "...", ...}.
nlohmann::json cad_mcp_describe_tools();

// Serve one Design/CAM method ON THE MAIN THREAD. Returns a complete JSON-RPC reply string;
// an unknown method is a -32601 error.
std::string cad_mcp_handle_on_main(const std::string& method, const nlohmann::json& params, const nlohmann::json& id);

}} // namespace Slic3r::GUI

#endif // slic3r_GUI_McpControl_hpp_

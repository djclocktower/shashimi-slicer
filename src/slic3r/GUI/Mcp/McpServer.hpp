#pragma once

// App-wide MCP control surface: a local JSON-RPC 2.0 server that lets an external MCP
// bridge (tools/orca_mcp_bridge.py) drive and perceive the whole application -- projects,
// objects, plates, presets, every setting, slicing, export, printers, the Design tab, and
// through the ui_* tools any window, dialog or menu. See docs/HLSD/mcp-control.md.
//
// Off unless an environment variable names the endpoint:
//   ORCA_MCP=1          -> /tmp/orca-mcp.sock          (Windows: \\.\pipe\orca-mcp)
//   ORCA_MCP=<path>     -> that socket path            (Windows: that pipe name)
//   ORCA_CAD_MCP=...    -> legacy spelling, default /tmp/orca-cad-mcp.sock
//
// Every tool runs on the wx main thread through the same code paths the GUI uses.

#include <nlohmann/json.hpp>

#include <functional>
#include <stdexcept>
#include <string>

namespace Slic3r { namespace GUI { namespace Mcp {

using json = nlohmann::json;

// Thrown by a tool to answer with a JSON-RPC error instead of a result.
struct ToolError : std::runtime_error
{
    int code;
    explicit ToolError(const std::string& msg, int code = -32000) : std::runtime_error(msg), code(code) {}
};

enum ToolFlags : unsigned {
    // Safe to run while a modal dialog is open or another tool is still on the UI thread.
    // Only perception and the ui_* tools carry it: everything else is refused then, because
    // it would run inside the dialog's nested event loop, under the operation that opened it.
    ModalSafe = 1 << 0,
    // Accepts `wait: true`: after the tool returns, the server waits (off the UI thread) until
    // slicing, export and UI jobs are idle, then runs Tool::after_wait for the final answer.
    Waitable  = 1 << 1,
    // May legitimately keep the UI thread busy for long (file loading, exports).
    Long      = 1 << 2,
};

struct Tool
{
    std::string                     name;
    std::string                     summary;
    json                            params = json::array();   // see param() below
    unsigned                        flags  = 0;
    std::function<json(const json&)> run;
    // For Waitable tools: the answer after waiting (receives the call's params).
    std::function<json(const json&)> after_wait;
};

void register_tool(Tool tool);

// Descriptor of one parameter, in the shape describe_tools has always used:
// {name, type, description, default?, enum?}. A parameter without a default is required.
json param(const std::string& name, const std::string& type, const std::string& description);
json param(const std::string& name, const std::string& type, const std::string& description, json default_value);
json param_enum(const std::string& name, json values, const std::string& description, json default_value);

// Typed parameter access. A wrong type or a missing required key is a -32602 ToolError
// naming the key, never an uncaught nlohmann exception.
template<class T> T arg(const json& params, const char* key, T def)
{
    if (!params.is_object() || !params.contains(key) || params[key].is_null())
        return def;
    try { return params[key].get<T>(); }
    catch (const std::exception&) { throw ToolError(std::string("parameter '") + key + "' has the wrong type", -32602); }
}
template<class T> T req(const json& params, const char* key)
{
    if (!params.is_object() || !params.contains(key) || params[key].is_null())
        throw ToolError(std::string("missing required parameter '") + key + "'", -32602);
    return arg<T>(params, key, T());
}

// Start the server thread iff ORCA_MCP / ORCA_CAD_MCP is set. Call once, after the MainFrame
// exists. Registers every tool module on first call.
void start_server_if_enabled();

// Tool modules (explicit, so the linker cannot drop them from the static GUI library).
void register_app_tools();
void register_slicer_tools();
void register_ui_tools();

// Busy state used by wait_idle and Waitable tools; provided by the slicer tool module.
// {"idle": bool, ...details}
json app_busy_state();

// Title of the first open modal dialog, or empty.
std::string open_modal_title();

}}} // namespace Slic3r::GUI::Mcp

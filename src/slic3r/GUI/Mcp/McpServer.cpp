#include "McpServer.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include <boost/log/trivial.hpp>
#include <wx/dialog.h>
#include <wx/evtloop.h>
#include <wx/toplevel.h>

#include "slic3r/GUI/GUI_App.hpp"
#ifdef SLIC3R_CAD
#include "slic3r/GUI/CAD/McpControl.hpp"
#endif

#ifdef _WIN32
#include <windows.h>
#include <wx/msw/winundef.h>   // windows.h redefines GetClassName & co. as macros
#else
#include <sys/socket.h>
#include <sys/stat.h>   // umask/chmod: the socket's file mode IS its access control
#include <sys/un.h>
#include <unistd.h>
#endif

namespace Slic3r { namespace GUI { namespace Mcp {

namespace {

using Clock = std::chrono::steady_clock;

// Written once by start_server_if_enabled() on the main thread before the server thread
// exists, read-only afterwards -- so the server thread may look tools up without a lock.
std::map<std::string, Tool>& registry()
{
    static std::map<std::string, Tool> tools;
    return tools;
}

const Tool* find_tool(const std::string& name)
{
    auto it = registry().find(name);
    return it == registry().end() ? nullptr : &it->second;
}

// Main thread only: how many tools are currently on the UI thread's stack. Above zero, a
// new request is being served from inside a nested event loop opened by an earlier one.
int g_depth = 0;
struct DepthGuard {
    DepthGuard() { ++g_depth; }
    ~DepthGuard() { --g_depth; }
};

std::string rpc_result(const json& id, const json& result)
{
    // replace: tool output may carry bytes that are not UTF-8 (file contents, mesh names),
    // and a throwing dump() on the server thread would terminate the application.
    return json{{"jsonrpc", "2.0"}, {"id", id}, {"result", result}}.dump(-1, ' ', false, json::error_handler_t::replace);
}
std::string rpc_error(const json& id, int code, const std::string& msg, const json& data = json())
{
    json err{{"code", code}, {"message", msg}};
    if (!data.is_null())
        err["data"] = data;
    return json{{"jsonrpc", "2.0"}, {"id", id}, {"error", err}}.dump(-1, ' ', false, json::error_handler_t::replace);
}

// ---- running work on the UI thread -------------------------------------------------------

struct Outcome
{
    enum Kind { Ok, Error, Timeout, Blocked } kind = Error;
    json        result;
    int         code = -32000;
    std::string message;

    static Outcome ok(json r) { Outcome o; o.kind = Ok; o.result = std::move(r); return o; }
    static Outcome error(int code, std::string msg) { Outcome o; o.kind = Error; o.code = code; o.message = std::move(msg); return o; }
};

struct Pending
{
    std::mutex              m;
    std::condition_variable cv;
    bool                    started = false;
    bool                    done    = false;
    Outcome                 out;
};

// Post fn to the UI thread and wait for it. Nothing may escape the posted lambda: it is
// invoked by the wx event loop, which does not catch, so an escaping exception would be
// std::terminate and the socket a way for any client to kill the application.
std::shared_ptr<Pending> post(std::function<Outcome()> fn)
{
    auto st = std::make_shared<Pending>();
    wxGetApp().CallAfter([st, fn = std::move(fn)]() {
        { std::lock_guard<std::mutex> lk(st->m); st->started = true; }
        Outcome o;
        try { o = fn(); }
        catch (const std::exception& ex) { o = Outcome::error(-32000, std::string("internal error: ") + ex.what()); }
        catch (...) { o = Outcome::error(-32000, "internal error: unknown exception"); }
        { std::lock_guard<std::mutex> lk(st->m); st->out = std::move(o); st->done = true; }
        st->cv.notify_all();
    });
    return st;
}

// What modal dialog, if any, is up right now -- asked of the UI thread. Empty when there is
// none or when the UI thread is busy computing (then it is not waiting on a dialog either).
std::string probe_modal()
{
    auto st = post([] { return Outcome::ok(open_modal_title()); });
    std::unique_lock<std::mutex> lk(st->m);
    if (!st->cv.wait_for(lk, std::chrono::seconds(2), [&] { return st->done; }))
        return {};
    return st->out.kind == Outcome::Ok ? st->out.result.get<std::string>() : std::string();
}

// Run fn on the UI thread and wait up to `timeout`. With watch_modal, a call whose handler
// has started but sits behind a modal dialog is answered early as Blocked, naming the dialog:
// the agent can then answer it with ui_* tools. The operation itself carries on afterwards.
Outcome call_on_main(std::function<Outcome()> fn, std::chrono::seconds timeout, bool watch_modal)
{
    auto       st         = post(std::move(fn));
    const auto deadline   = Clock::now() + timeout;
    auto       next_probe = Clock::now() + std::chrono::milliseconds(800);
    std::unique_lock<std::mutex> lk(st->m);
    while (!st->done) {
        st->cv.wait_until(lk, std::min(deadline, next_probe));
        if (st->done)
            break;
        const auto now = Clock::now();
        if (now >= deadline) {
            Outcome o;
            o.kind    = Outcome::Timeout;
            o.message = "still running on the UI thread after " + std::to_string(timeout.count()) +
                        " s; it continues in the background -- call wait_idle, then inspect the state";
            return o;
        }
        if (watch_modal && st->started && now >= next_probe) {
            lk.unlock();
            const std::string title = probe_modal();
            lk.lock();
            if (!title.empty() && !st->done) {
                Outcome o;
                o.kind    = Outcome::Blocked;
                o.message = title;
                return o;
            }
            next_probe = Clock::now() + std::chrono::milliseconds(800);
        }
    }
    return st->out;
}

// ---- dispatch (UI thread) -----------------------------------------------------------------

// Posted calls also run inside nested event loops (modal dialogs, wxYield in progress bars and
// popup menus), i.e. in the middle of whatever GUI operation opened that loop.
bool in_nested_loop()
{
    const wxEventLoopBase* active = wxEventLoopBase::GetActive();
    return active != nullptr && (active != wxTheApp->GetMainLoop() || active->IsYielding());
}

std::string busy_reason(const std::string& modal)
{
    if (!modal.empty())
        return "a modal dialog is open (\"" + modal + "\"); answer it first with ui_windows / ui_tree / ui_click";
    return "the UI is in the middle of another operation; call wait_idle first";
}

Outcome run_on_main(const std::string& method, const json& params)
{
    const std::string modal = open_modal_title();
    if (const Tool* tool = find_tool(method)) {
        if (!(tool->flags & ModalSafe) && (g_depth > 0 || !modal.empty() || in_nested_loop()))
            return Outcome::error(-32003, method + " refused: " + busy_reason(modal));
        DepthGuard guard;
        try {
            return Outcome::ok(tool->run(params));
        } catch (const ToolError& ex) {
            return Outcome::error(ex.code, ex.what());
        } catch (const std::exception& ex) {
            return Outcome::error(-32000, ex.what());
        }
    }
#ifdef SLIC3R_CAD
    // Everything not registered here is a Design/CAM tab method, served by its own module.
    if (g_depth > 0 || !modal.empty() || in_nested_loop())
        return Outcome::error(-32003, method + " refused: " + busy_reason(modal));
    DepthGuard  guard;
    const json  reply = json::parse(cad_mcp_handle_on_main(method, params, json(nullptr)));
    if (reply.contains("error"))
        return Outcome::error(reply["error"].value("code", -32000), reply["error"].value("message", std::string("error")));
    return Outcome::ok(reply.value("result", json()));
#else
    return Outcome::error(-32601, "Unknown method: " + method);
#endif
}

// ---- server-thread tools -------------------------------------------------------------------

json wait_idle(double timeout_s)
{
    const auto start = Clock::now();
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - start).count(); };
    int  streak  = 0;
    json last    = json::object();
    for (;;) {
        Outcome o = call_on_main([] {
            json s     = app_busy_state();
            s["modal"] = open_modal_title();
            return Outcome::ok(s);
        }, std::chrono::seconds(10), false);
        if (o.kind == Outcome::Ok) {
            last = o.result;
            if (!last.value("modal", std::string()).empty()) {
                last["blocked_by_dialog"] = true;
                break;
            }
            // Twice in a row: "slice all" passes through an idle instant between plates.
            streak = last.value("idle", false) ? streak + 1 : 0;
            if (streak >= 2)
                break;
        } else {
            streak       = 0;   // UI thread busy computing: not idle
            last["idle"] = false;
        }
        if (elapsed() >= timeout_s) {
            last["timed_out"] = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    last["waited_s"] = std::round(elapsed() * 100.) / 100.;
    return last;
}

json describe_tools()
{
    json tools = json::array();
    for (const auto& [name, t] : registry()) {
        json d{{"name", name}, {"summary", t.summary}, {"params", t.params}};
        if (t.flags & Waitable) {
            d["params"].push_back(param("wait", "boolean", "wait until slicing/export/jobs are idle and return the resulting status", false));
            d["params"].push_back(param("timeout", "number", "with wait: give up after this many seconds", 600));
        }
        tools.push_back(std::move(d));
    }
    tools.push_back(json{{"name", "wait_idle"},
                         {"summary", "Wait until slicing, G-code export and background jobs (arrange, orient...) are idle, or a modal dialog needs answering."},
                         {"params", json::array({param("timeout", "number", "seconds", 300)})}});
    json out{{"app", SLIC3R_APP_NAME},
             {"version", SLIC3R_VERSION},
             {"protocol", "jsonrpc-2.0"},
             {"conventions",
              "Indices (object, volume, instance, plate, filament slot) are 0-based and refer to the current state: "
              "re-read objects_list / plates_list after anything that adds or removes items. Lengths are mm, angles degrees. "
              "A call that opens a modal dialog is answered early with error -32004 naming the dialog; answer it with "
              "ui_windows / ui_tree / ui_click, then call wait_idle. While a dialog is open only perception and ui_* tools run."}};
#ifdef SLIC3R_CAD
    // Design/CAM tab tools, served by their own module. A slicer tool of the same name would
    // shadow one; describe only the reachable ones.
    json cad = cad_mcp_describe_tools();
    for (auto& t : cad["tools"])
        if (!find_tool(t.value("name", std::string())))
            tools.push_back(t);
    out["design_tab"] = {{"note", "Design (CAD) tools: describe_scene, sketch_*, extrude... and cam_* for the CAM tab."},
                         {"id_lifetime", cad.value("id_lifetime", std::string())}};
#endif
    out["tools"] = std::move(tools);
    return out;
}

// ---- one request ---------------------------------------------------------------------------

std::string handle_line(const std::string& line)
{
    json req;
    try { req = json::parse(line); }
    catch (const std::exception& ex) { return rpc_error(nullptr, -32700, std::string("parse error: ") + ex.what()); }
    if (!req.is_object())
        return rpc_error(nullptr, -32600, "request must be a JSON object");

    const json        id     = req.contains("id") ? req["id"] : json(nullptr);
    const std::string method = req.value("method", std::string());
    json              params = req.contains("params") ? req["params"] : json::object();
    if (method.empty())
        return rpc_error(id, -32600, "missing method");
    if (params.is_null())
        params = json::object();
    if (!params.is_object())
        return rpc_error(id, -32602, "params must be an object");

    const Tool*    tool  = find_tool(method);
    const unsigned flags = tool ? tool->flags : 0;
    const auto     limit = std::chrono::seconds((flags & Long) ? 600 : 60);
    bool           wait  = false;
    double         wait_timeout = 600.;
    try {
        if (method == "wait_idle")
            return rpc_result(id, wait_idle(arg<double>(params, "timeout", 300.)));
        // Checked before dispatch: a malformed value must not be found after the tool ran.
        wait         = tool && (flags & Waitable) && arg<bool>(params, "wait", false);
        wait_timeout = arg<double>(params, "timeout", 600.);
    } catch (const ToolError& ex) {
        return rpc_error(id, ex.code, ex.what());
    }

    Outcome o = call_on_main([method, params] { return run_on_main(method, params); }, limit, !(flags & ModalSafe));
    switch (o.kind) {
    case Outcome::Error:   return rpc_error(id, o.code, o.message);
    case Outcome::Timeout: return rpc_error(id, -32005, method + ": " + o.message);
    case Outcome::Blocked:
        return rpc_error(id, -32004,
                         method + " is waiting on a modal dialog (\"" + o.message + "\"). Answer it with ui_windows / ui_tree / "
                         "ui_click; the operation then continues -- call wait_idle and inspect the result.",
                         json{{"dialog", o.message}});
    case Outcome::Ok: break;
    }

    json result = std::move(o.result);
    if (wait) {
        json waited = wait_idle(wait_timeout);
        if (tool->after_wait && !waited.value("blocked_by_dialog", false)) {
            const auto after = tool->after_wait;
            Outcome    s     = call_on_main([after, params] { return Outcome::ok(after(params)); }, std::chrono::seconds(60), false);
            if (s.kind == Outcome::Ok)
                waited["status"] = std::move(s.result);
        }
        if (!result.is_object())
            result = json{{"value", result}};
        result["wait"] = std::move(waited);
    }
    return rpc_result(id, result);
}

// Serve newline-delimited requests read through `read_some` until EOF or error.
template<class ReadFn, class WriteFn> void serve_lines(ReadFn read_some, WriteFn write_all)
{
    constexpr size_t max_request = 64u << 20;   // a client streaming without newlines must not exhaust memory
    std::string buf;
    char        chunk[4096];
    for (;;) {
        const long n = read_some(chunk, sizeof(chunk));
        if (n <= 0)
            return;
        buf.append(chunk, size_t(n));
        size_t nl;
        while ((nl = buf.find('\n')) != std::string::npos) {
            std::string line = buf.substr(0, nl);
            buf.erase(0, nl + 1);
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            if (line.empty())
                continue;
            std::string reply;
            try {
                reply = handle_line(line);
            } catch (const std::exception& ex) {   // a detached thread: an escaping exception is std::terminate
                reply = rpc_error(nullptr, -32000, std::string("internal error: ") + ex.what());
            } catch (...) {
                reply = rpc_error(nullptr, -32000, "internal error");
            }
            reply.push_back('\n');
            if (!write_all(reply))
                return;
        }
        if (buf.size() > max_request)
            return;
    }
}

#ifdef _WIN32

void serve_pipe_client(HANDLE h)
{
    serve_lines(
        [h](char* data, size_t size) -> long {
            DWORD n = 0;
            return ::ReadFile(h, data, DWORD(size), &n, nullptr) ? long(n) : -1;
        },
        [h](const std::string& s) {
            DWORD n = 0;
            return ::WriteFile(h, s.data(), DWORD(s.size()), &n, nullptr) && n == s.size();
        });
    ::FlushFileBuffers(h);
    ::DisconnectNamedPipe(h);
    ::CloseHandle(h);
}

// A named pipe's default DACL gives full control to the creating user, LocalSystem and
// administrators and only read access to everyone else -- who therefore cannot send a
// request. Remote clients are rejected outright, and the first instance claims the name so
// another process cannot squat it.
void server_thread(std::string name)
{
    const std::wstring wname = wxString::FromUTF8(name).ToStdWstring();
    bool               first = true;
    for (;;) {
        HANDLE h = ::CreateNamedPipeW(wname.c_str(), PIPE_ACCESS_DUPLEX | (first ? FILE_FLAG_FIRST_PIPE_INSTANCE : 0),
                                      PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT | PIPE_REJECT_REMOTE_CLIENTS,
                                      PIPE_UNLIMITED_INSTANCES, 1 << 16, 1 << 16, 0, nullptr);
        if (h == INVALID_HANDLE_VALUE) {
            BOOST_LOG_TRIVIAL(error) << "MCP: CreateNamedPipe failed on " << name << " (" << ::GetLastError() << ")";
            return;
        }
        if (first)
            BOOST_LOG_TRIVIAL(info) << "MCP control listening on " << name;
        first = false;
        if (!::ConnectNamedPipe(h, nullptr) && ::GetLastError() != ERROR_PIPE_CONNECTED) {
            ::CloseHandle(h);
            continue;
        }
        std::thread(serve_pipe_client, h).detach();
    }
}

std::string endpoint_from_env(const char* value, bool legacy)
{
    std::string v = value;
    if (v == "1")
        return legacy ? R"(\\.\pipe\orca-cad-mcp)" : R"(\\.\pipe\orca-mcp)";
    return v.rfind(R"(\\.\pipe\)", 0) == 0 ? v : R"(\\.\pipe\)" + v;
}

#else // POSIX

void serve_socket_client(int cfd)
{
    serve_lines(
        [cfd](char* data, size_t size) -> long { return long(::read(cfd, data, size)); },
        [cfd](const std::string& s) {
            // Never a bare write(): a client that hangs up between its request and our reply
            // raises SIGPIPE, whose default action kills the process.
#ifdef MSG_NOSIGNAL
            return ::send(cfd, s.data(), s.size(), MSG_NOSIGNAL) == ssize_t(s.size());
#else
            return ::write(cfd, s.data(), s.size()) == ssize_t(s.size());   // SO_NOSIGPIPE set at accept
#endif
        });
    ::close(cfd);
}

void server_thread(std::string sock_path)
{
    ::unlink(sock_path.c_str());
    int sfd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (sfd < 0) { BOOST_LOG_TRIVIAL(error) << "MCP: socket() failed"; return; }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, sock_path.c_str(), sizeof(addr.sun_path) - 1);
    // The socket is the whole application's command surface, including reading and writing
    // files on absolute paths. It lands in a world-writable directory by default, so its
    // access control is its file mode and nothing else. umask around bind() makes it 0600
    // with no window in which a wider mode exists; the chmod covers platforms that do not
    // apply umask to sockets.
    const mode_t old_umask = ::umask(0177);
    const int    bind_rc   = ::bind(sfd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    ::umask(old_umask);
    if (bind_rc < 0) {
        BOOST_LOG_TRIVIAL(error) << "MCP: bind() failed on " << sock_path;
        ::close(sfd);
        return;
    }
    if (::chmod(sock_path.c_str(), S_IRUSR | S_IWUSR) < 0) {
        BOOST_LOG_TRIVIAL(error) << "MCP: cannot restrict " << sock_path << " to the owner; refusing to listen";
        ::close(sfd);
        ::unlink(sock_path.c_str());
        return;
    }
    if (::listen(sfd, 8) < 0) { BOOST_LOG_TRIVIAL(error) << "MCP: listen() failed"; ::close(sfd); return; }
    BOOST_LOG_TRIVIAL(info) << "MCP control listening on " << sock_path;

    for (;;) {
        int cfd = ::accept(sfd, nullptr, nullptr);
        if (cfd < 0)
            continue;
#if !defined(MSG_NOSIGNAL) && defined(SO_NOSIGPIPE)
        const int on = 1;   // macOS/BSD equivalent of MSG_NOSIGNAL
        ::setsockopt(cfd, SOL_SOCKET, SO_NOSIGPIPE, &on, sizeof(on));
#endif
        // One thread per connection: a call parked behind a modal dialog must not stop a
        // second connection from inspecting and answering that dialog.
        std::thread(serve_socket_client, cfd).detach();
    }
}

std::string endpoint_from_env(const char* value, bool legacy)
{
    if (std::strcmp(value, "1") == 0)
        return legacy ? "/tmp/orca-cad-mcp.sock" : "/tmp/orca-mcp.sock";
    return value;
}

#endif

} // namespace

// ---- public ------------------------------------------------------------------------------------

void register_tool(Tool tool)
{
    if (find_tool(tool.name))
        BOOST_LOG_TRIVIAL(error) << "MCP: tool registered twice: " << tool.name;
    const std::string name = tool.name;
    registry()[name]       = std::move(tool);
}

json param(const std::string& name, const std::string& type, const std::string& description)
{
    return json{{"name", name}, {"type", type}, {"description", description}};
}

json param(const std::string& name, const std::string& type, const std::string& description, json default_value)
{
    json p       = param(name, type, description);
    p["default"] = std::move(default_value);
    return p;
}

json param_enum(const std::string& name, json values, const std::string& description, json default_value)
{
    json p    = param(name, values.empty() || values[0].is_string() ? "string" : "integer", description, std::move(default_value));
    p["enum"] = std::move(values);
    return p;
}

wxDialog* open_modal_dialog()
{
    // Newest first: with dialogs stacked, the last one opened owns the running event loop.
    for (auto* node = wxTopLevelWindows.GetLast(); node; node = node->GetPrevious())
        if (auto* dlg = dynamic_cast<wxDialog*>(node->GetData()); dlg && dlg->IsModal())
            return dlg;
    return nullptr;
}

std::string open_modal_title()
{
    wxDialog* dlg = open_modal_dialog();
    if (dlg == nullptr)
        return {};
    return dlg->GetTitle().empty() ? std::string("(untitled ") + wxString(dlg->GetClassInfo()->GetClassName()).utf8_string() + ")"
                                   : dlg->GetTitle().utf8_string();
}

void start_server_if_enabled()
{
    static bool started = false;
    if (started)
        return;
    const char* env    = std::getenv("ORCA_MCP");
    bool        legacy = false;
    if (!env || !*env) {
        env    = std::getenv("ORCA_CAD_MCP");
        legacy = true;
    }
    if (!env || !*env)
        return;
    started = true;

    register_tool({"describe_tools", "List every callable tool with its parameters.", json::array(), ModalSafe,
                   [](const json&) { return describe_tools(); }});
    register_app_tools();
    register_slicer_tools();
    register_ui_tools();

    std::thread(server_thread, endpoint_from_env(env, legacy)).detach();
}

}}} // namespace Slic3r::GUI::Mcp

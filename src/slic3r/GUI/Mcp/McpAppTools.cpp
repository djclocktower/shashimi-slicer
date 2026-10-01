// MCP tools for the application as a whole: state, tabs, mode, preferences, undo history,
// camera and screenshots. See docs/HLSD/mcp-control.md.

#include "McpUtil.hpp"

#include <GL/glew.h>
#include <wx/glcanvas.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <wx/base64.h>
#include <wx/dcmemory.h>
#include <wx/dcscreen.h>
#include <wx/dialog.h>
#include <wx/image.h>
#include <wx/mstream.h>
#include <wx/toplevel.h>

#include "libslic3r/AppConfig.hpp"
#include "libslic3r/GCode/ThumbnailData.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/libslic3r_version.h"
#include "slic3r/GUI/BackgroundSlicingProcess.hpp"
#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/Jobs/Worker.hpp"
#include "slic3r/GUI/MainFrame.hpp"
#include "slic3r/GUI/Notebook.hpp"
#include "slic3r/GUI/PartPlate.hpp"

namespace Slic3r { namespace GUI { namespace Mcp {

json app_busy_state()
{
    Plater* p = wxGetApp().plater();
    if (p == nullptr)
        return json{{"idle", false}, {"reason", "starting"}};
    const bool slicing   = p->is_background_process_slicing() || p->background_process().running();
    const bool exporting = p->is_export_gcode_scheduled();
    const bool jobs      = !p->get_ui_job_worker().is_idle();
    const bool loading   = p->is_loading_project();
    return json{{"idle", !(slicing || exporting || jobs || loading)},
                {"slicing", slicing},
                {"exporting", exporting},
                {"jobs_running", jobs},
                {"loading_project", loading}};
}

namespace {

const char* mode_name(ConfigOptionMode m)
{
    switch (m) {
    case comSimple:   return "simple";
    case comAdvanced: return "advanced";
    case comExpert:   return "expert";
    default:          return "develop";
    }
}

std::string current_tab()
{
    MainFrame* mf = wxGetApp().mainframe;
    return mf && mf->m_tabpanel ? utf8(mf->m_tabpanel->GetSelectedPageName()) : std::string();
}

json selected_presets()
{
    PresetBundle* b = wxGetApp().preset_bundle;
    if (b == nullptr)
        return json();
    return json{{"printer", b->printers.get_edited_preset().name},
                {"print", b->prints.get_edited_preset().name},
                {"filaments", b->filament_presets},
                {"physical_printer", b->physical_printers.has_selection() ? b->physical_printers.get_selected_full_printer_name() : std::string()}};
}

json app_info(const json&)
{
    Plater&        p     = plater();
    PartPlateList& plates = p.get_partplate_list();
    json info{{"app", SLIC3R_APP_NAME},
              {"version", SLIC3R_VERSION},
#ifdef _WIN32
              {"platform", "windows"},
#elif defined(__APPLE__)
              {"platform", "macos"},
#else
              {"platform", "linux"},
#endif
              {"printer_technology", p.printer_technology() == ptFFF ? "FFF" : "SLA"},
              {"tab", current_tab()},
              {"mode", mode_name(wxGetApp().get_mode())},
              {"project", {{"name", utf8(p.get_project_name())},
                           {"path", utf8(p.get_project_filename(".3mf"))},
                           {"dirty", p.is_project_dirty()},
                           {"presets_dirty", p.is_presets_dirty()}}},
              {"objects", p.model().objects.size()},
              {"plates", plates.get_plate_count()},
              {"current_plate", plates.get_curr_plate_index()},
              {"selection_empty", p.is_selection_empty()},
              {"can_undo", p.can_undo()},
              {"can_redo", p.can_redo()},
              {"presets", selected_presets()},
              {"busy", app_busy_state()},
              {"modal_dialog", open_modal_title()}};
    return info;
}

json select_tab(const json& params)
{
    const std::string tab = req<std::string>(params, "tab");
    MainFrame*        mf  = wxGetApp().mainframe;
    if (mf == nullptr || mf->m_tabpanel == nullptr)
        throw ToolError("main window not ready", -32001);
    mf->select_tab(from_utf8(tab));
    const std::string now = current_tab();
    if (now != tab)
        throw ToolError("no tab '" + tab + "' (current: '" + now + "')", -32602);
    return json{{"tab", now}};
}

json set_mode(const json& params)
{
    const std::string m = req<std::string>(params, "mode");
    if (m == "simple")        wxGetApp().set_mode(comSimple);
    else if (m == "advanced") wxGetApp().set_mode(comAdvanced);
    else if (m == "expert")   wxGetApp().set_mode(comExpert);
    else throw ToolError("mode must be simple, advanced or expert", -32602);
    return json{{"mode", mode_name(wxGetApp().get_mode())}};
}

// Preference values that are credentials are never read back over the socket.
bool is_secret_key(const std::string& key)
{
    for (const char* s : {"token", "secret", "password", "passwd", "apikey", "api_key", "cookie", "session"})
        if (boost::icontains(key, s))
            return true;
    return false;
}

json prefs_get(const json& params)
{
    AppConfig&        cfg = *wxGetApp().app_config;
    const std::string key = arg<std::string>(params, "key", "");
    if (!key.empty())
        return json{{"key", key}, {"value", is_secret_key(key) ? std::string("<redacted>") : cfg.get(key)}};
    json all = json::object();
    for (const auto& [k, v] : cfg.get_section("app"))
        all[k] = is_secret_key(k) ? std::string("<redacted>") : v;
    return all;
}

json prefs_set(const json& params)
{
    const std::string key   = req<std::string>(params, "key");
    const std::string value = req<std::string>(params, "value");
    AppConfig&        cfg   = *wxGetApp().app_config;
    cfg.set(key, value);
    cfg.save();
    wxGetApp().update_ui_from_settings();
    return json{{"key", key}, {"value", cfg.get(key)}};
}

json history(const json&)
{
    Plater& p = plater();
    auto    collect = [&p](bool undo) {
        json        out = json::array();
        const char* text = nullptr;
        for (int i = 0; i < 500 && p.undo_redo_string_getter(undo, i, &text); ++i)
            out.push_back(text ? text : "");
        return out;
    };
    return json{{"undo", collect(true)}, {"redo", collect(false)}};
}

json undo_redo(const json& params, bool undo)
{
    Plater&   p     = plater();
    const int steps = std::max(1, arg<int>(params, "steps", 1));
    int       done  = 0;
    for (; done < steps && (undo ? p.can_undo() : p.can_redo()); ++done)
        undo ? p.undo() : p.redo();
    return json{{"steps", done}, {"history", history(json())}};
}

json view_camera(const json& params)
{
    Plater&     p      = plater();
    GLCanvas3D* canvas = p.get_current_canvas3D();
    if (const std::string view = arg<std::string>(params, "view", ""); !view.empty()) {
        static const std::vector<std::string> views{"iso", "top", "bottom", "front", "rear", "left", "right", "topfront"};
        if (std::find(views.begin(), views.end(), view) == views.end())
            throw ToolError("view must be one of iso, top, bottom, front, rear, left, right, topfront", -32602);
        p.select_view(view);
    }
    if (const std::string zoom = arg<std::string>(params, "zoom", ""); !zoom.empty() && canvas) {
        if (zoom == "bed")            canvas->zoom_to_bed();
        else if (zoom == "objects")   canvas->zoom_to_volumes();
        else if (zoom == "selection") canvas->zoom_to_selection();
        else if (zoom == "gcode")     canvas->zoom_to_gcode();
        else if (zoom == "plate")     canvas->zoom_to_plate(arg<int>(params, "plate", -1));
        else throw ToolError("zoom must be bed, objects, selection, gcode or plate", -32602);
    }
    if (canvas)
        canvas->set_as_dirty();
    Camera& cam = p.get_camera();
    return json{{"position", vec3(cam.get_position())},
                {"target", vec3(cam.get_target())},
                {"zoom", cam.get_zoom()},
                {"type", cam.get_type_as_string()}};
}

// ---- screenshots ---------------------------------------------------------------------------

// Bottom-up RGBA rows (OpenGL order) -> PNG, either written to `path` or returned base64.
json encode_png(const std::vector<unsigned char>& rgba, unsigned w, unsigned h, bool bottom_up, const std::string& path)
{
    wxImage img(int(w), int(h), false);
    img.InitAlpha();
    unsigned char* rgb   = img.GetData();
    unsigned char* alpha = img.GetAlpha();
    for (unsigned y = 0; y < h; ++y) {
        const unsigned src_row = bottom_up ? h - 1 - y : y;
        for (unsigned x = 0; x < w; ++x) {
            const unsigned char* px = &rgba[4 * (size_t(src_row) * w + x)];
            const size_t         i  = size_t(y) * w + x;
            rgb[3 * i + 0] = px[0];
            rgb[3 * i + 1] = px[1];
            rgb[3 * i + 2] = px[2];
            alpha[i]       = px[3];
        }
    }
    json out{{"width", w}, {"height", h}, {"mime", "image/png"}};
    if (!path.empty()) {
        if (!img.SaveFile(from_utf8(path), wxBITMAP_TYPE_PNG))
            throw ToolError("cannot write " + path);
        out["path"] = path;
        return out;
    }
    wxMemoryOutputStream mem;
    if (!img.SaveFile(mem, wxBITMAP_TYPE_PNG))
        throw ToolError("PNG encoding failed");
    std::vector<unsigned char> bytes(mem.GetSize());
    mem.CopyTo(bytes.data(), bytes.size());
    out["png_base64"] = utf8(wxBase64Encode(bytes.data(), bytes.size()));
    return out;
}

Camera::ViewAngleType view_angle(const std::string& v)
{
    if (v == "top")      return Camera::ViewAngleType::Top;
    if (v == "bottom")   return Camera::ViewAngleType::Bottom;
    if (v == "front")    return Camera::ViewAngleType::Front;
    if (v == "rear")     return Camera::ViewAngleType::Rear;
    if (v == "left")     return Camera::ViewAngleType::Left;
    if (v == "right")    return Camera::ViewAngleType::Right;
    if (v == "topfront") return Camera::ViewAngleType::Top_Front;
    return Camera::ViewAngleType::Iso;
}

json screenshot(const json& params)
{
    const std::string source = arg<std::string>(params, "source", "viewport");
    const std::string path   = arg<std::string>(params, "path", "");
    Plater&           p      = plater();

    if (source == "render") {
        // Offscreen render of the plate's objects from a fixed view, the way thumbnails are
        // made: independent of the window, the tab shown and the camera.
        const unsigned w     = unsigned(std::clamp(arg<int>(params, "width", 800), 16, 4096));
        const unsigned h     = unsigned(std::clamp(arg<int>(params, "height", 600), 16, 4096));
        int            plate = arg<int>(params, "plate", -1);
        if (plate < 0 || plate >= p.get_partplate_list().get_plate_count())
            plate = p.get_partplate_list().get_curr_plate_index();
        ThumbnailData  data;
        ThumbnailsParams tp{{}, false, false, arg<bool>(params, "show_bed", false), false, plate};
        p.canvas3D()->render_thumbnail(data, w, h, tp, Camera::EType::Ortho, view_angle(arg<std::string>(params, "view", "iso")));
        if (!data.is_valid())
            throw ToolError("offscreen rendering failed");
        return encode_png(data.pixels, data.width, data.height, true, path);
    }

    if (source == "window") {
        // A top-level window (dialogs included) as it is on screen, by index from ui_windows.
        // Reads the screen, so it needs the window to be visible and unobscured.
        const int idx = arg<int>(params, "window", -1);
        wxWindow* win = nullptr;
        int       i   = 0;
        for (wxWindow* w : wxTopLevelWindows)
            if (i++ == idx || (idx < 0 && dynamic_cast<wxDialog*>(w) && static_cast<wxDialog*>(w)->IsModal()))
                win = w;
        if (win == nullptr)
            win = wxGetApp().mainframe;
        win->Update();
        const wxRect r = win->GetScreenRect();
        wxBitmap     bmp(r.width, r.height);
        {
            wxScreenDC screen;
            wxMemoryDC mem(bmp);
            mem.Blit(0, 0, r.width, r.height, &screen, r.x, r.y);
        }
        wxImage                    img = bmp.ConvertToImage();
        std::vector<unsigned char> rgba(size_t(r.width) * r.height * 4, 255);
        for (int k = 0; k < r.width * r.height; ++k)
            std::copy_n(img.GetData() + 3 * k, 3, rgba.data() + 4 * k);
        return encode_png(rgba, unsigned(r.width), unsigned(r.height), false, path);
    }

    if (source != "viewport")
        throw ToolError("source must be viewport, render or window", -32602);

    // The live 3D view (Prepare or Preview, whichever is shown) exactly as the user sees it,
    // toolpaths included: render a frame, then read it back from the front buffer.
    GLCanvas3D* canvas = p.get_current_canvas3D();
    if (canvas == nullptr || canvas->get_wxglcanvas() == nullptr || !canvas->get_wxglcanvas()->IsShownOnScreen())
        throw ToolError("no 3D view is shown; select the prepare or preview tab first (app_select_tab)");
    wxGLCanvas* wxc = canvas->get_wxglcanvas();
    canvas->set_as_dirty();
    canvas->render();
    wxGLContext* ctx = wxGetApp().init_glcontext(*wxc);
    if (ctx == nullptr || !wxc->SetCurrent(*ctx))
        throw ToolError("cannot make the OpenGL context current");
    const Size sz = canvas->get_canvas_size();
    const int  w  = sz.get_width(), h = sz.get_height();
    if (w <= 0 || h <= 0)
        throw ToolError("the 3D view has no size");
    std::vector<unsigned char> rgba(size_t(w) * h * 4);
    ::glPixelStorei(GL_PACK_ALIGNMENT, 1);
    ::glReadBuffer(GL_FRONT);
    ::glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
    ::glReadBuffer(GL_BACK);
    for (size_t k = 3; k < rgba.size(); k += 4)
        rgba[k] = 255;   // the default framebuffer's alpha is not meaningful
    return encode_png(rgba, unsigned(w), unsigned(h), true, path);
}

} // namespace

void register_app_tools()
{
    register_tool({"app_info", "Application state: version, tab, mode, project, counts, selected presets, busy state, open dialog.",
                   json::array(), ModalSafe, app_info});
    register_tool({"app_select_tab", "Switch the main window tab.",
                   json::array({required(param_enum("tab", json::array({"home", "prepare", "preview", "monitor", "multi_device", "project", "calibration", "sketch", "modeling", "cam", "laser"}), "tab id", json()))}),
                   0, select_tab});
    register_tool({"app_set_mode", "Set the settings complexity mode (which options the tabs show).",
                   json::array({required(param_enum("mode", json::array({"simple", "advanced", "expert"}), "", json()))}),
                   0, set_mode});
    register_tool({"app_preferences_get", "Read application preferences (Preferences dialog / app config). Credentials are redacted.",
                   json::array({param("key", "string", "one key; omit for all", "")}), ModalSafe, prefs_get});
    register_tool({"app_preferences_set", "Set an application preference (app config key) and refresh the UI from settings.",
                   json::array({param("key", "string", "app config key, e.g. 'use_perspective_camera'"),
                                param("value", "string", "string value; booleans are \"true\"/\"false\" or \"1\"/\"0\" as the key expects")}),
                   0, prefs_set});
    register_tool({"history", "Undo and redo stacks of the Prepare view, newest first.", json::array(), ModalSafe, history});
    register_tool({"undo", "Undo the last N actions in the Prepare view.", json::array({param("steps", "integer", "", 1)}), 0,
                   [](const json& p) { return undo_redo(p, true); }});
    register_tool({"redo", "Redo N undone actions in the Prepare view.", json::array({param("steps", "integer", "", 1)}), 0,
                   [](const json& p) { return undo_redo(p, false); }});
    register_tool({"view_camera", "Point the 3D view's camera and/or zoom; returns the camera state.",
                   json::array({param_enum("view", json::array({"iso", "top", "bottom", "front", "rear", "left", "right", "topfront"}), "standard view", ""),
                                param_enum("zoom", json::array({"bed", "objects", "selection", "gcode", "plate"}), "zoom to fit", ""),
                                param("plate", "integer", "for zoom=plate", -1)}),
                   0, view_camera});
    register_tool({"screenshot",
                   "Capture an image. source=viewport: the live 3D view as shown (Prepare or Preview, toolpaths included). "
                   "source=render: offscreen render of a plate's objects from a standard view. source=window: a top-level window or dialog from the screen. "
                   "Returns PNG (base64) or writes it to `path`.",
                   json::array({param_enum("source", json::array({"viewport", "render", "window"}), "", "viewport"),
                                param("path", "string", "write the PNG here instead of returning it", ""),
                                param("width", "integer", "source=render", 800), param("height", "integer", "source=render", 600),
                                param_enum("view", json::array({"iso", "top", "bottom", "front", "rear", "left", "right", "topfront"}), "source=render", "iso"),
                                param("plate", "integer", "source=render; default current", -1),
                                param("show_bed", "boolean", "source=render", false),
                                param("window", "integer", "source=window: index from ui_windows; default the open modal dialog, else the main window", -1)}),
                   ModalSafe, screenshot});
}

}}} // namespace Slic3r::GUI::Mcp

#include "LaserPanel.hpp"
#include "LaserCanvas.hpp"
#include "LaserDialogs.hpp"

#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Utils.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/I18N.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/wxExtensions.hpp"
#include "slic3r/Utils/Serial.hpp"

#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include <nlohmann/json.hpp>

#include <wx/bmpbuttn.h>
#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/choice.h>
#include <wx/dataview.h>
#include <wx/dcbuffer.h>
#include <wx/filedlg.h>
#include <wx/fontenum.h>
#include <wx/gauge.h>
#include <wx/listctrl.h>
#include <wx/menu.h>
#include <wx/msgdlg.h>
#include <wx/notebook.h>
#include <wx/progdlg.h>
#include <wx/radiobut.h>
#include <wx/scrolwin.h>
#include <wx/settings.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/statbmp.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/textdlg.h>
#include <wx/treectrl.h>
#include <wx/wrapsizer.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <sstream>

namespace Slic3r { namespace GUI {

using namespace Slic3r::Laser;
namespace fs = boost::filesystem;

namespace {

constexpr double kPi          = 3.14159265358979323846;
constexpr size_t kUndoLevels  = 100;
constexpr size_t kUndoBytes   = 256u << 20;   // ponytail: whole-document snapshots; per-shape diffs if photos make this bite
const char*      kSimulator   = "Simulator (no hardware)";
enum { TOOL_SELECT, TOOL_NODE, TOOL_LINE, TOOL_RECT, TOOL_ELLIPSE, TOOL_POLYGON, TOOL_TEXT };

std::string app_file(const char* name) { return (fs::path(data_dir()) / name).string(); }

Transform2d about(const Vec2d& c, const Transform2d& t)
{
    Transform2d r = Transform2d::Identity();
    r.translate(c);
    r = r * t;
    r.translate(-c);
    return r;
}

double shape_rotation_deg(const LaserShape& s)
{
    const Vec2d x = s.xform.linear().col(0);
    return std::atan2(x.y(), x.x()) * 180 / kPi;
}

wxBitmap swatch_bitmap(int layer, int size = 14)
{
    wxBitmap   bmp(size, size);
    wxMemoryDC dc(bmp);
    const Rgb  c = layer_color(layer);
    dc.SetBrush(wxBrush(wxColour(c.r, c.g, c.b)));
    dc.SetPen(wxPen(wxColour(90, 90, 90)));
    dc.DrawRectangle(0, 0, size, size);
    dc.SelectObject(wxNullBitmap);
    return bmp;
}

wxString font_label(const std::string& font)
{
    if (font.empty()) return _L("(default)");
    fs::path p(font);
    return wxString::FromUTF8(p.has_extension() ? p.stem().string() : font);
}

const std::vector<wxString>& system_faces()
{
    static std::vector<wxString> faces;
    if (faces.empty()) {
        wxArrayString all = wxFontEnumerator::GetFacenames(wxFONTENCODING_SYSTEM, false);
        all.Sort();
        for (const wxString& f : all)
            if (!f.StartsWith("@")) faces.push_back(f);
    }
    return faces;
}

struct TreeIndex : wxTreeItemData {
    explicit TreeIndex(int i) : index(i) {}
    int index;
};

// Device-type label for the choice.
wxString device_label(const LaserDevice& d) { return wxString::FromUTF8(d.name); }

} // namespace

// ---- Construction ----------------------------------------------------------------------------------

LaserPanel::LaserPanel(wxWindow* parent) : wxPanel(parent, wxID_ANY), m_props_timer(this)
{
    SetBackgroundColour(wxColour(246, 246, 246));
    load_prefs();

    auto* root = new wxBoxSizer(wxVERTICAL);
    build_toolbar(this, root);
    auto* mid = new wxBoxSizer(wxHORIZONTAL);
    build_toolstrip(this, mid);
    auto* centre = new wxBoxSizer(wxVERTICAL);
    m_hint = new wxPanel(this);
    m_hint->SetBackgroundColour(wxColour(255, 246, 214));
    m_hint_text = new wxStaticText(m_hint, wxID_ANY, "");
    auto* hs    = new wxBoxSizer(wxHORIZONTAL);
    hs->Add(m_hint_text, 1, wxALL, 6);
    m_hint->SetSizer(hs);
    m_hint->Bind(wxEVT_SIZE, [this](wxSizeEvent& e) {
        // Re-wrap for the new width; the height may change, so lay out again (once per width).
        static int last_width = -1;
        if (e.GetSize().x != last_width) {
            last_width = e.GetSize().x;
            m_hint_text->SetLabel(m_hint_text->GetLabel());
            m_hint_text->Wrap(std::max(100, last_width - 12));
            CallAfter([this] { Layout(); });
        }
        e.Skip();
    });
    centre->Add(m_hint, 0, wxEXPAND);
    m_canvas = new LaserCanvas(this, *this);
    centre->Add(m_canvas, 1, wxEXPAND);
    build_colour_strip(this, centre);
    mid->Add(centre, 1, wxEXPAND);

    m_dock = new wxNotebook(this, wxID_ANY, wxDefaultPosition, wxSize(FromDIP(420), -1));
    m_dock->AddPage(build_cuts_page(m_dock), _L("Cuts"));
    m_dock->AddPage(build_laser_page(m_dock), _L("Laser"));
    m_dock->AddPage(build_move_page(m_dock), _L("Move"));
    m_dock->AddPage(build_console_page(m_dock), _L("Console"));
    m_dock->AddPage(build_props_page(m_dock), _L("Shape"));
    m_dock->AddPage(build_library_page(m_dock), _L("Library"));
    mid->Add(m_dock, 0, wxEXPAND | wxLEFT, 2);
    root->Add(mid, 1, wxEXPAND);

    m_status_text = new wxStaticText(this, wxID_ANY, "");
    root->Add(m_status_text, 0, wxEXPAND | wxALL, 3);
    SetSizer(root);

    GrblStreamer::Callbacks cb;
    cb.on_status     = [this](const GrblStatus& st) { CallAfter([this, st] { on_streamer_status(st); }); };
    cb.on_console    = [this](const ConsoleLine& l) { CallAfter([this, l] { on_streamer_console(l); }); };
    cb.on_progress   = [this](const JobProgress& p) { CallAfter([this, p] { on_streamer_progress(p); }); };
    cb.on_error      = [this](const std::string& t, int line) { CallAfter([this, t, line] { on_streamer_error(t, line); }); };
    cb.on_job_done   = [this](bool ok, const std::string& r) { CallAfter([this, ok, r] { on_streamer_done(ok, r); }); };
    cb.on_connection = [this](bool c, const std::string& e) { CallAfter([this, c, e] { on_streamer_connection(c, e); }); };
    m_streamer.set_callbacks(cb);

    Bind(wxEVT_TIMER, [this](wxTimerEvent&) { commit("Edit properties"); }, m_props_timer.GetId());

    m_doc.device_name = device().name;
    load_recipe_if_changed();
    if (m_undo.empty()) push_undo();
    set_tool(TOOL_SELECT);
    refresh_all();
}

LaserPanel::~LaserPanel()
{
    m_props_timer.Stop();
    m_streamer.disconnect();
}

bool LaserPanel::Show(bool show)
{
    const bool r = wxPanel::Show(show);
    if (show) {
        load_recipe_if_changed();
        if (m_canvas) m_canvas->refresh();
    }
    return r;
}

static wxBitmapButton* make_icon_button(wxWindow* parent, const std::string& icon, const wxString& tip, int px = 22)
{
    auto* b = new wxBitmapButton(parent, wxID_ANY, create_scaled_bitmap(icon, parent, px), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT | wxBORDER_NONE);
    b->SetToolTip(tip);
    b->SetBackgroundColour(parent->GetBackgroundColour());
    return b;
}

void LaserPanel::build_toolbar(wxWindow* parent, wxSizer* sizer)
{
    auto* bar = new wxPanel(parent);
    bar->SetBackgroundColour(wxColour(236, 236, 238));
    auto* row = new wxBoxSizer(wxHORIZONTAL);
    auto  btn = [&](const std::string& icon, const wxString& tip, std::function<void()> fn) {
        auto* b = make_icon_button(bar, icon, tip);
        b->Bind(wxEVT_BUTTON, [fn](wxCommandEvent&) { fn(); });
        row->Add(b, 0, wxALL | wxALIGN_CENTER_VERTICAL, 3);
    };
    auto sep = [&] { row->Add(new wxStaticText(bar, wxID_ANY, "|"), 0, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, 4); };
    btn("laser_import", _L("Import (SVG, DXF, image, LightBurn .lbrn2)  Ctrl+I"), [this] { cmd_import(); });
    btn("laser_open", _L("Open laser project (.slaser)"), [this] { cmd_open_project(); });
    btn("laser_save", _L("Save laser project (.slaser)"), [this] { cmd_save_project(); });
    sep();
    btn("laser_undo", _L("Undo  Ctrl+Z"), [this] { undo(); });
    btn("laser_redo", _L("Redo  Ctrl+Y"), [this] { redo(); });
    sep();
    btn("laser_zoom_fit", _L("Zoom to bed"), [this] { m_canvas->zoom_fit(); });
    btn("laser_zoom_in", _L("Zoom in"), [this] { m_canvas->zoom_by(1.25); });
    btn("laser_zoom_out", _L("Zoom out"), [this] { m_canvas->zoom_by(0.8); });
    sep();
    btn("laser_preview", _L("Preview the job (time estimate)"), [this] { cmd_preview(); });
    btn("laser_frame", _L("Frame: trace the job's outline with the laser off"), [this] { cmd_frame(false); });
    btn("laser_start", _L("Start the job"), [this] { cmd_start(); });
    btn("laser_pause", _L("Pause / resume"), [this] {
        if (m_streamer.progress().paused) m_streamer.resume();
        else m_streamer.pause();
    });
    btn("laser_stop", _L("Stop"), [this] { m_streamer.stop(); });
    btn("laser_home", _L("Home ($H)"), [this] { m_streamer.home(); });
    sep();
    auto field = [&](const wxString& label, wxTextCtrl*& ctrl) {
        row->Add(new wxStaticText(bar, wxID_ANY, label), 0, wxALIGN_CENTER_VERTICAL | wxLEFT, 6);
        ctrl = new wxTextCtrl(bar, wxID_ANY, "", wxDefaultPosition, wxSize(FromDIP(64), -1), wxTE_PROCESS_ENTER);
        ctrl->Bind(wxEVT_TEXT_ENTER, [this](wxCommandEvent&) { apply_selection_fields(); });
        ctrl->Bind(wxEVT_KILL_FOCUS, [this](wxFocusEvent& e) { apply_selection_fields(); e.Skip(); });
        row->Add(ctrl, 0, wxALIGN_CENTER_VERTICAL | wxLEFT, 3);
    };
    field("X", m_fx);
    field("Y", m_fy);
    field(_L("W"), m_fw);
    field(_L("H"), m_fh);
    field(_L("Rot"), m_frot);
    m_lock_aspect = new wxCheckBox(bar, wxID_ANY, _L("Lock"));
    m_lock_aspect->SetValue(true);
    m_lock_aspect->SetToolTip(_L("Keep the width/height ratio when typing a size"));
    row->Add(m_lock_aspect, 0, wxALIGN_CENTER_VERTICAL | wxLEFT, 6);
    bar->SetSizer(row);
    sizer->Add(bar, 0, wxEXPAND);
}

void LaserPanel::build_toolstrip(wxWindow* parent, wxSizer* sizer)
{
    auto* strip = new wxPanel(parent);
    strip->SetBackgroundColour(wxColour(236, 236, 238));
    auto* col = new wxBoxSizer(wxVERTICAL);
    auto  tool = [&](int id, const std::string& icon, const wxString& tip) {
        auto* b = make_icon_button(strip, icon, tip);
        b->Bind(wxEVT_BUTTON, [this, id](wxCommandEvent&) { set_tool(id); });
        col->Add(b, 0, wxALL, 2);
        m_tool_buttons.push_back(b);
    };
    auto cmd = [&](const std::string& icon, const wxString& tip, std::function<void()> fn) {
        auto* b = make_icon_button(strip, icon, tip);
        b->Bind(wxEVT_BUTTON, [fn](wxCommandEvent&) { fn(); });
        col->Add(b, 0, wxALL, 2);
        return b;
    };
    tool(TOOL_SELECT, "laser_select", _L("Select (Esc)"));
    tool(TOOL_NODE, "laser_node", _L("Edit Nodes: drag vertices, double-click an edge to add, Delete to remove"));
    tool(TOOL_LINE, "laser_line", _L("Draw Lines: click points, double-click or Enter to finish, click the first point to close"));
    tool(TOOL_RECT, "laser_rect", _L("Rectangle (Shift: square)"));
    tool(TOOL_ELLIPSE, "laser_ellipse", _L("Ellipse (Shift: circle)"));
    tool(TOOL_POLYGON, "laser_polygon", _L("Polygon (sides in Shape Properties)"));
    tool(TOOL_TEXT, "laser_text", _L("Text: click to place"));
    col->AddSpacer(6);
    cmd("laser_offset", _L("Offset Shapes"), [this] { cmd_offset(); });
    cmd("laser_union", _L("Boolean Union (2 shapes)"), [this] { cmd_boolean(BooleanOp::Union); });
    cmd("laser_subtract", _L("Boolean Subtract: first selected minus second"), [this] { cmd_boolean(BooleanOp::Subtract); });
    cmd("laser_intersect", _L("Boolean Intersect (2 shapes)"), [this] { cmd_boolean(BooleanOp::Intersect); });
    cmd("laser_weld", _L("Weld: merge all selected shapes into one outline"), [this] { cmd_weld(); });
    cmd("laser_array", _L("Grid Array"), [this] { cmd_array(); });
    cmd("laser_circarray", _L("Circular Array"), [this] { cmd_circular_array(); });
    wxBitmapButton* align = cmd("laser_align", _L("Align"), [] {});
    align->Bind(wxEVT_BUTTON, [this, align](wxCommandEvent&) {
        wxMenu menu;
        const wxString names[6] = {_L("Align left"), _L("Align centres horizontally"), _L("Align right"),
                                   _L("Align top"), _L("Align centres vertically"), _L("Align bottom")};
        for (int i = 0; i < 6; ++i) {
            wxMenuItem* it = menu.Append(wxID_ANY, names[i]);
            menu.Bind(wxEVT_MENU, [this, i](wxCommandEvent&) { cmd_align(i); }, it->GetId());
        }
        align->PopupMenu(&menu, wxPoint(align->GetSize().x, 0));
    });
    cmd("laser_group", _L("Group  Ctrl+G"), [this] { cmd_group(); });
    cmd("laser_ungroup", _L("Ungroup  Ctrl+U"), [this] { cmd_ungroup(); });
    cmd("laser_mirror_h", _L("Mirror horizontally"), [this] { cmd_mirror(true); });
    cmd("laser_mirror_v", _L("Mirror vertically"), [this] { cmd_mirror(false); });
    cmd("laser_rotate90", _L("Rotate 90° clockwise"), [this] { cmd_rotate90(); });
    cmd("laser_trace", _L("Trace Image: turn a picture into cuttable outlines"), [this] { cmd_trace(); });
    strip->SetSizer(col);
    sizer->Add(strip, 0, wxEXPAND);
}

void LaserPanel::build_colour_strip(wxWindow* parent, wxSizer* sizer)
{
    auto* strip = new wxPanel(parent);
    strip->SetBackgroundColour(wxColour(236, 236, 238));
    auto* row = new wxBoxSizer(wxHORIZONTAL);
    for (int l = 0; l < kLayerCount; ++l) {
        auto* sw = new wxWindow(strip, wxID_ANY, wxDefaultPosition, wxSize(FromDIP(22), FromDIP(20)));
        sw->SetBackgroundStyle(wxBG_STYLE_PAINT);
        sw->SetToolTip(wxString::Format(_L("C%02d: click to put the selection on this layer; right-click for Line/Fill"), l));
        sw->Bind(wxEVT_PAINT, [this, sw, l](wxPaintEvent&) {
            wxAutoBufferedPaintDC dc(sw);
            const wxSize sz = sw->GetClientSize();
            const Rgb    c  = layer_color(l);
            dc.SetBackground(wxBrush(sw->GetParent()->GetBackgroundColour()));
            dc.Clear();
            const bool cur = l == m_current_layer;
            dc.SetPen(cur ? wxPen(wxColour(20, 20, 20), 2) : wxPen(wxColour(150, 150, 150)));
            dc.SetBrush(wxBrush(wxColour(c.r, c.g, c.b)));
            dc.DrawRectangle(cur ? 1 : 2, cur ? 1 : 2, sz.x - (cur ? 2 : 4), sz.y - (cur ? 2 : 4));
            dc.SetTextForeground((c.r * 3 + c.g * 6 + c.b) / 10 > 128 ? *wxBLACK : *wxWHITE);
            dc.SetFont(wxFont(7, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL));
            const wxString t = wxString::Format("%02d", l);
            const wxSize   te = dc.GetTextExtent(t);
            dc.DrawText(t, (sz.x - te.x) / 2, (sz.y - te.y) / 2);
        });
        sw->Bind(wxEVT_LEFT_DOWN, [this, l](wxMouseEvent&) { assign_layer(l); });
        sw->Bind(wxEVT_RIGHT_DOWN, [this, sw, l](wxMouseEvent&) {
            wxMenu menu;
            const auto names = laser_mode_names();
            for (int m = 0; m < int(names.size()); ++m) {
                wxMenuItem* it = menu.AppendRadioItem(wxID_ANY, names[m]);
                it->Check(int(m_doc.layers[l].mode) == m);
                menu.Bind(wxEVT_MENU, [this, l, m](wxCommandEvent&) {
                    m_doc.layers[l].mode = LayerMode(m);
                    commit("Layer mode");
                }, it->GetId());
            }
            menu.AppendSeparator();
            wxMenuItem* ed = menu.Append(wxID_ANY, _L("Cut Settings..."));
            menu.Bind(wxEVT_MENU, [this, l](wxCommandEvent&) { edit_layer(l); }, ed->GetId());
            sw->PopupMenu(&menu);
        });
        row->Add(sw, 0, wxALL, 1);
        m_swatches.push_back(sw);
    }
    strip->SetSizer(row);
    sizer->Add(strip, 0, wxEXPAND);
}

// ---- Right dock pages ------------------------------------------------------------------------------

wxWindow* LaserPanel::build_cuts_page(wxWindow* book)
{
    auto* page = new wxPanel(book);
    auto* col  = new wxBoxSizer(wxVERTICAL);
    m_cuts     = new wxDataViewListCtrl(page, wxID_ANY, wxDefaultPosition, wxSize(-1, FromDIP(260)), wxDV_ROW_LINES | wxDV_SINGLE);
    m_cuts->AppendIconTextColumn("#", wxDATAVIEW_CELL_INERT, FromDIP(58));
    m_cuts->AppendTextColumn(_L("Name"), wxDATAVIEW_CELL_INERT, FromDIP(62));
    m_cuts->AppendTextColumn(_L("Mode"), wxDATAVIEW_CELL_INERT, FromDIP(62));
    m_cuts->AppendTextColumn(_L("Spd/Pwr"), wxDATAVIEW_CELL_INERT, FromDIP(72));
    m_cuts->AppendToggleColumn(_L("Output"), wxDATAVIEW_CELL_ACTIVATABLE, FromDIP(46));
    m_cuts->AppendToggleColumn(_L("Show"), wxDATAVIEW_CELL_ACTIVATABLE, FromDIP(40));
    m_cuts->Bind(wxEVT_DATAVIEW_SELECTION_CHANGED, [this](wxDataViewEvent&) {
        const int row = m_cuts->GetSelectedRow();
        if (row >= 0 && row < int(m_cut_rows.size())) {
            m_current_layer = m_cut_rows[row];
            refresh_colour_strip();
        }
    });
    m_cuts->Bind(wxEVT_DATAVIEW_ITEM_ACTIVATED, [this](wxDataViewEvent& e) {
        const int row = m_cuts->ItemToRow(e.GetItem());
        if (row >= 0 && row < int(m_cut_rows.size())) edit_layer(m_cut_rows[row]);
    });
    m_cuts->Bind(wxEVT_DATAVIEW_ITEM_VALUE_CHANGED, [this](wxDataViewEvent& e) {
        const int row = m_cuts->ItemToRow(e.GetItem());
        if (row < 0 || row >= int(m_cut_rows.size())) return;
        LaserLayer& l = m_doc.layers[m_cut_rows[row]];
        l.output      = m_cuts->GetToggleValue(row, 4);
        l.visible     = m_cuts->GetToggleValue(row, 5);
        CallAfter([this] { commit("Layer output/show"); });
    });
    col->Add(m_cuts, 0, wxEXPAND | wxALL, 4);
    auto* row = new wxBoxSizer(wxHORIZONTAL);
    auto* edit = new wxButton(page, wxID_ANY, _L("Edit cut settings..."));
    edit->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { edit_layer(m_current_layer); });
    row->Add(edit, 0, wxRIGHT, 4);
    auto* opt = new wxButton(page, wxID_ANY, _L("Optimization..."));
    opt->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
        if (edit_optimize(this, m_doc.job.optimize)) commit("Optimization settings");
    });
    row->Add(opt);
    col->Add(row, 0, wxALL, 4);
    auto* tip = new wxStaticText(page, wxID_ANY, _L("Each colour is a layer with its own speed and power. Double-click a row to edit it. "
                                                    "Pick a colour in the strip under the workspace to move the selection to that layer."));
    tip->Wrap(FromDIP(340));
    tip->SetForegroundColour(wxColour(110, 110, 110));
    col->Add(tip, 0, wxALL, 6);
    page->SetSizer(col);
    return page;
}

wxWindow* LaserPanel::build_laser_page(wxWindow* book)
{
    auto* page = new wxScrolledWindow(book);
    page->SetScrollRate(0, 10);
    auto* col = new wxBoxSizer(wxVERTICAL);
    auto  hrow = [&] { return new wxBoxSizer(wxHORIZONTAL); };
    auto  wrow = [&] { return new wxWrapSizer(wxHORIZONTAL, wxREMOVE_LEADING_SPACES); };
    auto  button = [&](wxSizer* s, const wxString& label, std::function<void()> fn) {
        auto* b = new wxButton(page, wxID_ANY, label);
        b->Bind(wxEVT_BUTTON, [fn](wxCommandEvent&) { fn(); });
        s->Add(b, 0, wxRIGHT | wxBOTTOM, 3);
        return b;
    };

    // Device
    auto* r1 = hrow();
    r1->Add(new wxStaticText(page, wxID_ANY, _L("Device")), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
    m_device_choice = new wxChoice(page, wxID_ANY);
    m_device_choice->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) {
        m_device_idx      = std::max(0, m_device_choice->GetSelection());
        m_doc.device_name = device().name;
        save_prefs();
        m_canvas->zoom_fit();
        commit("Device");
    });
    r1->Add(m_device_choice, 1);
    col->Add(r1, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 6);
    auto* r1b = wrow();
    button(r1b, _L("Device settings..."), [this] {
        LaserDevice d = device();
        if (!edit_device(this, d)) return;
        m_devices[m_device_idx] = d;
        m_doc.device_name       = d.name;
        save_prefs();
        refresh_device_ui();
        m_canvas->zoom_fit();
        commit("Device settings");
    });
    button(r1b, _L("New device..."), [this] {
        LaserDevice d;
        d.name = "New laser";
        if (!edit_device(this, d)) return;
        m_devices.push_back(d);
        m_device_idx = int(m_devices.size()) - 1;
        m_doc.device_name = d.name;
        save_prefs();
        refresh_device_ui();
        m_canvas->zoom_fit();
    });
    col->Add(r1b, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 6);

    auto* r2 = hrow();
    r2->Add(new wxStaticText(page, wxID_ANY, _L("Port")), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
    m_port_choice = new wxChoice(page, wxID_ANY, wxDefaultPosition, wxSize(FromDIP(150), -1));
    r2->Add(m_port_choice, 1, wxRIGHT, 4);
    button(r2, _L("Rescan"), [this] { refresh_ports(); });
    col->Add(r2, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 6);
    r2 = hrow();
    r2->Add(new wxStaticText(page, wxID_ANY, _L("Baud")), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
    wxArrayString bauds;
    for (const char* b : {"9600", "19200", "38400", "57600", "115200", "230400", "250000", "500000", "921600"}) bauds.Add(b);
    m_baud_choice = new wxChoice(page, wxID_ANY, wxDefaultPosition, wxDefaultSize, bauds);
    r2->Add(m_baud_choice, 0);
    col->Add(r2, 0, wxEXPAND | wxLEFT | wxRIGHT, 6);

    auto* r3 = hrow();
    m_connect_btn = button(r3, _L("Connect"), [this] { m_streamer.is_connected() ? disconnect_device() : connect_device(); });
    m_unlock_btn  = button(r3, _L("Unlock ($X)"), [this] { m_streamer.unlock(); });
    m_unlock_btn->Hide();
    col->Add(r3, 0, wxALL, 6);
    m_state_text = new wxStaticText(page, wxID_ANY, "");
    m_state_text->SetFont(m_state_text->GetFont().Bold());
    m_state_text->SetLabel(_L("Not connected"));
    col->Add(m_state_text, 0, wxEXPAND | wxLEFT | wxRIGHT, 6);
    m_pos_text = new wxStaticText(page, wxID_ANY, "");
    col->Add(m_pos_text, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 6);
    m_progress = new wxGauge(page, wxID_ANY, 1000, wxDefaultPosition, wxSize(-1, FromDIP(14)));
    col->Add(m_progress, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 6);
    m_progress_text = new wxStaticText(page, wxID_ANY, "");
    col->Add(m_progress_text, 0, wxEXPAND | wxLEFT | wxRIGHT, 6);

    auto* r4 = wrow();
    button(r4, _L("Start"), [this] { cmd_start(); });
    button(r4, _L("Pause"), [this] { m_streamer.pause(); });
    button(r4, _L("Resume"), [this] { m_streamer.resume(); });
    button(r4, _L("Stop"), [this] { m_streamer.stop(); });
    col->Add(r4, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 6);
    auto* r5 = wrow();
    button(r5, _L("Frame"), [this] { cmd_frame(false); });
    button(r5, _L("Frame outline"), [this] { cmd_frame(true); });
    button(r5, _L("Home"), [this] { m_streamer.home(); });
    button(r5, _L("Go to origin"), [this] { m_streamer.send_line("G0 X0 Y0"); });
    col->Add(r5, 0, wxEXPAND | wxLEFT | wxRIGHT, 6);
    auto* r6 = wrow();
    button(r6, _L("Preview..."), [this] { cmd_preview(); });
    button(r6, _L("Save G-code..."), [this] { cmd_save_gcode(); });
    col->Add(r6, 0, wxEXPAND | wxLEFT | wxRIGHT, 6);

    // Job placement
    auto* r7 = hrow();
    r7->Add(new wxStaticText(page, wxID_ANY, _L("Start from")), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
    wxArrayString froms;
    froms.Add(_L("Absolute coords"));
    froms.Add(_L("User origin"));
    froms.Add(_L("Current position"));
    m_start_from = new wxChoice(page, wxID_ANY, wxDefaultPosition, wxDefaultSize, froms);
    m_start_from->SetToolTip(_L("Absolute: the job burns where it sits on the bed.\nUser origin: the job-origin point goes to the origin you set on the Move page.\n"
                                "Current position: the job-origin point goes to where the laser head is now."));
    m_start_from->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) {
        m_doc.job.start_from = JobSettings::StartFrom(m_start_from->GetSelection());
        refresh_device_ui();
        commit("Start from");
    });
    r7->Add(m_start_from, 1);
    col->Add(r7, 0, wxEXPAND | wxALL, 6);
    auto* r8 = hrow();
    r8->Add(new wxStaticText(page, wxID_ANY, _L("Job origin")), 0, wxRIGHT, 8);
    auto* grid = new wxGridSizer(3, 3, 0, 0);
    for (int i = 0; i < 9; ++i) {
        m_origin_radio[i] = new wxRadioButton(page, wxID_ANY, "", wxDefaultPosition, wxDefaultSize, i == 0 ? wxRB_GROUP : 0);
        m_origin_radio[i]->Bind(wxEVT_RADIOBUTTON, [this, i](wxCommandEvent&) {
            m_doc.job.job_origin = i;
            commit("Job origin");
        });
        grid->Add(m_origin_radio[i]);
    }
    r8->Add(grid);
    col->Add(r8, 0, wxLEFT | wxRIGHT, 6);
    m_cut_selected = new wxCheckBox(page, wxID_ANY, _L("Cut selected graphics"));
    m_cut_selected->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent&) { m_doc.job.cut_selected_only = m_cut_selected->GetValue(); commit("Cut selected"); });
    col->Add(m_cut_selected, 0, wxLEFT | wxTOP, 6);
    m_sel_origin = new wxCheckBox(page, wxID_ANY, _L("Use selection origin"));
    m_sel_origin->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent&) { m_doc.job.use_selection_origin = m_sel_origin->GetValue(); commit("Selection origin"); });
    col->Add(m_sel_origin, 0, wxLEFT | wxTOP, 6);
    auto* r9 = hrow();
    m_rotary_check = new wxCheckBox(page, wxID_ANY, _L("Enable rotary"));
    m_rotary_check->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent&) {
        m_devices[m_device_idx].rotary.enabled = m_rotary_check->GetValue();
        save_prefs();
    });
    r9->Add(m_rotary_check, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 6);
    button(r9, _L("Rotary setup..."), [this] {
        RotarySettings r = device().rotary;
        auto test = [this](const RotarySettings& rs) {
            // One full turn = mm_per_rotation of Y travel on the controller.
            m_streamer.jog(0, rs.mm_per_rotation, 0, std::max(5., device().travel_speed_mm_s / 4));
        };
        if (!edit_rotary(this, r, m_streamer.is_connected() ? std::function<void(const RotarySettings&)>(test) : nullptr)) return;
        m_devices[m_device_idx].rotary = r;
        save_prefs();
        refresh_device_ui();
    });
    col->Add(r9, 0, wxALL, 6);
    page->SetSizer(col);
    refresh_ports();
    return page;
}

wxWindow* LaserPanel::build_move_page(wxWindow* book)
{
    auto* page = new wxPanel(book);
    auto* col  = new wxBoxSizer(wxVERTICAL);
    auto  jog  = [this](double dx, double dy, double dz) {
        double d = 1;
        m_jog_dist->GetStringSelection().ToCDouble(&d);
        // Workspace directions -> machine axes for the device's origin corner.
        const Vec2d m = to_machine(Vec2d(dx * d, dy * d), device(), JobSettings::StartFrom::UserOrigin);
        m_streamer.jog(m.x(), m.y(), dz * d, m_jog_speed->GetValue());
    };
    auto* pad = new wxGridSizer(3, 4, 3, 3);
    auto  pb  = [&](const wxString& label, std::function<void()> fn) {
        auto* b = new wxButton(page, wxID_ANY, label, wxDefaultPosition, wxSize(FromDIP(52), FromDIP(40)));
        b->Bind(wxEVT_BUTTON, [fn](wxCommandEvent&) { fn(); });
        pad->Add(b);
    };
    pb("↖", [=] { jog(-1, 1, 0); });
    pb("↑ Y+", [=] { jog(0, 1, 0); });
    pb("↗", [=] { jog(1, 1, 0); });
    pb("Z+", [=] { jog(0, 0, 1); });
    pb("← X-", [=] { jog(-1, 0, 0); });
    pb(_L("Home"), [this] { m_streamer.home(); });
    pb("X+ →", [=] { jog(1, 0, 0); });
    pb("■", [this] { m_streamer.jog_cancel(); });
    pb("↙", [=] { jog(-1, -1, 0); });
    pb("↓ Y-", [=] { jog(0, -1, 0); });
    pb("↘", [=] { jog(1, -1, 0); });
    pb("Z-", [=] { jog(0, 0, -1); });
    col->Add(pad, 0, wxALL, 8);
    auto* r = new wxBoxSizer(wxHORIZONTAL);
    r->Add(new wxStaticText(page, wxID_ANY, _L("Distance")), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
    wxArrayString dists;
    for (const char* d : {"0.1", "1", "10", "100"}) dists.Add(d);
    m_jog_dist = new wxChoice(page, wxID_ANY, wxDefaultPosition, wxDefaultSize, dists);
    m_jog_dist->SetSelection(2);
    r->Add(m_jog_dist, 0, wxRIGHT, 4);
    r->Add(new wxStaticText(page, wxID_ANY, _L("mm   Speed")), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
    m_jog_speed = new wxSpinCtrlDouble(page, wxID_ANY, "", wxDefaultPosition, wxSize(FromDIP(80), -1), wxSP_ARROW_KEYS, 1, 500, 50, 5);
    r->Add(m_jog_speed, 0, wxRIGHT, 4);
    r->Add(new wxStaticText(page, wxID_ANY, "mm/s"), 0, wxALIGN_CENTER_VERTICAL);
    col->Add(r, 0, wxLEFT | wxRIGHT, 8);

    auto* r2 = new wxBoxSizer(wxHORIZONTAL);
    auto* so = new wxButton(page, wxID_ANY, _L("Set origin here"));
    so->SetToolTip(_L("Make the current head position the user origin (G10 L20). Used by 'Start from: User origin'."));
    so->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { m_streamer.set_origin(); });
    r2->Add(so, 0, wxRIGHT, 4);
    auto* go = new wxButton(page, wxID_ANY, _L("Go to origin"));
    go->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { m_streamer.send_line("G0 X0 Y0"); });
    r2->Add(go);
    col->Add(r2, 0, wxALL, 8);

    auto* r3 = new wxBoxSizer(wxHORIZONTAL);
    r3->Add(new wxStaticText(page, wxID_ANY, _L("Go to  X")), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
    m_goto_x = new wxTextCtrl(page, wxID_ANY, "0", wxDefaultPosition, wxSize(FromDIP(60), -1));
    r3->Add(m_goto_x, 0, wxRIGHT, 4);
    r3->Add(new wxStaticText(page, wxID_ANY, "Y"), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
    m_goto_y = new wxTextCtrl(page, wxID_ANY, "0", wxDefaultPosition, wxSize(FromDIP(60), -1));
    r3->Add(m_goto_y, 0, wxRIGHT, 4);
    auto* gob = new wxButton(page, wxID_ANY, _L("Go"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    gob->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
        double x = 0, y = 0;
        m_goto_x->GetValue().ToCDouble(&x);
        m_goto_y->GetValue().ToCDouble(&y);
        const Vec2d m = to_machine(Vec2d(x, y), device(), JobSettings::StartFrom::Absolute);
        m_streamer.send_line(wxString::Format("G0 X%.3f Y%.3f", m.x(), m.y()).ToStdString());
    });
    r3->Add(gob);
    col->Add(r3, 0, wxALL, 8);

    auto* r4 = new wxBoxSizer(wxHORIZONTAL);
    m_fire = new wxCheckBox(page, wxID_ANY, _L("Fire"));
    m_fire->SetToolTip(_L("Turns the laser on at low power while checked, to find the beam position. Wear eye protection. "
                          "GRBL in laser mode ($32=1) may only fire while moving."));
    m_fire->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent&) {
        if (m_fire->GetValue()) m_streamer.fire(m_fire_power->GetValue(), 0);
        else m_streamer.stop_fire();
    });
    r4->Add(m_fire, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 6);
    m_fire_power = new wxSpinCtrlDouble(page, wxID_ANY, "", wxDefaultPosition, wxSize(FromDIP(70), -1), wxSP_ARROW_KEYS, 0, 20, 1, 0.5);
    r4->Add(m_fire_power, 0, wxRIGHT, 4);
    r4->Add(new wxStaticText(page, wxID_ANY, _L("% power")), 0, wxALIGN_CENTER_VERTICAL);
    col->Add(r4, 0, wxALL, 8);
    m_move_pos = new wxStaticText(page, wxID_ANY, _L("Position: not connected"));
    m_move_pos->SetFont(m_move_pos->GetFont().Bold());
    col->Add(m_move_pos, 0, wxEXPAND | wxALL, 8);
    page->SetSizer(col);
    return page;
}

wxWindow* LaserPanel::build_console_page(wxWindow* book)
{
    auto* page = new wxPanel(book);
    auto* col  = new wxBoxSizer(wxVERTICAL);
    m_console  = new wxListCtrl(page, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLC_REPORT | wxLC_NO_HEADER | wxLC_SINGLE_SEL);
    m_console->AppendColumn("", wxLIST_FORMAT_LEFT, FromDIP(600));
    m_console->SetFont(wxFont(9, wxFONTFAMILY_TELETYPE, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL));
    col->Add(m_console, 1, wxEXPAND | wxALL, 4);
    auto* r   = new wxBoxSizer(wxHORIZONTAL);
    m_console_in = new wxTextCtrl(page, wxID_ANY, "", wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER);
    m_console_in->SetHint(_L("Type a command ($$, $H, G0 X10...) and press Enter"));
    m_console_in->Bind(wxEVT_TEXT_ENTER, [this](wxCommandEvent&) {
        const std::string line = m_console_in->GetValue().ToUTF8().data();
        if (line.empty()) return;
        if (!m_streamer.is_connected()) set_status(_L("Not connected: connect on the Laser page first."));
        else if (!m_streamer.send_line(line)) set_status(_L("Commands are not accepted while a job runs."));
        else m_console_in->Clear();
    });
    r->Add(m_console_in, 1, wxRIGHT, 4);
    auto* clear = new wxButton(page, wxID_ANY, _L("Clear"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    clear->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { m_console->DeleteAllItems(); m_streamer.clear_console(); });
    r->Add(clear);
    col->Add(r, 0, wxEXPAND | wxLEFT | wxRIGHT, 4);
    auto* macros = new wxBoxSizer(wxHORIZONTAL);
    for (int i = 0; i < 4; ++i) {
        m_macro_btn[i] = new wxButton(page, wxID_ANY, wxString::FromUTF8(m_macro_names[i]), wxDefaultPosition, wxSize(FromDIP(80), -1));
        m_macro_btn[i]->SetToolTip(_L("Click to send; right-click to edit"));
        m_macro_btn[i]->Bind(wxEVT_BUTTON, [this, i](wxCommandEvent&) {
            std::istringstream in(m_macros[i]);
            for (std::string l; std::getline(in, l);)
                if (!l.empty()) m_streamer.send_line(l);
        });
        m_macro_btn[i]->Bind(wxEVT_RIGHT_DOWN, [this, i](wxMouseEvent&) {
            wxTextEntryDialog name(this, _L("Button name"), _L("Edit macro"), wxString::FromUTF8(m_macro_names[i]));
            if (name.ShowModal() != wxID_OK) return;
            wxTextEntryDialog code(this, _L("G-code (one command per line)"), _L("Edit macro"), wxString::FromUTF8(m_macros[i]),
                                   wxOK | wxCANCEL | wxTE_MULTILINE);
            if (code.ShowModal() != wxID_OK) return;
            m_macro_names[i] = name.GetValue().ToUTF8().data();
            m_macros[i]      = code.GetValue().ToUTF8().data();
            m_macro_btn[i]->SetLabel(name.GetValue());
            save_prefs();
        });
        macros->Add(m_macro_btn[i], 1, wxRIGHT, 3);
    }
    col->Add(macros, 0, wxEXPAND | wxALL, 4);
    page->SetSizer(col);
    return page;
}

wxWindow* LaserPanel::build_props_page(wxWindow* book)
{
    m_props = new wxScrolledWindow(book);
    m_props->SetScrollRate(0, 10);
    m_props->SetSizer(new wxBoxSizer(wxVERTICAL));
    return m_props;
}

wxWindow* LaserPanel::build_library_page(wxWindow* book)
{
    auto* page = new wxPanel(book);
    auto* col  = new wxBoxSizer(wxVERTICAL);
    m_library  = new wxTreeCtrl(page, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTR_HAS_BUTTONS | wxTR_HIDE_ROOT | wxTR_SINGLE | wxTR_LINES_AT_ROOT);
    col->Add(m_library, 1, wxEXPAND | wxALL, 4);
    auto selected = [this]() -> int {
        const wxTreeItemId it = m_library->GetSelection();
        auto*              d  = it.IsOk() ? dynamic_cast<TreeIndex*>(m_library->GetItemData(it)) : nullptr;
        return d ? d->index : -1;
    };
    auto save = [this] {
        std::string err;
        if (!save_materials(app_file("laser_materials.json"), m_materials, &err)) set_status(wxString::FromUTF8(err));
        refresh_library();
    };
    auto* r = new wxWrapSizer(wxHORIZONTAL, wxREMOVE_LEADING_SPACES);
    auto  b = [&](const wxString& label, std::function<void()> fn) {
        auto* btn = new wxButton(page, wxID_ANY, label);
        btn->Bind(wxEVT_BUTTON, [fn](wxCommandEvent&) { fn(); });
        r->Add(btn, 0, wxRIGHT | wxBOTTOM, 3);
    };
    b(_L("Assign to layer"), [this, selected] {
        const int i = selected();
        if (i < 0) { set_status(_L("Pick a material entry (the innermost row) first.")); return; }
        LaserLayer&       l = m_doc.layers[m_current_layer];
        const LaserLayer& s = m_materials[i].settings;
        const std::string name = l.name;
        const bool        vis = l.visible, out = l.output;
        const int         prio = l.priority;
        l = s;
        l.name     = name;
        l.visible  = vis;
        l.output   = out;
        l.priority = prio;
        commit("Assign material");
        set_status(wxString::Format(_L("C%02d now uses %s %s"), m_current_layer, wxString::FromUTF8(m_materials[i].material),
                                    wxString::FromUTF8(m_materials[i].description)));
    });
    b(_L("Add"), [this, save] {
        MaterialEntry e;
        e.material    = "New material";
        e.description = "Cut";
        e.settings    = m_doc.layers[m_current_layer];
        if (edit_material(this, e)) { m_materials.push_back(e); save(); }
    });
    b(_L("Edit"), [this, selected, save] {
        const int i = selected();
        if (i >= 0 && edit_material(this, m_materials[i])) save();
    });
    b(_L("Delete"), [this, selected, save] {
        const int i = selected();
        if (i >= 0 && wxMessageBox(_L("Delete this material entry?"), _L("Library"), wxYES_NO | wxICON_QUESTION, this) == wxYES) {
            m_materials.erase(m_materials.begin() + i);
            save();
        }
    });
    col->Add(r, 0, wxEXPAND | wxALL, 4);
    auto* tip = new wxStaticText(page, wxID_ANY, _L("Starting points only: every machine differs. Test on a scrap piece first."));
    tip->Wrap(FromDIP(340));
    tip->SetForegroundColour(wxColour(110, 110, 110));
    col->Add(tip, 0, wxALL, 6);
    page->SetSizer(col);
    return page;
}

// ---- Refresh ---------------------------------------------------------------------------------------

void LaserPanel::refresh_all()
{
    refresh_cuts();
    refresh_selection_fields();
    refresh_props();
    refresh_hint();
    refresh_device_ui();
    refresh_library();
    refresh_colour_strip();
    m_canvas->document_changed();
}

void LaserPanel::refresh_cuts()
{
    std::vector<bool> used(kLayerCount, false);
    for (const LaserShape& s : m_doc.shapes)
        if (s.type != ShapeType::Group) used[std::clamp(s.layer, 0, kLayerCount - 1)] = true;
    m_cut_rows.clear();
    m_cuts->DeleteAllItems();
    const auto modes = laser_mode_names();
    for (int l = 0; l < kLayerCount; ++l) {
        if (!used[l]) continue;
        const LaserLayer& L = m_doc.layers[l];
        wxVector<wxVariant> row;
        row.push_back(wxVariant(wxDataViewIconText(wxString::Format("C%02d", l), wxBitmapBundle(swatch_bitmap(l)))));
        row.push_back(wxVariant(wxString::FromUTF8(L.name)));
        row.push_back(wxVariant(modes[std::clamp(int(L.mode), 0, int(modes.size()) - 1)]));
        row.push_back(wxVariant(wxString::Format("%g / %g%%", L.speed_mm_s, L.power_max)));
        row.push_back(wxVariant(L.output));
        row.push_back(wxVariant(L.visible));
        m_cuts->AppendItem(row);
        m_cut_rows.push_back(l);
    }
    for (size_t r = 0; r < m_cut_rows.size(); ++r)
        if (m_cut_rows[r] == m_current_layer) m_cuts->SelectRow(unsigned(r));
}

void LaserPanel::refresh_colour_strip()
{
    for (wxWindow* w : m_swatches) w->Refresh();
}

void LaserPanel::refresh_hint()
{
    const bool empty = m_doc.empty();
    if (empty)
        m_hint_text->SetLabel(_L("Start here:  1) Import an SVG, DXF or picture (first button on top) or draw with the tools on the left.  "
                                 "2) Pick a material in Library and assign it to the layer.  3) Preview.  4) Connect your laser, Frame, then Start."));
    else if (!m_streamer.is_connected())
        m_hint_text->SetLabel(_L("Next: check the layer settings in Cuts, Preview the job, then connect your laser on the Laser page."));
    m_hint_text->Wrap(std::max(100, m_hint->GetClientSize().x - 12));
    m_hint->Show(empty || !m_streamer.is_connected());
    Layout();
}

void LaserPanel::refresh_selection_fields()
{
    const bool has = !m_sel.empty();
    for (wxTextCtrl* t : {m_fx, m_fy, m_fw, m_fh, m_frot}) {
        t->Enable(has);
        if (!has) t->ChangeValue("");
    }
    if (!has) return;
    const BoundingBoxf b = m_doc.bounds(m_sel);
    if (!b.defined) return;
    m_fx->ChangeValue(wxString::Format("%.2f", b.center().x()));
    m_fy->ChangeValue(wxString::Format("%.2f", b.center().y()));
    m_fw->ChangeValue(wxString::Format("%.2f", b.size().x()));
    m_fh->ChangeValue(wxString::Format("%.2f", b.size().y()));
    m_frot->ChangeValue(m_sel.size() == 1 ? wxString::Format("%.1f", shape_rotation_deg(m_doc.shapes[m_sel[0]])) : "0");
}

void LaserPanel::apply_selection_fields()
{
    if (m_sel.empty()) return;
    const BoundingBoxf b = m_doc.bounds(m_sel);
    if (!b.defined) return;
    double x, y, w, h, rot;
    if (!m_fx->GetValue().ToCDouble(&x) || !m_fy->GetValue().ToCDouble(&y) || !m_fw->GetValue().ToCDouble(&w) ||
        !m_fh->GetValue().ToCDouble(&h) || !m_frot->GetValue().ToCDouble(&rot))
        return;
    const Vec2d c = b.center(), sz = b.size();
    // Fields show 2 decimals: an untouched field must not rescale or move anything.
    auto same = [](double typed, double exact, double tol) { return std::abs(typed - exact) < tol ? exact : typed; };
    x = same(x, c.x(), 0.006);
    y = same(y, c.y(), 0.006);
    w = same(w, sz.x(), 0.006);
    h = same(h, sz.y(), 0.006);
    double      sx = sz.x() > 1e-9 && w > 0 ? w / sz.x() : 1, sy = sz.y() > 1e-9 && h > 0 ? h / sz.y() : 1;
    if (m_lock_aspect->GetValue()) {
        // The field that changed drives both.
        if (std::abs(sx - 1) > 1e-6) sy = sx;
        else if (std::abs(sy - 1) > 1e-6) sx = sy;
    }
    const double old_rot = m_sel.size() == 1 ? shape_rotation_deg(m_doc.shapes[m_sel[0]]) : 0;
    rot                  = same(rot, old_rot, 0.06);
    Transform2d  t       = Transform2d::Identity();
    t.translate(Vec2d(x, y));
    t.rotate((rot - old_rot) * kPi / 180);
    t.scale(Vec2d(sx, sy));
    t.translate(-c);
    if (t.matrix().isApprox(Transform2d::Identity().matrix(), 1e-12)) return;
    for (int i : m_sel)
        if (!m_doc.shapes[i].locked) m_doc.transform(i, t);
    commit("Position / size");
}

void LaserPanel::refresh_device_ui()
{
    if (m_device_choice == nullptr) return;
    m_device_choice->Clear();
    for (const LaserDevice& d : m_devices) m_device_choice->Append(device_label(d));
    m_device_choice->SetSelection(m_device_idx);
    m_baud_choice->SetStringSelection(wxString::Format("%u", device().baud));
    if (m_baud_choice->GetSelection() < 0) m_baud_choice->SetSelection(4);
    m_start_from->SetSelection(int(m_doc.job.start_from));
    for (int i = 0; i < 9; ++i) {
        m_origin_radio[i]->SetValue(m_doc.job.job_origin == i);
        m_origin_radio[i]->Enable(m_doc.job.start_from != JobSettings::StartFrom::Absolute);
    }
    m_cut_selected->SetValue(m_doc.job.cut_selected_only);
    m_sel_origin->SetValue(m_doc.job.use_selection_origin);
    m_rotary_check->SetValue(device().rotary.enabled);
    const bool conn = m_streamer.is_connected();
    m_connect_btn->SetLabel(conn ? _L("Disconnect") : _L("Connect"));
    m_port_choice->Enable(!conn);
    m_baud_choice->Enable(!conn);
    m_unlock_btn->Show(conn && m_status.state == GrblState::Alarm);
    m_connect_btn->GetParent()->Layout();
}

void LaserPanel::refresh_library()
{
    if (m_library == nullptr) return;
    m_library->DeleteAllItems();
    const wxTreeItemId root = m_library->AddRoot("");
    std::map<std::string, wxTreeItemId>                 mats;
    std::map<std::pair<std::string, double>, wxTreeItemId> thick;
    for (int i = 0; i < int(m_materials.size()); ++i) {
        const MaterialEntry& e = m_materials[i];
        auto                 m = mats.find(e.material);
        if (m == mats.end()) m = mats.emplace(e.material, m_library->AppendItem(root, wxString::FromUTF8(e.material))).first;
        auto t = thick.find({e.material, e.thickness_mm});
        if (t == thick.end())
            t = thick.emplace(std::make_pair(e.material, e.thickness_mm),
                              m_library->AppendItem(m->second, e.thickness_mm > 0 ? wxString::Format("%g mm", e.thickness_mm) : _L("(any thickness)"))).first;
        const auto modes = laser_mode_names();
        m_library->AppendItem(t->second, wxString::Format("%s  -  %s, %g mm/s, %g%%%s", wxString::FromUTF8(e.description),
                                                          modes[std::clamp(int(e.settings.mode), 0, 3)], e.settings.speed_mm_s, e.settings.power_max,
                                                          e.settings.passes > 1 ? wxString::Format(", %d passes", e.settings.passes) : wxString()),
                              -1, -1, new TreeIndex(i));
    }
    m_library->ExpandAll();
}

// ---- Shape properties ------------------------------------------------------------------------------

void LaserPanel::props_changed(bool regenerate_text)
{
    if (regenerate_text)
        for (int i : m_sel) {
            LaserShape& s = m_doc.shapes[i];
            if (s.type != ShapeType::Text) continue;
            std::string err;
            if (!update_text_outlines(s, &err)) set_status(wxString::FromUTF8(err));
        }
    refresh_props_thumb();
    geometry_changed();
    m_props_timer.StartOnce(600);
}

void LaserPanel::refresh_props_thumb()
{
    if (m_props_thumb == nullptr || m_sel.size() != 1) return;
    const LaserShape& s = m_doc.shapes[m_sel[0]];
    if (s.type != ShapeType::Image || s.image_w <= 0) return;
    const int step = std::max(1, std::max(s.image_w, s.image_h) / 180);
    const int w = s.image_w / step, h = s.image_h / step;
    std::vector<uint8_t> g(size_t(w) * h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) g[size_t(y) * w + x] = s.gray[size_t(y * step) * s.image_w + x * step];
    apply_adjustments(g, w, h, s.brightness, s.contrast, s.gamma, s.invert);
    const LaserLayer& L    = m_doc.layers[std::clamp(s.layer, 0, kLayerCount - 1)];
    const double      dpi  = s.width_mm > 0 ? w * 25.4 / s.width_mm : L.image_dpi;
    std::vector<uint8_t> burn = dither(g, w, h, s.dither_override ? s.dither : L.image_dither, dpi, L.image_cells_per_inch, L.image_screen_angle_deg);
    wxImage im(w, h);
    for (size_t i = 0; i < burn.size(); ++i) im.GetData()[3 * i] = im.GetData()[3 * i + 1] = im.GetData()[3 * i + 2] = uint8_t(255 - burn[i]);
    m_props_thumb->SetBitmap(wxBitmap(im));
    m_props->Layout();
}

void LaserPanel::refresh_props()
{
    m_props_timer.Stop();
    m_props->DestroyChildren();
    m_props_thumb = nullptr;
    wxSizer* top  = m_props->GetSizer();
    top->Clear();
    LaserForm f(m_props, [this] { props_changed(false); });
    if (m_sel.empty()) {
        f.heading(_L("Drawing defaults"));
        f.integer(_L("Polygon sides"), m_canvas->polygon_sides, 3, 64);
        f.num(_L("Text height"), m_text_height, 0.5, 500, 1, 1, "mm");
        f.check(_L("Snap to grid and objects"), m_canvas->snap);
        top->Add(f.sizer(), 0, wxEXPAND | wxALL, 8);
        auto* t = new wxStaticText(m_props, wxID_ANY, _L("Select a shape to edit its properties."));
        t->SetForegroundColour(wxColour(110, 110, 110));
        top->Add(t, 0, wxALL, 8);
        m_props->FitInside();
        m_props->Layout();
        return;
    }
    if (m_sel.size() > 1) {
        top->Add(new wxStaticText(m_props, wxID_ANY, wxString::Format(_L("%zu shapes selected"), m_sel.size())), 0, wxALL, 8);
        m_props->FitInside();
        m_props->Layout();
        return;
    }
    LaserShape&   s     = m_doc.shapes[m_sel[0]];
    const wxString types[] = {_L("Path"), _L("Rectangle"), _L("Ellipse"), _L("Polygon"), _L("Text"), _L("Image"), _L("Group")};
    f.heading(types[int(s.type)] + wxString::Format("  (C%02d)", s.layer));
    f.text(_L("Name"), s.name);
    f.check(_L("Locked"), s.locked);
    switch (s.type) {
    case ShapeType::Rect:
        f.num(_L("Width"), s.width, 0.01, 5000, 1, 2, "mm");
        f.num(_L("Height"), s.height, 0.01, 5000, 1, 2, "mm");
        f.num(_L("Corner radius"), s.corner_radius, 0, 2500, 0.5, 2, "mm");
        break;
    case ShapeType::Ellipse:
        f.num(_L("Radius X"), s.rx, 0.01, 2500, 0.5, 2, "mm");
        f.num(_L("Radius Y"), s.ry, 0.01, 2500, 0.5, 2, "mm");
        break;
    case ShapeType::Polygon:
        f.integer(_L("Sides"), s.sides, 3, 64);
        f.num(_L("Radius"), s.rx, 0.01, 2500, 0.5, 2, "mm");
        break;
    case ShapeType::Text: {
        LaserForm ft(m_props, [this] { props_changed(true); });
        ft.text(_L("Text"), s.text, true);
        std::vector<wxString> faces = system_faces();
        int                   cur   = -1;
        const wxString        label = font_label(s.font);
        for (int i = 0; i < int(faces.size()); ++i)
            if (faces[i].IsSameAs(label, false)) cur = i;
        if (cur < 0) { faces.insert(faces.begin(), label); cur = 0; }
        const int idx = m_sel[0];
        ft.choice_int(_L("Font"), cur, faces, [this, idx, faces](int i) {
            // The family name: portable between machines; TextLayout finds the file (and the
            // bold / italic face) through resolve_font_path().
            m_doc.shapes[idx].font = faces[i].ToUTF8().data();
            m_text_font            = m_doc.shapes[idx].font;
        });
        ft.num(_L("Height"), s.text_height_mm, 0.5, 500, 1, 2, "mm");
        ft.num(_L("Letter spacing"), s.text_spacing_mm, -50, 50, 0.2, 2, "mm");
        ft.check(_L("Bold"), s.bold);
        ft.check(_L("Italic"), s.italic);
        ft.choice(_L("Align"), s.text_align, {_L("Left"), _L("Center"), _L("Right")});
        top->Add(f.sizer(), 0, wxEXPAND | wxALL, 8);
        top->Add(ft.sizer(), 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
        m_props->FitInside();
        m_props->Layout();
        return;
    }
    case ShapeType::Image: {
        LaserLayer& L = m_doc.layers[std::clamp(s.layer, 0, kLayerCount - 1)];
        f.num(_L("Brightness"), s.brightness, -100, 100, 5, 0);
        f.num(_L("Contrast"), s.contrast, -100, 100, 5, 0);
        f.num(_L("Gamma"), s.gamma, 0.1, 5, 0.05, 2);
        f.check(_L("Invert"), s.invert);
        f.check(_L("Override the layer's image mode"), s.dither_override);
        f.choice(_L("Image mode"), s.dither, dither_names());
        f.num(_L("Layer DPI"), L.image_dpi, 10, 1500, 10, 0, "", _L("Shared by every image on this layer (Cut Settings)."));
        f.row(_L("Pixels"), new wxStaticText(m_props, wxID_ANY, wxString::Format("%d x %d, %.1f x %.1f mm", s.image_w, s.image_h, s.width_mm, s.height_mm)));
        top->Add(f.sizer(), 0, wxEXPAND | wxALL, 8);
        m_props_thumb = new wxStaticBitmap(m_props, wxID_ANY, wxNullBitmap);
        top->Add(m_props_thumb, 0, wxALL, 8);
        refresh_props_thumb();
        m_props->FitInside();
        m_props->Layout();
        return;
    }
    default: break;
    }
    top->Add(f.sizer(), 0, wxEXPAND | wxALL, 8);
    m_props->FitInside();
    m_props->Layout();
}

// ---- Document plumbing -----------------------------------------------------------------------------

void LaserPanel::set_selection(std::vector<int> sel)
{
    sel.erase(std::remove_if(sel.begin(), sel.end(), [this](int i) { return i < 0 || i >= int(m_doc.shapes.size()); }), sel.end());
    if (sel == m_sel) return;
    if (m_props_timer.IsRunning()) commit("Edit properties");
    m_sel = std::move(sel);
    if (m_sel.size() == 1 && m_doc.shapes[m_sel[0]].type != ShapeType::Group) {
        m_current_layer = std::clamp(m_doc.shapes[m_sel[0]].layer, 0, kLayerCount - 1);
        refresh_colour_strip();
    }
    refresh_selection_fields();
    refresh_props();
    m_canvas->refresh();
}

void LaserPanel::geometry_changed()
{
    m_canvas->document_changed();
    refresh_selection_fields();
}

void LaserPanel::commit(const std::string& what)
{
    m_props_timer.Stop();
    push_undo();
    sync_recipe();
    m_canvas->document_changed();
    refresh_cuts();
    refresh_selection_fields();
    refresh_hint();
    refresh_colour_strip();
    if (what != "Edit properties") refresh_props();
    if (!what.empty()) set_status(wxString::FromUTF8(what));
}

void LaserPanel::add_shape_and_select(LaserShape shape, const std::string& what)
{
    const int i = m_doc.add_shape(std::move(shape));
    m_sel       = {i};
    if (!what.empty()) commit(what);
    else m_canvas->document_changed();
    refresh_selection_fields();
    refresh_props();
}

void LaserPanel::set_status(const wxString& text)
{
    if (m_status_text) m_status_text->SetLabel(text);
}

void LaserPanel::tool_finished() { set_tool(TOOL_SELECT); }

void LaserPanel::set_tool(int tool)
{
    for (int i = 0; i < int(m_tool_buttons.size()); ++i)
        m_tool_buttons[i]->SetBackgroundColour(i == tool ? wxColour(190, 215, 250) : m_tool_buttons[i]->GetParent()->GetBackgroundColour());
    m_canvas->set_tool(LaserTool(tool));
}

bool LaserPanel::head_position(Vec2d& pos) const
{
    if (!m_streamer.is_connected() || m_status.state == GrblState::Disconnected || m_status.state == GrblState::Unknown) return false;
    pos = from_machine(Vec2d(m_status.mpos.x(), m_status.mpos.y()), device(), JobSettings::StartFrom::Absolute);
    return true;
}

void LaserPanel::sync_recipe()
{
    Plater* plater = wxGetApp().plater();
    if (plater == nullptr) return;
    std::string blob = m_doc.empty() ? std::string() : m_doc.serialize();
    m_synced_blob    = blob;
    if (blob == plater->model().laser_recipe) return;
    plater->model().laser_recipe = std::move(blob);
    Slic3r::put_other_changes();
    plater->set_plater_dirty(true);   // Save Project enabled and the title marked, like any other edit
}

void LaserPanel::load_recipe_if_changed()
{
    Plater* plater = wxGetApp().plater();
    if (plater == nullptr || plater->model().laser_recipe == m_synced_blob) return;
    m_synced_blob = plater->model().laser_recipe;
    if (m_synced_blob.empty()) m_doc.clear();
    else if (!m_doc.deserialize(m_synced_blob)) {
        m_doc.clear();
        wxMessageBox(_L("The laser part of this project could not be read."), _L("Laser"), wxOK | wxICON_WARNING, this);
    }
    for (const std::string& w : m_doc.warnings) set_status(wxString::FromUTF8(w));
    for (int i = 0; i < int(m_devices.size()); ++i)
        if (!m_doc.device_name.empty() && m_devices[i].name == m_doc.device_name) m_device_idx = i;
    m_sel.clear();
    m_undo.clear();
    m_undo_pos = -1;
    push_undo();
    if (m_canvas) refresh_all();
}

void LaserPanel::push_undo()
{
    std::string blob = m_doc.serialize();
    if (m_undo_pos >= 0 && m_undo_pos < int(m_undo.size()) && m_undo[m_undo_pos] == blob) return;
    m_undo.resize(size_t(m_undo_pos + 1));
    m_undo.push_back(std::move(blob));
    size_t bytes = 0;
    for (const std::string& s : m_undo) bytes += s.size();
    while (m_undo.size() > 1 && (m_undo.size() > kUndoLevels || bytes > kUndoBytes)) {
        bytes -= m_undo.front().size();
        m_undo.erase(m_undo.begin());
    }
    m_undo_pos = int(m_undo.size()) - 1;
}

void LaserPanel::restore(const std::string& blob)
{
    m_doc.deserialize(blob);
    m_sel.clear();
    sync_recipe();
    refresh_all();
}

void LaserPanel::undo()
{
    if (m_undo_pos <= 0) { set_status(_L("Nothing to undo")); return; }
    restore(m_undo[--m_undo_pos]);
    set_status(_L("Undo"));
}

void LaserPanel::redo()
{
    if (m_undo_pos + 1 >= int(m_undo.size())) { set_status(_L("Nothing to redo")); return; }
    restore(m_undo[++m_undo_pos]);
    set_status(_L("Redo"));
}

// ---- Prefs -----------------------------------------------------------------------------------------

void LaserPanel::load_prefs()
{
    std::string err;
    if (!load_devices(app_file("laser_devices.json"), m_devices, &err) || m_devices.empty()) m_devices = default_devices();
    if (m_devices.empty()) m_devices.push_back(LaserDevice{});
    if (!load_materials(app_file("laser_materials.json"), m_materials, &err))
        m_materials = default_materials(m_devices.front().source);
    m_macro_names = {"Laser test", "Unlock", "Settings", "Laser off"};
    m_macros      = {"M3 S10\nG4 P0.5\nM5", "$X", "$$", "M5"};
    std::string device_name;
    try {
        boost::nowide::ifstream in(app_file("laser_ui.json"));
        if (in) {
            nlohmann::json j;
            in >> j;
            device_name = j.value("device", "");
            m_last_port = j.value("port", "");
            m_text_font = j.value("text_font", "");
            if (j.contains("macros"))
                for (size_t i = 0; i < 4 && i < j["macros"].size(); ++i) {
                    m_macro_names[i] = j["macros"][i].value("name", m_macro_names[i]);
                    m_macros[i]      = j["macros"][i].value("gcode", m_macros[i]);
                }
        }
    } catch (const std::exception&) {
        // Unreadable prefs: defaults.
    }
    for (int i = 0; i < int(m_devices.size()); ++i)
        if (m_devices[i].name == device_name) m_device_idx = i;
    if (m_text_font.empty()) m_text_font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT).GetFaceName().ToUTF8().data();
}

void LaserPanel::save_prefs()
{
    std::string err;
    if (!save_devices(app_file("laser_devices.json"), m_devices, &err)) set_status(wxString::FromUTF8(err));
    try {
        nlohmann::json j;
        j["device"]    = device().name;
        j["port"]      = m_last_port;
        j["text_font"] = m_text_font;
        for (int i = 0; i < 4; ++i) j["macros"].push_back({{"name", m_macro_names[i]}, {"gcode", m_macros[i]}});
        boost::nowide::ofstream out(app_file("laser_ui.json"));
        out << j.dump(2);
    } catch (const std::exception&) {
    }
}

// ---- Commands on the selection ---------------------------------------------------------------------

namespace {
// Roots + every descendant.
std::vector<int> with_members(const LaserDocument& doc, const std::vector<int>& roots)
{
    std::vector<int> out, stack(roots.begin(), roots.end());
    while (!stack.empty()) {
        const int i = stack.back();
        stack.pop_back();
        out.push_back(i);
        for (int c : doc.children(i)) stack.push_back(c);
    }
    return out;
}
} // namespace

std::vector<int> LaserPanel::job_selection() const { return with_members(m_doc, m_sel); }

void LaserPanel::cmd_select_all()
{
    std::vector<int> all;
    for (int i = 0; i < int(m_doc.shapes.size()); ++i)
        if (m_doc.shapes[i].parent < 0) all.push_back(i);
    set_selection(all);
}

void LaserPanel::cmd_delete()
{
    if (m_sel.empty()) return;
    m_doc.remove_shapes(m_sel);
    m_sel.clear();
    commit("Delete");
    refresh_props();
}

void LaserPanel::cmd_copy(bool cut)
{
    if (m_sel.empty()) return;
    m_clipboard.clear();
    std::map<int, int> remap;
    for (int i : with_members(m_doc, m_sel)) {
        remap[i]       = int(m_clipboard.size());
        LaserShape s   = m_doc.shapes[i];
        m_clipboard.push_back(std::move(s));
    }
    for (LaserShape& s : m_clipboard) s.parent = remap.count(s.parent) ? remap[s.parent] : -1;
    if (cut) cmd_delete();
    else set_status(wxString::Format(_L("Copied %zu shapes"), m_sel.size()));
}

void LaserPanel::cmd_paste()
{
    if (m_clipboard.empty()) return;
    const std::vector<int> added = m_doc.add_shapes(m_clipboard);
    std::vector<int>       roots;
    for (int i : added)
        if (m_doc.shapes[i].parent < 0) roots.push_back(i);
    Transform2d t = Transform2d::Identity();
    t.translate(Vec2d(5, -5));
    for (int i : roots) m_doc.transform(i, t);
    m_sel = roots;
    commit("Paste");
    refresh_props();
}

void LaserPanel::cmd_duplicate()
{
    if (m_sel.empty()) return;
    m_sel = m_doc.duplicate(m_sel);
    commit("Duplicate");
    refresh_props();
}

void LaserPanel::cmd_group()
{
    if (m_sel.size() < 2) { set_status(_L("Select two or more shapes to group.")); return; }
    const int g = m_doc.group(m_sel);
    if (g >= 0) m_sel = {g};
    commit("Group");
}

void LaserPanel::cmd_ungroup()
{
    std::vector<uint64_t> groups, members;
    for (int i : m_sel)
        if (m_doc.shapes[i].type == ShapeType::Group) groups.push_back(m_doc.shapes[i].id);
    if (groups.empty()) return;
    for (uint64_t id : groups) {
        const int g = m_doc.find(id);
        if (g < 0) continue;
        for (int m : m_doc.ungroup(g)) members.push_back(m_doc.shapes[m].id);
    }
    m_sel.clear();
    for (uint64_t id : members)
        if (int i = m_doc.find(id); i >= 0) m_sel.push_back(i);
    commit("Ungroup");
}

void LaserPanel::cmd_lock(bool lock)
{
    for (int i : with_members(m_doc, m_sel)) m_doc.shapes[i].locked = lock;
    commit(lock ? "Lock" : "Unlock");
}

void LaserPanel::cmd_to_layer(int layer)
{
    for (int i : with_members(m_doc, m_sel))
        if (m_doc.shapes[i].type != ShapeType::Group) m_doc.shapes[i].layer = layer;
    m_current_layer = layer;
    commit(wxString::Format("Move to C%02d", layer).ToStdString());
}

void LaserPanel::assign_layer(int layer)
{
    m_current_layer = layer;
    refresh_colour_strip();
    if (!m_sel.empty()) cmd_to_layer(layer);
    else refresh_cuts();
}

void LaserPanel::cmd_to_path()
{
    for (int i : m_sel) {
        LaserShape& s = m_doc.shapes[i];
        if (s.type == ShapeType::Group || s.type == ShapeType::Image || s.type == ShapeType::Path) continue;
        LaserPaths p = m_doc.flatten(i, 0.05);   // coarser than the burn tolerance: fewer nodes to edit
        s.type  = ShapeType::Path;
        s.paths = std::move(p);
        s.xform = Transform2d::Identity();
        s.text_outlines.clear();
    }
    commit("Convert to path");
}

void LaserPanel::nudge(double dx, double dy)
{
    if (m_sel.empty()) return;
    Transform2d t = Transform2d::Identity();
    t.translate(Vec2d(dx, dy));
    for (int i : m_sel)
        if (!m_doc.shapes[i].locked) m_doc.transform(i, t);
    commit("Nudge");
}

void LaserPanel::edit_text(int index)
{
    if (index < 0 || index >= int(m_doc.shapes.size()) || m_doc.shapes[index].type != ShapeType::Text) return;
    LaserShape&       s   = m_doc.shapes[index];
    const bool        brand_new = s.text.empty();
    wxTextEntryDialog dlg(this, _L("Text (Enter for a new line in the box; OK when done)"), _L("Edit Text"), wxString::FromUTF8(s.text),
                          wxOK | wxCANCEL | wxTE_MULTILINE);
    if (dlg.ShowModal() != wxID_OK || dlg.GetValue().IsEmpty()) {
        if (brand_new) {
            m_doc.remove_shapes({index});
            m_sel.clear();
            m_canvas->document_changed();
            refresh_props();
        }
        return;
    }
    LaserShape& t = m_doc.shapes[index];
    t.text        = dlg.GetValue().ToUTF8().data();
    if (t.font.empty()) t.font = m_text_font;
    if (brand_new) t.text_height_mm = m_text_height;
    std::string err;
    if (!update_text_outlines(t, &err))
        wxMessageBox(_L("The text could not be drawn with this font:") + "\n" + wxString::FromUTF8(err), _L("Text"), wxOK | wxICON_WARNING, this);
    m_sel = {index};
    commit(brand_new ? "Add text" : "Edit text");
    refresh_props();
}

void LaserPanel::cmd_offset()
{
    if (m_sel.empty()) { set_status(_L("Select shapes to offset.")); return; }
    static OffsetParams p;
    if (!ask_offset(this, p)) return;
    LaserPaths src;
    for (int i : m_sel) {
        LaserPaths f = m_doc.flatten(i);
        src.insert(src.end(), f.begin(), f.end());
    }
    LaserShape s;
    s.type  = ShapeType::Path;
    s.layer = m_doc.shapes[m_sel[0]].type == ShapeType::Group ? m_current_layer : m_doc.shapes[m_sel[0]].layer;
    s.paths = offset_paths(src, p.distance, p.dir, p.corners);
    if (s.paths.empty()) { set_status(_L("The offset produced nothing (open paths cannot be offset, or the inward offset is too large).")); return; }
    if (p.delete_original) m_doc.remove_shapes(m_sel);
    m_sel = {m_doc.add_shape(std::move(s))};
    commit("Offset");
    refresh_props();
}

void LaserPanel::cmd_boolean(BooleanOp op)
{
    if (m_sel.size() != 2) { set_status(_L("Boolean needs exactly two selected closed shapes.")); return; }
    LaserShape s;
    s.type  = ShapeType::Path;
    s.layer = m_doc.shapes[m_sel[0]].type == ShapeType::Group ? m_current_layer : m_doc.shapes[m_sel[0]].layer;
    s.paths = boolean_op(m_doc.flatten(m_sel[0]), m_doc.flatten(m_sel[1]), op);
    if (s.paths.empty()) { set_status(_L("The result is empty: do the shapes overlap, and are they closed?")); return; }
    m_doc.remove_shapes(m_sel);
    m_sel = {m_doc.add_shape(std::move(s))};
    commit(op == BooleanOp::Union ? "Union" : op == BooleanOp::Subtract ? "Subtract" : "Intersect");
    refresh_props();
}

void LaserPanel::cmd_weld()
{
    if (m_sel.empty()) return;
    std::vector<LaserPaths> shapes;
    for (int i : m_sel) shapes.push_back(m_doc.flatten(i));
    LaserShape s;
    s.type  = ShapeType::Path;
    s.layer = m_doc.shapes[m_sel[0]].type == ShapeType::Group ? m_current_layer : m_doc.shapes[m_sel[0]].layer;
    s.paths = weld(shapes);
    if (s.paths.empty()) { set_status(_L("Weld works on closed shapes.")); return; }
    m_doc.remove_shapes(m_sel);
    m_sel = {m_doc.add_shape(std::move(s))};
    commit("Weld");
    refresh_props();
}

void LaserPanel::cmd_array()
{
    if (m_sel.empty()) { set_status(_L("Select shapes to array.")); return; }
    static ArrayParams p;
    if (!ask_array(this, p)) return;
    const BoundingBoxf b = m_doc.bounds(m_sel);
    if (!b.defined) return;
    const std::vector<int> orig = m_sel;
    std::vector<int>       all  = orig;
    for (int r = 0; r < p.rows; ++r)
        for (int c = 0; c < p.columns; ++c) {
            if (r == 0 && c == 0) continue;
            Transform2d t = Transform2d::Identity();
            t.translate(Vec2d(c * (b.size().x() + p.dx), -r * (b.size().y() + p.dy)));
            for (int i : m_doc.duplicate(orig)) {
                m_doc.transform(i, t);
                all.push_back(i);
            }
        }
    m_sel = all;
    commit("Array");
}

void LaserPanel::cmd_circular_array()
{
    if (m_sel.empty()) { set_status(_L("Select shapes to array.")); return; }
    static CircularArrayParams p;
    const BoundingBoxf         b = m_doc.bounds(m_sel);
    if (!b.defined) return;
    p.cx = b.center().x();
    p.cy = b.center().y() - std::max(b.size().x(), b.size().y());
    if (!ask_circular_array(this, p)) return;
    const Vec2d            c(p.cx, p.cy);
    const std::vector<int> orig = m_sel;
    std::vector<int>       all  = orig;
    const bool             full = std::abs(std::abs(p.span_deg) - 360) < 1e-6;
    for (int k = 1; k < p.count; ++k) {
        const double a = (p.start_deg + p.span_deg * k / (full ? p.count : std::max(1, p.count - 1))) * kPi / 180;
        Transform2d  t = Transform2d::Identity();
        if (p.rotate_copies) {
            Transform2d r = Transform2d::Identity();
            r.rotate(a);
            t = about(c, r);
        } else {
            const Vec2d d  = b.center() - c;
            const Vec2d rd = Eigen::Rotation2Dd(a) * d;
            t.translate(rd - d);
        }
        for (int i : m_doc.duplicate(orig)) {
            m_doc.transform(i, t);
            all.push_back(i);
        }
    }
    m_sel = all;
    commit("Circular array");
}

void LaserPanel::cmd_align(int how)
{
    if (m_sel.size() < 2) { set_status(_L("Select two or more shapes to align.")); return; }
    const BoundingBoxf all = m_doc.bounds(m_sel);
    for (int i : m_sel) {
        const BoundingBoxf b = m_doc.bounds(std::vector<int>{i});
        if (!b.defined || m_doc.shapes[i].locked) continue;
        Vec2d d(0, 0);
        switch (how) {
        case 0: d.x() = all.min.x() - b.min.x(); break;
        case 1: d.x() = all.center().x() - b.center().x(); break;
        case 2: d.x() = all.max.x() - b.max.x(); break;
        case 3: d.y() = all.max.y() - b.max.y(); break;
        case 4: d.y() = all.center().y() - b.center().y(); break;
        case 5: d.y() = all.min.y() - b.min.y(); break;
        }
        Transform2d t = Transform2d::Identity();
        t.translate(d);
        m_doc.transform(i, t);
    }
    commit("Align");
}

void LaserPanel::cmd_mirror(bool horizontal)
{
    const BoundingBoxf b = m_doc.bounds(m_sel);
    if (!b.defined) return;
    Transform2d s = Transform2d::Identity();
    s.scale(horizontal ? Vec2d(-1, 1) : Vec2d(1, -1));
    const Transform2d t = about(b.center(), s);
    for (int i : m_sel)
        if (!m_doc.shapes[i].locked) m_doc.transform(i, t);
    commit(horizontal ? "Mirror horizontally" : "Mirror vertically");
}

void LaserPanel::cmd_rotate90()
{
    const BoundingBoxf b = m_doc.bounds(m_sel);
    if (!b.defined) return;
    Transform2d r = Transform2d::Identity();
    r.rotate(-kPi / 2);
    const Transform2d t = about(b.center(), r);
    for (int i : m_sel)
        if (!m_doc.shapes[i].locked) m_doc.transform(i, t);
    commit("Rotate 90");
}

void LaserPanel::cmd_trace()
{
    if (m_sel.size() != 1 || m_doc.shapes[m_sel[0]].type != ShapeType::Image) {
        set_status(_L("Select one image to trace."));
        return;
    }
    static TraceOptions opts;
    if (!ask_trace(this, m_doc.shapes[m_sel[0]], opts)) return;
    wxBusyCursor busy;
    LaserShape   s = trace_image(m_doc.shapes[m_sel[0]], opts);
    if (s.paths.empty()) { set_status(_L("Nothing was traced: try a higher threshold.")); return; }
    s.name = "Trace";
    m_sel  = {m_doc.add_shape(std::move(s))};
    commit("Trace image");
    refresh_props();
}

// ---- Files -----------------------------------------------------------------------------------------

void LaserPanel::cmd_import()
{
    wxFileDialog dlg(this, _L("Import into the laser workspace"), "", "",
                     _L("Laser files") + " (*.svg;*.dxf;*.png;*.jpg;*.jpeg;*.bmp;*.tif;*.tiff;*.lbrn2;*.lbrn)|*.svg;*.dxf;*.png;*.jpg;*.jpeg;*.bmp;*.tif;*.tiff;*.lbrn2;*.lbrn;*.SVG;*.DXF;*.PNG;*.JPG|" +
                         _L("All files") + " (*.*)|*.*",
                     wxFD_OPEN | wxFD_FILE_MUST_EXIST | wxFD_MULTIPLE);
    if (dlg.ShowModal() != wxID_OK) return;
    wxArrayString paths;
    dlg.GetPaths(paths);
    for (const wxString& p : paths) import_file(p.ToUTF8().data());
}

void LaserPanel::import_file(const std::string& path)
{
    wxBusyCursor busy;
    ImportResult r = Laser::import_file(path);
    if (!r.ok()) {
        wxMessageBox(wxString::FromUTF8(r.error), _L("Import"), wxOK | wxICON_WARNING, this);
        return;
    }
    if (r.shapes.empty()) {
        wxMessageBox(_L("Nothing to import was found in this file."), _L("Import"), wxOK | wxICON_INFORMATION, this);
        return;
    }
    for (const auto& [layer, settings] : r.cut_settings)
        if (layer >= 0 && layer < kLayerCount) m_doc.layers[layer] = settings;
    const std::vector<int> added = m_doc.add_shapes(std::move(r.shapes));
    std::vector<int>       roots;
    for (int i : added)
        if (m_doc.shapes[i].parent < 0) roots.push_back(i);
    // Centre on the bed (files rarely share the bed's origin).
    const BoundingBoxf b = m_doc.bounds(roots);
    if (b.defined) {
        Transform2d t = Transform2d::Identity();
        t.translate(Vec2d(device().bed_w / 2, device().bed_h / 2) - b.center());
        for (int i : roots) m_doc.transform(i, t);
    }
    m_sel = roots;
    commit("Import " + fs::path(path).filename().string());
    refresh_props();
    if (!r.warnings.empty()) {
        wxString w;
        for (size_t i = 0; i < r.warnings.size() && i < 8; ++i) w += "• " + wxString::FromUTF8(r.warnings[i]) + "\n";
        set_status(wxString::FromUTF8(r.warnings.front()));
        wxMessageBox(_L("Imported with notes:") + "\n\n" + w, _L("Import"), wxOK | wxICON_INFORMATION, this);
    }
}

void LaserPanel::cmd_save_project()
{
    wxFileDialog dlg(this, _L("Save laser project"), "", "laser.slaser", _L("Shashimi laser project") + " (*.slaser)|*.slaser",
                     wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (dlg.ShowModal() != wxID_OK) return;
    boost::nowide::ofstream f(dlg.GetPath().ToUTF8().data(), std::ios::binary);
    f << m_doc.serialize();
    set_status(f ? _L("Laser project saved") : _L("Could not write the file"));
}

void LaserPanel::cmd_open_project()
{
    wxFileDialog dlg(this, _L("Open laser project"), "", "", _L("Shashimi laser project") + " (*.slaser)|*.slaser", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dlg.ShowModal() != wxID_OK) return;
    if (!m_doc.empty() &&
        wxMessageBox(_L("Replace the current laser workspace?"), _L("Open laser project"), wxYES_NO | wxICON_QUESTION, this) != wxYES)
        return;
    boost::nowide::ifstream f(dlg.GetPath().ToUTF8().data(), std::ios::binary);
    std::stringstream       ss;
    ss << f.rdbuf();
    LaserDocument d;
    if (!d.deserialize(ss.str())) {
        wxMessageBox(_L("This file is not a readable laser project."), _L("Open laser project"), wxOK | wxICON_WARNING, this);
        return;
    }
    m_doc = std::move(d);
    m_sel.clear();
    commit("Open laser project");
    refresh_all();
}

// ---- Jobs ------------------------------------------------------------------------------------------

bool LaserPanel::plan_job(LaserJob& job, const wxString& title)
{
    if (m_doc.empty()) {
        wxMessageBox(_L("The workspace is empty: import or draw something first."), title, wxOK | wxICON_INFORMATION, this);
        return false;
    }
    std::atomic<double> frac{0};
    std::atomic<bool>   cancel{false};
    const LaserDocument doc    = m_doc;
    const LaserDevice   dev    = device();
    const auto          sel    = job_selection();
    auto                future = std::async(std::launch::async, [&] {
        return plan(doc, dev, sel, [&](double f) {
            frac = f;
            return cancel.load();
        });
    });
    std::unique_ptr<wxProgressDialog> dlg;
    const auto                        t0 = std::chrono::steady_clock::now();
    while (future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
        if (!dlg && std::chrono::steady_clock::now() - t0 > std::chrono::milliseconds(300))
            dlg = std::make_unique<wxProgressDialog>(title, _L("Planning the job..."), 1000, this, wxPD_APP_MODAL | wxPD_CAN_ABORT | wxPD_AUTO_HIDE);
        if (dlg && !dlg->Update(int(std::clamp(frac.load(), 0., 1.) * 999))) cancel = true;
    }
    job = future.get();
    dlg.reset();
    if (!job.ok()) {
        if (job.error != "Cancelled") wxMessageBox(wxString::FromUTF8(job.error), title, wxOK | wxICON_WARNING, this);
        return false;
    }
    return true;
}

void LaserPanel::cmd_preview()
{
    LaserJob job;
    if (!plan_job(job, _L("Preview"))) return;
    show_preview(this, m_doc, job, device(), [this] { cmd_save_gcode(); },
                 m_streamer.is_connected() ? std::function<void()>([this] { CallAfter([this] { cmd_start(); }); }) : nullptr);
}

void LaserPanel::cmd_save_gcode()
{
    LaserJob job;
    if (!plan_job(job, _L("Save G-code"))) return;
    GCodeOptions opts;
    std::string  err;
    const std::string text = gcode(job, device(), opts, &err);
    if (text.empty()) {
        wxMessageBox(err.empty() ? _L("No G-code was produced.") : wxString::FromUTF8(err), _L("Save G-code"), wxOK | wxICON_WARNING, this);
        return;
    }
    wxFileDialog dlg(this, _L("Save G-code"), "", "laser.gcode", "G-code (*.gcode;*.nc;*.gc)|*.gcode;*.nc;*.gc", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (dlg.ShowModal() != wxID_OK) return;
    boost::nowide::ofstream f(dlg.GetPath().ToUTF8().data(), std::ios::binary);
    f << text;
    set_status(f ? wxString::Format(_L("G-code saved (%s estimated)"), format_time(job.estimated_time_s)) : _L("Could not write the file"));
}

void LaserPanel::cmd_frame(bool outline)
{
    LaserJob job;
    if (!plan_job(job, _L("Frame"))) return;
    const std::string text = frame_gcode(job, device(), outline ? FrameMode::Outline : FrameMode::Rect);
    if (text.empty()) { set_status(_L("Nothing to frame.")); return; }
    if (!m_streamer.is_connected()) {
        show_gcode(this, _L("Frame"), _L("No laser is connected, so here is the framing G-code instead. Connect on the Laser page to run it on the machine."), text);
        return;
    }
    std::vector<std::string> lines;
    std::istringstream       in(text);
    for (std::string l; std::getline(in, l);) lines.push_back(l);
    if (!m_streamer.start(std::move(lines))) set_status(_L("The laser is busy or not ready (Idle) - try again, or Unlock it."));
    else m_job_running = true;
}

void LaserPanel::cmd_start()
{
    if (!m_streamer.is_connected()) {
        if (wxMessageBox(_L("No laser is connected. Connect it on the Laser page (choose the port, then Connect).\n\nSave the job as G-code instead?"),
                         _L("Start"), wxYES_NO | wxICON_INFORMATION, this) == wxYES)
            cmd_save_gcode();
        return;
    }
    LaserJob job;
    if (!plan_job(job, _L("Start"))) return;
    const LaserDevice& dev = device();
    wxString           summary = wxString::Format(_L("Device: %s\nEstimated time: %s\n\nLayers:\n"), wxString::FromUTF8(dev.name), format_time(job.estimated_time_s));
    const auto         modes = laser_mode_names();
    wxString           warnings;
    for (int l = 0; l < kLayerCount; ++l) {
        if (job.layer_time_s[l] <= 0) continue;
        const LaserLayer& L = m_doc.layers[l];
        const wxString name = L.name.empty() || L.name == wxString::Format("C%02d", l).ToStdString() ? wxString() : " " + wxString::FromUTF8(L.name);
        summary += wxString::Format("  C%02d%s: %s, %g mm/s, %g%% power, %d pass(es)\n", l, name, modes[int(L.mode)], L.speed_mm_s, L.power_max, L.passes);
        if (dev.source == LaserSource::Diode && L.power_max >= 100 && L.mode != LayerMode::Line)
            warnings += wxString::Format(_L("• C%02d fills at 100%% power: diode lasers last longer below ~90%%.\n"), l);
    }
    if (job.start_from == JobSettings::StartFrom::Absolute && job.bounds.defined &&
        (job.bounds.min.x() < -1e-3 || job.bounds.min.y() < -1e-3 || job.bounds.max.x() > dev.bed_w + 1e-3 || job.bounds.max.y() > dev.bed_h + 1e-3))
        warnings += _L("• The job goes past the edge of the bed.\n");
    if (m_status.state != GrblState::Idle) warnings += wxString::Format(_L("• The laser reports '%s', not Idle.\n"), grbl_state_name(m_status.state));
    for (const std::string& w : job.warnings) warnings += "• " + wxString::FromUTF8(w) + "\n";
    if (!warnings.empty()) summary += "\n" + _L("Please check:") + "\n" + warnings;
    summary += "\n" + _L("Keep the lid closed or wear laser safety glasses, and never leave a running laser alone.");
    wxMessageDialog confirm(this, summary, _L("Start the job?"), wxOK | wxCANCEL | wxICON_QUESTION);
    confirm.SetOKCancelLabels(_L("Start"), _L("Cancel"));
    if (confirm.ShowModal() != wxID_OK) return;
    std::string       err;
    const std::string text = gcode(job, dev, GCodeOptions{}, &err);
    if (text.empty()) {
        wxMessageBox(wxString::FromUTF8(err), _L("Start"), wxOK | wxICON_WARNING, this);
        return;
    }
    std::vector<std::string> lines;
    std::istringstream       in(text);
    for (std::string l; std::getline(in, l);) lines.push_back(l);
    if (!m_streamer.start(std::move(lines), job.estimated_time_s)) {
        wxMessageBox(_L("The laser did not accept the job: it must be connected and Idle. If it shows Alarm, press Unlock or Home."), _L("Start"),
                     wxOK | wxICON_WARNING, this);
        return;
    }
    m_job_running = true;
    m_dock->SetSelection(1);
}

void LaserPanel::edit_layer(int layer)
{
    const LaserShape* sample = nullptr;
    for (const LaserShape& s : m_doc.shapes)
        if (s.type == ShapeType::Image && s.layer == layer) { sample = &s; break; }
    if (edit_cut_settings(this, m_doc.layers[layer], layer, sample)) commit(wxString::Format("C%02d settings", layer).ToStdString());
}

// ---- Device ----------------------------------------------------------------------------------------

bool LaserPanel::is_simulator_port() const { return m_port_choice->GetStringSelection() == kSimulator; }

void LaserPanel::refresh_ports()
{
    m_port_choice->Clear();
    for (const std::string& p : Utils::scan_serial_ports()) m_port_choice->Append(wxString::FromUTF8(p));
    m_port_choice->Append(kSimulator);
    if (!m_last_port.empty()) m_port_choice->SetStringSelection(wxString::FromUTF8(m_last_port));
    if (m_port_choice->GetSelection() < 0) m_port_choice->SetSelection(0);
}

void LaserPanel::connect_device()
{
    const std::string port = m_port_choice->GetStringSelection().ToUTF8().data();
    unsigned long     baud = 115200;
    m_baud_choice->GetStringSelection().ToULong(&baud);
    if (port.empty()) {
        wxMessageBox(_L("No serial port found. Plug the laser in with USB, press ↻, and try again."), _L("Connect"), wxOK | wxICON_INFORMATION, this);
        return;
    }
    std::unique_ptr<ITransport> transport;
    if (is_simulator_port()) {
        auto sim         = std::make_shared<GrblSimulator::Shared>();
        sim->time_scale  = 1.0;   // moves take as long as on a real machine
        transport        = std::make_unique<GrblSimulator>(sim);
    }
    else transport = std::make_unique<SerialTransport>(port, unsigned(baud));
    wxBusyCursor busy;
    std::string  err;
    if (!m_streamer.connect(std::move(transport), device(), &err)) {
        wxMessageBox(_L("Could not connect:") + "\n" + wxString::FromUTF8(err), _L("Connect"), wxOK | wxICON_WARNING, this);
        return;
    }
    m_last_port = port;
    if (m_devices[m_device_idx].baud != baud) m_devices[m_device_idx].baud = unsigned(baud);
    save_prefs();
    m_state_text->SetLabel(_L("Connecting..."));
    refresh_device_ui();
}

void LaserPanel::disconnect_device()
{
    if (m_streamer.progress().running &&
        wxMessageBox(_L("A job is running. Disconnecting stops streaming but the laser may keep running what it has buffered. Disconnect anyway?"),
                     _L("Disconnect"), wxYES_NO | wxICON_WARNING, this) != wxYES)
        return;
    m_streamer.disconnect();
    refresh_device_ui();
}

void LaserPanel::on_streamer_status(const GrblStatus& st)
{
    const bool moved = (st.mpos - m_status.mpos).norm() > 1e-4 || st.state != m_status.state;
    const bool alarm_changed = (st.state == GrblState::Alarm) != (m_status.state == GrblState::Alarm);
    m_status = st;
    wxString state = wxString::FromUTF8(grbl_state_name(st.state));
    if (st.state == GrblState::Alarm && st.alarm > 0) state += wxString::Format(" %d", st.alarm);
    if (st.ov_feed != 100 || st.ov_spindle != 100) state += wxString::Format(_L("   (feed %d%%, power %d%%)"), st.ov_feed, st.ov_spindle);
    m_state_text->SetLabel(state);
    m_state_text->SetForegroundColour(st.state == GrblState::Alarm ? wxColour(200, 40, 40) : st.state == GrblState::Run ? wxColour(40, 140, 40) : wxColour(40, 40, 40));
    const wxString pos = wxString::Format(_L("X %.2f  Y %.2f  Z %.2f   (work)   F %.0f  S %.0f"), st.wpos.x(), st.wpos.y(), st.wpos.z(), st.feed, st.spindle);
    m_pos_text->SetLabel(pos);
    m_move_pos->SetLabel(wxString::Format(_L("Position: X %.2f  Y %.2f  Z %.2f"), st.wpos.x(), st.wpos.y(), st.wpos.z()));
    if (alarm_changed) refresh_device_ui();
    if (moved) m_canvas->refresh();
}

void LaserPanel::on_streamer_console(const ConsoleLine& l)
{
    if (m_console == nullptr) return;
    const wxString prefix = l.dir == ConsoleLine::Dir::Tx ? "> " : l.dir == ConsoleLine::Dir::Rx ? "< " : "* ";
    const long     row    = m_console->InsertItem(m_console->GetItemCount(), prefix + wxString::FromUTF8(l.text));
    const bool     bad    = l.text.rfind("error", 0) == 0 || l.text.rfind("ALARM", 0) == 0;
    m_console->SetItemTextColour(row, bad ? wxColour(200, 40, 40) : l.dir == ConsoleLine::Dir::Tx ? wxColour(30, 90, 200)
                                     : l.dir == ConsoleLine::Dir::Info ? wxColour(120, 120, 120) : wxColour(30, 30, 30));
    while (m_console->GetItemCount() > long(GrblStreamer::kConsoleLines)) m_console->DeleteItem(0);
    m_console->EnsureVisible(m_console->GetItemCount() - 1);
}

void LaserPanel::on_streamer_progress(const JobProgress& p)
{
    m_progress->SetValue(p.lines_total ? int(1000 * p.lines_acked / p.lines_total) : 0);
    if (p.running)
        m_progress_text->SetLabel(wxString::Format(_L("%s%zu / %zu lines   %s elapsed%s"), p.paused ? _L("Paused - ") : wxString(), p.lines_acked, p.lines_total,
                                                   format_time(p.elapsed_s), p.eta_s > 0 ? wxString::Format(_L(", about %s left"), format_time(p.eta_s)) : wxString()));
}

void LaserPanel::on_streamer_error(const std::string& text, int line)
{
    set_status(wxString::FromUTF8(text));
    if (line >= 0) {
        wxMessageBox(wxString::Format(_L("The laser reported a problem at line %d:\n\n%s\n\nThe job is paused. Resume to skip that line, or Stop."), line,
                                      wxString::FromUTF8(text)),
                     _L("Laser error"), wxOK | wxICON_WARNING, this);
    } else if (m_status.state == GrblState::Alarm || text.find("alarm") != std::string::npos || text.find("limit") != std::string::npos) {
        wxMessageDialog dlg(this, wxString::FromUTF8(text) + "\n\n" + _L("Unlock the machine to continue, or Home it if the position was lost."),
                            _L("Laser alarm"), wxYES_NO | wxICON_WARNING);
        dlg.SetYesNoLabels(_L("Unlock ($X)"), _L("Close"));
        if (dlg.ShowModal() == wxID_YES) m_streamer.unlock();
    }
}

void LaserPanel::on_streamer_done(bool completed, const std::string& reason)
{
    m_job_running = false;
    const JobProgress p = m_streamer.progress();
    set_status(completed ? wxString::Format(_L("Job finished in %s"), format_time(p.elapsed_s)) : _L("Job ended: ") + wxString::FromUTF8(reason));
    m_progress_text->SetLabel(completed ? _L("Done.") : wxString::FromUTF8(reason));
    if (completed) m_progress->SetValue(1000);
}

void LaserPanel::on_streamer_connection(bool connected, const std::string& error)
{
    if (!connected) {
        m_status = GrblStatus{};
        m_state_text->SetLabel(_L("Not connected"));
        m_pos_text->SetLabel("");
        m_move_pos->SetLabel(_L("Position: not connected"));
        if (m_fire) m_fire->SetValue(false);
        if (!error.empty()) wxMessageBox(wxString::FromUTF8(error), _L("Laser"), wxOK | wxICON_WARNING, this);
    }
    refresh_device_ui();
    refresh_hint();
    m_canvas->refresh();
}

}} // namespace Slic3r::GUI

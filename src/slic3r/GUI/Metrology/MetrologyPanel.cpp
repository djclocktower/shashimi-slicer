// I18N first: wx defines its own _() otherwise.
#include "slic3r/GUI/I18N.hpp"

#include "MetrologyPanel.hpp"
#include "MetrologyMesh.hpp"
#include "MetrologyView.hpp"

#include "libslic3r/Utils.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/GUI_ObjectList.hpp"

#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include <nlohmann/json.hpp>

#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/choice.h>
#include <wx/dialog.h>
#include <wx/dirdlg.h>
#include <wx/filedlg.h>
#include <wx/graphics.h>
#include <wx/image.h>
#include <wx/listctrl.h>
#include <wx/msgdlg.h>
#include <wx/scrolwin.h>
#include <wx/simplebook.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/stattext.h>
#include <wx/utils.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <random>
#include <sstream>
#include <type_traits>

namespace Slic3r { namespace GUI {

namespace M  = Slic3r::Metrology;
namespace fs = boost::filesystem;

namespace {

// Instrument colours: a dark frame around a black image, like an image-dimension system.
const wxColour kBar(33, 37, 43);
const wxColour kBarText(226, 229, 233);
const wxColour kStrip(44, 49, 57);
const wxColour kActive(0, 132, 222);
const wxColour kPage(243, 244, 246);
const wxColour kOk(36, 160, 72);
const wxColour kNg(206, 44, 44);
const wxColour kNone(92, 98, 108);

const wxColour kLevelColours[] = {{80, 170, 255}, {255, 170, 60}, {120, 220, 120}, {230, 110, 220},
                                  {250, 230, 80}, {90, 220, 220}, {240, 120, 120}, {180, 160, 255}};

wxString num(double v, int digits = 3) { return wxString::Format("%.*f", digits, v); }

std::string settings_file() { return (fs::path(data_dir()) / "metrology.json").string(); }

bool write_text(const wxString& path, const std::string& text)
{
    boost::nowide::ofstream f(path.ToUTF8().data(), std::ios::binary);
    if (!f) return false;
    f << text;
    return bool(f);
}

wxStaticText* label(wxWindow* parent, const wxString& text, bool bold = false)
{
    auto* t = new wxStaticText(parent, wxID_ANY, text);
    if (bold) t->SetFont(t->GetFont().Bold());
    return t;
}

wxStaticText* hint(wxWindow* parent, const wxString& text, int wrap)
{
    auto* t = new wxStaticText(parent, wxID_ANY, text);
    t->SetForegroundColour(wxColour(100, 104, 112));
    t->Wrap(wrap);
    return t;
}

wxListCtrl* report(wxWindow* parent, const std::vector<std::pair<wxString, int>>& columns, int height)
{
    auto* l = new wxListCtrl(parent, wxID_ANY, wxDefaultPosition, wxSize(-1, height), wxLC_REPORT | wxBORDER_SIMPLE);
    for (size_t i = 0; i < columns.size(); ++i) l->InsertColumn(long(i), columns[i].first, wxLIST_FORMAT_LEFT, columns[i].second);
    return l;
}

} // namespace

// ---- Construction -----------------------------------------------------------------------------

MetrologyPanel::MetrologyPanel(wxWindow* parent) : wxPanel(parent, wxID_ANY)
{
    SetBackgroundColour(kPage);
    m_scene = M::sim_preset_bracket();
    load_settings();

    auto* root = new wxBoxSizer(wxVERTICAL);
    build_top_bar(root);
    auto* mid = new wxBoxSizer(wxHORIZONTAL);
    mid->Add(build_tool_strip(this), 0, wxEXPAND);
    m_view = new MetrologyView(this);
    mid->Add(m_view, 1, wxEXPAND);
    m_pages = new wxSimplebook(this, wxID_ANY);
    m_pages->SetMinSize(wxSize(FromDIP(420), -1));
    m_pages->AddPage(build_inspect_page(m_pages), "");
    m_pages->AddPage(build_calibrate_page(m_pages), "");
    m_pages->AddPage(build_trace_page(m_pages), "");
    m_pages->AddPage(build_reverse_page(m_pages), "");
    m_pages->AddPage(build_probe_page(m_pages), "");
    mid->Add(m_pages, 0, wxEXPAND);
    root->Add(mid, 1, wxEXPAND);

    // Status line.
    auto* bar = new wxPanel(this);
    bar->SetBackgroundColour(kBar);
    auto*        bs   = new wxBoxSizer(wxHORIZONTAL);
    const wxFont mono = wxFont(wxFontInfo(9).Family(wxFONTFAMILY_TELETYPE));
    m_cursor          = new wxStaticText(bar, wxID_ANY, "X ---.--- Y ---.--- mm");
    m_cal_text        = new wxStaticText(bar, wxID_ANY, "");
    m_status          = new wxStaticText(bar, wxID_ANY, "");
    for (wxStaticText* t : {m_cursor, m_cal_text, m_status}) {
        t->SetForegroundColour(kBarText);
        t->SetFont(mono);
    }
    bs->Add(m_cursor, 0, wxALL, 5);
    bs->Add(m_cal_text, 0, wxALL, 5);
    bs->Add(m_status, 1, wxALL, 5);
    bar->SetSizer(bs);
    root->Add(bar, 0, wxEXPAND);
    SetSizer(root);

    m_view->on_overlay    = [this](wxGraphicsContext& gc) { draw_overlay(gc); };
    m_view->on_drag_done  = [this](const P2& a, const P2& b) {
        if (m_tool == ToolBox) set_box(a, b);
        else if (m_tool == ToolLine || m_tool == ToolCircle || m_tool == ToolWidth) add_feature_from_drag(a, b);
    };
    m_view->on_click = [this](const P2& p) {
        if (m_tool == ToolTwoPoint) two_point_click(p);
    };
    m_view->on_hover = [this](const P2& p, bool inside) {
        if (!inside) return;
        const double z  = m_mode == ModeInspect ? m_feature_z : 0.0;
        const float  g  = m_frame.empty() ? 0.f : m_frame.sample(p.x, p.y);
        wxString     xy = wxString::Format("px %7.1f %7.1f  ", p.x, p.y);
        if (m_cam.ok) {
            const P2 mm = m_cam.to_plane(p, z);
            xy += wxString::Format("X %8.3f Y %8.3f mm", mm.x, mm.y);
        }
        m_cursor->SetLabel(xy + wxString::Format("  grey %3.0f", g));
    };

    set_mode(ModeInspect);
    refresh_cal_status();
    refresh_budget();
    update_judge();
    // The first frame (and, in the simulator, a calibration) once the tab is on screen.
    CallAfter([this] {
        if (simulator() && !m_cam.ok) auto_calibrate_simulator();
        capture();
    });
}

MetrologyPanel::~MetrologyPanel() { save_settings(); }

void MetrologyPanel::build_top_bar(wxSizer* root)
{
    auto* bar = new wxPanel(this);
    bar->SetBackgroundColour(kBar);
    auto* s     = new wxBoxSizer(wxHORIZONTAL);
    auto* title = new wxStaticText(bar, wxID_ANY, _L("MEASURE"));
    title->SetForegroundColour(kBarText);
    title->SetFont(wxFont(wxFontInfo(13).Bold()));
    s->Add(title, 0, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, FromDIP(12));

    const wxString names[ModeCount] = {_L("Inspect"), _L("Calibrate"), _L("Trace"), _L("Reverse Engineer"), _L("Probe")};
    for (int i = 0; i < ModeCount; ++i) {
        m_mode_btn[size_t(i)] = new wxButton(bar, wxID_ANY, names[i], wxDefaultPosition, wxDefaultSize, wxBORDER_NONE);
        m_mode_btn[size_t(i)]->Bind(wxEVT_BUTTON, [this, i](wxCommandEvent&) { set_mode(Mode(i)); });
        s->Add(m_mode_btn[size_t(i)], 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    }
    s->AddSpacer(FromDIP(16));

    auto* src = new wxStaticText(bar, wxID_ANY, _L("Source"));
    src->SetForegroundColour(kBarText);
    s->Add(src, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
    m_source_choice = new wxChoice(bar, wxID_ANY);
    m_source_choice->Append(_L("Simulator"));
    m_source_choice->Append(_L("Image files"));
    m_source_choice->SetSelection(int(m_source));
    m_source_choice->SetToolTip(_L("Simulator: a synthetic P1P camera and probe, for trying every step without a printer.\n"
                                   "Image files: frames saved from the printer's camera."));
    m_source_choice->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) {
        m_source = Source(m_source_choice->GetSelection());
        m_scene_choice->Enable(simulator());
    });
    s->Add(m_source_choice, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
    m_scene_choice = new wxChoice(bar, wxID_ANY);
    m_scene_choice->Append(_L("Bracket"));
    m_scene_choice->Append(_L("Test coupon"));
    m_scene_choice->Append(_L("Mat only"));
    m_scene_choice->SetSelection(0);
    m_scene_choice->Enable(simulator());
    m_scene_choice->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) {
        const int k = m_scene_choice->GetSelection();
        m_scene     = k == 0 ? M::sim_preset_bracket() : k == 1 ? M::sim_preset_coupon() : M::sim_preset_empty();
        capture();
    });
    s->Add(m_scene_choice, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 8);

    auto add_btn = [&](const wxString& text, const wxString& tip, std::function<void()> fn) {
        auto* b = new wxButton(bar, wxID_ANY, text);
        b->SetToolTip(tip);
        b->Bind(wxEVT_BUTTON, [fn](wxCommandEvent&) { fn(); });
        s->Add(b, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 4);
    };
    add_btn(_L("Capture"), _L("Take a frame: render one in the simulator, or open a saved camera frame."), [this] { capture(); });
    add_btn(_L("Light-off frame…"),
            _L("Open the same view with the chamber light off. With \"Light pair\" on, it is subtracted so only the printer's own light remains."),
            [this] { open_image(true); });
    m_light_pair = new wxCheckBox(bar, wxID_ANY, _L("Light pair"));
    m_light_pair->SetForegroundColour(kBarText);
    m_light_pair->SetValue(true);
    m_light_pair->SetToolTip(_L("Subtract a light-off frame from the light-on frame to remove room light."));
    s->Add(m_light_pair, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 8);
    add_btn(_L("Run"), _L("Re-run every inspection feature on the current frame."), [this] { run_program(); });

    s->AddStretchSpacer();
    m_judge_box  = new wxPanel(bar, wxID_ANY, wxDefaultPosition, wxSize(FromDIP(110), FromDIP(40)));
    m_judge_text = new wxStaticText(m_judge_box, wxID_ANY, "--", wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE_HORIZONTAL);
    m_judge_text->SetFont(wxFont(wxFontInfo(20).Bold()));
    m_judge_text->SetForegroundColour(*wxWHITE);
    auto* js = new wxBoxSizer(wxVERTICAL);
    js->AddStretchSpacer();
    js->Add(m_judge_text, 0, wxEXPAND);
    js->AddStretchSpacer();
    m_judge_box->SetSizer(js);
    s->Add(m_judge_box, 0, wxALL, 4);
    bar->SetSizer(s);
    root->Add(bar, 0, wxEXPAND);
}

wxWindow* MetrologyPanel::build_tool_strip(wxWindow* parent)
{
    auto* strip = new wxPanel(parent);
    strip->SetBackgroundColour(kStrip);
    auto*          s = new wxBoxSizer(wxVERTICAL);
    const wxString names[ToolCount] = {_L("Select"), _L("Line"), _L("Circle"), _L("Width"), _L("Box"), _L("2-Point")};
    const wxString tips[ToolCount]  = {
        _L("Pan with the right or middle button, zoom with the wheel, double-click to fit."),
        _L("Line: drag across an edge, in the direction the scans should run."),
        _L("Circle: drag from the centre of a hole or boss out to its edge."),
        _L("Width: drag across both edges of a rib or slot."),
        _L("Box: drag around the part to trace or reverse engineer."),
        _L("2-Point: click two points a known distance apart to set the scale without a mat."),
    };
    for (int i = 0; i < ToolCount; ++i) {
        auto* b = new wxButton(strip, wxID_ANY, names[i], wxDefaultPosition, wxSize(FromDIP(78), FromDIP(34)), wxBORDER_NONE);
        b->SetToolTip(tips[i]);
        b->Bind(wxEVT_BUTTON, [this, i](wxCommandEvent&) { set_tool(Tool(i)); });
        m_tool_btn[size_t(i)] = b;
        s->Add(b, 0, wxALL, 3);
    }
    strip->SetSizer(s);
    return strip;
}

wxSpinCtrlDouble* MetrologyPanel::add_number(wxWindow* parent, wxFlexGridSizer* grid, const wxString& text, double& value, double lo,
                                             double hi, double step, int digits, std::function<void()> changed)
{
    grid->Add(label(parent, text), 0, wxALIGN_CENTER_VERTICAL);
    auto* sp = new wxSpinCtrlDouble(parent, wxID_ANY, "", wxDefaultPosition, wxSize(FromDIP(110), -1), wxSP_ARROW_KEYS, lo, hi, value, step);
    sp->SetDigits(unsigned(digits));
    auto apply = [sp, &value, changed](wxEvent&) {
        value = sp->GetValue();
        if (changed) changed();
    };
    sp->Bind(wxEVT_SPINCTRLDOUBLE, apply);
    sp->Bind(wxEVT_TEXT_ENTER, apply);
    grid->Add(sp, 0, wxEXPAND);
    return sp;
}

// ---- Pages ----------------------------------------------------------------------------------------

wxWindow* MetrologyPanel::build_inspect_page(wxWindow* parent)
{
    auto* p = new wxScrolledWindow(parent);
    p->SetScrollRate(0, 10);
    auto* s = new wxBoxSizer(wxVERTICAL);
    s->Add(label(p, _L("Inspection program"), true), 0, wxALL, 8);
    s->Add(hint(p, _L("Pick Line, Circle or Width in the tool strip and drag over an edge in the image. Every feature "
                      "re-runs on each new frame. Give a feature a tolerance to get an OK/NG judgement."),
                FromDIP(390)),
           0, wxLEFT | wxRIGHT, 8);
    auto* g = new wxFlexGridSizer(2, 4, 8);
    g->AddGrowableCol(1);
    add_number(p, g, _L("Feature height above the mat (mm)"), m_feature_z, 0, 200, 0.1, 2);
    add_number(p, g, _L("Edge contrast (grey levels)"), m_contrast, 2, 200, 1, 0);
    add_number(p, g, _L("Scans per feature"), m_scans, 3, 200, 1, 0);
    add_number(p, g, _L("Line/width box width (px)"), m_roi_width, 4, 2000, 2, 0);
    g->Add(label(p, _L("Edge polarity")), 0, wxALIGN_CENTER_VERTICAL);
    m_polarity = new wxChoice(p, wxID_ANY);
    m_polarity->Append(_L("Any"));
    m_polarity->Append(_L("Dark to light"));
    m_polarity->Append(_L("Light to dark"));
    m_polarity->SetSelection(0);
    g->Add(m_polarity, 0, wxEXPAND);
    s->Add(g, 0, wxEXPAND | wxALL, 8);
    auto* bright = new wxCheckBox(p, wxID_ANY, _L("Parts are brighter than the background (black mat face)"));
    bright->SetValue(m_part_bright);
    bright->Bind(wxEVT_CHECKBOX, [this, bright](wxCommandEvent&) {
        m_part_bright = bright->GetValue();
        run_program();
    });
    s->Add(bright, 0, wxLEFT | wxRIGHT | wxBOTTOM, 8);

    m_features = report(p,
                        {{_L("No."), 40}, {_L("Name"), 90}, {_L("Measured"), 80}, {_L("Nominal"), 64}, {_L("Tol."), 70},
                         {_L("Dev."), 64}, {_L("Judge"), 50}},
                        FromDIP(260));
    m_features->Bind(wxEVT_LIST_ITEM_SELECTED, [this](wxListEvent&) { m_view->Refresh(); });
    m_features->Bind(wxEVT_LIST_ITEM_ACTIVATED, [this](wxListEvent&) { edit_tolerance(); });
    s->Add(m_features, 0, wxEXPAND | wxLEFT | wxRIGHT, 8);

    auto* row1 = new wxBoxSizer(wxHORIZONTAL);
    auto* row2 = new wxBoxSizer(wxHORIZONTAL);
    auto  btn  = [p](wxSizer* row, const wxString& text, std::function<void()> fn) {
        auto* b = new wxButton(p, wxID_ANY, text);
        b->Bind(wxEVT_BUTTON, [fn](wxCommandEvent&) { fn(); });
        row->Add(b, 0, wxRIGHT, 4);
    };
    btn(row1, _L("Distance (2 selected)"), [this] { add_derived(M::FeatureKind::Distance); });
    btn(row1, _L("Angle (2 selected)"), [this] { add_derived(M::FeatureKind::Angle); });
    btn(row2, _L("Tolerance…"), [this] { edit_tolerance(); });
    btn(row2, _L("Delete"), [this] { delete_feature(); });
    btn(row2, _L("Export CSV…"), [this] { export_csv(); });
    s->Add(row1, 0, wxALL, 8);
    s->Add(row2, 0, wxLEFT | wxRIGHT | wxBOTTOM, 8);
    p->SetSizer(s);
    return p;
}

wxWindow* MetrologyPanel::build_calibrate_page(wxWindow* parent)
{
    auto* p = new wxScrolledWindow(parent);
    p->SetScrollRate(0, 10);
    auto* s = new wxBoxSizer(wxVERTICAL);
    s->Add(label(p, _L("Measuring mat"), true), 0, wxALL, 8);
    s->Add(hint(p, _L("Lay the printed measuring mat on the plate, whole and flat, with its marker dot in view. "
                      "Detect it in a frame taken at the bed height you will measure at."),
                FromDIP(390)),
           0, wxLEFT | wxRIGHT, 8);
    auto* g = new wxFlexGridSizer(2, 4, 8);
    g->AddGrowableCol(1);
    add_number(p, g, _L("Ring length X (mm)"), m_mat.ring_w, 20, 400, 1, 1);
    add_number(p, g, _L("Ring length Y (mm)"), m_mat.ring_h, 20, 400, 1, 1);
    add_number(p, g, _L("Dot pitch (mm)"), m_mat.pitch, 2, 50, 0.5, 2);
    add_number(p, g, _L("Dot diameter (mm)"), m_mat.dot_d, 0.5, 20, 0.1, 2);
    add_number(p, g, _L("Marker diameter (mm)"), m_mat.marker_d, 1, 40, 0.1, 2);
    add_number(p, g, _L("Mat thickness (mm)"), m_mat.thickness, 0.5, 20, 0.1, 2);
    s->Add(g, 0, wxEXPAND | wxALL, 8);
    auto* bright = new wxCheckBox(p, wxID_ANY, _L("White dots on the black face"));
    bright->SetValue(m_mat.bright_dots);
    bright->Bind(wxEVT_CHECKBOX, [this, bright](wxCommandEvent&) { m_mat.bright_dots = bright->GetValue(); });
    s->Add(bright, 0, wxLEFT | wxRIGHT, 8);
    auto* detect = new wxButton(p, wxID_ANY, _L("Detect mat in this frame"));
    detect->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { detect_mat(); });
    s->Add(detect, 0, wxALL, 8);
    m_cal_result = new wxStaticText(p, wxID_ANY, "");
    s->Add(m_cal_result, 0, wxLEFT | wxRIGHT, 8);

    s->Add(label(p, _L("Camera height"), true), 0, wxLEFT | wxRIGHT | wxTOP, 8);
    s->Add(hint(p, _L("Raise the bed towards the camera by a known amount (on a P1P, lower Z) and add a frame of the mat. "
                      "The camera's position comes out of how much the dots spread. It is needed for parallax "
                      "correction and for reading probe touches."),
                FromDIP(390)),
           0, wxALL, 8);
    auto* g2 = new wxFlexGridSizer(2, 4, 8);
    g2->AddGrowableCol(1);
    add_number(p, g2, _L("Bed raised by (mm)"), m_raise_mm, -50, 100, 0.5, 2);
    s->Add(g2, 0, wxEXPAND | wxLEFT | wxRIGHT, 8);
    auto* r1  = new wxBoxSizer(wxHORIZONTAL);
    auto* add = new wxButton(p, wxID_ANY, _L("Add raised frame"));
    add->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { add_raised_frame(); });
    auto* solve = new wxButton(p, wxID_ANY, _L("Solve camera height"));
    solve->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { solve_camera(); });
    r1->Add(add, 0, wxRIGHT, 4);
    r1->Add(solve, 0);
    s->Add(r1, 0, wxALL, 8);
    m_raised_list = report(p, {{_L("Raised (mm)"), 90}, {_L("Dots"), 70}, {_L("RMS (mm)"), 80}}, FromDIP(90));
    s->Add(m_raised_list, 0, wxEXPAND | wxLEFT | wxRIGHT, 8);
    auto* autob = new wxButton(p, wxID_ANY, _L("Auto-calibrate (simulator)"));
    autob->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { auto_calibrate_simulator(); });
    s->Add(autob, 0, wxALL, 8);

    s->Add(label(p, _L("Scale without a mat"), true), 0, wxLEFT | wxRIGHT | wxTOP, 8);
    auto* g3 = new wxFlexGridSizer(2, 4, 8);
    g3->AddGrowableCol(1);
    add_number(p, g3, _L("Known distance (mm)"), m_two_point_mm, 0.1, 1000, 1, 3);
    s->Add(g3, 0, wxEXPAND | wxALL, 8);
    s->Add(hint(p, _L("Pick 2-Point and click both ends of the known distance. Only flat, straight-on work is accurate this way."),
                FromDIP(390)),
           0, wxLEFT | wxRIGHT, 8);
    auto* r2   = new wxBoxSizer(wxHORIZONTAL);
    auto* save = new wxButton(p, wxID_ANY, _L("Save calibration"));
    save->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
        save_settings();
        set_status(_L("Calibration saved."));
    });
    r2->Add(save, 0);
    s->Add(r2, 0, wxALL, 8);
    p->SetSizer(s);
    return p;
}

wxWindow* MetrologyPanel::build_trace_page(wxWindow* parent)
{
    auto* p = new wxScrolledWindow(parent);
    p->SetScrollRate(0, 10);
    auto* s = new wxBoxSizer(wxVERTICAL);
    s->Add(label(p, _L("Outline trace"), true), 0, wxALL, 8);
    s->Add(hint(p, _L("Pick Box and drag around the part. The outline is traced on the mat plane. For a part with "
                      "height, Reverse Engineer corrects it for parallax using probe heights."),
                FromDIP(390)),
           0, wxLEFT | wxRIGHT, 8);
    auto* g = new wxFlexGridSizer(2, 4, 8);
    g->AddGrowableCol(1);
    add_number(p, g, _L("Resolution (mm)"), m_trace_res, 0.02, 1, 0.01, 3);
    add_number(p, g, _L("Simplify (mm)"), m_trace_simplify, 0, 1, 0.01, 3);
    add_number(p, g, _L("Extrusion height (mm)"), m_extrude_h, 0.2, 200, 0.5, 2);
    s->Add(g, 0, wxEXPAND | wxALL, 8);
    auto* r1 = new wxBoxSizer(wxHORIZONTAL);
    auto  btn = [p, r1](const wxString& text, std::function<void()> fn) {
        auto* b = new wxButton(p, wxID_ANY, text);
        b->Bind(wxEVT_BUTTON, [fn](wxCommandEvent&) { fn(); });
        r1->Add(b, 0, wxRIGHT, 4);
    };
    btn(_L("Trace outline"), [this] { do_trace(); });
    btn(_L("Export SVG…"), [this] { export_svg(m_trace); });
    btn(_L("Extrude to plate"), [this] { send_extrusion(); });
    s->Add(r1, 0, wxALL, 8);
    m_trace_info = new wxStaticText(p, wxID_ANY, "");
    s->Add(m_trace_info, 0, wxALL, 8);
    p->SetSizer(s);
    return p;
}

wxWindow* MetrologyPanel::build_reverse_page(wxWindow* parent)
{
    auto* p = new wxScrolledWindow(parent);
    p->SetScrollRate(0, 10);
    auto* s = new wxBoxSizer(wxVERTICAL);
    s->Add(label(p, _L("Reverse engineering"), true), 0, wxALL, 8);
    s->Add(hint(p, _L("1. Calibrate the mat and the camera height.  2. Pick Box and drag around the part.  3. Start, then "
                      "run each batch of touches: in the simulator, or as G-code on the printer with the heights read "
                      "back from the camera frames.  4. Build the model and send it to the plate."),
                FromDIP(390)),
           0, wxLEFT | wxRIGHT, 8);
    auto* g = new wxFlexGridSizer(2, 4, 8);
    g->AddGrowableCol(1);
    add_number(p, g, _L("Grid pitch (mm)"), m_re_set.grid, 1, 30, 0.5, 2);
    add_number(p, g, _L("Nozzle face radius (mm)"), m_re_set.tip_radius, 0.1, 2, 0.05, 2);
    add_number(p, g, _L("Level tolerance (mm)"), m_re_set.level_tol, 0.05, 5, 0.05, 2);
    add_number(p, g, _L("Step resolution (mm)"), m_re_set.target_res, 0.02, 1, 0.01, 3);
    add_number(p, g, _L("Touches per step per round"), m_re_touches, 1, 9, 1, 0);
    add_number(p, g, _L("Outline anchors"), m_re_anchors, 0, 12, 1, 0);
    add_number(p, g, _L("Tallest expected part (mm)"), m_max_height, 1, 200, 1, 1);
    add_number(p, g, _L("Simulator touch noise (mm)"), m_sim_noise, 0, 0.5, 0.005, 3);
    s->Add(g, 0, wxEXPAND | wxALL, 8);

    auto row = [](wxSizer* parent_sizer) {
        auto* r = new wxBoxSizer(wxHORIZONTAL);
        parent_sizer->Add(r, 0, wxLEFT | wxRIGHT | wxBOTTOM, 8);
        return r;
    };
    auto btn = [p](wxSizer* r, const wxString& text, std::function<void()> fn) {
        auto* b = new wxButton(p, wxID_ANY, text);
        b->Bind(wxEVT_BUTTON, [fn](wxCommandEvent&) { fn(); });
        r->Add(b, 0, wxRIGHT, 4);
    };
    auto* r0 = row(s);
    btn(r0, _L("Start session"), [this] { re_start(); });
    btn(r0, _L("Run batch (simulator)"), [this] { re_run_batch_simulator(); });
    m_re_state = new wxStaticText(p, wxID_ANY, "");
    s->Add(m_re_state, 0, wxLEFT | wxRIGHT | wxBOTTOM, 8);
    m_re_batch = report(p, {{_L("#"), 50}, {_L("X (mm)"), 80}, {_L("Y (mm)"), 80}, {_L("Touch"), 150}}, FromDIP(140));
    s->Add(m_re_batch, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
    auto* r1 = row(s);
    btn(r1, _L("Save batch G-code…"), [this] { re_save_batch_gcode(); });
    btn(r1, _L("Heights from frames…"), [this] { re_read_heights_from_frames(); });
    auto* r2 = row(s);
    btn(r2, _L("Heights from CSV…"), [this] { re_load_heights_csv(); });
    btn(r2, _L("Build model"), [this] { re_build(); });
    m_re_levels = report(p, {{_L("Level"), 50}, {_L("Height (mm)"), 90}, {_L("σ (mm)"), 70}, {_L("Touches"), 70}, {_L("Area (mm²)"), 90}},
                         FromDIP(110));
    s->Add(m_re_levels, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 8);
    auto* r3 = row(s);
    btn(r3, _L("Send to plate"), [this] { re_send_to_plate(); });
    btn(r3, _L("Export STL…"), [this] { re_export_stl(); });
    btn(r3, _L("Export SVG…"), [this] { export_svg(m_re_model.footprint); });
    p->SetSizer(s);
    return p;
}

wxWindow* MetrologyPanel::build_probe_page(wxWindow* parent)
{
    auto* p = new wxScrolledWindow(parent);
    p->SetScrollRate(0, 10);
    auto* s = new wxBoxSizer(wxVERTICAL);
    s->Add(label(p, _L("Nozzle-touch probe"), true), 0, wxALL, 8);
    s->Add(hint(p, _L("Untested on hardware. The G-code raises the bed with G380 until the nozzle touches, then holds so the "
                      "camera can see where the bed stopped. Run it first on an empty plate with the filament unloaded and "
                      "the nozzle cold, and be ready to stop the print."),
                FromDIP(390)),
           0, wxLEFT | wxRIGHT, 8);
    auto* g = new wxFlexGridSizer(2, 4, 8);
    g->AddGrowableCol(1);
    add_number(p, g, _L("Mat marker on printer X (mm)"), m_probe.mat_x, -50, 400, 0.5, 2);
    add_number(p, g, _L("Mat marker on printer Y (mm)"), m_probe.mat_y, -50, 400, 0.5, 2);
    add_number(p, g, _L("Mat rotation (°)"), m_probe.mat_rot_deg, -180, 180, 0.5, 2);
    add_number(p, g, _L("Mat reference touch X (mm, mat)"), m_mat_ref.x, -20, 400, 1, 1);
    add_number(p, g, _L("Mat reference touch Y (mm, mat)"), m_mat_ref.y, -20, 400, 1, 1);
    add_number(p, g, _L("Hold at contact (s)"), m_probe.hold_s, 1, 10, 1, 0);
    add_number(p, g, _L("Probe feed (mm/min)"), m_probe.probe_feed, 30, 1200, 10, 0);
    add_number(p, g, _L("Travel clearance (mm)"), m_probe.clearance, 1, 30, 0.5, 1);
    s->Add(g, 0, wxEXPAND | wxALL, 8);

    s->Add(label(p, _L("Repeatability test"), true), 0, wxLEFT | wxRIGHT | wxTOP, 8);
    auto* g2 = new wxFlexGridSizer(2, 4, 8);
    g2->AddGrowableCol(1);
    add_number(p, g2, _L("Printer X (mm)"), m_repeat_x, 0, 400, 1, 1);
    add_number(p, g2, _L("Printer Y (mm)"), m_repeat_y, 0, 400, 1, 1);
    add_number(p, g2, _L("Touches"), m_repeat_n, 2, 50, 1, 0);
    s->Add(g2, 0, wxEXPAND | wxALL, 8);
    auto* r1 = new wxBoxSizer(wxHORIZONTAL);
    auto* gb = new wxButton(p, wxID_ANY, _L("Save test G-code…"));
    gb->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { save_repeatability_gcode(); });
    auto* ab = new wxButton(p, wxID_ANY, _L("Analyze frames…"));
    ab->SetToolTip(_L("Pick a folder of camera frames recorded during the test: one image per hold, or a frame sequence "
                      "in which the holds are found automatically."));
    ab->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { analyze_frames(); });
    r1->Add(gb, 0, wxRIGHT, 4);
    r1->Add(ab, 0);
    s->Add(r1, 0, wxLEFT | wxRIGHT, 8);
    m_probe_result = new wxStaticText(p, wxID_ANY, "");
    s->Add(m_probe_result, 0, wxALL, 8);

    s->Add(label(p, _L("Error budget (estimates)"), true), 0, wxLEFT | wxRIGHT | wxTOP, 8);
    auto* g3 = new wxFlexGridSizer(2, 4, 8);
    g3->AddGrowableCol(1);
    add_number(p, g3, _L("Trigger repeatability, 1σ (mm)"), m_trigger, 0.001, 0.2, 0.001, 3, [this] { refresh_budget(); });
    s->Add(g3, 0, wxEXPAND | wxALL, 8);
    m_budget_text = new wxStaticText(p, wxID_ANY, "");
    s->Add(m_budget_text, 0, wxALL, 8);
    p->SetSizer(s);
    return p;
}

// ---- Modes and tools ----------------------------------------------------------------------------

void MetrologyPanel::set_mode(Mode m)
{
    m_mode = m;
    m_pages->ChangeSelection(size_t(m));
    for (int i = 0; i < ModeCount; ++i) {
        m_mode_btn[size_t(i)]->SetBackgroundColour(i == m ? kActive : kStrip);
        m_mode_btn[size_t(i)]->SetForegroundColour(kBarText);
    }
    const Tool t = m == ModeInspect ? ToolLine : (m == ModeTrace || m == ModeReverse) ? ToolBox : ToolSelect;
    set_tool(t);
    if (m == ModeReverse) refresh_re();
    if (m == ModeProbe) refresh_budget();
    Layout();
}

void MetrologyPanel::set_tool(Tool t)
{
    m_tool = t;
    for (int i = 0; i < ToolCount; ++i) {
        m_tool_btn[size_t(i)]->SetBackgroundColour(i == t ? kActive : kStrip);
        m_tool_btn[size_t(i)]->SetForegroundColour(kBarText);
        m_tool_btn[size_t(i)]->Refresh();
    }
    using DK = MetrologyView::DragKind;
    m_view->set_drag(t == ToolLine || t == ToolWidth ? DK::Line : t == ToolCircle ? DK::Circle : t == ToolBox ? DK::Box : DK::Click);
    if (t == ToolTwoPoint) m_two_points.clear();
    m_view->Refresh();
}

void MetrologyPanel::set_status(const wxString& text) { m_status->SetLabel(text); }

void MetrologyPanel::refresh_cal_status()
{
    wxString t;
    if (!m_cam.ok) t = _L("Not calibrated");
    else {
        t = wxString::Format(_L("%.3f mm/px"), m_cam.mm_per_px);
        t += m_cam.has_center ? wxString::Format(_L(" | camera %.0f mm up"), m_cam.cz) : _L(" | no camera height");
    }
    m_cal_text->SetLabel("[" + t + "]");
    m_view->set_scale(m_cam.ok ? m_cam.mm_per_px : 0.0);
}

void MetrologyPanel::update_judge()
{
    const M::Judge j = m_prog.overall();
    m_judge_box->SetBackgroundColour(j == M::Judge::OK ? kOk : j == M::Judge::NG ? kNg : kNone);
    m_judge_text->SetLabel(j == M::Judge::OK ? "OK" : j == M::Judge::NG ? "NG" : "--");
    m_judge_box->Layout();
    m_judge_box->Refresh();
}

// ---- Frames -----------------------------------------------------------------------------------------

bool MetrologyPanel::load_gray(const wxString& path, M::GrayImage& out)
{
    wxImage img;
    if (!img.LoadFile(path) || !img.IsOk()) return false;
    out = M::GrayImage(img.GetWidth(), img.GetHeight());
    const unsigned char* rgb = img.GetData();
    for (size_t i = 0; i < out.px.size(); ++i) out.px[i] = 0.299f * rgb[3 * i] + 0.587f * rgb[3 * i + 1] + 0.114f * rgb[3 * i + 2];
    return true;
}

void MetrologyPanel::capture()
{
    if (!simulator()) {
        open_image(false);
        return;
    }
    wxBusyCursor busy;
    M::GrayImage on = M::sim_render(m_scene, m_simcam, m_sim_offset, true, m_seed++);
    if (m_light_pair->GetValue()) on = M::subtract(on, M::sim_render(m_scene, m_simcam, m_sim_offset, false, m_seed++));
    set_frame(std::move(on));
    set_status(wxString::Format(_L("Simulator frame: %s, bed raised %.1f mm."), wxString::FromUTF8(m_scene.name), m_sim_offset));
}

void MetrologyPanel::open_image(bool light_off)
{
    wxFileDialog dlg(this, light_off ? _L("Open the light-off frame") : _L("Open a camera frame"), "", "",
                     _L("Images") + " (*.png;*.jpg;*.jpeg;*.bmp)|*.png;*.jpg;*.jpeg;*.bmp", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dlg.ShowModal() != wxID_OK) return;
    M::GrayImage img;
    if (!load_gray(dlg.GetPath(), img)) {
        set_status(_L("That image could not be read."));
        return;
    }
    if (light_off) {
        m_light_off = std::move(img);
        set_status(_L("Light-off frame loaded. It is subtracted from the next frames while Light pair is on."));
        return;
    }
    if (m_light_pair->GetValue() && m_light_off.w == img.w && m_light_off.h == img.h) img = M::subtract(img, m_light_off);
    set_frame(std::move(img));
    set_status(dlg.GetFilename());
}

void MetrologyPanel::set_frame(M::GrayImage img)
{
    m_frame = std::move(img);
    m_view->set_image(m_frame);
    if (!m_prog.features.empty()) run_program();
}

// ---- Inspect ----------------------------------------------------------------------------------------

int MetrologyPanel::selected_feature() const
{
    const long row = m_features->GetNextItem(-1, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);
    return (row >= 0 && size_t(row) < m_prog.features.size()) ? m_prog.features[size_t(row)].id : -1;
}

void MetrologyPanel::add_feature_from_drag(const P2& a, const P2& b)
{
    M::Feature f;
    f.kind = m_tool == ToolCircle ? M::FeatureKind::Circle : m_tool == ToolWidth ? M::FeatureKind::Width : M::FeatureKind::Line;
    const P2 d = b - a;
    if (f.kind == M::FeatureKind::Circle) {
        const double r = std::max(3.0, d.norm());
        f.center       = a;
        f.r_in         = std::max(1.0, 0.6 * r);
        f.r_out        = 1.4 * r;
        f.scans        = std::max(8, int(m_scans) * 2);
    } else {
        f.rect.center = (a + b) * 0.5;
        f.rect.angle  = std::atan2(d.y, d.x);
        f.rect.length = d.norm();
        f.rect.width  = m_roi_width;
        f.scans       = int(m_scans);
    }
    f.edge.min_contrast = m_contrast;
    f.edge.polarity     = M::Polarity(std::max(0, m_polarity->GetSelection()));
    if (f.kind == M::FeatureKind::Width) {
        f.edge.pick = M::EdgePick::First;
    }
    f.plane_z = m_feature_z;
    m_prog.add(f);
    run_program();
    m_features->SetItemState(long(m_prog.features.size()) - 1, wxLIST_STATE_SELECTED, wxLIST_STATE_SELECTED);
}

void MetrologyPanel::add_derived(M::FeatureKind kind)
{
    std::vector<int> ids;
    for (long row = m_features->GetNextItem(-1, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED); row >= 0; row = m_features->GetNextItem(row, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED))
        if (size_t(row) < m_prog.features.size()) ids.push_back(m_prog.features[size_t(row)].id);
    if (ids.size() != 2) {
        set_status(_L("Select exactly two features in the list (Ctrl+click)."));
        return;
    }
    M::Feature f;
    f.kind  = kind;
    f.ref_a = ids[0];
    f.ref_b = ids[1];
    m_prog.add(f);
    run_program();
}

void MetrologyPanel::run_program()
{
    if (m_frame.empty()) return;
    M::CameraModel cam = m_cam;
    if (!cam.ok) {
        // Uncalibrated: results come out in pixels.
        cam.ok        = true;
        cam.mm_per_px = 1;
    }
    m_prog.part_bright = m_part_bright;
    m_prog.run(m_frame, cam);
    refresh_feature_list();
    update_judge();
    if (!m_cam.ok && !m_prog.features.empty()) set_status(_L("Not calibrated: results are in pixels."));
    m_view->Refresh();
}

void MetrologyPanel::refresh_feature_list()
{
    const long sel = m_features->GetNextItem(-1, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);
    m_features->DeleteAllItems();
    for (size_t i = 0; i < m_prog.features.size(); ++i) {
        const M::Feature& f   = m_prog.features[i];
        const long        row = m_features->InsertItem(long(i), wxString::Format("%d", f.id));
        m_features->SetItem(row, 1, wxString::FromUTF8(f.name));
        const wxString unit = (!m_cam.ok && std::string(f.unit()) == "mm") ? wxString("px") : wxString::FromUTF8(f.unit());
        m_features->SetItem(row, 2, f.ok ? num(f.value) + " " + unit : _L("failed"));
        if (f.tol.enabled) {
            m_features->SetItem(row, 3, num(f.tol.nominal));
            m_features->SetItem(row, 4, "+" + num(f.tol.plus) + "/-" + num(f.tol.minus));
            if (f.ok) m_features->SetItem(row, 5, num(f.value - f.tol.nominal));
        }
        m_features->SetItem(row, 6, f.verdict == M::Judge::OK ? "OK" : f.verdict == M::Judge::NG ? "NG" : "");
        if (f.verdict == M::Judge::NG || !f.ok) m_features->SetItemTextColour(row, kNg);
        else if (f.verdict == M::Judge::OK) m_features->SetItemTextColour(row, kOk);
    }
    if (sel >= 0 && sel < m_features->GetItemCount()) m_features->SetItemState(sel, wxLIST_STATE_SELECTED, wxLIST_STATE_SELECTED);
}

void MetrologyPanel::edit_tolerance()
{
    M::Feature* f = m_prog.find(selected_feature());
    if (!f) {
        set_status(_L("Select a feature first."));
        return;
    }
    wxDialog dlg(this, wxID_ANY, wxString::Format(_L("Tolerance: %s"), wxString::FromUTF8(f->name)));
    auto*    s  = new wxBoxSizer(wxVERTICAL);
    auto*    en = new wxCheckBox(&dlg, wxID_ANY, _L("Judge this feature"));
    en->SetValue(true);
    s->Add(en, 0, wxALL, 10);
    auto* g = new wxFlexGridSizer(2, 6, 10);
    auto  field = [&](const wxString& text, double v) {
        g->Add(new wxStaticText(&dlg, wxID_ANY, text), 0, wxALIGN_CENTER_VERTICAL);
        auto* sp = new wxSpinCtrlDouble(&dlg, wxID_ANY, "", wxDefaultPosition, wxSize(FromDIP(120), -1), wxSP_ARROW_KEYS, -1e6, 1e6, v, 0.01);
        sp->SetDigits(4);
        g->Add(sp, 0, wxEXPAND);
        return sp;
    };
    auto* nom   = field(_L("Nominal"), f->tol.enabled ? f->tol.nominal : (f->ok ? f->value : 0.0));
    auto* plus  = field(_L("Upper tolerance (+)"), f->tol.plus);
    auto* minus = field(_L("Lower tolerance (-)"), f->tol.minus);
    s->Add(g, 0, wxALL, 10);
    s->Add(dlg.CreateButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxALL, 10);
    dlg.SetSizerAndFit(s);
    if (dlg.ShowModal() != wxID_OK) return;
    f->tol.enabled = en->GetValue();
    f->tol.nominal = nom->GetValue();
    f->tol.plus    = std::abs(plus->GetValue());
    f->tol.minus   = std::abs(minus->GetValue());
    run_program();
}

void MetrologyPanel::delete_feature()
{
    const int id = selected_feature();
    if (id < 0) return;
    m_prog.remove(id);
    run_program();
    refresh_feature_list();
    update_judge();
    m_view->Refresh();
}

void MetrologyPanel::export_csv()
{
    wxFileDialog dlg(this, _L("Export inspection results"), "", "inspection.csv", "CSV (*.csv)|*.csv", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (dlg.ShowModal() != wxID_OK) return;
    set_status(write_text(dlg.GetPath(), m_prog.to_csv()) ? _L("Results exported.") : _L("The file could not be written."));
}

// ---- Calibrate --------------------------------------------------------------------------------------

void MetrologyPanel::detect_mat()
{
    if (m_frame.empty()) {
        set_status(_L("Capture a frame of the mat first."));
        return;
    }
    wxBusyCursor busy;
    m_cal = M::calibrate_mat(m_frame, m_mat);
    if (!m_cal.ok) {
        m_cal_result->SetLabel(wxString::FromUTF8(m_cal.error));
        set_status(wxString::FromUTF8(m_cal.error));
        return;
    }
    m_cam             = M::camera_from(m_cal);
    m_cam.ref_bed_z   = 0;
    m_raised.clear();
    m_raised_offsets.clear();
    m_raised_list->DeleteAllItems();
    m_cal_result->SetLabel(wxString::Format(_L("%d of %d dots, residual %.3f mm RMS, %.3f mm per pixel at the mat centre."), m_cal.matched,
                                            m_cal.expected, m_cal.rms_mm, m_cal.mm_per_px));
    refresh_cal_status();
    save_settings();
    run_program();
    m_view->Refresh();
}

void MetrologyPanel::add_raised_frame()
{
    if (!m_cam.ok) {
        set_status(_L("Detect the mat at the reference bed height first."));
        return;
    }
    M::GrayImage img;
    if (simulator()) {
        wxBusyCursor busy;
        img = M::sim_render(m_scene, m_simcam, m_raise_mm, true, m_seed++);
        if (m_light_pair->GetValue()) img = M::subtract(img, M::sim_render(m_scene, m_simcam, m_raise_mm, false, m_seed++));
    } else {
        wxFileDialog dlg(this, _L("Open a frame of the mat with the bed raised"), "", "", _L("Images") + " (*.png;*.jpg;*.jpeg;*.bmp)|*.png;*.jpg;*.jpeg;*.bmp",
                         wxFD_OPEN | wxFD_FILE_MUST_EXIST);
        if (dlg.ShowModal() != wxID_OK || !load_gray(dlg.GetPath(), img)) return;
        if (m_light_pair->GetValue() && m_light_off.w == img.w && m_light_off.h == img.h) img = M::subtract(img, m_light_off);
    }
    const M::PlateCalibration c = M::calibrate_mat(img, m_mat);
    if (!c.ok) {
        set_status(wxString::FromUTF8(c.error));
        return;
    }
    m_raised.push_back(c);
    m_raised_offsets.push_back(m_raise_mm);
    const long row = m_raised_list->InsertItem(m_raised_list->GetItemCount(), num(m_raise_mm, 2));
    m_raised_list->SetItem(row, 1, wxString::Format("%d", c.matched));
    m_raised_list->SetItem(row, 2, num(c.rms_mm));
    set_status(_L("Raised frame added. Solve the camera height when you have one or more."));
}

void MetrologyPanel::solve_camera()
{
    std::string err;
    if (!M::calibrate_camera_center(m_cam, m_raised_offsets, m_raised, &err)) {
        set_status(wxString::FromUTF8(err));
        return;
    }
    m_cal_result->SetLabel(m_cal_result->GetLabel() + "\n" +
                           wxString::Format(_L("Camera %.1f mm above the mat, over X %.1f Y %.1f mm."), m_cam.cz, m_cam.nadir.x, m_cam.nadir.y));
    refresh_cal_status();
    save_settings();
}

void MetrologyPanel::auto_calibrate_simulator()
{
    if (!simulator()) {
        set_status(_L("Auto-calibration only runs in the simulator."));
        return;
    }
    wxBusyCursor busy;
    set_status(_L("Calibrating the simulated camera…"));
    const double keep = m_sim_offset;
    m_sim_offset      = 0;
    capture();
    detect_mat();
    if (m_cam.ok) {
        add_raised_frame();
        solve_camera();
    }
    m_sim_offset = keep;
}

void MetrologyPanel::two_point_click(const P2& p)
{
    m_two_points.push_back(p);
    if (m_two_points.size() == 2) {
        m_cal = M::calibrate_two_points(m_two_points[0], m_two_points[1], m_two_point_mm);
        if (m_cal.ok) {
            m_cam = M::camera_from(m_cal);
            set_status(wxString::Format(_L("Scale set: %.4f mm per pixel (flat, straight-on only)."), m_cal.mm_per_px));
            refresh_cal_status();
            run_program();
        } else
            set_status(wxString::FromUTF8(m_cal.error));
        m_two_points.clear();
    } else
        set_status(_L("Click the second point."));
    m_view->Refresh();
}

void MetrologyPanel::save_settings() const
{
    using nlohmann::json;
    json j;
    j["mat"] = {{"ring_w", m_mat.ring_w}, {"ring_h", m_mat.ring_h}, {"pitch", m_mat.pitch}, {"dot_d", m_mat.dot_d},
                {"marker_d", m_mat.marker_d}, {"thickness", m_mat.thickness}, {"bright_dots", m_mat.bright_dots}};
    j["camera"] = {{"ok", m_cam.ok}, {"mm_to_px", std::vector<double>(m_cam.mm_to_px.m.begin(), m_cam.mm_to_px.m.end())},
                   {"mm_per_px", m_cam.mm_per_px}, {"has_center", m_cam.has_center}, {"nadir", {m_cam.nadir.x, m_cam.nadir.y}},
                   {"cz", m_cam.cz}};
    j["probe"] = {{"mat_x", m_probe.mat_x}, {"mat_y", m_probe.mat_y}, {"mat_rot", m_probe.mat_rot_deg}, {"hold", m_probe.hold_s},
                  {"feed", m_probe.probe_feed}, {"clearance", m_probe.clearance}, {"ref_x", m_mat_ref.x}, {"ref_y", m_mat_ref.y}};
    j["reverse"] = {{"grid", m_re_set.grid}, {"tip", m_re_set.tip_radius}, {"level_tol", m_re_set.level_tol},
                    {"target", m_re_set.target_res}, {"touches", m_re_touches}, {"anchors", m_re_anchors}};
    j["source"] = int(m_source);
    try {
        boost::nowide::ofstream f(settings_file());
        f << j.dump(2);
    } catch (...) {}
}

void MetrologyPanel::load_settings()
{
    using nlohmann::json;
    try {
        boost::nowide::ifstream f(settings_file());
        if (!f) return;
        json j;
        f >> j;
        auto get = [](const json& o, const char* k, auto& v) {
            if (o.contains(k)) v = o[k].get<std::decay_t<decltype(v)>>();
        };
        if (j.contains("mat")) {
            const json& m = j["mat"];
            get(m, "ring_w", m_mat.ring_w);
            get(m, "ring_h", m_mat.ring_h);
            get(m, "pitch", m_mat.pitch);
            get(m, "dot_d", m_mat.dot_d);
            get(m, "marker_d", m_mat.marker_d);
            get(m, "thickness", m_mat.thickness);
            get(m, "bright_dots", m_mat.bright_dots);
        }
        if (j.contains("camera")) {
            const json& c = j["camera"];
            get(c, "ok", m_cam.ok);
            if (c.contains("mm_to_px") && c["mm_to_px"].size() == 9)
                for (size_t i = 0; i < 9; ++i) m_cam.mm_to_px.m[i] = c["mm_to_px"][i].get<double>();
            m_cam.px_to_mm = m_cam.mm_to_px.inverse();
            get(c, "mm_per_px", m_cam.mm_per_px);
            get(c, "has_center", m_cam.has_center);
            if (c.contains("nadir") && c["nadir"].size() == 2) m_cam.nadir = {c["nadir"][0].get<double>(), c["nadir"][1].get<double>()};
            get(c, "cz", m_cam.cz);
        }
        if (j.contains("probe")) {
            const json& p = j["probe"];
            get(p, "mat_x", m_probe.mat_x);
            get(p, "mat_y", m_probe.mat_y);
            get(p, "mat_rot", m_probe.mat_rot_deg);
            get(p, "hold", m_probe.hold_s);
            get(p, "feed", m_probe.probe_feed);
            get(p, "clearance", m_probe.clearance);
            get(p, "ref_x", m_mat_ref.x);
            get(p, "ref_y", m_mat_ref.y);
        }
        if (j.contains("reverse")) {
            const json& r = j["reverse"];
            get(r, "grid", m_re_set.grid);
            get(r, "tip", m_re_set.tip_radius);
            get(r, "level_tol", m_re_set.level_tol);
            get(r, "target", m_re_set.target_res);
            get(r, "touches", m_re_touches);
            get(r, "anchors", m_re_anchors);
        }
        int src = 0;
        get(j, "source", src);
        m_source = src == 1 ? SourceFiles : SourceSimulator;
    } catch (...) {
        // A damaged settings file just means defaults.
    }
}

// ---- Trace ------------------------------------------------------------------------------------------

void MetrologyPanel::set_box(const P2& a_px, const P2& b_px)
{
    if (!m_cam.ok) {
        set_status(_L("Calibrate first: the box is placed on the mat plane."));
        return;
    }
    const P2 c[4] = {a_px, {b_px.x, a_px.y}, b_px, {a_px.x, b_px.y}};
    m_box_lo      = {1e9, 1e9};
    m_box_hi      = {-1e9, -1e9};
    for (const P2& p : c) {
        const P2 mm = m_cam.to_plane(p, 0);
        m_box_lo    = {std::min(m_box_lo.x, mm.x), std::min(m_box_lo.y, mm.y)};
        m_box_hi    = {std::max(m_box_hi.x, mm.x), std::max(m_box_hi.y, mm.y)};
    }
    m_has_box = true;
    set_status(wxString::Format(_L("Box %.1f x %.1f mm."), m_box_hi.x - m_box_lo.x, m_box_hi.y - m_box_lo.y));
    if (m_mode == ModeTrace) do_trace();
    m_view->Refresh();
}

std::vector<M::Region> MetrologyPanel::trace_box() const
{
    if (!m_has_box || !m_cam.ok || m_frame.empty()) return {};
    return M::trace_silhouette(m_frame, m_cam, m_box_lo, m_box_hi, m_trace_res, -1, m_part_bright, m_trace_simplify, 1.0);
}

void MetrologyPanel::do_trace()
{
    if (!m_has_box) {
        set_status(_L("Pick Box and drag around the part first."));
        return;
    }
    wxBusyCursor busy;
    m_trace = trace_box();
    double area = 0, perim = 0;
    size_t holes = 0;
    for (const M::Region& r : m_trace) {
        area += M::signed_area(r.outer);
        for (size_t k = 0; k < r.outer.size(); ++k) perim += (r.outer[(k + 1) % r.outer.size()] - r.outer[k]).norm();
        for (const M::Polygon2& h : r.holes) area += M::signed_area(h);
        holes += r.holes.size();
    }
    P2 lo, hi;
    M::bounding_box(m_trace, lo, hi);
    m_trace_info->SetLabel(m_trace.empty() ? _L("No outline found in the box.")
                                           : wxString::Format(_L("%d outline(s), %d hole(s)\nArea %.2f mm², perimeter %.2f mm\nSize %.3f x %.3f mm"),
                                                              int(m_trace.size()), int(holes), area, perim, hi.x - lo.x, hi.y - lo.y));
    m_view->Refresh();
}

void MetrologyPanel::export_svg(const std::vector<M::Region>& regions)
{
    if (regions.empty()) {
        set_status(_L("Nothing to export yet."));
        return;
    }
    wxFileDialog dlg(this, _L("Export outline"), "", "outline.svg", "SVG (*.svg)|*.svg", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (dlg.ShowModal() != wxID_OK) return;
    set_status(write_text(dlg.GetPath(), M::regions_svg(regions)) ? _L("Outline exported.") : _L("The file could not be written."));
}

void MetrologyPanel::send_extrusion()
{
    if (m_trace.empty()) {
        set_status(_L("Trace an outline first."));
        return;
    }
    TriangleMesh mesh(extrude_regions(m_trace, 0, m_extrude_h));
    if (mesh.empty()) {
        set_status(_L("The outline could not be turned into a solid."));
        return;
    }
    wxGetApp().obj_list()->load_mesh_object(mesh, _L("Traced outline"));
    set_status(_L("The extruded outline is on the plate (Prepare)."));
}

// ---- Reverse engineering ----------------------------------------------------------------------------

void MetrologyPanel::re_start()
{
    if (!m_cam.ok) {
        set_status(_L("Calibrate the mat first."));
        return;
    }
    if (!m_has_box) {
        set_status(_L("Pick Box and drag around the part first."));
        return;
    }
    wxBusyCursor busy;
    auto silhouette = trace_box();
    if (silhouette.empty()) {
        set_status(_L("No outline was found in the box."));
        return;
    }
    m_re_set.touches_per_edge = std::max(1, int(m_re_touches));
    m_re_set.anchors          = std::max(0, int(m_re_anchors));
    m_re.begin(std::move(silhouette), m_cam, m_re_set);
    m_re_built = false;
    m_re.next_batch();
    if (!m_cam.has_center) set_status(_L("No camera height: the outline will not be corrected for parallax."));
    refresh_re();
}

void MetrologyPanel::re_run_batch_simulator()
{
    if (!simulator()) {
        set_status(_L("Switch the source to Simulator, or run the batch on the printer."));
        return;
    }
    if (m_re.pending().empty()) {
        set_status(_L("No batch is waiting. Start a session first."));
        return;
    }
    std::mt19937                     rng(m_seed++);
    std::normal_distribution<double> noise(0.0, std::max(0.0, m_sim_noise));
    std::vector<M::ReResult>         res;
    for (const M::ReTouch& t : m_re.pending()) res.push_back({t.id, M::sim_touch(m_scene, t.xy, m_re_set.tip_radius) + noise(rng)});
    m_re.submit(res);
    m_re.next_batch();
    if (m_re.stage() == M::ReSession::Stage::Done) re_build();
    refresh_re();
}

void MetrologyPanel::re_save_batch_gcode()
{
    const auto& batch = m_re.pending();
    if (batch.empty()) {
        set_status(_L("No batch is waiting. Start a session first."));
        return;
    }
    wxFileDialog dlg(this, _L("Save the touch batch"), "", wxString::Format("touch_batch_%d.gcode", m_re.rounds() + 1),
                     "G-code (*.gcode)|*.gcode", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (dlg.ShowModal() != wxID_OK) return;
    std::vector<M::ProbeTouch> touches;
    std::ostringstream         csv;
    csv << "id,x_mm,y_mm,label,height_mm\n";
    for (const M::ReTouch& t : batch) {
        touches.push_back({t.xy, t.label});
        csv << t.id << ',' << t.xy.x << ',' << t.xy.y << ',' << t.label << ",\n";
    }
    m_probe.mat_thickness = m_mat.thickness;
    const bool ok = write_text(dlg.GetPath(), M::probe_gcode(touches, m_probe, 0, m_max_height, m_mat_ref)) &&
                    write_text(dlg.GetPath() + ".csv", csv.str());
    set_status(ok ? _L("Batch saved, with a CSV of the touches beside it. The first touch is the mat reference.")
                  : _L("The file could not be written."));
}

void MetrologyPanel::re_load_heights_csv()
{
    if (m_re.pending().empty()) {
        set_status(_L("No batch is waiting."));
        return;
    }
    wxFileDialog dlg(this, _L("Heights for the batch (id, ..., height_mm)"), "", "", "CSV (*.csv)|*.csv", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dlg.ShowModal() != wxID_OK) return;
    boost::nowide::ifstream f(dlg.GetPath().ToUTF8().data());
    std::string             line;
    std::vector<M::ReResult> res;
    while (std::getline(f, line)) {
        // The first and the last comma-separated fields: the id and the height.
        const size_t first = line.find(','), last = line.rfind(',');
        if (first == std::string::npos || last == std::string::npos) continue;
        try {
            const int    id = std::stoi(line.substr(0, first));
            const double z  = std::stod(line.substr(last + 1));
            res.push_back({id, z});
        } catch (...) {
        }
    }
    if (res.size() < m_re.pending().size()) {
        set_status(wxString::Format(_L("The file has %d heights for %d touches."), int(res.size()), int(m_re.pending().size())));
        return;
    }
    m_re.submit(res);
    m_re.next_batch();
    refresh_re();
}

bool MetrologyPanel::offsets_from_folder(const wxString& dir, size_t expected, std::vector<double>& offsets, wxString& error)
{
    offsets.clear();
    if (!m_cam.has_center) {
        error = _L("Reading heights needs the camera height: calibrate it first.");
        return false;
    }
    std::vector<std::string> files;
    for (const auto& e : fs::directory_iterator(fs::path(dir.ToUTF8().data()))) {
        std::string ext = e.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp") files.push_back(e.path().string());
    }
    std::sort(files.begin(), files.end());
    if (files.empty()) {
        error = _L("The folder has no images.");
        return false;
    }
    std::vector<M::GrayImage> frames(files.size());
    for (size_t i = 0; i < files.size(); ++i)
        if (!load_gray(wxString::FromUTF8(files[i]), frames[i])) {
            error = wxString::Format(_L("%s could not be read."), wxString::FromUTF8(files[i]));
            return false;
        }
    // One image per hold, or a sequence in which the holds are the still stretches.
    std::vector<std::vector<size_t>> stills;
    if (expected == 0 || frames.size() != expected) {
        std::vector<double> motion;
        for (size_t i = 0; i + 1 < frames.size(); ++i) motion.push_back(M::frame_motion(frames[i], frames[i + 1]));
        std::vector<double> sorted = motion;
        std::sort(sorted.begin(), sorted.end());
        const double thr = 2.0 + (sorted.empty() ? 0.0 : 0.25 * sorted[sorted.size() / 2]);
        for (const M::StillSegment& seg : M::find_stills(motion, thr, 2)) {
            std::vector<size_t> idx;
            for (int k = seg.first; k <= seg.last; ++k) idx.push_back(size_t(k));
            stills.push_back(idx);
        }
    }
    if (stills.empty() || (expected > 0 && stills.size() != expected)) {
        stills.clear();
        for (size_t i = 0; i < frames.size(); ++i) stills.push_back({i});
    }
    if (expected > 0 && stills.size() != expected) {
        error = wxString::Format(_L("Found %d holds but the batch has %d touches (including the mat reference)."), int(stills.size()), int(expected));
        return false;
    }
    for (const auto& idx : stills) {
        std::vector<M::GrayImage> group;
        for (size_t k : idx) group.push_back(frames[k]);
        const M::PlateCalibration c = M::calibrate_mat(M::average(group), m_mat);
        double off = 0;
        if (!c.ok || !M::plane_offset(m_cam, c, off)) {
            error = _L("The mat could not be read in one of the holds.");
            return false;
        }
        offsets.push_back(off);
    }
    return true;
}

void MetrologyPanel::re_read_heights_from_frames()
{
    const auto& batch = m_re.pending();
    if (batch.empty()) {
        set_status(_L("No batch is waiting."));
        return;
    }
    wxDirDialog dlg(this, _L("Folder of camera frames recorded while the batch ran"));
    if (dlg.ShowModal() != wxID_OK) return;
    wxBusyCursor        busy;
    std::vector<double> off;
    wxString            err;
    if (!offsets_from_folder(dlg.GetPath(), batch.size() + 1, off, err)) {
        set_status(err);
        return;
    }
    // The first hold is the mat reference touch. A taller surface stops the bed lower, so it is
    // raised less than at the mat.
    std::vector<M::ReResult> res;
    for (size_t i = 0; i < batch.size(); ++i) res.push_back({batch[i].id, off[0] - off[i + 1]});
    m_re.submit(res);
    m_re.next_batch();
    refresh_re();
}

void MetrologyPanel::re_build()
{
    if (m_re.stage() == M::ReSession::Stage::Idle) {
        set_status(_L("Start a session first."));
        return;
    }
    wxBusyCursor busy;
    m_re_model = m_re.build();
    m_re_built = true;
    wxString msg = wxString::Format(_L("Model: %d level(s)."), int(m_re_model.levels.size()));
    if (m_re_model.anchors > 0) msg += wxString::Format(_L(" Camera edge bias %+.3f mm removed (%d anchors)."), m_re_model.camera_bias, m_re_model.anchors);
    for (const std::string& w : m_re_model.warnings) msg += " " + wxString::FromUTF8(w);
    set_status(msg);
    refresh_re();
}

void MetrologyPanel::re_send_to_plate()
{
    if (!m_re_built || m_re_model.levels.empty()) {
        set_status(_L("Build the model first."));
        return;
    }
    TriangleMesh mesh = model_mesh(m_re_model);
    if (mesh.empty()) {
        set_status(_L("The model could not be turned into a solid."));
        return;
    }
    wxGetApp().obj_list()->load_mesh_object(mesh, _L("Reverse engineered"));
    set_status(_L("The model is on the plate (Prepare)."));
}

void MetrologyPanel::re_export_stl()
{
    if (!m_re_built || m_re_model.levels.empty()) {
        set_status(_L("Build the model first."));
        return;
    }
    wxFileDialog dlg(this, _L("Export the model"), "", "reverse_engineered.stl", "STL (*.stl)|*.stl", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (dlg.ShowModal() != wxID_OK) return;
    const TriangleMesh mesh = model_mesh(m_re_model);
    set_status(mesh.write_binary(dlg.GetPath().ToUTF8().data()) ? _L("Model exported.") : _L("The file could not be written."));
}

void MetrologyPanel::refresh_re()
{
    if (!m_re_state) return;
    const char* stage = "";
    switch (m_re.stage()) {
    case M::ReSession::Stage::Idle: stage = "idle"; break;
    case M::ReSession::Stage::Survey: stage = "survey"; break;
    case M::ReSession::Stage::Grid: stage = "grid"; break;
    case M::ReSession::Stage::Edges: stage = "steps"; break;
    case M::ReSession::Stage::Done: stage = "done"; break;
    }
    const int    remaining = m_re.estimate_remaining();
    const double secs      = (remaining + int(m_re.pending().size())) * (m_probe.hold_s + 5.0);
    m_re_state->SetLabel(wxString::Format(_L("Stage: %s | touches done %d | rounds %d | batch %d | about %d more (%.0f min on a P1P)"),
                                          stage, m_re.touches_done(), m_re.rounds(), int(m_re.pending().size()), remaining, secs / 60));
    m_re_batch->DeleteAllItems();
    long row = 0;
    for (const M::ReTouch& t : m_re.pending()) {
        if (row >= 500) break;   // the list is a preview
        m_re_batch->InsertItem(row, wxString::Format("%d", t.id));
        m_re_batch->SetItem(row, 1, num(t.xy.x, 2));
        m_re_batch->SetItem(row, 2, num(t.xy.y, 2));
        m_re_batch->SetItem(row, 3, wxString::FromUTF8(t.label));
        ++row;
    }
    m_re_levels->DeleteAllItems();
    if (m_re_built)
        for (size_t k = 0; k < m_re_model.levels.size(); ++k) {
            const M::ReLevel& l = m_re_model.levels[k];
            double            a = 0;
            for (const M::Region& r : l.regions) {
                a += M::signed_area(r.outer);
                for (const M::Polygon2& h : r.holes) a += M::signed_area(h);
            }
            const long rr = m_re_levels->InsertItem(long(k), wxString::Format("%d", int(k + 1)));
            m_re_levels->SetItem(rr, 1, num(l.z));
            m_re_levels->SetItem(rr, 2, num(l.stddev));
            m_re_levels->SetItem(rr, 3, wxString::Format("%d", l.samples));
            m_re_levels->SetItem(rr, 4, num(a, 1));
            m_re_levels->SetItemTextColour(rr, kLevelColours[k % 8].ChangeLightness(70));
        }
    m_view->Refresh();
}

// ---- Probe ------------------------------------------------------------------------------------------

void MetrologyPanel::save_repeatability_gcode()
{
    wxFileDialog dlg(this, _L("Save the repeatability test"), "", "touch_repeatability.gcode", "G-code (*.gcode)|*.gcode",
                     wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (dlg.ShowModal() != wxID_OK) return;
    const std::string g = M::repeatability_gcode({m_repeat_x, m_repeat_y}, int(m_repeat_n), m_probe);
    set_status(write_text(dlg.GetPath(), g) ? _L("Test saved. Record the camera while it runs.") : _L("The file could not be written."));
}

void MetrologyPanel::analyze_frames()
{
    wxDirDialog dlg(this, _L("Folder of camera frames from the repeatability test"));
    if (dlg.ShowModal() != wxID_OK) return;
    wxBusyCursor        busy;
    std::vector<double> off;
    wxString            err;
    if (!offsets_from_folder(dlg.GetPath(), 0, off, err)) {
        m_probe_result->SetLabel(err);
        return;
    }
    const M::RepeatStats st = M::repeat_stats(off);
    m_probe_result->SetLabel(wxString::Format(_L("%d holds: spread %.4f mm, 1σ %.4f mm.\n%s"), st.n, st.range, st.stddev,
                                              st.n < 2           ? _L("Too few holds to judge.")
                                              : st.range < 0.03 ? _L("Under 0.03 mm: the touch probe idea works on this printer.")
                                                                 : _L("Over 0.03 mm: heights will be coarse; edges may still work.")));
    m_trigger = st.stddev;
    refresh_budget();
}

void MetrologyPanel::refresh_budget()
{
    if (!m_budget_text) return;
    M::BudgetInputs in;
    if (m_cam.ok) in.mm_per_px = m_cam.mm_per_px;
    in.trigger = m_trigger;
    const M::BudgetResult r = M::budget(in);
    m_budget_text->SetLabel(wxString::Format(_L("At %.3f mm/px (±2σ):\nheight per touch ±%.3f mm\nprobe edge ±%.3f mm\ncamera edge ±%.3f mm"),
                                             in.mm_per_px, 2 * r.height, 2 * r.edge, 2 * r.camera_edge));
}

// ---- Overlay ----------------------------------------------------------------------------------------

void MetrologyPanel::draw_overlay(wxGraphicsContext& gc)
{
    if (m_frame.empty()) return;
    auto W   = [this](const P2& img) { return m_view->to_win(img); };
    auto seg = [&](const P2& a, const P2& b) {
        const auto p = W(a), q = W(b);
        gc.StrokeLine(p.m_x, p.m_y, q.m_x, q.m_y);
    };
    auto cross = [&](const P2& a, double r) {
        const auto p = W(a);
        gc.StrokeLine(p.m_x - r, p.m_y, p.m_x + r, p.m_y);
        gc.StrokeLine(p.m_x, p.m_y - r, p.m_x, p.m_y + r);
    };
    // A plate-frame loop at height z as an image path.
    auto loop_path = [&](const M::Polygon2& loop, double z, wxGraphicsPath& path) {
        for (size_t k = 0; k < loop.size(); ++k) {
            const auto p = W(mm_to_px(loop[k], z));
            if (k == 0) path.MoveToPoint(p);
            else path.AddLineToPoint(p);
        }
        path.CloseSubpath();
    };
    auto regions_path = [&](const std::vector<M::Region>& regions, double z) {
        wxGraphicsPath path = gc.CreatePath();
        for (const M::Region& r : regions) {
            loop_path(r.outer, z, path);
            for (const M::Polygon2& h : r.holes) loop_path(h, z, path);
        }
        return path;
    };
    auto tag = [&](const P2& at_img, const wxString& text, const wxColour& c) {
        const auto p = W(at_img);
        gc.SetFont(GetFont().Bold(), c);
        double w = 0, h = 0;
        gc.GetTextExtent(text, &w, &h);
        gc.SetPen(*wxTRANSPARENT_PEN);
        gc.SetBrush(wxBrush(wxColour(0, 0, 0, 170)));
        gc.DrawRectangle(p.m_x + 6, p.m_y - h - 4, w + 8, h + 4);
        gc.DrawText(text, p.m_x + 10, p.m_y - h - 2);
    };

    // Calibration dots.
    if (m_mode == ModeCalibrate && m_cal.ok) {
        gc.SetPen(wxPen(wxColour(60, 230, 90), 1));
        for (const P2& d : m_cal.dot_px) cross(d, 6);
        if (!m_cal.dot_px.empty()) tag(m_cal.dot_px.front(), _L("origin"), wxColour(60, 230, 90));
    }
    if (m_tool == ToolTwoPoint) {
        gc.SetPen(wxPen(wxColour(255, 210, 0), 2));
        for (const P2& p : m_two_points) cross(p, 8);
    }

    // Inspection features.
    if (m_mode == ModeInspect) {
        const int sel = selected_feature();
        for (const M::Feature& f : m_prog.features) {
            const bool    is_sel = f.id == sel;
            const wxColour roi   = is_sel ? wxColour(255, 210, 0) : wxColour(0, 200, 255);
            gc.SetPen(wxPen(roi, is_sel ? 2 : 1, wxPENSTYLE_SHORT_DASH));
            gc.SetBrush(*wxTRANSPARENT_BRUSH);
            P2 label_at;
            if (f.kind == M::FeatureKind::Circle) {
                for (double r : {f.r_in, f.r_out}) {
                    const auto c = W(f.center);
                    gc.DrawEllipse(c.m_x - r * m_view->zoom(), c.m_y - r * m_view->zoom(), 2 * r * m_view->zoom(), 2 * r * m_view->zoom());
                }
                label_at = f.center + P2{f.r_out, 0};
            } else if (f.kind == M::FeatureKind::Line || f.kind == M::FeatureKind::Width) {
                const auto c = f.rect.corners();
                for (int k = 0; k < 4; ++k) seg(c[size_t(k)], c[size_t((k + 1) % 4)]);
                gc.SetPen(wxPen(roi, 1));
                seg(f.rect.center - f.rect.dir() * (f.rect.length * 0.5), f.rect.center + f.rect.dir() * (f.rect.length * 0.5));
                label_at = c[2];
            } else {
                const M::Feature* a = m_prog.find(f.ref_a);
                const M::Feature* b = m_prog.find(f.ref_b);
                if (a && b) {
                    const P2 pa = a->kind == M::FeatureKind::Circle ? a->center : a->rect.center;
                    const P2 pb = b->kind == M::FeatureKind::Circle ? b->center : b->rect.center;
                    gc.SetPen(wxPen(roi, 1, wxPENSTYLE_DOT));
                    seg(pa, pb);
                    label_at = (pa + pb) * 0.5;
                }
            }
            gc.SetPen(wxPen(wxColour(255, 240, 60), 1));
            for (const P2& p : f.edge_px) cross(p, 3);
            for (const P2& p : f.edge2_px) cross(p, 3);
            if (f.ok && m_cam.ok) {
                gc.SetPen(wxPen(wxColour(60, 230, 90), 2));
                if (f.kind == M::FeatureKind::Circle) {
                    wxGraphicsPath path = gc.CreatePath();
                    M::Polygon2    circ;
                    for (int k = 0; k < 96; ++k) {
                        const double t = 2 * M::kPi * k / 96;
                        circ.push_back(f.circle.c + P2{std::cos(t), std::sin(t)} * f.circle.r);
                    }
                    loop_path(circ, f.plane_z, path);
                    gc.StrokePath(path);
                } else if (f.kind == M::FeatureKind::Line || f.kind == M::FeatureKind::Width) {
                    for (const M::LineFit* lf : {&f.line, &f.line2}) {
                        if (!lf->ok) continue;
                        const double half = f.rect.width * 0.5 * m_cam.mm_per_px;
                        seg(mm_to_px(lf->p - lf->d * half, f.plane_z), mm_to_px(lf->p + lf->d * half, f.plane_z));
                        if (f.kind == M::FeatureKind::Line) break;
                    }
                }
            }
            const wxColour tc = f.verdict == M::Judge::OK ? wxColour(80, 235, 110) : (f.verdict == M::Judge::NG || !f.ok) ? wxColour(255, 90, 90)
                                                                                                                      : wxColour(235, 235, 235);
            const wxString unit = (!m_cam.ok && std::string(f.unit()) == "mm") ? wxString("px") : wxString::FromUTF8(f.unit());
            tag(label_at, wxString::Format("%d %s", f.id, f.ok ? num(f.value) + " " + unit : _L("failed")), tc);
        }
    }

    // Box, traced outline and reverse-engineering state.
    if ((m_mode == ModeTrace || m_mode == ModeReverse) && m_has_box && m_cam.ok) {
        M::Polygon2 box{m_box_lo, {m_box_hi.x, m_box_lo.y}, m_box_hi, {m_box_lo.x, m_box_hi.y}};
        wxGraphicsPath path = gc.CreatePath();
        loop_path(box, 0, path);
        gc.SetPen(wxPen(wxColour(255, 210, 0), 1, wxPENSTYLE_SHORT_DASH));
        gc.SetBrush(*wxTRANSPARENT_BRUSH);
        gc.StrokePath(path);
    }
    if (m_mode == ModeTrace && !m_trace.empty() && m_cam.ok) {
        gc.SetPen(wxPen(wxColour(0, 220, 255), 2));
        gc.StrokePath(regions_path(m_trace, 0));
    }
    if (m_mode == ModeReverse && m_cam.ok && m_re.stage() != M::ReSession::Stage::Idle) {
        gc.SetPen(wxPen(wxColour(0, 220, 255), 1, wxPENSTYLE_SHORT_DASH));
        gc.SetBrush(*wxTRANSPARENT_BRUSH);
        gc.StrokePath(regions_path(m_re.silhouette(), 0));
        if (m_re_built) {
            for (size_t k = 0; k < m_re_model.levels.size(); ++k) {
                const M::ReLevel& l = m_re_model.levels[k];
                wxColour          c = kLevelColours[k % 8];
                gc.SetBrush(wxBrush(wxColour(c.Red(), c.Green(), c.Blue(), 70)));
                gc.SetPen(wxPen(c, 2));
                wxGraphicsPath p = regions_path(l.regions, l.z);
                gc.FillPath(p, wxODDEVEN_RULE);
                gc.StrokePath(p);
            }
        }
        const std::vector<double> H = m_re.level_heights();
        for (const M::ReSession::Node& n : m_re.nodes()) {
            if (!n.active && !n.measured) continue;
            const wxColour c = n.level >= 0 ? kLevelColours[size_t(n.level) % 8] : wxColour(150, 150, 150);
            gc.SetPen(wxPen(c, 1));
            gc.SetBrush(n.measured ? wxBrush(c) : *wxTRANSPARENT_BRUSH);
            const auto p = W(mm_to_px(n.pos, n.measured ? std::max(0.0, n.z) : 0.0));
            gc.DrawEllipse(p.m_x - 3, p.m_y - 3, 6, 6);
        }
        gc.SetPen(wxPen(wxColour(255, 80, 255), 1));
        for (const M::ReSession::Crossing& c : m_re.crossings()) cross(mm_to_px(c.p, H.empty() ? 0.0 : H.back()), 3);
        gc.SetPen(wxPen(*wxWHITE, 1));
        gc.SetBrush(*wxTRANSPARENT_BRUSH);
        for (const M::ReTouch& t : m_re.pending()) {
            const auto p = W(mm_to_px(t.xy, 0));
            gc.DrawEllipse(p.m_x - 4, p.m_y - 4, 8, 8);
        }
    }
    if (m_mode == ModeProbe && m_cam.ok) {
        gc.SetPen(wxPen(wxColour(255, 210, 0), 2));
        cross(mm_to_px(m_mat_ref, 0), 8);
        tag(mm_to_px(m_mat_ref, 0), _L("mat reference touch"), wxColour(255, 210, 0));
    }
}

}} // namespace Slic3r::GUI

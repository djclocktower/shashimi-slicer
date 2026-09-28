#include "LaserDialogs.hpp"

#include "slic3r/GUI/I18N.hpp"
#include "slic3r/GUI/GUI_Utils.hpp"

#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/choice.h>
#include <wx/dcbuffer.h>
#include <wx/dialog.h>
#include <wx/filedlg.h>
#include <wx/image.h>
#include <wx/listctrl.h>
#include <wx/msgdlg.h>
#include <wx/notebook.h>
#include <wx/panel.h>
#include <wx/sizer.h>
#include <wx/slider.h>
#include <wx/spinctrl.h>
#include <wx/statbmp.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include <boost/nowide/fstream.hpp>

#include <algorithm>
#include <cmath>

namespace Slic3r { namespace GUI {

using namespace Slic3r::Laser;

// ---- Form ------------------------------------------------------------------------------------------

LaserForm::LaserForm(wxWindow* parent, std::function<void()> on_change)
    : m_parent(parent), m_grid(new wxFlexGridSizer(2, 4, 8)), m_on_change(std::move(on_change))
{
    m_grid->AddGrowableCol(1);
}

wxSizer* LaserForm::sizer() const { return m_grid; }

void LaserForm::heading(const wxString& text)
{
    auto* t = new wxStaticText(m_parent, wxID_ANY, text);
    t->SetFont(t->GetFont().Bold());
    m_grid->Add(t, 0, wxTOP, 6);
    m_grid->AddSpacer(1);
}

void LaserForm::row(const wxString& label, wxWindow* control)
{
    m_grid->Add(new wxStaticText(m_parent, wxID_ANY, label), 0, wxALIGN_CENTER_VERTICAL);
    m_grid->Add(control, 1, wxEXPAND);
}

void LaserForm::num(const wxString& label, double& v, double min, double max, double inc, int digits, const wxString& unit,
                    const wxString& tip)
{
    auto* box  = new wxBoxSizer(wxHORIZONTAL);
    auto* spin = new wxSpinCtrlDouble(m_parent, wxID_ANY, "", wxDefaultPosition, wxSize(m_parent->FromDIP(150), -1), wxSP_ARROW_KEYS, min, max, v, inc);
    spin->SetDigits(digits);
    if (!tip.empty()) spin->SetToolTip(tip);
    auto update = [cb = m_on_change, spin, &v, min, max](wxCommandEvent& e) {
        double d;
        if (e.GetEventType() == wxEVT_TEXT) {
            if (!e.GetString().ToDouble(&d) && !e.GetString().ToCDouble(&d)) return;
        } else
            d = spin->GetValue();
        d = std::clamp(d, min, max);
        if (d != v) {
            v = d;
            if (cb) cb();
        }
    };
    spin->Bind(wxEVT_SPINCTRLDOUBLE, [update](wxSpinDoubleEvent& e) { update(e); });
    spin->Bind(wxEVT_TEXT, update);
    box->Add(spin, 0);
    if (!unit.empty()) box->Add(new wxStaticText(m_parent, wxID_ANY, unit), 0, wxALIGN_CENTER_VERTICAL | wxLEFT, 4);
    m_grid->Add(new wxStaticText(m_parent, wxID_ANY, label), 0, wxALIGN_CENTER_VERTICAL);
    m_grid->Add(box, 1, wxEXPAND);
}

void LaserForm::integer(const wxString& label, int& v, int min, int max, const wxString& tip)
{
    auto* spin = new wxSpinCtrl(m_parent, wxID_ANY, "", wxDefaultPosition, wxSize(m_parent->FromDIP(150), -1), wxSP_ARROW_KEYS, min, max, v);
    if (!tip.empty()) spin->SetToolTip(tip);
    auto update = [cb = m_on_change, spin, &v] {
        if (spin->GetValue() != v) {
            v = spin->GetValue();
            if (cb) cb();
        }
    };
    spin->Bind(wxEVT_SPINCTRL, [update](wxSpinEvent&) { update(); });
    spin->Bind(wxEVT_TEXT, [update](wxCommandEvent&) { update(); });
    m_grid->Add(new wxStaticText(m_parent, wxID_ANY, label), 0, wxALIGN_CENTER_VERTICAL);
    m_grid->Add(spin, 0);
}

void LaserForm::check(const wxString& label, bool& v, const wxString& tip)
{
    auto* cb = new wxCheckBox(m_parent, wxID_ANY, label);
    cb->SetValue(v);
    if (!tip.empty()) cb->SetToolTip(tip);
    cb->Bind(wxEVT_CHECKBOX, [on_change = m_on_change, cb, &v](wxCommandEvent&) {
        v = cb->GetValue();
        if (on_change) on_change();
    });
    m_grid->AddSpacer(1);
    m_grid->Add(cb);
}

void LaserForm::text(const wxString& label, std::string& v, bool multiline)
{
    auto* t = new wxTextCtrl(m_parent, wxID_ANY, wxString::FromUTF8(v), wxDefaultPosition, wxSize(-1, multiline ? 70 : -1),
                             multiline ? wxTE_MULTILINE : 0);
    t->Bind(wxEVT_TEXT, [cb = m_on_change, t, &v](wxCommandEvent&) {
        v = t->GetValue().ToUTF8().data();
        if (cb) cb();
    });
    row(label, t);
}

void LaserForm::choice_int(const wxString& label, int value, const std::vector<wxString>& names, std::function<void(int)> set)
{
    wxArrayString arr;
    for (const wxString& n : names) arr.Add(n);
    auto* c = new wxChoice(m_parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, arr);
    c->SetSelection(std::clamp(value, 0, int(names.size()) - 1));
    c->Bind(wxEVT_CHOICE, [cb = m_on_change, c, set](wxCommandEvent&) {
        set(c->GetSelection());
        if (cb) cb();
    });
    row(label, c);
}

std::vector<wxString> laser_mode_names() { return {_L("Line"), _L("Fill"), _L("Fill + Line"), _L("Offset Fill")}; }
std::vector<wxString> dither_names()
{
    return {_L("Threshold"), _L("Ordered"), _L("Dither (Floyd-Steinberg)"), _L("Jarvis"), _L("Stucki"), _L("Atkinson"),
            _L("Newsprint"), _L("Halftone"), _L("Grayscale")};
}

wxString format_time(double s)
{
    if (s <= 0) return "0 s";
    const int t = int(std::lround(s));
    if (t >= 3600) return wxString::Format("%dh %02dm %02ds", t / 3600, (t / 60) % 60, t % 60);
    if (t >= 60) return wxString::Format("%dm %02ds", t / 60, t % 60);
    return wxString::Format("%.1f s", s);
}

namespace {

// OK/Cancel dialog around one page built by `build(panel)`.
bool run_dialog(wxWindow* parent, const wxString& title, const std::function<void(wxWindow*, wxSizer*)>& build)
{
    wxDialog dlg(parent, wxID_ANY, title, wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
    auto*    top = new wxBoxSizer(wxVERTICAL);
    build(&dlg, top);
    top->Add(dlg.CreateStdDialogButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxALL, 8);
    dlg.SetSizerAndFit(top);
    dlg.CentreOnParent();
    return dlg.ShowModal() == wxID_OK;
}

// Downscaled copy of an image shape (<= max_px on the long side), same physical size.
LaserShape downscaled(const LaserShape& img, int max_px)
{
    LaserShape s   = img;
    const int  stp = std::max(1, (std::max(img.image_w, img.image_h) + max_px - 1) / max_px);
    if (stp == 1) return s;
    s.image_w = img.image_w / stp;
    s.image_h = img.image_h / stp;
    s.gray.assign(size_t(s.image_w) * s.image_h, 255);
    for (int y = 0; y < s.image_h; ++y)
        for (int x = 0; x < s.image_w; ++x) s.gray[size_t(y) * s.image_w + x] = img.gray[size_t(y * stp) * img.image_w + x * stp];
    return s;
}

wxBitmap gray_bitmap(const std::vector<uint8_t>& g, int w, int h, bool burn)
{
    wxImage im(w, h);
    unsigned char* d = im.GetData();
    for (size_t i = 0; i < g.size(); ++i) d[3 * i] = d[3 * i + 1] = d[3 * i + 2] = burn ? uint8_t(255 - g[i]) : g[i];
    return wxBitmap(im);
}

} // namespace

// ---- Cut settings ----------------------------------------------------------------------------------

bool edit_cut_settings(wxWindow* parent, LaserLayer& layer, int index, const LaserShape* sample)
{
    LaserLayer l = layer;
    wxDialog   dlg(parent, wxID_ANY, wxString::Format(_L("Cut Settings Editor - C%02d"), index), wxDefaultPosition, wxDefaultSize,
                   wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
    auto* top  = new wxBoxSizer(wxVERTICAL);
    auto* book = new wxNotebook(&dlg, wxID_ANY);

    // Common
    auto*     common = new wxPanel(book);
    LaserForm fc(common);
    fc.text(_L("Name"), l.name);
    fc.choice(_L("Mode"), l.mode, laser_mode_names());
    fc.num(_L("Speed"), l.speed_mm_s, 0.1, 1000, 5, 1, "mm/s");
    fc.num(_L("Max power"), l.power_max, 0, 100, 1, 1, "%", _L("Laser power for this layer, % of the device's S max."));
    fc.num(_L("Min power"), l.power_min, 0, 100, 1, 1, "%", _L("Used on corners and slow moves with dynamic power (M4), and for white in Grayscale."));
    fc.integer(_L("Passes"), l.passes, 1, 100);
    fc.check(_L("Output"), l.output, _L("Unchecked layers are not burned."));
    fc.check(_L("Air assist"), l.air_assist);
    fc.heading(_L("Fill"));
    fc.num(_L("Line interval"), l.interval_mm, 0.01, 5, 0.01, 3, "mm");
    fc.num(_L("Scan angle"), l.angle_deg, -360, 360, 5, 1, "°");
    fc.check(_L("Bi-directional fill"), l.bidirectional);
    fc.check(_L("Crosshatch"), l.crosshatch);
    fc.num(_L("Overscanning"), l.overscan_pct, 0, 50, 0.5, 1, "%", _L("Extra travel at the ends of scan lines so the head is at speed when it fires."));
    fc.choice(_L("Fill grouping"), l.fill_grouping, {_L("Fill all shapes at once"), _L("Fill shapes individually"), _L("Fill groups together")});
    auto* sc = new wxBoxSizer(wxVERTICAL);
    sc->Add(fc.sizer(), 1, wxEXPAND | wxALL, 10);
    common->SetSizer(sc);
    book->AddPage(common, _L("Common"));

    // Advanced
    auto*     adv = new wxPanel(book);
    LaserForm fa(adv);
    fa.num(_L("Kerf offset"), l.kerf_offset_mm, -5, 5, 0.01, 3, "mm", _L("Positive grows closed shapes (outer out, holes in) to compensate the beam width."));
    fa.num(_L("Lead-in"), l.lead_in_mm, 0, 20, 0.5, 2, "mm");
    fa.check(_L("Tabs / bridges"), l.tabs, _L("Leave small uncut bridges so parts stay in the sheet."));
    fa.integer(_L("Tab count"), l.tab_count, 1, 100);
    fa.num(_L("Tab spacing"), l.tab_spacing_mm, 0, 1000, 5, 1, "mm", _L("0 = use the tab count"));
    fa.num(_L("Tab size"), l.tab_size_mm, 0.05, 10, 0.1, 2, "mm");
    fa.check(_L("Perforation / dot mode"), l.dot_mode);
    fa.num(_L("Dot time"), l.dot_dwell_ms, 0.1, 1000, 1, 1, "ms");
    fa.num(_L("Dot spacing"), l.dot_spacing_mm, 0.05, 50, 0.1, 2, "mm");
    fa.num(_L("Z offset"), l.z_offset_mm, -50, 50, 0.1, 2, "mm");
    fa.num(_L("Z step per pass"), l.z_step_per_pass, -10, 10, 0.1, 2, "mm");
    fa.integer(_L("Priority"), l.priority, -100, 100, _L("With 'Order by priority', lower numbers run first."));
    auto* sa = new wxBoxSizer(wxVERTICAL);
    sa->Add(fa.sizer(), 1, wxEXPAND | wxALL, 10);
    adv->SetSizer(sa);
    book->AddPage(adv, _L("Advanced"));

    // Image
    auto*           imgp  = new wxPanel(book);
    wxStaticBitmap* thumb = nullptr;
    LaserShape      thumb_src;
    if (sample != nullptr && sample->type == ShapeType::Image && sample->image_w > 0) thumb_src = downscaled(*sample, 200);
    else {   // a gradient with a dark circle, so every mode shows something
        thumb_src.type    = ShapeType::Image;
        thumb_src.image_w = thumb_src.image_h = 160;
        thumb_src.width_mm = thumb_src.height_mm = 20;
        thumb_src.gray.resize(160 * 160);
        for (int y = 0; y < 160; ++y)
            for (int x = 0; x < 160; ++x) {
                const double r = std::hypot(x - 80, y - 80);
                thumb_src.gray[y * 160 + x] = uint8_t(r < 35 ? 40 + r * 2 : x * 255 / 159);
            }
    }
    auto update_thumb = [&] {
        if (thumb == nullptr) return;
        std::vector<uint8_t> g = thumb_src.gray;
        apply_adjustments(g, thumb_src.image_w, thumb_src.image_h, thumb_src.brightness, thumb_src.contrast, thumb_src.gamma, thumb_src.invert);
        if (l.image_negative) for (uint8_t& v : g) v = 255 - v;
        const double dpi = thumb_src.width_mm > 0 ? thumb_src.image_w * 25.4 / thumb_src.width_mm : l.image_dpi;
        std::vector<uint8_t> burn = l.image_pass_through ? g : dither(g, thumb_src.image_w, thumb_src.image_h, l.image_dither, dpi,
                                                                      l.image_cells_per_inch, l.image_screen_angle_deg);
        if (l.image_pass_through) for (uint8_t& v : burn) v = v < 128 ? 255 : 0;
        thumb->SetBitmap(gray_bitmap(burn, thumb_src.image_w, thumb_src.image_h, true));
        thumb->GetParent()->Layout();
    };
    LaserForm fi(imgp, update_thumb);
    fi.choice(_L("Image mode"), l.image_dither, dither_names());
    fi.num(_L("DPI"), l.image_dpi, 10, 1500, 10, 0, "", _L("Scan lines per inch; 254 DPI = 0.1 mm."));
    fi.check(_L("Negative image"), l.image_negative);
    fi.check(_L("Pass-through"), l.image_pass_through, _L("The image is already dithered: burn it as is."));
    fi.num(_L("Cells per inch"), l.image_cells_per_inch, 5, 300, 1, 1, "", _L("Newsprint / Halftone screen"));
    fi.num(_L("Halftone angle"), l.image_screen_angle_deg, 0, 180, 1, 1, "°");
    thumb = new wxStaticBitmap(imgp, wxID_ANY, wxNullBitmap, wxDefaultPosition, wxSize(thumb_src.image_w, thumb_src.image_h));
    auto* si = new wxBoxSizer(wxHORIZONTAL);
    si->Add(fi.sizer(), 1, wxEXPAND | wxALL, 10);
    auto* tcol = new wxBoxSizer(wxVERTICAL);
    tcol->Add(new wxStaticText(imgp, wxID_ANY, sample ? _L("Preview (first image on this layer)") : _L("Preview (sample)")), 0, wxBOTTOM, 4);
    tcol->Add(thumb);
    si->Add(tcol, 0, wxALL, 10);
    imgp->SetSizer(si);
    update_thumb();
    book->AddPage(imgp, _L("Image"));

    top->Add(book, 1, wxEXPAND | wxALL, 8);
    top->Add(dlg.CreateStdDialogButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxALL, 8);
    dlg.SetSizerAndFit(top);
    dlg.CentreOnParent();
    if (dlg.ShowModal() != wxID_OK) return false;
    layer = l;
    return true;
}

// ---- Device / rotary / optimisation / material --------------------------------------------------------

bool edit_device(wxWindow* parent, LaserDevice& device)
{
    LaserDevice d    = device;
    int         baud = int(d.baud);
    bool        ok   = run_dialog(parent, _L("Device Settings"), [&](wxWindow* w, wxSizer* top) {
        LaserForm f(w);
        f.text(_L("Name"), d.name);
        f.choice(_L("Controller"), d.type, {"GRBL", "grblHAL", "Marlin", "Smoothieware", _L("Ruida (not supported yet)")});
        f.choice(_L("Laser type"), d.source, {_L("Diode"), "CO2"});
        f.num(_L("Bed width"), d.bed_w, 10, 3000, 10, 1, "mm");
        f.num(_L("Bed height"), d.bed_h, 10, 3000, 10, 1, "mm");
        f.choice(_L("Origin"), d.origin_corner, {_L("Front left"), _L("Rear left"), _L("Front right"), _L("Rear right")});
        f.num(_L("S value max"), d.s_max, 1, 100000, 100, 0, "", _L("GRBL $30: the S value for 100% power."));
        f.check(_L("Dynamic power (M4)"), d.laser_mode_dynamic, _L("Needs GRBL laser mode ($32=1). Scales power with speed so corners do not over-burn."));
        f.check(_L("Use G0 for travel"), d.uses_g0_for_travel, _L("Off: travel as G1 S0 (some controllers leave the laser on during G0)."));
        f.num(_L("Max speed"), d.max_speed_mm_s, 1, 2000, 10, 0, "mm/s");
        f.num(_L("Travel speed"), d.travel_speed_mm_s, 1, 2000, 10, 0, "mm/s");
        f.num(_L("Acceleration"), d.accel_mm_s2, 10, 50000, 100, 0, "mm/s²", _L("Only used for the time estimate."));
        f.integer(_L("Baud rate"), baud, 1200, 2000000);
        f.check(_L("Enable Z axis"), d.enable_z);
        f.num(_L("Frame power"), d.frame_power_pct, 0, 10, 0.5, 1, "%", _L("> 0: diode lasers frame with a faint visible beam."));
        f.check(_L("Marlin: fan-pin laser (M106)"), d.marlin_fan_laser);
        f.check(_L("Marlin: inline power (M3 I)"), d.marlin_inline);
        f.text(_L("Air assist on"), d.air_on_gcode);
        f.text(_L("Air assist off"), d.air_off_gcode);
        f.text(_L("Start G-code"), d.start_gcode, true);
        f.text(_L("End G-code"), d.end_gcode, true);
        top->Add(f.sizer(), 1, wxEXPAND | wxALL, 10);
    });
    d.baud = unsigned(baud);
    if (ok) device = d;
    return ok;
}

bool edit_rotary(wxWindow* parent, RotarySettings& rotary, std::function<void(const RotarySettings&)> test)
{
    RotarySettings r  = rotary;
    bool           ok = run_dialog(parent, _L("Rotary Setup"), [&](wxWindow* w, wxSizer* top) {
        LaserForm f(w);
        f.check(_L("Enable rotary"), r.enabled);
        f.choice(_L("Rotary type"), r.type, {_L("Chuck"), _L("Roller")});
        f.num(_L("Travel per rotation"), r.mm_per_rotation, 0.1, 10000, 1, 2, "mm", _L("Y distance the controller moves for one full turn."));
        f.num(_L("Roller diameter"), r.roller_diameter, 1, 500, 1, 2, "mm");
        f.num(_L("Object diameter"), r.object_diameter, 1, 1000, 1, 2, "mm");
        top->Add(f.sizer(), 1, wxEXPAND | wxALL, 10);
        auto* btn = new wxButton(w, wxID_ANY, _L("Test: rotate one turn"));
        btn->Enable(bool(test));
        btn->Bind(wxEVT_BUTTON, [&](wxCommandEvent&) { test(r); });
        top->Add(btn, 0, wxLEFT | wxRIGHT, 10);
    });
    if (ok) rotary = r;
    return ok;
}

bool edit_optimize(wxWindow* parent, JobSettings::Optimize& opt)
{
    JobSettings::Optimize o  = opt;
    bool                  ok = run_dialog(parent, _L("Optimization Settings"), [&](wxWindow* w, wxSizer* top) {
        LaserForm f(w);
        f.choice(_L("Order by"), o.order_by, {_L("Layer"), _L("Priority"), _L("Group")});
        f.check(_L("Cut inner shapes first"), o.cut_inner_first, _L("Holes before outlines, so parts do not drop before they are finished."));
        f.check(_L("Reduce travel moves"), o.reduce_travel);
        f.check(_L("Reduce direction changes"), o.reduce_direction_changes);
        top->Add(f.sizer(), 1, wxEXPAND | wxALL, 10);
    });
    if (ok) opt = o;
    return ok;
}

bool edit_material(wxWindow* parent, MaterialEntry& entry)
{
    MaterialEntry e  = entry;
    bool          ok = run_dialog(parent, _L("Material Entry"), [&](wxWindow* w, wxSizer* top) {
        LaserForm f(w);
        f.text(_L("Material"), e.material);
        f.num(_L("Thickness"), e.thickness_mm, 0, 100, 0.5, 2, "mm", _L("0 = not applicable (engraving)"));
        f.text(_L("Description"), e.description);
        f.choice(_L("Mode"), e.settings.mode, laser_mode_names());
        f.num(_L("Speed"), e.settings.speed_mm_s, 0.1, 1000, 5, 1, "mm/s");
        f.num(_L("Max power"), e.settings.power_max, 0, 100, 1, 1, "%");
        f.num(_L("Min power"), e.settings.power_min, 0, 100, 1, 1, "%");
        f.integer(_L("Passes"), e.settings.passes, 1, 100);
        f.num(_L("Line interval"), e.settings.interval_mm, 0.01, 5, 0.01, 3, "mm");
        f.text(_L("Notes"), e.notes, true);
        top->Add(f.sizer(), 1, wxEXPAND | wxALL, 10);
    });
    if (ok) entry = e;
    return ok;
}

// ---- Tool parameter dialogs ------------------------------------------------------------------------

bool ask_offset(wxWindow* parent, OffsetParams& p)
{
    return run_dialog(parent, _L("Offset Shapes"), [&](wxWindow* w, wxSizer* top) {
        LaserForm f(w);
        f.num(_L("Offset distance"), p.distance, 0.01, 500, 0.5, 2, "mm");
        f.choice(_L("Direction"), p.dir, {_L("Outward"), _L("Inward"), _L("Both")});
        f.choice(_L("Corner style"), p.corners, {_L("Round"), _L("Sharp"), _L("Bevel")});
        f.check(_L("Delete original shapes"), p.delete_original);
        top->Add(f.sizer(), 1, wxEXPAND | wxALL, 10);
    });
}

bool ask_array(wxWindow* parent, ArrayParams& p)
{
    return run_dialog(parent, _L("Grid Array"), [&](wxWindow* w, wxSizer* top) {
        LaserForm f(w);
        f.integer(_L("Columns"), p.columns, 1, 200);
        f.integer(_L("Rows"), p.rows, 1, 200);
        f.num(_L("Horizontal gap"), p.dx, -1000, 1000, 1, 2, "mm");
        f.num(_L("Vertical gap"), p.dy, -1000, 1000, 1, 2, "mm");
        top->Add(f.sizer(), 1, wxEXPAND | wxALL, 10);
    });
}

bool ask_circular_array(wxWindow* parent, CircularArrayParams& p)
{
    return run_dialog(parent, _L("Circular Array"), [&](wxWindow* w, wxSizer* top) {
        LaserForm f(w);
        f.integer(_L("Number of copies"), p.count, 2, 360);
        f.num(_L("Centre X"), p.cx, -10000, 10000, 1, 2, "mm");
        f.num(_L("Centre Y"), p.cy, -10000, 10000, 1, 2, "mm");
        f.num(_L("Start angle"), p.start_deg, -360, 360, 5, 1, "°");
        f.num(_L("Span"), p.span_deg, -360, 360, 5, 1, "°", _L("360° spreads the copies around the full circle."));
        f.check(_L("Rotate copies"), p.rotate_copies);
        top->Add(f.sizer(), 1, wxEXPAND | wxALL, 10);
    });
}

bool ask_trace(wxWindow* parent, const LaserShape& image, TraceOptions& opts)
{
    const LaserShape small = downscaled(image, 320);
    TraceOptions     o     = opts;
    wxDialog         dlg(parent, wxID_ANY, _L("Trace Image"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
    auto*            top  = new wxBoxSizer(wxHORIZONTAL);
    const double     fit  = 360. / std::max(small.image_w, small.image_h);
    const wxSize     psz(int(small.image_w * fit) + 2, int(small.image_h * fit) + 2);
    auto*            view = new wxPanel(&dlg, wxID_ANY, wxDefaultPosition, psz);
    view->SetBackgroundStyle(wxBG_STYLE_PAINT);
    std::vector<uint8_t> adj = small.gray;
    apply_adjustments(adj, small.image_w, small.image_h, small.brightness, small.contrast, small.gamma, small.invert);
    const wxBitmap bmp = wxBitmap(gray_bitmap(adj, small.image_w, small.image_h, false).ConvertToImage().Scale(psz.x - 2, psz.y - 2));
    LaserShape     traced;
    wxStaticText*  info = nullptr;
    auto           retrace = [&] {
        traced = trace_image(small, o);
        size_t n = 0;
        for (const LaserPath& p : traced.paths) n += p.pts.points.size();
        if (info) info->SetLabel(wxString::Format(_L("%zu outlines, %zu points"), traced.paths.size(), n));
        view->Refresh();
    };
    view->Bind(wxEVT_PAINT, [&](wxPaintEvent&) {
        wxAutoBufferedPaintDC dc(view);
        dc.SetBackground(*wxWHITE_BRUSH);
        dc.Clear();
        dc.DrawBitmap(bmp, 1, 1);
        dc.SetPen(wxPen(wxColour(230, 40, 40), 1));
        // Local mm (centred, Y up) -> panel px.
        const double sx = (psz.x - 2) / small.width_mm, sy = (psz.y - 2) / small.height_mm;
        for (const LaserPath& p : traced.paths) {
            std::vector<wxPoint> pts;
            for (const Point& q : p.pts.points)
                pts.emplace_back(int(1 + (unscale<double>(q.x()) + small.width_mm / 2) * sx), int(1 + (small.height_mm / 2 - unscale<double>(q.y())) * sy));
            if (p.closed && !pts.empty()) pts.push_back(pts.front());
            if (pts.size() > 1) dc.DrawLines(int(pts.size()), pts.data());
        }
    });
    auto* side = new wxBoxSizer(wxVERTICAL);
    auto  slider = [&](const wxString& label, int& v, int min, int max) {
        side->Add(new wxStaticText(&dlg, wxID_ANY, label), 0, wxTOP, 6);
        auto* s = new wxSlider(&dlg, wxID_ANY, v, min, max, wxDefaultPosition, wxSize(220, -1), wxSL_HORIZONTAL | wxSL_LABELS);
        s->Bind(wxEVT_SLIDER, [pv = &v, s, &retrace](wxCommandEvent&) { *pv = s->GetValue(); retrace(); });
        side->Add(s, 0, wxEXPAND);
    };
    int smooth10 = int(std::lround(o.smoothness_px * 10));
    slider(_L("Threshold (darker than this is traced)"), o.threshold, 0, 255);
    slider(_L("Cutoff (ignore darker than this)"), o.cutoff, 0, 255);
    slider(_L("Ignore less than (px)"), o.ignore_less_than_px, 0, 50);
    side->Add(new wxStaticText(&dlg, wxID_ANY, _L("Smoothness (x0.1 px)")), 0, wxTOP, 6);
    auto* sm = new wxSlider(&dlg, wxID_ANY, smooth10, 0, 50, wxDefaultPosition, wxSize(220, -1), wxSL_HORIZONTAL | wxSL_LABELS);
    sm->Bind(wxEVT_SLIDER, [&](wxCommandEvent&) { o.smoothness_px = sm->GetValue() / 10.; retrace(); });
    side->Add(sm, 0, wxEXPAND);
    info = new wxStaticText(&dlg, wxID_ANY, "");
    side->Add(info, 0, wxTOP, 10);
    side->AddStretchSpacer();
    side->Add(dlg.CreateStdDialogButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxTOP, 8);
    top->Add(view, 0, wxALL, 10);
    top->Add(side, 1, wxEXPAND | wxALL, 10);
    dlg.SetSizerAndFit(top);
    retrace();
    dlg.CentreOnParent();
    if (dlg.ShowModal() != wxID_OK) return false;
    opts = o;
    return true;
}

// ---- Preview ---------------------------------------------------------------------------------------

namespace {

class PreviewView : public wxPanel
{
public:
    PreviewView(wxWindow* parent, const LaserJob& job, const LaserDevice& dev) : wxPanel(parent, wxID_ANY, wxDefaultPosition, wxSize(640, 520)), m_job(job)
    {
        SetBackgroundStyle(wxBG_STYLE_PAINT);
        // The job with a margin (the bed when there is nothing lit).
        if (job.bounds.defined) {
            m_box = job.bounds;
            m_box.offset(std::max(5., 0.1 * std::max(m_box.size().x(), m_box.size().y())));
        } else {
            m_box.merge(Vec2d(0, 0));
            m_box.merge(Vec2d(dev.bed_w, dev.bed_h));
        }
        m_bed = BoundingBoxf(Vec2d(0, 0), Vec2d(dev.bed_w, dev.bed_h));
        m_upto = job.segments.size();
        Bind(wxEVT_PAINT, [this](wxPaintEvent&) { paint(); });
        Bind(wxEVT_SIZE, [this](wxSizeEvent& e) { m_cache = wxBitmap(); Refresh(); e.Skip(); });
    }
    void set_upto(size_t n) { m_upto = std::min(n, m_job.segments.size()); m_cache = wxBitmap(); Refresh(); }
    void set_travel(bool on) { m_travel = on; m_cache = wxBitmap(); Refresh(); }

private:
    void line(std::vector<uint8_t>& img, int w, int h, Vec2d a, Vec2d b, const uint8_t rgb[3], bool dotted) const
    {
        const int n = std::max(1, int(std::ceil(std::max(std::abs(b.x() - a.x()), std::abs(b.y() - a.y())))));
        for (int i = 0; i <= n; ++i) {
            if (dotted && (i / 3) % 2) continue;
            const int x = int(a.x() + (b.x() - a.x()) * i / n), y = int(a.y() + (b.y() - a.y()) * i / n);
            if (x < 0 || y < 0 || x >= w || y >= h) continue;
            uint8_t* p = &img[(size_t(y) * w + x) * 3];
            p[0] = rgb[0]; p[1] = rgb[1]; p[2] = rgb[2];
        }
    }
    void paint()
    {
        wxAutoBufferedPaintDC dc(this);
        const wxSize sz = GetClientSize();
        if (sz.x < 10 || sz.y < 10) return;
        const double s  = std::min((sz.x - 20) / m_box.size().x(), (sz.y - 20) / m_box.size().y());
        const Vec2d  o(sz.x / 2. - m_box.center().x() * s, sz.y / 2. + m_box.center().y() * s);
        auto         px = [&](const Vec2d& p) { return Vec2d(o.x() + p.x() * s, o.y() - p.y() * s); };
        if (!m_cache.IsOk()) {
            std::vector<uint8_t> img(size_t(sz.x) * sz.y * 3, 200);
            const Vec2d a = px(m_bed.min), b = px(m_bed.max);
            for (int y = std::max(0, int(b.y())); y < std::min(sz.y, int(a.y())); ++y)
                for (int x = std::max(0, int(a.x())); x < std::min(sz.x, int(b.x())); ++x) {
                    uint8_t* p = &img[(size_t(y) * sz.x + x) * 3];
                    p[0] = p[1] = p[2] = 255;
                }
            const uint8_t red[3] = {230, 50, 50};
            for (size_t i = 0; i < m_upto; ++i) {
                const Segment& sg = m_job.segments[i];
                switch (sg.kind) {
                case Segment::Kind::Travel:
                    if (m_travel) line(img, sz.x, sz.y, px(sg.from), px(sg.to), red, true);
                    break;
                case Segment::Kind::Cut:
                case Segment::Kind::Dwell: {
                    const Rgb c = layer_color(sg.layer);
                    const uint8_t rgb[3] = {c.r, c.g, c.b};
                    line(img, sz.x, sz.y, px(sg.from), px(sg.to), rgb, false);
                    break;
                }
                case Segment::Kind::Scan: {
                    if (sg.scan < 0 || sg.scan >= int(m_job.scans.size())) break;
                    const ScanLine& sl  = m_job.scans[sg.scan];
                    const double    len = (sl.end - sl.start).norm();
                    if (len <= 0) break;
                    const Vec2d dir = (sl.end - sl.start) / len;
                    for (const Run& r : sl.runs) {
                        const uint8_t g = uint8_t(std::clamp(235. - 2.3 * r.power_pct, 0., 235.));
                        const uint8_t rgb[3] = {g, g, g};
                        line(img, sz.x, sz.y, px(sl.start + dir * r.x0), px(sl.start + dir * r.x1), rgb, false);
                    }
                    break;
                }
                }
            }
            wxImage im(sz.x, sz.y);
            std::copy(img.begin(), img.end(), im.GetData());
            m_cache = wxBitmap(im);
        }
        dc.DrawBitmap(m_cache, 0, 0);
        if (m_upto > 0 && m_upto <= m_job.segments.size()) {
            const Vec2d h = px(m_job.segments[m_upto - 1].to);
            dc.SetPen(wxPen(wxColour(230, 40, 40), 2));
            dc.SetBrush(*wxTRANSPARENT_BRUSH);
            dc.DrawCircle(int(h.x()), int(h.y()), 6);
        }
    }

    const LaserJob& m_job;
    BoundingBoxf    m_box, m_bed;
    size_t          m_upto{0};
    bool            m_travel{true};
    wxBitmap        m_cache;
};

} // namespace

void show_preview(wxWindow* parent, const LaserDocument& doc, const LaserJob& job, const LaserDevice& device, std::function<void()> save,
                  std::function<void()> start)
{
    wxDialog dlg(parent, wxID_ANY, _L("Preview"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER | wxMAXIMIZE_BOX);
    auto*    top  = new wxBoxSizer(wxHORIZONTAL);
    auto*    left = new wxBoxSizer(wxVERTICAL);
    auto*    view = new PreviewView(&dlg, job, device);
    left->Add(view, 1, wxEXPAND);
    // Playback: cumulative time per segment (length / speed, scaled to the planner's estimate).
    std::vector<double> t(job.segments.size() + 1, 0.);
    for (size_t i = 0; i < job.segments.size(); ++i) {
        const Segment& s = job.segments[i];
        const double   d = s.kind == Segment::Kind::Dwell ? s.dwell_ms / 1000. : (s.to - s.from).norm() / std::max(1e-3, s.speed_mm_s);
        t[i + 1] = t[i] + d;
    }
    const double scale = t.back() > 0 && job.estimated_time_s > 0 ? job.estimated_time_s / t.back() : 1.;
    auto*        slider = new wxSlider(&dlg, wxID_ANY, int(job.segments.size()), 0, std::max(1, int(job.segments.size())));
    auto*        at     = new wxStaticText(&dlg, wxID_ANY, "");
    auto         moved  = [&] {
        const size_t n = size_t(slider->GetValue());
        view->set_upto(n);
        at->SetLabel(wxString::Format(_L("Time %s of %s"), format_time(t[std::min(n, t.size() - 1)] * scale), format_time(job.estimated_time_s)));
    };
    slider->Bind(wxEVT_SLIDER, [&](wxCommandEvent&) { moved(); });
    auto* play = new wxBoxSizer(wxHORIZONTAL);
    play->Add(slider, 1, wxEXPAND);
    play->Add(at, 0, wxALIGN_CENTER_VERTICAL | wxLEFT, 8);
    left->Add(play, 0, wxEXPAND | wxTOP, 6);
    top->Add(left, 1, wxEXPAND | wxALL, 8);

    auto* side = new wxBoxSizer(wxVERTICAL);
    auto* total = new wxStaticText(&dlg, wxID_ANY, wxString::Format(_L("Estimated time: %s"), format_time(job.estimated_time_s)));
    total->SetFont(total->GetFont().Bold());
    side->Add(total, 0, wxBOTTOM, 8);
    auto* layers = new wxListCtrl(&dlg, wxID_ANY, wxDefaultPosition, wxSize(230, 200), wxLC_REPORT | wxLC_SINGLE_SEL);
    layers->AppendColumn(_L("Layer"), wxLIST_FORMAT_LEFT, 120);
    layers->AppendColumn(_L("Time"), wxLIST_FORMAT_RIGHT, 100);
    for (int l = 0; l < kLayerCount; ++l)
        if (job.layer_time_s[l] > 0) {
            const std::string& name = doc.layers[l].name;
            const bool         own  = !name.empty() && name != wxString::Format("C%02d", l).ToStdString();
            const long row = layers->InsertItem(layers->GetItemCount(), wxString::Format("C%02d", l) + (own ? " " + wxString::FromUTF8(name) : wxString()));
            layers->SetItem(row, 1, format_time(job.layer_time_s[l]));
            const Rgb c = layer_color(l);
            layers->SetItemTextColour(row, wxColour(c.r, c.g, c.b));
        }
    side->Add(layers, 0, wxEXPAND);
    if (job.bounds.defined)
        side->Add(new wxStaticText(&dlg, wxID_ANY, wxString::Format(_L("Job size: %.1f x %.1f mm"), job.bounds.size().x(), job.bounds.size().y())), 0, wxTOP, 8);
    for (const std::string& w : job.warnings) {
        auto* wt = new wxStaticText(&dlg, wxID_ANY, "⚠ " + wxString::FromUTF8(w));
        wt->SetForegroundColour(wxColour(180, 100, 0));
        wt->Wrap(230);
        side->Add(wt, 0, wxTOP, 6);
    }
    auto* trav = new wxCheckBox(&dlg, wxID_ANY, _L("Show traversal moves"));
    trav->SetValue(true);
    trav->Bind(wxEVT_CHECKBOX, [&](wxCommandEvent&) { view->set_travel(trav->GetValue()); });
    side->Add(trav, 0, wxTOP, 10);
    side->AddStretchSpacer();
    auto* bsave = new wxButton(&dlg, wxID_ANY, _L("Save G-code..."));
    bsave->Bind(wxEVT_BUTTON, [&](wxCommandEvent&) { if (save) save(); });
    auto* bstart = new wxButton(&dlg, wxID_ANY, _L("Start"));
    bstart->Enable(bool(start));
    if (!start) bstart->SetToolTip(_L("Connect a laser on the Laser page first."));
    bstart->Bind(wxEVT_BUTTON, [&](wxCommandEvent&) {
        dlg.EndModal(wxID_OK);
        start();
    });
    side->Add(bsave, 0, wxEXPAND | wxTOP, 4);
    side->Add(bstart, 0, wxEXPAND | wxTOP, 4);
    side->Add(new wxButton(&dlg, wxID_CANCEL, _L("Close")), 0, wxEXPAND | wxTOP, 4);
    top->Add(side, 0, wxEXPAND | wxALL, 8);
    dlg.SetSizerAndFit(top);
    moved();
    dlg.CentreOnParent();
    dlg.ShowModal();
}

void show_gcode(wxWindow* parent, const wxString& title, const wxString& intro, const std::string& gcode)
{
    wxDialog dlg(parent, wxID_ANY, title, wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
    auto*    top = new wxBoxSizer(wxVERTICAL);
    auto*    t   = new wxStaticText(&dlg, wxID_ANY, intro);
    t->Wrap(520);
    top->Add(t, 0, wxALL, 10);
    auto* text = new wxTextCtrl(&dlg, wxID_ANY, wxString::FromUTF8(gcode), wxDefaultPosition, wxSize(540, 320), wxTE_MULTILINE | wxTE_READONLY | wxHSCROLL);
    text->SetFont(wxFont(9, wxFONTFAMILY_TELETYPE, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL));
    top->Add(text, 1, wxEXPAND | wxLEFT | wxRIGHT, 10);
    auto* row  = new wxBoxSizer(wxHORIZONTAL);
    auto* save = new wxButton(&dlg, wxID_ANY, _L("Save..."));
    save->Bind(wxEVT_BUTTON, [&](wxCommandEvent&) {
        wxFileDialog fd(&dlg, _L("Save G-code"), "", "frame.gcode", "G-code (*.gcode;*.nc)|*.gcode;*.nc", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
        if (fd.ShowModal() != wxID_OK) return;
        boost::nowide::ofstream f(fd.GetPath().ToUTF8().data(), std::ios::binary);
        f << gcode;
    });
    row->Add(save);
    row->AddStretchSpacer();
    row->Add(new wxButton(&dlg, wxID_OK, _L("Close")));
    top->Add(row, 0, wxEXPAND | wxALL, 10);
    dlg.SetSizerAndFit(top);
    dlg.CentreOnParent();
    dlg.ShowModal();
}

}} // namespace Slic3r::GUI

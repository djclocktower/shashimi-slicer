#pragma once

// Dialogs of the Laser tab and the small form builder they (and the Shape Properties page) share.

#include <wx/string.h>

#include "libslic3r/Laser/Laser.hpp"

#include <functional>
#include <string>
#include <vector>

class wxWindow;
class wxFlexGridSizer;
class wxSizer;

namespace Slic3r { namespace GUI {

// Label + control rows bound straight to fields: every edit writes the field and calls on_change.
// The fields must outlive the controls; the form itself may go away once built (the controls keep
// a copy of on_change).
class LaserForm
{
public:
    LaserForm(wxWindow* parent, std::function<void()> on_change = {});
    wxSizer*  sizer() const;
    wxWindow* parent() const { return m_parent; }

    void heading(const wxString& text);
    void num(const wxString& label, double& v, double min, double max, double inc = 1, int digits = 2, const wxString& unit = "",
             const wxString& tip = "");
    void integer(const wxString& label, int& v, int min, int max, const wxString& tip = "");
    void check(const wxString& label, bool& v, const wxString& tip = "");
    void text(const wxString& label, std::string& v, bool multiline = false);
    void choice_int(const wxString& label, int value, const std::vector<wxString>& names, std::function<void(int)> set);
    template<class E> void choice(const wxString& label, E& v, const std::vector<wxString>& names)
    {
        choice_int(label, int(v), names, [&v](int i) { v = E(i); });
    }
    // Any other control, created with parent() as its parent.
    void row(const wxString& label, wxWindow* control);

private:
    wxWindow*             m_parent;
    wxFlexGridSizer*      m_grid;
    std::function<void()> m_on_change;
};

std::vector<wxString> laser_mode_names();
std::vector<wxString> dither_names();

// Cut Settings Editor (Common / Advanced / Image). `sample` (may be null): an image on the layer
// for the live dither thumbnail. True on OK (layer updated).
bool edit_cut_settings(wxWindow* parent, Laser::LaserLayer& layer, int index, const Laser::LaserShape* sample);
bool edit_device(wxWindow* parent, Laser::LaserDevice& device);
// `test` jogs the rotary one full turn with the given settings (null when not connected).
bool edit_rotary(wxWindow* parent, Laser::RotarySettings& rotary, std::function<void(const Laser::RotarySettings&)> test);
bool edit_optimize(wxWindow* parent, Laser::JobSettings::Optimize& opt);
bool edit_material(wxWindow* parent, Laser::MaterialEntry& entry);

struct OffsetParams {
    double              distance{1};
    Laser::OffsetDir    dir{Laser::OffsetDir::Outward};
    Laser::CornerStyle  corners{Laser::CornerStyle::Round};
    bool                delete_original{false};
};
bool ask_offset(wxWindow* parent, OffsetParams& p);

struct ArrayParams {
    int    columns{3}, rows{2};
    double dx{5}, dy{5};       // gap between copies (mm)
};
bool ask_array(wxWindow* parent, ArrayParams& p);

struct CircularArrayParams {
    int    count{6};
    double cx{0}, cy{0};       // centre (mm)
    double start_deg{0}, span_deg{360};
    bool   rotate_copies{true};
};
bool ask_circular_array(wxWindow* parent, CircularArrayParams& p);

// Trace Image with a live preview (B's trace_image on a downscaled copy).
bool ask_trace(wxWindow* parent, const Laser::LaserShape& image, Laser::TraceOptions& opts);

// Preview of a planned job: layer times, traversal toggle, playback slider. `save` writes G-code,
// `start` streams it (null disables the button).
void show_preview(wxWindow* parent, const Laser::LaserDocument& doc, const Laser::LaserJob& job, const Laser::LaserDevice& device,
                  std::function<void()> save, std::function<void()> start);

// Read-only G-code viewer with Save... (framing without a device).
void show_gcode(wxWindow* parent, const wxString& title, const wxString& intro, const std::string& gcode);

wxString format_time(double seconds);

}} // namespace Slic3r::GUI

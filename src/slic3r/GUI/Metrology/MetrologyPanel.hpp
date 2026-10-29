#pragma once

// The Measure tab: camera inspection, calibration, outline tracing, reverse engineering and
// nozzle-touch probing on a Bambu printer, laid out like an image-dimension measuring system.
// All measurement comes from the Slic3r::Metrology kernel (src/libslic3r/Metrology/README.md);
// docs/HLSD/shashimi-measure-front-end.md describes the design.

#include "libslic3r/Metrology/Metrology.hpp"

#include <wx/panel.h>

#include <array>
#include <functional>
#include <string>
#include <vector>

class wxButton;
class wxCheckBox;
class wxChoice;
class wxFlexGridSizer;
class wxGraphicsContext;
class wxListCtrl;
class wxSimplebook;
class wxSpinCtrlDouble;
class wxStaticText;

namespace Slic3r { namespace GUI {

class MetrologyView;

class MetrologyPanel : public wxPanel
{
public:
    explicit MetrologyPanel(wxWindow* parent);
    ~MetrologyPanel() override;

private:
    using P2 = Metrology::P2;

    enum Mode { ModeInspect, ModeCalibrate, ModeTrace, ModeReverse, ModeProbe, ModeCount };
    enum Tool { ToolSelect, ToolLine, ToolCircle, ToolWidth, ToolBox, ToolTwoPoint, ToolCount };
    enum Source { SourceSimulator, SourceFiles };

    // ---- Layout ----------------------------------------------------------------------------------
    void      build_top_bar(wxSizer* root);
    wxWindow* build_tool_strip(wxWindow* parent);
    wxWindow* build_inspect_page(wxWindow* parent);
    wxWindow* build_calibrate_page(wxWindow* parent);
    wxWindow* build_trace_page(wxWindow* parent);
    wxWindow* build_reverse_page(wxWindow* parent);
    wxWindow* build_probe_page(wxWindow* parent);
    // A labelled number field bound to `value`; `changed` runs after every edit.
    wxSpinCtrlDouble* add_number(wxWindow* parent, wxFlexGridSizer* grid, const wxString& label, double& value, double lo, double hi,
                                 double step, int digits, std::function<void()> changed = {});

    void set_mode(Mode m);
    void set_tool(Tool t);
    void set_status(const wxString& text);
    void refresh_cal_status();
    void update_judge();

    // ---- Frames ----------------------------------------------------------------------------------
    bool simulator() const { return m_source == SourceSimulator; }
    void capture();
    void open_image(bool light_off);
    void set_frame(Metrology::GrayImage img);
    bool load_gray(const wxString& path, Metrology::GrayImage& out);

    // ---- Inspect ---------------------------------------------------------------------------------
    void add_feature_from_drag(const P2& a, const P2& b);
    void add_derived(Metrology::FeatureKind kind);
    void run_program();
    void refresh_feature_list();
    void edit_tolerance();
    void delete_feature();
    void export_csv();
    int  selected_feature() const;

    // ---- Calibrate -------------------------------------------------------------------------------
    void detect_mat();
    void add_raised_frame();
    void solve_camera();
    void auto_calibrate_simulator();
    void two_point_click(const P2& p);
    void save_settings() const;
    void load_settings();

    // ---- Trace -----------------------------------------------------------------------------------
    void set_box(const P2& a_px, const P2& b_px);
    std::vector<Metrology::Region> trace_box() const;
    void do_trace();
    void export_svg(const std::vector<Metrology::Region>& regions);
    void send_extrusion();

    // ---- Reverse engineering ---------------------------------------------------------------------
    void re_start();
    void re_run_batch_simulator();
    void re_save_batch_gcode();
    void re_load_heights_csv();
    void re_read_heights_from_frames();
    void re_build();
    void re_send_to_plate();
    void re_export_stl();
    void refresh_re();

    // ---- Probe -----------------------------------------------------------------------------------
    void save_repeatability_gcode();
    void analyze_frames();
    // Plane offsets of the mat in each still of a folder of frames; false with an error message.
    bool offsets_from_folder(const wxString& dir, size_t expected, std::vector<double>& offsets, wxString& error);
    void refresh_budget();

    // ---- Drawing ---------------------------------------------------------------------------------
    void draw_overlay(wxGraphicsContext& gc);
    P2   mm_to_px(const P2& mm, double z) const { return m_cam.to_px(mm, z); }

    // Widgets.
    MetrologyView*                  m_view       = nullptr;
    wxSimplebook*                   m_pages      = nullptr;
    std::array<wxButton*, ModeCount> m_mode_btn{};
    std::array<wxButton*, ToolCount> m_tool_btn{};
    wxChoice*     m_source_choice = nullptr;
    wxChoice*     m_scene_choice  = nullptr;
    wxCheckBox*   m_light_pair    = nullptr;
    wxPanel*      m_judge_box     = nullptr;
    wxStaticText* m_judge_text    = nullptr;
    wxStaticText* m_status        = nullptr;
    wxStaticText* m_cursor        = nullptr;
    wxStaticText* m_cal_text      = nullptr;
    wxListCtrl*   m_features      = nullptr;
    wxChoice*     m_polarity      = nullptr;
    wxStaticText* m_cal_result    = nullptr;
    wxListCtrl*   m_raised_list   = nullptr;
    wxStaticText* m_trace_info    = nullptr;
    wxStaticText* m_re_state      = nullptr;
    wxListCtrl*   m_re_batch      = nullptr;
    wxListCtrl*   m_re_levels     = nullptr;
    wxStaticText* m_probe_result  = nullptr;
    wxStaticText* m_budget_text   = nullptr;

    // State.
    Mode   m_mode   = ModeInspect;
    Tool   m_tool   = ToolSelect;
    Source m_source = SourceSimulator;

    Metrology::GrayImage m_frame, m_light_off;
    Metrology::CameraModel        m_cam;
    Metrology::MatLayout          m_mat;
    Metrology::PlateCalibration   m_cal;
    std::vector<double>           m_raised_offsets;
    std::vector<Metrology::PlateCalibration> m_raised;
    double m_raise_mm = 10;

    Metrology::Program m_prog;
    double m_feature_z      = 0;
    double m_contrast       = 12;
    double m_scans          = 16;
    double m_roi_width      = 40;
    bool   m_part_bright    = true;

    bool   m_has_box = false;
    P2     m_box_lo, m_box_hi;          // plate mm
    double m_trace_res      = 0.1;
    double m_trace_simplify = 0.03;
    double m_extrude_h      = 3;
    std::vector<Metrology::Region> m_trace;

    std::vector<P2> m_two_points;       // image px
    double          m_two_point_mm = 50;

    Metrology::ReSettings m_re_set;
    Metrology::ReSession  m_re;
    Metrology::ReModel    m_re_model;
    bool                  m_re_built     = false;
    double                m_re_touches   = 3;   // number fields are doubles; copied into m_re_set
    double                m_re_anchors   = 4;
    double                m_max_height   = 30;
    double                m_sim_noise    = 0.02;

    Metrology::ProbeSettings m_probe;
    double m_repeat_x = 128, m_repeat_y = 128, m_repeat_n = 10;
    P2     m_mat_ref{174, 6};

    Metrology::SimScene  m_scene;
    Metrology::SimCamera m_simcam;
    double   m_sim_offset = 0;
    unsigned m_seed       = 1;
    double   m_trigger    = 0.010;
};

}} // namespace Slic3r::GUI

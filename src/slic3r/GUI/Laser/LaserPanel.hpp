#pragma once

// The Laser tab (LightBurn workflow): a 2D bed workspace with vector shapes, text and images on
// 30 colour layers, a device connection that streams G-code to GRBL-family controllers, preview
// and G-code export. docs/HLSD/shashimi-laser-front-end.md describes the design.

#include <wx/panel.h>
#include <wx/timer.h>

#include "libslic3r/Laser/Laser.hpp"
#include "slic3r/Utils/GrblStreamer.hpp"

#include <array>
#include <string>
#include <vector>

class wxBitmapButton;
class wxButton;
class wxCheckBox;
class wxChoice;
class wxListCtrl;
class wxNotebook;
class wxRadioButton;
class wxSpinCtrlDouble;
class wxStaticText;
class wxTextCtrl;
class wxTreeCtrl;
class wxDataViewListCtrl;
class wxSlider;
class wxGauge;
class wxScrolledWindow;
class wxStaticBitmap;

namespace Slic3r { namespace GUI {

class LaserCanvas;

class LaserPanel : public wxPanel
{
public:
    explicit LaserPanel(wxWindow* parent);
    ~LaserPanel() override;

    // Tab activation (LazyPage forwards Show): picks up a project loaded or reset elsewhere.
    bool Show(bool show = true) override;

    // ---- Document -------------------------------------------------------------------------------
    Laser::LaserDocument&       doc() { return m_doc; }
    const Laser::LaserDocument& doc() const { return m_doc; }
    // Top-level shape indices (a group stands for its members).
    const std::vector<int>& selection() const { return m_sel; }
    void                    set_selection(std::vector<int> sel);
    int                     current_layer() const { return m_current_layer; }
    // After an edit: undo snapshot, Model::laser_recipe sync, refresh of every view.
    void commit(const std::string& what);
    // Live change during a drag (no undo step yet).
    void geometry_changed();
    void add_shape_and_select(Laser::LaserShape shape, const std::string& what);
    void edit_text(int index);
    void set_status(const wxString& text);
    void tool_finished();   // a drawing tool ended: back to Select

    const Laser::LaserDevice& device() const { return m_devices[m_device_idx]; }
    // Head position (workspace mm) when connected, else false.
    bool head_position(Vec2d& pos) const;

    // Canvas context menu / keys.
    void cmd_delete();
    void cmd_copy(bool cut);
    void cmd_paste();
    void cmd_duplicate();
    void cmd_group();
    void cmd_ungroup();
    void cmd_lock(bool lock);
    void cmd_to_layer(int layer);
    void cmd_to_path();
    void cmd_select_all();
    void undo();
    void redo();
    void nudge(double dx, double dy);

private:
    // Layout
    void build_toolbar(wxWindow* parent, wxSizer* sizer);
    void build_toolstrip(wxWindow* parent, wxSizer* sizer);
    void build_colour_strip(wxWindow* parent, wxSizer* sizer);
    wxWindow* build_cuts_page(wxWindow* book);
    wxWindow* build_laser_page(wxWindow* book);
    wxWindow* build_move_page(wxWindow* book);
    wxWindow* build_console_page(wxWindow* book);
    wxWindow* build_props_page(wxWindow* book);
    wxWindow* build_library_page(wxWindow* book);

    // Refresh
    void refresh_all();
    void refresh_cuts();
    void refresh_selection_fields();
    void refresh_props();
    void refresh_props_thumb();
    void props_changed(bool regenerate_text);
    void refresh_hint();
    void refresh_device_ui();
    void refresh_library();
    void refresh_colour_strip();
    void apply_selection_fields();

    // Tools and commands
    void set_tool(int tool);
    void cmd_import();
    void import_file(const std::string& path);
    void cmd_save_project();
    void cmd_open_project();
    void cmd_offset();
    void cmd_boolean(Laser::BooleanOp op);
    void cmd_weld();
    void cmd_array();
    void cmd_circular_array();
    void cmd_align(int how);
    void cmd_mirror(bool horizontal);
    void cmd_rotate90();
    void cmd_trace();
    void cmd_preview();
    void cmd_frame(bool outline);
    void cmd_start();
    void cmd_save_gcode();
    void edit_layer(int layer);
    void assign_layer(int layer);

    // Device
    void connect_device();
    void disconnect_device();
    void on_streamer_status(const GrblStatus& st);
    void on_streamer_console(const ConsoleLine& line);
    void on_streamer_progress(const JobProgress& p);
    void on_streamer_error(const std::string& text, int line);
    void on_streamer_done(bool completed, const std::string& reason);
    void on_streamer_connection(bool connected, const std::string& error);
    void refresh_ports();
    bool is_simulator_port() const;

    // Planning helpers: plan on a worker thread with a progress dialog; false when cancelled/failed
    // (the error is shown).
    bool plan_job(Laser::LaserJob& job, const wxString& title);
    std::vector<int> job_selection() const;

    // Persistence of UI prefs, devices, materials (data dir JSON).
    void load_prefs();
    void save_prefs();
    void sync_recipe();
    void load_recipe_if_changed();

    // Undo (document snapshots)
    void push_undo();
    void restore(const std::string& blob);

    Laser::LaserDocument m_doc;
    std::vector<int>     m_sel;
    int                  m_current_layer{0};
    std::string          m_synced_blob;   // the Model::laser_recipe this panel last wrote/read
    std::vector<std::string> m_undo;
    int                      m_undo_pos{-1};
    std::vector<Laser::LaserShape> m_clipboard;   // copied shapes, `parent` relative to the batch

    std::vector<Laser::LaserDevice>    m_devices;
    int                                m_device_idx{0};
    std::vector<Laser::MaterialEntry>  m_materials;
    std::array<std::string, 4>         m_macro_names;
    std::array<std::string, 4>         m_macros;
    std::string                        m_last_port;
    GrblStreamer                       m_streamer;
    GrblStatus                         m_status;
    bool                               m_job_running{false};

    // Widgets
    LaserCanvas*        m_canvas{nullptr};
    wxPanel*            m_hint{nullptr};
    wxStaticText*       m_hint_text{nullptr};
    wxStaticText*       m_status_text{nullptr};
    std::vector<wxBitmapButton*> m_tool_buttons;
    std::vector<wxWindow*>       m_swatches;
    wxTextCtrl*         m_fx{nullptr};
    wxTextCtrl*         m_fy{nullptr};
    wxTextCtrl*         m_fw{nullptr};
    wxTextCtrl*         m_fh{nullptr};
    wxTextCtrl*         m_frot{nullptr};
    wxCheckBox*         m_lock_aspect{nullptr};
    wxNotebook*         m_dock{nullptr};
    // Cuts
    wxDataViewListCtrl* m_cuts{nullptr};
    std::vector<int>    m_cut_rows;   // layer per row
    // Laser page
    wxChoice*           m_device_choice{nullptr};
    wxChoice*           m_port_choice{nullptr};
    wxChoice*           m_baud_choice{nullptr};
    wxButton*           m_connect_btn{nullptr};
    wxStaticText*       m_state_text{nullptr};
    wxStaticText*       m_pos_text{nullptr};
    wxGauge*            m_progress{nullptr};
    wxStaticText*       m_progress_text{nullptr};
    wxButton*           m_unlock_btn{nullptr};
    wxChoice*           m_start_from{nullptr};
    std::array<wxRadioButton*, 9> m_origin_radio{};
    wxCheckBox*         m_rotary_check{nullptr};
    wxCheckBox*         m_cut_selected{nullptr};
    wxCheckBox*         m_sel_origin{nullptr};
    // Move page
    wxChoice*           m_jog_dist{nullptr};
    wxSpinCtrlDouble*   m_jog_speed{nullptr};
    wxTextCtrl*         m_goto_x{nullptr};
    wxTextCtrl*         m_goto_y{nullptr};
    wxCheckBox*         m_fire{nullptr};
    wxSpinCtrlDouble*   m_fire_power{nullptr};
    wxStaticText*       m_move_pos{nullptr};
    // Console
    wxListCtrl*         m_console{nullptr};
    wxTextCtrl*         m_console_in{nullptr};
    std::array<wxButton*, 4> m_macro_btn{};
    // Shape properties
    wxScrolledWindow*   m_props{nullptr};
    wxStaticBitmap*     m_props_thumb{nullptr};
    wxTimer             m_props_timer;   // property edits are committed once typing pauses
    // Library
    wxTreeCtrl*         m_library{nullptr};
    // Drawing defaults (Shape Properties with nothing selected)
    std::string         m_text_font;
    double              m_text_height{10};
};

}} // namespace Slic3r::GUI

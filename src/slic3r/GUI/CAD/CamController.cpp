#include "slic3r/GUI/CAD/CamController.hpp"
#include "slic3r/GUI/CAD/CadPropertyManager.hpp"
#include "slic3r/GUI/CAD/CadRibbon.hpp"
#include "slic3r/GUI/CAD/DesignCanvas.hpp"
#include "slic3r/GUI/CAD/DesignPanel.hpp"
#include "slic3r/GUI/CAD/DesignSketchTool.hpp"

#include "libslic3r/CAM/CAM.hpp"
#include "libslic3r/CAM/CamGeometry.hpp"
#include "libslic3r/CAD/GeometryEngine.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"   // put_other_changes
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Utils.hpp"            // data_dir, resources_dir

#include "slic3r/GUI/3DScene.hpp"   // glsafe
#include "slic3r/GUI/Camera.hpp"
#include "slic3r/GUI/GLModel.hpp"
#include "slic3r/GUI/GLShader.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/MainFrame.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/Widgets/Label.hpp"

#include <GL/glew.h>
#include <boost/filesystem/path.hpp>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <nlohmann/json.hpp>

#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/checklst.h>
#include <wx/choice.h>
#include <wx/dialog.h>
#include <wx/filedlg.h>
#include <wx/filename.h>
#include <wx/image.h>
#include <wx/imaglist.h>
#include <wx/listbox.h>
#include <wx/progdlg.h>
#include <wx/menu.h>
#include <wx/msgdlg.h>
#include <wx/radiobut.h>
#include <wx/scrolwin.h>
#include <wx/simplebook.h>
#include <wx/sizer.h>
#include <wx/slider.h>
#include <wx/spinctrl.h>
#include <wx/statline.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/timer.h>
#include <wx/tglbtn.h>
#include <wx/treectrl.h>
#include <wx/utils.h>

#include <algorithm>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <set>

// The CAD workspace is pinned to English (see DesignPanel.cpp): literals, not catalogue lookups.
#ifdef _L
#undef _L
#endif
#define _L(s) wxString::FromUTF8(s)

namespace Slic3r { namespace GUI {

using namespace Slic3r::CAM;
using json = nlohmann::json;

namespace {

constexpr int kOpTypeCount = int(OpType::RotaryFinish) + 1;

uint32_t bit(OpType t) { return 1u << int(t); }
template<class... T> uint32_t bits(T... t) { return (bit(t) | ...); }
constexpr uint32_t kAllOps = 0xFFFFFFFFu;

struct OpInfo {
    OpType      type;
    const char* id;       // mcp / ribbon id
    const char* prefix;   // default name: prefix + number
    const char* icon;
    const char* label;    // ribbon label
    const char* tip;
    const char* hint;     // what to pick
};

const OpInfo kOps[] = {
    {OpType::Face,        "face",          "Face",          "sw_cam_face",          "Face",
     "Face — flatten the top of the stock down to the model top",
     "Face machines the whole stock outline; nothing needs to be picked. Set the depth in Heights."},
    {OpType::Adaptive2D,  "adaptive",      "Adaptive",      "sw_cam_adaptive",      "Adaptive",
     "Adaptive clearing — roughing with a constant tool load (fast, easy on tools)",
     "Pick the floor face(s) of the area to clear, or a closed sketch."},
    {OpType::Pocket2D,    "pocket",        "Pocket",        "sw_cam_pocket",        "Pocket",
     "Pocket — clear a pocket with offset passes",
     "Pick the pocket's floor face(s), or a closed sketch."},
    {OpType::Contour2D,   "contour",       "Contour",       "sw_cam_contour",       "Contour",
     "2D Contour — cut along a profile (outside, inside or on the line)",
     "Pick the bottom face of the profile, its edges, or a sketch."},
    {OpType::Slot,        "slot",          "Slot",          "sw_cam_slot",          "Slot",
     "Slot — tool centre along a line; slot width = tool diameter",
     "Pick the slot's edges or an open sketch line."},
    {OpType::Drill,       "drill",         "Drill",         "sw_cam_drill",         "Drill",
     "Drill — spot, drill, peck or tap holes",
     "Holes are found automatically; or pick hole faces / circular edges."},
    {OpType::Bore,        "bore",          "Bore",          "sw_cam_bore",          "Bore",
     "Bore — helical milling of round holes with an end mill",
     "Pick the hole's cylindrical face or its circular edge."},
    {OpType::Chamfer2D,   "chamfer",       "Chamfer",       "sw_cam_chamfer",       "Chamfer",
     "Chamfer — break edges with a chamfer mill",
     "Pick the edges to chamfer, or the face whose outline gets chamfered."},
    {OpType::Engrave,     "engrave",       "Engrave",       "sw_cam_engrave",       "Engrave",
     "Engrave — V-bit along sketch lines or text",
     "Pick a sketch (text, lines) or edges to engrave."},
    {OpType::Trace,       "trace",         "Trace",         "sw_cam_trace",         "Trace",
     "Trace — tool tip follows picked edges or sketch lines",
     "Pick edges or a sketch to follow."},
    {OpType::Adaptive3D,  "adaptive3d",    "3D Adaptive",   "sw_cam_adaptive3d",    "3D\nAdaptive",
     "3D Adaptive — rough the whole model level by level",
     "Uses the whole model by default."},
    {OpType::Parallel3D,  "parallel3d",    "Parallel",      "sw_cam_parallel3d",    "3D\nParallel",
     "3D Parallel — finish curved surfaces with a ball mill",
     "Uses the whole model by default."},
    {OpType::Contour3D,   "contour3d",     "3D Contour",    "sw_cam_contour3d",     "3D\nContour",
     "3D Contour — finish steep walls with constant-Z passes",
     "Uses the whole model by default."},
    {OpType::RotaryWrap,  "rotary_wrap",   "Rotary Wrap",   "sw_cam_rotary_wrap",   "Rotary\nWrap",
     "Rotary Wrap — a 2D operation wrapped around the A axis",
     "Pick a sketch to wrap around the cylinder."},
    {OpType::RotaryFinish,"rotary_finish", "Rotary Finish", "sw_cam_rotary_finish", "Rotary\nFinish",
     "Rotary Finish — 4-axis finishing around X",
     "Uses the whole model by default."},
};

const OpInfo& op_info(OpType t)
{
    for (const OpInfo& i : kOps)
        if (i.type == t) return i;
    return kOps[0];
}

const OpInfo* op_info_by_id(const std::string& id)
{
    for (const OpInfo& i : kOps)
        if (id == i.id) return &i;
    return nullptr;
}

bool is_3d(OpType t)
{
    return t == OpType::Adaptive3D || t == OpType::Parallel3D || t == OpType::Contour3D || t == OpType::RotaryFinish;
}

const char* kDialectNames[] = {"GRBL", "LinuxCNC", "Mach3 / Mach4", "Fanuc", "Marlin"};
const char* kDialectIds[]   = {"grbl", "linuxcnc", "mach3", "fanuc", "marlin"};
const char* kDialectExt[]   = {"nc", "ngc", "nc", "nc", "gcode"};

wxString fmt_time(double s)
{
    const long t = long(std::lround(std::max(0.0, s)));
    if (t >= 3600) return wxString::Format("%ld:%02ld:%02ld", t / 3600, (t / 60) % 60, t % 60);
    return wxString::Format("%ld:%02ld", t / 60, t % 60);
}

wxString tool_label(const CamTool& t)
{
    return wxString::Format("T%d  %s", t.number,
                            t.name.empty() ? wxString::Format("%g mm %s", t.diameter, tool_type_name(t.type))
                                           : wxString::FromUTF8(t.name));
}

// First tool that suits `type` (the beginner default), else the first tool.
int suggest_tool(const std::vector<CamTool>& tools, OpType type)
{
    auto first = [&](std::initializer_list<ToolType> kinds) {
        for (ToolType k : kinds)
            for (size_t i = 0; i < tools.size(); ++i)
                if (tools[i].type == k) return int(i);
        return -1;
    };
    int i = -1;
    switch (type) {
    case OpType::Drill:        i = first({ToolType::Drill, ToolType::SpotDrill}); break;
    case OpType::Chamfer2D:    i = first({ToolType::ChamferMill, ToolType::VBit}); break;
    case OpType::Engrave:      i = first({ToolType::VBit, ToolType::ChamferMill}); break;
    case OpType::Parallel3D:
    case OpType::RotaryFinish: i = first({ToolType::BallEndMill, ToolType::BullEndMill}); break;
    default:                   i = first({ToolType::FlatEndMill, ToolType::BullEndMill}); break;
    }
    return i >= 0 ? i : (tools.empty() ? -1 : 0);
}

// ---- small control helpers ---------------------------------------------------------------------

// THE WHEEL SCROLLS THE PAGE, IT DOES NOT EDIT THE VALUE (see DesignPanel's make_spin): a spin or a
// choice under the pointer takes the wheel only once it was focused on purpose; otherwise the
// wheel goes to the scrolled page around it.
void wheel_scrolls_page(wxWindow* w)
{
    w->Bind(wxEVT_MOUSEWHEEL, [w](wxMouseEvent& e) {
        if (wxWindow::FindFocus() == w) { e.Skip(); return; }
        for (wxWindow* p = w->GetParent(); p != nullptr; p = p->GetParent())
            if (auto* sw = dynamic_cast<wxScrolledWindow*>(p)) { wxPostEvent(sw, e); return; }
    });
}

wxSpinCtrlDouble* make_spin(wxWindow* parent, double mn, double mx, int digits = 2, double inc = 0.1)
{
    auto* s = new wxSpinCtrlDouble(parent, wxID_ANY, "", wxDefaultPosition, wxSize(parent->FromDIP(96), -1),
                                   wxSP_ARROW_KEYS);
    s->SetRange(mn, mx);
    s->SetDigits(digits);
    s->SetIncrement(inc);
    wheel_scrolls_page(s);
    return s;
}

wxChoice* make_choice(wxWindow* parent, const std::vector<wxString>& items)
{
    auto* c = new wxChoice(parent, wxID_ANY, wxDefaultPosition, wxSize(parent->FromDIP(96), -1));
    for (const wxString& s : items) c->Append(s);
    if (!items.empty()) c->SetSelection(0);
    wheel_scrolls_page(c);
    return c;
}

// A composite image: `icon` with a status dot in its bottom-right corner.
wxBitmap badged_icon(const std::string& icon, const wxColour& dot, wxWindow* win, int px)
{
    wxBitmap bmp = CadRibbon::icon(icon, px, false, win);
    if (!bmp.IsOk()) return bmp;
    wxImage img = bmp.ConvertToImage();
    if (!img.HasAlpha()) img.InitAlpha();
    const int w = img.GetWidth(), h = img.GetHeight();
    const double r = w * 0.24, cx = w - r - 0.5, cy = h - r - 0.5;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const double d = std::hypot(x + 0.5 - cx, y + 0.5 - cy);
            if (d <= r + 1.0) {
                const bool rim = d > r - 0.2;
                img.SetRGB(x, y, rim ? 255 : dot.Red(), rim ? 255 : dot.Green(), rim ? 255 : dot.Blue());
                img.SetAlpha(x, y, 255);
            }
        }
    return wxBitmap(img);
}

// Sampled position along a move, t in [0, 1]. `from` is the previous end point.
Vec3d move_point(const Vec3d& from, const Move& m, double t)
{
    if (!is_arc(m)) return from + (m.to - from) * t;
    const Vec2d c(m.center.x(), m.center.y());
    const Vec2d a(from.x() - c.x(), from.y() - c.y()), b(m.to.x() - c.x(), m.to.y() - c.y());
    const double a0 = std::atan2(a.y(), a.x());
    double       a1 = std::atan2(b.y(), b.x());
    double sweep;
    const bool full = (a - b).norm() < 1e-6;
    if (arc_dir(m) == ArcDir::CCW) {
        sweep = a1 - a0;
        while (sweep <= 1e-9) sweep += 2 * M_PI;
        if (full) sweep = 2 * M_PI;
    } else {
        sweep = a1 - a0;
        while (sweep >= -1e-9) sweep -= 2 * M_PI;
        if (full) sweep = -2 * M_PI;
    }
    const double r0 = a.norm(), r1 = b.norm();
    const double ang = a0 + sweep * t, r = r0 + (r1 - r0) * t;
    return Vec3d(c.x() + r * std::cos(ang), c.y() + r * std::sin(ang), from.z() + (m.to.z() - from.z()) * t);
}

double move_length(const Vec3d& from, const Move& m)
{
    if (!is_arc(m)) return (m.to - from).norm();
    // Chord sum is plenty for timing.
    double len = 0;
    Vec3d  prev = from;
    for (int i = 1; i <= 16; ++i) {
        const Vec3d q = move_point(from, m, i / 16.0);
        len += (q - prev).norm();
        prev = q;
    }
    return len;
}

// The part is rotated by A on the rotary; a machine point at A shows on the unrotated part at
// Rx(-A) * p.
Vec3d part_point(const Vec3d& p, double a_deg)
{
    if (std::abs(a_deg) < 1e-9) return p;
    const double a = -a_deg * M_PI / 180.0, c = std::cos(a), s = std::sin(a);
    return Vec3d(p.x(), c * p.y() - s * p.z(), s * p.y() + c * p.z());
}

} // namespace

// =================================================================================================
// Impl
// =================================================================================================

struct CamController::Impl
{
    explicit Impl(DesignPanel& panel);
    ~Impl();

    DesignPanel& p;
    CamDocument& doc;

    CamModel         model;
    bool             model_dirty{true};
    uint64_t         model_gen{UINT64_MAX};

    std::vector<MachineProfile> machines;
    std::vector<CamTool>        library;
    std::string                 library_path;

    // ---- UI -------------------------------------------------------------------------------------
    wxPanel*            tree_panel{nullptr};
    wxTreeCtrl*         tree{nullptr};
    wxTreeItemId        menu_item;
    CadPropertyManager* pm{nullptr};
    wxPanel*            setup_page{nullptr};
    wxPanel*            op_page{nullptr};
    wxPanel*            simbar{nullptr};
    CadRibbon*          ribbon{nullptr};
    bool                rebuilding{false};

    enum class NodeKind { None, Root, Setup, Op, ToolsFolder, Tool };
    struct Node { NodeKind kind{NodeKind::None}; int index{-1}; int setup{-1}; };
    struct NodeData : wxTreeItemData { explicit NodeData(Node n) : node(n) {} Node node; };
    std::vector<wxTreeItemId> op_items, setup_items;
    int sel_setup{-1}, sel_op{-1};

    enum class Page { None, Setup, Op };
    Page         page{Page::None};
    int          edit_setup{-1};   // -1: new setup
    int          edit_op{-1};      // -1: new operation
    CamSetup     draft_setup;
    CamOperation draft_op;
    bool         pick_active{false};
    bool         pick_wcs{false};

    struct Row { wxWindow* win; wxSizer* group; std::function<bool()> visible; };
    std::vector<Row> setup_rows, op_rows;

    // Setup page
    wxChoice*         s_machine{nullptr};
    wxChoice*         s_material{nullptr};
    wxChoice*         s_stock_kind{nullptr};
    wxSpinCtrlDouble* s_off[6]{};
    wxSpinCtrlDouble* s_cyl_r{nullptr};
    wxSpinCtrlDouble* s_cyl_len{nullptr};
    wxSpinCtrlDouble* s_cyl_extra{nullptr};
    wxSpinCtrlDouble* s_box_min[3]{};
    wxSpinCtrlDouble* s_box_max[3]{};
    wxChoice*         s_wcs_box{nullptr};
    wxRadioButton*    s_wcs_pt[9]{};
    wxRadioButton*    s_wcs_top{nullptr};
    wxRadioButton*    s_wcs_bottom{nullptr};
    wxCheckBox*       s_wcs_custom{nullptr};
    wxSpinCtrlDouble* s_wcs_xyz[3]{};
    wxToggleButton*   s_wcs_pick{nullptr};
    wxChoice*         s_offset{nullptr};
    wxRadioButton*    s_bodies_all{nullptr};
    wxRadioButton*    s_bodies_sel{nullptr};
    wxCheckListBox*   s_bodies{nullptr};
    wxSpinCtrlDouble* s_a_index{nullptr};

    // Operation page
    std::vector<CamTool> tool_list;   // what o_tool shows
    wxChoice*         o_tool{nullptr};
    wxCheckBox*       o_feeds_auto{nullptr};
    wxSpinCtrlDouble* o_rpm{nullptr};
    wxSpinCtrlDouble* o_feed{nullptr};
    wxSpinCtrlDouble* o_plunge{nullptr};
    wxSpinCtrlDouble* o_ramp{nullptr};
    wxStaticText*     o_feeds_info{nullptr};
    wxStaticText*     o_geom_hint{nullptr};
    wxToggleButton*   o_pick{nullptr};
    wxListBox*        o_geom{nullptr};
    wxCheckBox*       o_whole{nullptr};
    wxCheckBox*       o_holes_auto{nullptr};
    wxSpinCtrlDouble* o_hole_min{nullptr};
    wxSpinCtrlDouble* o_hole_max{nullptr};
    wxChoice*         o_href[4]{};
    wxSpinCtrlDouble* o_hoff[4]{};
    wxSpinCtrlDouble *o_stepover{}, *o_stepdown{}, *o_stl_r{}, *o_stl_a{}, *o_finish_so{}, *o_tol{};
    wxCheckBox*       o_climb{nullptr};
    wxChoice*         o_side{nullptr};
    wxSpinCtrl*       o_finish_n{nullptr};
    wxChoice*         o_cycle{nullptr};
    wxSpinCtrlDouble *o_peck{}, *o_dwell{}, *o_break{}, *o_ch_width{}, *o_ch_tip{}, *o_angle{};
    wxChoice*         o_boundary{nullptr};
    wxSpinCtrlDouble *o_wrap_r{}, *o_a_step{};
    wxChoice*         o_wrap_strategy{nullptr};
    wxCheckBox*       o_spiral{nullptr};
    wxCheckBox*       o_bidir{nullptr};
    wxSizer*          o_geom_group{nullptr};
    wxChoice*         o_entry{nullptr};
    wxSpinCtrlDouble *o_ramp_angle{}, *o_helix_d{}, *o_lead_r{};
    wxCheckBox*       o_rest{nullptr};
    wxChoice*         o_order{nullptr};
    wxSpinCtrlDouble *o_lift{}, *o_load{}, *o_min_sd{}, *o_helix_angle{};

    // ---- viewport -------------------------------------------------------------------------------
    struct OpGl { GLModel lines[4]; GLModel warn; };
    std::vector<OpGl> op_gl;
    bool              gl_dirty{true};
    GLModel           stock_faces, stock_edges, triad[3];
    int               stock_setup{-2};
    bool              stock_dirty{true};

    // ---- simulation -----------------------------------------------------------------------------
    struct SimMove { int op; Vec3d from; Move m; double t0, t1; };
    std::vector<SimMove> sim;
    double   sim_t{0}, sim_total{0}, sim_speed{10};
    bool     sim_on{false}, sim_playing{false};
    int      sim_setup{0};
    wxTimer  sim_timer;
    std::chrono::steady_clock::time_point sim_last_tick;
    double   sim_since_mesh{0};
    StockSim stocksim;
    // Copies of the stock taken while cutting forward, every ~5 % of the moves (index = moves cut),
    // so a backward scrub restarts from the nearest one instead of from fresh stock.
    std::map<size_t, StockSim> sim_snapshots;
    size_t   sim_cut_upto{0};   // moves [0, sim_cut_upto) are cut into stocksim
    bool     sim_stock_ok{false};
    TriangleMesh sim_mesh;
    bool     sim_mesh_dirty{false};
    GLModel  sim_mesh_gl;
    GLModel  tool_body, tool_shank, tool_ball;
    int      tool_gl_number{-1};
    bool     bodies_hidden{false};
    wxButton*     sb_play{nullptr};
    wxChoice*     sb_speed{nullptr};
    wxSlider*     sb_slider{nullptr};
    wxStaticText* sb_time{nullptr};
    wxStaticText* sb_info{nullptr};

    // ---- helpers --------------------------------------------------------------------------------
    void status(const wxString& text, bool error = false);
    void sync_recipe();
    void ops_resized();
    bool ensure_model();
    const MachineProfile& machine_of(int setup) const;
    CAM::Material              material_of(int setup) const;
    int  current_setup() const;
    std::string next_op_name(OpType t) const;
    const CamTool* tool_of(const CamOperation& op) const;
    CamTool*       ensure_tool_in_doc(const CamTool& t, int& number_out);
    bool has_bodies() const { return !p.m_doc.bodies.empty(); }

    // tree
    void build_tree();
    void rebuild_tree();
    Node node_of(const wxTreeItemId& id) const;
    void on_tree_selected(Node n);
    void on_tree_activated(Node n);
    void on_tree_menu(Node n, const wxPoint& screen);
    enum class OpState { Ok, Stale, Error, Suppressed };
    OpState op_state(int i) const;

    // ribbon
    void refresh_ribbon_gates();

    // setup page
    void build_setup_page();
    void open_setup(int index);   // -1 = new
    void load_setup_page(const CamSetup& s);
    void read_setup_page(CamSetup& s) const;
    void apply_setup_rows();
    bool confirm_setup();
    int  auto_setup();

    // op page
    void build_op_page();
    void open_op(int index, OpType type_for_new);   // index -1 = new of type
    void load_op_page(const CamOperation& op);
    void read_op_page(CamOperation& op) const;
    void apply_op_rows();
    void fill_tool_choice(int want_number);
    void refresh_feeds();
    void refresh_geom_list();
    bool confirm_op();
    void close_page();
    void show_pm_page(bool pm);   // left pane: CAM tree <-> PropertyManager

    // generation
    bool regenerate(const std::vector<int>& ops, const wxString& what);
    void regenerate_all() { std::vector<int> all(doc.operations.size()); for (size_t i = 0; i < all.size(); ++i) all[i] = int(i); regenerate(all, _L("Generating toolpaths…")); }
    void after_ops_changed();

    // op commands
    void delete_op(int i);
    void duplicate_op(int i);
    void move_op(int i, int delta);
    void toggle_suppress(int i);
    void delete_setup(int s);

    // dialogs
    bool tool_library_dialog();
    struct PostPreset { int dialect{-1}; std::string path; bool show{false}; bool write{false}; int scope{-1}; };
    void post_dialog(const PostPreset* preset = nullptr);
    std::vector<int> post_ops(int scope) const;   // 0 selected op, 1 selected setup, 2 all

    // viewport
    void render();
    void rebuild_gl();
    void rebuild_stock_gl(int setup);
    void render_tool(const Camera& cam);
    void render_sim_stock(const Camera& cam);
    void repaint() { if (p.m_viewport) p.m_viewport->request_repaint(); }

    // simulation
    void build_simbar(wxWindow* parent);
    void sim_open();
    void sim_close();
    void sim_play(bool on);
    void sim_seek(double t);
    void sim_tick();
    void sim_update_stock(bool force);
    int  sim_index(double t) const;
    Vec3d sim_pos(double t, double* a_deg = nullptr, int* idx = nullptr) const;
    void sim_update_labels();
};

// ---- construction -------------------------------------------------------------------------------

CamController::Impl::Impl(DesignPanel& panel) : p(panel), doc(panel.m_cam)
{
    machines = load_machines(resources_dir());
    if (machines.empty()) machines = builtin_machines();
    set_machines(machines);

    library_path = (boost::filesystem::path(data_dir()) / "cam_tools.json").string();
    std::string err;
    if (!load_tool_library(library_path, library, &err) || library.empty())
        library = default_tools();

    // Left pane pages 2 (tree) and 3 (PropertyManager), next to the CAD ones.
    wxSimplebook* book = p.m_left_book;
    tree_panel = new wxPanel(book, wxID_ANY);
    tree_panel->SetBackgroundColour(CadTheme::panel_bg());
    tree = new wxTreeCtrl(tree_panel, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                          wxTR_HAS_BUTTONS | wxTR_SINGLE | wxTR_NO_LINES | wxTR_FULL_ROW_HIGHLIGHT | wxBORDER_NONE);
    tree->SetBackgroundColour(CadTheme::panel_bg());
    tree->SetForegroundColour(CadTheme::text());
    auto* ts = new wxBoxSizer(wxVERTICAL);
    ts->Add(tree, 1, wxEXPAND);
    tree_panel->SetSizer(ts);
    pm = new CadPropertyManager(book);
    book->AddPage(tree_panel, wxEmptyString);
    book->AddPage(pm, wxEmptyString);
    pm->set_on_ok([this] { if (page == Page::Setup) confirm_setup(); else if (page == Page::Op) confirm_op(); });
    pm->set_on_cancel([this] { close_page(); status(_L("Cancelled.")); });
    pm->set_on_group_toggled([this] { if (page == Page::Setup) apply_setup_rows(); else apply_op_rows(); });

    auto* body_sizer = new wxBoxSizer(wxVERTICAL);
    pm->body()->SetSizer(body_sizer);
    build_setup_page();
    build_op_page();
    body_sizer->Add(setup_page, 0, wxEXPAND);
    body_sizer->Add(op_page, 0, wxEXPAND);
    setup_page->Hide();
    op_page->Hide();

    build_tree();
    build_simbar(&p);

    sim_timer.SetOwner(simbar);
    simbar->Bind(wxEVT_TIMER, [this](wxTimerEvent&) { sim_tick(); });

    DesignSketchTool& st = p.m_viewport->mcp_sketch_tool();
    st.on_render_overlay = [this] { render(); };
    st.overlay_on        = false;
    ops_resized();
    rebuild_tree();
}

CamController::Impl::~Impl()
{
    sim_timer.Stop();
    if (p.m_viewport) {
        p.m_viewport->mcp_sketch_tool().on_render_overlay = nullptr;
        p.m_viewport->mcp_sketch_tool().overlay_on        = false;
    }
}

void CamController::Impl::status(const wxString& text, bool error)
{
    if (p.m_status) p.m_status->SetForegroundColour(error ? wxColour(235, 110, 110) : wxNullColour);
    p.set_status(text);
    if (page != Page::None)
        pm->set_message(text, error ? wxColour(235, 110, 110) : wxNullColour);
}

void CamController::Impl::sync_recipe()
{
    Plater* plater = wxGetApp().plater();
    if (plater == nullptr) return;
    std::string blob = (doc.setups.empty() && doc.operations.empty()) ? std::string() : doc.serialize();
    if (blob == plater->model().cam_recipe) return;
    plater->model().cam_recipe = std::move(blob);
    Slic3r::put_other_changes();
}

void CamController::Impl::ops_resized()
{
    if (doc.paths.size() != doc.operations.size()) doc.paths.resize(doc.operations.size());
    gl_dirty = true;
}

bool CamController::Impl::ensure_model()
{
    const uint64_t gen = p.m_doc.topo_generation;
    if (!model_dirty && gen == model_gen) return true;
    wxBusyCursor busy;
    try {
        std::vector<TopoDS_Shape> shapes;
        shapes.reserve(p.m_doc.bodies.size());
        for (const CadBody& b : p.m_doc.bodies) shapes.push_back(b.shape);
        model = build_cam_model(shapes, 0.02, 0.2);
        model.topo_generation = gen;
        for (int i = 0; i < int(p.m_doc.features.size()); ++i) {
            const CadFeature& f = p.m_doc.features[i];
            if (f.type == CadFeatureType::Sketch && f.enabled && !f.entities.empty())
                add_cam_sketch(model, i, f.entities, f.plane);
        }
        update_setup_frames(model, doc);
    } catch (const Standard_Failure& e) {
        status(_L("CAM could not read the model: ") + wxString::FromUTF8(e.GetMessageString() ? e.GetMessageString() : "OCCT failure"), true);
        return false;
    } catch (const std::exception& e) {
        status(_L("CAM could not read the model: ") + wxString::FromUTF8(e.what()), true);
        return false;
    }
    model_dirty = false;
    model_gen   = gen;
    stock_dirty = true;
    return true;
}

const MachineProfile& CamController::Impl::machine_of(int setup) const
{
    return find_machine(setup >= 0 && setup < int(doc.setups.size()) ? doc.setups[setup].machine : std::string());
}

CAM::Material CamController::Impl::material_of(int setup) const
{
    return setup >= 0 && setup < int(doc.setups.size()) ? doc.setups[setup].material : CAM::Material::Aluminum;
}

int CamController::Impl::current_setup() const
{
    if (sel_setup >= 0 && sel_setup < int(doc.setups.size())) return sel_setup;
    if (sel_op >= 0 && sel_op < int(doc.operations.size())) return doc.operations[sel_op].setup_index;
    return doc.setups.empty() ? -1 : 0;
}

std::string CamController::Impl::next_op_name(OpType t) const
{
    const std::string prefix = op_info(t).prefix;
    int n = 0;
    for (const CamOperation& op : doc.operations)
        if (op.name.rfind(prefix, 0) == 0) {
            const std::string rest = op.name.substr(prefix.size());
            if (!rest.empty() && std::all_of(rest.begin(), rest.end(), ::isdigit)) n = std::max(n, std::atoi(rest.c_str()));
        }
    return prefix + std::to_string(n + 1);
}

const CamTool* CamController::Impl::tool_of(const CamOperation& op) const { return doc.find_tool(op.tool_number); }

CamTool* CamController::Impl::ensure_tool_in_doc(const CamTool& t, int& number_out)
{
    for (CamTool& d : doc.tools)
        if (d.number == t.number && d.name == t.name && d.type == t.type && d.diameter == t.diameter) {
            number_out = d.number;
            return &d;
        }
    const int idx = doc.add_tool(t);
    if (idx < 0 || idx >= int(doc.tools.size())) { number_out = t.number; return nullptr; }
    number_out = doc.tools[idx].number;
    return &doc.tools[idx];
}

// =================================================================================================
// Tree
// =================================================================================================

void CamController::Impl::build_tree()
{
    tree->Bind(wxEVT_TREE_SEL_CHANGED, [this](wxTreeEvent& e) {
        if (!rebuilding) on_tree_selected(node_of(e.GetItem()));
    });
    tree->Bind(wxEVT_TREE_ITEM_ACTIVATED, [this](wxTreeEvent& e) { on_tree_activated(node_of(e.GetItem())); });
    // Menu on the right-button RELEASE (see CadFeatureTree: a menu opened on the press takes
    // the release as a pick).
    tree->Bind(wxEVT_TREE_ITEM_MENU, [this](wxTreeEvent& e) {
        if (!e.GetItem().IsOk()) return;
        tree->SelectItem(e.GetItem());
        if (wxGetMouseState().RightIsDown()) { menu_item = e.GetItem(); return; }
        wxRect r;
        tree->GetBoundingRect(e.GetItem(), r, true);
        const Node n = node_of(e.GetItem());
        const wxPoint at = tree->ClientToScreen(r.GetBottomLeft());
        tree->CallAfter([this, n, at] { on_tree_menu(n, at); });
    });
    tree->Bind(wxEVT_RIGHT_UP, [this](wxMouseEvent& e) {
        e.Skip();
        if (!menu_item.IsOk()) return;
        const Node n = node_of(menu_item);
        menu_item = wxTreeItemId();
        const wxPoint at = tree->ClientToScreen(e.GetPosition());
        tree->CallAfter([this, n, at] { on_tree_menu(n, at); });
    });
}

CamController::Impl::Node CamController::Impl::node_of(const wxTreeItemId& id) const
{
    if (!id.IsOk()) return {};
    const auto* d = static_cast<const NodeData*>(tree->GetItemData(id));
    return d ? d->node : Node{};
}

CamController::Impl::OpState CamController::Impl::op_state(int i) const
{
    if (i < 0 || i >= int(doc.operations.size())) return OpState::Stale;
    if (!doc.operations[i].enabled) return OpState::Suppressed;
    if (i >= int(doc.paths.size())) return OpState::Stale;
    if (doc.is_stale(i)) return OpState::Stale;
    return doc.paths[i].ok() ? OpState::Ok : OpState::Error;
}

void CamController::Impl::rebuild_tree()
{
    rebuilding = true;
    tree->Freeze();
    menu_item = wxTreeItemId();
    tree->DeleteAllItems();
    op_items.assign(doc.operations.size(), wxTreeItemId());
    setup_items.assign(doc.setups.size(), wxTreeItemId());

    const int px = 16;
    auto* images = new wxImageList(tree->FromDIP(px), tree->FromDIP(px));
    std::map<std::string, int> index;
    auto img = [&](const std::string& key, const std::function<wxBitmap()>& make) {
        auto it = index.find(key);
        if (it != index.end()) return it->second;
        const int i = images->Add(make());
        index.emplace(key, i);
        return i;
    };
    auto plain = [&](const std::string& name) { return img(name, [&] { return CadRibbon::icon(name, px, false, tree); }); };

    const int i_root = plain("sw_cam_root");
    const wxTreeItemId root = tree->AddRoot(_L("CAM"), i_root, i_root, new NodeData({NodeKind::Root, -1, -1}));
    for (int s = 0; s < int(doc.setups.size()); ++s) {
        const CamSetup& su = doc.setups[s];
        const int i_setup = plain("sw_cam_setup");
        const wxString label = wxString::Format("%s (%s · %s)", wxString::FromUTF8(su.name), wxString::FromUTF8(su.machine),
                                                material_name(su.material));
        const wxTreeItemId sid = tree->AppendItem(root, label, i_setup, i_setup, new NodeData({NodeKind::Setup, s, s}));
        setup_items[s] = sid;
        std::vector<int> tools_used;
        double total = 0;
        for (int o = 0; o < int(doc.operations.size()); ++o) {
            const CamOperation& op = doc.operations[o];
            if (op.setup_index != s) continue;
            const OpState st = op_state(o);
            const wxColour dot = st == OpState::Ok ? wxColour(0x3F, 0xA3, 0x4D)
                               : st == OpState::Stale ? wxColour(0xE8, 0xA2, 0x1C)
                               : st == OpState::Error ? wxColour(0xD6, 0x3B, 0x30) : wxColour(0x9A, 0x9A, 0x9A);
            const std::string icon = op_info(op.type).icon;
            const int i_op = img(icon + "#" + std::to_string(int(st)), [&] { return badged_icon(icon, dot, tree, px); });
            wxString label = wxString::Format("[T%d] %s", op.tool_number, wxString::FromUTF8(op.name));
            const Toolpath* tp = o < int(doc.paths.size()) ? &doc.paths[o] : nullptr;
            switch (st) {
            case OpState::Ok:         label += "   " + wxString::FromUTF8("\xE2\x8F\xB1 ") + fmt_time(tp->time_s); total += tp->time_s; break;
            case OpState::Stale:      label += "   " + wxString::FromUTF8("\xE2\x9A\xA0 regenerate"); break;
            case OpState::Error:      label += "   " + wxString::FromUTF8("\xE2\x9C\x97 error"); break;
            case OpState::Suppressed: label += "   (suppressed)"; break;
            }
            const wxTreeItemId oid = tree->AppendItem(sid, label, i_op, i_op, new NodeData({NodeKind::Op, o, s}));
            tree->SetItemTextColour(oid, st == OpState::Suppressed ? CadTheme::text_dim()
                                       : st == OpState::Error      ? wxColour(0xD6, 0x3B, 0x30)
                                                                   : CadTheme::text());
            if (st == OpState::Error && tp) tree->SetItemTextColour(oid, wxColour(0xD6, 0x3B, 0x30));
            op_items[o] = oid;
            if (std::find(tools_used.begin(), tools_used.end(), op.tool_number) == tools_used.end())
                tools_used.push_back(op.tool_number);
        }
        if (total > 0) tree->SetItemText(sid, label + "   " + wxString::FromUTF8("\xE2\x8F\xB1 ") + fmt_time(total));
        if (!tools_used.empty()) {
            const int i_folder = plain("sw_tree_folder");
            const wxTreeItemId fid = tree->AppendItem(sid, _L("Tools"), i_folder, i_folder, new NodeData({NodeKind::ToolsFolder, -1, s}));
            std::sort(tools_used.begin(), tools_used.end());
            const int i_tool = plain("sw_cam_tool");
            for (int n : tools_used) {
                const CamTool* t = doc.find_tool(n);
                const wxString tl = t ? wxString::Format("T%d — %s", n, t->name.empty() ? wxString::Format("%g mm %s", t->diameter, tool_type_name(t->type))
                                                                                        : wxString::FromUTF8(t->name))
                                      : wxString::Format("T%d — (missing tool)", n);
                tree->AppendItem(fid, tl, i_tool, i_tool, new NodeData({NodeKind::Tool, n, s}));
            }
        }
        tree->Expand(sid);
    }
    tree->AssignImageList(images);
    tree->Expand(root);
    if (sel_op >= 0 && sel_op < int(op_items.size()) && op_items[sel_op].IsOk()) tree->SelectItem(op_items[sel_op]);
    else if (sel_setup >= 0 && sel_setup < int(setup_items.size())) tree->SelectItem(setup_items[sel_setup]);
    tree->Thaw();
    rebuilding = false;
}

void CamController::Impl::on_tree_selected(Node n)
{
    sel_op    = n.kind == NodeKind::Op ? n.index : -1;
    sel_setup = n.kind == NodeKind::Setup ? n.index : -1;
    stock_dirty = true;
    if (sel_op >= 0) {
        const CamOperation& op = doc.operations[sel_op];
        const Toolpath&     tp = doc.paths[sel_op];
        wxString msg = wxString::FromUTF8(op.name) + ": ";
        switch (op_state(sel_op)) {
        case OpState::Ok:
            msg += wxString::Format(_L("%s · %.0f mm cutting · %.0f mm rapid"), fmt_time(tp.time_s), tp.cut_length, tp.rapid_length);
            if (!tp.warnings.empty()) msg += "  ·  " + wxString::FromUTF8(tp.warnings.front().text);
            break;
        case OpState::Stale:      msg += _L("needs regenerating (right-click → Regenerate, or Regenerate All)."); break;
        case OpState::Error:      msg += wxString::FromUTF8(tp.error); break;
        case OpState::Suppressed: msg += _L("suppressed."); break;
        }
        status(msg, op_state(sel_op) == OpState::Error);
    }
    repaint();
}

void CamController::Impl::on_tree_activated(Node n)
{
    if (n.kind == NodeKind::Op) open_op(n.index, OpType::Face);
    else if (n.kind == NodeKind::Setup) open_setup(n.index);
    else if (n.kind == NodeKind::Tool || n.kind == NodeKind::ToolsFolder) tool_library_dialog();
}

void CamController::Impl::on_tree_menu(Node n, const wxPoint& screen)
{
    wxMenu menu;
    std::vector<std::function<void()>> acts;
    auto add = [&](const wxString& label, std::function<void()> f, bool enabled = true) {
        const int id = 2000 + int(acts.size());
        menu.Append(id, label);
        menu.Enable(id, enabled);
        acts.push_back(std::move(f));
    };
    switch (n.kind) {
    case NodeKind::Op: {
        const int i = n.index;
        add(_L("Edit"), [this, i] { open_op(i, OpType::Face); });
        add(doc.operations[i].enabled ? _L("Suppress") : _L("Unsuppress"), [this, i] { toggle_suppress(i); });
        add(_L("Regenerate"), [this, i] { regenerate({i}, _L("Generating toolpath…")); });
        add(_L("Duplicate"), [this, i] { duplicate_op(i); });
        menu.AppendSeparator();
        add(_L("Move Up"), [this, i] { move_op(i, -1); }, i > 0 && doc.operations[i - 1].setup_index == doc.operations[i].setup_index);
        add(_L("Move Down"), [this, i] { move_op(i, +1); },
            i + 1 < int(doc.operations.size()) && doc.operations[i + 1].setup_index == doc.operations[i].setup_index);
        menu.AppendSeparator();
        add(_L("Simulate"), [this] { sim_open(); });
        add(_L("Post Process (selected)…"), [this] { PostPreset pp; pp.scope = 0; post_dialog(&pp); });
        menu.AppendSeparator();
        add(_L("Delete"), [this, i] { delete_op(i); });
        break;
    }
    case NodeKind::Setup: {
        const int s = n.index;
        add(_L("Edit Setup"), [this, s] { open_setup(s); });
        add(_L("Regenerate Setup"), [this, s] {
            std::vector<int> ops;
            for (int i = 0; i < int(doc.operations.size()); ++i) if (doc.operations[i].setup_index == s) ops.push_back(i);
            regenerate(ops, _L("Generating toolpaths…"));
        });
        add(_L("Simulate"), [this] { sim_open(); });
        add(_L("Post Process…"), [this] { PostPreset pp; pp.scope = 1; post_dialog(&pp); });
        menu.AppendSeparator();
        add(_L("Delete Setup"), [this, s] { delete_setup(s); });
        break;
    }
    case NodeKind::Root:
        add(_L("New Setup"), [this] { open_setup(-1); });
        add(_L("Regenerate All"), [this] { regenerate_all(); }, !doc.operations.empty());
        add(_L("Post Process All…"), [this] { PostPreset pp; pp.scope = 2; post_dialog(&pp); }, !doc.operations.empty());
        break;
    case NodeKind::ToolsFolder:
    case NodeKind::Tool:
        add(_L("Tool Library…"), [this] { tool_library_dialog(); });
        break;
    default: return;
    }
    int chosen = -1;
    menu.Bind(wxEVT_MENU, [&chosen](wxCommandEvent& e) { chosen = e.GetId() - 2000; });
    tree->PopupMenu(&menu, tree->ScreenToClient(screen));
    if (chosen >= 0 && chosen < int(acts.size())) acts[chosen]();
}

// =================================================================================================
// Ribbon
// =================================================================================================

void CamController::Impl::refresh_ribbon_gates()
{
    if (ribbon == nullptr) return;
    const bool bodies = has_bodies();
    for (const OpInfo& i : kOps) ribbon->enable(std::string("cam_") + i.id, bodies);
    ribbon->enable("cam_engrave_menu", bodies);
    ribbon->enable("cam_rotary", bodies);
    ribbon->enable("cam_setup", bodies);
    const bool ops = !doc.operations.empty();
    ribbon->enable("cam_regenerate", ops);
    ribbon->enable("cam_simulate", ops);
    ribbon->enable("cam_post", ops);
}

// =================================================================================================
// Setup page
// =================================================================================================

// A labelled row of a PropertyManager page: [label | controls]. The row panel is what shows/hides.
static wxPanel* make_row(wxWindow* page, const wxString& label, const std::vector<wxWindow*>& ctrls, int label_w)
{
    auto* r = new wxPanel(page, wxID_ANY);
    r->SetBackgroundColour(CadTheme::panel_bg());
    auto* s = new wxBoxSizer(wxHORIZONTAL);
    if (!label.empty()) {
        auto* l = new wxStaticText(r, wxID_ANY, label, wxDefaultPosition, wxSize(label_w, -1), wxST_ELLIPSIZE_END | wxST_NO_AUTORESIZE);
        l->SetForegroundColour(CadTheme::text());
        l->SetToolTip(label);
        s->Add(l, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, r->FromDIP(4));
    }
    for (wxWindow* c : ctrls) {
        c->Reparent(r);
        s->Add(c, dynamic_cast<wxButton*>(c) ? 0 : 1, wxALIGN_CENTER_VERTICAL | wxRIGHT, r->FromDIP(2));
    }
    r->SetSizer(s);
    return r;
}

void CamController::Impl::build_setup_page()
{
    setup_page = new wxPanel(pm->body(), wxID_ANY);
    setup_page->SetBackgroundColour(CadTheme::panel_bg());
    wxWindow* pg = setup_page;
    const int lw = pg->FromDIP(110);
    auto* root = new wxBoxSizer(wxVERTICAL);
    auto group = [&](const wxString& title, bool expanded) {
        auto* content = new wxBoxSizer(wxVERTICAL);
        wxSizer* g = pm->make_group(pg, title, content, expanded);
        root->Add(g, 0, wxEXPAND);
        return std::make_pair(g, content);
    };
    auto add_row = [&](std::pair<wxSizer*, wxBoxSizer*> g, const wxString& label, std::vector<wxWindow*> ctrls,
                       std::function<bool()> vis = nullptr) {
        wxPanel* r = make_row(pg, label, ctrls, lw);
        g.second->Add(r, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, pg->FromDIP(4));
        setup_rows.push_back({r, g.first, vis ? vis : [] { return true; }});
        return r;
    };

    // Machine & material
    auto gm = group(_L("Machine and Material"), true);
    std::vector<wxString> mnames;
    for (const MachineProfile& m : machines) mnames.push_back(wxString::FromUTF8(m.name));
    s_machine = make_choice(pg, mnames);
    add_row(gm, _L("Machine"), {s_machine});
    std::vector<wxString> mats;
    for (int i = 0; i < kMaterialCount; ++i) mats.push_back(material_name(CAM::Material(i)));
    s_material = make_choice(pg, mats);
    add_row(gm, _L("Material"), {s_material});
    s_machine->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) { apply_setup_rows(); });

    // Stock
    auto gs = group(_L("Stock"), true);
    s_stock_kind = make_choice(pg, {_L("Box around the model"), _L("Cylinder (along X)"), _L("Custom box")});
    add_row(gs, _L("Shape"), {s_stock_kind});
    s_stock_kind->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) { apply_setup_rows(); });
    auto is_kind = [this](int k) { return [this, k] { return s_stock_kind->GetSelection() == k; }; };
    const char* off_names[6] = {"Left (−X)", "Right (+X)", "Front (−Y)", "Back (+Y)", "Bottom (−Z)", "Top (+Z)"};
    for (int i = 0; i < 6; ++i) {
        s_off[i] = make_spin(pg, 0, 1000, 2, 0.5);
        add_row(gs, _L(off_names[i]) + _L(" mm"), {s_off[i]}, is_kind(0));
    }
    s_cyl_r   = make_spin(pg, 0, 5000, 2, 0.5);
    s_cyl_len = make_spin(pg, 0, 5000, 2, 1);
    s_cyl_extra = make_spin(pg, 0, 1000, 2, 0.5);
    add_row(gs, _L("Radius (0 = auto)"), {s_cyl_r}, is_kind(1));
    add_row(gs, _L("Length (0 = auto)"), {s_cyl_len}, is_kind(1));
    add_row(gs, _L("Extra radius mm"), {s_cyl_extra}, is_kind(1));
    for (int i = 0; i < 3; ++i) { s_box_min[i] = make_spin(pg, -10000, 10000, 2, 1); s_box_max[i] = make_spin(pg, -10000, 10000, 2, 1); }
    add_row(gs, _L("Min X / Y / Z"), {s_box_min[0], s_box_min[1], s_box_min[2]}, is_kind(2));
    add_row(gs, _L("Max X / Y / Z"), {s_box_max[0], s_box_max[1], s_box_max[2]}, is_kind(2));

    // WCS
    auto gw = group(_L("Work Coordinate System (WCS)"), true);
    s_wcs_box = make_choice(pg, {_L("Stock box"), _L("Model box")});
    add_row(gw, _L("Origin on"), {s_wcs_box});
    {
        // The 9 box points seen from above: back row first, front row last.
        auto* gridp = new wxPanel(pg, wxID_ANY);
        gridp->SetBackgroundColour(CadTheme::panel_bg());
        auto* grid = new wxGridSizer(3, 3, gridp->FromDIP(2), gridp->FromDIP(10));
        const WcsPoint order[9] = {WcsPoint::BackLeft, WcsPoint::Back, WcsPoint::BackRight, WcsPoint::Left, WcsPoint::Center,
                                   WcsPoint::Right, WcsPoint::FrontLeft, WcsPoint::Front, WcsPoint::FrontRight};
        const char* tips[9] = {"Back left", "Back", "Back right", "Left", "Centre", "Right", "Front left", "Front", "Front right"};
        for (int k = 0; k < 9; ++k) {
            auto* rb = new wxRadioButton(gridp, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, k == 0 ? wxRB_GROUP : 0);
            rb->SetToolTip(_L(tips[k]));
            s_wcs_pt[int(order[k])] = rb;
            grid->Add(rb, 0, wxALIGN_CENTER);
        }
        gridp->SetSizer(grid);
        auto* zp = new wxPanel(pg, wxID_ANY);
        zp->SetBackgroundColour(CadTheme::panel_bg());
        auto* zs = new wxBoxSizer(wxVERTICAL);
        s_wcs_top    = new wxRadioButton(zp, wxID_ANY, _L("Top"), wxDefaultPosition, wxDefaultSize, wxRB_GROUP);
        s_wcs_bottom = new wxRadioButton(zp, wxID_ANY, _L("Bottom"));
        s_wcs_top->SetForegroundColour(CadTheme::text());
        s_wcs_bottom->SetForegroundColour(CadTheme::text());
        zs->Add(s_wcs_top, 0, wxBOTTOM, zp->FromDIP(2));
        zs->Add(s_wcs_bottom, 0);
        zp->SetSizer(zs);
        add_row(gw, _L("Origin point"), {gridp, zp}, [this] { return !s_wcs_custom->GetValue(); });
    }
    s_wcs_custom = new wxCheckBox(pg, wxID_ANY, _L("Custom point"));
    s_wcs_custom->SetForegroundColour(CadTheme::text());
    s_wcs_pick = new wxToggleButton(pg, wxID_ANY, _L("Pick"));
    s_wcs_pick->SetToolTip(_L("Click a face (its centre) or a circular edge (its centre) in the viewport"));
    add_row(gw, wxEmptyString, {s_wcs_custom, s_wcs_pick});
    s_wcs_custom->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent&) { apply_setup_rows(); });
    s_wcs_pick->Bind(wxEVT_TOGGLEBUTTON, [this](wxCommandEvent&) {
        pick_wcs = s_wcs_pick->GetValue();
        if (pick_wcs) { s_wcs_custom->SetValue(true); apply_setup_rows(); status(_L("Click a face or a round edge for the WCS origin.")); }
    });
    for (int i = 0; i < 3; ++i) s_wcs_xyz[i] = make_spin(pg, -10000, 10000, 3, 0.5);
    add_row(gw, _L("X / Y / Z"), {s_wcs_xyz[0], s_wcs_xyz[1], s_wcs_xyz[2]}, [this] { return s_wcs_custom->GetValue(); });
    s_offset = make_choice(pg, {"G54", "G55", "G56", "G57", "G58", "G59"});
    add_row(gw, _L("Work offset"), {s_offset});

    // Bodies
    auto gb = group(_L("Model"), true);
    s_bodies_all = new wxRadioButton(pg, wxID_ANY, _L("All bodies"), wxDefaultPosition, wxDefaultSize, wxRB_GROUP);
    s_bodies_sel = new wxRadioButton(pg, wxID_ANY, _L("Selected bodies"));
    s_bodies_all->SetForegroundColour(CadTheme::text());
    s_bodies_sel->SetForegroundColour(CadTheme::text());
    add_row(gb, wxEmptyString, {s_bodies_all, s_bodies_sel});
    s_bodies = new wxCheckListBox(pg, wxID_ANY, wxDefaultPosition, wxSize(-1, pg->FromDIP(70)));
    add_row(gb, wxEmptyString, {s_bodies}, [this] { return s_bodies_sel->GetValue(); });
    s_bodies_all->Bind(wxEVT_RADIOBUTTON, [this](wxCommandEvent&) { apply_setup_rows(); });
    s_bodies_sel->Bind(wxEVT_RADIOBUTTON, [this](wxCommandEvent&) { apply_setup_rows(); });

    // 4th axis
    auto ga = group(_L("4th Axis"), true);
    s_a_index = make_spin(pg, -360, 360, 1, 15);
    add_row(ga, _L("A index angle °"), {s_a_index}, [this] {
        const int m = s_machine->GetSelection();
        return m >= 0 && m < int(machines.size()) && machines[m].has_a_axis;
    });
    setup_page->SetSizer(root);
}

void CamController::Impl::apply_setup_rows()
{
    if (page != Page::Setup) return;
    for (const Row& r : setup_rows) r.win->Show(r.visible() && pm->is_expanded(r.group));
    setup_page->Layout();
    pm->body()->Layout();
    pm->body()->FitInside();
}

void CamController::Impl::load_setup_page(const CamSetup& s)
{
    int mi = 0;
    for (int i = 0; i < int(machines.size()); ++i) if (machines[i].name == s.machine) mi = i;
    s_machine->SetSelection(mi);
    s_material->SetSelection(int(s.material));
    s_stock_kind->SetSelection(int(s.stock.kind));
    const double offs[6] = {s.stock.offset_neg.x(), s.stock.offset_pos.x(), s.stock.offset_neg.y(),
                            s.stock.offset_pos.y(), s.stock.offset_neg.z(), s.stock.offset_pos.z()};
    for (int i = 0; i < 6; ++i) s_off[i]->SetValue(offs[i]);
    s_cyl_r->SetValue(s.stock.radius);
    s_cyl_len->SetValue(s.stock.length);
    s_cyl_extra->SetValue(std::max(s.stock.offset_pos.y(), s.stock.offset_pos.z()));
    for (int i = 0; i < 3; ++i) { s_box_min[i]->SetValue(s.stock.box_min[i]); s_box_max[i]->SetValue(s.stock.box_max[i]); }
    s_wcs_box->SetSelection(int(s.wcs.box));
    s_wcs_pt[int(s.wcs.point)]->SetValue(true);
    (s.wcs.z == WcsZ::Top ? s_wcs_top : s_wcs_bottom)->SetValue(true);
    s_wcs_custom->SetValue(s.wcs.custom);
    for (int i = 0; i < 3; ++i) s_wcs_xyz[i]->SetValue(s.wcs.custom_point[i]);
    s_wcs_pick->SetValue(false);
    pick_wcs = false;
    s_offset->SetSelection(std::clamp(s.work_offset - 54, 0, 5));
    s_bodies->Clear();
    for (int b = 0; b < int(p.m_doc.bodies.size()); ++b) {
        const CadBody& cb = p.m_doc.bodies[b];
        s_bodies->Append(cb.has_user_name ? wxString::FromUTF8(cb.user_name) : wxString::Format(_L("Body %d"), b + 1));
        s_bodies->Check(b, std::find(s.body_ids.begin(), s.body_ids.end(), b) != s.body_ids.end());
    }
    (s.body_ids.empty() ? s_bodies_all : s_bodies_sel)->SetValue(true);
    s_a_index->SetValue(s.a_index_deg);
}

void CamController::Impl::read_setup_page(CamSetup& s) const
{
    const int mi = s_machine->GetSelection();
    if (mi >= 0 && mi < int(machines.size())) s.machine = machines[mi].name;
    s.material   = CAM::Material(std::max(0, s_material->GetSelection()));
    s.stock.kind = StockKind(std::max(0, s_stock_kind->GetSelection()));
    if (s.stock.kind == StockKind::Box) {
        s.stock.offset_neg = Vec3d(s_off[0]->GetValue(), s_off[2]->GetValue(), s_off[4]->GetValue());
        s.stock.offset_pos = Vec3d(s_off[1]->GetValue(), s_off[3]->GetValue(), s_off[5]->GetValue());
    } else if (s.stock.kind == StockKind::Cylinder) {
        s.stock.radius = s_cyl_r->GetValue();
        s.stock.length = s_cyl_len->GetValue();
        s.stock.offset_pos.y() = s.stock.offset_pos.z() = s_cyl_extra->GetValue();
    } else {
        for (int i = 0; i < 3; ++i) { s.stock.box_min[i] = s_box_min[i]->GetValue(); s.stock.box_max[i] = s_box_max[i]->GetValue(); }
    }
    s.wcs.box = WcsBox(std::max(0, s_wcs_box->GetSelection()));
    for (int k = 0; k < 9; ++k) if (s_wcs_pt[k]->GetValue()) s.wcs.point = WcsPoint(k);
    s.wcs.z      = s_wcs_top->GetValue() ? WcsZ::Top : WcsZ::Bottom;
    s.wcs.custom = s_wcs_custom->GetValue();
    s.wcs.custom_point = Vec3d(s_wcs_xyz[0]->GetValue(), s_wcs_xyz[1]->GetValue(), s_wcs_xyz[2]->GetValue());
    s.work_offset = 54 + std::max(0, s_offset->GetSelection());
    s.body_ids.clear();
    if (s_bodies_sel->GetValue())
        for (unsigned b = 0; b < s_bodies->GetCount(); ++b) if (s_bodies->IsChecked(b)) s.body_ids.push_back(int(b));
    const bool four = mi >= 0 && mi < int(machines.size()) && machines[mi].has_a_axis;
    s.a_index_deg = four ? s_a_index->GetValue() : 0.0;
}

void CamController::Impl::open_setup(int index)
{
    if (sim_on) sim_close();
    close_page();
    ensure_model();
    edit_setup = index;
    if (index >= 0 && index < int(doc.setups.size())) {
        draft_setup = doc.setups[index];
    } else {
        edit_setup  = -1;
        draft_setup = CamSetup();
        int n = int(doc.setups.size()) + 1;
        draft_setup.name = "Setup" + std::to_string(n);
        if (!doc.setups.empty()) {   // a second setup: same machine/material, flipped
            draft_setup.machine  = doc.setups.back().machine;
            draft_setup.material = doc.setups.back().material;
        }
    }
    page = Page::Setup;
    load_setup_page(draft_setup);
    pm->set_title(wxString::FromUTF8(draft_setup.name), "sw_cam_setup");
    op_page->Hide();
    setup_page->Show();
    pm->reapply_collapsed();
    apply_setup_rows();
    show_pm_page(true);
    status(edit_setup < 0 ? _L("New setup: choose the machine and material, check the stock and where X0 Y0 Z0 is, then ✓.")
                          : _L("Edit the setup, then ✓. Its operations will need regenerating."));
}

bool CamController::Impl::confirm_setup()
{
    read_setup_page(draft_setup);
    int idx = edit_setup;
    if (idx >= 0 && idx < int(doc.setups.size())) {
        doc.setups[idx] = draft_setup;
        for (int i = 0; i < int(doc.operations.size()); ++i)
            if (doc.operations[i].setup_index == idx) doc.invalidate(i);
    } else {
        idx = doc.add_setup(draft_setup);
    }
    ensure_model();
    try { update_setup_frames(model, doc); } catch (...) {}
    sel_setup = idx; sel_op = -1;
    stock_dirty = gl_dirty = true;
    close_page();
    after_ops_changed();
    status(wxString::Format(_L("%s ready. Next: pick an operation in the CAM ribbon (Adaptive or Pocket to clear material, Contour for walls)."),
                            wxString::FromUTF8(doc.setups[idx].name)));
    return true;
}

int CamController::Impl::auto_setup()
{
    CamSetup s;   // struct defaults: box stock +1 mm sides and top, 0 below; WCS top-front-left of the stock
    s.name = "Setup" + std::to_string(doc.setups.size() + 1);
    s.machine = machines.empty() ? s.machine : machines.front().name;
    const int idx = doc.add_setup(s);
    ensure_model();
    try { update_setup_frames(model, doc); } catch (...) {}
    stock_dirty = true;
    return idx;
}

// =================================================================================================
// Operation page
// =================================================================================================

void CamController::Impl::build_op_page()
{
    op_page = new wxPanel(pm->body(), wxID_ANY);
    op_page->SetBackgroundColour(CadTheme::panel_bg());
    wxWindow* pg = op_page;
    const int lw = pg->FromDIP(120);
    auto* root = new wxBoxSizer(wxVERTICAL);
    auto group = [&](const wxString& title, bool expanded) {
        auto* content = new wxBoxSizer(wxVERTICAL);
        wxSizer* g = pm->make_group(pg, title, content, expanded);
        root->Add(g, 0, wxEXPAND);
        return std::make_pair(g, content);
    };
    auto add_row = [&](std::pair<wxSizer*, wxBoxSizer*> g, const wxString& label, std::vector<wxWindow*> ctrls, uint32_t mask,
                       std::function<bool()> extra = nullptr) {
        wxPanel* r = make_row(pg, label, ctrls, lw);
        g.second->Add(r, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, pg->FromDIP(4));
        op_rows.push_back({r, g.first, [this, mask, extra] { return (mask & bit(draft_op.type)) != 0 && (!extra || extra()); }});
        return r;
    };
    using T = OpType;

    // Tool
    auto gt = group(_L("Tool"), true);
    o_tool = make_choice(pg, {});
    auto* lib = new wxButton(pg, wxID_ANY, _L("Library…"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    lib->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
        const int keep = o_tool->GetSelection() >= 0 && o_tool->GetSelection() < int(tool_list.size()) ? tool_list[o_tool->GetSelection()].number : -1;
        if (tool_library_dialog()) { fill_tool_choice(keep); refresh_feeds(); }
    });
    add_row(gt, _L("Tool"), {o_tool, lib}, kAllOps);
    o_tool->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) { refresh_feeds(); });
    o_feeds_auto = new wxCheckBox(pg, wxID_ANY, _L("Feeds && speeds from the material (auto)"));
    o_feeds_auto->SetForegroundColour(CadTheme::text());
    add_row(gt, wxEmptyString, {o_feeds_auto}, kAllOps);
    o_feeds_auto->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent&) { refresh_feeds(); });
    o_rpm    = make_spin(pg, 0, 100000, 0, 100);
    o_feed   = make_spin(pg, 0, 100000, 0, 10);
    o_plunge = make_spin(pg, 0, 100000, 0, 10);
    o_ramp   = make_spin(pg, 0, 100000, 0, 10);
    add_row(gt, _L("Spindle RPM"), {o_rpm}, kAllOps);
    add_row(gt, _L("Cutting feed mm/min"), {o_feed}, kAllOps);
    add_row(gt, _L("Plunge feed mm/min"), {o_plunge}, kAllOps);
    add_row(gt, _L("Ramp feed mm/min"), {o_ramp}, kAllOps);
    o_feeds_info = new wxStaticText(pg, wxID_ANY, wxEmptyString);
    o_feeds_info->SetForegroundColour(CadTheme::text_dim());
    add_row(gt, wxEmptyString, {o_feeds_info}, kAllOps);

    // Geometry
    auto gg = group(_L("Geometry"), true);
    o_geom_group = gg.first;
    o_geom_hint = new wxStaticText(pg, wxID_ANY, wxEmptyString);
    o_geom_hint->SetForegroundColour(CadTheme::text_dim());
    add_row(gg, wxEmptyString, {o_geom_hint}, kAllOps);
    o_pick = new wxToggleButton(pg, wxID_ANY, _L("Pick in viewport"));
    o_pick->SetToolTip(_L("While on, clicking a face, an edge or a sketch adds it (click again to remove)"));
    auto* clear = new wxButton(pg, wxID_ANY, _L("Clear"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    add_row(gg, wxEmptyString, {o_pick, clear}, ~bit(T::Face));
    o_pick->Bind(wxEVT_TOGGLEBUTTON, [this](wxCommandEvent&) {
        pick_active = o_pick->GetValue();
        if (p.m_viewport) p.m_viewport->set_escalate_on_repick(!pick_active);
    });
    clear->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
        draft_op.geom.faces.clear(); draft_op.geom.edges.clear(); draft_op.geom.points.clear();
        draft_op.geom.sketch_feature = -1;
        refresh_geom_list();
    });
    o_geom = new wxListBox(pg, wxID_ANY, wxDefaultPosition, wxSize(-1, pg->FromDIP(76)));
    add_row(gg, wxEmptyString, {o_geom}, ~bit(T::Face));
    o_geom->Bind(wxEVT_KEY_DOWN, [this](wxKeyEvent& e) {   // Delete removes the highlighted entry
        if (e.GetKeyCode() != WXK_DELETE && e.GetKeyCode() != WXK_BACK) { e.Skip(); return; }
        int k = o_geom->GetSelection();
        if (k < 0) return;
        GeometrySelection& g = draft_op.geom;
        if (k < int(g.faces.size())) { g.faces.erase(g.faces.begin() + k); refresh_geom_list(); return; }
        k -= int(g.faces.size());
        if (k < int(g.edges.size())) { g.edges.erase(g.edges.begin() + k); refresh_geom_list(); return; }
        k -= int(g.edges.size());
        if (g.sketch_feature >= 0) { if (k == 0) { g.sketch_feature = -1; refresh_geom_list(); return; } --k; }
        if (k < int(g.points.size())) { g.points.erase(g.points.begin() + k); refresh_geom_list(); }
    });
    o_whole = new wxCheckBox(pg, wxID_ANY, _L("Whole model (every body of the setup)"));
    o_whole->SetForegroundColour(CadTheme::text());
    add_row(gg, wxEmptyString, {o_whole}, bits(T::Adaptive3D, T::Parallel3D, T::Contour3D, T::RotaryFinish));
    o_holes_auto = new wxCheckBox(pg, wxID_ANY, _L("Holes: auto-detect"));
    o_holes_auto->SetForegroundColour(CadTheme::text());
    add_row(gg, wxEmptyString, {o_holes_auto}, bits(T::Drill, T::Bore));
    o_holes_auto->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent&) { apply_op_rows(); });
    o_hole_min = make_spin(pg, 0, 1000, 2, 0.5);
    o_hole_max = make_spin(pg, 0, 1000, 2, 0.5);
    add_row(gg, _L("Hole diameter from / to (0 = any)"), {o_hole_min, o_hole_max}, bits(T::Drill, T::Bore));

    // Heights
    auto gh = group(_L("Heights"), true);
    const std::vector<wxString> refs = {_L("Stock top"), _L("Stock bottom"), _L("Model top"), _L("Model bottom"),
                                        _L("Selection top"), _L("Selection bottom"), _L("Absolute Z")};
    const char* hnames[4] = {"Clearance", "Retract", "Top", "Bottom"};
    for (int i = 0; i < 4; ++i) {
        o_href[i] = make_choice(pg, refs);
        o_hoff[i] = make_spin(pg, -10000, 10000, 2, 0.5);
        add_row(gh, wxString::Format(_L("%s  + offset"), _L(hnames[i])), {o_href[i], o_hoff[i]}, kAllOps);
    }

    // Passes
    auto gp = group(_L("Passes"), true);
    const uint32_t cutting = bits(T::Face, T::Adaptive2D, T::Pocket2D, T::Contour2D, T::Slot, T::Bore, T::Engrave, T::Trace,
                                  T::Adaptive3D, T::Contour3D, T::RotaryWrap);
    o_stepover = make_spin(pg, 0.001, 1000, 3, 0.1);
    add_row(gp, _L("Stepover mm"), {o_stepover}, bits(T::Face, T::Adaptive2D, T::Pocket2D, T::Adaptive3D, T::Parallel3D, T::RotaryWrap));
    o_stepdown = make_spin(pg, 0.001, 1000, 3, 0.1);
    add_row(gp, _L("Stepdown mm"), {o_stepdown}, cutting);
    o_stl_r = make_spin(pg, -100, 100, 3, 0.05);
    add_row(gp, _L("Stock to leave, walls mm"), {o_stl_r},
            bits(T::Adaptive2D, T::Pocket2D, T::Contour2D, T::Adaptive3D, T::Parallel3D, T::Contour3D, T::RotaryFinish));
    o_stl_a = make_spin(pg, -100, 100, 3, 0.05);
    add_row(gp, _L("Stock to leave, floors mm"), {o_stl_a},
            bits(T::Face, T::Adaptive2D, T::Pocket2D, T::Contour2D, T::Adaptive3D, T::Parallel3D, T::Contour3D, T::RotaryFinish));
    o_climb = new wxCheckBox(pg, wxID_ANY, _L("Climb milling"));
    o_climb->SetForegroundColour(CadTheme::text());
    add_row(gp, wxEmptyString, {o_climb}, bits(T::Adaptive2D, T::Pocket2D, T::Contour2D, T::Slot, T::Adaptive3D, T::Contour3D));
    o_side = make_choice(pg, {_L("Outside"), _L("Inside"), _L("On the line")});
    add_row(gp, _L("Side"), {o_side}, bits(T::Contour2D, T::Chamfer2D));
    o_finish_n = new wxSpinCtrl(pg, wxID_ANY, "", wxDefaultPosition, wxSize(pg->FromDIP(96), -1), wxSP_ARROW_KEYS, 0, 20, 0);
    wheel_scrolls_page(o_finish_n);
    add_row(gp, _L("Finishing passes"), {o_finish_n}, bits(T::Pocket2D, T::Contour2D));
    o_finish_so = make_spin(pg, 0, 100, 3, 0.05);
    add_row(gp, _L("Finishing stepover mm"), {o_finish_so}, bits(T::Pocket2D, T::Contour2D));
    o_cycle = make_choice(pg, {_L("Drill (G81)"), _L("Peck (G83)"), _L("Chip break (G73)"), _L("Bore (G85)"), _L("Tap (G84)")});
    add_row(gp, _L("Cycle"), {o_cycle}, bit(T::Drill));
    o_peck = make_spin(pg, 0, 1000, 2, 0.5);
    add_row(gp, _L("Peck depth mm"), {o_peck}, bit(T::Drill));
    o_dwell = make_spin(pg, 0, 60, 2, 0.1);
    add_row(gp, _L("Dwell s"), {o_dwell}, bit(T::Drill));
    o_break = make_spin(pg, 0, 100, 2, 0.1);
    add_row(gp, _L("Break-through mm"), {o_break}, bits(T::Drill, T::Bore));
    o_ch_width = make_spin(pg, 0, 100, 2, 0.1);
    add_row(gp, _L("Chamfer width mm"), {o_ch_width}, bit(T::Chamfer2D));
    o_ch_tip = make_spin(pg, 0, 100, 2, 0.1);
    add_row(gp, _L("Tip offset mm"), {o_ch_tip}, bit(T::Chamfer2D));
    o_angle = make_spin(pg, -360, 360, 1, 5);
    add_row(gp, _L("Pass direction °"), {o_angle}, bits(T::Face, T::Parallel3D));
    o_boundary = make_choice(pg, {_L("Stock"), _L("Model silhouette"), _L("Selection")});
    add_row(gp, _L("Boundary"), {o_boundary}, bit(T::Parallel3D));
    o_wrap_strategy = make_choice(pg, {_L("Engrave"), _L("Pocket"), _L("Contour"), _L("Trace")});
    add_row(gp, _L("Wrapped operation"), {o_wrap_strategy}, bit(T::RotaryWrap));
    o_wrap_r = make_spin(pg, 0, 5000, 2, 0.5);
    add_row(gp, _L("Wrap radius (0 = stock)"), {o_wrap_r}, bit(T::RotaryWrap));
    o_a_step = make_spin(pg, 0.01, 90, 2, 0.5);
    add_row(gp, _L("A stepover °"), {o_a_step}, bit(T::RotaryFinish));
    o_spiral = new wxCheckBox(pg, wxID_ANY, _L("Spiral instead of passes"));
    o_spiral->SetForegroundColour(CadTheme::text());
    add_row(gp, wxEmptyString, {o_spiral}, bit(T::RotaryFinish));
    o_bidir = new wxCheckBox(pg, wxID_ANY, _L("Cut in both directions (zig-zag)"));
    o_bidir->SetForegroundColour(CadTheme::text());
    add_row(gp, wxEmptyString, {o_bidir}, bits(T::Parallel3D, T::RotaryFinish));
    o_tol = make_spin(pg, 0.0001, 1, 4, 0.005);
    add_row(gp, _L("Tolerance mm"), {o_tol}, ~bit(T::Drill));

    // Linking
    auto gl = group(_L("Linking"), true);
    o_entry = make_choice(pg, {_L("Helix"), _L("Ramp"), _L("Plunge")});
    add_row(gl, _L("Entry"), {o_entry}, bits(T::Adaptive2D, T::Pocket2D, T::Slot, T::Adaptive3D));
    o_ramp_angle = make_spin(pg, 0.1, 90, 1, 0.5);
    add_row(gl, _L("Ramp angle °"), {o_ramp_angle}, bits(T::Adaptive2D, T::Pocket2D, T::Slot, T::Adaptive3D, T::Contour2D));
    o_helix_d = make_spin(pg, 0, 1000, 2, 0.5);
    add_row(gl, _L("Helix diameter (0 = auto)"), {o_helix_d}, bits(T::Adaptive2D, T::Pocket2D, T::Adaptive3D, T::Bore));
    o_lead_r = make_spin(pg, 0, 100, 2, 0.1);
    add_row(gl, _L("Lead-in radius mm"), {o_lead_r}, bits(T::Contour2D, T::Chamfer2D, T::Contour3D));

    // Advanced
    auto ga = group(_L("Advanced"), false);
    o_rest = new wxCheckBox(pg, wxID_ANY, _L("Rest machining (only what earlier tools left)"));
    o_rest->SetForegroundColour(CadTheme::text());
    add_row(ga, wxEmptyString, {o_rest}, bits(T::Adaptive2D, T::Pocket2D, T::Adaptive3D));
    o_order = make_choice(pg, {_L("Depth first"), _L("Level first")});
    add_row(ga, _L("Ordering"), {o_order}, bits(T::Adaptive2D, T::Pocket2D, T::Contour2D, T::Adaptive3D, T::Contour3D));
    o_load = make_spin(pg, 0.01, 100, 3, 0.05);
    add_row(ga, _L("Optimal load mm"), {o_load}, bits(T::Adaptive2D, T::Adaptive3D));
    o_lift = make_spin(pg, 0, 100, 2, 0.05);
    add_row(ga, _L("Lift height mm"), {o_lift}, bits(T::Adaptive2D, T::Adaptive3D));
    o_min_sd = make_spin(pg, 0, 100, 3, 0.05);
    add_row(ga, _L("Minimum stepdown mm"), {o_min_sd}, bit(T::Adaptive3D));
    o_helix_angle = make_spin(pg, 0.1, 45, 1, 0.5);
    add_row(ga, _L("Helix angle °"), {o_helix_angle}, bits(T::Adaptive2D, T::Adaptive3D, T::Bore, T::Pocket2D));

    op_page->SetSizer(root);
}

void CamController::Impl::apply_op_rows()
{
    if (page != Page::Op) return;
    for (const Row& r : op_rows) r.win->Show(r.visible() && pm->is_expanded(r.group));
    op_page->Layout();
    pm->body()->Layout();
    pm->body()->FitInside();
}

void CamController::Impl::fill_tool_choice(int want_number)
{
    tool_list = doc.tools;
    for (const CamTool& t : library) {
        const bool dup = std::any_of(tool_list.begin(), tool_list.end(), [&](const CamTool& d) {
            return d.number == t.number && d.name == t.name && d.type == t.type && d.diameter == t.diameter;
        });
        if (!dup) tool_list.push_back(t);
    }
    o_tool->Clear();
    int sel = -1;
    for (int i = 0; i < int(tool_list.size()); ++i) {
        o_tool->Append(tool_label(tool_list[i]));
        if (sel < 0 && tool_list[i].number == want_number) sel = i;
    }
    if (sel < 0) sel = tool_list.empty() ? -1 : 0;
    o_tool->SetSelection(sel);
}

void CamController::Impl::refresh_feeds()
{
    const int ti = o_tool->GetSelection();
    const bool automatic = o_feeds_auto->GetValue();
    for (wxSpinCtrlDouble* s : {o_rpm, o_feed, o_plunge, o_ramp}) s->Enable(!automatic);
    if (ti < 0 || ti >= int(tool_list.size())) { o_feeds_info->SetLabel(_L("Add a tool in the Tool Library.")); return; }
    const CamTool tool = tool_list[ti];
    // New operation: rescale the tool-dependent defaults (stepover, stepdown, heights) to the tool.
    if (edit_op < 0 && tool.number != draft_op.tool_number) {
        CamOperation d = default_operation(draft_op.type, &tool);
        d.geom = draft_op.geom; d.name = draft_op.name; d.setup_index = draft_op.setup_index;
        d.tool_number = tool.number;
        d.feeds_auto  = o_feeds_auto->GetValue();
        draft_op = d;
        load_op_page(draft_op);
    }
    const int      setup = std::clamp(draft_op.setup_index, 0, std::max(0, int(doc.setups.size()) - 1));
    FeedsSpeeds fs;
    try {
        CamOperation probe = draft_op;
        probe.feeds_auto   = true;
        fs = effective_feeds(probe, tool, material_of(setup), machine_of(setup));
    } catch (...) {}
    if (automatic) {
        o_rpm->SetValue(fs.rpm);
        o_feed->SetValue(fs.feed);
        o_plunge->SetValue(fs.plunge_feed);
        o_ramp->SetValue(fs.ramp_feed);
    }
    o_feeds_info->SetLabel(wxString::Format(_L("%s · chip load %.3f mm/tooth · %d flutes"), material_name(material_of(setup)),
                                            fs.chipload, tool.flutes));
}

void CamController::Impl::refresh_geom_list()
{
    o_geom->Clear();
    const GeometrySelection& g = draft_op.geom;
    auto body_name = [this](int b) {
        if (b >= 0 && b < int(p.m_doc.bodies.size()) && p.m_doc.bodies[b].has_user_name) return wxString::FromUTF8(p.m_doc.bodies[b].user_name);
        return wxString::Format(_L("Body %d"), b + 1);
    };
    for (const FaceRef& f : g.faces) o_geom->Append(wxString::Format(_L("Face %d · %s"), f.face, body_name(f.body)));
    for (const EdgeRef& e : g.edges) o_geom->Append(wxString::Format(_L("Edge %d · %s"), e.edge, body_name(e.body)));
    if (g.sketch_feature >= 0 && g.sketch_feature < int(p.m_doc.features.size()))
        o_geom->Append(wxString::FromUTF8(p.m_doc.features[g.sketch_feature].name));
    for (const Vec3d& pt : g.points) o_geom->Append(wxString::Format(_L("Point %.2f, %.2f, %.2f"), pt.x(), pt.y(), pt.z()));
    const size_t n = g.faces.size() + g.edges.size() + g.points.size() + (g.sketch_feature >= 0 ? 1 : 0);
    if (n == 0) o_geom->Append(_L("(nothing picked yet)"));
}

void CamController::Impl::load_op_page(const CamOperation& op)
{
    fill_tool_choice(op.tool_number);
    o_feeds_auto->SetValue(op.feeds_auto);
    if (!op.feeds_auto) {
        o_rpm->SetValue(op.feeds.rpm); o_feed->SetValue(op.feeds.feed);
        o_plunge->SetValue(op.feeds.plunge_feed); o_ramp->SetValue(op.feeds.ramp_feed);
    }
    o_geom_hint->SetLabel(CadPropertyManager::wrap_text(o_geom_hint, _L(op_info(op.type).hint), op_page->FromDIP(250)));
    o_whole->SetValue(op.geom.whole_model);
    o_holes_auto->SetValue(op.geom.whole_model);
    o_hole_min->SetValue(op.hole_diameter_min);
    o_hole_max->SetValue(op.hole_diameter_max);
    o_bidir->SetValue(op.bidirectional);
    const Height* hs[4] = {&op.heights.clearance, &op.heights.retract, &op.heights.top, &op.heights.bottom};
    for (int i = 0; i < 4; ++i) { o_href[i]->SetSelection(int(hs[i]->ref)); o_hoff[i]->SetValue(hs[i]->offset); }
    o_stepover->SetValue(op.stepover);
    o_stepdown->SetValue(op.stepdown);
    o_stl_r->SetValue(op.stock_to_leave_radial);
    o_stl_a->SetValue(op.stock_to_leave_axial);
    o_climb->SetValue(op.climb);
    o_side->SetSelection(int(op.side));
    o_finish_n->SetValue(op.finishing_passes);
    o_finish_so->SetValue(op.finish_stepover);
    o_tol->SetValue(op.tolerance);
    o_cycle->SetSelection(int(op.cycle));
    o_peck->SetValue(op.peck_depth);
    o_dwell->SetValue(op.dwell_s);
    o_break->SetValue(op.break_through);
    o_ch_width->SetValue(op.chamfer_width);
    o_ch_tip->SetValue(op.tip_offset);
    o_angle->SetValue(op.angle_deg);
    o_boundary->SetSelection(int(op.boundary));
    const OpType wraps[4] = {OpType::Engrave, OpType::Pocket2D, OpType::Contour2D, OpType::Trace};
    int ws = 0;
    for (int i = 0; i < 4; ++i) if (wraps[i] == op.wrap_strategy) ws = i;
    o_wrap_strategy->SetSelection(ws);
    o_wrap_r->SetValue(op.wrap_radius);
    o_a_step->SetValue(op.a_stepover_deg);
    o_spiral->SetValue(op.rotary_spiral);
    o_entry->SetSelection(int(op.entry));
    o_ramp_angle->SetValue(op.ramp_angle_deg);
    o_helix_d->SetValue(op.helix_diameter);
    o_lead_r->SetValue(op.lead_in_radius);
    o_rest->SetValue(op.rest_machining);
    o_order->SetSelection(int(op.ordering));
    o_load->SetValue(op.optimal_load);
    o_lift->SetValue(op.lift_height);
    o_min_sd->SetValue(op.min_stepdown);
    o_helix_angle->SetValue(op.helix_angle_deg);
    refresh_geom_list();
}

void CamController::Impl::read_op_page(CamOperation& op) const
{
    const int ti = o_tool->GetSelection();
    if (ti >= 0 && ti < int(tool_list.size())) op.tool_number = tool_list[ti].number;
    op.feeds_auto = o_feeds_auto->GetValue();
    op.feeds.rpm = o_rpm->GetValue(); op.feeds.feed = o_feed->GetValue();
    op.feeds.plunge_feed = o_plunge->GetValue(); op.feeds.ramp_feed = o_ramp->GetValue();
    if (is_3d(op.type)) op.geom.whole_model = o_whole->GetValue();
    else if (op.type == OpType::Drill || op.type == OpType::Bore) op.geom.whole_model = o_holes_auto->GetValue();
    else op.geom.whole_model = false;
    Height* hs[4] = {&op.heights.clearance, &op.heights.retract, &op.heights.top, &op.heights.bottom};
    for (int i = 0; i < 4; ++i) { hs[i]->ref = HeightRef(std::max(0, o_href[i]->GetSelection())); hs[i]->offset = o_hoff[i]->GetValue(); }
    op.stepover = o_stepover->GetValue();
    op.stepdown = o_stepdown->GetValue();
    op.stock_to_leave_radial = o_stl_r->GetValue();
    op.stock_to_leave_axial  = o_stl_a->GetValue();
    op.climb = o_climb->GetValue();
    op.side  = ContourSide(std::max(0, o_side->GetSelection()));
    op.finishing_passes = o_finish_n->GetValue();
    op.finish_stepover  = o_finish_so->GetValue();
    op.tolerance        = o_tol->GetValue();
    op.cycle            = DrillCycle(std::max(0, o_cycle->GetSelection()));
    op.peck_depth       = o_peck->GetValue();
    op.dwell_s          = o_dwell->GetValue();
    op.break_through    = o_break->GetValue();
    op.chamfer_width    = o_ch_width->GetValue();
    op.tip_offset       = o_ch_tip->GetValue();
    op.angle_deg        = o_angle->GetValue();
    op.boundary         = Boundary(std::max(0, o_boundary->GetSelection()));
    const OpType wraps[4] = {OpType::Engrave, OpType::Pocket2D, OpType::Contour2D, OpType::Trace};
    op.wrap_strategy    = wraps[std::clamp(o_wrap_strategy->GetSelection(), 0, 3)];
    op.wrap_radius      = o_wrap_r->GetValue();
    op.a_stepover_deg   = o_a_step->GetValue();
    op.rotary_spiral    = o_spiral->GetValue();
    op.entry            = EntryType(std::max(0, o_entry->GetSelection()));
    op.ramp_angle_deg   = o_ramp_angle->GetValue();
    op.helix_diameter   = o_helix_d->GetValue();
    op.lead_in_radius   = o_lead_r->GetValue();
    op.rest_machining   = o_rest->GetValue();
    op.ordering         = CutOrdering(std::max(0, o_order->GetSelection()));
    op.optimal_load     = o_load->GetValue();
    op.lift_height      = o_lift->GetValue();
    op.min_stepdown     = o_min_sd->GetValue();
    op.helix_angle_deg  = o_helix_angle->GetValue();
    op.hole_diameter_min = o_hole_min->GetValue();
    op.hole_diameter_max = o_hole_max->GetValue();
    op.bidirectional     = o_bidir->GetValue();
}

void CamController::Impl::open_op(int index, OpType type_for_new)
{
    if (sim_on) sim_close();
    close_page();
    if (!has_bodies()) { status(_L("CAM needs a model: build or import a solid in the Modeling tab first."), true); return; }
    ensure_model();
    wxString next;
    if (index >= 0 && index < int(doc.operations.size())) {
        edit_op  = index;
        draft_op = doc.operations[index];
    } else {
        edit_op = -1;
        // Beginner default: the first operation brings its Setup with it.
        int s = current_setup();
        bool made_setup = false;
        if (s < 0) { s = auto_setup(); made_setup = true; sel_setup = s; rebuild_tree(); }
        std::vector<CamTool> all = doc.tools;
        for (const CamTool& t : library) all.push_back(t);
        const int ti = suggest_tool(all, type_for_new);
        draft_op = ti >= 0 ? default_operation(type_for_new, &all[ti]) : default_operation(type_for_new);
        draft_op.name        = next_op_name(type_for_new);
        draft_op.setup_index = s;
        draft_op.feeds_auto  = true;
        if (ti >= 0) draft_op.tool_number = all[ti].number;
        if (is_3d(type_for_new)) draft_op.geom.whole_model = true;
        if (type_for_new == OpType::Drill) draft_op.geom.whole_model = true;
        if (made_setup)
            next = wxString::Format(_L("Setup1 made from the model (box stock +1 mm, X0 Y0 Z0 at the top front left corner). "));
    }
    page = Page::Op;
    load_op_page(draft_op);
    refresh_feeds();
    pm->set_title(wxString::FromUTF8(draft_op.name), op_info(draft_op.type).icon);
    setup_page->Hide();
    op_page->Show();
    pm->reapply_collapsed();
    apply_op_rows();
    const bool needs_pick = draft_op.type != OpType::Face && !draft_op.geom.whole_model;
    o_pick->SetValue(needs_pick);
    pick_active = needs_pick;
    if (p.m_viewport) p.m_viewport->set_escalate_on_repick(!pick_active);
    show_pm_page(true);
    status(next + _L(op_info(draft_op.type).hint) + _L(" Then ✓ to calculate."));
}

bool CamController::Impl::confirm_op()
{
    read_op_page(draft_op);
    // The chosen tool goes into the document (the post lists the document's tools).
    const int ti = o_tool->GetSelection();
    if (ti < 0 || ti >= int(tool_list.size())) { status(_L("Choose a tool first (Tool Library… adds one)."), true); return false; }
    int number = draft_op.tool_number;
    ensure_tool_in_doc(tool_list[ti], number);
    draft_op.tool_number = number;
    int idx = edit_op;
    if (idx >= 0 && idx < int(doc.operations.size())) {
        doc.operations[idx] = draft_op;
    } else {
        idx = doc.add_operation(draft_op);
        if (idx < 0) { status(_L("Could not add the operation (no setup?)."), true); return false; }
    }
    ops_resized();
    sel_op = idx; sel_setup = -1;
    edit_op = idx;   // a failed calculation leaves the page open on the stored operation
    regenerate({idx}, _L("Calculating ") + wxString::FromUTF8(draft_op.name) + _L("…"));
    if (doc.is_stale(idx)) {   // cancelled: the operation is stored, its toolpath is not
        status(wxString::FromUTF8(draft_op.name) + _L(": calculation cancelled. ✓ calculates it again."), true);
        return false;
    }
    const Toolpath& tp = doc.paths[idx];
    if (!tp.ok()) {
        status(wxString::FromUTF8(draft_op.name) + ": " + wxString::FromUTF8(tp.error), true);
        return false;   // page stays open: fix and ✓ again
    }
    close_page();
    wxString msg = wxString::Format(_L("%s: %s machining · %.0f mm of cutting."), wxString::FromUTF8(draft_op.name), fmt_time(tp.time_s), tp.cut_length);
    if (!tp.warnings.empty()) msg += "  " + wxString::FromUTF8(tp.warnings.front().text);
    msg += _L("  Next: another operation, Simulate, or Post Process.");
    status(msg, false);
    return true;
}

void CamController::Impl::close_page()
{
    page        = Page::None;
    pick_active = false;
    pick_wcs    = false;
    if (p.m_viewport) p.m_viewport->set_escalate_on_repick(true);
    setup_page->Hide();
    op_page->Hide();
    pm->set_message(wxString(), wxNullColour);
    show_pm_page(false);
}

// =================================================================================================
// Generation and op commands
// =================================================================================================

bool CamController::Impl::regenerate(const std::vector<int>& ops, const wxString& what)
{
    if (ops.empty()) return true;
    if (!ensure_model()) return false;
    try { update_setup_frames(model, doc); } catch (...) {}
    ops_resized();

    // Generation runs on a worker thread; the kernel's progress callback only stores the fraction
    // and reads the cancel flag (atomics), and this thread shows them. The document is read-only
    // meanwhile: input is disabled and the results are stored after the worker has joined.
    std::vector<std::pair<int, Toolpath>> results;
    std::atomic<double> fraction{0};
    std::atomic<bool>   cancel{false}, done{false};
    std::thread worker([&] {
        for (size_t k = 0; k < ops.size() && !cancel; ++k) {
            const int i = ops[k];
            if (i < 0 || i >= int(doc.operations.size())) continue;
            const CAM::ProgressFn progress = [&, k](double f) {
                fraction = (double(k) + f) / double(ops.size());
                return cancel.load();
            };
            Toolpath tp;
            try {
                tp = generate_toolpath(doc, i, model, progress);
            } catch (const Standard_Failure& e) {
                tp.error = std::string("Geometry kernel failure: ") + (e.GetMessageString() ? e.GetMessageString() : "OCCT");
            } catch (const std::exception& e) {
                tp.error = e.what();
            }
            results.emplace_back(i, std::move(tp));
        }
        done = true;
    });
    {
        wxWindowDisabler disabler;
        std::unique_ptr<wxProgressDialog> dlg;
        int elapsed_ms = 0;
        while (!done) {
            // Only a slow calculation gets a dialog, so a quick one does not flash.
            if (dlg == nullptr && elapsed_ms >= 300)
                dlg = std::make_unique<wxProgressDialog>(_L("CAM"), what, 1000, &p,
                                                         wxPD_CAN_ABORT | wxPD_ELAPSED_TIME | wxPD_AUTO_HIDE | wxPD_SMOOTH);
            if (dlg != nullptr) {
                if (!dlg->Update(std::clamp(int(fraction * 1000), 0, 999))) cancel = true;
            } else {
                wxYield();
            }
            wxMilliSleep(30);
            elapsed_ms += 30;
        }
    }
    worker.join();

    bool all_ok = true;
    std::vector<int> repick;
    for (auto& [i, tp] : results) {
        if (cancel && tp.error == "Cancelled") { doc.invalidate(i); all_ok = false; continue; }   // keeps the old path, marked stale
        const Toolpath& old = doc.paths[i];
        // Picked face / edge ids name the topology they were picked on. An operation that worked on
        // an older model and fails now most likely points at faces that have moved on.
        const GeometrySelection& g = doc.operations[i].geom;
        if (!tp.ok() && old.generation != 0 && old.generation != doc.model_generation && old.ok()
            && (!g.faces.empty() || !g.edges.empty()))
            repick.push_back(i);
        all_ok = all_ok && tp.ok();
        doc.paths[i] = std::move(tp);
    }
    gl_dirty = true;
    after_ops_changed();
    if (cancel) status(_L("Cancelled: the operations that were not finished still need regenerating."), true);
    if (!repick.empty() && page == Page::None) {
        const int i = repick.front();
        const std::string err = doc.paths[i].error;
        sel_op = i; sel_setup = -1;
        rebuild_tree();
        open_op(i, OpType::Face);
        draft_op.geom.faces.clear();
        draft_op.geom.edges.clear();
        refresh_geom_list();
        pm->expand(o_geom_group);
        apply_op_rows();
        status(_L("Re-pick the faces for this operation: the model changed, so the faces picked before are not the same "
                  "faces any more (") + wxString::FromUTF8(err) + ")", true);
    }
    return all_ok;
}

void CamController::Impl::after_ops_changed()
{
    ops_resized();
    rebuild_tree();
    refresh_ribbon_gates();
    sync_recipe();
    repaint();
}

void CamController::Impl::delete_op(int i)
{
    if (i < 0 || i >= int(doc.operations.size())) return;
    doc.remove_operation(i);
    sel_op = -1;
    after_ops_changed();
}

void CamController::Impl::duplicate_op(int i)
{
    const int n = doc.duplicate_operation(i);
    if (n < 0) return;
    ops_resized();
    doc.operations[n].name = next_op_name(doc.operations[n].type);
    sel_op = n;
    after_ops_changed();
}

void CamController::Impl::move_op(int i, int delta)
{
    const int n = doc.move_operation(i, i + delta);
    if (n < 0) return;
    sel_op = n;
    after_ops_changed();
}

void CamController::Impl::toggle_suppress(int i)
{
    if (i < 0 || i >= int(doc.operations.size())) return;
    doc.operations[i].enabled = !doc.operations[i].enabled;
    after_ops_changed();
}

void CamController::Impl::delete_setup(int s)
{
    if (s < 0 || s >= int(doc.setups.size())) return;
    wxMessageDialog dlg(&p, wxString::Format(_L("Delete %s and its operations?"), wxString::FromUTF8(doc.setups[s].name)),
                        _L("Delete Setup"), wxYES_NO | wxICON_QUESTION);
    if (dlg.ShowModal() != wxID_YES) return;
    doc.remove_setup(s);
    sel_setup = sel_op = -1;
    try { update_setup_frames(model, doc); } catch (...) {}
    stock_dirty = true;
    after_ops_changed();
}

// =================================================================================================
// Tool Library dialog
// =================================================================================================

bool CamController::Impl::tool_library_dialog()
{
    wxDialog dlg(&p, wxID_ANY, _L("Tool Library"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
    std::vector<CamTool> tools = library;
    auto* list = new wxListBox(&dlg, wxID_ANY, wxDefaultPosition, wxSize(dlg.FromDIP(260), dlg.FromDIP(340)));
    for (const CamTool& t : tools) list->Append(tool_label(t));

    auto* form = new wxFlexGridSizer(2, dlg.FromDIP(6), dlg.FromDIP(10));
    form->AddGrowableCol(1, 1);
    auto row = [&](const wxString& label, wxWindow* w) {
        form->Add(new wxStaticText(&dlg, wxID_ANY, label), 0, wxALIGN_CENTER_VERTICAL);
        form->Add(w, 1, wxEXPAND);
    };
    auto* f_number = new wxSpinCtrl(&dlg, wxID_ANY, "", wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 999, 1);
    auto* f_name   = new wxTextCtrl(&dlg, wxID_ANY);
    std::vector<wxString> types;
    for (int i = 0; i <= int(ToolType::Tap); ++i) types.push_back(tool_type_name(ToolType(i)));
    auto* f_type = make_choice(&dlg, types);
    auto* f_mat  = make_choice(&dlg, {_L("HSS"), _L("Carbide")});
    auto* f_d    = make_spin(&dlg, 0.01, 200, 3, 0.1);
    auto* f_cr   = make_spin(&dlg, 0, 100, 3, 0.1);
    auto* f_ta   = make_spin(&dlg, 1, 180, 1, 1);
    auto* f_td   = make_spin(&dlg, 0, 100, 3, 0.1);
    auto* f_fl   = new wxSpinCtrl(&dlg, wxID_ANY, "", wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 12, 2);
    auto* f_flen = make_spin(&dlg, 0.1, 500, 2, 1);
    auto* f_olen = make_spin(&dlg, 0.1, 1000, 2, 1);
    auto* f_sh   = make_spin(&dlg, 0.1, 100, 3, 0.1);
    auto* f_tp   = make_spin(&dlg, 0, 10, 3, 0.05);
    row(_L("Tool number (T)"), f_number);
    row(_L("Name"), f_name);
    row(_L("Type"), f_type);
    row(_L("Tool material"), f_mat);
    row(_L("Diameter mm"), f_d);
    row(_L("Corner radius mm"), f_cr);
    row(_L("Tip angle °"), f_ta);
    row(_L("Tip diameter mm"), f_td);
    row(_L("Flutes"), f_fl);
    row(_L("Flute length mm"), f_flen);
    row(_L("Overall length mm"), f_olen);
    row(_L("Shank diameter mm"), f_sh);
    row(_L("Thread pitch mm (tap)"), f_tp);

    int cur = -1;
    auto store = [&] {
        if (cur < 0 || cur >= int(tools.size())) return;
        CamTool& t = tools[cur];
        t.number = f_number->GetValue();
        t.name   = f_name->GetValue().ToUTF8().data();
        t.type   = ToolType(std::max(0, f_type->GetSelection()));
        t.material = ToolMaterial(std::max(0, f_mat->GetSelection()));
        t.diameter = f_d->GetValue(); t.corner_radius = f_cr->GetValue();
        t.tip_angle_deg = f_ta->GetValue(); t.tip_diameter = f_td->GetValue();
        t.flutes = f_fl->GetValue(); t.flute_length = f_flen->GetValue();
        t.overall_length = f_olen->GetValue(); t.shank_diameter = f_sh->GetValue();
        t.thread_pitch = f_tp->GetValue();
        list->SetString(cur, tool_label(t));
    };
    auto load = [&](int i) {
        cur = i;
        const bool ok = i >= 0 && i < int(tools.size());
        for (wxWindow* w : std::initializer_list<wxWindow*>{f_number, f_name, f_type, f_mat, f_d, f_cr, f_ta, f_td, f_fl, f_flen, f_olen, f_sh, f_tp})
            w->Enable(ok);
        if (!ok) return;
        const CamTool& t = tools[i];
        f_number->SetValue(t.number); f_name->SetValue(wxString::FromUTF8(t.name));
        f_type->SetSelection(int(t.type)); f_mat->SetSelection(int(t.material));
        f_d->SetValue(t.diameter); f_cr->SetValue(t.corner_radius); f_ta->SetValue(t.tip_angle_deg);
        f_td->SetValue(t.tip_diameter); f_fl->SetValue(t.flutes); f_flen->SetValue(t.flute_length);
        f_olen->SetValue(t.overall_length); f_sh->SetValue(t.shank_diameter); f_tp->SetValue(t.thread_pitch);
    };
    auto next_number = [&] {
        int n = 0;
        for (const CamTool& t : tools) n = std::max(n, t.number);
        return n + 1;
    };
    list->Bind(wxEVT_LISTBOX, [&](wxCommandEvent&) { store(); load(list->GetSelection()); });
    auto* b_add = new wxButton(&dlg, wxID_ANY, _L("Add"));
    auto* b_dup = new wxButton(&dlg, wxID_ANY, _L("Duplicate"));
    auto* b_del = new wxButton(&dlg, wxID_ANY, _L("Delete"));
    auto* b_def = new wxButton(&dlg, wxID_ANY, _L("Restore defaults"));
    b_add->Bind(wxEVT_BUTTON, [&](wxCommandEvent&) {
        store();
        CamTool t;
        t.number = next_number();
        t.name   = "6 mm flat end mill";
        tools.push_back(t);
        list->Append(tool_label(t));
        list->SetSelection(int(tools.size()) - 1);
        load(int(tools.size()) - 1);
    });
    b_dup->Bind(wxEVT_BUTTON, [&](wxCommandEvent&) {
        if (cur < 0) return;
        store();
        CamTool t = tools[cur];
        t.number = next_number();
        tools.push_back(t);
        list->Append(tool_label(t));
        list->SetSelection(int(tools.size()) - 1);
        load(int(tools.size()) - 1);
    });
    b_del->Bind(wxEVT_BUTTON, [&](wxCommandEvent&) {
        if (cur < 0) return;
        tools.erase(tools.begin() + cur);
        list->Delete(cur);
        const int n = std::min(cur, int(tools.size()) - 1);
        cur = -1;
        if (n >= 0) list->SetSelection(n);
        load(n);
    });
    b_def->Bind(wxEVT_BUTTON, [&](wxCommandEvent&) {
        tools = default_tools();
        list->Clear();
        for (const CamTool& t : tools) list->Append(tool_label(t));
        cur = -1;
        if (!tools.empty()) list->SetSelection(0);
        load(tools.empty() ? -1 : 0);
    });
    auto* lbtns = new wxBoxSizer(wxHORIZONTAL);
    for (wxButton* b : {b_add, b_dup, b_del, b_def}) lbtns->Add(b, 0, wxRIGHT, dlg.FromDIP(4));
    auto* left = new wxBoxSizer(wxVERTICAL);
    left->Add(list, 1, wxEXPAND);
    left->Add(lbtns, 0, wxTOP, dlg.FromDIP(6));
    auto* body = new wxBoxSizer(wxHORIZONTAL);
    body->Add(left, 1, wxEXPAND | wxALL, dlg.FromDIP(10));
    body->Add(form, 1, wxEXPAND | wxTOP | wxRIGHT | wxBOTTOM, dlg.FromDIP(10));
    auto* hint = new wxStaticText(&dlg, wxID_ANY, wxString::Format(_L("Saved to %s"), wxString::FromUTF8(library_path)));
    auto* root = new wxBoxSizer(wxVERTICAL);
    root->Add(body, 1, wxEXPAND);
    root->Add(hint, 0, wxLEFT | wxRIGHT, dlg.FromDIP(10));
    root->Add(dlg.CreateStdDialogButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxALL, dlg.FromDIP(10));
    dlg.SetSizerAndFit(root);
    if (!tools.empty()) { list->SetSelection(0); load(0); } else load(-1);
    wxGetApp().UpdateDlgDarkUI(&dlg);
    if (dlg.ShowModal() != wxID_OK) return false;
    store();
    library = tools;
    std::string err;
    if (!save_tool_library(library_path, library, &err))
        status(_L("Could not save the tool library: ") + wxString::FromUTF8(err), true);
    else
        status(wxString::Format(_L("Tool library saved (%zu tools)."), library.size()));
    return true;
}

// =================================================================================================
// Post Process dialog
// =================================================================================================

std::vector<int> CamController::Impl::post_ops(int scope) const
{
    std::vector<int> ops;
    const int s = current_setup();
    for (int i = 0; i < int(doc.operations.size()); ++i) {
        const CamOperation& op = doc.operations[i];
        if (!op.enabled) continue;
        if (scope == 0 && i != sel_op) continue;
        if (scope == 1 && op.setup_index != s) continue;
        ops.push_back(i);
    }
    return ops;
}

void CamController::Impl::post_dialog(const PostPreset* pre)
{
    if (doc.operations.empty()) { status(_L("Nothing to post yet: add an operation first."), true); return; }
    if (sim_on) sim_close();
    const int setup = std::max(0, current_setup());
    const MachineProfile& mach = machine_of(setup);

    wxDialog dlg(&p, wxID_ANY, _L("Post Process"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
    auto* form = new wxFlexGridSizer(2, dlg.FromDIP(6), dlg.FromDIP(10));
    form->AddGrowableCol(1, 1);
    auto row = [&](const wxString& label, wxWindow* w) {
        form->Add(new wxStaticText(&dlg, wxID_ANY, label), 0, wxALIGN_CENTER_VERTICAL);
        form->Add(w, 1, wxEXPAND);
    };
    std::vector<wxString> mnames;
    int msel = 0;
    for (int i = 0; i < int(machines.size()); ++i) {
        mnames.push_back(wxString::FromUTF8(machines[i].name));
        if (machines[i].name == mach.name) msel = i;
    }
    auto* c_machine = make_choice(&dlg, mnames);
    c_machine->SetSelection(msel);
    std::vector<wxString> dn(std::begin(kDialectNames), std::end(kDialectNames));
    auto* c_dialect = make_choice(&dlg, dn);
    c_dialect->SetSelection(pre && pre->dialect >= 0 ? pre->dialect : int(mach.post));
    auto* t_prog = new wxTextCtrl(&dlg, wxID_ANY, "shashimi");
    auto* s_prog = new wxSpinCtrl(&dlg, wxID_ANY, "", wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 1, 9999, 1000);
    auto* units = new wxChoice(&dlg, wxID_ANY);
    units->Append(_L("Millimetres (G21)"));
    units->Append(_L("Inches (G20)"));
    units->SetSelection(0);
    auto* cb_arcs = new wxCheckBox(&dlg, wxID_ANY, _L("Output arcs as G2/G3 (off: short lines)"));
    cb_arcs->SetValue(true);
    auto* cb_inv = new wxCheckBox(&dlg, wxID_ANY, _L("Inverse-time feed (G93) on A-axis moves"));
    auto* c_scope = new wxChoice(&dlg, wxID_ANY);
    c_scope->Append(_L("Selected operation"));
    c_scope->Append(_L("Selected setup"));
    c_scope->Append(_L("All operations"));
    const int scope0 = pre && pre->scope >= 0 ? pre->scope : (sel_op >= 0 ? 0 : 1);
    c_scope->SetSelection(scope0 == 0 && sel_op < 0 ? 1 : scope0);
    auto* t_path = new wxTextCtrl(&dlg, wxID_ANY);
    auto* b_browse = new wxButton(&dlg, wxID_ANY, _L("Browse…"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    auto* path_row = new wxBoxSizer(wxHORIZONTAL);
    path_row->Add(t_path, 1, wxEXPAND | wxRIGHT, dlg.FromDIP(4));
    path_row->Add(b_browse, 0);
    row(_L("Machine"), c_machine);
    row(_L("Post (G-code dialect)"), c_dialect);
    row(_L("Program name"), t_prog);
    row(_L("Program number (O)"), s_prog);
    row(_L("Units"), units);
    row(_L("Operations"), c_scope);
    form->Add(new wxStaticText(&dlg, wxID_ANY, _L("Output file")), 0, wxALIGN_CENTER_VERTICAL);
    form->Add(path_row, 1, wxEXPAND);
    form->AddSpacer(0);
    form->Add(cb_arcs);
    form->AddSpacer(0);
    form->Add(cb_inv);

    auto* preview = new wxTextCtrl(&dlg, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(dlg.FromDIP(640), dlg.FromDIP(300)),
                                   wxTE_MULTILINE | wxTE_READONLY | wxTE_DONTWRAP | wxHSCROLL);
    preview->SetFont(wxFont(wxFontInfo(9).Family(wxFONTFAMILY_TELETYPE)));
    auto* info = new wxStaticText(&dlg, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxST_ELLIPSIZE_END | wxST_NO_AUTORESIZE);
    auto* b_show  = new wxButton(&dlg, wxID_ANY, _L("Show G-code"));
    auto* b_post  = new wxButton(&dlg, wxID_ANY, _L("Post"));
    auto* b_close = new wxButton(&dlg, wxID_CANCEL, _L("Close"));
    auto* btns = new wxBoxSizer(wxHORIZONTAL);
    btns->AddStretchSpacer(1);
    btns->Add(b_show, 0, wxRIGHT, dlg.FromDIP(6));
    btns->Add(b_post, 0, wxRIGHT, dlg.FromDIP(6));
    btns->Add(b_close, 0);

    auto default_path = [&](int dialect) {
        wxFileName fn(wxGetHomeDir(), t_prog->GetValue());
        fn.SetExt(kDialectExt[std::clamp(dialect, 0, 4)]);
        return fn.GetFullPath();
    };
    t_path->SetValue(pre && !pre->path.empty() ? wxString::FromUTF8(pre->path) : default_path(c_dialect->GetSelection()));
    auto sync_dialect = [&] {
        const PostDialect d = PostDialect(std::max(0, c_dialect->GetSelection()));
        cb_inv->Enable(dialect_has_inverse_time(d));
        if (!cb_inv->IsEnabled()) cb_inv->SetValue(false);
        wxFileName fn(t_path->GetValue());
        fn.SetExt(kDialectExt[int(d)]);
        t_path->SetValue(fn.GetFullPath());
    };
    sync_dialect();
    c_machine->Bind(wxEVT_CHOICE, [&](wxCommandEvent&) {
        const int m = c_machine->GetSelection();
        if (m >= 0 && m < int(machines.size())) c_dialect->SetSelection(int(machines[m].post));
        sync_dialect();
    });
    c_dialect->Bind(wxEVT_CHOICE, [&](wxCommandEvent&) { sync_dialect(); });

    std::string last;
    auto make = [&]() -> bool {
        const std::vector<int> ops = post_ops(c_scope->GetSelection());
        if (ops.empty()) { info->SetLabel(_L("No enabled operations in that choice.")); return false; }
        std::vector<int> todo;
        for (int i : ops) if (op_state(i) != OpState::Ok) todo.push_back(i);
        if (!todo.empty()) regenerate(todo, _L("Generating toolpaths…"));
        for (int i : ops)
            if (!doc.paths[i].ok()) {
                info->SetLabel(wxString::FromUTF8(doc.operations[i].name) + ": " + wxString::FromUTF8(doc.paths[i].error));
                return false;
            }
        PostOptions o;
        const int m = std::clamp(c_machine->GetSelection(), 0, std::max(0, int(machines.size()) - 1));
        if (!machines.empty()) o.machine = machines[m];
        o.machine.post     = PostDialect(std::max(0, c_dialect->GetSelection()));
        o.program_name     = t_prog->GetValue().ToUTF8().data();
        o.program_number   = s_prog->GetValue();
        o.inch             = units->GetSelection() == 1;
        o.arcs             = cb_arcs->GetValue();
        o.inverse_time     = cb_inv->GetValue();
        std::string err;
        try {
            last = post_process(doc, ops, o, &err);
        } catch (const std::exception& e) {
            err = e.what();
            last.clear();
        }
        if (last.empty()) { info->SetLabel(_L("Post failed: ") + wxString::FromUTF8(err)); return false; }
        double t = 0;
        std::set<int> tools;
        for (int i : ops) { t += doc.paths[i].time_s; tools.insert(doc.operations[i].tool_number); }
        const size_t lines = size_t(std::count(last.begin(), last.end(), '\n'));
        info->SetLabel(wxString::Format(_L("%zu operations · %zu tools · %zu lines · estimated %s"), ops.size(), tools.size(), lines, fmt_time(t)));
        status(wxString::Format(_L("Post: %zu operations, estimated machining time %s."), ops.size(), fmt_time(t)));
        return true;
    };
    auto show = [&] {
        if (!make()) return;
        // A multi-megabyte program would make the text control crawl; the file has all of it.
        constexpr size_t kMax = 2u << 20;
        preview->SetValue(wxString::FromUTF8(last.size() > kMax ? last.substr(0, kMax) + "\n(… preview truncated; the file has everything)\n" : last));
    };
    auto post = [&] {
        if (!make()) return;
        wxString path = t_path->GetValue();
        if (path.empty()) return;
        std::ofstream f(path.ToUTF8().data(), std::ios::binary);
        f << last;
        f.close();
        if (!f) { info->SetLabel(_L("Could not write ") + path); return; }
        if (preview->IsEmpty()) show();
        info->SetLabel(info->GetLabel() + _L(" · saved"));
        status(wxString::Format(_L("G-code written to %s."), path));
    };
    b_browse->Bind(wxEVT_BUTTON, [&](wxCommandEvent&) {
        const int d = std::max(0, c_dialect->GetSelection());
        wxFileName cur(t_path->GetValue());
        wxFileDialog fd(&dlg, _L("Save G-code"), cur.GetPath(), cur.GetFullName(),
                        wxString::Format("G-code (*.%s)|*.%s|G-code (*.nc;*.ngc;*.gcode;*.tap)|*.nc;*.ngc;*.gcode;*.tap|All files|*",
                                         kDialectExt[d], kDialectExt[d]),
                        wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
        if (fd.ShowModal() == wxID_OK) t_path->SetValue(fd.GetPath());
    });
    b_show->Bind(wxEVT_BUTTON, [&](wxCommandEvent&) { show(); });
    b_post->Bind(wxEVT_BUTTON, [&](wxCommandEvent&) { post(); });

    auto* root = new wxBoxSizer(wxVERTICAL);
    root->Add(form, 0, wxEXPAND | wxALL, dlg.FromDIP(10));
    root->Add(preview, 1, wxEXPAND | wxLEFT | wxRIGHT, dlg.FromDIP(10));
    root->Add(info, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, dlg.FromDIP(10));
    root->Add(btns, 0, wxEXPAND | wxALL, dlg.FromDIP(10));
    dlg.SetSizerAndFit(root);
    wxGetApp().UpdateDlgDarkUI(&dlg);
    if (pre && (pre->show || pre->write))
        dlg.CallAfter([&, pre_show = pre->show, pre_write = pre->write] {
            if (pre_write) post(); else if (pre_show) show();
        });
    dlg.ShowModal();
}

// =================================================================================================
// Viewport layer
// =================================================================================================

namespace {

const ColorRGBA kPathColours[4] = {
    ColorRGBA(1.00f, 0.84f, 0.10f, 1.f),   // rapid / retract: yellow
    ColorRGBA(0.16f, 0.48f, 1.00f, 1.f),   // cutting feed: blue
    ColorRGBA(0.20f, 0.80f, 0.32f, 1.f),   // lead in / out: green
    ColorRGBA(0.95f, 0.22f, 0.18f, 1.f),   // plunge / ramp: red
};

int path_bucket(const Move& m)
{
    switch (m.kind) {
    case Move::Kind::Rapid:
    case Move::Kind::Retract: return 0;
    case Move::Kind::LeadIn:
    case Move::Kind::LeadOut: return 2;
    case Move::Kind::Plunge:
    case Move::Kind::Ramp:    return 3;
    default:                  return 1;
    }
}

GLModel::Geometry lines_geometry()
{
    GLModel::Geometry g;
    g.format = {GLModel::Geometry::EPrimitiveType::Lines, GLModel::Geometry::EVertexLayout::P3};
    return g;
}

void add_segment(GLModel::Geometry& g, const Vec3d& a, const Vec3d& b)
{
    const unsigned int i = unsigned(g.vertices_count());
    g.add_vertex(Vec3f(a.cast<float>()));
    g.add_vertex(Vec3f(b.cast<float>()));
    g.add_line(i, i + 1);
}

void init_or_reset(GLModel& m, GLModel::Geometry&& g, const ColorRGBA& c)
{
    m.reset();
    if (g.vertices_count() == 0) return;
    m.init_from(std::move(g));
    m.set_color(c);
}

} // namespace

void CamController::Impl::rebuild_gl()
{
    gl_dirty = false;
    op_gl.clear();
    op_gl.resize(doc.operations.size());
    for (size_t i = 0; i < doc.operations.size() && i < doc.paths.size(); ++i) {
        const Toolpath& tp = doc.paths[i];
        GLModel::Geometry g[4] = {lines_geometry(), lines_geometry(), lines_geometry(), lines_geometry()};
        Vec3d  pos;
        double a = 0;
        bool   have = false;
        for (const Move& m : tp.moves) {
            if (!have) { pos = m.to; a = m.a_deg; have = true; continue; }
            GLModel::Geometry& dst = g[path_bucket(m)];
            int n = 1;
            if (is_arc(m)) n = std::max(n, int(std::ceil(move_length(pos, m) / 0.25)));
            if (std::abs(m.a_deg - a) > 1e-9) n = std::max(n, int(std::ceil(std::abs(m.a_deg - a) / 1.0)));
            n = std::min(n, 720);
            Vec3d prev = part_point(pos, a);
            for (int k = 1; k <= n; ++k) {
                const double t = double(k) / n;
                const Vec3d  q = part_point(move_point(pos, m, t), a + (m.a_deg - a) * t);
                add_segment(dst, prev, q);
                prev = q;
            }
            pos = m.to;
            a   = m.a_deg;
        }
        for (int k = 0; k < 4; ++k) init_or_reset(op_gl[i].lines[k], std::move(g[k]), kPathColours[k]);
        GLModel::Geometry w = lines_geometry();
        for (const CAM::Warning& wa : tp.warnings) {
            if (wa.move_index < 0) continue;
            const double r = 0.8;
            for (int ax = 0; ax < 3; ++ax) {
                Vec3d d = Vec3d::Zero();
                d[ax] = r;
                add_segment(w, wa.pos - d, wa.pos + d);
            }
        }
        init_or_reset(op_gl[i].warn, std::move(w), ColorRGBA(1.f, 0.1f, 0.1f, 1.f));
    }
}

void CamController::Impl::rebuild_stock_gl(int s)
{
    stock_dirty = false;
    stock_setup = s;
    stock_faces.reset();
    stock_edges.reset();
    for (GLModel& t : triad) t.reset();
    if (s < 0 || s >= int(model.setups.size()) || s >= int(doc.setups.size())) return;
    const CamSetupFrame& f = model.setups[s];
    if (!f.stock.defined) return;
    const Vec3d lo = f.stock.min, hi = f.stock.max;
    GLModel::Geometry faces;
    faces.format = {GLModel::Geometry::EPrimitiveType::Triangles, GLModel::Geometry::EVertexLayout::P3};
    GLModel::Geometry edges = lines_geometry();
    if (doc.setups[s].stock.kind == StockKind::Cylinder && f.stock_radius > 0) {
        const int    N = 64;
        const double r = f.stock_radius;
        auto at = [&](double x, int k) { const double t = 2 * M_PI * k / N; return Vec3d(x, r * std::cos(t), r * std::sin(t)); };
        for (int k = 0; k < N; ++k) {
            const Vec3d a0 = at(lo.x(), k), a1 = at(lo.x(), k + 1), b0 = at(hi.x(), k), b1 = at(hi.x(), k + 1);
            const unsigned i = unsigned(faces.vertices_count());
            for (const Vec3d& v : {a0, a1, b1, b0}) faces.add_vertex(Vec3f(v.cast<float>()));
            faces.add_triangle(i, i + 1, i + 2);
            faces.add_triangle(i, i + 2, i + 3);
            add_segment(edges, a0, a1);
            add_segment(edges, b0, b1);
            if (k % 16 == 0) add_segment(edges, a0, b0);
        }
    } else {
        const Vec3d c[8] = {{lo.x(), lo.y(), lo.z()}, {hi.x(), lo.y(), lo.z()}, {hi.x(), hi.y(), lo.z()}, {lo.x(), hi.y(), lo.z()},
                            {lo.x(), lo.y(), hi.z()}, {hi.x(), lo.y(), hi.z()}, {hi.x(), hi.y(), hi.z()}, {lo.x(), hi.y(), hi.z()}};
        for (const Vec3d& v : c) faces.add_vertex(Vec3f(v.cast<float>()));
        const int quads[6][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};
        for (const auto& q : quads) { faces.add_triangle(q[0], q[1], q[2]); faces.add_triangle(q[0], q[2], q[3]); }
        const int e[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
        for (const auto& k : e) add_segment(edges, c[k[0]], c[k[1]]);
    }
    init_or_reset(stock_faces, std::move(faces), ColorRGBA(0.62f, 0.70f, 0.82f, 0.16f));
    init_or_reset(stock_edges, std::move(edges), ColorRGBA(0.45f, 0.62f, 0.90f, 0.85f));
    const double L = std::clamp(0.25 * (hi - lo).maxCoeff(), 5.0, 40.0);
    const ColorRGBA tc[3] = {ColorRGBA(0.92f, 0.25f, 0.22f, 1.f), ColorRGBA(0.25f, 0.80f, 0.30f, 1.f), ColorRGBA(0.25f, 0.50f, 1.f, 1.f)};
    for (int ax = 0; ax < 3; ++ax) {
        GLModel::Geometry g = lines_geometry();
        Vec3d d = Vec3d::Zero();
        d[ax] = L;
        add_segment(g, Vec3d::Zero(), d);
        // A small arrow head so the direction reads.
        Vec3d side = Vec3d::Zero();
        side[(ax + 1) % 3] = L * 0.08;
        add_segment(g, d, d * 0.85 + side);
        add_segment(g, d, d * 0.85 - side);
        init_or_reset(triad[ax], std::move(g), tc[ax]);
    }
}

void CamController::Impl::render()
{
    if (!p.m_cam_shown) return;
    GLShaderProgram* flat = wxGetApp().get_shader("flat");
    if (flat == nullptr) return;
    if (gl_dirty) rebuild_gl();
    const int cs = sim_on ? sim_setup : current_setup();
    if (stock_dirty || stock_setup != cs) rebuild_stock_gl(cs);

    const Camera&     cam  = wxGetApp().plater()->get_camera();
    const Transform3d view = cam.get_view_matrix();
    auto frame_of = [&](int s) -> const Transform3d* {
        return s >= 0 && s < int(model.setups.size()) ? &model.setups[s].to_setup : nullptr;
    };

    glsafe(::glEnable(GL_BLEND));
    glsafe(::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
    flat->start_using();
    flat->set_uniform("projection_matrix", cam.get_projection_matrix());

    const bool sim_stock = sim_on && sim_mesh_gl.is_initialized();
    if (const Transform3d* f = frame_of(cs)) {
        flat->set_uniform("view_model_matrix", view * f->inverse());
        if (!sim_stock && stock_faces.is_initialized()) {
            glsafe(::glEnable(GL_DEPTH_TEST));
            glsafe(::glDepthMask(GL_FALSE));
            stock_faces.render();
            glsafe(::glDepthMask(GL_TRUE));
        }
        glsafe(::glDisable(GL_DEPTH_TEST));
        if (stock_edges.is_initialized()) stock_edges.render();
        for (GLModel& t : triad) if (t.is_initialized()) t.render();
    }

    // The simulated stock goes under the toolpaths.
    if (sim_on) {
        flat->stop_using();
        render_sim_stock(cam);
        flat->start_using();
        flat->set_uniform("projection_matrix", cam.get_projection_matrix());
    }

    // Toolpaths of the current setup over everything, like the CAD sketch overlay: the selected
    // one in full colour, the others faded.
    glsafe(::glDisable(GL_DEPTH_TEST));
    for (int i = 0; i < int(op_gl.size()) && i < int(doc.operations.size()); ++i) {
        const CamOperation& op = doc.operations[i];
        if (!op.enabled) continue;
        const Transform3d* f = frame_of(op.setup_index);
        if (f == nullptr) continue;
        if (op.setup_index != cs) continue;   // one setup at a time: each has its own frame and stock
        const bool focus = sel_op < 0 || sel_op == i;
        flat->set_uniform("view_model_matrix", view * f->inverse());
        for (int k = 0; k < 4; ++k) {
            GLModel& m = op_gl[i].lines[k];
            if (!m.is_initialized()) continue;
            ColorRGBA c = kPathColours[k];
            c.a(focus ? 1.0f : 0.4f);
            m.set_color(c);
            m.render();
        }
        if (op_gl[i].warn.is_initialized()) op_gl[i].warn.render();
    }
    flat->stop_using();

    if (sim_on) render_tool(cam);
    glsafe(::glDisable(GL_BLEND));
    glsafe(::glEnable(GL_DEPTH_TEST));
}

// Lit meshes (simulated stock, tool) in the setup frame.
struct LitDraw
{
    GLShaderProgram* sh{nullptr};
    Transform3d      view;
    LitDraw(const Camera& cam)
    {
        sh = wxGetApp().get_shader("gouraud_light");
        if (sh == nullptr) return;
        view = cam.get_view_matrix();
        glsafe(::glEnable(GL_DEPTH_TEST));
        sh->start_using();
        sh->set_uniform("emission_factor", 0.1f);
        sh->set_uniform("projection_matrix", cam.get_projection_matrix());
    }
    ~LitDraw() { if (sh) sh->stop_using(); }
    void operator()(GLModel& m, const Transform3d& model_m) const
    {
        if (sh == nullptr || !m.is_initialized()) return;
        sh->set_uniform("view_model_matrix", view * model_m);
        const Matrix3d vn = view.matrix().block(0, 0, 3, 3) * model_m.matrix().block(0, 0, 3, 3).inverse().transpose();
        sh->set_uniform("view_normal_matrix", vn);
        m.render();
    }
};

void CamController::Impl::render_sim_stock(const Camera& cam)
{
    if (sim_setup < 0 || sim_setup >= int(model.setups.size())) return;
    LitDraw draw(cam);
    if (sim_mesh_dirty) {
        sim_mesh_dirty = false;
        sim_mesh_gl.reset();
        if (!sim_mesh.empty()) {
            sim_mesh_gl.init_from(sim_mesh.its);
            sim_mesh_gl.set_color(ColorRGBA(0.74f, 0.77f, 0.82f, 1.f));
        }
    }
    draw(sim_mesh_gl, model.setups[sim_setup].to_setup.inverse());
}

void CamController::Impl::render_tool(const Camera& cam)
{
    if (sim_setup < 0 || sim_setup >= int(model.setups.size())) return;
    const Transform3d w = model.setups[sim_setup].to_setup.inverse();
    LitDraw draw(cam);
    if (!sim.empty()) {
        double a = 0;
        int    idx = 0;
        const Vec3d pos = sim_pos(sim_t, &a, &idx);
        const CamOperation& op = doc.operations[sim[std::min<size_t>(idx, sim.size() - 1)].op];
        const CamTool* t = tool_of(op);
        if (t != nullptr) {
            if (t->number != tool_gl_number) {
                tool_gl_number = t->number;
                const float r  = float(std::max(0.1, t->diameter / 2));
                const float fl = float(std::max(t->flute_length, 2.0 * r));
                const float sr = float(std::max(0.1, t->shank_diameter / 2));
                const float ol = float(std::max(t->overall_length, double(fl) + 5));
                const bool  ball = t->type == ToolType::BallEndMill;
                tool_ball.reset(); tool_body.reset(); tool_shank.reset();
                if (ball) {
                    tool_ball.init_from(smooth_sphere(24, r));
                    tool_ball.set_color(ColorRGBA(0.85f, 0.55f, 0.20f, 1.f));
                }
                tool_body.init_from(smooth_cylinder(32, r, ball ? fl - r : fl));
                tool_body.set_color(ColorRGBA(0.85f, 0.55f, 0.20f, 1.f));   // fluted part: orange, as in Fusion
                tool_shank.init_from(smooth_cylinder(32, sr, ol - fl));
                tool_shank.set_color(ColorRGBA(0.70f, 0.72f, 0.76f, 1.f));
            }
            const float r  = float(std::max(0.1, t->diameter / 2));
            const float fl = float(std::max(t->flute_length, 2.0 * r));
            const bool  ball = t->type == ToolType::BallEndMill;
            const Transform3d base = w * Geometry::translation_transform(part_point(pos, a))
                                   * Geometry::rotation_transform(Vec3d(-a * M_PI / 180.0, 0, 0));
            if (ball) draw(tool_ball, base * Geometry::translation_transform(Vec3d(0, 0, r)));
            draw(tool_body, base * Geometry::translation_transform(Vec3d(0, 0, ball ? r : 0)));
            draw(tool_shank, base * Geometry::translation_transform(Vec3d(0, 0, fl)));
        }
    }
}

// =================================================================================================
// Simulation
// =================================================================================================

void CamController::Impl::build_simbar(wxWindow* parent)
{
    simbar = new wxPanel(parent, wxID_ANY);
    simbar->SetBackgroundColour(CadTheme::ribbon_bg());
    sb_play = new wxButton(simbar, wxID_ANY, wxString::FromUTF8("\xE2\x96\xB6 Play"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    auto* restart = new wxButton(simbar, wxID_ANY, wxString::FromUTF8("\xE2\x8F\xAE"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    restart->SetToolTip(_L("Back to the start"));
    sb_speed = make_choice(simbar, {wxString::FromUTF8("1\xC3\x97"), wxString::FromUTF8("2\xC3\x97"), wxString::FromUTF8("5\xC3\x97"),
                                    wxString::FromUTF8("10\xC3\x97"), wxString::FromUTF8("25\xC3\x97"), wxString::FromUTF8("100\xC3\x97")});
    sb_speed->SetSelection(3);
    sb_speed->SetToolTip(_L("Playback speed (times real machining time)"));
    sb_slider = new wxSlider(simbar, wxID_ANY, 0, 0, 1000);
    sb_time = new wxStaticText(simbar, wxID_ANY, "0:00 / 0:00");
    sb_info = new wxStaticText(simbar, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize,
                               wxST_ELLIPSIZE_END | wxST_NO_AUTORESIZE);
    for (wxStaticText* t : {sb_time, sb_info}) t->SetForegroundColour(CadTheme::text());
    auto* close = new wxButton(simbar, wxID_ANY, _L("Close"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    auto* s = new wxBoxSizer(wxHORIZONTAL);
    auto* col = new wxBoxSizer(wxVERTICAL);
    auto* title = new wxStaticText(simbar, wxID_ANY, _L("Simulation"));
    wxFont f = title->GetFont();
    f.SetWeight(wxFONTWEIGHT_BOLD);
    title->SetFont(f);
    title->SetForegroundColour(CadTheme::text());
    const int m = simbar->FromDIP(4);
    s->Add(title, 0, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, 2 * m);
    s->Add(sb_play, 0, wxALIGN_CENTER_VERTICAL | wxALL, m);
    s->Add(restart, 0, wxALIGN_CENTER_VERTICAL | wxALL, m);
    s->Add(sb_speed, 0, wxALIGN_CENTER_VERTICAL | wxALL, m);
    s->Add(sb_slider, 1, wxALIGN_CENTER_VERTICAL | wxALL, m);
    s->Add(sb_time, 0, wxALIGN_CENTER_VERTICAL | wxALL, m);
    s->Add(close, 0, wxALIGN_CENTER_VERTICAL | wxALL, m);
    col->Add(s, 0, wxEXPAND);
    col->Add(sb_info, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 2 * m);
    simbar->SetSizer(col);
    simbar->Hide();
    const double speeds[6] = {1, 2, 5, 10, 25, 100};
    sb_speed->Bind(wxEVT_CHOICE, [this, speeds](wxCommandEvent&) { sim_speed = speeds[std::clamp(sb_speed->GetSelection(), 0, 5)]; });
    sb_play->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { sim_play(!sim_playing); });
    restart->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { sim_seek(0); });
    close->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { sim_close(); });
    sb_slider->Bind(wxEVT_SLIDER, [this](wxCommandEvent&) {
        sim_play(false);
        sim_seek(sim_total * sb_slider->GetValue() / 1000.0);
    });
}

int CamController::Impl::sim_index(double t) const
{
    auto it = std::upper_bound(sim.begin(), sim.end(), t, [](double v, const SimMove& m) { return v < m.t1; });
    return int(it - sim.begin());
}

Vec3d CamController::Impl::sim_pos(double t, double* a_deg, int* idx) const
{
    if (sim.empty()) return Vec3d::Zero();
    int k = sim_index(t);
    if (k >= int(sim.size())) {
        const SimMove& last = sim.back();
        if (a_deg) *a_deg = last.m.a_deg;
        if (idx) *idx = int(sim.size()) - 1;
        return last.m.to;
    }
    const SimMove& s = sim[k];
    const double frac = s.t1 > s.t0 ? std::clamp((t - s.t0) / (s.t1 - s.t0), 0.0, 1.0) : 1.0;
    if (idx) *idx = k;
    // The A position at the start of this move is the previous move's end.
    const double a0 = k > 0 ? sim[k - 1].m.a_deg : s.m.a_deg;
    if (a_deg) *a_deg = a0 + (s.m.a_deg - a0) * frac;
    return move_point(s.from, s.m, frac);
}

void CamController::Impl::sim_open()
{
    if (doc.operations.empty()) { status(_L("Nothing to simulate yet: add an operation first."), true); return; }
    close_page();
    sim_setup = std::max(0, current_setup());
    std::vector<int> ops;
    if (sel_op >= 0 && sel_op < int(doc.operations.size()) && doc.operations[sel_op].enabled) ops.push_back(sel_op);
    else
        for (int i = 0; i < int(doc.operations.size()); ++i)
            if (doc.operations[i].enabled && doc.operations[i].setup_index == sim_setup) ops.push_back(i);
    std::vector<int> todo;
    for (int i : ops) if (op_state(i) != OpState::Ok) todo.push_back(i);
    if (!todo.empty()) regenerate(todo, _L("Generating toolpaths…"));

    const MachineProfile& mach = machine_of(sim_setup);
    sim.clear();
    double t = 0;
    for (int i : ops) {
        const Toolpath& tp = doc.paths[i];
        if (!tp.ok() || tp.moves.empty()) continue;
        Vec3d pos = sim.empty() ? tp.moves.front().to : sim.back().m.to;
        size_t k0 = sim.empty() ? 1 : 0;
        for (size_t k = k0; k < tp.moves.size(); ++k) {
            const Move& mv = tp.moves[k];
            const bool rapid = mv.kind == Move::Kind::Rapid || mv.kind == Move::Kind::Retract;
            const double feed = rapid ? std::max(1.0, mach.rapid_feed) : (mv.feed > 0 ? mv.feed : std::max(1.0, mach.max_feed_xy));
            double dt = move_length(pos, mv) / feed * 60.0;
            dt = std::max(dt, 1e-4);
            sim.push_back({i, pos, mv, t, t + dt});
            t += dt;
            pos = mv.to;
        }
    }
    if (sim.empty()) { status(_L("No valid toolpath to simulate (see the operation's error)."), true); return; }
    sim_total = t;
    sim_t     = 0;
    sim_on    = true;
    sim_cut_upto = 0;
    sim_snapshots.clear();
    sim_mesh = TriangleMesh();
    sim_mesh_dirty = true;
    sim_stock_ok = false;
    try {
        ensure_model();
        const CamSetupFrame& f = model.setups.at(sim_setup);
        if (doc.setups[sim_setup].stock.kind == StockKind::Cylinder && f.stock_radius > 0)
            stocksim.init_cylinder(f.stock.min.x(), f.stock.max.x(), f.stock_radius);
        else
            stocksim.init_box(f.stock);
        sim_stock_ok = true;
    } catch (...) {}
    stock_dirty = true;
    simbar->Show();
    p.Layout();
    sim_update_stock(true);
    sim_update_labels();
    sim_last_tick = std::chrono::steady_clock::now();
    sim_timer.Start(33);
    status(wxString::Format(_L("Simulating %zu operation(s), %s of machining. Space plays/pauses, drag the slider to scrub."),
                            ops.size(), fmt_time(sim_total)));
    repaint();
}

void CamController::Impl::sim_close()
{
    if (!sim_on) return;
    sim_timer.Stop();
    sim_on = sim_playing = false;
    simbar->Hide();
    p.Layout();
    if (bodies_hidden && p.m_viewport) { p.m_viewport->set_body_hidden(false); bodies_hidden = false; }
    sim_mesh = TriangleMesh();
    sim_snapshots.clear();
    sim_mesh_dirty = true;
    stock_dirty    = true;
    repaint();
}

void CamController::Impl::sim_play(bool on)
{
    if (on && sim_t >= sim_total) sim_seek(0);
    sim_playing = on;
    sb_play->SetLabel(wxString::FromUTF8(on ? "\xE2\x8F\xB8 Pause" : "\xE2\x96\xB6 Play"));
    sim_last_tick = std::chrono::steady_clock::now();
}

void CamController::Impl::sim_seek(double t)
{
    sim_t = std::clamp(t, 0.0, sim_total);
    sim_update_stock(true);
    sim_update_labels();
    repaint();
}

void CamController::Impl::sim_tick()
{
    const auto   now = std::chrono::steady_clock::now();
    const double dt  = std::chrono::duration<double>(now - sim_last_tick).count();
    sim_last_tick = now;
    if (!sim_on || !sim_playing) return;
    sim_t = std::min(sim_total, sim_t + dt * sim_speed);
    sim_since_mesh += dt;
    if (sim_since_mesh >= 0.1) { sim_since_mesh = 0; sim_update_stock(false); }
    if (sim_t >= sim_total) { sim_play(false); sim_update_stock(true); }
    sim_update_labels();
    repaint();
}

void CamController::Impl::sim_update_stock(bool)
{
    if (!sim_stock_ok || sim.empty()) return;
    const size_t upto = std::min<size_t>(sim_index(sim_t), sim.size());
    try {
        if (upto < sim_cut_upto) {   // scrubbed back: restart from the nearest snapshot at or before it
            auto it = sim_snapshots.upper_bound(upto);
            if (it == sim_snapshots.begin()) {
                const CamSetupFrame& f = model.setups.at(sim_setup);
                if (doc.setups[sim_setup].stock.kind == StockKind::Cylinder && f.stock_radius > 0)
                    stocksim.init_cylinder(f.stock.min.x(), f.stock.max.x(), f.stock_radius);
                else
                    stocksim.init_box(f.stock);
                sim_cut_upto = 0;
            } else {
                --it;
                stocksim     = it->second;
                sim_cut_upto = it->first;
            }
        }
        const size_t step = std::max<size_t>(1, sim.size() / 20);
        for (size_t k = sim_cut_upto; k < upto; ++k) {
            if (k > 0 && k % step == 0 && sim_snapshots.count(k) == 0) sim_snapshots.emplace(k, stocksim);
            const SimMove& s = sim[k];
            if (s.m.kind == Move::Kind::Rapid) continue;
            const CamTool* t = tool_of(doc.operations[s.op]);
            if (t == nullptr) continue;
            Move from;
            from.to    = s.from;
            from.a_deg = k > 0 ? sim[k - 1].m.a_deg : s.m.a_deg;
            stocksim.cut(*t, from, s.m);
        }
        sim_cut_upto = upto;
        TriangleMesh mesh = stocksim.to_mesh();
        if (!mesh.empty()) {
            sim_mesh       = std::move(mesh);
            sim_mesh_dirty = true;
            if (!bodies_hidden && p.m_viewport) { p.m_viewport->set_body_hidden(true); bodies_hidden = true; }
        }
    } catch (...) {
        sim_stock_ok = false;   // no material removal: the tool still animates over the paths
    }
}

void CamController::Impl::sim_update_labels()
{
    sb_slider->SetValue(sim_total > 0 ? int(std::lround(1000.0 * sim_t / sim_total)) : 0);
    sb_time->SetLabel(fmt_time(sim_t) + " / " + fmt_time(sim_total));
    if (sim.empty()) return;
    double a = 0;
    int    k = 0;
    const Vec3d pos = sim_pos(sim_t, &a, &k);
    const SimMove& s = sim[std::min<size_t>(k, sim.size() - 1)];
    const CamOperation& op = doc.operations[s.op];
    const CamTool* t = tool_of(op);
    const bool rapid = s.m.kind == Move::Kind::Rapid || s.m.kind == Move::Kind::Retract;
    wxString txt = wxString::FromUTF8(op.name) + "  ·  " + (t ? tool_label(*t) : wxString::Format("T%d", op.tool_number)) + "  ·  "
                 + (rapid ? wxString(_L("rapid")) : wxString::Format("F %.0f", s.m.feed))
                 + wxString::Format("  ·  X %.3f  Y %.3f  Z %.3f", pos.x(), pos.y(), pos.z());
    if (machine_of(sim_setup).has_a_axis) txt += wxString::Format("  A %.2f", a);
    sb_info->SetLabel(txt);
    sb_info->SetToolTip(txt);
}

void CamController::Impl::show_pm_page(bool pm_on)
{
    p.show_left_page(pm_on);
}

// =================================================================================================
// CamController
// =================================================================================================

CamController::CamController(DesignPanel& panel) : m(std::make_unique<Impl>(panel)) {}
CamController::~CamController() = default;

wxWindow* CamController::tree_page() const { return m->tree_panel; }
wxWindow* CamController::pm_page() const { return m->pm; }
wxWindow* CamController::sim_bar() const { return m->simbar; }
bool      CamController::pm_active() const { return m->page != Impl::Page::None; }
void      CamController::show_pm(bool pm) { m->show_pm_page(pm); }

void CamController::build_ribbon(CadRibbon& r)
{
    using Page = CadRibbon::Page;
    const Page C = Page::Cam;
    Impl* im = m.get();
    m->ribbon = &r;
    auto op_cmd = [im](OpType t) {
        const OpInfo& i = op_info(t);
        return CadCommand{i.icon, _L(i.label), _L(i.tip), [im, t] { im->open_op(-1, t); }};
    };
    r.add_button(C, "cam_setup", {"sw_cam_setup", _L("Setup"), _L("Setup — machine, material, stock and the work origin (WCS)"),
                                  [im] { im->open_setup(-1); }}, true);
    r.add_separator(C);
    for (OpType t : {OpType::Face, OpType::Adaptive2D, OpType::Pocket2D, OpType::Contour2D, OpType::Slot, OpType::Drill, OpType::Bore,
                     OpType::Chamfer2D})
        r.add_button(C, std::string("cam_") + op_info(t).id, op_cmd(t));
    r.add_flyout(C, "cam_engrave_menu", {op_cmd(OpType::Engrave), op_cmd(OpType::Trace)});
    r.add_separator(C);
    for (OpType t : {OpType::Adaptive3D, OpType::Parallel3D, OpType::Contour3D})
        r.add_button(C, std::string("cam_") + op_info(t).id, op_cmd(t));
    r.add_separator(C);
    CadCommand wrap = op_cmd(OpType::RotaryWrap), fin = op_cmd(OpType::RotaryFinish);
    r.add_menu_button(C, "cam_rotary", {"sw_cam_rotary_wrap", _L("4-Axis"), _L("4th-axis (A about X) operations"), nullptr}, {wrap, fin});
    r.add_separator(C);
    r.add_button(C, "cam_tools", {"sw_cam_tool_library", _L("Tool\nLibrary"), _L("Tool Library — your cutters, saved for every project"),
                                  [im] { im->tool_library_dialog(); }});
    r.add_button(C, "cam_regenerate", {"sw_cam_regenerate", _L("Regenerate\nAll"), _L("Regenerate every operation's toolpath"),
                                       [im] { im->regenerate_all(); }});
    r.add_button(C, "cam_simulate", {"sw_cam_simulate", _L("Simulate"), _L("Simulate — watch the tool cut the stock"),
                                     [im] { im->sim_open(); }});
    r.add_tail(C, "cam_post", {"sw_cam_post", _L("Post\nProcess"), _L("Post Process — write the G-code file for your machine"),
                               [im] { im->post_dialog(); }}, true);
    m->refresh_ribbon_gates();
}

void CamController::on_tab_entered()
{
    DesignSketchTool& st = m->p.m_viewport->mcp_sketch_tool();
    st.overlay_on = true;
    m->ensure_model();
    m->stock_dirty = m->gl_dirty = true;
    m->refresh_ribbon_gates();
    m->rebuild_tree();
    m->p.show_left_page(false);
    // Re-send the bodies once the page is on screen: bodies fed while the workspace was hidden
    // (a project load, scripted modelling) can still be the previous tessellation on the canvas.
    m->p.CallAfter([this] { if (m->p.m_cam_shown) { m->p.feed_bodies(); m->repaint(); } });
    if (!m->has_bodies())
        m->status(_L("CAM needs a solid: model one in the Modeling tab (or import a STEP), then come back."));
    else if (m->doc.setups.empty())
        m->status(_L("Pick an operation in the CAM ribbon (Adaptive clears material); a Setup is made for you from the model."));
    else
        m->status(wxString::Format(_L("%zu operation(s). Select one to see its toolpath; Simulate or Post Process when ready."),
                                   m->doc.operations.size()));
    m->repaint();
}

void CamController::on_tab_left()
{
    m->sim_close();
    m->close_page();
    m->p.m_viewport->mcp_sketch_tool().overlay_on = false;
    m->repaint();
}

void CamController::on_cad_changed()
{
    if (m->p.m_doc.topo_generation == m->model_gen && !m->model_dirty) return;
    m->model_dirty = true;
    m->doc.mark_model_changed();   // every toolpath is now stale (the tree says "regenerate")
    m->gl_dirty = m->stock_dirty = true;
    if (m->p.m_cam_shown) {
        m->ensure_model();
        m->rebuild_tree();
        m->refresh_ribbon_gates();
    }
}

void CamController::on_solid_pick(int level, int body, int face, int edge)
{
    Impl& I = *m;
    constexpr int kFace = 2, kEdge = 3;
    if (I.page == Impl::Page::Setup && I.pick_wcs) {
        if (body < 0 || body >= int(I.p.m_doc.bodies.size())) return;
        const TopoDS_Shape& sh = I.p.m_doc.bodies[body].shape;
        Vec3d pt;
        bool  ok = false;
        try {
            if (level == kEdge && edge >= 0) {
                const auto c = GeometryEngine::circle_of_edge(GeometryEngine::edge_by_index(sh, edge));
                if (c.ok) { pt = c.base; ok = true; }
                else {
                    const auto pts = GeometryEngine::sample_edge_world(GeometryEngine::edge_by_index(sh, edge));
                    if (!pts.empty()) { pt = pts[pts.size() / 2]; ok = true; }
                }
            } else if (level == kFace && face >= 0) {
                pt = GeometryEngine::face_centroid_world(GeometryEngine::face_by_index(sh, face));
                ok = true;
            }
        } catch (...) {}
        if (!ok) { I.status(_L("Click a face or a round edge for the WCS origin.")); return; }
        // Stored in the part frame (world rotated by the setup's A index), as WcsOrigin wants it.
        I.read_setup_page(I.draft_setup);
        try { pt = world_to_part_frame(I.draft_setup, I.model, pt); } catch (...) {}
        for (int i = 0; i < 3; ++i) I.s_wcs_xyz[i]->SetValue(pt[i]);
        I.s_wcs_custom->SetValue(true);
        I.s_wcs_pick->SetValue(false);
        I.pick_wcs = false;
        I.apply_setup_rows();
        I.status(wxString::Format(_L("WCS origin at %.3f, %.3f, %.3f. ✓ to apply."), pt.x(), pt.y(), pt.z()));
        return;
    }
    if (I.page == Impl::Page::Op && I.pick_active) {
        GeometrySelection& g = I.draft_op.geom;
        if (level == kFace && face >= 0) {
            auto it = std::find_if(g.faces.begin(), g.faces.end(), [&](const FaceRef& f) { return f.body == body && f.face == face; });
            if (it != g.faces.end()) g.faces.erase(it); else g.faces.push_back({body, face});
        } else if (level == kEdge && edge >= 0) {
            auto it = std::find_if(g.edges.begin(), g.edges.end(), [&](const EdgeRef& e) { return e.body == body && e.edge == edge; });
            if (it != g.edges.end()) g.edges.erase(it); else g.edges.push_back({body, edge});
        } else {
            I.status(_L("Click a face or an edge (a corner is not used)."));
            return;
        }
        if (is_3d(I.draft_op.type) && (!g.faces.empty() || !g.edges.empty())) I.o_whole->SetValue(false);
        if ((I.draft_op.type == OpType::Drill || I.draft_op.type == OpType::Bore) && (!g.faces.empty() || !g.edges.empty()))
            I.o_holes_auto->SetValue(false), I.apply_op_rows();
        I.refresh_geom_list();
        I.status(wxString::Format(_L("%zu face(s), %zu edge(s) picked. ✓ to calculate."), g.faces.size(), g.edges.size()));
        return;
    }
    if (level >= 1 && I.page == Impl::Page::None)
        I.status(_L("Open an operation (CAM ribbon) to machine what you pick."));
}

void CamController::on_sketch_pick(int feature)
{
    Impl& I = *m;
    if (I.page != Impl::Page::Op || !I.pick_active) return;
    GeometrySelection& g = I.draft_op.geom;
    g.sketch_feature = g.sketch_feature == feature ? -1 : feature;
    I.refresh_geom_list();
    I.status(g.sketch_feature >= 0 ? _L("Sketch picked. ✓ to calculate.") : _L("Sketch removed."));
}

bool CamController::on_key(wxKeyEvent& e)
{
    Impl& I = *m;
    const int key = e.GetKeyCode();
    if (key == WXK_ESCAPE) {
        if (I.sim_on) I.sim_close();
        else if (I.page != Impl::Page::None) { I.close_page(); I.status(_L("Cancelled.")); }
        else { I.sel_op = I.sel_setup = -1; I.tree->UnselectAll(); I.repaint(); }
        return true;
    }
    if (key == WXK_SPACE && I.sim_on) { I.sim_play(!I.sim_playing); return true; }
    if (key == WXK_DELETE && I.page == Impl::Page::None && I.sel_op >= 0) { I.delete_op(I.sel_op); return true; }
    return false;
}

void CamController::load_recipe(const std::string& blob)
{
    Impl& I = *m;
    if (!I.doc.deserialize(blob)) {
        I.status(_L("Could not read the CAM setups of this project."), true);
        return;
    }
    I.ops_resized();
    I.model_dirty = true;
    I.stock_dirty = true;
    I.rebuild_tree();
    I.refresh_ribbon_gates();
    if (!I.doc.warnings.empty()) I.status(wxString::FromUTF8(I.doc.warnings.front()), true);
}

void CamController::clear()
{
    Impl& I = *m;
    I.sim_close();
    I.close_page();
    I.doc.clear();
    I.sel_op = I.sel_setup = -1;
    I.ops_resized();
    I.model_dirty = I.stock_dirty = true;
    I.rebuild_tree();
    I.refresh_ribbon_gates();
}

std::string CamController::recipe() const { return m->doc.serialize(); }

// =================================================================================================
// Scripted control ("cam_*" over the MCP socket)
// =================================================================================================

json CamController::mcp(const std::string& method, const json& params)
{
    Impl& I = *m;
    auto op_json = [&](int i) {
        const CamOperation& op = I.doc.operations[i];
        const Toolpath&     tp = I.doc.paths[i];
        const char* st[] = {"ok", "stale", "error", "suppressed"};
        return json{{"index", i}, {"name", op.name}, {"type", op_info(op.type).id}, {"setup", op.setup_index},
                    {"tool", op.tool_number}, {"state", st[int(I.op_state(i))]}, {"moves", tp.moves.size()},
                    {"time_s", tp.time_s}, {"cut_length", tp.cut_length}, {"error", tp.error},
                    {"warnings", tp.warnings.size()}};
    };
    if (method == "cam_describe") {
        json setups = json::array(), ops = json::array(), tools = json::array();
        for (const CamSetup& s : I.doc.setups) setups.push_back({{"name", s.name}, {"machine", s.machine}, {"material", material_name(s.material)}});
        for (int i = 0; i < int(I.doc.operations.size()); ++i) ops.push_back(op_json(i));
        for (const CamTool& t : I.doc.tools) tools.push_back({{"number", t.number}, {"name", t.name}, {"diameter", t.diameter}});
        json frames = json::array();
        for (const CamSetupFrame& f : I.model.setups)
            frames.push_back({{"stock_min", {f.stock.min.x(), f.stock.min.y(), f.stock.min.z()}},
                              {"stock_max", {f.stock.max.x(), f.stock.max.y(), f.stock.max.z()}}});
        return json{{"setups", setups}, {"operations", ops}, {"tools", tools}, {"frames", frames},
                    {"library", I.library.size()}, {"page", int(I.page)}, {"sim", I.sim_on}};
    }
    if (method == "cam_tab") {   // enter the CAM tab (what clicking it does)
        const std::string tab = params.value("tab", std::string("cam"));
        if (MainFrame* mf = wxGetApp().mainframe)
            mf->select_tab(tab == "modeling" ? TAB_ID_MODELING : tab == "sketch" ? TAB_ID_SKETCH : TAB_ID_CAM);
        return json{{"ok", true}};
    }
    if (method == "cam_new_setup") {
        if (params.value("dialog", false)) { I.open_setup(-1); return json{{"ok", true}}; }
        const int s = I.auto_setup();
        CamSetup& su = I.doc.setups[s];
        if (params.contains("machine")) su.machine = params["machine"].get<std::string>();
        if (params.value("stock", std::string()) == "cylinder") su.stock.kind = StockKind::Cylinder;
        if (params.contains("material")) {
            for (int k = 0; k < kMaterialCount; ++k)
                if (params["material"].get<std::string>() == material_name(CAM::Material(k))) su.material = CAM::Material(k);
        }
        try { update_setup_frames(I.model, I.doc); } catch (...) {}
        I.sel_setup = s; I.sel_op = -1;
        I.after_ops_changed();
        return json{{"ok", true}, {"setup", s}};
    }
    if (method == "cam_view") {   // camera: a named view ("iso", "top", "front", ...) zoomed to the model
        const std::string v = params.value("view", std::string("iso"));
        I.p.m_viewport->set_view(v);
        return json{{"ok", true}};
    }
    if (method == "cam_project_save") {   // the whole project as a 3MF (the CAM recipe rides in it)
        Plater* pl = wxGetApp().plater();
        const int rc = pl ? pl->export_3mf(boost::filesystem::path(params.value("path", std::string()))) : -1;
        return json{{"ok", rc >= 0}, {"rc", rc}};
    }
    if (method == "cam_edit_op") { I.open_op(params.value("op", 0), OpType::Face); return json{{"ok", I.page == Impl::Page::Op}}; }
    if (method == "cam_edit_setup") { I.open_setup(params.value("setup", 0)); return json{{"ok", true}}; }
    if (method == "cam_add_op") {
        const OpInfo* info = op_info_by_id(params.value("type", std::string("adaptive")));
        if (info == nullptr) throw std::runtime_error("unknown operation type");
        I.open_op(-1, info->type);
        if (I.page != Impl::Page::Op) throw std::runtime_error("could not open the operation page");
        // Tool by number or by diameter (flat end mills first).
        if (params.contains("tool_diameter")) {
            const double d = params["tool_diameter"].get<double>();
            for (int k = 0; k < int(I.tool_list.size()); ++k)
                if (std::abs(I.tool_list[k].diameter - d) < 1e-6 && I.tool_list[k].type == ToolType::FlatEndMill) { I.o_tool->SetSelection(k); break; }
            I.refresh_feeds();
        } else if (params.contains("tool")) {
            I.fill_tool_choice(params["tool"].get<int>());
            I.refresh_feeds();
        }
        GeometrySelection& g = I.draft_op.geom;
        if (params.contains("faces")) for (const auto& f : params["faces"]) g.faces.push_back({f[0].get<int>(), f[1].get<int>()});
        if (params.contains("edges")) for (const auto& e : params["edges"]) g.edges.push_back({e[0].get<int>(), e[1].get<int>()});
        if (params.contains("sketch")) g.sketch_feature = params["sketch"].get<int>();
        if (params.contains("whole_model")) { I.o_whole->SetValue(params["whole_model"].get<bool>()); I.o_holes_auto->SetValue(params["whole_model"].get<bool>()); }
        if (params.contains("stepdown")) I.o_stepdown->SetValue(params["stepdown"].get<double>());
        if (params.contains("stepover")) I.o_stepover->SetValue(params["stepover"].get<double>());
        if (params.contains("lead_in_radius")) I.o_lead_r->SetValue(params["lead_in_radius"].get<double>());
        if (params.contains("side")) {
            const std::string sd = params["side"].get<std::string>();
            I.o_side->SetSelection(sd == "inside" ? 1 : sd == "on" ? 2 : 0);
        }
        if (params.contains("hole_min")) I.o_hole_min->SetValue(params["hole_min"].get<double>());
        if (params.contains("hole_max")) I.o_hole_max->SetValue(params["hole_max"].get<double>());
        I.refresh_geom_list();
        I.apply_op_rows();
        if (params.value("confirm", false)) {
            const bool ok = I.confirm_op();
            const int idx = I.sel_op;
            json r{{"ok", ok}};
            if (idx >= 0 && idx < int(I.doc.operations.size())) r["op"] = op_json(idx);
            return r;
        }
        return json{{"ok", true}, {"page", "op"}};
    }
    if (method == "cam_pm_ok") {
        const bool ok = I.page == Impl::Page::Setup ? I.confirm_setup() : I.page == Impl::Page::Op ? I.confirm_op() : false;
        json r{{"ok", ok}};
        if (I.sel_op >= 0 && I.sel_op < int(I.doc.operations.size())) r["op"] = op_json(I.sel_op);
        return r;
    }
    if (method == "cam_pm_cancel") { I.close_page(); return json{{"ok", true}}; }
    if (method == "cam_select") {
        I.sel_op    = params.value("op", -1);
        I.sel_setup = params.value("setup", -1);
        I.rebuild_tree();
        I.on_tree_selected({I.sel_op >= 0 ? Impl::NodeKind::Op : I.sel_setup >= 0 ? Impl::NodeKind::Setup : Impl::NodeKind::Root,
                            I.sel_op >= 0 ? I.sel_op : I.sel_setup, -1});
        return json{{"ok", true}};
    }
    if (method == "cam_regenerate") {
        bool ok;
        if (params.contains("op")) ok = I.regenerate({params["op"].get<int>()}, _L("Generating toolpath…"));
        else { I.regenerate_all(); ok = true; }
        json ops = json::array();
        for (int i = 0; i < int(I.doc.operations.size()); ++i) ops.push_back(op_json(i));
        return json{{"ok", ok}, {"operations", ops}};
    }
    if (method == "cam_simulate") {
        if (params.value("close", false)) { I.sim_close(); return json{{"ok", true}}; }
        if (!I.sim_on) I.sim_open();
        if (!I.sim_on) return json{{"ok", false}};
        if (params.contains("t")) { I.sim_play(false); I.sim_seek(I.sim_total * params["t"].get<double>()); }
        if (params.contains("play")) I.sim_play(params["play"].get<bool>());
        return json{{"ok", true}, {"total_s", I.sim_total}, {"t", I.sim_t}, {"moves", I.sim.size()},
                    {"stock_sim", I.sim_stock_ok}, {"stock_triangles", I.sim_mesh.its.indices.size()}};
    }
    if (method == "cam_post") {
        Impl::PostPreset pp;
        const std::string d = params.value("dialect", std::string("grbl"));
        for (int k = 0; k < 5; ++k) if (d == kDialectIds[k]) pp.dialect = k;
        pp.path  = params.value("path", std::string());
        pp.show  = true;
        pp.write = !pp.path.empty();
        pp.scope = params.value("scope", 2);
        // Modal: open it after this reply so the socket is not held by the dialog's loop.
        wxGetApp().CallAfter([&I, pp] { I.post_dialog(&pp); });
        return json{{"ok", true}};
    }
    if (method == "cam_tool_library") {
        wxGetApp().CallAfter([&I] { I.tool_library_dialog(); });
        return json{{"ok", true}};
    }
    throw std::runtime_error("unknown CAM method: " + method);
}

}} // namespace Slic3r::GUI

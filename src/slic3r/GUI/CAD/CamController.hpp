#pragma once

// The CAM tab of the CAD workspace: ribbon page, CAM tree, Setup/Operation PropertyManager,
// Tool Library and Post Process dialogs, viewport layer (stock, WCS, toolpaths, simulation).
// It lives inside DesignPanel (which owns the CamDocument, next to the CadDocument) and only adds
// UI: every toolpath, feed and G-code line comes from the Slic3r::CAM kernel.

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "libslic3r/CAM/CamDocument.hpp"

class wxWindow;
class wxKeyEvent;
class wxTimer;

namespace Slic3r { namespace GUI {

class DesignPanel;
class CadRibbon;

class CamController
{
public:
    explicit CamController(DesignPanel& panel);
    ~CamController();

    // Built by DesignPanel: the two left-pane pages (tree, PropertyManager) and the simulation
    // bar (shown above the viewport while simulating).
    wxWindow* tree_page() const;
    wxWindow* pm_page() const;
    wxWindow* sim_bar() const;
    void      build_ribbon(CadRibbon& ribbon);

    // Workspace events.
    void on_tab_entered();
    void on_tab_left();
    void on_cad_changed();                                    // the CAD document recomputed / changed
    void on_solid_pick(int level, int body, int face, int edge);
    void on_sketch_pick(int feature);
    bool on_key(wxKeyEvent& e);                               // true = handled
    bool pm_active() const;
    bool busy() const;                                        // toolpaths are being generated (worker running)
    void show_pm(bool pm);                                    // left pane: CAM tree <-> PropertyManager

    // Persistence (Model::cam_recipe).
    void load_recipe(const std::string& blob);
    void clear();
    std::string recipe() const;

    // Scripted control (McpControl: every "cam_*" method).
    nlohmann::json mcp(const std::string& method, const nlohmann::json& params);

    struct Impl;
private:
    std::unique_ptr<Impl> m;
};

}} // namespace Slic3r::GUI

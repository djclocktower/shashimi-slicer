#pragma once

#include <wx/panel.h>
#include <wx/treebase.h>   // wxTreeItemId

#include <functional>
#include <set>
#include <string>
#include <vector>

#include "libslic3r/Color.hpp"

class wxTreeCtrl;

namespace Slic3r {

class CadDocument;
struct CadFeature;

namespace GUI {

// The SolidWorks FeatureManager design tree:
//
//   Part (Shashimi)
//     Solid Bodies(n)      <- every body, with its colour swatch; hidden ones greyed
//     Origin
//     Front Plane / Top Plane / Right Plane    (XZ / XY / YZ)
//     Sketch1              <- a sketch no feature consumes stays at the top level
//     Boss-Extrude1
//       Sketch2            <- a consumed sketch nests under the feature that uses it
//     Fillet1 ...
//
// A view of the document only: it reports what the user points at (on_selected / on_activated /
// on_menu / on_renamed) and DesignPanel decides what that means.
class CadFeatureTree : public wxPanel
{
public:
    enum class NodeKind { None, Part, BodiesFolder, Body, Origin, RefPlane, Feature };
    // index: feature index (Feature), body index (Body), 0/1/2 = XY/XZ/YZ (RefPlane), else -1.
    struct Node
    {
        NodeKind kind{NodeKind::None};
        int      index{-1};
    };
    struct BodyRow
    {
        wxString  label;
        ColorRGBA colour;
        bool      visible{true};
    };

    explicit CadFeatureTree(wxWindow* parent);

    // Rebuild every row from the document. Body rows come from the panel, which owns per-body
    // display state (name, colour, visibility); `conflict(i)` marks feature row i in red.
    // The selected row survives the rebuild and no selection callback fires for it.
    void rebuild(const CadDocument& doc, const std::vector<BodyRow>& bodies, const std::function<bool(int)>& conflict);

    Node selected() const;
    int  selected_feature() const;   // -1 unless a feature row is selected
    int  selected_body() const;      // -1 unless a body row is selected
    void select(Node n);             // fires on_selected like a click (none if already selected)
    void clear_selection();          // silently: the next click on any row reports it
    void edit_label(Node n);         // in-place rename of a feature or body row

    // The first sketch `feature` consumes (the one "Edit Sketch" opens), or -1.
    static int consumed_sketch(const CadDocument& doc, int feature);

    std::function<void(Node)>                        on_selected;
    std::function<void(Node)>                        on_activated;
    std::function<void(Node, const wxPoint& screen)> on_menu;
    std::function<bool(Node, const wxString&)>       on_renamed;   // false vetoes the new label

private:
    wxTreeItemId item_of(Node n) const;
    Node         node_of(const wxTreeItemId& item) const;
    static std::string icon_for(const CadFeature& f);

    wxTreeCtrl*               m_tree{nullptr};
    wxTreeItemId              m_bodies_folder;
    std::vector<wxTreeItemId> m_feature_items;   // by feature index
    std::vector<wxTreeItemId> m_body_items;      // by body index
    wxTreeItemId              m_plane_items[3];  // XY, XZ, YZ
    wxTreeItemId              m_menu_item;       // right-pressed row, its menu opens on release
    bool                      m_rebuilding{false};
    bool                      m_bodies_expanded{true};
    std::set<int>             m_expanded_features;
};

}} // namespace Slic3r::GUI

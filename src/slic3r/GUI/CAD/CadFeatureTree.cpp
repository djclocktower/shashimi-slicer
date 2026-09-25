#include "slic3r/GUI/CAD/CadFeatureTree.hpp"
#include "slic3r/GUI/CAD/CadRibbon.hpp"   // CadTheme, CadRibbon::icon

#include <wx/dcmemory.h>
#include <wx/imaglist.h>
#include <wx/sizer.h>
#include <wx/treectrl.h>
#include <wx/utils.h>   // wxGetMouseState

#include <map>

#include "libslic3r/CAD/CadDocument.hpp"

// The CAD workspace is pinned to English (see DesignPanel.cpp): literals, not catalogue lookups.
#ifdef _L
#undef _L
#endif
#define _L(s) wxString::FromUTF8(s)

namespace Slic3r { namespace GUI {

namespace {

struct NodeData : public wxTreeItemData
{
    explicit NodeData(CadFeatureTree::Node n) : node(n) {}
    CadFeatureTree::Node node;
};

// The sketches a feature consumes, in the order the feature lists them.
std::vector<int> sketch_inputs(const CadFeature& f)
{
    std::vector<int> refs;
    switch (f.type) {
    case CadFeatureType::Extrude:
    case CadFeatureType::Revolve:
    case CadFeatureType::SurfaceExtrude:
    case CadFeatureType::SurfaceRevolve:
    case CadFeatureType::SurfaceFill:
        refs.push_back(f.sketch_ref);
        break;
    case CadFeatureType::Sweep:
        refs.push_back(f.sketch_ref);
        refs.push_back(f.sweep_path_ref);
        break;
    case CadFeatureType::Loft:
    case CadFeatureType::SurfaceLoft:
        refs = f.loft_profile_refs;
        break;
    case CadFeatureType::Rib:
        refs.push_back(f.rib_sketch_ref);
        break;
    default: break;
    }
    return refs;
}

bool is_sketch_like(const CadFeature& f)
{
    return f.type == CadFeatureType::Sketch || f.type == CadFeatureType::Project;
}

// A small swatch in the body's display colour; hollow when the body is hidden.
wxBitmap body_swatch(const ColorRGBA& c, bool visible, const wxColour& bg, int px)
{
    wxBitmap   bmp(px, px);
    wxMemoryDC dc(bmp);
    dc.SetBackground(wxBrush(bg));
    dc.Clear();
    const wxColour col(c.r_uchar(), c.g_uchar(), c.b_uchar());
    dc.SetPen(wxPen(wxColour(0x4D, 0x4D, 0x4D)));
    dc.SetBrush(visible ? wxBrush(col) : *wxTRANSPARENT_BRUSH);
    dc.DrawRoundedRectangle(2, 2, px - 4, px - 4, 2);
    dc.SelectObject(wxNullBitmap);
    return bmp;
}

} // namespace

std::string CadFeatureTree::icon_for(const CadFeature& f)
{
    const bool cut = f.mode == BooleanMode::Cut;
    switch (f.type) {
    case CadFeatureType::Sketch:         return "sw_sketch";
    case CadFeatureType::Project:        return "sw_sketch";
    case CadFeatureType::Extrude:        return cut ? "sw_extrude_cut" : "sw_extrude_boss";
    case CadFeatureType::Revolve:        return cut ? "sw_revolve_cut" : "sw_revolve_boss";
    case CadFeatureType::Sweep:          return "sw_sweep_boss";
    case CadFeatureType::Loft:           return "sw_loft_boss";
    case CadFeatureType::Fillet:         return "sw_fillet";
    case CadFeatureType::Chamfer:        return "sw_chamfer";
    case CadFeatureType::Hole:           return "sw_hole_wizard";
    case CadFeatureType::Thread:         return "sw_helix";
    case CadFeatureType::Helix:          return "sw_helix";
    case CadFeatureType::Shell:          return "sw_shell";
    case CadFeatureType::Draft:          return "sw_draft";
    case CadFeatureType::Pattern:        return f.pattern_circular ? "sw_circular_pattern" : "sw_linear_pattern";
    case CadFeatureType::Mirror:         return "sw_mirror";
    case CadFeatureType::Plane:          return "sw_plane";
    case CadFeatureType::Axis:           return "sw_axis";
    case CadFeatureType::CoordSys:       return "sw_coordsys";
    case CadFeatureType::Boolean:        return "sw_combine";
    case CadFeatureType::Cut:            return "sw_split";
    case CadFeatureType::Import:         return "sw_import_step";
    case CadFeatureType::Transform:      return "sw_move_body";
    case CadFeatureType::Mate:           return "sw_mate";
    case CadFeatureType::Thicken:
    case CadFeatureType::ThickenSurface: return "sw_thicken";
    case CadFeatureType::Rib:            return "sw_rib";
    case CadFeatureType::DeleteFace:     return "sw_delete_face";
    case CadFeatureType::SurfaceExtrude:
    case CadFeatureType::SurfaceRevolve:
    case CadFeatureType::SurfaceOffset:
    case CadFeatureType::SurfaceLoft:
    case CadFeatureType::SurfaceFill:    return "sw_surface";
    }
    return "sw_sketch";
}

int CadFeatureTree::consumed_sketch(const CadDocument& doc, int feature)
{
    if (feature < 0 || feature >= int(doc.features.size())) return -1;
    for (int r : sketch_inputs(doc.features[feature]))
        if (r >= 0 && r < int(doc.features.size()) && is_sketch_like(doc.features[r])) return r;
    return -1;
}

CadFeatureTree::CadFeatureTree(wxWindow* parent)
    : wxPanel(parent, wxID_ANY)
{
    SetBackgroundColour(CadTheme::panel_bg());
    m_tree = new wxTreeCtrl(this, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                            wxTR_HAS_BUTTONS | wxTR_SINGLE | wxTR_NO_LINES | wxTR_FULL_ROW_HIGHLIGHT |
                                wxTR_EDIT_LABELS | wxBORDER_NONE);
    m_tree->SetBackgroundColour(CadTheme::panel_bg());
    m_tree->SetForegroundColour(CadTheme::text());
    auto* s = new wxBoxSizer(wxVERTICAL);
    s->Add(m_tree, 1, wxEXPAND);
    SetSizer(s);

    m_tree->Bind(wxEVT_TREE_SEL_CHANGED, [this](wxTreeEvent&) {
        if (!m_rebuilding && on_selected) on_selected(selected());
    });
    m_tree->Bind(wxEVT_TREE_ITEM_ACTIVATED, [this](wxTreeEvent& e) {
        if (on_activated) on_activated(node_of(e.GetItem()));
    });
    // The context menu. wxEVT_TREE_ITEM_MENU arrives for the keyboard menu key and, in the generic
    // tree, on the right button's PRESS — and a menu popped up under a pressed button takes the
    // release as a pick of whatever item is under the pointer. So a mouse right-click opens the
    // menu on the release instead (wxEVT_RIGHT_UP); the keyboard opens it at once, on the row.
    auto open_menu = [this](const wxTreeItemId& item, const wxPoint& screen) {
        const Node n = node_of(item);
        // After the handler: the owner shows a modal menu, and a nested loop from inside the
        // tree's own event dispatch is how right-click menus have failed to appear before.
        CallAfter([this, n, screen] { if (on_menu) on_menu(n, screen); });
    };
    m_tree->Bind(wxEVT_TREE_ITEM_MENU, [this, open_menu](wxTreeEvent& e) {
        if (!e.GetItem().IsOk()) return;
        m_tree->SelectItem(e.GetItem());   // the row under the pointer, never a stale one
        if (wxGetMouseState().RightIsDown()) {
            m_menu_item = e.GetItem();
            return;
        }
        wxRect r;
        m_tree->GetBoundingRect(e.GetItem(), r, true);
        open_menu(e.GetItem(), m_tree->ClientToScreen(r.GetBottomLeft()));
    });
    m_tree->Bind(wxEVT_RIGHT_UP, [this, open_menu](wxMouseEvent& e) {
        e.Skip();
        if (!m_menu_item.IsOk()) return;
        const wxTreeItemId item = m_menu_item;
        m_menu_item = wxTreeItemId();
        open_menu(item, m_tree->ClientToScreen(e.GetPosition()));
    });
    m_tree->Bind(wxEVT_TREE_BEGIN_LABEL_EDIT, [this](wxTreeEvent& e) {
        const Node n = node_of(e.GetItem());
        if (n.kind != NodeKind::Feature && n.kind != NodeKind::Body) e.Veto();
    });
    m_tree->Bind(wxEVT_TREE_END_LABEL_EDIT, [this](wxTreeEvent& e) {
        if (e.IsEditCancelled()) return;
        wxString label = e.GetLabel();
        label.Trim(true).Trim(false);
        const Node n = node_of(e.GetItem());
        if (label.empty() || !on_renamed || !on_renamed(n, label)) e.Veto();
    });
    auto track = [this](wxTreeEvent& e, bool expanded) {
        if (m_rebuilding) return;
        const Node n = node_of(e.GetItem());
        if (n.kind == NodeKind::BodiesFolder) m_bodies_expanded = expanded;
        else if (n.kind == NodeKind::Feature) {
            if (expanded) m_expanded_features.insert(n.index);
            else          m_expanded_features.erase(n.index);
        }
    };
    m_tree->Bind(wxEVT_TREE_ITEM_EXPANDED,  [track](wxTreeEvent& e) { track(e, true); });
    m_tree->Bind(wxEVT_TREE_ITEM_COLLAPSED, [track](wxTreeEvent& e) { track(e, false); });
}

void CadFeatureTree::rebuild(const CadDocument& doc, const std::vector<BodyRow>& bodies,
                             const std::function<bool(int)>& conflict)
{
    const Node keep = selected();
    m_rebuilding = true;
    m_tree->Freeze();
    m_menu_item = wxTreeItemId();   // about to be destroyed
    m_tree->DeleteAllItems();
    m_feature_items.assign(doc.features.size(), wxTreeItemId());
    m_body_items.assign(bodies.size(), wxTreeItemId());

    // One image list per rebuild: the fixed node icons, each feature icon in use, and a swatch
    // per body (its colour changes with the document).
    const int px = 16;
    auto* images = new wxImageList(FromDIP(px), FromDIP(px));
    std::map<std::string, int> index;
    auto img = [&](const std::string& name) {
        auto it = index.find(name);
        if (it != index.end()) return it->second;
        const int i = images->Add(CadRibbon::icon(name, px, false, this));
        index.emplace(name, i);
        return i;
    };
    std::vector<int> swatch(bodies.size());
    for (size_t b = 0; b < bodies.size(); ++b)
        swatch[b] = images->Add(body_swatch(bodies[b].colour, bodies[b].visible, CadTheme::panel_bg(), FromDIP(px)));

    const int i_part = img("sw_tree_part");
    const wxTreeItemId root = m_tree->AddRoot(_L("Part (Shashimi)"), i_part, i_part, new NodeData({NodeKind::Part, -1}));

    const int i_folder = img("sw_tree_folder");
    m_bodies_folder = m_tree->AppendItem(root, wxString::Format(_L("Solid Bodies(%zu)"), bodies.size()),
                                         i_folder, i_folder, new NodeData({NodeKind::BodiesFolder, -1}));
    for (size_t b = 0; b < bodies.size(); ++b) {
        const wxTreeItemId id = m_tree->AppendItem(m_bodies_folder, bodies[b].label, swatch[b], swatch[b],
                                                   new NodeData({NodeKind::Body, int(b)}));
        m_tree->SetItemTextColour(id, bodies[b].visible ? CadTheme::text() : CadTheme::text_dim());
        m_body_items[b] = id;
    }

    const int i_origin = img("sw_tree_origin");
    m_tree->AppendItem(root, _L("Origin"), i_origin, i_origin, new NodeData({NodeKind::Origin, -1}));
    // SolidWorks order and names: Front = XZ, Top = XY, Right = YZ.
    const int i_plane = img("sw_plane");
    const std::pair<const char*, int> planes[] = {{"Front Plane", 1}, {"Top Plane", 0}, {"Right Plane", 2}};
    for (const auto& [name, base] : planes)
        m_plane_items[base] = m_tree->AppendItem(root, _L(name), i_plane, i_plane, new NodeData({NodeKind::RefPlane, base}));

    // Each sketch nests under the first feature that consumes it; the rest stay at the top level.
    const int nf = int(doc.features.size());
    std::vector<int> owner(static_cast<size_t>(nf), -1);
    std::vector<std::vector<int>> owned(static_cast<size_t>(nf));
    for (int f = 0; f < nf; ++f)
        for (int r : sketch_inputs(doc.features[f]))
            if (r >= 0 && r < f && owner[r] < 0 && is_sketch_like(doc.features[r])) {
                owner[r] = f;
                owned[f].push_back(r);
            }
    auto add_feature = [&](const wxTreeItemId& parent, int f) {
        const CadFeature& feat = doc.features[f];
        const int         i    = img(icon_for(feat));
        const wxTreeItemId id  = m_tree->AppendItem(parent, wxString::FromUTF8(feat.name), i, i,
                                                    new NodeData({NodeKind::Feature, f}));
        // Suppressed reads as suppressed first (it is the user's answer to a conflict), then a
        // mate conflict in red, else normal.
        m_tree->SetItemTextColour(id, !feat.enabled   ? CadTheme::text_dim()
                                      : conflict && conflict(f) ? wxColour(235, 110, 110)
                                                                : CadTheme::text());
        m_feature_items[size_t(f)] = id;
        return id;
    };
    for (int f = 0; f < nf; ++f) {
        if (owner[f] >= 0) continue;
        const wxTreeItemId id = add_feature(root, f);
        for (int s : owned[f]) add_feature(id, s);
        if (!owned[f].empty() && m_expanded_features.count(f)) m_tree->Expand(id);
    }

    m_tree->AssignImageList(images);
    m_tree->Expand(root);
    if (m_bodies_expanded && !bodies.empty()) m_tree->Expand(m_bodies_folder);
    const wxTreeItemId sel = item_of(keep);
    if (sel.IsOk()) m_tree->SelectItem(sel);
    m_tree->Thaw();
    m_rebuilding = false;
}

wxTreeItemId CadFeatureTree::item_of(Node n) const
{
    switch (n.kind) {
    case NodeKind::Feature:
        return (n.index >= 0 && n.index < int(m_feature_items.size())) ? m_feature_items[n.index] : wxTreeItemId();
    case NodeKind::Body:
        return (n.index >= 0 && n.index < int(m_body_items.size())) ? m_body_items[n.index] : wxTreeItemId();
    case NodeKind::RefPlane:
        return (n.index >= 0 && n.index < 3) ? m_plane_items[n.index] : wxTreeItemId();
    case NodeKind::BodiesFolder: return m_bodies_folder;
    case NodeKind::Part:         return m_tree->GetRootItem();
    default:                     return wxTreeItemId();
    }
}

CadFeatureTree::Node CadFeatureTree::node_of(const wxTreeItemId& item) const
{
    if (!item.IsOk()) return {};
    const auto* d = static_cast<const NodeData*>(m_tree->GetItemData(item));
    return d ? d->node : Node{};
}

CadFeatureTree::Node CadFeatureTree::selected() const { return node_of(m_tree->GetSelection()); }

int CadFeatureTree::selected_feature() const
{
    const Node n = selected();
    return n.kind == NodeKind::Feature ? n.index : -1;
}

int CadFeatureTree::selected_body() const
{
    const Node n = selected();
    return n.kind == NodeKind::Body ? n.index : -1;
}

void CadFeatureTree::select(Node n)
{
    const wxTreeItemId id = item_of(n);
    if (!id.IsOk()) return;
    m_tree->EnsureVisible(id);
    m_tree->SelectItem(id);   // no event when the row is already selected, as with a click
}

void CadFeatureTree::clear_selection()
{
    m_rebuilding = true;
    m_tree->UnselectAll();
    m_rebuilding = false;
}

void CadFeatureTree::edit_label(Node n)
{
    const wxTreeItemId id = item_of(n);
    if (!id.IsOk()) return;
    m_tree->SetFocus();
    m_tree->EditLabel(id);
}

}} // namespace Slic3r::GUI

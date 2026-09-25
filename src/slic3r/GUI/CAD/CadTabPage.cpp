#include "CadTabPage.hpp"

#include "slic3r/GUI/LazyPage.hpp"   // apply_dark_ui_to_lazy_panel

#include <wx/glcanvas.h>
#include <wx/sizer.h>
#include <wx/toplevel.h>

namespace Slic3r { namespace GUI {

// Order -1: never prebuilt at idle, like the Design page it replaces; it is heavy and only
// built when a CAD tab is first opened (or by MCP control).
CadWorkspace::CadWorkspace() : Lazy<DesignPanel>("cad_workspace", -1, [this] { return build(); })
{
    when_built([](DesignPanel& panel) { apply_dark_ui_to_lazy_panel(&panel); });
}

DesignPanel* CadWorkspace::ensure_in(CadTabPage* host)
{
    if (!built())
        m_host = host;
    return ensure();
}

DesignPanel* CadWorkspace::build()
{
    if (m_host == nullptr)
        return nullptr;
    auto* panel = new DesignPanel(m_host);
    m_host->GetSizer()->Add(panel, 1, wxEXPAND);
    m_host->Layout();
    return panel;
}

CadTabPage::CadTabPage(wxWindow* parent, CadWorkspace& workspace, DesignPanel::WorkspaceTab tab)
    : wxPanel(parent), m_workspace(workspace), m_tab(tab)
{
    SetSizer(new wxBoxSizer(wxVERTICAL));
    // Shown by the book when its tab is selected.
    Hide();
    m_workspace.offer_host(this);
}

bool CadTabPage::Show(bool show)
{
    // Move the panel in before this page appears: the book has already hidden the other CAD
    // page, so the reparent happens entirely off screen. While the frame is hidden nothing is
    // built (the book shows its first page as it is inserted); MainFrame::Show() shows the
    // current page again once the frame is up.
    if (show && (DesignPanel::if_built() != nullptr || wxGetTopLevelParent(this)->IsShown()))
        adopt_workspace();
    return wxPanel::Show(show);
}

void CadTabPage::show_workspace()
{
    DesignPanel* panel = adopt_workspace();
    if (panel == nullptr)
        return;
    const bool entering = !m_workspace.shown();
    m_workspace.set_shown(true);
    panel->set_workspace_tab(m_tab);
    // Re-syncs the bed to the active printer, borrows the shared camera and repaints.
    if (entering)
        panel->on_tab_shown();
}

// The GL canvas keeps its size across a reparent, so it gets no size event of its own, and a
// native GL surface may need one to rebind to its new parent window (NSOpenGLContext on macOS).
// The repaint stands in for DesignPanel::on_tab_shown()'s when the move is between CAD tabs.
static void refresh_gl_canvases(wxWindow* window)
{
    for (wxWindow* child : window->GetChildren()) {
        if (dynamic_cast<wxGLCanvas*>(child) != nullptr) {
            child->SendSizeEvent();
            child->Refresh();
        } else {
            refresh_gl_canvases(child);
        }
    }
}

DesignPanel* CadTabPage::adopt_workspace()
{
    DesignPanel* panel = m_workspace.ensure_in(this);
    if (panel == nullptr || panel->GetParent() == this)
        return panel;

    // Normally both CAD pages are hidden here (see Show()); freeze only if this one is not.
    const bool freeze = IsShown();
    if (freeze)
        Freeze();
    if (wxWindow* old_host = panel->GetParent(); old_host != nullptr && old_host->GetSizer() != nullptr)
        old_host->GetSizer()->Detach(panel);
    panel->Reparent(this);
    GetSizer()->Add(panel, 1, wxEXPAND);
    Layout();
    SendSizeEvent();
    refresh_gl_canvases(panel);
    if (freeze)
        Thaw();
    return panel;
}

}} // namespace Slic3r::GUI

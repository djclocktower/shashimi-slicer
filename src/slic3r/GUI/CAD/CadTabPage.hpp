#pragma once

#include <wx/panel.h>

#include "slic3r/GUI/CAD/DesignPanel.hpp"
#include "slic3r/GUI/Lazy.hpp"

namespace Slic3r { namespace GUI {

class CadTabPage;

// The one DesignPanel shown under both CAD tabs (Sketch and Modeling), and the holder that
// DesignPanel::ensure() / if_built() reach. A LazyPage cannot hold it: a LazyPage's panel lives
// in that page for good, while this panel moves to whichever CAD page is shown. MainFrame owns
// one per frame; the panel's wx parent (a CadTabPage) owns the panel.
class CadWorkspace : public Lazy<DesignPanel>
{
public:
    CadWorkspace();

    // Builds the panel if needed, inside `host` when it is built by this call.
    DesignPanel* ensure_in(CadTabPage* host);
    // Where a build that no page asked for goes (MCP control builds through the statics).
    void offer_host(CadTabPage* host)
    {
        if (m_host == nullptr)
            m_host = host;
    }
    // Whether a CAD tab is the selected page, so moving between the two is not a fresh entry.
    bool shown() const { return m_shown; }
    void set_shown(bool shown) { m_shown = shown; }

private:
    DesignPanel* build();

    CadTabPage* m_host{ nullptr };
    bool        m_shown{ false };
};

// A top-bar page (Sketch or Modeling) that hosts the shared DesignPanel while its tab is
// selected. The book hides the old page before it shows the new one, so the panel is moved
// here from Show(), while both CAD pages are hidden; show_workspace() then activates it.
class CadTabPage : public wxPanel
{
public:
    CadTabPage(wxWindow* parent, CadWorkspace& workspace, DesignPanel::WorkspaceTab tab);

    DesignPanel::WorkspaceTab tab() const { return m_tab; }

    // This page's tab was selected: move the panel here if it is elsewhere and switch it to
    // this tab's tool set. Coming from a non-CAD page it also runs DesignPanel::on_tab_shown();
    // coming from the other CAD tab it does not, since the workspace never left the screen and
    // on_tab_shown() would replace the live sketch guidance with the idle hint.
    void show_workspace();
    // A non-CAD page was selected. The panel stays where it is, hidden with its page; MainFrame
    // calls DesignPanel::on_tab_hidden().
    void hide_workspace() { m_workspace.set_shown(false); }

    // Forwarded like LazyPage::Show(): builds nothing while the frame is still hidden.
    bool Show(bool show = true) override;

private:
    // Builds the panel if needed and makes it this page's only child.
    DesignPanel* adopt_workspace();

    CadWorkspace&             m_workspace;
    DesignPanel::WorkspaceTab m_tab;
};

}} // namespace Slic3r::GUI

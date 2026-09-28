#pragma once

#include <wx/panel.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

class wxBoxSizer;
class wxScrolledWindow;
class wxSizer;
class wxStaticBitmap;
class wxStaticText;

namespace Slic3r { namespace GUI {

class CadToolButton;

// The SolidWorks PropertyManager: what the left pane shows while a command is open.
//
//   [icon] Boss-Extrude1
//   [✓] [✗]
//   | Message: what the command is waiting for |
//   ▾ Direction 1        <- collapsible group boxes holding the command's controls
//     ...
//
// It owns no command state. DesignPanel builds its per-tool controls straight into body() (they
// were always one set of controls shown one card at a time) and wraps them in groups with
// make_group() / wrap_in_group(); ✓ and ✗ call back into DesignPanel's tool_confirm/tool_cancel.
class CadPropertyManager : public wxPanel
{
public:
    explicit CadPropertyManager(wxWindow* parent);

    // The scrolled area the command controls live in.
    wxScrolledWindow* body() const { return m_body; }

    void set_title(const wxString& title, const std::string& icon);
    // The Message box. An empty text hides it.
    void set_message(const wxString& text, const wxColour& colour);
    void set_on_ok(std::function<void()> cb) { m_on_ok = std::move(cb); }
    void set_on_cancel(std::function<void()> cb) { m_on_cancel = std::move(cb); }
    CadToolButton* ok_button() const { return m_ok; }

    // A collapsible group box: a clickable header over `content`. The header is created as a
    // child of `parent`, which must be the window every control in `content` is parented to.
    // Returns the group's own sizer, to be added where the content would have gone.
    wxSizer* make_group(wxWindow* parent, const wxString& title, wxSizer* content, bool expanded = true);
    // Move everything currently in `box` into one group and put that group in `box` instead.
    void     wrap_in_group(wxWindow* parent, wxSizer* box, const wxString& title, bool expanded = true);
    // Open a collapsed group (by the sizer make_group returned).
    void     expand(wxSizer* group);
    bool     is_expanded(wxSizer* group) const;
    // Called after a group opens or closes, before the relayout: a page that shows only some of a
    // group's rows (the CAM operation page) hides the others again here, since opening a group
    // shows everything in it.
    void     set_on_group_toggled(std::function<void()> cb) { m_on_group_toggled = std::move(cb); }
    // Showing a card re-shows everything under it recursively (wxSizer::Show), collapsed group
    // contents included. Call after any card show to put the collapsed ones back.
    void     reapply_collapsed();

    // `text` with line breaks inserted so no line is wider than `width` pixels in `win`'s font.
    // wxStaticText::Wrap() is no substitute: it does nothing when called again with the width it
    // last wrapped at, so a label that is re-set and re-wrapped at the same width stays one line.
    static wxString wrap_text(wxWindow* win, const wxString& text, int width);

private:
    struct Group
    {
        wxSizer*  outer{nullptr};
        wxSizer*  content{nullptr};
        wxWindow* header{nullptr};
        bool      expanded{true};
    };
    void toggle(Group& g);
    void relayout(wxWindow* from);
    void rewrap_message();

    wxStaticBitmap*   m_icon{nullptr};
    wxStaticText*     m_title{nullptr};
    CadToolButton*    m_ok{nullptr};
    CadToolButton*    m_cancel{nullptr};
    wxPanel*          m_msg_panel{nullptr};
    wxStaticText*     m_msg{nullptr};
    wxString          m_msg_text;
    int               m_msg_wrap{-1};
    wxScrolledWindow* m_body{nullptr};
    std::string       m_icon_name;

    std::function<void()>               m_on_ok;
    std::function<void()>               m_on_cancel;
    std::function<void()>               m_on_group_toggled;
    std::vector<std::unique_ptr<Group>> m_groups;
};

}} // namespace Slic3r::GUI

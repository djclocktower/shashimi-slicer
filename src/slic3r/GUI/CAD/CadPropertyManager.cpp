#include "slic3r/GUI/CAD/CadPropertyManager.hpp"
#include "slic3r/GUI/CAD/CadRibbon.hpp"

#include <wx/dcbuffer.h>
#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/statbmp.h>
#include <wx/stattext.h>
#include <wx/textwrapper.h>

#include "slic3r/GUI/Widgets/Label.hpp"

// The CAD workspace is pinned to English (see DesignPanel.cpp): literals, not catalogue lookups.
#ifdef _L
#undef _L
#endif
#define _L(s) wxString::FromUTF8(s)

namespace Slic3r { namespace GUI {

namespace {

// The clickable title band of a group box: ▾/▸ and a bold title on a light band.
class GroupHeader : public wxWindow
{
public:
    GroupHeader(wxWindow* parent, const wxString& title, std::function<void()> on_click, std::function<bool()> expanded)
        : wxWindow(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxBORDER_NONE | wxFULL_REPAINT_ON_RESIZE)
        , m_title(title)
        , m_on_click(std::move(on_click))
        , m_expanded(std::move(expanded))
    {
        SetBackgroundStyle(wxBG_STYLE_PAINT);
        wxFont f = Label::Body_12;
        f.SetWeight(wxFONTWEIGHT_BOLD);
        SetFont(f);
        SetCursor(wxCursor(wxCURSOR_HAND));
        Bind(wxEVT_PAINT, [this](wxPaintEvent&) { paint(); });
        Bind(wxEVT_LEFT_UP, [this](wxMouseEvent&) { if (m_on_click) m_on_click(); });
    }
    bool AcceptsFocus() const override { return false; }
    bool AcceptsFocusFromKeyboard() const override { return false; }

protected:
    wxSize DoGetBestClientSize() const override
    {
        wxClientDC dc(const_cast<GroupHeader*>(this));
        dc.SetFont(GetFont());
        return wxSize(dc.GetTextExtent(m_title).GetWidth() + FromDIP(28), dc.GetCharHeight() + FromDIP(8));
    }

private:
    void paint()
    {
        wxAutoBufferedPaintDC dc(this);
        const wxSize sz = GetClientSize();
        dc.SetBackground(wxBrush(GetParent()->GetBackgroundColour()));
        dc.Clear();
        dc.SetPen(wxPen(CadTheme::separator()));
        dc.SetBrush(wxBrush(CadTheme::group_header_bg()));
        dc.DrawRoundedRectangle(0, 0, sz.x, sz.y, FromDIP(3));
        dc.SetFont(GetFont());
        dc.SetTextForeground(CadTheme::text());
        const int ty = (sz.y - dc.GetCharHeight()) / 2;
        dc.DrawText(m_expanded() ? wxString::FromUTF8("\xE2\x96\xBE") : wxString::FromUTF8("\xE2\x96\xB8"),
                    FromDIP(6), ty);
        dc.DrawText(m_title, FromDIP(20), ty);
    }

    wxString              m_title;
    std::function<void()> m_on_click;
    std::function<bool()> m_expanded;
};

} // namespace

CadPropertyManager::CadPropertyManager(wxWindow* parent)
    : wxPanel(parent, wxID_ANY)
{
    SetBackgroundColour(CadTheme::panel_bg());
    const int m = FromDIP(8);

    auto* title_row = new wxBoxSizer(wxHORIZONTAL);
    m_icon  = new wxStaticBitmap(this, wxID_ANY, wxNullBitmap);
    m_title = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize,
                               wxST_ELLIPSIZE_END | wxST_NO_AUTORESIZE);
    m_title->SetFont(Label::Head_14);
    m_title->SetForegroundColour(CadTheme::text());
    title_row->Add(m_icon, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, FromDIP(6));
    title_row->Add(m_title, 1, wxALIGN_CENTER_VERTICAL);

    auto* btn_row = new wxBoxSizer(wxHORIZONTAL);
    m_ok     = new CadToolButton(this, {"sw_ok", _L("OK"), _L("OK — accept and close"),
                                        [this] { if (m_on_ok) m_on_ok(); }}, CadToolButton::Size::Small);
    m_cancel = new CadToolButton(this, {"sw_cancel", _L("Cancel"), _L("Cancel — discard and close"),
                                        [this] { if (m_on_cancel) m_on_cancel(); }}, CadToolButton::Size::Small);
    btn_row->Add(m_ok, 0, wxRIGHT, FromDIP(4));
    btn_row->Add(m_cancel, 0);

    m_msg_panel = new wxPanel(this, wxID_ANY);
    m_msg_panel->SetBackgroundColour(CadTheme::message_bg());
    m_msg = new wxStaticText(m_msg_panel, wxID_ANY, wxEmptyString);
    m_msg->SetFont(Label::Body_12);
    auto* msg_sizer = new wxBoxSizer(wxVERTICAL);
    msg_sizer->Add(m_msg, 0, wxEXPAND | wxALL, FromDIP(6));
    m_msg_panel->SetSizer(msg_sizer);
    m_msg_panel->Hide();

    m_body = new wxScrolledWindow(this, wxID_ANY);
    m_body->SetBackgroundColour(CadTheme::panel_bg());
    m_body->SetScrollRate(0, FromDIP(10));   // vertical only: never scroll labels out sideways

    auto* root = new wxBoxSizer(wxVERTICAL);
    root->Add(title_row, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, m);
    root->Add(btn_row, 0, wxLEFT | wxRIGHT | wxTOP, m);
    root->Add(m_msg_panel, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, m);
    root->Add(m_body, 1, wxEXPAND | wxTOP, FromDIP(4));
    SetSizer(root);

    Bind(wxEVT_SIZE, [this](wxSizeEvent& e) { e.Skip(); CallAfter([this] { rewrap_message(); }); });
}

void CadPropertyManager::set_title(const wxString& title, const std::string& icon)
{
    if (m_title->GetLabel() != title) m_title->SetLabel(title);
    if (icon != m_icon_name) {
        m_icon_name = icon;
        m_icon->SetBitmap(icon.empty() ? wxNullBitmap : CadRibbon::icon(icon, 20, false, this));
        m_icon->Show(!icon.empty());
        Layout();
    }
}

void CadPropertyManager::set_message(const wxString& text, const wxColour& colour)
{
    if (text == m_msg_text && colour == m_msg->GetForegroundColour()) return;
    m_msg_text = text;
    // The status colours (error red, plane-pick green) are chosen for the dark HUD; on the light
    // message box only the error red reads, so everything else takes the theme text colour.
    m_msg->SetForegroundColour(colour.IsOk() && colour.Red() > 200 && colour.Green() < 150 ? colour : CadTheme::text());
    m_msg_wrap = -1;
    rewrap_message();
}

wxString CadPropertyManager::wrap_text(wxWindow* win, const wxString& text, int width)
{
    struct Joiner : wxTextWrapper
    {
        wxString out;
        void OnOutputLine(const wxString& line) override { out += line; }
        void OnNewLine() override { out += '\n'; }
    } joiner;
    joiner.Wrap(win, text, width);
    return joiner.out;
}

void CadPropertyManager::rewrap_message()
{
    const bool show = !m_msg_text.IsEmpty();
    const int  w    = std::max(GetClientSize().GetWidth() - FromDIP(32), FromDIP(120));
    if (show && w != m_msg_wrap) {
        m_msg->SetLabel(wrap_text(m_msg, m_msg_text, w));
        m_msg_wrap = w;
    }
    if (m_msg_panel->IsShown() != show || show) {
        m_msg_panel->Show(show);
        Layout();
    }
}

wxSizer* CadPropertyManager::make_group(wxWindow* parent, const wxString& title, wxSizer* content, bool expanded)
{
    auto   g  = std::make_unique<Group>();
    Group* gp = g.get();
    gp->header   = new GroupHeader(parent, title, [this, gp] { toggle(*gp); }, [gp] { return gp->expanded; });
    gp->content  = content;
    gp->expanded = expanded;
    gp->outer    = new wxBoxSizer(wxVERTICAL);
    gp->outer->Add(gp->header, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, FromDIP(6));
    gp->outer->Add(content, 0, wxEXPAND | wxBOTTOM, FromDIP(4));
    if (!expanded) gp->outer->Show(content, false);
    m_groups.push_back(std::move(g));
    return gp->outer;
}

void CadPropertyManager::wrap_in_group(wxWindow* parent, wxSizer* box, const wxString& title, bool expanded)
{
    auto* content = new wxBoxSizer(wxVERTICAL);
    while (box->GetItemCount() > 0) {
        wxSizerItem* it     = box->GetItem(size_t(0));
        const int    prop   = it->GetProportion();
        const int    flag   = it->GetFlag();
        const int    border = it->GetBorder();
        if (it->IsWindow()) {
            wxWindow* w = it->GetWindow();
            box->Detach(0);
            content->Add(w, prop, flag, border);
        } else if (it->IsSizer()) {
            wxSizer* s = it->GetSizer();
            box->Detach(0);
            content->Add(s, prop, flag, border);
        } else {
            const wxSize sp = it->GetSpacer();
            box->Detach(0);
            content->Add(sp.GetWidth(), sp.GetHeight(), prop, flag, border);
        }
    }
    box->Add(make_group(parent, title, content, expanded), 0, wxEXPAND);
}

void CadPropertyManager::expand(wxSizer* group)
{
    for (auto& g : m_groups)
        if (g->outer == group && !g->expanded) toggle(*g);
}

bool CadPropertyManager::is_expanded(wxSizer* group) const
{
    for (const auto& g : m_groups)
        if (g->outer == group) return g->expanded;
    return true;
}

void CadPropertyManager::reapply_collapsed()
{
    for (auto& g : m_groups)
        if (!g->expanded && g->outer->IsShown(g->content))
            g->outer->Show(g->content, false);
}

void CadPropertyManager::toggle(Group& g)
{
    g.expanded = !g.expanded;
    g.outer->Show(g.content, g.expanded);
    g.header->Refresh();
    if (m_on_group_toggled) m_on_group_toggled();
    relayout(g.header->GetParent());
}

void CadPropertyManager::relayout(wxWindow* from)
{
    from->InvalidateBestSize();
    m_body->Layout();
    from->Layout();
    m_body->FitInside();
    m_body->Refresh();
}

}} // namespace Slic3r::GUI

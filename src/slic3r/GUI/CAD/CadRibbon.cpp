#include "slic3r/GUI/CAD/CadRibbon.hpp"

#include <wx/dcbuffer.h>
#include <wx/image.h>
#include <wx/menu.h>
#include <wx/scrolwin.h>
#include <wx/simplebook.h>
#include <wx/sizer.h>
#include <wx/tokenzr.h>

#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/wxExtensions.hpp"    // create_scaled_bitmap
#include "slic3r/GUI/Widgets/Label.hpp"

namespace Slic3r { namespace GUI {

bool     CadTheme::dark()            { return wxGetApp().dark_mode(); }
wxColour CadTheme::ribbon_bg()       { return dark() ? wxColour(0x36, 0x36, 0x3C) : wxColour(0xF3, 0xF3, 0xF5); }
wxColour CadTheme::panel_bg()        { return dark() ? wxColour(0x2D, 0x2D, 0x30) : wxColour(0xFB, 0xFB, 0xFD); }
wxColour CadTheme::hover_bg()        { return dark() ? wxColour(0x44, 0x4F, 0x5C) : wxColour(0xDD, 0xEB, 0xF8); }
wxColour CadTheme::hover_border()    { return dark() ? wxColour(0x6A, 0x8C, 0xB0) : wxColour(0x99, 0xC4, 0xEB); }
wxColour CadTheme::pressed_bg()      { return dark() ? wxColour(0x3D, 0x5A, 0x80) : wxColour(0xC4, 0xDE, 0xF6); }
wxColour CadTheme::text()            { return dark() ? wxColour(0xE0, 0xE0, 0xE0) : wxColour(0x22, 0x22, 0x24); }
wxColour CadTheme::text_dim()        { return dark() ? wxColour(0x7A, 0x7A, 0x7E) : wxColour(0xA0, 0xA0, 0xA4); }
wxColour CadTheme::separator()       { return dark() ? wxColour(0x55, 0x55, 0x5C) : wxColour(0xC8, 0xC8, 0xCC); }
wxColour CadTheme::sketch_blue()     { return dark() ? wxColour(0x5B, 0x9B, 0xF0) : wxColour(0x1E, 0x63, 0xC8); }
wxColour CadTheme::group_header_bg() { return dark() ? wxColour(0x3A, 0x3A, 0x40) : wxColour(0xE8, 0xEB, 0xEF); }
wxColour CadTheme::message_bg()      { return dark() ? wxColour(0x4A, 0x46, 0x32) : wxColour(0xFF, 0xFB, 0xDC); }

// ---------------------------------------------------------------------------------------------
// CadToolButton
// ---------------------------------------------------------------------------------------------

// Layout constants, in DIP. Every ribbon button reserves the Large icon slot and two label lines,
// so the whole row shares one baseline however the labels wrap.
static constexpr int kPad        = 4;
static constexpr int kPadX       = 2;
static constexpr int kSlotLarge  = 32;
static constexpr int kIconNormal = 24;
static constexpr int kIconSmall  = 18;
static constexpr int kSmallBox   = 26;
static constexpr int kLabelGap   = 2;
static const wxString kArrow     = wxString::FromUTF8("\xE2\x96\xBE");   // ▾

CadToolButton::CadToolButton(wxWindow* parent, const CadCommand& cmd, Size size)
    : wxWindow(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxBORDER_NONE | wxFULL_REPAINT_ON_RESIZE)
    , m_cmd(cmd)
    , m_size(size)
{
    SetBackgroundStyle(wxBG_STYLE_PAINT);
    SetFont(Label::Body_11);
    load_bitmaps();
    SetToolTip(cmd.tip.IsEmpty() ? cmd.label : cmd.tip);

    Bind(wxEVT_PAINT, &CadToolButton::on_paint, this);
    Bind(wxEVT_LEFT_DOWN, &CadToolButton::on_left_down, this);
    Bind(wxEVT_LEFT_DCLICK, &CadToolButton::on_left_down, this);   // GTK sends DCLICK for a fast 2nd press
    Bind(wxEVT_LEFT_UP, &CadToolButton::on_left_up, this);
    Bind(wxEVT_ENTER_WINDOW, [this](wxMouseEvent&) { m_hover = true; Refresh(); });
    Bind(wxEVT_LEAVE_WINDOW, [this](wxMouseEvent&) { m_hover = m_pressed = false; Refresh(); });
}

int CadToolButton::icon_px() const
{
    switch (m_size) {
    case Size::Small:  return kIconSmall;
    case Size::Normal: return kIconNormal;
    case Size::Large:  return kSlotLarge;
    }
    return kIconNormal;
}

int CadToolButton::label_top() const { return FromDIP(kPad + kSlotLarge + kLabelGap); }

void CadToolButton::load_bitmaps()
{
    m_bmp = m_cmd.icon.empty() ? wxBitmap() : CadRibbon::icon(m_cmd.icon, icon_px(), m_cmd.sketch_glyph, this);
    m_bmp_disabled = m_bmp.IsOk() ? m_bmp.ConvertToDisabled(CadTheme::dark() ? 70 : 255) : wxBitmap();
}

void CadToolButton::set_command(const CadCommand& cmd)
{
    m_cmd = cmd;
    load_bitmaps();
    SetToolTip(cmd.tip.IsEmpty() ? cmd.label : cmd.tip);
    InvalidateBestSize();
    if (wxWindow* p = GetParent()) p->Layout();
    Refresh();
}

void CadToolButton::set_menu(std::vector<CadCommand> items, bool menu_only)
{
    m_menu      = std::move(items);
    m_menu_only = menu_only;
    InvalidateBestSize();
    Refresh();
}

void CadToolButton::set_checked(bool on)
{
    if (m_checked == on) return;
    m_checked = on;
    Refresh();
}

bool CadToolButton::Enable(bool enable)
{
    const bool changed = wxWindow::Enable(enable);
    if (!enable) m_hover = m_pressed = false;
    Refresh();
    return changed;
}

wxSize CadToolButton::DoGetBestClientSize() const
{
    if (m_size == Size::Small)
        return wxSize(FromDIP(kSmallBox), FromDIP(kSmallBox));
    wxClientDC dc(const_cast<CadToolButton*>(this));
    dc.SetFont(GetFont());
    int text_w = 0;
    const int line_h = dc.GetCharHeight();
    wxStringTokenizer lines(m_cmd.label, "\n", wxTOKEN_RET_EMPTY_ALL);
    while (lines.HasMoreTokens()) {
        wxString line = lines.GetNextToken();
        if (!lines.HasMoreTokens() && !m_menu.empty()) line += " " + kArrow;
        text_w = std::max(text_w, dc.GetTextExtent(line).GetWidth());
    }
    // Tight sides (kPadX): the Sketch page has to fit a 1366 px window without scrolling.
    const int w = std::max(FromDIP(icon_px()), text_w) + FromDIP(2 * kPadX);
    const int h = label_top() + 2 * line_h + FromDIP(kPad);
    return wxSize(std::max(w, FromDIP(38)), h);
}

void CadToolButton::on_paint(wxPaintEvent&)
{
    wxAutoBufferedPaintDC dc(this);
    const wxSize sz = GetClientSize();
    const wxColour bg = GetParent() ? GetParent()->GetBackgroundColour() : CadTheme::ribbon_bg();
    dc.SetBackground(wxBrush(bg));
    dc.Clear();

    const bool enabled = IsEnabled();
    if (enabled && (m_checked || m_pressed || m_hover)) {
        dc.SetPen(wxPen(CadTheme::hover_border()));
        dc.SetBrush(wxBrush((m_checked || m_pressed) ? CadTheme::pressed_bg() : CadTheme::hover_bg()));
        dc.DrawRoundedRectangle(0, 0, sz.x, sz.y, FromDIP(3));
    }

    const wxBitmap& bmp = enabled ? m_bmp : m_bmp_disabled;
    if (bmp.IsOk()) {
        const wxSize bs = bmp.GetLogicalSize();
        const int    y  = (m_size == Size::Small) ? (sz.y - bs.y) / 2
                                                  : FromDIP(kPad) + (FromDIP(kSlotLarge) - bs.y) / 2;
        dc.DrawBitmap(bmp, (sz.x - bs.x) / 2, y, true);
    }

    dc.SetTextForeground(enabled ? CadTheme::text() : CadTheme::text_dim());
    if (m_size == Size::Small) {
        if (!m_menu.empty()) {   // a small flyout marks itself in the corner
            dc.SetFont(Label::Body_9);
            const wxSize a = dc.GetTextExtent(kArrow);
            dc.DrawText(kArrow, sz.x - a.x, sz.y - a.y);
        }
        return;
    }
    dc.SetFont(GetFont());
    const int line_h = dc.GetCharHeight();
    int y = label_top();
    wxStringTokenizer lines(m_cmd.label, "\n", wxTOKEN_RET_EMPTY_ALL);
    while (lines.HasMoreTokens()) {
        wxString line = lines.GetNextToken();
        if (!lines.HasMoreTokens() && !m_menu.empty()) line += " " + kArrow;
        const int w = dc.GetTextExtent(line).GetWidth();
        dc.DrawText(line, (sz.x - w) / 2, y);
        y += line_h;
    }
}

void CadToolButton::on_left_down(wxMouseEvent&)
{
    m_pressed = true;
    Refresh();
}

void CadToolButton::on_left_up(wxMouseEvent& e)
{
    if (!m_pressed) return;
    m_pressed = false;
    Refresh();
    if (!GetClientRect().Contains(e.GetPosition())) return;
    // The label row of a flyout (where the ▾ is) opens the variants; the icon runs the face.
    const bool menu = !m_menu.empty()
                   && (m_menu_only || (m_size != Size::Small && e.GetPosition().y >= label_top()));
    if (menu) {
        popup_menu();
        return;
    }
    const std::function<void()> act = m_cmd.action;   // a copy: the action may re-face this button
    if (act) act();
}

void CadToolButton::popup_menu()
{
    wxMenu menu;
    constexpr int kFirstId = 1000;
    int chosen = -1;
    for (size_t i = 0; i < m_menu.size(); ++i) {
        const CadCommand& c  = m_menu[i];
        wxString          label = c.label.IsEmpty() ? c.tip : c.label;
        label.Replace("\n", " ");   // a two-line ribbon label is one menu line
        auto*             it = new wxMenuItem(&menu, kFirstId + int(i), label);
        // Bitmap BEFORE Append: wxGTK builds the image item inside Append (see DesignPanel's
        // append_offer_item for the same trap).
        if (!c.icon.empty())
            it->SetBitmap(CadRibbon::icon(c.icon, 16, c.sketch_glyph, this));
        menu.Append(it);
    }
    menu.Bind(wxEVT_MENU, [&chosen](wxCommandEvent& e) { chosen = e.GetId() - kFirstId; });
    m_hover = true;
    Refresh();
    PopupMenu(&menu, 0, GetSize().y);
    m_hover = false;
    Refresh();
    if (chosen < 0 || chosen >= int(m_menu.size())) return;
    // Run AFTER the menu has closed: an action may open a dialog or another menu of its own.
    const CadCommand c = m_menu[size_t(chosen)];
    if (!m_menu_only)
        set_command(c);
    if (c.action) c.action();
}

// ---------------------------------------------------------------------------------------------
// CadRibbon
// ---------------------------------------------------------------------------------------------

wxBitmap CadRibbon::icon(const std::string& name, int px, bool sketch_glyph, wxWindow* win)
{
    wxBitmap bmp = create_scaled_bitmap(name, win, px);
    if (!sketch_glyph || !bmp.IsOk()) return bmp;
    // Keep the glyph's alpha (its shape), replace its colour. ponytail: wxBitmap(img) drops a HiDPI
    // scale factor, same limitation as the flyout tint this replaces.
    wxImage img = bmp.ConvertToImage();
    if (!img.HasAlpha()) img.InitAlpha();
    const wxColour c = CadTheme::sketch_blue();
    const int      w = img.GetWidth(), h = img.GetHeight();
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            img.SetRGB(x, y, c.Red(), c.Green(), c.Blue());
    return wxBitmap(img);
}

CadRibbon::CadRibbon(wxWindow* parent)
    : wxPanel(parent, wxID_ANY)
{
    SetBackgroundColour(CadTheme::ribbon_bg());

    m_corner = new wxPanel(this, wxID_ANY);
    m_corner->SetBackgroundColour(CadTheme::ribbon_bg());
    m_corner_sizer = new wxBoxSizer(wxVERTICAL);
    m_corner->SetSizer(m_corner_sizer);

    m_book = new wxSimplebook(this, wxID_ANY);
    m_book->SetBackgroundColour(CadTheme::ribbon_bg());
    for (int i = 0; i < kPages; ++i) {
        // Horizontal scroll only: a narrow window must still reach the far-right commands
        // (Send to Plater) rather than clipping them off the edge.
        auto* page = new wxScrolledWindow(m_book, wxID_ANY);
        page->SetBackgroundColour(CadTheme::ribbon_bg());
        page->SetScrollRate(FromDIP(12), 0);
        page->ShowScrollbars(wxSHOW_SB_DEFAULT, wxSHOW_SB_NEVER);
        m_page_sizers[i] = new wxBoxSizer(wxHORIZONTAL);
        m_page_sizers[i]->AddSpacer(FromDIP(4));
        page->SetSizer(m_page_sizers[i]);
        m_pages[i] = page;
        m_book->AddPage(page, wxEmptyString);
    }
    m_book->ChangeSelection(int(m_page));

    m_tail_book = new wxSimplebook(this, wxID_ANY);
    m_tail_book->SetBackgroundColour(CadTheme::ribbon_bg());
    for (int i = 0; i < kPages; ++i) {
        m_tails[i] = new wxPanel(m_tail_book, wxID_ANY);
        m_tails[i]->SetBackgroundColour(CadTheme::ribbon_bg());
        m_tail_sizers[i] = new wxBoxSizer(wxHORIZONTAL);
        m_tails[i]->SetSizer(m_tail_sizers[i]);
        m_tail_book->AddPage(m_tails[i], wxEmptyString);
    }
    m_tail_book->ChangeSelection(int(m_page));

    auto separator = [this] {
        auto* sep = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxSize(1, 1));
        sep->SetBackgroundColour(CadTheme::separator());
        return sep;
    };
    auto* row = new wxBoxSizer(wxHORIZONTAL);
    row->Add(m_corner, 0, wxEXPAND | wxLEFT | wxTOP | wxBOTTOM, FromDIP(3));
    row->Add(separator(), 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP | wxBOTTOM, FromDIP(4));
    row->Add(m_book, 1, wxEXPAND);
    row->Add(separator(), 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP | wxBOTTOM, FromDIP(4));
    row->Add(m_tail_book, 0, wxEXPAND | wxRIGHT, FromDIP(4));
    SetSizer(row);
}

wxScrolledWindow* CadRibbon::page_panel(Page page) const { return m_pages[int(page)]; }
wxBoxSizer*       CadRibbon::page_sizer(Page page) const { return m_page_sizers[int(page)]; }

CadToolButton* CadRibbon::add_button(Page page, const std::string& id, const CadCommand& cmd, bool large)
{
    auto* b = new CadToolButton(page_panel(page), cmd, large ? CadToolButton::Size::Large : CadToolButton::Size::Normal);
    page_sizer(page)->Add(b, 0, wxTOP | wxBOTTOM, FromDIP(3));
    if (!id.empty()) m_buttons[id] = b;
    return b;
}

CadToolButton* CadRibbon::add_flyout(Page page, const std::string& id, std::vector<CadCommand> variants, bool large)
{
    CadToolButton* b = add_button(page, id, variants.front(), large);
    b->set_menu(std::move(variants), false);
    return b;
}

CadToolButton* CadRibbon::add_menu_button(Page page, const std::string& id, const CadCommand& face, std::vector<CadCommand> items)
{
    CadToolButton* b = add_button(page, id, face, false);
    b->set_menu(std::move(items), true);
    return b;
}

void CadRibbon::add_separator(Page page)
{
    auto* sep = new wxPanel(page_panel(page), wxID_ANY, wxDefaultPosition, wxSize(1, 1));
    sep->SetBackgroundColour(CadTheme::separator());
    page_sizer(page)->Add(sep, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP | wxBOTTOM, FromDIP(3));
}

CadToolButton* CadRibbon::add_corner(const std::string& id, const CadCommand& cmd)
{
    auto* b = new CadToolButton(m_corner, cmd, CadToolButton::Size::Small);
    m_corner_sizer->Add(b, 0, wxBOTTOM, FromDIP(1));
    if (!id.empty()) m_buttons[id] = b;
    return b;
}

CadToolButton* CadRibbon::add_tail(Page page, const std::string& id, const CadCommand& cmd, bool large)
{
    auto* b = new CadToolButton(m_tails[int(page)], cmd, large ? CadToolButton::Size::Large : CadToolButton::Size::Normal);
    m_tail_sizers[int(page)]->Add(b, 0, wxTOP | wxBOTTOM, FromDIP(3));
    if (!id.empty()) m_buttons[id] = b;
    return b;
}

void CadRibbon::add_tail_window(Page page, wxWindow* w)
{
    m_tail_sizers[int(page)]->Add(w, 0, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, FromDIP(6));
}

wxWindow* CadRibbon::tail_window(Page page) const { return m_tails[int(page)]; }

void CadRibbon::set_page(Page page)
{
    m_page = page;
    m_book->ChangeSelection(int(page));
    m_tail_book->ChangeSelection(int(page));
    page_panel(page)->FitInside();
    Layout();
}

CadToolButton* CadRibbon::button(const std::string& id) const
{
    auto it = m_buttons.find(id);
    return it == m_buttons.end() ? nullptr : it->second;
}

void CadRibbon::enable(const std::string& id, bool on)
{
    if (CadToolButton* b = button(id)) b->Enable(on);
}

void CadRibbon::check(const std::string& id, bool on)
{
    if (CadToolButton* b = button(id)) b->set_checked(on);
}

}} // namespace Slic3r::GUI

#pragma once

#include <wx/panel.h>
#include <wx/bitmap.h>

#include <functional>
#include <map>
#include <string>
#include <vector>

class wxBoxSizer;
class wxScrolledWindow;
class wxSimplebook;

namespace Slic3r { namespace GUI {

// Colours of the SolidWorks-style front end (ribbon, FeatureManager, PropertyManager), resolved
// against the app theme at call time so a theme switch needs no rebuild.
struct CadTheme
{
    static bool     dark();
    static wxColour ribbon_bg();
    static wxColour panel_bg();
    static wxColour hover_bg();
    static wxColour hover_border();
    static wxColour pressed_bg();
    static wxColour text();
    static wxColour text_dim();
    static wxColour separator();
    static wxColour sketch_blue();
    static wxColour group_header_bg();
    static wxColour message_bg();
};

// One command of the CAD front end: a ribbon button, a flyout row, a PropertyManager ✓/✗.
// The action is whatever DesignPanel already runs for that command; nothing here decides policy.
struct CadCommand
{
    std::string           icon;               // resources/images/<icon>.svg
    wxString              label;              // '\n' splits a ribbon label over two lines
    wxString              tip;
    std::function<void()> action;
    // The design_* glyphs are single-colour grey line art. Re-tinted in the sketch-geometry blue
    // they read as SolidWorks sketch tools without a second copy of every drawing.
    bool                  sketch_glyph{false};
};

// A custom-drawn command button: icon above a short label (SolidWorks CommandManager), or a bare
// icon for the small sizes. Drawn rather than a wxButton so the hover/checked/disabled states look
// the same on every platform, and it never takes keyboard focus: the Design panel's shortcuts are
// a CHAR_HOOK that only works while focus stays inside the viewport, and a focused button used to
// strand it (see DesignPanel::update_action_bar's history).
class CadToolButton : public wxWindow
{
public:
    enum class Size { Small, Normal, Large };

    CadToolButton(wxWindow* parent, const CadCommand& cmd, Size size);

    // Change what the button shows and runs (a flyout takes on its last-picked variant).
    void set_command(const CadCommand& cmd);
    // Attach a flyout: the ▾ marks it, and a click on the label row (or anywhere, when
    // `menu_only`) opens the variants. Picking one runs it and, unless menu_only, makes it the face.
    void set_menu(std::vector<CadCommand> items, bool menu_only);
    void set_checked(bool on);
    bool checked() const { return m_checked; }

    bool Enable(bool enable = true) override;
    bool AcceptsFocus() const override { return false; }
    bool AcceptsFocusFromKeyboard() const override { return false; }

protected:
    wxSize DoGetBestClientSize() const override;

private:
    void load_bitmaps();
    int  icon_px() const;
    int  label_top() const;
    void on_paint(wxPaintEvent& e);
    void on_left_down(wxMouseEvent& e);
    void on_left_up(wxMouseEvent& e);
    void popup_menu();

    CadCommand              m_cmd;
    Size                    m_size;
    std::vector<CadCommand> m_menu;
    bool                    m_menu_only{false};
    wxBitmap                m_bmp;
    wxBitmap                m_bmp_disabled;
    bool                    m_hover{false};
    bool                    m_pressed{false};
    bool                    m_checked{false};
};

// The SolidWorks CommandManager: one row of large command buttons grouped by thin separators,
// with one page of tools per CAD tab (Sketch / Features), a small corner group shared by both on
// the left, and a per-page tail on the right that never scrolls away (Send to Plater, the mode).
class CadRibbon : public wxPanel
{
public:
    enum class Page { Sketch, Features };

    explicit CadRibbon(wxWindow* parent);

    // Build API. `id` names the button for enable()/check()/set_face() later; may be empty.
    CadToolButton* add_button(Page page, const std::string& id, const CadCommand& cmd, bool large = false);
    // A flyout: the face runs `variants[0]` (then the last one picked); ▾ opens all of them.
    CadToolButton* add_flyout(Page page, const std::string& id, std::vector<CadCommand> variants, bool large = false);
    // A button that only opens its menu (Add Relation: there is no default relation).
    CadToolButton* add_menu_button(Page page, const std::string& id, const CadCommand& face, std::vector<CadCommand> items);
    void           add_separator(Page page);
    CadToolButton* add_corner(const std::string& id, const CadCommand& cmd);   // small, left of both pages
    // The right end of a page, outside its scrolling area.
    CadToolButton* add_tail(Page page, const std::string& id, const CadCommand& cmd, bool large = false);
    void           add_tail_window(Page page, wxWindow* w);   // w must be parented to tail_window(page)
    wxWindow*      tail_window(Page page) const;

    void set_page(Page page);
    Page page() const { return m_page; }

    CadToolButton* button(const std::string& id) const;
    void           enable(const std::string& id, bool on);
    void           check(const std::string& id, bool on);

    // Sketch-geometry blue for design_* glyphs (lighter in dark mode, where the SW blue is too dim).
    static wxBitmap icon(const std::string& name, int px, bool sketch_glyph, wxWindow* win);

private:
    wxScrolledWindow* page_panel(Page page) const;
    wxBoxSizer*       page_sizer(Page page) const;

    wxPanel*                               m_corner{nullptr};
    wxBoxSizer*                            m_corner_sizer{nullptr};
    wxSimplebook*                          m_book{nullptr};
    wxScrolledWindow*                      m_pages[2]{nullptr, nullptr};
    wxBoxSizer*                            m_page_sizers[2]{nullptr, nullptr};
    wxSimplebook*                          m_tail_book{nullptr};
    wxPanel*                               m_tails[2]{nullptr, nullptr};
    wxBoxSizer*                            m_tail_sizers[2]{nullptr, nullptr};
    Page                                   m_page{Page::Features};
    std::map<std::string, CadToolButton*>  m_buttons;
};

}} // namespace Slic3r::GUI

// MCP tools that reach any window: list top-level windows and dialogs, read a window's widget
// tree, click buttons, set control values, close dialogs, list and invoke menu items, and
// synthesize mouse / keyboard input. They are what keeps every part of the program in reach,
// including the dialogs and panels no dedicated tool covers. See docs/HLSD/mcp-control.md.

#include "McpUtil.hpp"

#include <cctype>

#include <boost/algorithm/string.hpp>

#include <wx/bookctrl.h>
#include <wx/checkbox.h>
#include <wx/choice.h>
#include <wx/ctrlsub.h>
#include <wx/dialog.h>
#include <wx/glcanvas.h>
#include <wx/menu.h>
#include <wx/radiobut.h>
#include <wx/slider.h>
#include <wx/spinctrl.h>
#include <wx/stattext.h>
#include <wx/stockitem.h>
#include <wx/textctrl.h>
#include <wx/textentry.h>
#include <wx/tglbtn.h>
#include <wx/toplevel.h>
#include <wx/uiaction.h>

#include "slic3r/GUI/BBLTopbar.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/MainFrame.hpp"
#include "slic3r/GUI/Widgets/ComboBox.hpp"
#include "slic3r/GUI/Widgets/SpinInput.hpp"
#include "slic3r/GUI/Widgets/TextInput.hpp"

namespace Slic3r { namespace GUI { namespace Mcp {

namespace {

// ---- addressing -----------------------------------------------------------------------------

std::string class_name(const wxObject* o) { return utf8(wxString(o->GetClassInfo()->GetClassName())); }

// Labels compare without mnemonics, accelerators and trailing ellipses / colons.
std::string normalize_label(const wxString& s)
{
    std::string out = utf8(wxStripMenuCodes(s, wxStrip_Mnemonics | wxStrip_Accel));
    boost::trim(out);
    while (!out.empty() && (out.back() == '.' || out.back() == ':' || out.back() == ' '))
        out.pop_back();
    return boost::to_lower_copy(out);
}

wxWindow* default_window()
{
    for (wxWindow* w : wxTopLevelWindows)
        if (auto* d = dynamic_cast<wxDialog*>(w); d && d->IsModal())
            return w;
    return wxGetApp().mainframe;
}

wxWindow* top_window(const json& params)
{
    const int idx = arg<int>(params, "window", -1);
    if (idx < 0)
        return default_window();
    int i = 0;
    for (wxWindow* w : wxTopLevelWindows)
        if (i++ == idx)
            return w;
    throw ToolError("no window " + std::to_string(idx) + " (see ui_windows)", -32602);
}

std::vector<wxWindow*> children_of(wxWindow* w)
{
    std::vector<wxWindow*> out;
    for (wxWindow* c : w->GetChildren())
        if (!c->IsTopLevel())
            out.push_back(c);
    return out;
}

std::string path_of(wxWindow* top, wxWindow* w)
{
    std::vector<int> steps;
    for (wxWindow* cur = w; cur && cur != top; cur = cur->GetParent()) {
        const auto siblings = children_of(cur->GetParent());
        steps.push_back(int(std::find(siblings.begin(), siblings.end(), cur) - siblings.begin()));
    }
    std::string out;
    for (auto it = steps.rbegin(); it != steps.rend(); ++it)
        out += (out.empty() ? "" : "/") + std::to_string(*it);
    return out;
}

wxWindow* find_by(wxWindow* w, const std::function<bool(wxWindow*)>& pred, bool include_hidden)
{
    for (wxWindow* c : children_of(w)) {
        if (!include_hidden && !c->IsShown())
            continue;
        if (pred(c))
            return c;
        if (wxWindow* found = find_by(c, pred, include_hidden))
            return found;
    }
    return nullptr;
}

// The control a ui_* call addresses: by `path` (from ui_tree), or the first match of
// `label` / `name` / `class` (exact label first, then substring), or the window itself.
wxWindow* target(const json& params)
{
    wxWindow*         top    = top_window(params);
    const std::string path   = arg<std::string>(params, "path", "");
    const std::string label  = normalize_label(from_utf8(arg<std::string>(params, "label", "")));
    const std::string name   = arg<std::string>(params, "name", "");
    const std::string cls    = arg<std::string>(params, "class", "");
    const bool        hidden = arg<bool>(params, "include_hidden", false);
    if (!path.empty()) {
        wxWindow* cur = top;
        std::vector<std::string> steps;
        boost::split(steps, path, boost::is_any_of("/"));
        for (const std::string& s : steps) {
            const auto kids = children_of(cur);
            int        i    = -1;
            try { i = std::stoi(s); } catch (...) {}
            if (i < 0 || i >= int(kids.size()))
                throw ToolError("path '" + path + "' no longer exists; re-read ui_tree", -32602);
            cur = kids[i];
        }
        return cur;
    }
    if (label.empty() && name.empty() && cls.empty())
        return top;
    auto matches = [&](wxWindow* w, bool exact) {
        if (!name.empty() && utf8(w->GetName()) != name)
            return false;
        if (!cls.empty() && class_name(w) != cls)
            return false;
        if (!label.empty()) {
            const std::string l = normalize_label(w->GetLabel());
            return exact ? l == label : l.find(label) != std::string::npos;
        }
        return true;
    };
    if (wxWindow* w = find_by(top, [&](wxWindow* w) { return matches(w, true); }, hidden))
        return w;
    if (!label.empty())
        if (wxWindow* w = find_by(top, [&](wxWindow* w) { return matches(w, false); }, hidden))
            return w;
    throw ToolError("no control matches in window '" + utf8(top->GetLabel()) + "' (see ui_tree)", -32602);
}

// ---- reading ----------------------------------------------------------------------------------

json items_of(wxItemContainerImmutable* c)
{
    json items = json::array();
    for (unsigned i = 0; i < c->GetCount() && i < 200; ++i)
        items.push_back(utf8(c->GetString(i)));
    return items;
}

void add_value(wxWindow* w, json& j)
{
    if (auto* ti = dynamic_cast<::TextInput*>(w); ti && ti->GetTextCtrl() && !dynamic_cast<::ComboBox*>(w))
        j["value"] = utf8(ti->GetTextCtrl()->GetValue());
    if (auto* si = dynamic_cast<::SpinInput*>(w))
        j["value"] = si->GetValue();
    if (auto* cb = dynamic_cast<::ComboBox*>(w)) {
        j["value"]     = utf8(cb->GetValue());
        j["selection"] = cb->GetSelection();
        j["items"]     = items_of(cb);
        return;
    }
    if (auto* ic = dynamic_cast<wxItemContainer*>(w)) {
        j["selection"] = ic->GetSelection();
        j["items"]     = items_of(ic);
        if (ic->GetSelection() != wxNOT_FOUND)
            j["value"] = utf8(ic->GetString(ic->GetSelection()));
    }
    if (auto* te = dynamic_cast<wxTextEntry*>(w); te && !j.contains("value"))
        j["value"] = utf8(te->GetValue());
    if (auto* c = dynamic_cast<wxCheckBox*>(w))     j["value"] = c->GetValue();
    if (auto* c = dynamic_cast<wxToggleButton*>(w)) j["value"] = c->GetValue();
    if (auto* c = dynamic_cast<wxRadioButton*>(w))  j["value"] = c->GetValue();
    if (auto* c = dynamic_cast<wxSpinCtrl*>(w))     j["value"] = c->GetValue();
    if (auto* c = dynamic_cast<wxSpinCtrlDouble*>(w)) j["value"] = c->GetValue();
    if (auto* c = dynamic_cast<wxSlider*>(w)) {
        j["value"] = c->GetValue();
        j["min"]   = c->GetMin();
        j["max"]   = c->GetMax();
    }
    if (auto* b = dynamic_cast<wxBookCtrlBase*>(w)) {
        j["selection"] = b->GetSelection();
        json pages     = json::array();
        for (size_t i = 0; i < b->GetPageCount(); ++i)
            pages.push_back(utf8(b->GetPageText(i)));
        j["pages"] = pages;
    }
}

json node(wxWindow* top, wxWindow* w, int depth, bool include_hidden)
{
    json j{{"path", path_of(top, w)}, {"class", class_name(w)}};
    if (const wxString l = w->GetLabel(); !l.empty())
        j["label"] = utf8(wxStripMenuCodes(l, wxStrip_Mnemonics));
    if (const wxString n = w->GetName(); !n.empty() && n != "panel" && n != "staticText" && n != "button")
        j["name"] = utf8(n);
    if (wxIsStockID(w->GetId()))   // a standard id (OK, Cancel, Yes...) says what a button does
        j["stock_id"] = utf8(wxStripMenuCodes(wxGetStockLabel(w->GetId(), wxSTOCK_NOFLAGS)));
    if (!w->IsShown())   j["hidden"] = true;
    if (!w->IsEnabled()) j["disabled"] = true;
    if (const wxString tip = w->GetToolTipText(); !tip.empty())
        j["tooltip"] = utf8(tip);
    add_value(w, j);
    if (depth > 0) {
        json kids = json::array();
        for (wxWindow* c : children_of(w))
            if (include_hidden || c->IsShown())
                kids.push_back(node(top, c, depth - 1, include_hidden));
        if (!kids.empty())
            j["children"] = std::move(kids);
    } else if (!children_of(w).empty())
        j["more_children"] = children_of(w).size();
    return j;
}

json ui_windows(const json&)
{
    json out = json::array();
    int  i   = 0;
    for (wxWindow* w : wxTopLevelWindows) {
        auto*        tlw  = dynamic_cast<wxTopLevelWindow*>(w);
        auto*        dlg  = dynamic_cast<wxDialog*>(w);
        const wxRect r    = w->GetScreenRect();
        json         j{{"window", i++},
                       {"class", class_name(w)},
                       {"title", utf8(w->GetLabel())},
                       {"shown", w->IsShown()},
                       {"modal", dlg != nullptr && dlg->IsModal()},
                       {"active", tlw != nullptr && tlw->IsActive()},
                       {"main", w == wxGetApp().mainframe},
                       {"rect", json::array({r.x, r.y, r.width, r.height})}};
        if (w->IsShown())
            out.push_back(std::move(j));
    }
    return json{{"windows", out}, {"modal_dialog", open_modal_title()}};
}

json ui_tree(const json& params)
{
    wxWindow* top   = top_window(params);
    wxWindow* root  = target(params);
    const int depth = std::clamp(arg<int>(params, "depth", 8), 0, 40);
    return node(top, root, depth, arg<bool>(params, "include_hidden", false));
}

json ui_find(const json& params)
{
    wxWindow*         top    = top_window(params);
    const std::string text   = normalize_label(from_utf8(req<std::string>(params, "text")));
    const bool        hidden = arg<bool>(params, "include_hidden", false);
    json              out    = json::array();
    std::function<void(wxWindow*)> walk = [&](wxWindow* w) {
        for (wxWindow* c : children_of(w)) {
            if (!hidden && !c->IsShown())
                continue;
            json j = node(top, c, 0, hidden);
            std::string hay = normalize_label(c->GetLabel()) + "\n" + boost::to_lower_copy(utf8(c->GetName())) + "\n" +
                              boost::to_lower_copy(j.value("value", json("")).is_string() ? j.value("value", std::string()) : std::string()) + "\n" +
                              boost::to_lower_copy(utf8(c->GetToolTipText()));
            if (hay.find(text) != std::string::npos && out.size() < 100)
                out.push_back(std::move(j));
            walk(c);
        }
    };
    walk(top);
    return json{{"matches", out}};
}

// ---- acting -----------------------------------------------------------------------------------

// Events are queued, not processed inline: a handler that opens another dialog must not run
// on this call's stack, where it would hold the UI thread until the timeout.
void queue_command(wxWindow* w, wxEventType type, int int_value = 0, const wxString& str = wxString())
{
    auto* e = new wxCommandEvent(type, w->GetId());
    e->SetEventObject(w);
    e->SetInt(int_value);
    e->SetString(str);
    w->GetEventHandler()->QueueEvent(e);
}

json ui_click(const json& params)
{
    wxWindow* w = target(params);
    if (!w->IsEnabled())
        throw ToolError("'" + utf8(w->GetLabel()) + "' is disabled", -32012);
    if (auto* c = dynamic_cast<wxCheckBox*>(w)) {
        c->SetValue(!c->GetValue());
        queue_command(w, wxEVT_CHECKBOX, c->GetValue());
    } else if (auto* t = dynamic_cast<wxToggleButton*>(w)) {   // Orca CheckBox, SwitchButton, RadioBox
        t->SetValue(!t->GetValue());
        queue_command(w, wxEVT_TOGGLEBUTTON, t->GetValue());
    } else if (auto* r = dynamic_cast<wxRadioButton*>(w)) {
        r->SetValue(true);
        queue_command(w, wxEVT_RADIOBUTTON, 1);
    } else if (dynamic_cast<wxStaticText*>(w) && !arg<bool>(params, "mouse", false)) {
        throw ToolError("that is a text label; click its control (pass mouse: true to click it anyway)", -32602);
    } else if (arg<bool>(params, "mouse", false)) {
        // A synthetic left click delivered to the window itself, for controls that react to
        // mouse events rather than emitting a button event.
        const wxPoint c(w->GetClientSize().x / 2, w->GetClientSize().y / 2);
        for (wxEventType type : {wxEVT_LEFT_DOWN, wxEVT_LEFT_UP}) {
            auto* e = new wxMouseEvent(type);
            e->SetPosition(c);
            e->SetEventObject(w);
            e->m_leftDown = type == wxEVT_LEFT_DOWN;
            w->GetEventHandler()->QueueEvent(e);
        }
    } else
        queue_command(w, wxEVT_BUTTON);   // wxButton and Orca's Button both answer wxEVT_BUTTON
    return json{{"clicked", class_name(w)}, {"label", utf8(w->GetLabel())}, {"queued", true}};
}

json ui_set_value(const json& params)
{
    wxWindow*  w = target(params);
    const json v = req<json>(params, "value");
    if (!w->IsEnabled())
        throw ToolError("'" + utf8(w->GetLabel()) + "' is disabled", -32012);
    auto as_string = [&]() { return from_utf8(v.is_string() ? v.get<std::string>() : v.dump()); };
    auto as_bool   = [&]() { return v.is_boolean() ? v.get<bool>() : v.is_number() ? v.get<double>() != 0 : as_string() == "true" || as_string() == "1"; };

    // Choose by index (number) or by item text (string).
    auto choose = [&](wxItemContainer* ic, wxEventType type) {
        int n = v.is_number_integer() ? v.get<int>() : ic->FindString(as_string(), false);
        if (n == wxNOT_FOUND || n < 0 || n >= int(ic->GetCount()))
            throw ToolError("no item " + v.dump() + " (items: " + items_of(ic).dump() + ")", -32602);
        ic->SetSelection(n);
        queue_command(w, type, n, ic->GetString(n));
    };

    if (auto* cb = dynamic_cast<::ComboBox*>(w)) {
        choose(cb, wxEVT_COMBOBOX);
    } else if (auto* si = dynamic_cast<::SpinInput*>(w)) {
        si->SetValue(v.is_number() ? v.get<int>() : std::stoi(utf8(as_string())));
        queue_command(w, wxEVT_SPINCTRL, si->GetValue());
    } else if (auto* ti = dynamic_cast<::TextInput*>(w); ti && ti->GetTextCtrl()) {
        w = ti->GetTextCtrl();
        ti->GetTextCtrl()->SetValue(as_string());
        queue_command(w, wxEVT_TEXT_ENTER, 0, as_string());
        w->GetEventHandler()->QueueEvent(new wxFocusEvent(wxEVT_KILL_FOCUS, w->GetId()));
    } else if (auto* c = dynamic_cast<wxCheckBox*>(w)) {
        c->SetValue(as_bool());
        queue_command(w, wxEVT_CHECKBOX, c->GetValue());
    } else if (auto* t = dynamic_cast<wxToggleButton*>(w)) {
        t->SetValue(as_bool());
        queue_command(w, wxEVT_TOGGLEBUTTON, t->GetValue());
    } else if (auto* r = dynamic_cast<wxRadioButton*>(w)) {
        r->SetValue(true);
        queue_command(w, wxEVT_RADIOBUTTON, 1);
    } else if (auto* s = dynamic_cast<wxSpinCtrl*>(w)) {
        s->SetValue(v.is_number() ? v.get<int>() : std::stoi(utf8(as_string())));
        queue_command(w, wxEVT_SPINCTRL, s->GetValue());
    } else if (auto* s = dynamic_cast<wxSpinCtrlDouble*>(w)) {
        s->SetValue(v.is_number() ? v.get<double>() : std::stod(utf8(as_string())));
        auto* e = new wxSpinDoubleEvent(wxEVT_SPINCTRLDOUBLE, w->GetId(), s->GetValue());
        e->SetEventObject(w);
        w->GetEventHandler()->QueueEvent(e);
    } else if (auto* s = dynamic_cast<wxSlider*>(w)) {
        s->SetValue(v.is_number() ? v.get<int>() : std::stoi(utf8(as_string())));
        queue_command(w, wxEVT_SLIDER, s->GetValue());
    } else if (auto* b = dynamic_cast<wxBookCtrlBase*>(w)) {
        if (!v.is_number_integer() || v.get<int>() < 0 || v.get<int>() >= int(b->GetPageCount()))
            throw ToolError("value must be a page index", -32602);
        b->SetSelection(size_t(v.get<int>()));
    } else if (auto* ch = dynamic_cast<wxChoice*>(w)) {
        choose(ch, wxEVT_CHOICE);
    } else if (auto* ic = dynamic_cast<wxItemContainer*>(w)) {
        choose(ic, dynamic_cast<wxComboBox*>(w) ? wxEVT_COMBOBOX : wxEVT_LISTBOX);
    } else if (auto* tc = dynamic_cast<wxTextCtrl*>(w)) {
        tc->SetValue(as_string());
        queue_command(w, wxEVT_TEXT_ENTER, 0, as_string());
        w->GetEventHandler()->QueueEvent(new wxFocusEvent(wxEVT_KILL_FOCUS, w->GetId()));
    } else if (auto* te = dynamic_cast<wxTextEntry*>(w)) {
        te->SetValue(as_string());
    } else
        throw ToolError("a " + class_name(w) + " has no settable value (try ui_click or ui_input)", -32602);
    json j{{"class", class_name(w)}};
    add_value(w, j);
    return j;
}

json ui_close(const json& params)
{
    wxWindow* w = top_window(params);
    if (w == wxGetApp().mainframe)
        throw ToolError("refusing to close the main window", -32602);
    auto* dlg = dynamic_cast<wxDialog*>(w);
    if (dlg && dlg->IsModal()) {
        static const std::map<std::string, int> ids{{"ok", wxID_OK}, {"cancel", wxID_CANCEL}, {"yes", wxID_YES},
                                                    {"no", wxID_NO}, {"apply", wxID_APPLY}, {"close", wxID_CLOSE}};
        const json r    = arg<json>(params, "result", json("cancel"));
        int        code = wxID_CANCEL;
        if (r.is_number_integer())
            code = r.get<int>();
        else if (r.is_string() && !r.get<std::string>().empty() && r.get<std::string>().size() < 7 &&
                 r.get<std::string>().find_first_not_of("0123456789") == std::string::npos)
            code = std::stoi(r.get<std::string>());
        else if (auto it = ids.find(boost::to_lower_copy(r.get<std::string>())); it != ids.end())
            code = it->second;
        else
            throw ToolError("result must be ok, cancel, yes, no, apply, close or a numeric id", -32602);
        wxGetApp().CallAfter([dlg, code]() {
            for (wxWindow* t : wxTopLevelWindows)   // still there?
                if (t == dlg && dlg->IsModal())
                    dlg->EndModal(code);
        });
        return json{{"closed", utf8(w->GetLabel())}, {"result", code}};
    }
    wxGetApp().CallAfter([w]() {
        for (wxWindow* t : wxTopLevelWindows)
            if (t == w)
                w->Close();
    });
    return json{{"closed", utf8(w->GetLabel())}};
}

// ---- menus ------------------------------------------------------------------------------------

std::vector<std::pair<std::string, wxMenu*>> menu_roots(const std::string& which)
{
    std::vector<std::pair<std::string, wxMenu*>> roots;
    Plater& p = plater();
    if (which == "main") {
        MainFrame* mf = wxGetApp().mainframe;
        if (wxMenuBar* bar = mf->GetMenuBar())
            for (size_t i = 0; i < bar->GetMenuCount(); ++i)
                roots.emplace_back(utf8(bar->GetMenuLabelText(i)), bar->GetMenu(i));
        if (BBLTopbar* tb = mf->topbar()) {
            if (tb->GetFileMenu()) roots.emplace_back("File", tb->GetFileMenu());
            roots.emplace_back("Menu", tb->GetTopMenu());
            roots.emplace_back("Calibration", tb->GetCalibMenu());
        }
    } else if (which == "object")          roots.emplace_back("Object", p.object_menu());
    else if (which == "part")              roots.emplace_back("Part", p.part_menu());
    else if (which == "plate")             roots.emplace_back("Plate", p.plate_menu());
    else if (which == "instance")          roots.emplace_back("Instance", p.instance_menu());
    else if (which == "layer")             roots.emplace_back("Layer", p.layer_menu());
    else if (which == "multi_selection")   roots.emplace_back("Selection", p.multi_selection_menu());
    else if (which == "default")           roots.emplace_back("Default", p.default_menu());
    else
        throw ToolError("menu must be main, object, part, plate, instance, layer, multi_selection or default", -32602);
    roots.erase(std::remove_if(roots.begin(), roots.end(), [](const auto& r) { return r.second == nullptr; }), roots.end());
    return roots;
}

void collect_menu(wxMenu* menu, const std::string& prefix, json& out)
{
    menu->UpdateUI(wxGetApp().mainframe);   // run the items' enable / check conditions
    for (wxMenuItem* item : menu->GetMenuItems()) {
        if (item->IsSeparator())
            continue;
        const std::string path = prefix + "/" + utf8(item->GetItemLabelText());
        if (wxMenu* sub = item->GetSubMenu()) {
            collect_menu(sub, path, out);
            continue;
        }
        json j{{"path", path}, {"id", item->GetId()}, {"enabled", item->IsEnabled()}};
        if (item->IsCheckable())
            j["checked"] = item->IsChecked();
        if (const wxString help = item->GetHelp(); !help.empty())
            j["help"] = utf8(help);
        out.push_back(std::move(j));
    }
}

json ui_menu_list(const json& params)
{
    json out = json::array();
    for (const auto& [label, menu] : menu_roots(arg<std::string>(params, "menu", "main")))
        collect_menu(menu, label, out);
    return json{{"items", out}};
}

json ui_menu_invoke(const json& params)
{
    const std::string which = arg<std::string>(params, "menu", "main");
    const std::string path  = arg<std::string>(params, "path", "");
    const int         id    = arg<int>(params, "id", wxID_NONE);
    if (path.empty() && id == wxID_NONE)
        throw ToolError("pass the item's path or id (ui_menu_list)", -32602);

    // Compare paths label by label, so "File/Export/Export G-code..." matches "File/Export/Export G-code".
    std::string norm_path;
    if (!path.empty()) {
        std::vector<std::string> parts;
        boost::split(parts, path, boost::is_any_of("/"));
        for (const auto& part : parts)
            norm_path += (norm_path.empty() ? "" : "/") + normalize_label(from_utf8(part));
    }
    wxMenu*     owner = nullptr;
    wxMenuItem* found = nullptr;
    std::function<void(wxMenu*, const std::string&)> walk = [&](wxMenu* menu, const std::string& prefix) {
        menu->UpdateUI(wxGetApp().mainframe);
        for (wxMenuItem* item : menu->GetMenuItems()) {
            if (found || item->IsSeparator())
                continue;
            const std::string p = prefix + "/" + normalize_label(item->GetItemLabelText());
            if (wxMenu* sub = item->GetSubMenu())
                walk(sub, p);
            else if ((id != wxID_NONE && item->GetId() == id) || (!norm_path.empty() && p == norm_path)) {
                owner = menu;
                found = item;
            }
        }
    };
    for (const auto& [label, menu] : menu_roots(which))
        if (!found)
            walk(menu, normalize_label(from_utf8(label)));
    if (!found)
        throw ToolError("no such menu item (see ui_menu_list)", -32602);
    if (!found->IsEnabled())
        throw ToolError("menu item '" + utf8(found->GetItemLabelText()) + "' is disabled", -32012);

    const int  item_id = found->GetId();
    const bool checked = found->IsCheckable() ? !found->IsChecked() : false;
    if (found->IsCheckable())
        found->Check(checked);
    // Queued, like a real menu selection: the menu's own handlers first, then the main frame's.
    wxGetApp().CallAfter([owner, item_id, checked]() {
        wxCommandEvent e(wxEVT_MENU, item_id);
        e.SetEventObject(owner);
        e.SetInt(checked);
        if (!owner->ProcessEvent(e))
            wxGetApp().mainframe->GetEventHandler()->ProcessEvent(e);
    });
    return json{{"invoked", utf8(found->GetItemLabelText())}, {"id", item_id}, {"queued", true}};
}

// ---- synthesized OS input ----------------------------------------------------------------------

#if wxUSE_UIACTIONSIMULATOR
int key_code(const std::string& k)
{
    static const std::map<std::string, int> named{
        {"enter", WXK_RETURN}, {"return", WXK_RETURN}, {"escape", WXK_ESCAPE}, {"esc", WXK_ESCAPE}, {"tab", WXK_TAB},
        {"space", WXK_SPACE}, {"backspace", WXK_BACK}, {"delete", WXK_DELETE}, {"del", WXK_DELETE}, {"insert", WXK_INSERT},
        {"home", WXK_HOME}, {"end", WXK_END}, {"pageup", WXK_PAGEUP}, {"pagedown", WXK_PAGEDOWN},
        {"left", WXK_LEFT}, {"right", WXK_RIGHT}, {"up", WXK_UP}, {"down", WXK_DOWN}};
    const std::string lk = boost::to_lower_copy(k);
    if (auto it = named.find(lk); it != named.end())
        return it->second;
    if (lk.size() >= 2 && lk[0] == 'f' && std::all_of(lk.begin() + 1, lk.end(), ::isdigit)) {
        const int n = std::stoi(lk.substr(1));
        if (n >= 1 && n <= 24)
            return WXK_F1 + n - 1;
    }
    if (k.size() == 1)
        return std::toupper(static_cast<unsigned char>(k[0]));
    throw ToolError("unknown key '" + k + "'", -32602);
}

json ui_input(const json& params)
{
    const std::string action = req<std::string>(params, "action");
    // Where: the 3D view, a control (window/path/label as elsewhere), or the screen.
    wxWindow* w = nullptr;
    if (arg<std::string>(params, "target", "") == "canvas") {
        GLCanvas3D* c = plater().get_current_canvas3D();
        w             = c ? c->get_wxglcanvas() : nullptr;
        if (w == nullptr)
            throw ToolError("no 3D view is shown", -32602);
    } else if (!params.contains("screen"))
        w = target(params);

    wxPoint at;
    if (params.contains("screen")) {
        const auto s = req<std::vector<int>>(params, "screen");
        if (s.size() != 2)
            throw ToolError("screen must be [x, y]", -32602);
        at = wxPoint(s[0], s[1]);
    } else {
        wxPoint local(w->GetClientSize().x / 2, w->GetClientSize().y / 2);
        if (params.contains("at")) {
            const auto a = req<std::vector<int>>(params, "at");
            if (a.size() != 2)
                throw ToolError("at must be [x, y] in the window's client pixels", -32602);
            local = wxPoint(a[0], a[1]);
        }
        at = w->ClientToScreen(local);
        if (wxTopLevelWindow* tlw = dynamic_cast<wxTopLevelWindow*>(wxGetTopLevelParent(w)))
            tlw->Raise();
        w->SetFocus();
    }

    int mods = 0;
    for (const std::string& m : arg<std::vector<std::string>>(params, "modifiers", {})) {
        if (m == "ctrl")       mods |= wxMOD_CONTROL;
        else if (m == "shift") mods |= wxMOD_SHIFT;
        else if (m == "alt")   mods |= wxMOD_ALT;
        else if (m == "cmd")   mods |= wxMOD_CMD;
        else throw ToolError("modifiers are ctrl, shift, alt, cmd", -32602);
    }
    const std::string btn_name = arg<std::string>(params, "button", "left");
    const int button = btn_name == "right" ? wxMOUSE_BTN_RIGHT : btn_name == "middle" ? wxMOUSE_BTN_MIDDLE : wxMOUSE_BTN_LEFT;

    wxUIActionSimulator sim;
    if (action == "move") {
        sim.MouseMove(at);
    } else if (action == "click" || action == "double_click") {
        sim.MouseMove(at);
        action == "click" ? sim.MouseClick(button) : sim.MouseDblClick(button);
    } else if (action == "drag") {
        const auto to = req<std::vector<int>>(params, "to");
        if (to.size() != 2)
            throw ToolError("to must be [x, y] (same frame as at)", -32602);
        const wxPoint end = params.contains("screen") ? wxPoint(to[0], to[1]) : w->ClientToScreen(wxPoint(to[0], to[1]));
        sim.MouseDragDrop(at.x, at.y, end.x, end.y, button);
    } else if (action == "scroll") {
        // No wheel in wxUIActionSimulator: deliver a wheel event to the window instead.
        if (w == nullptr)
            throw ToolError("scroll needs a window target", -32602);
        auto* e = new wxMouseEvent(wxEVT_MOUSEWHEEL);
        e->SetPosition(w->ScreenToClient(at));
        e->m_wheelRotation = arg<int>(params, "amount", 1) * 120;
        e->m_wheelDelta    = 120;
        e->m_linesPerAction = 3;
        e->SetEventObject(w);
        w->GetEventHandler()->QueueEvent(e);
    } else if (action == "key") {
        sim.Char(key_code(req<std::string>(params, "key")), mods);
    } else if (action == "text") {
        sim.Text(req<std::string>(params, "text").c_str());
    } else
        throw ToolError("action must be move, click, double_click, drag, scroll, key or text", -32602);
    return json{{"action", action}, {"screen", json::array({at.x, at.y})}};
}
#endif

} // namespace

void register_ui_tools()
{
    const json win   = param("window", "integer", "top-level window index (ui_windows); default the open modal dialog, else the main window", -1);
    const json path  = param("path", "string", "control path from ui_tree", "");
    const json label = param("label", "string", "or: first control with this label (exact, then substring)", "");
    const json name  = param("name", "string", "or: by window name", "");

    register_tool({"ui_windows", "Top-level windows and dialogs (index, title, class, modal, shown) and the open modal dialog.",
                   json::array(), ModalSafe, ui_windows});
    register_tool({"ui_tree", "Widget tree of a window or of one control: class, label, value, items, enabled, and the path to address it by.",
                   json::array({win, path, label, name, param("depth", "integer", "", 8), param("include_hidden", "boolean", "", false)}),
                   ModalSafe, ui_tree});
    register_tool({"ui_find", "Controls of a window whose label, name, value or tooltip contains a text.",
                   json::array({win, param("text", "string", ""), param("include_hidden", "boolean", "", false)}), ModalSafe, ui_find});
    register_tool({"ui_click", "Click a button, checkbox, toggle or radio button (queued, as a real click). mouse: true sends a raw left click to any control.",
                   json::array({win, path, label, name, param("class", "string", "or: by class", ""), param("mouse", "boolean", "", false)}),
                   ModalSafe, ui_click});
    register_tool({"ui_set_value", "Set a control's value and fire its change event: text, number, checkbox/toggle (bool), choice/combo/list (index or item text), slider, spin, notebook page.",
                   json::array({win, path, label, name, param("value", "any", "new value: string, number or boolean")}),
                   ModalSafe, ui_set_value});
    register_tool({"ui_close", "Close a window; for a modal dialog, end it with a result (as its OK/Cancel/Yes/No button would).",
                   json::array({win, param("result", "string", "ok | cancel | yes | no | apply | close | numeric id", "cancel")}),
                   ModalSafe, ui_close});
    register_tool({"ui_menu_list", "Menu items with path, id, enabled and checked state: the main menus, or a context menu for the current selection.",
                   json::array({param_enum("menu", json::array({"main", "object", "part", "plate", "instance", "layer", "multi_selection", "default"}), "", "main")}),
                   0, ui_menu_list});
    register_tool({"ui_menu_invoke", "Invoke a menu item by path (\"File/Export/Export G-code\") or id. Queued, as a real menu selection.",
                   json::array({param_enum("menu", json::array({"main", "object", "part", "plate", "instance", "layer", "multi_selection", "default"}), "", "main"),
                                param("path", "string", "", ""), param("id", "integer", "", json())}),
                   0, ui_menu_invoke});
#if wxUSE_UIACTIONSIMULATOR
    register_tool({"ui_input",
                   "Synthesize real OS mouse/keyboard input (reaches anything, including the 3D view's gizmos): move, click, double_click, drag, scroll, key, text. "
                   "Target the 3D view (target: canvas), a control (window/path/label) or absolute screen pixels.",
                   json::array({param_enum("action", json::array({"move", "click", "double_click", "drag", "scroll", "key", "text"}), "", json()),
                                param("target", "string", "\"canvas\" for the 3D view", ""), win, path, label,
                                param("at", "array", "[x, y] client pixels in the target; default its centre", json()),
                                param("screen", "array", "[x, y] absolute screen pixels instead of a target", json()),
                                param("to", "array", "drag end, same frame as at/screen", json()),
                                param_enum("button", json::array({"left", "right", "middle"}), "", "left"),
                                param("amount", "integer", "scroll notches, positive = away from the user", 1),
                                param("key", "string", "a character or enter, escape, tab, space, backspace, delete, arrows, home, end, pageup, pagedown, f1..f24", ""),
                                param("modifiers", "array", "ctrl, shift, alt, cmd (with key)", json::array()),
                                param("text", "string", "text to type", "")}),
                   ModalSafe, ui_input});
#endif
}

}}} // namespace Slic3r::GUI::Mcp

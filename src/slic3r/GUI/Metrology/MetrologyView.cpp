// I18N first: wx defines its own _() otherwise.
#include "slic3r/GUI/I18N.hpp"

#include "MetrologyView.hpp"

#include <wx/dcbuffer.h>
#include <wx/graphics.h>
#include <wx/image.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace Slic3r { namespace GUI {

MetrologyView::MetrologyView(wxWindow* parent) : wxPanel(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxFULL_REPAINT_ON_RESIZE | wxWANTS_CHARS)
{
    SetBackgroundStyle(wxBG_STYLE_PAINT);
    SetMinSize(FromDIP(wxSize(320, 240)));
    Bind(wxEVT_PAINT, &MetrologyView::on_paint, this);
    Bind(wxEVT_LEFT_DOWN, &MetrologyView::on_mouse, this);
    Bind(wxEVT_LEFT_UP, &MetrologyView::on_mouse, this);
    Bind(wxEVT_LEFT_DCLICK, &MetrologyView::on_mouse, this);
    Bind(wxEVT_MIDDLE_DOWN, &MetrologyView::on_mouse, this);
    Bind(wxEVT_MIDDLE_UP, &MetrologyView::on_mouse, this);
    Bind(wxEVT_RIGHT_DOWN, &MetrologyView::on_mouse, this);
    Bind(wxEVT_RIGHT_UP, &MetrologyView::on_mouse, this);
    Bind(wxEVT_MOTION, &MetrologyView::on_mouse, this);
    Bind(wxEVT_MOUSEWHEEL, &MetrologyView::on_mouse, this);
    Bind(wxEVT_LEAVE_WINDOW, &MetrologyView::on_mouse, this);
    Bind(wxEVT_SIZE, [this](wxSizeEvent& e) {
        if (m_fit_pending) fit();
        e.Skip();
    });
}

void MetrologyView::set_image(const Metrology::GrayImage& img)
{
    const bool size_changed = img.w != m_img_w || img.h != m_img_h;
    m_img_w                 = img.w;
    m_img_h                 = img.h;
    if (img.empty()) {
        m_bitmap = wxBitmap();
        Refresh();
        return;
    }
    wxImage        wi(img.w, img.h);
    unsigned char* rgb = wi.GetData();
    for (size_t i = 0; i < img.px.size(); ++i) {
        const unsigned char v = (unsigned char) std::min(255.f, std::max(0.f, img.px[i] + 0.5f));
        rgb[3 * i] = rgb[3 * i + 1] = rgb[3 * i + 2] = v;
    }
    m_bitmap = wxBitmap(wi);
    if (size_changed) fit();
    Refresh();
}

void MetrologyView::fit()
{
    const wxSize sz = GetClientSize();
    if (m_img_w <= 0 || sz.x <= 4 || sz.y <= 4) {
        m_fit_pending = true;
        return;
    }
    m_fit_pending = false;
    m_zoom        = std::min(double(sz.x) / m_img_w, double(sz.y) / m_img_h) * 0.98;
    m_ox          = (sz.x - m_img_w * m_zoom) / 2;
    m_oy          = (sz.y - m_img_h * m_zoom) / 2;
    Refresh();
}

wxPoint2DDouble MetrologyView::to_win(const P2& img) const { return {img.x * m_zoom + m_ox, img.y * m_zoom + m_oy}; }

MetrologyView::P2 MetrologyView::to_img(const wxPoint& win) const { return {(win.x - m_ox) / m_zoom, (win.y - m_oy) / m_zoom}; }

void MetrologyView::zoom_at(const wxPoint& at, double factor)
{
    const P2 before = to_img(at);
    m_zoom          = std::min(64.0, std::max(0.05, m_zoom * factor));
    m_ox            = at.x - before.x * m_zoom;
    m_oy            = at.y - before.y * m_zoom;
    Refresh();
}

void MetrologyView::on_mouse(wxMouseEvent& e)
{
    const wxPoint pos = e.GetPosition();
    if (e.GetEventType() == wxEVT_MOUSEWHEEL) {
        zoom_at(pos, e.GetWheelRotation() > 0 ? 1.25 : 0.8);
        return;
    }
    if (e.LeftDClick()) {
        fit();
        return;
    }
    if (e.MiddleDown() || e.RightDown()) {
        m_panning  = true;
        m_pan_last = pos;
        if (!HasCapture()) CaptureMouse();
        return;
    }
    if ((e.MiddleUp() || e.RightUp()) && m_panning) {
        m_panning = false;
        if (HasCapture()) ReleaseMouse();
        return;
    }
    if (e.LeftDown()) {
        SetFocus();
        m_dragging   = true;
        m_drag_start = m_drag_now = pos;
        if (!HasCapture()) CaptureMouse();
        return;
    }
    if (e.LeftUp() && m_dragging) {
        m_dragging = false;
        if (HasCapture()) ReleaseMouse();
        const P2 a = to_img(m_drag_start), b = to_img(pos);
        const bool moved = std::abs(pos.x - m_drag_start.x) + std::abs(pos.y - m_drag_start.y) > 4;
        if (m_drag_kind != DragKind::Click && moved) {
            if (on_drag_done) on_drag_done(a, b);
        } else if (on_click)
            on_click(b);
        Refresh();
        return;
    }
    if (e.Dragging() && m_panning) {
        m_ox += pos.x - m_pan_last.x;
        m_oy += pos.y - m_pan_last.y;
        m_pan_last = pos;
        Refresh();
        return;
    }
    if (e.Dragging() && m_dragging) {
        m_drag_now = pos;
        Refresh();
    }
    if (on_hover) {
        const P2 p = to_img(pos);
        on_hover(p, e.GetEventType() != wxEVT_LEAVE_WINDOW && p.x >= 0 && p.y >= 0 && p.x < m_img_w && p.y < m_img_h);
    }
}

void MetrologyView::on_paint(wxPaintEvent&)
{
    wxAutoBufferedPaintDC dc(this);
    dc.SetBackground(wxBrush(wxColour(12, 12, 14)));
    dc.Clear();
    std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::Create(dc));
    if (!gc) return;
    if (m_bitmap.IsOk()) {
        // Crisp pixels when zoomed in, as a measuring microscope shows them.
        gc->SetInterpolationQuality(m_zoom >= 2 ? wxINTERPOLATION_NONE : wxINTERPOLATION_GOOD);
        gc->DrawBitmap(m_bitmap, m_ox, m_oy, m_img_w * m_zoom, m_img_h * m_zoom);
    } else {
        gc->SetFont(GetFont(), wxColour(150, 150, 150));
        gc->DrawText(_L("No image. Capture a frame or open one."), 16, 16);
    }
    if (on_overlay) on_overlay(*gc);

    // Drag preview.
    if (m_dragging && m_drag_kind != DragKind::Click) {
        gc->SetPen(wxPen(wxColour(255, 210, 0), 1, wxPENSTYLE_SHORT_DASH));
        gc->SetBrush(*wxTRANSPARENT_BRUSH);
        const double x0 = m_drag_start.x, y0 = m_drag_start.y, x1 = m_drag_now.x, y1 = m_drag_now.y;
        switch (m_drag_kind) {
        case DragKind::Line: gc->StrokeLine(x0, y0, x1, y1); break;
        case DragKind::Box: gc->DrawRectangle(std::min(x0, x1), std::min(y0, y1), std::abs(x1 - x0), std::abs(y1 - y0)); break;
        case DragKind::Circle: {
            const double r = std::hypot(x1 - x0, y1 - y0);
            gc->DrawEllipse(x0 - r, y0 - r, 2 * r, 2 * r);
            gc->StrokeLine(x0, y0, x1, y1);
            break;
        }
        default: break;
        }
    }

    // Scale bar: a round length near a sixth of the view.
    const wxSize sz = GetClientSize();
    if (m_mm_per_px > 0 && m_bitmap.IsOk()) {
        const double mm_per_win = m_mm_per_px / m_zoom;
        const double target     = sz.x / 6.0 * mm_per_win;
        double       len        = std::pow(10.0, std::floor(std::log10(std::max(target, 1e-6))));
        for (double m : {2.0, 5.0})
            if (len * m <= target) len = std::pow(10.0, std::floor(std::log10(std::max(target, 1e-6)))) * m;
        const double w  = len / mm_per_win;
        const double x  = 16, y = sz.y - 22;
        gc->SetPen(wxPen(wxColour(255, 255, 255), 2));
        gc->StrokeLine(x, y, x + w, y);
        gc->StrokeLine(x, y - 5, x, y + 5);
        gc->StrokeLine(x + w, y - 5, x + w, y + 5);
        gc->SetFont(GetFont(), wxColour(255, 255, 255));
        gc->DrawText(wxString::Format("%g mm", len), x + w + 8, y - 8);
    }
}

}} // namespace Slic3r::GUI

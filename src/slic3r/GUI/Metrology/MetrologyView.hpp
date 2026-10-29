#pragma once

// The Measure tab's image view: the camera frame on black, with zoom, pan, a cursor readout and
// overlays drawn by the panel, in the style of an image-dimension measuring system. Painted with
// wxGraphicsContext; no OpenGL.

#include "libslic3r/Metrology/MetrologyTypes.hpp"

#include <wx/bitmap.h>
#include <wx/geometry.h>
#include <wx/panel.h>

#include <functional>

class wxGraphicsContext;

namespace Slic3r { namespace GUI {

class MetrologyView : public wxPanel
{
public:
    using P2 = Metrology::P2;

    // What a left-button drag draws: nothing (click only), a line (scan direction and length),
    // a box, or a circle (centre then radius).
    enum class DragKind { Click, Line, Box, Circle };

    explicit MetrologyView(wxWindow* parent);

    void set_image(const Metrology::GrayImage& img);
    bool has_image() const { return m_img_w > 0; }
    void fit();
    void set_drag(DragKind k) { m_drag_kind = k; }
    // mm per image pixel for the scale bar (0 hides it).
    void set_scale(double mm_per_px) { m_mm_per_px = mm_per_px; Refresh(); }

    // Image pixel <-> window pixel.
    wxPoint2DDouble to_win(const P2& img) const;
    P2              to_img(const wxPoint& win) const;
    double          zoom() const { return m_zoom; }

    std::function<void(const P2& a, const P2& b)> on_drag_done;   // image px
    std::function<void(const P2& p)>              on_click;
    std::function<void(const P2& p, bool inside)> on_hover;
    std::function<void(wxGraphicsContext& gc)>    on_overlay;     // window coordinates

private:
    void on_paint(wxPaintEvent&);
    void on_mouse(wxMouseEvent&);
    void zoom_at(const wxPoint& at, double factor);

    wxBitmap m_bitmap;
    int      m_img_w = 0, m_img_h = 0;
    double   m_zoom = 1, m_ox = 0, m_oy = 0;   // window = image * zoom + offset
    double   m_mm_per_px = 0;
    DragKind m_drag_kind = DragKind::Click;
    bool     m_dragging  = false, m_panning = false;
    wxPoint  m_drag_start, m_drag_now, m_pan_last;
    bool     m_fit_pending = true;
};

}} // namespace Slic3r::GUI

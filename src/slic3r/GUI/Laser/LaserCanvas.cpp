#include "LaserCanvas.hpp"
#include "LaserPanel.hpp"

#include "libslic3r/Laser/ImageEngrave.hpp"
#include "libslic3r/Laser/VectorOps.hpp"
#include "slic3r/GUI/3DScene.hpp"
#include "slic3r/GUI/GLModel.hpp"
#include "slic3r/GUI/GLShader.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/I18N.hpp"
#include "slic3r/GUI/OpenGLManager.hpp"

#include <glad/gl.h>
#include <wx/dcbuffer.h>
#include <wx/glcanvas.h>
#include <wx/menu.h>
#include <wx/sizer.h>

#include <algorithm>
#include <cmath>

namespace Slic3r { namespace GUI {

using namespace Slic3r::Laser;

namespace {

constexpr int    kRulerPx  = 20;
constexpr int    kHandlePx = 4;
constexpr int    kRotatePx = 22;
constexpr double kPi       = 3.14159265358979323846;

ColorRGBA rgba(const Rgb& c, float a = 1.f) { return ColorRGBA(c.r / 255.f, c.g / 255.f, c.b / 255.f, a); }

Vec2d unscaled(const Point& p) { return Vec2d(unscale<double>(p.x()), unscale<double>(p.y())); }

double seg_dist(const Vec2d& p, const Vec2d& a, const Vec2d& b)
{
    const Vec2d  ab = b - a;
    const double l2 = ab.squaredNorm();
    const double t  = l2 > 0 ? std::clamp((p - a).dot(ab) / l2, 0., 1.) : 0.;
    return (a + t * ab - p).norm();
}

// Even-odd point in closed paths (mm).
bool inside(const LaserPaths& paths, const Vec2d& p)
{
    bool in = false;
    for (const LaserPath& lp : paths) {
        if (!lp.closed) continue;
        const Points& pts = lp.pts.points;
        for (size_t i = 0, j = pts.size() - 1; i < pts.size(); j = i++) {
            const Vec2d a = unscaled(pts[i]), b = unscaled(pts[j]);
            if ((a.y() > p.y()) != (b.y() > p.y()) && p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x())
                in = !in;
        }
    }
    return in;
}

Matrix4d ortho(double w, double h)
{
    Matrix4d m = Matrix4d::Identity();
    m(0, 0)    = 2. / w;
    m(1, 1)    = -2. / h;
    m(0, 3)    = -1.;
    m(1, 3)    = 1.;
    return m;
}

} // namespace

// Vertices (x, y pairs) for one GLModel, uploaded lazily and kept until the batch is cleared.
struct LaserCanvas::Batch {
    GLModel::Geometry::EPrimitiveType type{GLModel::Geometry::EPrimitiveType::Lines};
    std::vector<float>                v;
    ColorRGBA                         color;
    GLModel                           model;
    bool                              uploaded{false};

    explicit Batch(ColorRGBA c = ColorRGBA::BLACK(), GLModel::Geometry::EPrimitiveType t = GLModel::Geometry::EPrimitiveType::Lines)
        : type(t), color(c) {}
    void seg(const Vec2d& a, const Vec2d& b)
    {
        v.insert(v.end(), {float(a.x()), float(a.y()), float(b.x()), float(b.y())});
    }
    void path(const LaserPath& p)
    {
        const Points& pts = p.pts.points;
        for (size_t i = 1; i < pts.size(); ++i) seg(unscaled(pts[i - 1]), unscaled(pts[i]));
        if (p.closed && pts.size() > 2) seg(unscaled(pts.back()), unscaled(pts.front()));
    }
    void dashed(const Vec2d& a, const Vec2d& b, double dash)
    {
        const double len = (b - a).norm();
        if (len <= 0 || dash <= 0) return;
        const Vec2d d = (b - a) / len;
        for (double t = 0; t < len; t += 2 * dash) seg(a + t * d, a + std::min(t + dash, len) * d);
    }
    void rect(const Vec2d& a, const Vec2d& b)   // filled (Triangles batch)
    {
        v.insert(v.end(), {float(a.x()), float(a.y()), float(b.x()), float(a.y()), float(b.x()), float(b.y()),
                           float(a.x()), float(a.y()), float(b.x()), float(b.y()), float(a.x()), float(b.y())});
    }
    void draw(GLShaderProgram* shader)
    {
        if (v.empty()) return;
        if (!uploaded) {
            GLModel::Geometry g;
            g.format = {type, GLModel::Geometry::EVertexLayout::P2};
            g.color  = color;
            g.reserve_vertices(v.size() / 2);
            g.reserve_indices(v.size() / 2);
            for (size_t i = 0; i + 1 < v.size(); i += 2) {
                g.add_vertex(Vec2f(v[i], v[i + 1]));
                g.add_index(unsigned(i / 2));
            }
            model.reset();
            model.init_from(std::move(g));
            uploaded = true;
        }
        model.set_color(color);
        model.render(shader);
    }
};

// An image shape's texture. Not GLTexture: its load_from_raw_data() fills the mip levels with
// level-0 bytes, which shows garbage as soon as the image is drawn smaller than its pixels.
struct LaserCanvas::Tex {
    GLuint id{0};
    size_t signature{0};
    ~Tex()
    {
        if (id != 0) glsafe(::glDeleteTextures(1, &id));
    }
    void load(const std::vector<unsigned char>& rgba, int w, int h)
    {
        if (id == 0) glsafe(::glGenTextures(1, &id));
        glsafe(::glBindTexture(GL_TEXTURE_2D, id));
        glsafe(::glPixelStorei(GL_UNPACK_ALIGNMENT, 1));
        glsafe(::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data()));
        glsafe(::glGenerateMipmap(GL_TEXTURE_2D));
        glsafe(::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR));
        glsafe(::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));   // pixels stay crisp when zoomed in
        glsafe(::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
        glsafe(::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
        glsafe(::glBindTexture(GL_TEXTURE_2D, 0));
    }
};

LaserCanvas::LaserCanvas(wxWindow* parent, LaserPanel& panel) : wxPanel(parent), m_panel(panel)
{
    auto* grid = new wxFlexGridSizer(2, 2, 0, 0);
    grid->AddGrowableCol(1);
    grid->AddGrowableRow(1);
    auto* corner  = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxSize(kRulerPx, kRulerPx));
    corner->SetBackgroundColour(wxColour(236, 236, 236));
    m_ruler_top  = new wxWindow(this, wxID_ANY, wxDefaultPosition, wxSize(-1, kRulerPx));
    m_ruler_left = new wxWindow(this, wxID_ANY, wxDefaultPosition, wxSize(kRulerPx, -1));
    for (wxWindow* r : {m_ruler_top, m_ruler_left}) {
        r->SetBackgroundStyle(wxBG_STYLE_PAINT);
        r->Bind(wxEVT_PAINT, [this, r](wxPaintEvent&) { paint_ruler(r, r == m_ruler_top); });
    }
    m_gl = OpenGLManager::create_wxglcanvas(*this);
    grid->Add(corner);
    grid->Add(m_ruler_top, 1, wxEXPAND);
    grid->Add(m_ruler_left, 1, wxEXPAND);
    grid->Add(m_gl, 1, wxEXPAND);
    SetSizer(grid);

    m_gl->Bind(wxEVT_PAINT, [this](wxPaintEvent&) {
        wxPaintDC dc(m_gl);
        render();
    });
    m_gl->Bind(wxEVT_ERASE_BACKGROUND, [](wxEraseEvent&) {});
    m_gl->Bind(wxEVT_SIZE, [this](wxSizeEvent& e) {
        if (m_fitted && e.GetSize().x > 50 && e.GetSize().y > 50) zoom_fit();
        refresh();
        e.Skip();
    });
    for (auto type : {wxEVT_LEFT_DOWN, wxEVT_LEFT_UP, wxEVT_LEFT_DCLICK, wxEVT_MIDDLE_DOWN, wxEVT_MIDDLE_UP, wxEVT_RIGHT_DOWN,
                      wxEVT_RIGHT_UP, wxEVT_MOTION, wxEVT_MOUSEWHEEL, wxEVT_LEAVE_WINDOW})
        m_gl->Bind(type, &LaserCanvas::on_mouse, this);
    m_gl->Bind(wxEVT_KEY_DOWN, &LaserCanvas::on_key, this);
    m_gl->Bind(wxEVT_MOUSE_CAPTURE_LOST, [this](wxMouseCaptureLostEvent&) { m_drag = Drag::None; });
}

LaserCanvas::~LaserCanvas()
{
    // GL objects must die with the context current.
    if (m_ctx && m_gl) m_gl->SetCurrent(*m_ctx);
    m_layer_lines.clear();
    m_layer_hatch.clear();
    m_textures.clear();
}

// ---- View ------------------------------------------------------------------------------------------

Vec2d LaserCanvas::to_world(const wxPoint& px) const { return Vec2d((px.x - m_off.x()) / m_scale, (m_off.y() - px.y) / m_scale); }
Vec2d LaserCanvas::to_screen(const Vec2d& p) const { return Vec2d(m_off.x() + p.x() * m_scale, m_off.y() - p.y() * m_scale); }

double LaserCanvas::grid_step() const
{
    for (double s : {1., 2., 5., 10., 20., 50., 100.})
        if (s * m_scale >= 8) return s;
    return 200.;
}

Vec2d LaserCanvas::snap_point(const Vec2d& p) const
{
    if (!snap) return p;
    const double g = grid_step();
    return Vec2d(std::round(p.x() / g) * g, std::round(p.y() / g) * g);
}

void LaserCanvas::zoom_fit()
{
    const wxSize sz = m_gl->GetClientSize();
    if (sz.x < 20 || sz.y < 20) return;
    const LaserDevice& dev = m_panel.device();
    BoundingBoxf       box(Vec2d(0, 0), Vec2d(dev.bed_w, dev.bed_h));
    m_scale = std::min((sz.x - 30) / box.size().x(), (sz.y - 30) / box.size().y());
    m_off   = Vec2d(sz.x / 2. - box.center().x() * m_scale, sz.y / 2. + box.center().y() * m_scale);
    m_fitted = true;   // keeps fitting on resize until the user zooms or pans
    refresh();
}

void LaserCanvas::zoom_by(double f)
{
    const wxSize sz = m_gl->GetClientSize();
    const wxPoint c(sz.x / 2, sz.y / 2);
    const Vec2d   w = to_world(c);
    m_scale         = std::clamp(m_scale * f, 0.05, 400.);
    m_off           = Vec2d(c.x - w.x() * m_scale, c.y + w.y() * m_scale);
    m_fitted        = false;
    m_dirty         = true;
    refresh();
}

void LaserCanvas::refresh()
{
    m_gl->Refresh(false);
    m_ruler_top->Refresh(false);
    m_ruler_left->Refresh(false);
}

void LaserCanvas::document_changed()
{
    m_dirty = true;
    refresh();
}

void LaserCanvas::set_tool(LaserTool tool)
{
    cancel_drawing();
    m_tool = tool;
    m_gl->SetCursor(tool == LaserTool::Select || tool == LaserTool::Node ? wxNullCursor : wxCursor(wxCURSOR_CROSS));
    m_gl->SetFocus();
    refresh();
}

const LaserPaths& LaserCanvas::outline(int index) const
{
    static const LaserPaths empty;
    return index >= 0 && index < int(m_flat.size()) ? m_flat[index] : empty;
}

// ---- Cache -----------------------------------------------------------------------------------------

void LaserCanvas::rebuild_cache()
{
    const LaserDocument& doc = m_panel.doc();
    m_flat.assign(doc.shapes.size(), {});
    m_bbox.assign(doc.shapes.size(), BoundingBoxf());
    m_layer_lines.clear();
    m_layer_hatch.clear();
    for (int l = 0; l < kLayerCount; ++l) {
        m_layer_lines.push_back(std::make_unique<Batch>(rgba(layer_color(l))));
        m_layer_hatch.push_back(std::make_unique<Batch>(rgba(layer_color(l), 0.35f)));
    }
    for (int i = 0; i < int(doc.shapes.size()); ++i) {
        const LaserShape& s = doc.shapes[i];
        if (s.type == ShapeType::Group) continue;
        m_flat[i] = doc.flatten(i, std::max(0.02, 0.5 / m_scale));
        for (const LaserPath& p : m_flat[i])
            for (const Point& pt : p.pts.points) m_bbox[i].merge(unscaled(pt));
        const int layer = std::clamp(s.layer, 0, kLayerCount - 1);
        if (!s.visible || !doc.layers[layer].visible) continue;
        for (const LaserPath& p : m_flat[i]) m_layer_lines[layer]->path(p);
        const LayerMode mode = doc.layers[layer].mode;
        if (s.type != ShapeType::Image && mode != LayerMode::Line && m_bbox[i].defined) {
            // Light preview hatch, ~7 px apart whatever the zoom (the cache is rebuilt on zoom).
            const double interval = std::max(7. / m_scale, std::max(m_bbox[i].size().x(), m_bbox[i].size().y()) / 400.);
            for (const Polyline& pl : hatch_fill(to_expolygons(m_flat[i]), interval, 45, false, true))
                for (size_t k = 1; k < pl.points.size(); ++k) m_layer_hatch[layer]->seg(unscaled(pl.points[k - 1]), unscaled(pl.points[k]));
        }
    }
    // Drop textures of images that are gone.
    for (auto it = m_textures.begin(); it != m_textures.end();)
        it = doc.find(it->first) < 0 ? m_textures.erase(it) : std::next(it);
    m_dirty = false;
}

// ---- Rendering -------------------------------------------------------------------------------------

void LaserCanvas::render()
{
    if (!m_gl->IsShownOnScreen()) return;
    if (m_ctx == nullptr) m_ctx = wxGetApp().init_glcontext(*m_gl);
    if (m_ctx == nullptr || !m_gl->SetCurrent(*m_ctx) || !wxGetApp().init_opengl()) return;
    if (m_dirty) rebuild_cache();

    const LaserDocument& doc = m_panel.doc();
    const LaserDevice&   dev = m_panel.device();
    const wxSize         sz  = m_gl->GetClientSize();
    const double         sf  = m_gl->GetContentScaleFactor();
    glsafe(::glViewport(0, 0, GLsizei(sz.x * sf), GLsizei(sz.y * sf)));
    glsafe(::glClearColor(0.80f, 0.81f, 0.83f, 1.f));
    glsafe(::glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
    glsafe(::glDisable(GL_DEPTH_TEST));
    glsafe(::glDisable(GL_CULL_FACE));   // the shared context may carry GLCanvas3D's culling
    glsafe(::glEnable(GL_BLEND));
    glsafe(::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

    const Matrix4d proj = ortho(std::max(1, sz.x), std::max(1, sz.y));
    Transform3d    view = Transform3d::Identity();
    view.matrix()(0, 0) = m_scale;
    view.matrix()(1, 1) = -m_scale;
    view.matrix()(0, 3) = m_off.x();
    view.matrix()(1, 3) = m_off.y();
    const double px = 1. / m_scale;   // one screen pixel in mm

    GLShaderProgram* flat = wxGetApp().get_shader("flat");
    GLShaderProgram* texs = wxGetApp().get_shader("flat_texture");
    if (flat == nullptr) return;
    auto use_flat = [&] {
        flat->start_using();
        flat->set_uniform("view_model_matrix", view);
        flat->set_uniform("projection_matrix", proj);
    };
    use_flat();

    // Bed and grid.
    {
        Batch bed(ColorRGBA(1.f, 1.f, 1.f, 1.f), GLModel::Geometry::EPrimitiveType::Triangles);
        bed.rect(Vec2d(0, 0), Vec2d(dev.bed_w, dev.bed_h));
        bed.draw(flat);
        const double g = grid_step();
        Batch        minor(ColorRGBA(0.93f, 0.93f, 0.93f, 1.f)), major(ColorRGBA(0.80f, 0.80f, 0.82f, 1.f));
        for (double x = 0; x <= dev.bed_w + 1e-6; x += g)
            (std::fmod(x + 1e-6, 10 * g) < 1e-3 ? major : minor).seg(Vec2d(x, 0), Vec2d(x, dev.bed_h));
        for (double y = 0; y <= dev.bed_h + 1e-6; y += g)
            (std::fmod(y + 1e-6, 10 * g) < 1e-3 ? major : minor).seg(Vec2d(0, y), Vec2d(dev.bed_w, y));
        minor.draw(flat);
        major.draw(flat);
        Batch border(ColorRGBA(0.45f, 0.45f, 0.48f, 1.f));
        border.path(LaserPath{Polyline(Points{Point::new_scale(0, 0), Point::new_scale(dev.bed_w, 0),
                                              Point::new_scale(dev.bed_w, dev.bed_h), Point::new_scale(0, dev.bed_h)}),
                              true});
        border.draw(flat);
        Batch ox(ColorRGBA(0.85f, 0.2f, 0.2f, 1.f)), oy(ColorRGBA(0.2f, 0.65f, 0.2f, 1.f));
        ox.seg(Vec2d(0, 0), Vec2d(20 * px, 0));
        oy.seg(Vec2d(0, 0), Vec2d(0, 20 * px));
        ox.draw(flat);
        oy.draw(flat);
    }
    flat->stop_using();

    // Images as textures (adjusted grayscale).
    if (texs != nullptr) {
        texs->start_using();
        texs->set_uniform("view_model_matrix", view);
        texs->set_uniform("projection_matrix", proj);
        texs->set_uniform("uniform_texture", 0);
        for (const LaserShape& s : doc.shapes) {
            if (s.type != ShapeType::Image || !s.visible || s.image_w <= 0 || s.image_h <= 0 ||
                !doc.layers[std::clamp(s.layer, 0, kLayerCount - 1)].visible)
                continue;
            const size_t sig = s.gray.size() * 31 + size_t(s.image_w) * 7 + size_t((s.brightness + 200) * 13) +
                               size_t((s.contrast + 200) * 17) * 3 + size_t(s.gamma * 1000) * 5 + (s.invert ? 1 : 0) +
                               (s.gray.empty() ? 0 : s.gray[s.gray.size() / 2] * 101);
            auto& tex = m_textures[s.id];
            if (!tex) tex = std::make_unique<Tex>();
            if (tex->signature != sig || tex->id == 0) {
                // Downsample to <= 1024 px for display.
                const int step = std::max(1, std::max(s.image_w, s.image_h) / 1024);
                const int w = s.image_w / step, h = s.image_h / step;
                std::vector<uint8_t> g(size_t(w) * h);
                for (int y = 0; y < h; ++y)
                    for (int x = 0; x < w; ++x) g[size_t(y) * w + x] = s.gray[size_t(y * step) * s.image_w + x * step];
                apply_adjustments(g, w, h, s.brightness, s.contrast, s.gamma, s.invert);
                std::vector<unsigned char> rgba_data(size_t(w) * h * 4);
                for (size_t i = 0; i < g.size(); ++i) {
                    rgba_data[4 * i] = rgba_data[4 * i + 1] = rgba_data[4 * i + 2] = g[i];
                    rgba_data[4 * i + 3] = 255;
                }
                tex->load(rgba_data, w, h);
                tex->signature = sig;
            }
            const double hw = s.width_mm / 2, hh = s.height_mm / 2;
            const Vec2d  c[4] = {s.xform * Vec2d(-hw, hh), s.xform * Vec2d(hw, hh), s.xform * Vec2d(hw, -hh), s.xform * Vec2d(-hw, -hh)};
            const Vec2f  uv[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
            GLModel::Geometry geo;
            geo.format = {GLModel::Geometry::EPrimitiveType::Triangles, GLModel::Geometry::EVertexLayout::P2T2};
            for (int k = 0; k < 4; ++k) geo.add_vertex(Vec2f(float(c[k].x()), float(c[k].y())), uv[k]);
            geo.add_triangle(0, 2, 1);   // counter-clockwise on screen
            geo.add_triangle(0, 3, 2);
            GLModel quad;
            quad.init_from(std::move(geo));
            glsafe(::glActiveTexture(GL_TEXTURE0));
            glsafe(::glBindTexture(GL_TEXTURE_2D, tex->id));
            quad.render(texs);
            glsafe(::glBindTexture(GL_TEXTURE_2D, 0));
        }
        texs->stop_using();
    }

    use_flat();
    for (auto& b : m_layer_hatch) b->draw(flat);
    for (auto& b : m_layer_lines) b->draw(flat);

    // Hover and selection outlines.
    auto outline_of = [&](int root, Batch& b) {
        std::vector<int> stack{root};
        while (!stack.empty()) {
            const int i = stack.back();
            stack.pop_back();
            if (i < 0 || i >= int(m_flat.size())) continue;
            for (const LaserPath& p : m_flat[i]) b.path(p);
            for (int c : doc.children(i)) stack.push_back(c);
        }
    };
    if (m_hover >= 0 && m_drag == Drag::None) {
        Batch hov(ColorRGBA(0.2f, 0.55f, 1.f, 0.6f));
        outline_of(m_hover, hov);
        hov.draw(flat);
    }
    BoundingBoxf box;
    if (selection_box(box)) {
        Batch sel(ColorRGBA(0.1f, 0.45f, 0.95f, 1.f));
        for (int i : m_panel.selection()) outline_of(i, sel);
        sel.draw(flat);
        const Vec2d a = box.min, b = box.max;
        Batch       frame(ColorRGBA(0.1f, 0.1f, 0.1f, 0.8f));
        frame.dashed(a, Vec2d(b.x(), a.y()), 4 * px);
        frame.dashed(Vec2d(b.x(), a.y()), b, 4 * px);
        frame.dashed(b, Vec2d(a.x(), b.y()), 4 * px);
        frame.dashed(Vec2d(a.x(), b.y()), a, 4 * px);
        if (m_tool == LaserTool::Select) {
            const Vec2d top(box.center().x(), b.y());
            frame.seg(top, top + Vec2d(0, kRotatePx * px));
        }
        frame.draw(flat);
        if (m_tool == LaserTool::Select) {
            Batch handles(ColorRGBA(0.1f, 0.45f, 0.95f, 1.f), GLModel::Geometry::EPrimitiveType::Triangles);
            const Vec2d c = box.center(), hs(kHandlePx * px, kHandlePx * px);
            const Vec2d pos[9] = {a, {c.x(), a.y()}, {b.x(), a.y()}, {b.x(), c.y()}, b, {c.x(), b.y()}, {a.x(), b.y()},
                                  {a.x(), c.y()}, {c.x(), b.y() + kRotatePx * px}};
            for (const Vec2d& p : pos) handles.rect(p - hs, p + hs);
            handles.draw(flat);
        }
    }
    // Node Edit vertices.
    if (m_tool == LaserTool::Node) {
        const int ns = node_shape();
        if (ns >= 0) {
            Batch nodes(ColorRGBA(0.95f, 0.45f, 0.1f, 1.f), GLModel::Geometry::EPrimitiveType::Triangles);
            const LaserShape& s = doc.shapes[ns];
            const Vec2d       hs(3 * px, 3 * px);
            for (size_t pi = 0; pi < s.paths.size(); ++pi)
                for (size_t k = 0; k < s.paths[pi].pts.points.size(); ++k) {
                    const Vec2d w = s.xform * unscaled(s.paths[pi].pts.points[k]);
                    const bool  active = int(pi) == m_node_path && int(k) == m_node_pt;
                    nodes.rect(w - hs * (active ? 1.6 : 1.), w + hs * (active ? 1.6 : 1.));
                }
            nodes.draw(flat);
        }
    }
    // Rubber band / drawing preview.
    if (m_drag == Drag::Rubber) {
        Batch rb(ColorRGBA(0.1f, 0.45f, 0.95f, 0.9f));
        const Vec2d a = m_down, b = m_cursor;
        rb.dashed(a, Vec2d(b.x(), a.y()), 3 * px);
        rb.dashed(Vec2d(b.x(), a.y()), b, 3 * px);
        rb.dashed(b, Vec2d(a.x(), b.y()), 3 * px);
        rb.dashed(Vec2d(a.x(), b.y()), a, 3 * px);
        rb.draw(flat);
    }
    if (!m_pts.empty()) {
        Batch       pv(rgba(layer_color(m_panel.current_layer())));
        const Vec2d a = m_pts.front(), b = m_cursor;
        switch (m_tool) {
        case LaserTool::Line:
            for (size_t i = 1; i < m_pts.size(); ++i) pv.seg(m_pts[i - 1], m_pts[i]);
            pv.seg(m_pts.back(), b);
            break;
        case LaserTool::Rect:
            pv.seg(a, {b.x(), a.y()}); pv.seg({b.x(), a.y()}, b); pv.seg(b, {a.x(), b.y()}); pv.seg({a.x(), b.y()}, a);
            break;
        case LaserTool::Ellipse:
        case LaserTool::Polygon: {
            const bool   ell = m_tool == LaserTool::Ellipse;
            const Vec2d  c   = ell ? (a + b) / 2 : a;
            const double rx  = ell ? std::abs(b.x() - a.x()) / 2 : (b - a).norm();
            const double ry  = ell ? std::abs(b.y() - a.y()) / 2 : rx;
            const int    n   = ell ? 72 : std::max(3, polygon_sides);
            for (int k = 0; k < n; ++k) {
                const double t0 = kPi / 2 + 2 * kPi * k / n, t1 = kPi / 2 + 2 * kPi * (k + 1) / n;
                pv.seg(c + Vec2d(rx * std::cos(t0), ry * std::sin(t0)), c + Vec2d(rx * std::cos(t1), ry * std::sin(t1)));
            }
            break;
        }
        default: break;
        }
        pv.draw(flat);
    }
    // Laser head.
    Vec2d head;
    if (m_panel.head_position(head)) {
        Batch h(ColorRGBA(0.9f, 0.1f, 0.1f, 1.f));
        h.seg(head - Vec2d(8 * px, 0), head + Vec2d(8 * px, 0));
        h.seg(head - Vec2d(0, 8 * px), head + Vec2d(0, 8 * px));
        for (int k = 0; k < 24; ++k)
            h.seg(head + 5 * px * Vec2d(std::cos(k * kPi / 12), std::sin(k * kPi / 12)),
                  head + 5 * px * Vec2d(std::cos((k + 1) * kPi / 12), std::sin((k + 1) * kPi / 12)));
        h.draw(flat);
    }
    flat->stop_using();
    m_gl->SwapBuffers();
}

void LaserCanvas::paint_ruler(wxWindow* w, bool horizontal)
{
    wxAutoBufferedPaintDC dc(w);
    const wxSize sz = w->GetClientSize();
    dc.SetBackground(wxBrush(wxColour(236, 236, 236)));
    dc.Clear();
    dc.SetPen(wxPen(wxColour(120, 120, 120)));
    dc.SetFont(wxFont(7, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL));
    dc.SetTextForeground(wxColour(70, 70, 70));
    // Minor ticks >= 5 px apart, labels >= 50 px apart.
    double minor = 1;
    for (double s : {1., 2., 5., 10., 20., 50., 100., 200.})
        if ((minor = s) * m_scale >= 5) break;
    double label = minor;
    for (double s : {1., 2., 5., 10., 20., 50., 100., 200., 500., 1000.})
        if ((label = s) * m_scale >= 50) break;
    const int    len   = horizontal ? sz.x : sz.y;
    const Vec2d  w0    = to_world(wxPoint(0, len)), w1 = to_world(wxPoint(len, 0));
    const double from  = horizontal ? w0.x() : w0.y();
    const double to    = horizontal ? w1.x() : w1.y();
    for (double v = std::floor(std::min(from, to) / minor) * minor; v <= std::max(from, to); v += minor) {
        const Vec2d  s     = to_screen(Vec2d(v, v));
        const int    pos   = int(std::lround(horizontal ? s.x() : s.y()));
        const bool   major = std::abs(std::remainder(v, label)) < 1e-6;
        const int    tick  = major ? kRulerPx - 2 : kRulerPx / 4;
        if (horizontal) dc.DrawLine(pos, sz.y - tick, pos, sz.y);
        else dc.DrawLine(sz.x - tick, pos, sz.x, pos);
        if (major) {
            const wxString t = wxString::Format("%g", std::abs(v) < 1e-9 ? 0. : v);
            if (horizontal) dc.DrawText(t, pos + 2, 0);
            else dc.DrawRotatedText(t, 0, pos - 2, 90);
        }
    }
}

// ---- Hit tests -------------------------------------------------------------------------------------

int LaserCanvas::hit_shape(const Vec2d& p) const
{
    const LaserDocument& doc = m_panel.doc();
    const double         tol = 5. / m_scale;
    for (int i = int(doc.shapes.size()) - 1; i >= 0; --i) {
        const LaserShape& s = doc.shapes[i];
        if (s.type == ShapeType::Group || !s.visible || i >= int(m_bbox.size()) || !m_bbox[i].defined) continue;
        if (!doc.layers[std::clamp(s.layer, 0, kLayerCount - 1)].visible) continue;
        BoundingBoxf bb = m_bbox[i];
        bb.offset(tol);
        if (!bb.contains(p)) continue;
        if (s.type == ShapeType::Text || s.type == ShapeType::Image || inside(m_flat[i], p)) return i;
        for (const LaserPath& lp : m_flat[i]) {
            const Points& pts = lp.pts.points;
            for (size_t k = 1; k < pts.size(); ++k)
                if (seg_dist(p, unscaled(pts[k - 1]), unscaled(pts[k])) < tol) return i;
            if (lp.closed && pts.size() > 2 && seg_dist(p, unscaled(pts.back()), unscaled(pts.front())) < tol) return i;
        }
    }
    return -1;
}

bool LaserCanvas::selection_box(BoundingBoxf& box) const
{
    box = BoundingBoxf();
    const LaserDocument& doc = m_panel.doc();
    std::vector<int>     stack(m_panel.selection().begin(), m_panel.selection().end());
    while (!stack.empty()) {
        const int i = stack.back();
        stack.pop_back();
        if (i < 0 || i >= int(m_bbox.size())) continue;
        if (m_bbox[i].defined) box.merge(m_bbox[i]);
        for (int c : doc.children(i)) stack.push_back(c);
    }
    return box.defined;
}

int LaserCanvas::hit_handle(const wxPoint& px) const
{
    BoundingBoxf box;
    if (m_tool != LaserTool::Select || !selection_box(box)) return -1;
    const Vec2d a = to_screen(box.min), b = to_screen(box.max), c = (a + b) / 2;
    const Vec2d pos[9] = {a, {c.x(), a.y()}, {b.x(), a.y()}, {b.x(), c.y()}, b, {c.x(), b.y()}, {a.x(), b.y()},
                          {a.x(), c.y()}, {c.x(), b.y() - kRotatePx}};
    for (int k = 8; k >= 0; --k)
        if (std::abs(pos[k].x() - px.x) <= kHandlePx + 2 && std::abs(pos[k].y() - px.y) <= kHandlePx + 2) return k;
    return -1;
}

int LaserCanvas::node_shape() const
{
    const auto& sel = m_panel.selection();
    if (sel.size() != 1) return -1;
    const LaserShape& s = m_panel.doc().shapes[sel.front()];
    return s.type == ShapeType::Path && !s.locked ? sel.front() : -1;
}

bool LaserCanvas::node_hit(const wxPoint& px, int& path, int& pt) const
{
    const int ns = node_shape();
    if (ns < 0) return false;
    const LaserShape& s = m_panel.doc().shapes[ns];
    for (size_t pi = 0; pi < s.paths.size(); ++pi)
        for (size_t k = 0; k < s.paths[pi].pts.points.size(); ++k) {
            const Vec2d q = to_screen(s.xform * unscaled(s.paths[pi].pts.points[k]));
            if (std::abs(q.x() - px.x) <= 6 && std::abs(q.y() - px.y) <= 6) {
                path = int(pi);
                pt   = int(k);
                return true;
            }
        }
    return false;
}

// ---- Interaction -----------------------------------------------------------------------------------

void LaserCanvas::begin_transform(Drag kind)
{
    m_drag = kind;
    m_orig_xf.clear();
    for (const LaserShape& s : m_panel.doc().shapes) m_orig_xf.push_back(s.xform);
    selection_box(m_box0);
}

void LaserCanvas::update_transform(const Vec2d& p, bool shift)
{
    LaserDocument& doc = m_panel.doc();
    for (size_t i = 0; i < doc.shapes.size() && i < m_orig_xf.size(); ++i) doc.shapes[i].xform = m_orig_xf[i];
    Transform2d t = Transform2d::Identity();
    if (m_drag == Drag::Move) {
        Vec2d d = p - m_down;
        if (shift) (std::abs(d.x()) > std::abs(d.y()) ? d.y() : d.x()) = 0;
        if (snap) {
            // Snap the moved box's edges/centre to the grid, the bed edges and other shapes.
            const double tol = 6. / m_scale;
            const LaserDevice& dev = m_panel.device();
            for (int axis = 0; axis < 2; ++axis) {
                if (shift && d[axis] == 0) continue;
                std::vector<double> targets{0., axis == 0 ? dev.bed_w : dev.bed_h};
                for (int i = 0; i < int(m_bbox.size()); ++i) {
                    if (!m_bbox[i].defined || std::find(m_panel.selection().begin(), m_panel.selection().end(), doc.root_of(i)) != m_panel.selection().end())
                        continue;
                    targets.insert(targets.end(), {m_bbox[i].min[axis], m_bbox[i].max[axis], m_bbox[i].center()[axis]});
                }
                const double lo = m_box0.min[axis] + d[axis], hi = m_box0.max[axis] + d[axis], mid = (lo + hi) / 2;
                double       best = tol, adj = 0;
                for (double tv : targets)
                    for (double v : {lo, hi, mid})
                        if (std::abs(tv - v) < best) { best = std::abs(tv - v); adj = tv - v; }
                if (best < tol) d[axis] += adj;
                else {
                    const double g = grid_step();
                    d[axis] = std::round(lo / g) * g - m_box0.min[axis];
                }
            }
        }
        t.translate(d);
    } else if (m_drag == Drag::Scale) {
        const Vec2d a = m_box0.min, b = m_box0.max, c = m_box0.center();
        const Vec2d hp[8] = {a, {c.x(), a.y()}, {b.x(), a.y()}, {b.x(), c.y()}, b, {c.x(), b.y()}, {a.x(), b.y()}, {a.x(), c.y()}};
        const Vec2d anchor = hp[(m_handle + 4) % 8], from = hp[m_handle];
        double      sx = 1, sy = 1;
        const bool  corner = m_handle % 2 == 0;
        if (corner || m_handle == 3 || m_handle == 7)
            if (std::abs(from.x() - anchor.x()) > 1e-9) sx = (p.x() - anchor.x()) / (from.x() - anchor.x());
        if (corner || m_handle == 1 || m_handle == 5)
            if (std::abs(from.y() - anchor.y()) > 1e-9) sy = (p.y() - anchor.y()) / (from.y() - anchor.y());
        if (corner && shift) sx = sy = (std::abs(sx) > std::abs(sy) ? sx : sy);
        auto clamp_s = [](double s) { return std::abs(s) < 1e-3 ? (s < 0 ? -1e-3 : 1e-3) : s; };
        t.translate(anchor);
        t.scale(Vec2d(clamp_s(sx), clamp_s(sy)));
        t.translate(-anchor);
    } else if (m_drag == Drag::Rotate) {
        const Vec2d c = m_box0.center();
        double      ang = std::atan2(p.y() - c.y(), p.x() - c.x()) - std::atan2(m_down.y() - c.y(), m_down.x() - c.x());
        if (shift) ang = std::round(ang / (kPi / 12)) * (kPi / 12);
        t.translate(c);
        t.rotate(ang);
        t.translate(-c);
        m_panel.set_status(wxString::Format(_L("Rotate %.1f°"), ang * 180 / kPi));
    }
    for (int i : m_panel.selection())
        if (!doc.shapes[i].locked) doc.transform(i, t);
    m_panel.geometry_changed();
}

void LaserCanvas::cancel_drawing()
{
    m_pts.clear();
    if (m_drag == Drag::Draw) m_drag = Drag::None;
    refresh();
}

void LaserCanvas::finish_drawing(bool closed)
{
    if (m_tool != LaserTool::Line || m_pts.size() < 2) { cancel_drawing(); return; }
    LaserShape s;
    s.type  = ShapeType::Path;
    s.layer = m_panel.current_layer();
    LaserPath lp;
    lp.closed = closed && m_pts.size() >= 3;
    for (const Vec2d& p : m_pts) lp.pts.points.push_back(Point::new_scale(p.x(), p.y()));
    s.paths.push_back(lp);
    m_pts.clear();
    m_drag = Drag::None;
    m_panel.add_shape_and_select(std::move(s), lp.closed ? "Draw polygon" : "Draw line");
    m_panel.tool_finished();
}

void LaserCanvas::on_mouse(wxMouseEvent& evt)
{
    const wxPoint px = evt.GetPosition();
    const Vec2d   p  = to_world(px);
    LaserDocument& doc = m_panel.doc();
    m_cursor           = (m_tool == LaserTool::Select || m_tool == LaserTool::Node) ? p : snap_point(p);

    if (evt.GetEventType() == wxEVT_MOUSEWHEEL) {
        const double f = evt.GetWheelRotation() > 0 ? 1.2 : 1 / 1.2;
        m_scale        = std::clamp(m_scale * f, 0.05, 400.);
        m_off          = Vec2d(px.x - p.x() * m_scale, px.y + p.y() * m_scale);
        m_dirty        = true;   // outline tolerance follows the zoom
        m_fitted       = false;
        refresh();
        return;
    }
    if (evt.Leaving()) {
        if (m_hover >= 0) { m_hover = -1; refresh(); }
        return;
    }
    if (evt.MiddleDown() || evt.RightDown()) {
        m_drag    = Drag::Pan;
        m_down_px = px;
        m_moved   = false;
        if (!m_gl->HasCapture()) m_gl->CaptureMouse();
        return;
    }
    if ((evt.MiddleUp() || evt.RightUp()) && m_drag == Drag::Pan) {
        m_drag = Drag::None;
        if (m_gl->HasCapture()) m_gl->ReleaseMouse();
        if (evt.RightUp() && !m_moved) context_menu(px);
        return;
    }
    if (evt.Dragging() && m_drag == Drag::Pan) {
        m_off += Vec2d(px.x - m_down_px.x, px.y - m_down_px.y);
        m_fitted = false;
        m_moved |= std::abs(px.x - m_down_px.x) + std::abs(px.y - m_down_px.y) > 0;
        m_down_px = px;
        refresh();
        return;
    }

    const bool shift = evt.ShiftDown(), ctrl = evt.ControlDown() || evt.CmdDown();
    m_panel.set_status(wxString::Format("X %.2f  Y %.2f mm", m_cursor.x(), m_cursor.y()));

    if (evt.LeftDClick()) {
        if (m_tool == LaserTool::Line) { finish_drawing(false); return; }
        if (m_tool == LaserTool::Select) {
            const int i = hit_shape(p);
            if (i >= 0 && doc.shapes[i].type == ShapeType::Text) m_panel.edit_text(i);
            return;
        }
        if (m_tool == LaserTool::Node) {
            // Insert a vertex on the edge under the cursor.
            const int ns = node_shape();
            if (ns < 0) return;
            LaserShape&  s   = doc.shapes[ns];
            const Vec2d  loc = s.xform.inverse() * p;
            const double tol = 6. / m_scale / std::max(1e-9, std::sqrt(std::abs(s.xform.linear().determinant())));
            for (LaserPath& lp : s.paths) {
                Points&      pts = lp.pts.points;
                const size_t n   = pts.size();
                for (size_t k = 0; k < n; ++k) {
                    if (k + 1 == n && !lp.closed) break;
                    if (seg_dist(loc, unscaled(pts[k]), unscaled(pts[(k + 1) % n])) < tol) {
                        pts.insert(pts.begin() + k + 1, Point::new_scale(loc.x(), loc.y()));
                        m_panel.commit("Insert node");
                        return;
                    }
                }
            }
        }
        return;
    }

    if (evt.LeftDown()) {
        m_gl->SetFocus();
        m_down_px = px;
        m_down    = m_cursor;
        m_moved   = false;
        if (!m_gl->HasCapture()) m_gl->CaptureMouse();
        switch (m_tool) {
        case LaserTool::Select: {
            if (int h = hit_handle(px); h >= 0) {
                m_handle = h;
                begin_transform(h == 8 ? Drag::Rotate : Drag::Scale);
                return;
            }
            const int i = hit_shape(p);
            if (i >= 0) {
                const int        root = doc.root_of(i);
                std::vector<int> sel  = m_panel.selection();
                auto             it   = std::find(sel.begin(), sel.end(), root);
                if (shift || ctrl) {
                    if (it != sel.end()) { sel.erase(it); m_panel.set_selection(sel); return; }
                    sel.push_back(root);
                } else if (it == sel.end())
                    sel = {root};
                m_panel.set_selection(sel);
                begin_transform(Drag::Move);
            } else {
                if (!shift && !ctrl) m_panel.set_selection({});
                m_drag = Drag::Rubber;
            }
            return;
        }
        case LaserTool::Node: {
            if (node_hit(px, m_node_path, m_node_pt)) {
                begin_transform(Drag::Node);
                refresh();
                return;
            }
            m_node_path = m_node_pt = -1;
            const int i = hit_shape(p);
            m_panel.set_selection(i >= 0 ? std::vector<int>{doc.root_of(i)} : std::vector<int>{});
            return;
        }
        case LaserTool::Line:
            if (m_pts.size() >= 3 && (to_screen(m_pts.front()) - Vec2d(px.x, px.y)).norm() < 8) { finish_drawing(true); return; }
            m_pts.push_back(m_cursor);
            m_drag = Drag::Draw;
            refresh();
            return;
        case LaserTool::Rect:
        case LaserTool::Ellipse:
        case LaserTool::Polygon:
            m_pts  = {m_cursor};
            m_drag = Drag::Draw;
            return;
        case LaserTool::Text: {
            LaserShape s;
            s.type  = ShapeType::Text;
            s.layer = m_panel.current_layer();
            s.xform = Transform2d::Identity();
            s.xform.translate(m_cursor);
            if (m_gl->HasCapture()) m_gl->ReleaseMouse();
            m_panel.add_shape_and_select(std::move(s), "");
            m_panel.edit_text(int(doc.shapes.size()) - 1);
            m_panel.tool_finished();
            return;
        }
        }
    }

    if (evt.Dragging() && evt.LeftIsDown()) {
        m_moved |= std::abs(px.x - m_down_px.x) + std::abs(px.y - m_down_px.y) > 2;
        if (!m_moved) return;
        switch (m_drag) {
        case Drag::Move:
        case Drag::Scale:
        case Drag::Rotate: update_transform(p, shift); break;
        case Drag::Node: {
            LaserShape& s = doc.shapes[node_shape()];
            const Vec2d loc = s.xform.inverse() * (snap ? snap_point(p) : p);
            s.paths[m_node_path].pts.points[m_node_pt] = Point::new_scale(loc.x(), loc.y());
            m_panel.geometry_changed();
            break;
        }
        case Drag::Rubber:
        case Drag::Draw:
            if (m_tool == LaserTool::Rect || m_tool == LaserTool::Ellipse) {
                if (shift && !m_pts.empty()) {   // square / circle
                    const Vec2d d = m_cursor - m_pts.front();
                    const double s = std::max(std::abs(d.x()), std::abs(d.y()));
                    m_cursor = m_pts.front() + Vec2d(d.x() < 0 ? -s : s, d.y() < 0 ? -s : s);
                }
            }
            refresh();
            break;
        default: break;
        }
        return;
    }

    if (evt.LeftUp()) {
        if (m_gl->HasCapture()) m_gl->ReleaseMouse();
        const Drag drag = m_drag;
        if (drag != Drag::Draw || m_tool != LaserTool::Line) m_drag = Drag::None;
        switch (drag) {
        case Drag::Move:
        case Drag::Scale:
        case Drag::Rotate:
        case Drag::Node:
            if (m_moved) m_panel.commit(drag == Drag::Move ? "Move" : drag == Drag::Scale ? "Scale" : drag == Drag::Rotate ? "Rotate" : "Edit nodes");
            break;
        case Drag::Rubber: {
            if (!m_moved) break;
            BoundingBoxf rb;
            rb.merge(m_down);
            rb.merge(p);
            const bool       crossing = p.x() < m_down.x();   // right-to-left selects what it touches
            std::vector<int> sel      = (shift || ctrl) ? m_panel.selection() : std::vector<int>{};
            for (int i = 0; i < int(m_bbox.size()); ++i) {
                if (!m_bbox[i].defined) continue;
                const BoundingBoxf& b   = m_bbox[i];
                const bool          hit = crossing ? (b.min.x() <= rb.max.x() && b.max.x() >= rb.min.x() && b.min.y() <= rb.max.y() && b.max.y() >= rb.min.y())
                                                   : (rb.contains(b.min) && rb.contains(b.max));
                const int root = doc.root_of(i);
                if (hit && std::find(sel.begin(), sel.end(), root) == sel.end()) sel.push_back(root);
            }
            m_panel.set_selection(sel);
            break;
        }
        case Drag::Draw: {
            if (m_tool == LaserTool::Line || m_pts.empty()) break;
            const Vec2d a = m_pts.front(), b = m_cursor;
            m_pts.clear();
            if ((b - a).norm() < 0.5) { refresh(); break; }
            LaserShape s;
            s.layer = m_panel.current_layer();
            s.xform = Transform2d::Identity();
            if (m_tool == LaserTool::Rect) {
                s.type   = ShapeType::Rect;
                s.width  = std::max(0.1, std::abs(b.x() - a.x()));
                s.height = std::max(0.1, std::abs(b.y() - a.y()));
                s.xform.translate((a + b) / 2);
            } else if (m_tool == LaserTool::Ellipse) {
                s.type = ShapeType::Ellipse;
                s.rx   = std::max(0.05, std::abs(b.x() - a.x()) / 2);
                s.ry   = std::max(0.05, std::abs(b.y() - a.y()) / 2);
                s.xform.translate((a + b) / 2);
            } else {
                s.type  = ShapeType::Polygon;
                s.rx = s.ry = (b - a).norm();
                s.sides = std::max(3, polygon_sides);
                s.xform.translate(a);
            }
            m_panel.add_shape_and_select(std::move(s), m_tool == LaserTool::Rect ? "Draw rectangle" : m_tool == LaserTool::Ellipse ? "Draw ellipse" : "Draw polygon");
            m_panel.tool_finished();
            break;
        }
        default: break;
        }
        refresh();
        return;
    }

    if (evt.Moving()) {
        int hover = -1;
        if (m_tool == LaserTool::Select || m_tool == LaserTool::Node) {
            const int i = hit_shape(p);
            hover       = i >= 0 ? doc.root_of(i) : -1;
            const int h = hit_handle(px);
            m_gl->SetCursor(h == 8 ? wxCursor(wxCURSOR_HAND) : h >= 0 ? wxCursor(h % 4 == 1 ? wxCURSOR_SIZENS : h % 4 == 3 ? wxCURSOR_SIZEWE : h % 4 == 0 ? wxCURSOR_SIZENESW : wxCURSOR_SIZENWSE)
                                          : hover >= 0 ? wxCursor(wxCURSOR_SIZING) : wxNullCursor);
        }
        if (hover != m_hover || !m_pts.empty()) {
            m_hover = hover;
            refresh();
        }
    }
}

void LaserCanvas::on_key(wxKeyEvent& evt)
{
    const int  k    = evt.GetKeyCode();
    const bool ctrl = evt.ControlDown() || evt.CmdDown();
    if (ctrl) {
        switch (k) {
        case 'Z': evt.ShiftDown() ? m_panel.redo() : m_panel.undo(); return;
        case 'Y': m_panel.redo(); return;
        case 'C': m_panel.cmd_copy(false); return;
        case 'X': m_panel.cmd_copy(true); return;
        case 'V': m_panel.cmd_paste(); return;
        case 'D': m_panel.cmd_duplicate(); return;
        case 'G': evt.ShiftDown() ? m_panel.cmd_ungroup() : m_panel.cmd_group(); return;
        case 'U': m_panel.cmd_ungroup(); return;
        case 'A': m_panel.cmd_select_all(); return;
        default: break;
        }
    }
    const double step = evt.ShiftDown() ? 10. : 1.;
    switch (k) {
    case WXK_ESCAPE:
        if (!m_pts.empty()) cancel_drawing();
        else if (m_tool != LaserTool::Select) m_panel.tool_finished();
        else m_panel.set_selection({});
        return;
    case WXK_RETURN:
    case WXK_NUMPAD_ENTER: finish_drawing(false); return;
    case WXK_BACK:
        if (m_tool == LaserTool::Line && !m_pts.empty()) { m_pts.pop_back(); refresh(); return; }
        [[fallthrough]];
    case WXK_DELETE:
        if (m_tool == LaserTool::Node && m_node_pt >= 0 && node_shape() >= 0) {
            LaserShape& s  = m_panel.doc().shapes[node_shape()];
            Points&     pts = s.paths[m_node_path].pts.points;
            if (pts.size() > (s.paths[m_node_path].closed ? 3u : 2u)) {
                pts.erase(pts.begin() + m_node_pt);
                m_node_pt = -1;
                m_panel.commit("Delete node");
            }
            return;
        }
        m_panel.cmd_delete();
        return;
    case WXK_LEFT: m_panel.nudge(-step, 0); return;
    case WXK_RIGHT: m_panel.nudge(step, 0); return;
    case WXK_UP: m_panel.nudge(0, step); return;
    case WXK_DOWN: m_panel.nudge(0, -step); return;
    default: evt.Skip();
    }
}

void LaserCanvas::context_menu(const wxPoint& px)
{
    const Vec2d p = to_world(px);
    if (m_panel.selection().empty()) {
        const int i = hit_shape(p);
        if (i >= 0) m_panel.set_selection({m_panel.doc().root_of(i)});
    }
    const bool has = !m_panel.selection().empty();
    wxMenu     menu;
    auto       item = [&](const wxString& label, bool enable, std::function<void()> fn) {
        wxMenuItem* it = menu.Append(wxID_ANY, label);
        it->Enable(enable);
        menu.Bind(wxEVT_MENU, [fn](wxCommandEvent&) { fn(); }, it->GetId());
    };
    item(_L("Cut") + "\tCtrl+X", has, [this] { m_panel.cmd_copy(true); });
    item(_L("Copy") + "\tCtrl+C", has, [this] { m_panel.cmd_copy(false); });
    item(_L("Paste") + "\tCtrl+V", true, [this] { m_panel.cmd_paste(); });
    item(_L("Duplicate") + "\tCtrl+D", has, [this] { m_panel.cmd_duplicate(); });
    item(_L("Delete") + "\tDel", has, [this] { m_panel.cmd_delete(); });
    menu.AppendSeparator();
    item(_L("Group") + "\tCtrl+G", m_panel.selection().size() > 1, [this] { m_panel.cmd_group(); });
    item(_L("Ungroup") + "\tCtrl+U", has, [this] { m_panel.cmd_ungroup(); });
    item(_L("Lock"), has, [this] { m_panel.cmd_lock(true); });
    item(_L("Unlock"), has, [this] { m_panel.cmd_lock(false); });
    item(_L("Convert to Path"), has, [this] { m_panel.cmd_to_path(); });
    auto* layers = new wxMenu;
    for (int l = 0; l < kLayerCount; ++l) {
        const std::string& name = m_panel.doc().layers[l].name;
        wxMenuItem* it = layers->Append(wxID_ANY, wxString::Format("C%02d  %s", l, wxString::FromUTF8(name)));
        layers->Bind(wxEVT_MENU, [this, l](wxCommandEvent&) { m_panel.cmd_to_layer(l); }, it->GetId());
    }
    menu.AppendSubMenu(layers, _L("Move to layer"))->Enable(has);
    m_gl->PopupMenu(&menu, px);
}

}} // namespace Slic3r::GUI

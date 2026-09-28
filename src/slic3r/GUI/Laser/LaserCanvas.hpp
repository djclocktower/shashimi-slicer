#pragma once

// The Laser workspace: the bed seen from above (X right, Y up, origin front-left), drawn with its
// own orthographic 2D OpenGL renderer on a wxGLCanvas sharing the app's context (not GLCanvas3D),
// with wx-drawn rulers along the top and left edges. Owns the interactive tools; every edit goes
// through LaserPanel (commit / geometry_changed).

#include <wx/panel.h>

#include "libslic3r/Laser/LaserTypes.hpp"

#include <map>
#include <memory>
#include <vector>

class wxGLCanvas;
class wxGLContext;

namespace Slic3r { namespace GUI {

class LaserPanel;

enum class LaserTool { Select, Node, Line, Rect, Ellipse, Polygon, Text };

class LaserCanvas : public wxPanel
{
public:
    LaserCanvas(wxWindow* parent, LaserPanel& panel);
    ~LaserCanvas() override;

    void      set_tool(LaserTool tool);
    LaserTool tool() const { return m_tool; }
    // The document changed: rebuild the cached outlines / textures and repaint.
    void document_changed();
    void refresh();
    void zoom_fit();
    void zoom_by(double factor);

    bool   snap{true};          // snap to grid and to other shapes' edges
    int    polygon_sides{6};
    // Paths of the shape (workspace mm, scaled), cached from LaserDocument::flatten.
    const Laser::LaserPaths& outline(int index) const;

private:
    struct Batch;
    enum class Drag { None, Pan, Move, Scale, Rotate, Rubber, Draw, Node };

    Vec2d to_world(const wxPoint& px) const;
    Vec2d to_screen(const Vec2d& mm) const;
    double grid_step() const;
    Vec2d snap_point(const Vec2d& p) const;
    void  rebuild_cache();
    void  render();
    void  paint_ruler(wxWindow* w, bool horizontal);
    // Hit tests (-1 = none).
    int   hit_shape(const Vec2d& p) const;
    int   hit_handle(const wxPoint& px) const;   // 0..7 scale handles, 8 rotate
    bool  selection_box(BoundingBoxf& box) const;
    bool  node_hit(const wxPoint& px, int& path, int& pt) const;
    int   node_shape() const;   // selected Path shape for Node Edit, -1 none

    void on_mouse(wxMouseEvent& evt);
    void on_key(wxKeyEvent& evt);
    void begin_transform(Drag kind);
    void update_transform(const Vec2d& p, bool shift);
    void finish_drawing(bool closed);
    void cancel_drawing();
    void context_menu(const wxPoint& px);

    LaserPanel&  m_panel;
    wxGLCanvas*  m_gl{nullptr};
    wxGLContext* m_ctx{nullptr};
    wxWindow*    m_ruler_top{nullptr};
    wxWindow*    m_ruler_left{nullptr};
    LaserTool    m_tool{LaserTool::Select};

    double m_scale{2.0};          // px per mm
    Vec2d  m_off{40, 400};        // screen px of the workspace origin
    bool   m_fitted{true};         // the view follows the bed (fit) until the user zooms or pans

    // Cache
    bool                                  m_dirty{true};
    std::vector<Laser::LaserPaths>        m_flat;   // per shape (groups empty)
    std::vector<BoundingBoxf>             m_bbox;
    std::vector<std::unique_ptr<Batch>>   m_layer_lines;   // per layer
    std::vector<std::unique_ptr<Batch>>   m_layer_hatch;
    struct Tex;
    std::map<uint64_t, std::unique_ptr<Tex>> m_textures;

    // Interaction
    Drag                     m_drag{Drag::None};
    wxPoint                  m_down_px;
    Vec2d                    m_down{0, 0}, m_cursor{0, 0};
    bool                     m_moved{false};
    std::vector<Transform2d> m_orig_xf;
    BoundingBoxf             m_box0;
    int                      m_handle{-1};
    int                      m_hover{-1};
    std::vector<Vec2d>       m_pts;       // drawing tool points
    int                      m_node_path{-1}, m_node_pt{-1};
};

}} // namespace Slic3r::GUI

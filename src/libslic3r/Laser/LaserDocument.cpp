#include "libslic3r/Laser/LaserDocument.hpp"

#include <cereal/archives/binary.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <set>
#include <sstream>

namespace Slic3r::Laser {

std::array<LaserLayer, kLayerCount> default_layers()
{
    std::array<LaserLayer, kLayerCount> layers;
    for (int i = 0; i < kLayerCount; ++i) {
        char name[8];
        std::snprintf(name, sizeof(name), "C%02d", i);
        layers[i].name = name;
    }
    return layers;
}

// ---- Framed item blocks ------------------------------------------------------------------------

static constexpr uint32_t kItemBlockVersion = 1;

template<class T> std::string laser_encode_items(const std::vector<T>& items)
{
    std::ostringstream os;
    {
        cereal::BinaryOutputArchive ar(os);
        const uint32_t version = kItemBlockVersion, count = uint32_t(items.size());
        ar(version, count);
        for (const T& it : items) {
            std::ostringstream is;
            {
                cereal::BinaryOutputArchive ia(is);
                ia(const_cast<T&>(it));
            }
            const std::string item = is.str();
            const uint32_t    len  = uint32_t(item.size());
            ar(len);
            ar(cereal::binary_data(item.data(), item.size()));
        }
    }
    return os.str();
}

template<class T> bool laser_decode_items(const std::string& block, std::vector<T>& items)
{
    items.clear();
    if (block.empty())
        return true;
    try {
        std::istringstream         in(block);
        cereal::BinaryInputArchive ar(in);
        uint32_t                   version = 0, count = 0;
        ar(version, count);
        // A newer block version only ever appends item fields: read on regardless.
        if (count > block.size() / sizeof(uint32_t))
            return false;
        for (uint32_t i = 0; i < count; ++i) {
            uint32_t len = 0;
            ar(len);
            if (len > block.size())
                return false;
            std::string item(len, '\0');
            if (len > 0)
                ar(cereal::binary_data(&item[0], len));
            T t;
            try {
                std::istringstream         is(item);
                cereal::BinaryInputArchive ia(is);
                ia(t);   // stops at the fields this build knows; trailing bytes are skipped
            } catch (...) {
                // An item from an older build ends early: what was read is kept, the rest defaults.
            }
            items.push_back(std::move(t));
        }
    } catch (...) {
        return false;
    }
    return true;
}

namespace {
// The settings block's single item: device_name, then the JobSettings fields (flat, append-only).
struct SettingsRecord {
    std::string device_name;
    JobSettings job;
    template<class Archive> void serialize(Archive& ar)
    {
        ar(device_name);
        job.serialize(ar);
    }
};
} // namespace

template std::string laser_encode_items<LaserShape>(const std::vector<LaserShape>&);
template std::string laser_encode_items<LaserLayer>(const std::vector<LaserLayer>&);
template bool        laser_decode_items<LaserShape>(const std::string&, std::vector<LaserShape>&);
template bool        laser_decode_items<LaserLayer>(const std::string&, std::vector<LaserLayer>&);

// ---- Serialization -----------------------------------------------------------------------------

std::string LaserDocument::serialize() const
{
    const std::vector<LaserLayer> layer_vec(layers.begin(), layers.end());
    const std::string blocks[3] = {laser_encode_items(shapes), laser_encode_items(layer_vec),
                                   laser_encode_items(std::vector<SettingsRecord>{{device_name, job}})};
    std::ostringstream os;
    {
        cereal::BinaryOutputArchive ar(os);
        const uint32_t version = kVersion;
        ar(version);
        for (const std::string& block : blocks) {
            const uint32_t len = uint32_t(block.size());
            ar(len);
            ar(cereal::binary_data(block.data(), block.size()));
        }
    }
    return os.str();
}

bool LaserDocument::deserialize(const std::string& blob)
{
    clear();
    if (blob.empty())
        return true;
    std::string blocks[3];
    try {
        std::istringstream         in(blob);
        cereal::BinaryInputArchive ar(in);
        uint32_t                   version = 0;
        ar(version);
        for (std::string& block : blocks) {
            uint32_t len = 0;
            ar(len);
            if (len > blob.size())
                return false;
            block.resize(len);
            if (len > 0)
                ar(cereal::binary_data(&block[0], len));
        }
        // Trailing blocks from a newer build are ignored.
    } catch (...) {
        return false;
    }
    std::vector<LaserLayer>     layer_vec;
    std::vector<SettingsRecord> settings;
    if (!laser_decode_items(blocks[0], shapes) || !laser_decode_items(blocks[1], layer_vec) ||
        !laser_decode_items(blocks[2], settings)) {
        clear();
        return false;
    }
    for (size_t i = 0; i < layer_vec.size() && i < layers.size(); ++i)
        layers[i] = std::move(layer_vec[i]);
    if (!settings.empty()) {
        device_name = std::move(settings.front().device_name);
        job         = settings.front().job;
    }

    // Repairs: layer range, unique non-zero ids, parents that exist, are groups and form no cycle.
    const int n = int(shapes.size());
    std::set<uint64_t> seen;
    for (LaserShape& s : shapes) m_next_id = std::max(m_next_id, s.id + 1);
    for (LaserShape& s : shapes) {
        if (s.id == 0 || !seen.insert(s.id).second)
            s.id = m_next_id++;
        if (s.layer < 0 || s.layer >= kLayerCount) {
            warnings.push_back("Shape \"" + s.name + "\" used a layer that does not exist; it was moved to C00.");
            s.layer = 0;
        }
        // Every reader indexes gray[] by image_w x image_h: a mismatch is dropped here, once.
        if (s.type == ShapeType::Image && (s.image_w <= 0 || s.image_h <= 0 || s.gray.size() != size_t(s.image_w) * size_t(s.image_h))) {
            if (!s.gray.empty() || s.image_w != 0 || s.image_h != 0)
                warnings.push_back("Image \"" + s.name + "\" was damaged; its pixels were removed.");
            s.image_w = s.image_h = 0;
            s.gray.clear();
        }
    }
    for (int i = 0; i < n; ++i) {
        int& p = shapes[i].parent;
        if (p == -1) continue;
        bool bad = p < 0 || p >= n || p == i || shapes[p].type != ShapeType::Group;
        // Cycle check: walk up at most n steps.
        for (int a = bad ? -1 : p, steps = 0; a != -1 && !bad; a = shapes[a].parent, ++steps)
            if (a < -1 || a >= n || a == i || steps > n) bad = true;
        if (bad) {
            warnings.push_back("Shape \"" + shapes[i].name + "\" belonged to a group that does not exist; it was moved to the top level.");
            p = -1;
        }
    }
    return true;
}

void LaserDocument::clear()
{
    shapes.clear();
    layers = default_layers();
    device_name.clear();
    job = JobSettings{};
    warnings.clear();
    m_next_id = 1;
}

// ---- Editing -----------------------------------------------------------------------------------

int LaserDocument::add_shape(LaserShape shape)
{
    shape.id = m_next_id++;
    if (shape.parent < -1 || shape.parent >= int(shapes.size()) ||
        (shape.parent >= 0 && shapes[shape.parent].type != ShapeType::Group))
        shape.parent = -1;
    shapes.push_back(std::move(shape));
    return int(shapes.size()) - 1;
}

std::vector<int> LaserDocument::add_shapes(std::vector<LaserShape> batch)
{
    const int base = int(shapes.size()), n = int(batch.size());
    std::vector<int> out;
    out.reserve(n);
    for (LaserShape& s : batch) {
        s.id = m_next_id++;
        s.parent = (s.parent >= 0 && s.parent < n && batch[s.parent].type == ShapeType::Group) ? base + s.parent : -1;
    }
    for (int i = 0; i < n; ++i) {
        shapes.push_back(std::move(batch[i]));
        out.push_back(base + i);
    }
    return out;
}

namespace {
// Keeps shapes with keep[i], remapping parent indices (a dropped parent -> the kept ancestor or -1).
std::vector<int> compact(std::vector<LaserShape>& shapes, const std::vector<bool>& keep)
{
    const int n = int(shapes.size());
    std::vector<int> map(n, -1);
    int k = 0;
    for (int i = 0; i < n; ++i)
        if (keep[i]) map[i] = k++;
    auto kept_ancestor = [&](int p) {
        while (p >= 0 && !keep[p]) p = shapes[p].parent;
        return p >= 0 ? map[p] : -1;
    };
    std::vector<int> new_parent(n, -1);
    for (int i = 0; i < n; ++i)
        if (keep[i]) new_parent[i] = kept_ancestor(shapes[i].parent);
    std::vector<LaserShape> out;
    out.reserve(k);
    for (int i = 0; i < n; ++i)
        if (keep[i]) {
            out.push_back(std::move(shapes[i]));
            out.back().parent = new_parent[i];
        }
    shapes.swap(out);
    return map;
}
} // namespace

void LaserDocument::remove_shapes(std::vector<int> indices)
{
    const int n = int(shapes.size());
    std::vector<bool> keep(n, true);
    for (int i : indices)
        if (i >= 0 && i < n) keep[i] = false;
    // Descendants of removed shapes go too.
    auto removed_ancestor = [&](int i) {
        for (int p = shapes[i].parent, steps = 0; p >= 0 && p < n && steps <= n; p = shapes[p].parent, ++steps)
            if (!keep[p]) return true;
        return false;
    };
    std::vector<bool> drop(n, false);
    for (int i = 0; i < n; ++i) drop[i] = !keep[i] || removed_ancestor(i);
    for (int i = 0; i < n; ++i) keep[i] = !drop[i];
    // Groups left without kept members go too (repeat: a group may only hold emptied groups).
    for (bool changed = true; changed;) {
        changed = false;
        std::vector<int> members(n, 0);
        for (int i = 0; i < n; ++i)
            if (keep[i] && shapes[i].parent >= 0) ++members[shapes[i].parent];
        for (int i = 0; i < n; ++i)
            if (keep[i] && shapes[i].type == ShapeType::Group && members[i] == 0) {
                keep[i] = false;
                changed = true;
            }
    }
    compact(shapes, keep);
}

std::vector<int> LaserDocument::duplicate(const std::vector<int>& indices)
{
    const int n = int(shapes.size());
    std::set<int> sel;
    for (int i : indices)
        if (i >= 0 && i < n) sel.insert(i);
    auto has_selected_ancestor = [&](int i) {
        for (int p = shapes[i].parent; p >= 0; p = shapes[p].parent)
            if (sel.count(p)) return true;
        return false;
    };
    std::vector<int> out;
    for (int i : indices) {
        if (i < 0 || i >= n || has_selected_ancestor(i)) continue;
        // Subtree of i in document order: i first, then descendants.
        std::vector<int> sub{i};
        for (int j = 0; j < n; ++j) {
            if (j == i) continue;
            for (int p = shapes[j].parent; p >= 0; p = shapes[p].parent)
                if (p == i) { sub.push_back(j); break; }
        }
        std::vector<int> map(n, -1);
        const int base = int(shapes.size());
        for (size_t k = 0; k < sub.size(); ++k) map[sub[k]] = base + int(k);
        for (int j : sub) {
            LaserShape c = shapes[j];
            c.id     = m_next_id++;
            c.parent = j == i ? shapes[i].parent : map[shapes[j].parent];
            shapes.push_back(std::move(c));
        }
        out.push_back(base);
    }
    return out;
}

int LaserDocument::group(const std::vector<int>& indices)
{
    std::set<int> sel;
    for (int i : indices)
        if (i >= 0 && i < int(shapes.size())) sel.insert(i);
    if (sel.size() < 2) return -1;
    const int parent = shapes[*sel.begin()].parent;
    for (int i : sel)
        if (shapes[i].parent != parent) return -1;
    LaserShape g;
    g.type   = ShapeType::Group;
    g.name   = "Group";
    g.parent = parent;
    const int gi = add_shape(std::move(g));
    for (int i : sel) shapes[i].parent = gi;
    return gi;
}

std::vector<int> LaserDocument::ungroup(int group_index)
{
    if (group_index < 0 || group_index >= int(shapes.size()) || shapes[group_index].type != ShapeType::Group)
        return {};
    const int parent = shapes[group_index].parent;
    std::vector<bool> is_member(shapes.size(), false);
    for (int m : children(group_index)) {
        shapes[m].parent = parent;
        is_member[m] = true;
    }
    std::vector<bool> keep(shapes.size(), true);
    keep[group_index] = false;
    const std::vector<int> map = compact(shapes, keep);
    std::vector<int> out;
    for (size_t i = 0; i < is_member.size(); ++i)
        if (is_member[i]) out.push_back(map[i]);
    return out;
}

void LaserDocument::transform(int index, const Transform2d& t)
{
    const int n = int(shapes.size());
    if (index < 0 || index >= n) return;
    for (int i = 0; i < n; ++i) {
        bool hit = i == index;
        for (int p = shapes[i].parent, steps = 0; !hit && p >= 0 && steps <= n; p = shapes[p].parent, ++steps)
            hit = p == index;
        if (hit) shapes[i].xform = t * shapes[i].xform;
    }
}

// ---- Queries -----------------------------------------------------------------------------------

std::vector<int> LaserDocument::children(int index) const
{
    std::vector<int> out;
    for (int i = 0; i < int(shapes.size()); ++i)
        if (shapes[i].parent == index && index >= 0) out.push_back(i);
    return out;
}

int LaserDocument::root_of(int index) const
{
    if (index < 0 || index >= int(shapes.size())) return -1;
    for (int steps = 0; shapes[index].parent >= 0 && steps <= int(shapes.size()); ++steps)
        index = shapes[index].parent;
    return index;
}

int LaserDocument::find(uint64_t id) const
{
    for (int i = 0; i < int(shapes.size()); ++i)
        if (shapes[i].id == id) return i;
    return -1;
}

BoundingBoxf LaserDocument::bounds() const
{
    std::vector<int> top;
    for (int i = 0; i < int(shapes.size()); ++i)
        if (shapes[i].parent < 0 && shapes[i].visible) top.push_back(i);
    return bounds(top);
}

BoundingBoxf LaserDocument::bounds(const std::vector<int>& indices) const
{
    BoundingBoxf bb;
    for (int i : indices)
        for (const LaserPath& p : flatten(i))
            for (const Point& pt : p.pts.points) bb.merge(unscale(pt));
    return bb;
}

namespace {

// Points of an arc of radius r around c from a0 to a1 (radians), end included, chord error <= tol.
int arc_segments(double r, double sweep, double tol)
{
    if (r <= tol) return 1;
    const double step = 2. * std::acos(std::max(-1., 1. - tol / r));
    return std::clamp(int(std::ceil(std::abs(sweep) / std::max(step, 1e-3))), 1, 10000);
}

LaserPath closed_path(const std::vector<Vec2d>& pts, const Transform2d& xf)
{
    LaserPath p;
    p.closed = true;
    p.pts.points.reserve(pts.size());
    for (const Vec2d& v : pts) p.pts.points.emplace_back(Point::new_scale(xf * v));
    return p;
}

} // namespace

LaserPaths LaserDocument::flatten(int index, double tol_mm) const
{
    LaserPaths out;
    if (index < 0 || index >= int(shapes.size())) return out;
    const LaserShape& s   = shapes[index];
    const double      tol = std::max(tol_mm, 1e-4);
    std::vector<Vec2d> pts;
    switch (s.type) {
    case ShapeType::Path:
        for (const LaserPath& p : s.paths) {
            LaserPath q;
            q.closed = p.closed;
            q.pts.points.reserve(p.pts.size());
            for (const Point& pt : p.pts.points) q.pts.points.emplace_back(Point::new_scale(s.xform * unscale(pt)));
            out.push_back(std::move(q));
        }
        return out;
    case ShapeType::Rect: {
        const double w = std::abs(s.width) / 2, h = std::abs(s.height) / 2;
        const double r = std::clamp(s.corner_radius, 0., std::min(w, h));
        if (r <= 0) {
            pts = {{-w, -h}, {w, -h}, {w, h}, {-w, h}};
        } else {
            const int    n = arc_segments(r, M_PI / 2, tol);
            const Vec2d  centres[4] = {{w - r, -h + r}, {w - r, h - r}, {-w + r, h - r}, {-w + r, -h + r}};
            for (int c = 0; c < 4; ++c)
                for (int k = 0; k <= n; ++k) {
                    const double a = (c - 1) * M_PI / 2 + k * (M_PI / 2) / n;   // -90 deg: bottom-right corner first
                    pts.push_back(centres[c] + r * Vec2d(std::cos(a), std::sin(a)));
                }
        }
        break;
    }
    case ShapeType::Ellipse: {
        // A multiple of 4, so the extremes (the bounding box) are vertices.
        const int n = (std::max(8, arc_segments(std::max(std::abs(s.rx), std::abs(s.ry)), 2 * M_PI, tol)) + 3) / 4 * 4;
        for (int k = 0; k < n; ++k) {
            const double a = 2 * M_PI * k / n;
            pts.emplace_back(s.rx * std::cos(a), s.ry * std::sin(a));
        }
        break;
    }
    case ShapeType::Polygon: {
        const int n = std::clamp(s.sides, 3, 1000);   // a corrupt file must not ask for 2e9 vertices
        for (int k = 0; k < n; ++k) {   // first vertex at the top (LightBurn)
            const double a = M_PI / 2 + 2 * M_PI * k / n;
            pts.emplace_back(s.rx * std::cos(a), s.ry * std::sin(a));
        }
        break;
    }
    case ShapeType::Text:
        for (const Polygon& poly : s.text_outlines) {
            LaserPath q;
            q.closed = true;
            for (const Point& pt : poly.points) q.pts.points.emplace_back(Point::new_scale(s.xform * unscale(pt)));
            out.push_back(std::move(q));
        }
        return out;
    case ShapeType::Image: {
        const double w = s.width_mm / 2, h = s.height_mm / 2;
        if (w <= 0 || h <= 0) return out;
        pts = {{-w, -h}, {w, -h}, {w, h}, {-w, h}};
        break;
    }
    case ShapeType::Group:
        // Hidden members are left out: they neither show nor burn.
        for (int c : children(index))
            if (shapes[c].visible) {
                LaserPaths sub = flatten(c, tol_mm);
                out.insert(out.end(), std::make_move_iterator(sub.begin()), std::make_move_iterator(sub.end()));
            }
        return out;
    }
    out.push_back(closed_path(pts, s.xform));
    return out;
}

} // namespace Slic3r::Laser

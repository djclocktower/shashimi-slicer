#pragma once

// The laser project: shapes, the 30 colour layers, the device name and the job settings.
// Persisted in the 3MF as Metadata/shashimi_laser.bin through Model::laser_recipe, and as a
// .slaser file. Selection is GUI state and is not stored here.

#include "libslic3r/Laser/LaserTypes.hpp"

#include <array>
#include <string>
#include <vector>

namespace Slic3r::Laser {

// LightBurn's defaults on every layer, named "C00".."C29".
std::array<LaserLayer, kLayerCount> default_layers();

class LaserDocument {
public:
    // Z order = vector order (last drawn on top). No ordering between a Group and its members:
    // members are found through LaserShape::parent.
    std::vector<LaserShape>             shapes;
    std::array<LaserLayer, kLayerCount> layers{default_layers()};
    std::string                         device_name;   // LaserDevice::name (Materials.hpp library)
    JobSettings                         job;
    // Problems found by the last deserialize(); not serialized.
    std::vector<std::string>            warnings;

    // Blob layout: u32 kVersion, then length-framed blocks in this order: shapes, layers (always
    // 30 items), settings (one item: device_name followed by the JobSettings fields). Each block
    // as laser_encode_items below. New blocks are appended; a reader ignores trailing blocks it
    // does not know.
    static constexpr int kVersion = 1;
    std::string serialize() const;
    // False (document left empty) when the blob is unreadable. Empty blob = empty document.
    // Recomputes the next id; drops bad parent indices with a warning.
    bool        deserialize(const std::string& blob);
    void        clear();
    bool        empty() const { return shapes.empty(); }

    // ---- Editing. Every helper keeps `parent` indices valid and ids unique. ----------------------
    // Assigns a fresh id; returns the new index (appended).
    int              add_shape(LaserShape shape);
    // `parent` inside `batch` indexes `batch` itself (as Import returns it); remapped on append.
    // Returns the new indices, in batch order.
    std::vector<int> add_shapes(std::vector<LaserShape> batch);
    // Removes the shapes and every descendant of removed groups; a group left with no members is
    // removed too.
    void             remove_shapes(std::vector<int> indices);
    // Deep copies (groups with their members), fresh ids, appended. Returns the new indices of
    // the copies of `indices` (not of their members).
    std::vector<int> duplicate(const std::vector<int>& indices);
    // New Group (appended) whose members are `indices` (their current parent must be the same;
    // the group takes that parent). Returns the group's index, -1 when fewer than 2 shapes.
    int              group(const std::vector<int>& indices);
    // Members go to the group's parent, the group is removed. Returns the members' indices after
    // the removal.
    std::vector<int> ungroup(int group_index);
    // Applies `t` (workspace frame, on the left) to the shape and, for a group, every descendant.
    void             transform(int index, const Transform2d& t);

    // ---- Queries --------------------------------------------------------------------------------
    std::vector<int> children(int index) const;      // direct members, document order
    int              root_of(int index) const;       // top-level ancestor (itself when top level)
    int              find(uint64_t id) const;        // -1 when missing
    // Workspace mm. The no-argument form covers every visible shape; the other the listed shapes
    // (groups through their members). Undefined box when nothing has geometry.
    BoundingBoxf     bounds() const;
    BoundingBoxf     bounds(const std::vector<int>& indices) const;
    // The shape's outline in the workspace frame (xform applied), curves flattened to `tol_mm`.
    // Rect/Ellipse/Polygon/Text: closed paths; Image: its closed border rectangle; Group: every
    // descendant's paths.
    LaserPaths       flatten(int index, double tol_mm = 0.02) const;

private:
    uint64_t m_next_id{1};
};

// ---- Framed item block (the same layout as sketch_dimensions_encode / cam_encode_items) -------
// u32 block version, u32 item count, then per item a u32 byte length + its cereal payload. A reader
// takes the fields it knows from each item and skips trailing bytes; a shorter (older) item keeps
// defaults for the missing fields. Instantiated in LaserDocument.cpp for LaserShape and LaserLayer.
template<class T> std::string laser_encode_items(const std::vector<T>& items);
template<class T> bool        laser_decode_items(const std::string& block, std::vector<T>& items);

} // namespace Slic3r::Laser

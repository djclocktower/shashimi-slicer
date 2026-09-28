#pragma once

// The CAM recipe of a project: tools, setups, operations. Persisted in the 3MF as
// Metadata/shashimi_cam.bin through Model::cam_recipe. No OCCT in this header.

#include "libslic3r/CAM/CamTypes.hpp"

#include <string>
#include <vector>

namespace Slic3r::CAM {

class CamDocument {
public:
    std::vector<CamTool>      tools;
    std::vector<CamSetup>     setups;
    std::vector<CamOperation> operations;
    // Cache, parallel to `operations` (the helpers below keep it so); never serialized.
    std::vector<Toolpath>     paths;
    // Problems found by the last deserialize() (e.g. an op naming a missing tool); not serialized.
    std::vector<std::string>  warnings;

    // Staleness (not serialized): generate_toolpath() stamps Toolpath::generation with this.
    // Bump it when the bodies, a setup or a tool change; call invalidate(op) when one op changes.
    uint64_t model_generation{1};
    void     mark_model_changed() { ++model_generation; }
    void     invalidate(int op_index);
    // True when paths[op_index] is missing or was generated before the last change.
    bool     is_stale(int op_index) const;

    // Blob layout: u32 kVersion, then three length-framed blocks in this order: tools, setups,
    // operations (each block as cam_encode_items below). New blocks are appended after them; a
    // reader ignores trailing blocks it does not know.
    static constexpr int kVersion = 1;
    std::string serialize() const;
    // False (document left empty) when the blob is unreadable. Empty blob = empty document.
    bool        deserialize(const std::string& blob);
    void        clear();

    // Tools. add_tool gives the tool the next free number when its number is taken. Returns the
    // index in `tools`.
    int            add_tool(const CamTool& tool);
    bool           remove_tool(int index);
    const CamTool* find_tool(int number) const;
    int            next_tool_number() const;

    // Setups. remove_setup also removes the setup's operations and shifts later setup_index down.
    int  add_setup(const CamSetup& setup);
    bool remove_setup(int index);

    // Operations. add_operation inserts after the last operation of op.setup_index (the list stays
    // grouped by setup). All return the operation's new index, -1 on a bad index.
    int  add_operation(const CamOperation& op);
    bool remove_operation(int index);
    int  move_operation(int from, int to);
    int  duplicate_operation(int index);
};

// A default operation of `type`, named and tuned for `tool` (stepover/stepdown/heights/cycle
// scaled to its diameter; tool_number set). tool == nullptr: the struct defaults.
CamOperation default_operation(OpType type, const CamTool* tool = nullptr);

// ---- Framed item block (the same layout as sketch_dimensions_encode) --------------------------
// u32 block version, u32 item count, then per item a u32 byte length + that item's cereal payload.
// A reader takes the fields it knows from each item and skips trailing bytes; a shorter (older)
// item keeps defaults for the missing fields. Instantiated in CamDocument.cpp for CamTool,
// CamSetup and CamOperation.
template<class T> std::string cam_encode_items(const std::vector<T>& items);
template<class T> bool        cam_decode_items(const std::string& block, std::vector<T>& items);

// ---- Setup geometry ----------------------------------------------------------------------------
// Stock and WCS rules are in CamTypes.hpp (Stock, WcsOrigin, CamSetup::a_index_deg).
CamSetupFrame compute_setup_frame(const CamSetup& setup, const CamModel& model);
// Fills model.setups (one per doc.setups). Call after the bodies or any setup changed.
void          update_setup_frames(CamModel& model, const CamDocument& doc);
// World point -> the setup's PART frame (world rotated by a_index_deg about the index axis), e.g. a
// picked point for WcsOrigin::custom_point.
Vec3d         world_to_part_frame(const CamSetup& setup, const CamModel& model, const Vec3d& world);
// The setup's bodies merged, in the setup frame.
TriangleMesh  setup_mesh(const CamDocument& doc, const CamModel& model, int setup_index);
// Heights to setup-frame Z; sel_top/sel_bottom are the op's selection extent (generators compute
// it; pass model top/bottom when the op has no selection).
ResolvedHeights resolve_heights(const Heights& h, const CamSetupFrame& frame, double sel_top, double sel_bottom);

} // namespace Slic3r::CAM

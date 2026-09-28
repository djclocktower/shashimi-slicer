#include "libslic3r/CAM/CamDocument.hpp"

#include <cereal/archives/binary.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace Slic3r::CAM {

// ---- Framed item blocks ------------------------------------------------------------------------

static constexpr uint32_t kItemBlockVersion = 1;

template<class T> std::string cam_encode_items(const std::vector<T>& items)
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

template<class T> bool cam_decode_items(const std::string& block, std::vector<T>& items)
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
        return false;   // truncated block: the complete items are kept
    }
    return true;
}

template std::string cam_encode_items<CamTool>(const std::vector<CamTool>&);
template std::string cam_encode_items<CamSetup>(const std::vector<CamSetup>&);
template std::string cam_encode_items<CamOperation>(const std::vector<CamOperation>&);
template bool        cam_decode_items<CamTool>(const std::string&, std::vector<CamTool>&);
template bool        cam_decode_items<CamSetup>(const std::string&, std::vector<CamSetup>&);
template bool        cam_decode_items<CamOperation>(const std::string&, std::vector<CamOperation>&);

// ---- CamDocument -------------------------------------------------------------------------------

std::string CamDocument::serialize() const
{
    std::ostringstream os;
    {
        cereal::BinaryOutputArchive ar(os);
        const uint32_t version = kVersion;
        ar(version);
        for (const std::string& block : {cam_encode_items(tools), cam_encode_items(setups), cam_encode_items(operations)}) {
            const uint32_t len = uint32_t(block.size());
            ar(len);
            ar(cereal::binary_data(block.data(), block.size()));
        }
    }
    return os.str();
}

bool CamDocument::deserialize(const std::string& blob)
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
    if (!cam_decode_items(blocks[0], tools) || !cam_decode_items(blocks[1], setups) || !cam_decode_items(blocks[2], operations)) {
        clear();
        return false;
    }
    paths.assign(operations.size(), Toolpath{});
    for (const CamOperation& op : operations) {
        if (!find_tool(op.tool_number))
            warnings.push_back("Operation \"" + op.name + "\" uses tool T" + std::to_string(op.tool_number) +
                               ", which is not in the document.");
        if (op.setup_index < 0 || op.setup_index >= int(setups.size()))
            warnings.push_back("Operation \"" + op.name + "\" belongs to a setup that does not exist.");
    }
    return true;
}

void CamDocument::clear()
{
    tools.clear();
    setups.clear();
    operations.clear();
    paths.clear();
    warnings.clear();
}

const CamTool* CamDocument::find_tool(int number) const
{
    for (const CamTool& t : tools)
        if (t.number == number)
            return &t;
    return nullptr;
}

int CamDocument::next_tool_number() const
{
    int n = 0;
    for (const CamTool& t : tools)
        n = std::max(n, t.number);
    return n + 1;
}

int CamDocument::add_tool(const CamTool& tool)
{
    CamTool t = tool;
    if (t.number < 1 || find_tool(t.number))
        t.number = next_tool_number();
    tools.push_back(std::move(t));
    return int(tools.size()) - 1;
}

bool CamDocument::remove_tool(int index)
{
    if (index < 0 || index >= int(tools.size()))
        return false;
    tools.erase(tools.begin() + index);
    return true;
}

void CamDocument::invalidate(int op_index)
{
    if (op_index >= 0 && op_index < int(paths.size()))
        paths[op_index].generation = 0;
}

bool CamDocument::is_stale(int op_index) const
{
    return op_index < 0 || op_index >= int(paths.size()) || paths[op_index].generation != model_generation;
}

int CamDocument::add_setup(const CamSetup& setup)
{
    setups.push_back(setup);
    return int(setups.size()) - 1;
}

bool CamDocument::remove_setup(int index)
{
    if (index < 0 || index >= int(setups.size()))
        return false;
    paths.resize(operations.size());
    for (int i = int(operations.size()) - 1; i >= 0; --i) {
        if (operations[i].setup_index == index) {
            operations.erase(operations.begin() + i);
            paths.erase(paths.begin() + i);
        } else if (operations[i].setup_index > index)
            --operations[i].setup_index;
    }
    setups.erase(setups.begin() + index);
    return true;
}

int CamDocument::add_operation(const CamOperation& op)
{
    if (op.setup_index < 0 || op.setup_index >= int(setups.size()))
        return -1;
    paths.resize(operations.size());
    int pos = 0;
    for (int i = 0; i < int(operations.size()); ++i)
        if (operations[i].setup_index <= op.setup_index)
            pos = i + 1;
    operations.insert(operations.begin() + pos, op);
    paths.insert(paths.begin() + pos, Toolpath{});
    return pos;
}

bool CamDocument::remove_operation(int index)
{
    if (index < 0 || index >= int(operations.size()))
        return false;
    paths.resize(operations.size());
    operations.erase(operations.begin() + index);
    paths.erase(paths.begin() + index);
    return true;
}

int CamDocument::move_operation(int from, int to)
{
    const int n = int(operations.size());
    if (from < 0 || from >= n || to < 0 || to >= n)
        return -1;
    paths.resize(operations.size());
    // Stay inside the op's setup group so the list remains grouped by setup.
    const int setup = operations[from].setup_index;
    int       lo = from, hi = from;
    while (lo > 0 && operations[lo - 1].setup_index == setup) --lo;
    while (hi + 1 < n && operations[hi + 1].setup_index == setup) ++hi;
    to = std::clamp(to, lo, hi);
    CamOperation op = std::move(operations[from]);
    Toolpath     tp = std::move(paths[from]);
    operations.erase(operations.begin() + from);
    paths.erase(paths.begin() + from);
    operations.insert(operations.begin() + to, std::move(op));
    paths.insert(paths.begin() + to, std::move(tp));
    return to;
}

int CamDocument::duplicate_operation(int index)
{
    if (index < 0 || index >= int(operations.size()))
        return -1;
    paths.resize(operations.size());
    CamOperation op = operations[index];
    op.name += " copy";
    operations.insert(operations.begin() + index + 1, std::move(op));
    paths.insert(paths.begin() + index + 1, paths[index]);
    return index + 1;
}

// ---- Defaults ----------------------------------------------------------------------------------

static const char* op_type_name(OpType t)
{
    switch (t) {
    case OpType::Face: return "Face";
    case OpType::Adaptive2D: return "Adaptive";
    case OpType::Pocket2D: return "Pocket";
    case OpType::Contour2D: return "Contour";
    case OpType::Slot: return "Slot";
    case OpType::Drill: return "Drill";
    case OpType::Bore: return "Bore";
    case OpType::Chamfer2D: return "Chamfer";
    case OpType::Engrave: return "Engrave";
    case OpType::Trace: return "Trace";
    case OpType::Adaptive3D: return "3D Adaptive";
    case OpType::Parallel3D: return "Parallel";
    case OpType::Contour3D: return "3D Contour";
    case OpType::RotaryWrap: return "Wrap";
    case OpType::RotaryFinish: return "Rotary Finish";
    }
    return "Operation";
}

CamOperation default_operation(OpType type, const CamTool* tool)
{
    CamOperation op;
    op.type = type;
    op.name = op_type_name(type);
    if (!tool)
        return op;
    const double d = std::max(0.1, tool->diameter);
    op.tool_number    = tool->number;
    op.stepover       = 0.4 * d;
    op.stepdown       = std::min(0.5 * d, tool->flute_length);
    op.lead_in_radius = 0.5 * d;
    op.optimal_load   = 0.1 * d;
    op.min_stepdown   = 0.1 * d;
    op.peck_depth     = d;
    switch (type) {
    case OpType::Face:
        op.stepover       = 0.7 * d;
        op.stepdown       = 1.0;
        op.heights.bottom = {HeightRef::ModelTop, 0};
        break;
    case OpType::Adaptive2D:
    case OpType::Adaptive3D:
        op.stepdown = std::min(2.0 * d, tool->flute_length);
        break;
    case OpType::Slot: op.stepdown = 0.25 * d; break;
    case OpType::Drill:
        op.cycle          = tool->type == ToolType::SpotDrill ? DrillCycle::Drill
                          : tool->type == ToolType::Tap      ? DrillCycle::Tap
                                                              : DrillCycle::Peck;
        op.heights.retract = {HeightRef::StockTop, 2};
        if (tool->type == ToolType::SpotDrill)
            op.heights.bottom = {HeightRef::SelectionTop, -0.25 * d};   // 90 deg spot: a d/2 wide mark
        break;
    case OpType::Chamfer2D:
        op.heights.bottom = {HeightRef::SelectionTop, 0};
        break;
    case OpType::Engrave:
    case OpType::Trace:
        op.heights.bottom = {HeightRef::SelectionTop, -0.5};
        op.stepdown       = 0.5;
        break;
    case OpType::Parallel3D:
        op.stepover        = tool->type == ToolType::BallEndMill ? 0.1 * d : 0.3 * d;
        op.geom.whole_model = true;
        op.heights.bottom  = {HeightRef::ModelBottom, 0};
        break;
    case OpType::Contour3D:
        op.stepdown         = std::max(0.05, 0.1 * d);
        op.geom.whole_model = true;
        op.heights.bottom   = {HeightRef::ModelBottom, 0};
        break;
    case OpType::RotaryFinish:
        op.geom.whole_model = true;
        op.heights.bottom   = {HeightRef::ModelBottom, 0};
        break;
    default: break;
    }
    if (type == OpType::Adaptive3D) {
        op.geom.whole_model          = true;
        op.stock_to_leave_radial     = 0.3;
        op.stock_to_leave_axial      = 0.2;
        op.heights.bottom            = {HeightRef::ModelBottom, 0};
    }
    return op;
}

// ---- Setup geometry ----------------------------------------------------------------------------

static bool setup_has_body(const CamSetup& setup, int body_id)
{
    return setup.body_ids.empty() || std::find(setup.body_ids.begin(), setup.body_ids.end(), body_id) != setup.body_ids.end();
}

// Rotation of a_deg about the X line through (y, z).
static Transform3d index_rotation(double a_deg, double y, double z)
{
    Transform3d t = Transform3d::Identity();
    if (a_deg == 0)
        return t;
    t.translate(Vec3d(0, y, z));
    t.rotate(Eigen::AngleAxisd(a_deg * M_PI / 180.0, Vec3d::UnitX()));
    t.translate(Vec3d(0, -y, -z));
    return t;
}

static double pick(double lo, double hi, int k) { return k == 0 ? lo : k == 1 ? 0.5 * (lo + hi) : hi; }

// Index axis (world Y, Z of the X line): the cylinder's axis, or the X line through the model
// bounds' YZ centre.
static Vec2d index_axis(const CamSetup& setup, const CamModel& model)
{
    if (setup.stock.kind == StockKind::Cylinder && !setup.stock.axis_auto)
        return setup.stock.axis_yz;
    BoundingBoxf3 world;
    for (const CamBody& b : model.bodies)
        if (setup_has_body(setup, b.body_id) && !b.mesh.empty())
            world.merge(b.mesh.bounding_box());
    if (!world.defined)
        return Vec2d::Zero();
    return Vec2d(0.5 * (world.min.y() + world.max.y()), 0.5 * (world.min.z() + world.max.z()));
}

Vec3d world_to_part_frame(const CamSetup& setup, const CamModel& model, const Vec3d& world)
{
    const Vec2d axis = index_axis(setup, model);
    return index_rotation(setup.a_index_deg, axis.x(), axis.y()) * world;
}

CamSetupFrame compute_setup_frame(const CamSetup& setup, const CamModel& model)
{
    CamSetupFrame     frame;
    const Stock&      st   = setup.stock;
    const bool        cyl  = st.kind == StockKind::Cylinder;
    const Vec2d       axis = index_axis(setup, model);
    const Transform3d R    = index_rotation(setup.a_index_deg, axis.x(), axis.y());

    BoundingBoxf3 part;
    double        model_r = 0;
    for (const CamBody& b : model.bodies)
        if (setup_has_body(setup, b.body_id) && !b.mesh.empty()) {
            part.merge(b.mesh.transformed_bounding_box(R));
            if (cyl)
                for (const stl_vertex& v : b.mesh.its.vertices)
                    model_r = std::max(model_r, Vec2d(double(v.y()) - axis.x(), double(v.z()) - axis.y()).norm());
        }
    if (!part.defined)
        part = BoundingBoxf3(Vec3d::Zero(), Vec3d::Zero());

    // Stock, part frame.
    BoundingBoxf3 stock;
    double        radius = 0;
    if (st.kind == StockKind::CustomBox)
        stock = BoundingBoxf3(st.box_min.cwiseMin(st.box_max), st.box_min.cwiseMax(st.box_max));
    else if (cyl) {
        radius          = st.radius > 0 ? st.radius : model_r + std::max(st.offset_pos.y(), st.offset_pos.z());
        const double x0 = part.min.x() - st.offset_neg.x();
        const double x1 = st.length > 0 ? x0 + st.length : part.max.x() + st.offset_pos.x();
        stock = BoundingBoxf3(Vec3d(x0, axis.x() - radius, axis.y() - radius), Vec3d(x1, axis.x() + radius, axis.y() + radius));
    } else
        stock = BoundingBoxf3(part.min - st.offset_neg, part.max + st.offset_pos);

    // WCS origin, part frame.
    const WcsOrigin&     w   = setup.wcs;
    const BoundingBoxf3& box = w.box == WcsBox::Stock ? stock : part;
    const int            col = int(w.point) % 3, row = int(w.point) / 3;
    Vec3d origin = w.custom ? w.custom_point
                            : Vec3d(pick(box.min.x(), box.max.x(), col), pick(box.min.y(), box.max.y(), row),
                                    w.z == WcsZ::Top ? box.max.z() : box.min.z());
    if (cyl)
        origin = Vec3d(origin.x(), axis.x(), axis.y());

    frame.to_setup     = Transform3d(Eigen::Translation3d(-origin)) * R;
    frame.stock        = BoundingBoxf3(stock.min - origin, stock.max - origin);
    frame.model        = BoundingBoxf3(part.min - origin, part.max - origin);
    frame.stock_radius = radius;
    return frame;
}

void update_setup_frames(CamModel& model, const CamDocument& doc)
{
    model.setups.clear();
    for (const CamSetup& s : doc.setups)
        model.setups.push_back(compute_setup_frame(s, model));
}

TriangleMesh setup_mesh(const CamDocument& doc, const CamModel& model, int setup_index)
{
    TriangleMesh out;
    if (setup_index < 0 || setup_index >= int(doc.setups.size()))
        return out;
    const Transform3d t = setup_index < int(model.setups.size()) ? model.setups[setup_index].to_setup
                                                                   : compute_setup_frame(doc.setups[setup_index], model).to_setup;
    for (const CamBody& b : model.bodies)
        if (setup_has_body(doc.setups[setup_index], b.body_id)) {
            TriangleMesh m = b.mesh;
            m.transform(t);
            out.merge(m);
        }
    return out;
}

ResolvedHeights resolve_heights(const Heights& h, const CamSetupFrame& frame, double sel_top, double sel_bottom)
{
    auto z = [&](const Height& ht) {
        switch (ht.ref) {
        case HeightRef::StockTop: return frame.stock.max.z() + ht.offset;
        case HeightRef::StockBottom: return frame.stock.min.z() + ht.offset;
        case HeightRef::ModelTop: return frame.model.max.z() + ht.offset;
        case HeightRef::ModelBottom: return frame.model.min.z() + ht.offset;
        case HeightRef::SelectionTop: return sel_top + ht.offset;
        case HeightRef::SelectionBottom: return sel_bottom + ht.offset;
        case HeightRef::Absolute: break;
        }
        return ht.offset;
    };
    ResolvedHeights r;
    r.top       = z(h.top);
    r.bottom    = z(h.bottom);
    // Never plunge from below the cut top, never rapid below the retract plane.
    r.retract   = std::max(z(h.retract), r.top);
    r.clearance = std::max(z(h.clearance), r.retract);
    return r;
}

} // namespace Slic3r::CAM

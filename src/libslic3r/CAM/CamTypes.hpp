#pragma once

// Shashimi CAM: the flat data model shared by every CAM source file.
//
// Conventions (see README.md in this directory):
//  - Units: mm, mm/min, RPM, degrees, seconds. ExPolygons/Polylines/Point are libslic3r SCALED
//    coordinates (scale_() / unscale()); everything typed double/Vec3d is plain mm.
//  - Frames: toolpaths live in the SETUP frame (X right, Y back, Z up, origin = the setup's WCS).
//    CamModel bodies/meshes are in the CAD WORLD frame; CamSetupFrame::to_setup maps world->setup
//    and already contains the setup's A index rotation.
//  - A axis: rotation about +X (right-hand rule). A rotary setup's A axis is the setup-frame X
//    axis (Y = Z = 0).
//  - Serialization: cereal, APPEND-ONLY. The top-level records (CamTool, CamSetup, CamOperation)
//    are length-framed one by one in the document blob (CamDocument.hpp), so a new field is added
//    by appending it to the END of that record's serialize() list, never reordering. The nested
//    value structs (FeedsSpeeds, Stock, WcsOrigin, FaceRef, EdgeRef, GeometrySelection, Height,
//    Heights) are serialized inline inside a record and are therefore FROZEN once shipped: a new
//    field for one of them is appended to the owning record instead.
//  - Every enum is serialized as its integer: APPEND-ONLY, never reorder.

#include "libslic3r/Point.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <cereal/cereal.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace Slic3r::CAM {

// ---- Tools -------------------------------------------------------------------------------------

enum class ToolType { FlatEndMill, BallEndMill, BullEndMill, ChamferMill, VBit, Drill, SpotDrill, Tap };
enum class ToolMaterial { HSS, Carbide };

struct FeedsSpeeds {
    double rpm{10000};
    double feed{800};          // cutting feed, mm/min
    double plunge_feed{300};   // straight Z entry, mm/min
    double ramp_feed{400};     // ramp / helix entry, mm/min
    double chipload{0.04};     // mm per tooth (informational once feed is set)
    double surface_speed{0};   // m/min (informational)

    template<class Archive> void serialize(Archive& ar)
    { ar(rpm, feed, plunge_feed, ramp_feed, chipload, surface_speed); }
};

struct CamTool {
    int          number{1};             // T number, unique within a document
    std::string  name;
    ToolType     type{ToolType::FlatEndMill};
    double       diameter{6};           // cutting diameter (V-bit/chamfer: the widest diameter)
    double       corner_radius{0};      // Bull: corner radius; Ball: ignored (= diameter/2)
    double       tip_angle_deg{90};     // V-bit/chamfer included angle; drill point angle
    double       tip_diameter{0};       // V-bit/chamfer flat tip diameter (0 = sharp)
    double       flute_length{20};
    double       overall_length{50};
    double       shank_diameter{6};
    int          flutes{2};
    ToolMaterial material{ToolMaterial::Carbide};
    std::string  coating;
    double       thread_pitch{0};       // Tap only, mm
    bool         unit_inch{false};      // display only; all fields stay mm
    bool         override_feeds{false}; // true: `feeds` below replaces recommend_feeds()
    FeedsSpeeds  feeds;

    template<class Archive> void serialize(Archive& ar)
    {
        ar(number, name, type, diameter, corner_radius, tip_angle_deg, tip_diameter, flute_length,
           overall_length, shank_diameter, flutes, material, coating, thread_pitch, unit_inch,
           override_feeds, feeds);
    }
};

// ---- Machine -----------------------------------------------------------------------------------

enum class Material { Aluminum, Brass, MildSteel, StainlessSteel, Plastic, Acrylic, HDPE, HardWood, SoftWood, Plywood, MDF, Foam, Wax };
enum class PostDialect { Grbl, LinuxCNC, Mach3, Fanuc, Marlin };
enum class ToolChange { ManualPause, M6 };

// Not stored in the document (a setup names its machine); lives in resources/cam/machines.json.
struct MachineProfile {
    std::string name{"Generic 3-axis"};
    double      max_rpm{24000};
    double      min_rpm{0};
    double      max_feed_xy{5000};   // mm/min
    double      max_feed_z{1000};
    double      rapid_feed{5000};
    Vec3d       travel{300, 300, 100};
    bool        has_a_axis{false};
    PostDialect post{PostDialect::Grbl};
    ToolChange  tool_change{ToolChange::ManualPause};
    bool        spindle_control{true};   // emits M3 Sn / M5
    bool        coolant{false};          // emits M8 / M9
};

// ---- Setup -------------------------------------------------------------------------------------

enum class StockKind { Box, Cylinder, CustomBox };

// All stock geometry is in the PART frame: CAD world rotated by CamSetup::a_index_deg about the
// index axis (Cylinder: the stock axis; Box: the X line through the model bounds' YZ centre).
struct Stock {
    StockKind kind{StockKind::Box};
    // Box and Cylinder: grow the model bounds by these (min side, max side). Cylinder uses x of
    // both for its ends and max(y, z) of offset_pos as extra radius.
    Vec3d  offset_neg{1, 1, 0};
    Vec3d  offset_pos{1, 1, 1};
    // Cylinder (axis parallel to X): radius/length 0 = from the model bounds + offsets.
    double radius{0};
    double length{0};
    // Cylinder axis position in world Y/Z; ignored while axis_auto (model bounds YZ centre).
    bool   axis_auto{true};
    Vec2d  axis_yz{0, 0};
    // CustomBox: explicit corners, part frame.
    Vec3d  box_min{0, 0, 0};
    Vec3d  box_max{100, 100, 20};

    template<class Archive> void serialize(Archive& ar)
    { ar(kind, offset_neg, offset_pos, radius, length, axis_auto, axis_yz, box_min, box_max); }
};

enum class WcsBox { Stock, Model };
// The 9 box points seen from above, front = -Y.
enum class WcsPoint { FrontLeft, Front, FrontRight, Left, Center, Right, BackLeft, Back, BackRight };
enum class WcsZ { Top, Bottom };

// Cylinder stock always puts setup Y = Z = 0 on the stock axis; only the X of the anchor
// (Left/Center/Right column) applies.
struct WcsOrigin {
    WcsBox   box{WcsBox::Stock};
    WcsPoint point{WcsPoint::FrontLeft};
    WcsZ     z{WcsZ::Top};
    bool     custom{false};
    Vec3d    custom_point{0, 0, 0};   // part frame

    template<class Archive> void serialize(Archive& ar) { ar(box, point, z, custom, custom_point); }
};

struct CamSetup {
    std::string      name{"Setup1"};
    std::string      machine{"Generic 3-axis"};   // MachineProfile::name (Machines.hpp)
    Material         material{Material::Aluminum};
    Stock            stock;
    WcsOrigin        wcs;
    int              work_offset{54};   // 54..59 -> G54..G59
    std::vector<int> body_ids;          // CadDocument body indices; empty = every body
    // Indexed 4-axis: part and stock rotated about X by this for every op of the setup (the post
    // emits G0 A<angle> at the setup start). On a 3-axis machine this is a manual flip
    // (180 = machine the bottom side).
    double           a_index_deg{0};

    template<class Archive> void serialize(Archive& ar)
    { ar(name, machine, material, stock, wcs, work_offset, body_ids, a_index_deg); }
};

// ---- Operations --------------------------------------------------------------------------------

enum class OpType { Face, Adaptive2D, Pocket2D, Contour2D, Slot, Drill, Bore, Chamfer2D, Engrave, Trace,
                    Adaptive3D, Parallel3D, Contour3D, RotaryWrap, RotaryFinish };

// Face / edge ids are CadDocument ordinal ids (TopExp maps) of body `body`: they are only valid
// for the topology generation the selection was made against.
struct FaceRef { int body{-1}; int face{-1}; template<class Archive> void serialize(Archive& ar) { ar(body, face); } };
struct EdgeRef { int body{-1}; int edge{-1}; template<class Archive> void serialize(Archive& ar) { ar(body, edge); } };

struct GeometrySelection {
    std::vector<FaceRef> faces;
    std::vector<EdgeRef> edges;
    int                  sketch_feature{-1};   // CadDocument feature index of a Sketch, -1 none
    bool                 whole_model{false};   // 3D ops: every body of the setup
    std::vector<Vec3d>   points;               // drill points, setup frame

    template<class Archive> void serialize(Archive& ar) { ar(faces, edges, sketch_feature, whole_model, points); }
};

enum class HeightRef { StockTop, StockBottom, ModelTop, ModelBottom, SelectionTop, SelectionBottom, Absolute };

// Setup-frame Z = reference + offset (Absolute: offset is the Z).
struct Height {
    HeightRef ref{HeightRef::StockTop};
    double    offset{0};
    template<class Archive> void serialize(Archive& ar) { ar(ref, offset); }
};

struct Heights {
    Height clearance{HeightRef::StockTop, 10};       // rapids between regions
    Height retract{HeightRef::StockTop, 5};          // short links; plunges start here
    Height top{HeightRef::StockTop, 0};              // first cut level
    Height bottom{HeightRef::SelectionBottom, 0};    // final depth
    template<class Archive> void serialize(Archive& ar) { ar(clearance, retract, top, bottom); }
};

// Heights evaluated to setup-frame Z (resolve_heights in CamDocument.hpp).
struct ResolvedHeights { double clearance{15}, retract{10}, top{0}, bottom{-1}; };

enum class ContourSide { Outside, Inside, On };
enum class CutOrdering { DepthFirst, LevelFirst };
enum class DrillCycle { Drill, Peck, ChipBreak, Bore, Tap };
enum class EntryType { Helix, Ramp, Plunge };
enum class Boundary { Stock, Silhouette, Selection };

// One flat struct for every OpType (like CadFeature); each generator reads the fields it needs.
// Defaults suit a 6 mm flat end mill in aluminum; default_operation() (CamDocument.hpp) rescales
// them to the actual tool.
struct CamOperation {
    OpType            type{OpType::Pocket2D};
    std::string       name{"Operation"};
    bool              enabled{true};
    int               setup_index{0};
    int               tool_number{1};
    bool              feeds_auto{true};   // true: effective_feeds() recomputes; false: `feeds` as typed
    FeedsSpeeds       feeds;
    GeometrySelection geom;
    Heights           heights;

    // Passes
    double      stepover{2.4};              // mm
    double      stepdown{1.0};              // mm
    double      stock_to_leave_radial{0};
    double      stock_to_leave_axial{0};
    bool        climb{true};
    ContourSide side{ContourSide::Outside};
    int         finishing_passes{0};
    double      finish_stepover{0.1};
    double      tolerance{0.01};
    bool        rest_machining{false};
    CutOrdering ordering{CutOrdering::DepthFirst};

    // Linking / entry
    EntryType   entry{EntryType::Helix};
    double      lead_in_radius{1.0};
    double      ramp_angle_deg{2.0};
    double      helix_diameter{0};          // 0 = auto (~90 % of the tool diameter)

    // Drill / Bore
    DrillCycle  cycle{DrillCycle::Drill};
    double      peck_depth{2.0};
    double      dwell_s{0};
    double      break_through{0.5};         // extra depth past a through hole, mm

    // Chamfer
    double      chamfer_width{0.5};
    double      tip_offset{0.5};            // how far the tool tip goes past the chamfer bottom

    // Parallel 3D
    double      angle_deg{0};               // pass direction in XY, degrees from +X
    Boundary    boundary{Boundary::Silhouette};

    // Rotary
    double      wrap_radius{0};             // 0 = stock radius
    double      a_stepover_deg{2.0};
    bool        rotary_spiral{false};       // RotaryFinish: spiral instead of passes along X
    OpType      wrap_strategy{OpType::Engrave};   // RotaryWrap: the 2D op run on the unrolled cylinder

    // Adaptive
    double      optimal_load{0.6};          // max radial engagement, mm
    double      min_stepdown{0.5};
    double      lift_height{0.2};
    double      helix_angle_deg{2.0};

    // Parallel3D / RotaryFinish: cut in both directions (zig-zag); false = every pass the same way.
    bool        bidirectional{true};

    // Drill / Bore: only holes with this diameter range (0 = no limit); picked points always pass.
    double      hole_diameter_min{0};
    double      hole_diameter_max{0};

    template<class Archive> void serialize(Archive& ar)
    {
        ar(type, name, enabled, setup_index, tool_number, feeds_auto, feeds, geom, heights,
           stepover, stepdown, stock_to_leave_radial, stock_to_leave_axial, climb, side,
           finishing_passes, finish_stepover, tolerance, rest_machining, ordering,
           entry, lead_in_radius, ramp_angle_deg, helix_diameter,
           cycle, peck_depth, dwell_s, break_through,
           chamfer_width, tip_offset,
           angle_deg, boundary,
           wrap_radius, a_stepover_deg, rotary_spiral, wrap_strategy,
           optimal_load, min_stepdown, lift_height, helix_angle_deg,
           bidirectional, hole_diameter_min, hole_diameter_max);
    }
};

// ---- Toolpath (generated, never serialized) ----------------------------------------------------

enum class ArcDir { None, CW, CCW };

// One motion from the previous move's end (the first move of a toolpath starts from wherever the
// machine is; generators always begin with a Rapid at clearance height).
struct Move {
    enum class Kind { Rapid, Feed, Plunge, Ramp, LeadIn, LeadOut, ArcCW, ArcCCW, Retract };
    Kind   kind{Kind::Rapid};
    Vec3d  to{0, 0, 0};       // setup frame
    double a_deg{0};          // absolute A position at the end of the move (turns <= 180 deg per move: the post unwraps)
    Vec3d  center{0, 0, 0};   // arc centre (XY plane, G17); Z ignored
    double feed{0};           // mm/min; 0 for Rapid
    // Kind carries the display/feed meaning. A LeadIn/LeadOut/Ramp move can also be an arc
    // (lead arcs, helical ramps): `arc` says so. Use is_arc()/arc_dir() instead of testing Kind.
    ArcDir arc{ArcDir::None};
    // Drill ops: index of the hole whose canned cycle this move belongs to (-1: not part of one).
    // The run of moves sharing it leaves the R plane (the position before the run) and returns
    // to it; the post collapses the run into one G81/G82/G83/G73/G85/G84 line or emits it as is.
    int    cycle{-1};
};

inline ArcDir arc_dir(const Move& m)
{
    if (m.kind == Move::Kind::ArcCW)  return ArcDir::CW;
    if (m.kind == Move::Kind::ArcCCW) return ArcDir::CCW;
    return m.arc;
}
inline bool is_arc(const Move& m) { return arc_dir(m) != ArcDir::None; }

struct Warning {
    int         move_index{-1};   // -1: the whole operation
    Vec3d       pos{0, 0, 0};
    std::string text;             // plain language, shown to the user as is
};

struct Toolpath {
    std::vector<Move>    moves;
    double               cut_length{0};    // mm, every move but Rapid/Retract (both run at rapid)
    double               rapid_length{0};  // mm, Rapid + Retract
    double               time_s{0};
    std::string          error;            // non-empty: generation failed; moves may be partial
    std::vector<Warning> warnings;
    // CamDocument::model_generation this path was generated against (0 = never): see is_stale().
    uint64_t             generation{0};
    bool ok() const { return error.empty(); }
};

// Generator progress: called with the fraction done (0..1), possibly from worker threads (the
// callback generate_toolpath() passes down is serialised); returns true to cancel.
using ProgressFn = std::function<bool(double fraction)>;

// ---- Geometry handed to the generators ---------------------------------------------------------

// A recognised hole, in the frame the finder was given (normally the setup frame).
struct HoleFeature {
    Vec3d  center{0, 0, 0};   // on the axis, at the hole's top (entry) face
    Vec3d  axis{0, 0, 1};     // unit, pointing OUT of the material (up for a hole drilled from +Z)
    double diameter{0};
    double depth{0};          // from center along -axis
    bool   through{false};
    int    body{-1};
    int    face{-1};          // the cylindrical face, -1 for an edge/point-derived hole
};

// Defined in CamGeometry.hpp (holds a TopoDS_Shape); opaque here so OCCT stays out of this header.
struct CamBodyShape;

struct CamBody {
    int                                 body_id{-1};   // CadDocument body index
    TriangleMesh                        mesh;          // fine tessellation, world frame
    std::shared_ptr<const CamBodyShape> shape;
};

// A sketch's resolved curves, in its own plane (2D, scaled). World point of (u, v) is
// origin + u * x_axis + v * y_axis (unscaled).
struct CamSketch {
    int                feature{-1};   // CadDocument feature index
    Vec3d              origin{0, 0, 0}, x_axis{1, 0, 0}, y_axis{0, 1, 0};
    ExPolygons         regions;       // closed, non-construction profiles
    Polylines          chains;        // open, non-construction chains
    std::vector<Vec2d> points;        // Point entities (drill points), mm
};

// Per-setup derived geometry (update_setup_frames in CamDocument.hpp).
struct CamSetupFrame {
    Transform3d   to_setup{Transform3d::Identity()};   // world -> setup, includes the A index
    BoundingBoxf3 stock;                               // setup frame (Cylinder: its bounding box)
    BoundingBoxf3 model;                               // the setup's bodies, setup frame
    double        stock_radius{0};                     // Cylinder stock only
};

// Built by the GUI from the CadDocument (build_cam_model in CamGeometry.hpp); read-only input to
// every generator.
struct CamModel {
    std::vector<CamBody>       bodies;
    std::vector<CamSketch>     sketches;
    std::vector<CamSetupFrame> setups;   // parallel to CamDocument::setups
    uint64_t                   topo_generation{0};   // CadDocument::topo_generation it was built from

    const CamBody*   body(int body_id) const
    { for (const CamBody& b : bodies) if (b.body_id == body_id) return &b; return nullptr; }
    const CamSketch* sketch(int feature) const
    { for (const CamSketch& s : sketches) if (s.feature == feature) return &s; return nullptr; }
};

// ---- Post --------------------------------------------------------------------------------------

struct PostOptions {
    MachineProfile machine;            // resolved by the caller (Machines.hpp); decides the dialect
    std::string    program_name{"shashimi"};
    int            program_number{1000};   // O-number (Fanuc/Mach3)
    bool           inch{false};        // output G20 and inch values
    bool           arcs{true};         // false: linearise arcs to `arc_tolerance`
    double         arc_tolerance{0.01};
    bool           inverse_time{false};   // G93 on A moves where the dialect supports it
    bool           line_numbers{false};
    bool           comments{true};
    bool           a_modulo{false};    // A words as [0, 360) instead of unwrapped continuous angles
};

} // namespace Slic3r::CAM

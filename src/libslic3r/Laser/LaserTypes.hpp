#pragma once

// Shashimi Laser: the flat data model shared by every Laser source file (LightBurn workflow).
//
// Conventions (see README.md in this directory):
//  - Units: mm, mm/s (speeds; G-code F is mm/min), power in % (0..100, S = pct * s_max / 100),
//    degrees, seconds. Polyline/Polygon/ExPolygon coordinates are libslic3r SCALED (scale_() /
//    unscale()) mm; every double / Vec2d is plain mm.
//  - Workspace frame (the bed seen from above): X right, Y up (towards the rear), origin at the
//    FRONT-LEFT corner, like LightBurn. LaserDevice::origin_corner says where the machine's own
//    origin is; only the G-code writer (to_machine() in LaserGCode.hpp) flips.
//  - Shape geometry is in the shape's LOCAL frame; LaserShape::xform maps local -> workspace and is
//    ABSOLUTE (not relative to the parent group). Transforming a group applies the transform to every
//    descendant (LaserDocument::transform).
//  - Serialization: cereal, APPEND-ONLY. LaserShape, LaserLayer and JobSettings are length-framed
//    record by record (LaserDocument.hpp), so a field is added by appending it to the END of the
//    record's serialize() list. LaserPath is serialized inline and is FROZEN. Enums are serialized
//    as integers: only append values.

#include "libslic3r/Point.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExPolygonSerialize.hpp"
#include "libslic3r/Polyline.hpp"

#include <cereal/cereal.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace Slic3r::Laser {

// ---- Layers (LightBurn colour layers C00..C29) ------------------------------------------------

constexpr int kLayerCount = 30;

struct Rgb { uint8_t r, g, b; };

// LightBurn's default palette; the layer index IS the colour (C00 black, C01 blue, C02 red, ...).
inline constexpr std::array<Rgb, kLayerCount> kLayerPalette{{
    {0x00, 0x00, 0x00}, {0x00, 0x00, 0xFF}, {0xFF, 0x00, 0x00}, {0x00, 0xE0, 0x00}, {0xD0, 0xD0, 0x00},
    {0xFF, 0x80, 0x00}, {0x00, 0xE0, 0xE0}, {0xFF, 0x00, 0xFF}, {0xB4, 0xB4, 0xB4}, {0x00, 0x00, 0xA0},
    {0xA0, 0x00, 0x00}, {0x00, 0xA0, 0x00}, {0xA0, 0xA0, 0x00}, {0xC0, 0x80, 0x00}, {0x00, 0xA0, 0xFF},
    {0xA0, 0x00, 0xA0}, {0x80, 0x80, 0x80}, {0x7D, 0x87, 0xB9}, {0xBB, 0x77, 0x84}, {0x4A, 0x6F, 0xE3},
    {0xD3, 0x3F, 0x6A}, {0x8C, 0xD7, 0x8C}, {0xF0, 0xB9, 0x8D}, {0xF6, 0xC4, 0xE1}, {0xFA, 0x9E, 0xD4},
    {0x50, 0x0A, 0x78}, {0xB4, 0x5A, 0x00}, {0x00, 0x47, 0x54}, {0x86, 0xFA, 0x88}, {0xFF, 0xDB, 0x66},
}};

// Out-of-range index -> C00.
inline Rgb layer_color(int index) { return kLayerPalette[(index >= 0 && index < kLayerCount) ? index : 0]; }

// Line: trace vectors. Fill: scan-fill closed shapes. FillLine: Fill then Line (LightBurn order).
// OffsetFill: concentric offsets inward. Image shapes on any layer are always rastered with the
// layer's image_* settings; the mode applies to the layer's vector shapes.
enum class LayerMode { Line, Fill, FillLine, OffsetFill };
// Burn value per pixel: Grayscale maps it to power_min..power_max, every other mode is on/off.
enum class DitherMode { Threshold, Ordered, FloydSteinberg, Jarvis, Stucki, Atkinson, Newsprint, Halftone, Grayscale };
// Fill: scan all the layer's shapes as one region, each shape on its own, or each group on its own.
enum class FillGrouping { AllAtOnce, PerShape, PerGroup };

// Flat on purpose (one framed record, no frozen nested structs). Defaults = LightBurn's new layer.
struct LaserLayer {
    std::string  name;                  // empty: shown as "C00".."C29"
    bool         visible{true};         // "Show"
    bool         output{true};          // "Output": false skips the layer in the job
    LayerMode    mode{LayerMode::Line};
    double       speed_mm_s{100};
    double       power_max{20};         // %
    double       power_min{10};         // %: corners/slow segments on M4 hardware; Grayscale white
    int          passes{1};
    double       interval_mm{0.1};      // Fill line interval (images: 25.4 / image_dpi instead)
    double       angle_deg{0};          // scan angle, from +X
    bool         bidirectional{true};
    bool         crosshatch{false};     // Fill: a second pass at angle + 90
    double       overscan_pct{2.5};     // overscan distance = speed_mm_s * overscan_pct / 100 mm ...
    double       overscan_mm{0};        // ... unless this is > 0
    FillGrouping fill_grouping{FillGrouping::AllAtOnce};
    double       kerf_offset_mm{0};     // Line: grows closed parts by this (outer out, holes in)
    double       lead_in_mm{0};         // Line, closed paths: approach length, 0 = off
    bool         tabs{false};           // Line, closed paths: leave uncut bridges
    int          tab_count{4};          // per path, used when tab_spacing_mm == 0
    double       tab_spacing_mm{0};     // > 0: one tab every this many mm instead of tab_count
    double       tab_size_mm{0.5};      // bridge length
    double       z_offset_mm{0};        // devices with enable_z only
    double       z_step_per_pass{0};    // lowered each pass (positive = down)
    bool         dot_mode{false};       // Line: fire dots instead of continuous cuts
    double       dot_dwell_ms{1};
    double       dot_spacing_mm{0.5};
    bool         air_assist{true};
    int          priority{0};           // JobSettings::OrderBy::Priority: lower runs first
    // Images
    DitherMode   image_dither{DitherMode::Jarvis};
    double       image_dpi{254};
    bool         image_negative{false};
    bool         image_pass_through{false}; // pixels are burnt as given (already dithered)
    double       image_cells_per_inch{50};  // Newsprint / Halftone screen
    double       image_screen_angle_deg{22.5};

    template<class Archive> void serialize(Archive& ar)
    {
        ar(name, visible, output, mode, speed_mm_s, power_max, power_min, passes, interval_mm, angle_deg,
           bidirectional, crosshatch, overscan_pct, overscan_mm, fill_grouping, kerf_offset_mm, lead_in_mm,
           tabs, tab_count, tab_spacing_mm, tab_size_mm, z_offset_mm, z_step_per_pass, dot_mode,
           dot_dwell_ms, dot_spacing_mm, air_assist, priority, image_dither, image_dpi, image_negative,
           image_pass_through, image_cells_per_inch, image_screen_angle_deg);
    }
};

// ---- Shapes ----------------------------------------------------------------------------------------

enum class ShapeType { Path, Rect, Ellipse, Polygon, Text, Image, Group };
enum class TextAlign { Left, Center, Right };

// One flattened polyline (Beziers flattened at import to 0.02 mm). Scaled, local frame. A closed
// path does not repeat its first point at the end. FROZEN.
struct LaserPath {
    Polyline pts;
    bool     closed{true};
    template<class Archive> void serialize(Archive& ar) { ar(pts.points, closed); }
};
using LaserPaths = std::vector<LaserPath>;

// Transform2d on the wire: its 6 affine coefficients (linear column-major, then translation).
struct Xform6 {
    Transform2d& t;
    template<class Archive> void serialize(Archive& ar)
    {
        double m[6] = {t.linear()(0, 0), t.linear()(1, 0), t.linear()(0, 1), t.linear()(1, 1),
                       t.translation().x(), t.translation().y()};
        ar(m[0], m[1], m[2], m[3], m[4], m[5]);
        t.linear() << m[0], m[2], m[1], m[3];
        t.translation() = Vec2d(m[4], m[5]);
    }
};

// One flat struct for every ShapeType (like CamOperation); each type reads the fields it needs.
// Local frames: Rect, Ellipse, Polygon and Image are centred on the local origin; Text sits on
// its first baseline at y = 0, aligned about x = 0 by `text_align`; Path is as imported/drawn.
struct LaserShape {
    ShapeType   type{ShapeType::Path};
    uint64_t    id{0};                  // unique in the document, assigned by LaserDocument
    std::string name;
    int         layer{0};               // 0..29; ignored for Group
    int         parent{-1};             // index of the Group in LaserDocument::shapes, -1 = top level
    bool        locked{false};
    bool        visible{true};
    Transform2d xform{Transform2d::Identity()};   // local -> workspace, absolute

    // Path
    LaserPaths  paths;
    // Rect
    double      width{10}, height{10};
    double      corner_radius{0};
    // Ellipse; Polygon (circumscribed ellipse + number of sides)
    double      rx{5}, ry{5};
    int         sides{6};
    // Text. `text_outlines` is a cache (TextLayout.hpp) that is also serialized, so a project
    // still shows its text on a machine that lacks the font.
    std::string text;                   // UTF-8, '\n' starts a new line
    std::string font;                   // family name or font file path
    double      text_height_mm{10};
    double      text_spacing_mm{0};     // extra letter spacing
    bool        bold{false}, italic{false};
    TextAlign   text_align{TextAlign::Left};
    Polygons    text_outlines;          // scaled, local frame
    // Image: 8-bit grayscale, row-major, row 0 = TOP row, 0 = black, 255 = white.
    int                  image_w{0}, image_h{0};
    std::vector<uint8_t> gray;
    double      width_mm{0}, height_mm{0};   // placed size before xform
    double      brightness{0};          // -100..100
    double      contrast{0};            // -100..100
    double      gamma{1};
    bool        invert{false};
    bool        dither_override{false}; // true: `dither` replaces the layer's image_dither
    DitherMode  dither{DitherMode::Jarvis};
    // Group: no own geometry; members are the shapes whose `parent` is this index.

    template<class Archive> void serialize(Archive& ar)
    {
        ar(type, id, name, layer, parent, locked, visible, Xform6{xform},
           paths, width, height, corner_radius, rx, ry, sides,
           text, font, text_height_mm, text_spacing_mm, bold, italic, text_align, text_outlines,
           image_w, image_h, gray, width_mm, height_mm, brightness, contrast, gamma, invert,
           dither_override, dither);
    }
};

// ---- Device ----------------------------------------------------------------------------------------

enum class DeviceType { Grbl, GrblHal, Marlin, Smoothie, Ruida };   // Ruida: not in v1 (error)
enum class LaserSource { Diode, CO2 };
enum class OriginCorner { FrontLeft, RearLeft, FrontRight, RearRight };
enum class RotaryType { Chuck, Roller };

// Rotary replaces the Y axis. Plan-time mapping of a workspace Y distance d (on the object surface):
//   Chuck:  d * mm_per_rotation / (pi * object_diameter)
//   Roller: d * mm_per_rotation / (pi * roller_diameter)   (the object's diameter cancels out)
struct RotarySettings {
    bool       enabled{false};
    RotaryType type{RotaryType::Roller};
    double     mm_per_rotation{40};    // Y travel the controller needs for one turn of the chuck/roller
    double     roller_diameter{20};
    double     object_diameter{60};
};

// Not stored in the document (the document names its device); a JSON library (Materials.hpp).
struct LaserDevice {
    std::string    name{"Generic GRBL diode"};
    DeviceType     type{DeviceType::Grbl};
    LaserSource    source{LaserSource::Diode};
    double         bed_w{400}, bed_h{400};
    OriginCorner   origin_corner{OriginCorner::FrontLeft};
    double         s_max{1000};             // GRBL $30
    bool           laser_mode_dynamic{true};   // M4 (needs $32=1); false: M3
    bool           uses_g0_for_travel{true};   // false: travel as G1 S0
    double         max_speed_mm_s{100};
    double         travel_speed_mm_s{100};
    double         accel_mm_s2{1000};       // time estimate only
    unsigned       baud{115200};
    bool           enable_z{false};
    double         frame_power_pct{0};      // > 0: frame with the laser at this power (diode "low fire")
    bool           marlin_fan_laser{false};    // Marlin: M106 S0..255 / M107 instead of M3/M4/M5
    bool           marlin_inline{false};       // Marlin: M3 I (inline laser power)
    RotarySettings rotary;
    std::string    start_gcode;
    std::string    end_gcode;
    std::string    air_on_gcode{"M8"};      // M7 on some machines
    std::string    air_off_gcode{"M9"};
};

// ---- Job settings (stored in the document) -----------------------------------------------------

struct JobSettings {
    // Absolute: workspace coords = machine coords. UserOrigin: the job-origin anchor goes to the
    // machine's work zero (G10 L20). CurrentPosition: the anchor goes to where the head is now.
    enum class StartFrom { Absolute, UserOrigin, CurrentPosition };
    enum class OrderBy { Layer, Priority, Groups };
    struct Optimize {
        bool    cut_inner_first{true};
        bool    reduce_travel{true};
        bool    reduce_direction_changes{false};
        OrderBy order_by{OrderBy::Layer};
    };

    StartFrom start_from{StartFrom::Absolute};
    // 9-point anchor of the job bounds, reading order as drawn: 0 = rear-left .. 8 = front-right
    // (6 = front-left). Unused for Absolute.
    int       job_origin{6};
    bool      cut_selected_only{false};
    bool      use_selection_origin{false};   // anchor from the selection bounds, not the job's
    Optimize  optimize;

    // One flat record: new fields (JobSettings or Optimize) go at the end of this list.
    template<class Archive> void serialize(Archive& ar)
    {
        ar(start_from, job_origin, cut_selected_only, use_selection_origin, optimize.cut_inner_first,
           optimize.reduce_travel, optimize.reduce_direction_changes, optimize.order_by);
    }
};

// ---- Raster (ImageEngrave.hpp) ---------------------------------------------------------------------

// Burn values after adjustments, resampling and dithering: 0 = off, 255 = full power_max
// (Grayscale: power_min + v/255 * (power_max - power_min)). Row 0 is the first scan line.
struct Raster {
    int                  w{0}, h{0};
    std::vector<uint8_t> burn;
    double               pixel_mm{0.1};    // along a scan line
    double               line_mm{0.1};     // between scan lines
    // Pixel (col, row) centre -> workspace mm, scan angle included.
    Transform2d          to_workspace{Transform2d::Identity()};
};

// One lit stretch along a scan line, in mm from ScanLine::start in the direction of travel.
struct Run {
    float x0{0}, x1{0};    // x0 < x1
    float power_pct{0};
};

// One pass of the head, workspace mm; start/end already include the overscan and bidirectional
// reversal. Runs are in travel order, equal-power neighbours merged, blank stretches dropped.
struct ScanLine {
    Vec2d            start{0, 0}, end{0, 0};
    std::vector<Run> runs;
};

// ---- Job (LaserPlan.hpp, never serialized) ---------------------------------------------------------

struct Segment {
    enum class Kind { Travel, Cut, Scan, Dwell };
    Kind   kind{Kind::Travel};
    Vec2d  from{0, 0}, to{0, 0};   // job frame (see LaserJob), rotary already mapped
    double speed_mm_s{0};
    double power_pct{0};           // Cut/Dwell; Scan: per run in LaserJob::scans[scan]
    double z{0};                   // absolute Z, devices with enable_z only
    int    layer{-1};
    bool   air_assist{false};
    int    scan{-1};               // Scan: index into LaserJob::scans (from/to = its start/end)
    double dwell_ms{0};            // Dwell (dot mode): fire in place at `to`
};

struct LaserJob {
    std::vector<Segment>              segments;
    // Scan lines stay compact (a dithered photo has millions of runs); Scan segments point here.
    std::vector<ScanLine>             scans;
    // Frame of every coordinate: Absolute = workspace; UserOrigin / CurrentPosition = workspace
    // shifted so the job-origin anchor is (0, 0). The G-code writer applies the origin corner.
    JobSettings::StartFrom            start_from{JobSettings::StartFrom::Absolute};
    BoundingBoxf                      bounds;          // job frame, every lit move incl. overscan
    double                            estimated_time_s{0};
    std::array<double, kLayerCount>   layer_time_s{};
    std::string                       error;           // non-empty: planning failed (plain language)
    std::vector<std::string>          warnings;
    bool ok() const { return error.empty(); }
};

// ---- Options ---------------------------------------------------------------------------------------

struct GCodeOptions {
    bool        comments{true};      // header with device, layers and time; per-layer comments
    int         decimals{3};         // X/Y digits
    std::string job_name{"shashimi"};
};

// Trace Image (LightBurn semantics). Pixels with cutoff <= value <= threshold are traced.
struct TraceOptions {
    int    cutoff{0};
    int    threshold{128};
    int    ignore_less_than_px{2};   // drop contours whose bounding box is smaller than this
    double smoothness_px{1.0};       // Douglas-Peucker tolerance, pixels
};

// Progress: fraction done (0..1); returning true cancels (the result then carries the error "Cancelled").
using ProgressFn = std::function<bool(double fraction)>;

} // namespace Slic3r::Laser

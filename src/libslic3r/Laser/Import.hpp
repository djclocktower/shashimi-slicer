#pragma once

// File import: SVG (NSVGUtils), DXF (own minimal reader), bitmaps (OpenCV), LightBurn .lbrn2.
// Best effort: unknown elements are skipped with a warning.

#include "libslic3r/Laser/LaserTypes.hpp"

#include <string>
#include <utility>
#include <vector>

namespace Slic3r::Laser {

// Shapes in workspace mm at the file's own positions (SVG / image rows flipped to Y up); the
// caller places them (e.g. centre on the bed). `parent` indexes `shapes` itself
// (LaserDocument::add_shapes remaps it). Layers: nearest palette colour (SVG stroke/fill colour,
// DXF ACI colour), a DXF/SVG layer named "C05" or "5" maps to that layer.
struct ImportResult {
    std::vector<LaserShape>                shapes;
    // .lbrn2 only: <CutSetting> per used layer index.
    std::vector<std::pair<int, LaserLayer>> cut_settings;
    std::vector<std::string>               warnings;   // plain language
    std::string                            error;      // non-empty: nothing imported
    bool ok() const { return error.empty(); }
};

// Filled shapes -> closed paths, stroked-only paths keep the SVG closed flag; Beziers flattened to
// 0.02 mm; units via NanoSVG (px at 96 dpi, mm, in ...). NanoSVG flattens groups, so a run of
// shapes sharing an id (a group id inherited by id-less children) comes back as one Group.
ImportResult import_svg(const std::string& path);
// LINE, LWPOLYLINE/POLYLINE (bulges), CIRCLE, ARC, ELLIPSE, SPLINE (de Boor), INSERT/BLOCK
// (recursive, as Groups), TEXT/MTEXT as Text shapes; $INSUNITS scales to mm (unitless = mm).
// Layer: a DXF layer named "C05"/"5", else the nearest palette colour of ACI 1..9, else ACI % 30.
ImportResult import_dxf(const std::string& path);
// One Image shape, grayscale, sized at `dpi` (files rarely carry a trustworthy one).
ImportResult import_image(const std::string& path, double dpi = 254);
// <Shape Type="Rect|Ellipse|Path|Text|Group|Bitmap"> with XForm; Path VertList/PrimList with line
// and Bezier prims; Bitmap base64 PNG; <CutSetting> into cut_settings.
ImportResult import_lbrn2(const std::string& path);
// Dispatch on the extension (.svg .dxf .lbrn2 .lbrn .png .jpg .jpeg .bmp .tif .tiff).
ImportResult import_file(const std::string& path);

} // namespace Slic3r::Laser

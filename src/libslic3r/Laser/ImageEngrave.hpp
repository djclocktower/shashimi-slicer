#pragma once

// Image engraving: adjustments, resampling (OpenCV), dithering, raster -> scan lines.

#include "libslic3r/Laser/LaserTypes.hpp"

namespace Slic3r::Laser {

// In place: brightness/contrast (-100..100), gamma, invert. `gray` is w*h, row-major.
void apply_adjustments(std::vector<uint8_t>& gray, int w, int h, double brightness, double contrast, double gamma, bool invert);

// Gray (0 black .. 255 white) -> burn values (0 off .. 255 full). Every mode but Grayscale returns
// only 0/255. Threshold at 128; Ordered = Bayer 8x8; FloydSteinberg / Jarvis / Stucki / Atkinson
// error diffusion; Newsprint = line screen, Halftone = dot screen, both `cells_per_inch` at
// `screen_angle_deg` given the pixel pitch `dpi`; Grayscale = 255 - gray. Also used by the GUI for
// the dither preview thumbnail.
std::vector<uint8_t> dither(const std::vector<uint8_t>& gray, int w, int h, DitherMode mode,
                            double dpi, double cells_per_inch, double screen_angle_deg);

// The Image shape ready to burn: adjustments, resampled to layer.image_dpi along and across the
// scan direction (layer.angle_deg, shape xform included), dithered with the shape's override or the
// layer's mode, negative applied; pass_through skips adjustments and dithering (gray < 128 burns).
// Empty raster when the shape is not an Image or has no pixels.
Raster prepare_image(const LaserShape& image, const LaserLayer& layer, const ProgressFn& progress = {});

// One ScanLine per raster row with lit pixels (blank rows skipped), runs of equal burn merged,
// power per run from the layer (Grayscale: power_min..power_max), overscan added at both ends,
// alternate rows reversed when layer.bidirectional. Lines are clipped to the device bed only by
// the planner.
std::vector<ScanLine> raster_to_scanlines(const Raster& raster, const LaserLayer& layer, const LaserDevice& device);

} // namespace Slic3r::Laser

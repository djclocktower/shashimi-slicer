#pragma once

// Trace Image: bitmap -> closed vector outlines (OpenCV threshold + findContours, Douglas-Peucker).

#include "libslic3r/Laser/LaserTypes.hpp"

namespace Slic3r::Laser {

// A Path shape (closed paths, holes included) in the image's LOCAL frame, same xform and layer
// as `image`, so it lands exactly on top of it. Adjustments of the shape are applied first.
// Empty `paths` when nothing passes the options or `image` is not an Image.
LaserShape trace_image(const LaserShape& image, const TraceOptions& opts);

} // namespace Slic3r::Laser

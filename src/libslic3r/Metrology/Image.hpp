#pragma once

// Grey-image operations for the Metrology kernel.

#include "MetrologyTypes.hpp"

#include <functional>

namespace Slic3r::Metrology {

// Separable Gaussian; sigma <= 0 returns a copy.
GrayImage gaussian_blur(const GrayImage& img, double sigma);

// a - b + offset, clamped to 0..255. With a light-on frame as `a` and a light-off frame as `b`
// this keeps only what the printer's own light lit, cancelling room light.
GrayImage subtract(const GrayImage& a, const GrayImage& b, float offset = 0.f);

// Mean of several frames of the same size (noise reduction). Empty input gives an empty image.
GrayImage average(const std::vector<GrayImage>& frames);

// Otsu threshold over the whole image (or the pixels where mask(x, y) is true).
double otsu_threshold(const GrayImage& img, const std::function<bool(int, int)>& mask = {});

// Mean absolute difference on a 4x subsampled grid: a cheap motion measure between frames.
double frame_motion(const GrayImage& a, const GrayImage& b);

// Resamples the image onto a mm grid: pixel (i, j) of the result is plate point
// (x0 + (i + 0.5) res, y0 + (j + 0.5) res), so row 0 is the LOWEST y (the result is stored Y up).
GrayImage rectify(const GrayImage& img, const std::function<P2(const P2&)>& mm_to_px, double x0, double y0, int w, int h, double res);

} // namespace Slic3r::Metrology

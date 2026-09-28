#include "libslic3r/Laser/Trace.hpp"
#include "libslic3r/Laser/ImageEngrave.hpp"
#include "libslic3r/ClipperUtils.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace Slic3r::Laser {

LaserShape trace_image(const LaserShape& image, const TraceOptions& opts)
{
    LaserShape out;
    out.type   = ShapeType::Path;
    out.name   = image.name.empty() ? "Trace" : image.name + " trace";
    out.layer  = image.layer;
    out.xform  = image.xform;
    if (image.type != ShapeType::Image || image.image_w <= 0 || image.image_h <= 0 ||
        image.gray.size() < size_t(image.image_w) * image.image_h || image.width_mm <= 0 || image.height_mm <= 0)
        return out;
    try {
        const int iw = image.image_w, ih = image.image_h;
        std::vector<uint8_t> g(image.gray.begin(), image.gray.begin() + size_t(iw) * ih);
        apply_adjustments(g, iw, ih, image.brightness, image.contrast, image.gamma, image.invert);

        cv::Mat mask;
        cv::inRange(cv::Mat(ih, iw, CV_8UC1, g.data()), cv::Scalar(std::clamp(opts.cutoff, 0, 255)),
                    cv::Scalar(std::clamp(opts.threshold, 0, 255)), mask);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i>              hier;   // next, prev, first child, parent
        cv::findContours(mask, contours, hier, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

        const double sx = image.width_mm / iw, sy = image.height_mm / ih;
        // Contour points are foreground pixel centres -> local mm (row 0 = top, Y up).
        auto to_polygon = [&](int i) {
            std::vector<cv::Point> c = contours[i];
            if (opts.smoothness_px > 0) cv::approxPolyDP(contours[i], c, opts.smoothness_px, true);
            Polygon p;
            p.points.reserve(c.size());
            for (const cv::Point& q : c)
                p.points.emplace_back(scale_(-image.width_mm / 2 + (q.x + 0.5) * sx), scale_(image.height_mm / 2 - (q.y + 0.5) * sy));
            return p;
        };
        auto small = [&](int i) {
            const cv::Rect bb = cv::boundingRect(contours[i]);
            return std::max(bb.width, bb.height) < opts.ignore_less_than_px;
        };

        ExPolygons ex;
        for (int i = 0; i < int(contours.size()); ++i) {
            if (hier[i][3] != -1 || small(i)) continue;   // RETR_CCOMP: top level = outer boundaries
            ExPolygon e;
            e.contour = to_polygon(i);
            e.contour.make_counter_clockwise();
            for (int h = hier[i][2]; h >= 0; h = hier[h][0])
                if (!small(h)) {
                    e.holes.push_back(to_polygon(h));
                    e.holes.back().make_clockwise();
                }
            ex.push_back(std::move(e));
        }
        // The contours run through the edge pixels' centres: grow by half a pixel to the real pixel
        // edge (holes shrink); the offset also unions everything into valid ExPolygons.
        ex = offset_ex(ex, float(scale_(0.25 * (sx + sy))));
        for (const ExPolygon& e : ex) {
            out.paths.push_back({Polyline(e.contour.points), true});
            for (const Polygon& h : e.holes) out.paths.push_back({Polyline(h.points), true});
        }
    } catch (...) {
        out.paths.clear();
    }
    return out;
}

} // namespace Slic3r::Laser

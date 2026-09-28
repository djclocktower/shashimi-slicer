#pragma once

// Job planning: document + device -> ordered segments with time estimate (LightBurn job order).

#include "libslic3r/Laser/LaserDocument.hpp"

namespace Slic3r::Laser {

// The whole job. Output layers only (visible shapes on layers with `output`), ordered by
// doc.job.optimize.order_by (Layer: index order; Priority: LaserLayer::priority then index;
// Groups: top-level groups in document order, layers inside each). Per layer and pass: Fill /
// OffsetFill before Line (FillLine), images as Scan segments, vectors with kerf, tabs, lead-in,
// dot mode, Z steps; paths ordered by optimize_order(). `selection` (document indices, GUI
// state) is used for cut_selected_only / use_selection_origin. Start position, job origin and
// the rotary mapping (LaserTypes.hpp) are applied; travel starts at the job-frame origin.
// Errors in LaserJob::error (empty job, Ruida device, shapes off the bed with Absolute).
LaserJob plan(const LaserDocument& doc, const LaserDevice& device, const std::vector<int>& selection = {},
              const ProgressFn& progress = {});

// Fills job.estimated_time_s and job.layer_time_s: trapezoidal speed profile per segment with
// device.accel_mm_s2, speeds capped at device.max_speed_mm_s (travel: travel_speed_mm_s), no
// junction blending, dwell times added. plan() already calls it.
void estimate_time(LaserJob& job, const LaserDevice& device);

} // namespace Slic3r::Laser

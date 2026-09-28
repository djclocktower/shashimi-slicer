#pragma once

// Umbrella header of the CAM kernel (namespace Slic3r::CAM). See README.md.

#include "libslic3r/CAM/CamTypes.hpp"
#include "libslic3r/CAM/CamDocument.hpp"
#include "libslic3r/CAM/Toolpath.hpp"
#include "libslic3r/CAM/FeedsSpeeds.hpp"
#include "libslic3r/CAM/ToolLibrary.hpp"
#include "libslic3r/CAM/Machines.hpp"
#include "libslic3r/CAM/Adaptive.hpp"
#include "libslic3r/CAM/Op2D.hpp"
#include "libslic3r/CAM/Drill.hpp"
#include "libslic3r/CAM/Op3D.hpp"
#include "libslic3r/CAM/Rotary.hpp"
#include "libslic3r/CAM/StockSim.hpp"
#include "libslic3r/CAM/Post.hpp"

namespace Slic3r::CAM {

// Toolpath of doc.operations[op_index], dispatched by OpType, with toolpath_stats/estimate_time
// applied. Errors (bad index, missing tool, disabled op, empty selection) come back in
// Toolpath::error in plain language ("Tool 6 mm cannot enter this 5 mm slot"). Does not touch
// doc.paths: the caller stores the result.
Toolpath generate_toolpath(const CamDocument& doc, int op_index, const CamModel& model);

} // namespace Slic3r::CAM

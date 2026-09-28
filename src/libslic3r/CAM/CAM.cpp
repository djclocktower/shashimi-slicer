#include "libslic3r/CAM/CAM.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>

namespace Slic3r::CAM {

static Toolpath dispatch(const CamDocument& doc, const CamOperation& op, const CamModel& model, const ProgressFn& progress)
{
    switch (op.type) {
    case OpType::Drill:
    case OpType::Bore: return generate_drill(doc, op, model, progress);
    case OpType::Adaptive3D: return generate_adaptive3d(doc, op, model, progress);
    case OpType::Parallel3D: return generate_parallel3d(doc, op, model, progress);
    case OpType::Contour3D: return generate_contour3d(doc, op, model, progress);
    case OpType::RotaryWrap: return generate_rotary_wrap(doc, op, model, progress);
    case OpType::RotaryFinish: return generate_rotary_finish(doc, op, model, progress);
    default: return generate_2d(doc, op, model, progress);
    }
}

Toolpath generate_toolpath(const CamDocument& doc, int op_index, const CamModel& model, const ProgressFn& progress)
{
    Toolpath tp;
    tp.generation = doc.model_generation;
    if (op_index < 0 || op_index >= int(doc.operations.size())) {
        tp.error = "The operation does not exist.";
        return tp;
    }
    const CamOperation& op = doc.operations[op_index];
    if (!op.enabled) {
        tp.error = "The operation is suppressed.";
        return tp;
    }
    if (op.setup_index < 0 || op.setup_index >= int(doc.setups.size())) {
        tp.error = "The operation's setup does not exist.";
        return tp;
    }
    const CamTool* tool = doc.find_tool(op.tool_number);
    if (!tool) {
        tp.error = "The operation's tool (T" + std::to_string(op.tool_number) + ") is not in the tool library.";
        return tp;
    }
    // One gate for every generator: a zero/NaN tool or pass size divides by zero somewhere below
    // (a corrupt project or a scripted edit can carry either).
    if (!(tool->diameter > 0) || !std::isfinite(tool->diameter)) {
        tp.error = "Tool T" + std::to_string(tool->number) + " has no diameter. Set its diameter in the Tool Library.";
        return tp;
    }
    for (double v : {op.stepover, op.stepdown, op.tolerance, op.optimal_load, op.a_stepover_deg, op.peck_depth,
                     op.stock_to_leave_radial, op.stock_to_leave_axial, op.lead_in_radius, op.helix_diameter})
        if (!std::isfinite(v)) {
            tp.error = "One of this operation's values is not a number. Open the operation and re-enter its values.";
            return tp;
        }

    // Generators may report from worker threads: serialise the callback, keep the fraction
    // monotonic, and remember a cancel so every later call answers "cancel" at once.
    std::mutex        mutex;
    std::atomic<bool> cancelled{false};
    double            last = 0;
    ProgressFn        guarded;
    if (progress)
        guarded = [&](double f) {
            if (cancelled)
                return true;
            std::lock_guard<std::mutex> lock(mutex);
            last = std::max(last, std::clamp(f, 0., 1.));
            if (progress(last))
                cancelled = true;
            return cancelled.load();
        };

    const CamSetup& setup = doc.setups[op.setup_index];
    tp                    = dispatch(doc, op, model, guarded);
    tp.generation         = doc.model_generation;
    if (cancelled) {
        Toolpath out;
        out.generation = doc.model_generation;
        out.error      = "Cancelled";
        return out;
    }
    // Indexed setup: every move of a non-rotary op sits at the index angle.
    if (op.type != OpType::RotaryWrap && op.type != OpType::RotaryFinish)
        for (Move& m : tp.moves)
            m.a_deg = setup.a_index_deg;
    if (tp.ok() && (op.type == OpType::Adaptive3D || op.type == OpType::Parallel3D || op.type == OpType::Contour3D))
        append(tp.warnings, gouge_check(tp, *tool, setup_mesh(doc, model, op.setup_index)));
    toolpath_stats(tp);
    tp.time_s = estimate_time(tp, find_machine(setup.machine));
    if (progress && !cancelled)
        progress(1.0);
    return tp;
}

} // namespace Slic3r::CAM

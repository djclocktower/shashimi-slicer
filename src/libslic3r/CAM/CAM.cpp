#include "libslic3r/CAM/CAM.hpp"

namespace Slic3r::CAM {

Toolpath generate_toolpath(const CamDocument& doc, int op_index, const CamModel& model)
{
    Toolpath tp;
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
    if (!doc.find_tool(op.tool_number)) {
        tp.error = "The operation's tool (T" + std::to_string(op.tool_number) + ") is not in the tool library.";
        return tp;
    }
    const CamSetup& setup  = doc.setups[op.setup_index];
    const bool      rotary = op.type == OpType::RotaryWrap || op.type == OpType::RotaryFinish;
    switch (op.type) {
    case OpType::Drill:
    case OpType::Bore: tp = generate_drill(doc, op, model); break;
    case OpType::Adaptive3D: tp = generate_adaptive3d(doc, op, model); break;
    case OpType::Parallel3D: tp = generate_parallel3d(doc, op, model); break;
    case OpType::Contour3D: tp = generate_contour3d(doc, op, model); break;
    case OpType::RotaryWrap: tp = generate_rotary_wrap(doc, op, model); break;
    case OpType::RotaryFinish: tp = generate_rotary_finish(doc, op, model); break;
    default: tp = generate_2d(doc, op, model); break;
    }
    // Indexed setup: every move of a non-rotary op sits at the index angle.
    if (!rotary)
        for (Move& m : tp.moves)
            m.a_deg = setup.a_index_deg;
    toolpath_stats(tp);
    tp.time_s = estimate_time(tp, find_machine(setup.machine));
    return tp;
}

} // namespace Slic3r::CAM

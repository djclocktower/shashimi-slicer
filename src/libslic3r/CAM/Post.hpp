#pragma once

// Post processor: CamDocument toolpaths -> G-code text.

#include "libslic3r/CAM/CamDocument.hpp"

#include <string>
#include <vector>

namespace Slic3r::CAM {

// G-code for doc.operations[op_indices] in that order, from doc.paths (the caller regenerates
// stale paths first). Header comment with the tool list and time estimate; safe start
// (G21 G90 G17 G54 G94, work offset per setup); tool changes (M6 Tn, or M0 pause with a message
// for ToolChange::ManualPause); spindle M3 Sn / M5; coolant; G43 Hn where the dialect supports
// it; arcs G2/G3 IJK or linearised (opts.arcs); A moves (G93 inverse time when opts.inverse_time
// and LinuxCNC/Fanuc); canned drill cycles G81/G83/G73/G85/G84 on LinuxCNC/Mach3/Fanuc, expanded
// to G0/G1 on Grbl/Marlin; G0 A<index> at the start of an indexed setup; M30.
// Empty string + `error` when an op has no valid path or names a missing tool.
std::string post_process(const CamDocument& doc, const std::vector<int>& op_indices,
                         const PostOptions& opts, std::string* error = nullptr);

// Options suited to `machine`'s dialect: arcs off for Marlin, line numbers for Fanuc, inverse
// time for LinuxCNC/Fanuc 4-axis machines.
PostOptions default_post_options(const MachineProfile& machine);

// Plain-language dialect name ("GRBL", "LinuxCNC", ...).
const char* dialect_name(PostDialect d);

bool dialect_has_canned_cycles(PostDialect d);   // LinuxCNC, Mach3, Fanuc
bool dialect_has_inverse_time(PostDialect d);    // LinuxCNC, Fanuc
bool dialect_has_tool_length_comp(PostDialect d); // G43: LinuxCNC, Mach3, Fanuc

} // namespace Slic3r::CAM

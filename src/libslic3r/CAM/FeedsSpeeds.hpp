#pragma once

// Beginner feeds & speeds: SFM/Vc and chipload tables per material x tool diameter class
// (constexpr data in FeedsSpeeds.cpp).

#include "libslic3r/CAM/CamTypes.hpp"

namespace Slic3r::CAM {

constexpr int kMaterialCount = int(Material::Wax) + 1;

// Plain-language name ("Aluminum", "Mild steel", ...); stable, used in the UI and the G-code header.
const char* material_name(Material m);

// rpm = Vc / (pi * D), clamped to [machine.min_rpm, machine.max_rpm]; feed = rpm * flutes *
// chipload (clamped to machine.max_feed_xy); plunge = 30-50 % of feed (by material, clamped to
// max_feed_z); ramp = 50 % of feed. Drill/Tap: feed is the Z feed (Tap: rpm * pitch).
FeedsSpeeds recommend_feeds(const CamTool& tool, Material material, const MachineProfile& machine);

// What an operation actually cuts with: op.feeds when !op.feeds_auto, else tool.feeds when
// tool.override_feeds, else recommend_feeds().
FeedsSpeeds effective_feeds(const CamOperation& op, const CamTool& tool, Material material,
                            const MachineProfile& machine);

} // namespace Slic3r::CAM

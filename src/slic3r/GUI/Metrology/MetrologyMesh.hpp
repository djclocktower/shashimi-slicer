#pragma once

// Metrology regions to libslic3r geometry: ExPolygons and extruded meshes for the plate.

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Metrology/Reverse.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <vector>

namespace Slic3r { namespace GUI {

ExPolygons to_expolygons(const std::vector<Metrology::Region>& regions);

// The regions as a prism from z0 to z1 (mm), closed and outward facing.
indexed_triangle_set extrude_regions(const std::vector<Metrology::Region>& regions, double z0, double z1);

// A reverse-engineered model as stacked slabs: level k spans from the level below it to its
// height, so the slabs share faces but never overlap.
TriangleMesh model_mesh(const Metrology::ReModel& model);

}} // namespace Slic3r::GUI

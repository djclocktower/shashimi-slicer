#pragma once

// OCCT-touching helpers of the CAM kernel. The only CAM header allowed to include OCCT.

#include "libslic3r/CAM/CamTypes.hpp"
#include "libslic3r/CAD/SketchEngine.hpp"   // SketchEntity, SketchPlane

#include <TopoDS_Shape.hxx>

#include <vector>

namespace Slic3r::CAM {

struct CamBodyShape {
    TopoDS_Shape shape;   // world frame
};

// Outline of a PLANAR face whose normal is parallel to setup Z, projected to setup XY (scaled).
// z_out = the face's setup-frame Z. Empty when the face is not planar or not facing +/-Z.
ExPolygons face_outline(const CamBodyShape& body, int face_id, const Transform3d& to_setup, double& z_out);

// One edge discretised to `tolerance` (mm) and projected to setup XY (scaled). z_out = the
// edge's lowest setup-frame Z. A closed edge (circle) returns a closed polyline (first == last).
Polyline edge_polyline(const CamBodyShape& body, int edge_id, const Transform3d& to_setup,
                       double tolerance, double& z_out);

// Holes from cylindrical faces (and circular edges bounding a planar face), in the frame given by
// `to_setup`. Every axis direction is returned; callers filter on HoleFeature::axis.
std::vector<HoleFeature> find_holes(const CamBodyShape& body, const Transform3d& to_setup);

// One CamBody per shape (body_id = index in `bodies`), each tessellated with
// SketchEngine::tessellate(shape, linear_deflection, angular_deflection). Sketches and setup
// frames are added separately (add_cam_sketch, update_setup_frames).
CamModel build_cam_model(const std::vector<TopoDS_Shape>& bodies, double linear_deflection,
                         double angular_deflection = 0.5);

// Resolves a sketch feature's entities into a CamSketch (closed profiles -> regions, open chains
// -> chains, Point entities -> points; construction geometry skipped) and appends it to the model.
void add_cam_sketch(CamModel& model, int feature_index, const std::vector<SketchEntity>& entities,
                    const SketchPlane& plane);

} // namespace Slic3r::CAM

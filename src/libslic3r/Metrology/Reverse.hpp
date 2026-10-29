#pragma once

// Reverse engineering: the camera's traced outline plus nozzle-touch heights become a stepped
// (2.5D) model.
//
// The session runs in rounds, because the printer has no live readback: every round is a batch of
// touches that is run (on the printer or in the simulator) and whose heights are submitted back.
//  1. Survey: one touch deep inside the outline gives a first height, so the outline can be
//     corrected for parallax before the grid is placed.
//  2. Grid: touches on a regular grid inside the outline. Heights are clustered into levels.
//  3. Edges: wherever two neighbouring grid touches landed on different levels, touches between
//     them narrow down where the step is, a few per round, until it is bracketed to target_res.
//     The same rounds bracket a few points of the outer outline (anchors): comparing them with the
//     camera's outline measures the camera's edge bias, which build() removes.
// build() then corrects the outline for parallax with the local heights, corrects every step for
// the nozzle-face radius, and returns one set of regions per level.

#include "Calibration.hpp"
#include "Contour.hpp"
#include "MetrologyTypes.hpp"

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace Slic3r::Metrology {

struct ReSettings
{
    double grid             = 5.0;    // probe grid pitch, mm
    double edge_margin      = 0.8;    // extra clearance from the outline beyond the tip radius
    double tip_radius       = 0.5;    // flat nozzle face radius
    double level_tol        = 0.3;    // heights closer than this belong to one level
    double target_res       = 0.1;    // a step is done when bracketed this tightly
    int    touches_per_edge = 3;      // per round, per step crossing
    int    max_rounds       = 12;
    double raster           = 0.1;    // model raster
    double simplify         = 0.04;
    double min_area         = 0.5;    // mm², smaller islands are dropped
    double mat_level        = 0.2;    // touches below this hit the mat, not the part
    int    anchors          = 4;      // outline points probed to measure the camera's edge bias (0: none)
};

struct ReTouch
{
    int         id = 0;
    P2          xy;
    std::string label;
};

struct ReResult
{
    int    id = 0;
    double z  = 0;
};

struct ReLevel
{
    double              z       = 0;
    double              stddev  = 0;
    int                 samples = 0;
    std::vector<Region> regions;   // where the part is at least this tall
};

struct ReModel
{
    std::vector<Region>      footprint;
    double                   camera_bias = 0;   // how much larger the camera saw each edge, mm
    int                      anchors     = 0;   // probe-measured outline points behind camera_bias
    std::vector<ReLevel>     levels;   // ascending z
    std::vector<std::string> warnings;
};

class ReSession
{
public:
    enum class Stage { Idle, Survey, Grid, Edges, Done };

    // `silhouette` is the outline traced on the mat plane (z = 0). Without a camera centre the
    // outline is not corrected for parallax (a warning says so).
    void begin(std::vector<Region> silhouette, const CameraModel& cam, const ReSettings& settings);
    void reset() { *this = ReSession(); }

    Stage stage() const { return m_stage; }
    // Creates the next batch (also used for the first one). Empty when the session is done.
    std::vector<ReTouch> next_batch();
    // The batch waiting for results.
    const std::vector<ReTouch>& pending() const { return m_pending; }
    // Results for the pending batch (ids that are not pending are ignored).
    void submit(const std::vector<ReResult>& results);

    int touches_done() const { return m_touches_done; }
    int rounds() const { return m_rounds; }
    // Rough count of touches still to come (for the time estimate).
    int estimate_remaining() const;

    // ---- Inspection for drawing --------------------------------------------------------------
    struct Node
    {
        P2     pos;
        bool   active   = false;
        bool   measured = false;
        double z        = 0;
        int    level    = -1;
    };
    struct Crossing
    {
        P2     p;            // current estimate, before the tip correction
        double width = 0;    // bracket width, mm
    };
    const std::vector<Node>& nodes() const { return m_nodes; }
    std::vector<Crossing>    crossings() const;
    std::vector<double>      level_heights() const;
    const std::vector<Region>& silhouette() const { return m_silhouette; }
    const ReSettings&        settings() const { return m_set; }

    ReModel build() const;

private:
    struct EdgeItem
    {
        int a = 0, b = 0;                         // node indices
        std::map<int, std::pair<double, double>> brk;   // threshold -> (t on a's side, t on b's side)
    };
    struct PendingRef
    {
        int    edge   = -1;   // -1: a grid node or the survey
        int    node   = -1;
        int    anchor = -1;
        double t      = 0;
    };
    // A line across the outer outline, from inside (t = 0) to outside (t = 1).
    struct Anchor
    {
        P2     a, b;
        double ta = 0, tb = 1;   // last t found on the part / off it
        double len() const { return (b - a).norm(); }
    };

    void   cluster_levels();
    int    classify(double z) const;   // nearest level, -1 below the lowest
    bool   needs_refine(const EdgeItem& e, int k) const;
    double edge_len(const EdgeItem& e) const { return (m_nodes[size_t(e.b)].pos - m_nodes[size_t(e.a)].pos).norm(); }
    // The part's footprint from the silhouette, given its height at each plate point (soft field
    // over lo..hi). Seen at an angle the silhouette is the union of the part's slices projected
    // onto the mat, so the footprint is where both the bottom slice and the top slice are inside it.
    Field footprint_field(const std::vector<Region>& silhouette, const std::function<double(const P2&)>& height, const P2& lo,
                          const P2& hi) const;
    std::vector<Region> outline_with_height(double h) const;
    int    node_index(int i, int j) const { return j * m_nx + i; }

    Stage                 m_stage = Stage::Idle;
    ReSettings            m_set;
    CameraModel           m_cam;
    std::vector<Region>   m_silhouette;
    std::vector<Region>   m_outline_est;   // silhouette corrected with the survey height
    double                m_survey_z = 0;
    P2                    m_origin;
    int                   m_nx = 0, m_ny = 0;
    std::vector<Node>     m_nodes;
    std::vector<EdgeItem> m_edges;
    std::vector<Anchor>   m_anchors;
    int                   m_anchor_count = 4;
    std::vector<double>   m_levels;        // cluster centres
    std::vector<std::vector<double>> m_level_samples;
    std::vector<ReTouch>  m_pending;
    std::map<int, PendingRef> m_refs;
    int                   m_next_id = 1;
    int                   m_touches_done = 0;
    int                   m_rounds       = 0;
};

} // namespace Slic3r::Metrology

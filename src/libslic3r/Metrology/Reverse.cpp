#include "Reverse.hpp"

#include "Contour.hpp"
#include "Geometry.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <numeric>

namespace Slic3r::Metrology {

namespace {

// Which side of the line a -> b the point x lies on.
bool side(const P2& a, const P2& b, const P2& x) { return (b - a).cross(x - a) > 0; }

// State of point q inside a unit cell from its corner states and the step crossings on its edges
// (marching-squares rules). Corners c0 (0,0), c1 (1,0), c2 (1,1), c3 (0,1); edge k joins
// corner k with corner k+1 (e0 bottom, e1 right, e2 top, e3 left).
bool cell_state(const bool s[4], const P2 cr[4], bool centre, const P2& q)
{
    const int cnt = int(s[0]) + int(s[1]) + int(s[2]) + int(s[3]);
    if (cnt == 0) return false;
    if (cnt == 4) return true;
    static const P2 corner[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    // Corner k is cut off by the segment between the crossings on edges k-1 and k.
    auto on_corner_side = [&](int k) {
        const P2& a = cr[(k + 3) % 4];
        const P2& b = cr[k];
        return side(a, b, q) == side(a, b, corner[k]);
    };
    if (cnt == 1 || cnt == 3) {
        int k = 0;
        for (int i = 0; i < 4; ++i)
            if (s[i] == (cnt == 1)) k = i;
        return on_corner_side(k) ? s[k] : !s[k];
    }
    if (s[0] == s[2]) {   // saddle
        if (centre == s[0]) {
            if (on_corner_side(1)) return s[1];
            if (on_corner_side(3)) return s[3];
        } else {
            if (on_corner_side(0)) return s[0];
            if (on_corner_side(2)) return s[2];
        }
        return centre;
    }
    if (s[0] == s[1])   // bottom row vs top row: crossings on the left and right edges
        return side(cr[3], cr[1], q) == side(cr[3], cr[1], corner[0]) ? s[0] : s[3];
    return side(cr[0], cr[2], q) == side(cr[0], cr[2], corner[0]) ? s[0] : s[1];
}

} // namespace

void ReSession::begin(std::vector<Region> silhouette, const CameraModel& cam, const ReSettings& settings)
{
    *this         = ReSession();
    m_set         = settings;
    m_cam         = cam;
    m_silhouette  = std::move(silhouette);
    m_outline_est = m_silhouette;
    m_stage       = m_silhouette.empty() ? Stage::Done : Stage::Survey;
}

std::vector<ReTouch> ReSession::next_batch()
{
    if (!m_pending.empty()) return m_pending;
    m_refs.clear();
    auto add = [&](const P2& p, std::string label, PendingRef ref) {
        ReTouch t{m_next_id++, p, std::move(label)};
        m_refs[t.id] = ref;
        m_pending.push_back(t);
    };
    if (m_stage == Stage::Survey) {
        // The point deepest inside the outline.
        P2 lo, hi;
        bounding_box(m_silhouette, lo, hi);
        const double step = std::max(0.25, std::min(hi.x - lo.x, hi.y - lo.y) / 60.0);
        P2           best = (lo + hi) * 0.5;
        double       bd   = -1e9;
        for (double y = lo.y; y <= hi.y; y += step)
            for (double x = lo.x; x <= hi.x; x += step) {
                const double d = signed_distance(m_silhouette, {x, y});
                if (d > bd) {
                    bd   = d;
                    best = {x, y};
                }
            }
        add(best, "survey", {});
    } else if (m_stage == Stage::Grid) {
        P2 lo, hi;
        bounding_box(m_outline_est, lo, hi);
        const double clear = m_set.tip_radius + m_set.edge_margin;
        double       g     = m_set.grid;
        for (int attempt = 0; attempt < 4; ++attempt, g *= 0.5) {
            m_nodes.clear();
            m_origin = lo;
            m_nx     = std::max(2, int(std::ceil((hi.x - lo.x) / g)) + 1);
            m_ny     = std::max(2, int(std::ceil((hi.y - lo.y) / g)) + 1);
            // Centre the grid on the box.
            m_origin = P2{(lo.x + hi.x - (m_nx - 1) * g) / 2, (lo.y + hi.y - (m_ny - 1) * g) / 2};
            int active = 0;
            for (int j = 0; j < m_ny; ++j)
                for (int i = 0; i < m_nx; ++i) {
                    Node n;
                    n.pos    = m_origin + P2{i * g, j * g};
                    n.active = signed_distance(m_outline_est, n.pos) >= clear;
                    active += n.active;
                    m_nodes.push_back(n);
                }
            if (active >= 4 || attempt == 3) break;
        }
        m_set.grid = g;
        for (int j = 0; j < m_ny; ++j)
            for (int i = 0; i < m_nx; ++i) {
                const Node& n = m_nodes[size_t(node_index(i, j))];
                if (n.active) add(n.pos, "grid " + std::to_string(i) + "," + std::to_string(j), {-1, node_index(i, j), -1, 0.0});
            }
        if (m_pending.empty()) m_stage = Stage::Done;
    } else if (m_stage == Stage::Edges) {
        for (size_t ei = 0; ei < m_edges.size(); ++ei) {
            const EdgeItem&     e = m_edges[ei];
            std::vector<double> ts;
            for (const auto& kv : e.brk) {
                if (!needs_refine(e, kv.first)) continue;
                const double a = kv.second.first, b = kv.second.second;
                for (int q = 1; q <= m_set.touches_per_edge; ++q) ts.push_back(a + (b - a) * q / (m_set.touches_per_edge + 1));
            }
            std::sort(ts.begin(), ts.end());
            const double len = edge_len(e);
            double       last = -1;
            for (double t : ts) {
                if (last >= 0 && (t - last) * len < 0.02) continue;
                last         = t;
                const P2 pa  = m_nodes[size_t(e.a)].pos, pb = m_nodes[size_t(e.b)].pos;
                add(pa + (pb - pa) * t, "step " + std::to_string(ei), {int(ei), -1, -1, t});
            }
        }
        for (size_t ai = 0; ai < m_anchors.size(); ++ai) {
            const Anchor& an = m_anchors[ai];
            if ((an.tb - an.ta) * an.len() <= m_set.target_res) continue;
            for (int q = 1; q <= m_set.touches_per_edge; ++q) {
                const double t = an.ta + (an.tb - an.ta) * q / (m_set.touches_per_edge + 1);
                PendingRef   ref;
                ref.anchor = int(ai);
                ref.t      = t;
                add(an.a + (an.b - an.a) * t, "outline " + std::to_string(ai), ref);
            }
        }
        if (m_pending.empty()) m_stage = Stage::Done;
    }
    return m_pending;
}

void ReSession::cluster_levels()
{
    std::vector<double> zs;
    for (const Node& n : m_nodes)
        if (n.active && n.measured) zs.push_back(n.z);
    std::sort(zs.begin(), zs.end());
    m_levels.clear();
    m_level_samples.clear();
    for (size_t i = 0; i < zs.size(); ++i) {
        if (i == 0 || zs[i] - zs[i - 1] > m_set.level_tol) m_level_samples.emplace_back();
        m_level_samples.back().push_back(zs[i]);
    }
    for (const auto& s : m_level_samples) m_levels.push_back(std::accumulate(s.begin(), s.end(), 0.0) / double(s.size()));
}

int ReSession::classify(double z) const
{
    if (m_levels.empty() || z < m_set.mat_level) return -1;
    if (z < m_levels.front() - m_set.level_tol) return -1;
    int    best = 0;
    double bd   = std::abs(z - m_levels[0]);
    for (size_t k = 1; k < m_levels.size(); ++k)
        if (std::abs(z - m_levels[k]) < bd) {
            bd   = std::abs(z - m_levels[k]);
            best = int(k);
        }
    return best;
}

bool ReSession::needs_refine(const EdgeItem& e, int k) const
{
    auto it = e.brk.find(k);
    if (it == e.brk.end()) return false;
    return (it->second.second - it->second.first) * edge_len(e) > m_set.target_res;
}

void ReSession::submit(const std::vector<ReResult>& results)
{
    if (m_pending.empty()) return;
    std::map<int, double> got;
    for (const ReResult& r : results)
        if (m_refs.count(r.id)) got[r.id] = r.z;
    m_touches_done += int(got.size());

    if (m_stage == Stage::Survey) {
        m_survey_z = got.empty() ? 0.0 : got.begin()->second;
        // A survey that hit the mat leaves the outline uncorrected.
        if (m_survey_z >= m_set.mat_level) m_outline_est = outline_with_height(m_survey_z);
        m_stage = Stage::Grid;
    } else if (m_stage == Stage::Grid) {
        for (const auto& kv : got) {
            Node& n = m_nodes[size_t(m_refs[kv.first].node)];
            n.measured = true;
            n.z        = kv.second;
            if (n.z < m_set.mat_level) n.active = false;   // landed beside the part
        }
        cluster_levels();
        for (Node& n : m_nodes) n.level = (n.active && n.measured) ? classify(n.z) : -1;
        // A crossing for every threshold between neighbouring levels.
        for (int j = 0; j < m_ny; ++j)
            for (int i = 0; i < m_nx; ++i) {
                const int a = node_index(i, j);
                for (int dir = 0; dir < 2; ++dir) {
                    const int ii = i + (dir == 0), jj = j + (dir == 1);
                    if (ii >= m_nx || jj >= m_ny) continue;
                    const int b = node_index(ii, jj);
                    const Node &na = m_nodes[size_t(a)], &nb = m_nodes[size_t(b)];
                    if (na.level < 0 || nb.level < 0 || na.level == nb.level) continue;
                    EdgeItem e;
                    e.a = a;
                    e.b = b;
                    for (int k = std::min(na.level, nb.level) + 1; k <= std::max(na.level, nb.level); ++k) e.brk[k] = {0.0, 1.0};
                    m_edges.push_back(e);
                }
            }
        // Anchors across the middle of the outline's longest straight runs.
        if (m_set.anchors > 0 && !m_outline_est.empty()) {
            std::vector<std::pair<double, size_t>> runs;
            const Polygon2& loop = m_outline_est.front().outer;
            for (size_t k = 0; k < loop.size(); ++k) runs.push_back({(loop[(k + 1) % loop.size()] - loop[k]).norm(), k});
            std::sort(runs.begin(), runs.end(), [](auto& x, auto& y) { return x.first > y.first; });
            for (size_t r = 0; r < runs.size() && int(m_anchors.size()) < m_set.anchors; ++r) {
                if (runs[r].first < 3 * m_set.tip_radius + 2) break;
                const P2 p0 = loop[runs[r].second], p1 = loop[(runs[r].second + 1) % loop.size()];
                const P2 mid = (p0 + p1) * 0.5;
                const P2 out = P2{(p1 - p0).y, -(p1 - p0).x}.normalized();
                m_anchors.push_back({mid - out * 2.0, mid + out * 2.0});
            }
        }
        m_stage = (m_edges.empty() && m_anchors.empty()) ? Stage::Done : Stage::Edges;
    } else if (m_stage == Stage::Edges) {
        for (const auto& kv : got) {
            const PendingRef& ref = m_refs[kv.first];
            if (ref.anchor >= 0) {
                Anchor& an = m_anchors[size_t(ref.anchor)];
                if (ref.t > an.ta && ref.t < an.tb) {
                    if (kv.second >= m_set.mat_level) an.ta = ref.t;
                    else an.tb = ref.t;
                }
                continue;
            }
            if (ref.edge < 0) continue;
            EdgeItem& e = m_edges[size_t(ref.edge)];
            const int L = classify(kv.second);
            if (L >= 0 && std::abs(kv.second - m_levels[size_t(L)]) < m_set.level_tol * 0.5)
                m_level_samples[size_t(L)].push_back(kv.second);
            const int la = m_nodes[size_t(e.a)].level;
            for (auto& br : e.brk) {
                const int  k  = br.first;
                const bool sa = la >= k, s = L >= k;
                double&    ta = br.second.first;
                double&    tb = br.second.second;
                if (ref.t <= ta || ref.t >= tb) continue;   // outside the bracket: ignore
                if (s == sa) ta = ref.t;
                else tb = ref.t;
            }
        }
        ++m_rounds;
        bool more = false;
        for (const EdgeItem& e : m_edges)
            for (const auto& kv : e.brk) more = more || needs_refine(e, kv.first);
        for (const Anchor& an : m_anchors) more = more || (an.tb - an.ta) * an.len() > m_set.target_res;
        if (!more || m_rounds >= m_set.max_rounds) m_stage = Stage::Done;
    }
    m_pending.clear();
    m_refs.clear();
}

int ReSession::estimate_remaining() const
{
    switch (m_stage) {
    case Stage::Survey: return 1 + int(m_nodes.size());
    case Stage::Grid: {
        int n = 0;
        for (const Node& nd : m_nodes) n += nd.active;
        return std::max(n, 1);
    }
    case Stage::Edges: {
        int n = 0;
        for (const EdgeItem& e : m_edges)
            for (const auto& kv : e.brk) {
                const double w = (kv.second.second - kv.second.first) * edge_len(e);
                if (w <= m_set.target_res) continue;
                const double shrink = m_set.touches_per_edge + 1.0;
                n += m_set.touches_per_edge * int(std::ceil(std::log(w / m_set.target_res) / std::log(shrink)));
            }
        for (const Anchor& an : m_anchors) {
            const double w = (an.tb - an.ta) * an.len();
            if (w > m_set.target_res)
                n += m_set.touches_per_edge * int(std::ceil(std::log(w / m_set.target_res) / std::log(m_set.touches_per_edge + 1.0)));
        }
        return n;
    }
    default: return 0;
    }
}

std::vector<ReSession::Crossing> ReSession::crossings() const
{
    std::vector<Crossing> out;
    for (const EdgeItem& e : m_edges)
        for (const auto& kv : e.brk) {
            const P2     pa = m_nodes[size_t(e.a)].pos, pb = m_nodes[size_t(e.b)].pos;
            const double t  = 0.5 * (kv.second.first + kv.second.second);
            out.push_back({pa + (pb - pa) * t, (kv.second.second - kv.second.first) * edge_len(e)});
        }
    return out;
}

std::vector<double> ReSession::level_heights() const
{
    std::vector<double> h;
    for (const auto& s : m_level_samples) h.push_back(s.empty() ? 0.0 : std::accumulate(s.begin(), s.end(), 0.0) / double(s.size()));
    return h;
}

Field ReSession::footprint_field(const std::vector<Region>& silhouette, const std::function<double(const P2&)>& height, const P2& lo,
                                 const P2& hi) const
{
    const double res = m_set.raster;
    const int    W   = std::max(1, int(std::ceil((hi.x - lo.x) / res)));
    const int    Hh  = std::max(1, int(std::ceil((hi.y - lo.y) / res)));
    const Field  S   = rasterize(silhouette, lo.x, lo.y, W, Hh, res);
    if (!m_cam.has_center || m_cam.cz <= 0) return S;
    Field F = S;
    for (int j = 0; j < Hh; ++j)
        for (int i = 0; i < W; ++i) {
            if (S.at(i, j) <= 0) continue;
            const P2     p = S.pos(i, j);
            const double h = std::min(std::max(height(p), 0.0), 0.9 * m_cam.cz);
            if (h <= 0) continue;
            // Where the top slice of this point appears on the mat.
            const P2 q = m_cam.nadir + (p - m_cam.nadir) * (m_cam.cz / (m_cam.cz - h));
            F.at(i, j) = std::min(S.at(i, j), S.sample(q));
        }
    return F;
}

std::vector<Region> ReSession::outline_with_height(double h) const
{
    P2 lo, hi;
    bounding_box(m_silhouette, lo, hi);
    const P2 m{2 * m_set.raster, 2 * m_set.raster};
    const Field F = footprint_field(m_silhouette, [h](const P2&) { return h; }, lo - m, hi + m);
    return contour_regions(F, 0.5f, true, m_set.simplify, m_set.min_area);
}

ReModel ReSession::build() const
{
    ReModel model;
    const std::vector<double> H = level_heights();
    if (!m_cam.has_center) model.warnings.push_back("The outline is not corrected for parallax: calibrate the camera height first.");

    // Levels for every node: measured ones as they are, the rest from the nearest measured node.
    std::vector<int> lvl(m_nodes.size(), -1);
    std::deque<int>  q;
    for (size_t i = 0; i < m_nodes.size(); ++i)
        if (m_nodes[i].level >= 0) {
            lvl[i] = m_nodes[i].level;
            q.push_back(int(i));
        }
    while (!q.empty()) {
        const int c = q.front();
        q.pop_front();
        const int ci = c % m_nx, cj = c / m_nx;
        const int nb[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (auto& o : nb) {
            const int ni = ci + o[0], nj = cj + o[1];
            if (ni < 0 || nj < 0 || ni >= m_nx || nj >= m_ny) continue;
            const int id = node_index(ni, nj);
            if (lvl[size_t(id)] >= 0) continue;
            lvl[size_t(id)] = lvl[size_t(c)];
            q.push_back(id);
        }
    }

    if (H.empty()) {
        model.footprint = m_outline_est;
        if (m_survey_z >= m_set.mat_level) {
            ReLevel l;
            l.z       = m_survey_z;
            l.samples = 1;
            l.regions = model.footprint;
            model.levels.push_back(l);
            model.warnings.push_back("Only the survey height is known: the model is a single flat level.");
        }
        return model;
    }
    // Step crossings as (item, threshold) -> t. They are where the nozzle face's rim met the step,
    // so the regions they bound are the true ones grown by the tip radius; build shrinks them back.
    const double g = m_set.grid;
    std::map<std::pair<int, int>, int> edge_of;   // (a, b) -> item index
    for (size_t i = 0; i < m_edges.size(); ++i) edge_of[{m_edges[i].a, m_edges[i].b}] = int(i);
    std::map<std::pair<int, int>, double> raw_t;
    for (size_t i = 0; i < m_edges.size(); ++i)
        for (const auto& kv : m_edges[i].brk) {
            raw_t[{int(i), kv.first}] = 0.5 * (kv.second.first + kv.second.second);
            if ((kv.second.second - kv.second.first) * edge_len(m_edges[i]) > m_set.target_res * 1.01)
                if (model.warnings.empty() || model.warnings.back().rfind("Some steps", 0) != 0)
                    model.warnings.push_back("Some steps were not narrowed down to the target before the round limit.");
        }
    auto measured_t = [&](int a, int b, int k, double& t) {
        auto it = edge_of.find({a, b});
        if (it == edge_of.end()) return false;
        auto ct = raw_t.find({it->second, k});
        if (ct == raw_t.end()) return false;
        t = ct->second;
        return true;
    };
    // Crossing parameter on the grid edge a -> b for threshold k. Edges without a measured
    // crossing (next to the outline, where there are no touches) continue the step from the
    // parallel edges beside them; failing that, the middle.
    auto t_on = [&](int a, int b, int k) {
        double t = 0.5;
        if (measured_t(a, b, k, t)) return t;
        const int  ai = a % m_nx, aj = a / m_nx;
        const bool horizontal = (b == a + 1);
        for (int side : {-1, 1}) {
            double t1 = 0, t2 = 0;
            auto shifted = [&](int d, double& tt) {
                const int ni = horizontal ? ai : ai + side * d, nj = horizontal ? aj + side * d : aj;
                if (ni < 0 || nj < 0 || ni >= m_nx || nj >= m_ny) return false;
                const int na = node_index(ni, nj);
                const int nb = horizontal ? na + 1 : na + m_nx;
                if ((horizontal && ni + 1 >= m_nx) || (!horizontal && nj + 1 >= m_ny)) return false;
                // Only where the neighbour's corners are on the same sides of the step.
                if ((lvl[size_t(na)] >= k) != (lvl[size_t(a)] >= k) || (lvl[size_t(nb)] >= k) != (lvl[size_t(b)] >= k)) return false;
                return measured_t(na, nb, k, tt);
            };
            if (shifted(1, t1)) {
                t = shifted(2, t2) ? 2 * t1 - t2 : t1;
                return std::min(1.0, std::max(0.0, t));
            }
        }
        return 0.5;
    };
    // At least level k at plate point p, from the grid cell that contains it.
    auto level_inside = [&](const P2& p, int k) {
        if (k <= 0) return true;
        const double fx = (p.x - m_origin.x) / g, fy = (p.y - m_origin.y) / g;
        const int    ci = std::min(m_nx - 2, std::max(0, int(std::floor(fx))));
        const int    cj = std::min(m_ny - 2, std::max(0, int(std::floor(fy))));
        const P2     qn{std::min(1.0, std::max(0.0, fx - ci)), std::min(1.0, std::max(0.0, fy - cj))};
        const int n0 = node_index(ci, cj), n1 = node_index(ci + 1, cj), n2 = node_index(ci + 1, cj + 1), n3 = node_index(ci, cj + 1);
        const bool st[4] = {lvl[size_t(n0)] >= k, lvl[size_t(n1)] >= k, lvl[size_t(n2)] >= k, lvl[size_t(n3)] >= k};
        const P2   cr[4] = {{t_on(n0, n1, k), 0}, {1, t_on(n1, n2, k)}, {t_on(n3, n2, k), 1}, {0, t_on(n0, n3, k)}};
        double zc = 0;
        for (int c : {n0, n1, n2, n3}) zc += (lvl[size_t(c)] >= 0 ? H[size_t(lvl[size_t(c)])] : 0.0);
        return cell_state(st, cr, zc / 4 >= 0.5 * (H[size_t(k - 1)] + H[size_t(k)]), qn);
    };
    auto height_at = [&](const P2& p) {
        for (int k = int(H.size()) - 1; k > 0; --k)
            if (level_inside(p, k)) return H[size_t(k)];
        return H[0];
    };

    // Rasterize the footprint and each level over the silhouette's box.
    P2 lo, hi;
    bounding_box(m_silhouette, lo, hi);
    const double res = m_set.raster;
    lo               = lo - P2{2 * res, 2 * res};
    hi               = hi + P2{2 * res, 2 * res};
    Field fp = footprint_field(m_silhouette, height_at, lo, hi);
    // Camera edge bias from the anchors: the probe found the edge where the nozzle face's rim met
    // it, a tip radius outside; the camera's footprint edge is where the field crosses 0.5.
    std::vector<double> bias;
    for (const Anchor& an : m_anchors) {
        if ((an.tb - an.ta) * an.len() > 4 * m_set.target_res) continue;
        const double s_probe = 0.5 * (an.ta + an.tb) * an.len() - m_set.tip_radius;
        double       s_cam   = -1;
        const int    steps   = int(an.len() / (0.1 * res)) + 1;
        for (int q = 1; q <= steps && s_cam < 0; ++q) {
            const double s0 = (q - 1) * an.len() / steps, s1 = q * an.len() / steps;
            const P2     u  = (an.b - an.a) / an.len();
            const double f0 = fp.sample(an.a + u * s0), f1 = fp.sample(an.a + u * s1);
            if (f0 >= 0.5 && f1 < 0.5) s_cam = s0 + (s1 - s0) * (f0 - 0.5) / std::max(f0 - f1, 1e-9);
        }
        if (s_cam >= 0) bias.push_back(s_cam - s_probe);
    }
    if (!bias.empty()) {
        std::sort(bias.begin(), bias.end());
        model.camera_bias = bias[bias.size() / 2];
        model.anchors     = int(bias.size());
        if (std::abs(model.camera_bias) > 0.005)
            fp = footprint_field(offset_regions(m_silhouette, -model.camera_bias), height_at, lo, hi);
    } else if (m_set.anchors > 0)
        model.warnings.push_back("No outline anchor was resolved: the camera's edge bias is not corrected.");
    const int W = fp.w, Hh = fp.h;
    for (size_t k = 0; k < H.size(); ++k) {
        ReLevel L;
        L.z       = H[k];
        L.samples = int(m_level_samples[k].size());
        double var = 0;
        for (double z : m_level_samples[k]) var += (z - L.z) * (z - L.z);
        L.stddev = L.samples > 1 ? std::sqrt(var / (L.samples - 1)) : 0.0;
        Field mask = fp;
        if (k > 0) {
            auto  inside_k = [&](const P2& p) { return level_inside(p, int(k)); };
            // Wider than the footprint's box, so shrinking does not pull the level off the box edge.
            const double m2 = m_set.tip_radius + 4 * res;
            const int    pad = int(std::ceil(m2 / res));
            Field        lf(W + 2 * pad, Hh + 2 * pad, lo.x - pad * res, lo.y - pad * res, res);
            for (int j = 0; j < lf.h; ++j)
                for (int i = 0; i < lf.w; ++i) lf.at(i, j) = inside_k(lf.pos(i, j)) ? 1.f : 0.f;
            soften_boundary(lf, inside_k);
            // Shrink by the tip radius before clipping to the footprint, so the outline itself is
            // not pulled in where a level reaches it.
            const auto  grown = contour_regions(lf, 0.5f, true, m_set.simplify, 0.0);
            const Field tf    = rasterize(offset_regions(grown, -m_set.tip_radius), lo.x, lo.y, W, Hh, res);
            for (size_t q = 0; q < mask.v.size(); ++q) mask.v[q] = std::min(mask.v[q], tf.v[q]);
        }
        L.regions = contour_regions(mask, 0.5f, true, m_set.simplify, m_set.min_area);
        if (!L.regions.empty()) model.levels.push_back(std::move(L));
    }
    // The level-0 contour is the clean union of the corrected outline.
    if (!model.levels.empty()) model.footprint = model.levels.front().regions;
    int missed = 0;
    for (const Node& n : m_nodes) missed += (n.measured && n.z < m_set.mat_level);
    if (missed > 0)
        model.warnings.push_back(std::to_string(missed) + " grid touches landed beside the part and were ignored.");
    return model;
}

} // namespace Slic3r::Metrology

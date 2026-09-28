#include "libslic3r/CAM/Post.hpp"

#include "libslic3r/CAM/FeedsSpeeds.hpp"
#include "libslic3r/CAM/ToolLibrary.hpp"
#include "libslic3r/CAM/Toolpath.hpp"
#include "libslic3r/Utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <set>
#include <sstream>

namespace Slic3r::CAM {

bool dialect_has_canned_cycles(PostDialect d) { return d == PostDialect::LinuxCNC || d == PostDialect::Mach3 || d == PostDialect::Fanuc; }
bool dialect_has_inverse_time(PostDialect d) { return d == PostDialect::LinuxCNC || d == PostDialect::Fanuc; }
bool dialect_has_tool_length_comp(PostDialect d) { return dialect_has_canned_cycles(d); }

const char* dialect_name(PostDialect d)
{
    switch (d) {
    case PostDialect::Grbl: return "GRBL";
    case PostDialect::LinuxCNC: return "LinuxCNC";
    case PostDialect::Mach3: return "Mach3/Mach4";
    case PostDialect::Fanuc: return "Fanuc";
    case PostDialect::Marlin: return "Marlin";
    }
    return "G-code";
}

PostOptions default_post_options(const MachineProfile& machine)
{
    PostOptions o;
    o.machine      = machine;
    o.arcs         = machine.post != PostDialect::Marlin;
    o.line_numbers = machine.post == PostDialect::Fanuc;
    o.inverse_time = machine.has_a_axis && dialect_has_inverse_time(machine.post);
    return o;
}

namespace {

constexpr double kToolChangeSeconds = 15;

using K = Move::Kind;

class Writer
{
public:
    Writer(const PostOptions& o) : m_o(o), m_d(o.machine.post), m_unit(o.inch ? 1 / 25.4 : 1), m_dec(o.inch ? 4 : 3),
        m_numbers(o.line_numbers || o.machine.post == PostDialect::Fanuc) {}

    std::string num(double v, int dec) const
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "%.*f", dec, v);
        std::string s = buf;
        if (s.find('.') != std::string::npos) {
            while (s.back() == '0') s.pop_back();
            if (s.back() == '.' && m_d != PostDialect::Fanuc) s.pop_back();   // Fanuc: keep "10." (no decimal = microns)
        }
        if (s == "-0" || s == "-0.") s = s.substr(1);
        return s;
    }
    std::string len(char w, double mm) const { return std::string(1, w) + num(mm * m_unit, m_dec); }
    std::string feed_word(double mm_min) const { return "F" + num(mm_min * m_unit, m_o.inch ? 2 : 1); }

    void line(const std::string& s)
    {
        if (s.empty()) return;
        if (m_numbers) {
            m_out << 'N' << m_n << ' ';
            m_n += 10;
        }
        m_out << s << '\n';
    }
    void raw(const std::string& s) { m_out << s << '\n'; }
    static std::string clean(std::string t)
    {
        std::replace(t.begin(), t.end(), '(', '[');
        std::replace(t.begin(), t.end(), ')', ']');
        std::replace(t.begin(), t.end(), '\n', ' ');
        return t;
    }
    std::string comment_text(const std::string& t) const { return m_d == PostDialect::Marlin ? "; " + clean(t) : "(" + clean(t) + ")"; }
    void comment(const std::string& t)
    {
        if (m_o.comments) line(comment_text(t));
    }
    void pause(const std::string& msg)
    {
        line(m_d == PostDialect::Marlin ? "M0 " + msg : "M0 (MSG, " + clean(msg) + ")");
    }

    // ---- motion ----
    void forget() { m_g = -1; m_known = false; m_f = -1; }

    void end_cycle()
    {
        if (m_in_cycle) {
            line("G80");
            m_in_cycle = false;
            m_g        = -1;
        }
    }

    void set_inverse(bool on)
    {
        if (on == m_g93) return;
        line(on ? "G93" : "G94");
        m_g93 = on;
        m_f   = -1;
    }

    // A word for the absolute angle a (unwrapped against the last one), empty when unchanged.
    std::string a_word(double a)
    {
        if (!m_o.machine.has_a_axis) return {};
        double out = a;
        if (m_a_known) {
            double delta = std::fmod(a - m_a_raw, 360.0);
            if (delta > 180) delta -= 360;
            if (delta <= -180) delta += 360;
            out = m_a_out + delta;
        }
        m_a_raw = a;
        if (m_a_known && std::abs(out - m_a_out) < 5e-4) return {};
        m_a_known = true;
        m_a_out   = out;
        double shown = out;
        if (m_o.a_modulo) {
            shown = std::fmod(out, 360.0);
            if (shown < 0) shown += 360;
        }
        return "A" + num(shown, 3);
    }

    // Turns A to `a`, first lifting Z to `safe_z` so the part never turns into the tool.
    void index_a(double a, double safe_z)
    {
        const std::string w = a_word(a);
        if (!w.empty()) {
            end_cycle();
            set_inverse(false);
            line("G0 " + len('Z', safe_z));
            line("G0 " + w);
            m_g = 0;
        }
    }

    // One linear or arc (ij != null) motion. `feed` is the surface feed over `length_mm`;
    // `combined` (output units, A in degrees) is what a control without inverse time meters for a
    // move that turns A.
    void motion(int g, const Vec3d& to, double a, double feed, const Vec2d* ij, double length_mm, double combined = 0)
    {
        end_cycle();
        if (!m_known && g == 0) {
            // Position unknown (program start, after a tool change or setup change): Z first, then
            // XY, so a tool left low (touch-off, manual change) never rapids diagonally into the work.
            set_inverse(false);
            line("G0 " + len('Z', to.z()));
            m_g     = 0;
            m_known = true;
            m_pos   = Vec3d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), to.z());
        }
        std::string w;
        for (int k = 0; k < 3; ++k)
            if (!m_known || std::abs(to[k] - m_pos[k]) * m_unit >= 0.5 * std::pow(10., -m_dec) || (ij && k < 2))
                w += (w.empty() ? "" : " ") + len("XYZ"[k], to[k]);
        const std::string aw = a_word(a);
        if (!aw.empty()) w += (w.empty() ? "" : " ") + aw;
        if (w.empty() && !ij) return;
        if (ij) w += " " + len('I', ij->x()) + " " + len('J', ij->y());
        const bool inverse = g != 0 && !aw.empty() && m_o.inverse_time && dialect_has_inverse_time(m_d);
        set_inverse(inverse);
        std::string s;
        // Arcs always name their direction; Marlin has no modal motion.
        if (g != m_g || ij || m_d == PostDialect::Marlin) s = "G" + std::to_string(g) + " ";
        s += w;
        if (g != 0) {
            if (inverse)
                s += " F" + num(feed / std::max(1e-6, length_mm), 3);
            else {
                // Keep the surface speed: a pure A move is metered in deg/min, a mixed one over
                // sqrt(dXYZ^2 + dA^2).
                const double f = !aw.empty() && combined > 0 && length_mm > 1e-9 ? feed * combined / (length_mm * m_unit) : feed;
                if (std::abs(f - m_f) > 1e-6) {
                    s += " " + feed_word(f);
                    m_f = f;
                }
            }
        }
        line(s);
        m_g     = g;
        m_pos   = to;
        m_known = true;
    }

    void emit_move(const Vec3d& from, double from_a, const Move& m)
    {
        const bool rapid = m.kind == K::Rapid || m.kind == K::Retract;
        if (rapid) {
            motion(0, m.to, m.a_deg, 0, nullptr, 0);
            return;
        }
        if (!is_arc(m) || std::abs(m.a_deg - from_a) > 1e-9) {
            double da = std::abs(std::fmod(m.a_deg - from_a, 360.0));   // the short way, as a_word unwraps
            if (da > 180) da = 360 - da;
            motion(1, m.to, m.a_deg, m.feed, nullptr, move_length(from, from_a, m), std::hypot((m.to - from).norm() * m_unit, da));
            return;
        }
        if (!m_o.arcs) {
            Vec3d p = from;
            for (const Vec3d& q : arc_points(from, m, std::max(1e-4, m_o.arc_tolerance))) {
                motion(1, q, m.a_deg, m.feed, nullptr, (q - p).norm());
                p = q;
            }
            return;
        }
        // G2/G3 with relative I/J, split into pieces of at most 180 deg.
        const double sweep = arc_sweep(from, m);
        const int    n     = std::max(1, int(std::ceil(std::abs(sweep) / M_PI - 1e-6)));
        const Vec2d  c     = m.center.head<2>();
        const Vec2d  a0v   = from.head<2>() - c;
        const double r0 = a0v.norm(), r1 = (m.to.head<2>() - c).norm(), a0 = std::atan2(a0v.y(), a0v.x());
        Vec3d        p     = from;
        for (int k = 1; k <= n; ++k) {
            const double t  = double(k) / n;
            const double an = a0 + sweep * t, rr = r0 + (r1 - r0) * t;
            const Vec3d  q  = k == n ? m.to : Vec3d(c.x() + rr * std::cos(an), c.y() + rr * std::sin(an), from.z() + (m.to.z() - from.z()) * t);
            const Vec2d  ij = c - p.head<2>();
            motion(arc_dir(m) == ArcDir::CW ? 2 : 3, q, m.a_deg, m.feed, &ij, 0);
            p = q;
        }
    }

    void canned(const std::string& code, const Vec3d& at, double bottom, double r, const std::string& extra, double feed)
    {
        std::string s = (m_in_cycle ? "" : "G99 ") + code + " " + len('X', at.x()) + " " + len('Y', at.y()) + " " + len('Z', bottom);
        if (!r_less(code)) s += " " + len('R', r);
        s += extra + " " + feed_word(feed);
        line(s);
        m_in_cycle = code != "G33.1";
        if (!m_in_cycle) m_g = -1;
        m_f     = feed;
        m_pos   = Vec3d(at.x(), at.y(), r);
        m_known = true;
    }
    static bool r_less(const std::string& code) { return code == "G33.1"; }

    std::string str() const { return m_out.str(); }
    int         dec() const { return m_dec; }
    double      unit() const { return m_unit; }

private:
    const PostOptions& m_o;
    PostDialect        m_d;
    double             m_unit;
    int                m_dec;
    bool               m_numbers;
    int                m_n{10};
    std::ostringstream m_out;
    int                m_g{-1};
    bool               m_known{false};
    Vec3d              m_pos{0, 0, 0};
    double             m_f{-1};
    bool               m_g93{false};
    bool               m_in_cycle{false};
    bool               m_a_known{false};
    double             m_a_raw{0}, m_a_out{0};
};

std::string duration(double s)
{
    const long t = std::lround(s);
    char       buf[64];
    if (t >= 3600)
        snprintf(buf, sizeof(buf), "%ld h %02ld min %02ld s", t / 3600, (t / 60) % 60, t % 60);
    else
        snprintf(buf, sizeof(buf), "%ld min %02ld s", t / 60, t % 60);
    return buf;
}

} // namespace

std::string post_process(const CamDocument& doc, const std::vector<int>& op_indices, const PostOptions& opts, std::string* error)
{
    auto fail = [&](const std::string& e) {
        if (error) *error = e;
        return std::string();
    };
    const PostDialect     d = opts.machine.post;
    const MachineProfile& machine = opts.machine;

    // Validate and collect.
    std::vector<int> ops;
    for (int i : op_indices) {
        if (i < 0 || i >= int(doc.operations.size()))
            return fail("An operation to post does not exist.");
        const CamOperation& op = doc.operations[i];
        if (!op.enabled)
            continue;
        if (i >= int(doc.paths.size()) || !doc.paths[i].ok() || doc.paths[i].moves.empty())
            return fail("Operation \"" + op.name + "\" has no valid toolpath. Generate it first.");
        if (!doc.find_tool(op.tool_number))
            return fail("Operation \"" + op.name + "\" uses tool T" + std::to_string(op.tool_number) + ", which is not in the tool library.");
        if (op.type == OpType::Drill && op.cycle == DrillCycle::Tap) {
            if (!dialect_has_canned_cycles(d))
                return fail("Tapping needs a control with synchronised tapping (LinuxCNC, Mach3, Fanuc).");
            if (!(doc.find_tool(op.tool_number)->thread_pitch > 0))
                return fail("Operation \"" + op.name + "\": the tap has no thread pitch. Set its pitch in the Tool Library.");
        }
        if (op.setup_index < 0 || op.setup_index >= int(doc.setups.size()))
            return fail("Operation \"" + op.name + "\" belongs to a setup that does not exist.");
        // "Xnan" would reach the machine as-is (or as X0 on a lenient control): refuse the post.
        for (const Move& m : doc.paths[i].moves)
            if (!m.to.allFinite() || !m.center.allFinite() || !std::isfinite(m.a_deg) || !std::isfinite(m.feed))
                return fail("Operation \"" + op.name + "\" has an invalid coordinate in its toolpath. Regenerate it.");
        if (!machine.has_a_axis)
            for (const Move& m : doc.paths[i].moves)
                if (std::abs(m.a_deg - doc.setups[op.setup_index].a_index_deg) > 1e-6)
                    return fail("Operation \"" + op.name + "\" turns the A axis, but the machine \"" + machine.name + "\" has none. Pick a 4-axis machine.");
        ops.push_back(i);
    }
    if (ops.empty())
        return fail("There are no operations to post.");

    Writer w(opts);
    // Header.
    if (d == PostDialect::Fanuc)
        w.raw("%");
    if (d == PostDialect::Fanuc || d == PostDialect::Mach3) {
        char buf[32];
        snprintf(buf, sizeof(buf), "O%04d", opts.program_number);
        w.line(std::string(buf) + " " + w.comment_text(opts.program_name));
    } else
        w.comment(opts.program_name);
    w.comment("Generated by " + header_slic3r_generated());
    w.comment("Machine: " + machine.name + " [" + dialect_name(d) + "]");
    {
        double           t = 0;
        int              changes = 0, last_tool = -1;
        std::set<int>    listed;
        std::vector<int> tool_order;
        for (int i : ops) {
            t += estimate_time(doc.paths[i], machine);
            const int tn = doc.operations[i].tool_number;
            if (tn != last_tool) ++changes;
            last_tool = tn;
            if (listed.insert(tn).second) tool_order.push_back(tn);
        }
        t += kToolChangeSeconds * std::max(0, changes - 1);
        w.comment("Material: " + std::string(material_name(doc.setups[doc.operations[ops.front()].setup_index].material)));
        w.comment("Estimated time: " + duration(t));
        w.comment("Tools:");
        for (int tn : tool_order) {
            const CamTool* tool = doc.find_tool(tn);
            w.comment("  T" + std::to_string(tn) + " D" + w.num(tool->diameter * w.unit(), w.dec()) + (opts.inch ? "in " : "mm ") + tool->name +
                      " - " + tool_type_name(tool->type));
        }
    }

    // Safe start.
    const std::string units = opts.inch ? "G20" : "G21";
    switch (d) {
    case PostDialect::Marlin: w.line(units); w.line("G90"); break;
    case PostDialect::Grbl: w.line("G17 " + units + " G90 G94"); break;
    default: w.line("G17 " + units + " G40 G49 G80 G90 G94"); break;
    }

    int  cur_tool = -1, cur_setup = -1;
    double cur_rpm = -1;
    bool spindle_on = false, coolant_on = false;
    for (int oi : ops) {
        const CamOperation& op    = doc.operations[oi];
        const CamSetup&     setup = doc.setups[op.setup_index];
        const CamTool&      tool  = *doc.find_tool(op.tool_number);
        const Toolpath&     tp    = doc.paths[oi];
        const FeedsSpeeds   fs    = effective_feeds(op, tool, setup.material, machine);

        if (op.setup_index != cur_setup) {
            w.end_cycle();
            if (cur_setup >= 0 && !machine.has_a_axis) {
                if (spindle_on) { w.line("M5"); spindle_on = false; cur_rpm = -1; }
                if (coolant_on) { w.line("M9"); coolant_on = false; }
                w.pause("Set up \"" + setup.name + "\" and zero the work offset, then resume");
            }
            w.comment("Setup: " + setup.name);
            if (d != PostDialect::Marlin)
                w.line("G" + std::to_string(std::clamp(setup.work_offset, 54, 59)));
            if (machine.has_a_axis)
                w.index_a(setup.a_index_deg, tp.moves.front().to.z());   // the op's first rapid is at clearance
            cur_setup = op.setup_index;
            w.forget();
        }
        w.comment("Operation: " + op.name);

        if (op.tool_number != cur_tool) {
            w.end_cycle();
            if (spindle_on) { w.line("M5"); spindle_on = false; cur_rpm = -1; }
            if (coolant_on) { w.line("M9"); coolant_on = false; }
            const std::string desc = "T" + std::to_string(tool.number) + " " + tool.name;
            if (machine.tool_change == ToolChange::M6 && d != PostDialect::Grbl && d != PostDialect::Marlin)
                w.line("T" + std::to_string(tool.number) + " M6" + (opts.comments ? " " + w.comment_text(tool.name) : ""));
            else
                w.pause("Change to " + desc);
            if (dialect_has_tool_length_comp(d))
                w.line("G43 H" + std::to_string(tool.number));
            cur_tool = op.tool_number;
            w.forget();
        }
        if (machine.spindle_control && (!spindle_on || std::abs(fs.rpm - cur_rpm) > 0.5)) {
            w.line("M3 S" + w.num(std::round(fs.rpm), 0));
            spindle_on = true;
            cur_rpm    = fs.rpm;
        }
        if (machine.coolant && !coolant_on) {
            w.line("M8");
            coolant_on = true;
        }

        const bool canned = dialect_has_canned_cycles(d);
        Vec3d      pos    = tp.moves.front().to;
        double     pos_a  = tp.moves.front().a_deg;
        for (size_t i = 0; i < tp.moves.size(); ++i) {
            const Move& m = tp.moves[i];
            if (m.cycle >= 0 && canned && i > 0) {
                size_t j = i;
                double bottom = m.to.z();
                while (j < tp.moves.size() && tp.moves[j].cycle == m.cycle) {
                    bottom = std::min(bottom, tp.moves[j].to.z());
                    ++j;
                }
                const double r = pos.z();
                std::string  code, extra;
                const bool   fanuc = d == PostDialect::Fanuc;
                const std::string dwell = op.dwell_s > 0 ? (fanuc ? " P" + w.num(std::round(op.dwell_s * 1000), 0) : " P" + w.num(op.dwell_s, 3)) : "";
                switch (op.cycle) {
                case DrillCycle::Drill: code = op.dwell_s > 0 ? "G82" : "G81"; extra = dwell; break;
                case DrillCycle::Peck: code = "G83"; extra = " " + w.len('Q', op.peck_depth); break;
                case DrillCycle::ChipBreak: code = "G73"; extra = " " + w.len('Q', op.peck_depth); break;
                case DrillCycle::Bore: code = op.dwell_s > 0 ? "G89" : "G85"; extra = dwell; break;
                case DrillCycle::Tap: code = d == PostDialect::LinuxCNC ? "G33.1" : "G84"; break;
                }
                if (code == "G33.1") {
                    w.line("G33.1 " + w.len('Z', bottom) + " " + w.len('K', tool.thread_pitch));
                    w.forget();
                } else
                    w.canned(code, m.to, bottom, r, extra, m.feed);
                pos = Vec3d(m.to.x(), m.to.y(), r);
                pos_a = m.a_deg;
                i = j - 1;
                continue;
            }
            if (i == 0)
                w.motion(0, m.to, m.a_deg, 0, nullptr, 0);
            else
                w.emit_move(pos, pos_a, m);
            // Expanded dwell at the bottom of a drill cycle.
            if (m.cycle >= 0 && op.dwell_s > 0 && (m.kind == K::Plunge) &&
                (i + 1 >= tp.moves.size() || tp.moves[i + 1].to.z() > m.to.z()))
                w.line(d == PostDialect::Marlin ? "G4 S" + w.num(op.dwell_s, 3) : "G4 P" + w.num(op.dwell_s, 3));
            pos   = m.to;
            pos_a = m.a_deg;
        }
        w.end_cycle();
    }

    // End.
    w.set_inverse(false);
    if (spindle_on) w.line("M5");
    if (coolant_on) w.line("M9");
    if (d == PostDialect::Fanuc || d == PostDialect::Mach3) {
        w.line("G91 G28 Z0");
        w.line("G90");
    }
    if (d != PostDialect::Marlin)   // Marlin's M30 deletes a file from the SD card
        w.line("M30");
    if (d == PostDialect::Fanuc)
        w.raw("%");
    if (error) error->clear();
    return w.str();
}

} // namespace Slic3r::CAM

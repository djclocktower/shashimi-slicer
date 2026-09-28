#include <catch2/catch_all.hpp>

#include "libslic3r/Laser/LaserGCode.hpp"
#include "libslic3r/Laser/Laser.hpp"

#include <sstream>

using namespace Slic3r;
using namespace Slic3r::Laser;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::StartsWith;

namespace {

std::vector<std::string> lines_of(const std::string& g)
{
    std::vector<std::string> out;
    std::istringstream in(g);
    for (std::string l; std::getline(in, l);)
        if (!l.empty() && l[0] != ';') out.push_back(l);
    return out;
}

bool has_line(const std::string& g, const std::string& line)
{
    for (const std::string& l : lines_of(g))
        if (l == line) return true;
    return false;
}

Segment seg(Segment::Kind k, Vec2d from, Vec2d to, double speed, double power = 0, int layer = 0)
{
    Segment s;
    s.kind       = k;
    s.from       = from;
    s.to         = to;
    s.speed_mm_s = speed;
    s.power_pct  = power;
    s.layer      = layer;
    return s;
}

// Travel to (10,10), cut a 10 mm line at 50 % and 20 mm/s, then another at the same power.
LaserJob cut_job()
{
    LaserJob job;
    job.segments.push_back(seg(Segment::Kind::Travel, {0, 0}, {10, 10}, 100));
    job.segments.push_back(seg(Segment::Kind::Cut, {10, 10}, {20, 10}, 20, 50));
    job.segments.push_back(seg(Segment::Kind::Cut, {20, 10}, {20, 20}, 20, 50));
    job.bounds.merge(Vec2d(10, 10));
    job.bounds.merge(Vec2d(20, 20));
    return job;
}

} // namespace

TEST_CASE("GRBL output: header, dynamic power, modal words, footer", "[LaserGCode]")
{
    LaserDevice dev;   // GRBL, S1000, M4, front-left
    dev.start_gcode = "; my start";
    dev.end_gcode   = "G0 X0 Y0";
    std::string err;
    const std::string g = gcode(cut_job(), dev, {}, &err);
    REQUIRE(err.empty());
    const std::vector<std::string> l = lines_of(g);
    CHECK(l[0] == "G21 G90 G54");
    CHECK(l[1] == "M4 S0");
    CHECK(has_line(g, "G0 X10 Y10"));
    CHECK(has_line(g, "G1 X20 S500 F1200"));   // S = 50 % of 1000, F = 20 mm/s * 60
    CHECK(has_line(g, "G1 Y20"));              // modal: unchanged X, S and F are not repeated
    CHECK(l[l.size() - 3] == "M5");
    CHECK(l[l.size() - 2] == "G0 X0 Y0");
    CHECK(l.back() == "M2");
    CHECK_THAT(g, ContainsSubstring("; my start"));

    SECTION("constant power: M3, travel carries S0")
    {
        LaserDevice d = dev;
        d.laser_mode_dynamic = false;
        const std::string g2 = gcode(cut_job(), d);
        CHECK(lines_of(g2)[1] == "M3 S0");
        CHECK(has_line(g2, "G0 X10 Y10 S0"));
        CHECK(has_line(g2, "G1 X20 S500 F1200"));
    }
    SECTION("travel as G1 S0")
    {
        LaserDevice d = dev;
        d.uses_g0_for_travel = false;
        CHECK(has_line(gcode(cut_job(), d), "G1 X10 Y10 S0 F6000"));
    }
    SECTION("S scaling and always-emit words")
    {
        LaserDevice d = dev;
        d.s_max = 255;
        GCodeOptions o;
        o.modal_fs = false;
        const std::string g2 = gcode(cut_job(), d, o);
        CHECK(has_line(g2, "G1 X20 Y10 S128 F1200"));   // round(127.5)
        CHECK(has_line(g2, "G1 X20 Y20 S128 F1200"));
    }
    SECTION("Z moves, air assist and dwell")
    {
        LaserDevice d = dev;
        d.enable_z = true;
        LaserJob job = cut_job();
        for (Segment& s : job.segments) { s.z = -1.5; s.air_assist = true; }
        Segment dot = seg(Segment::Kind::Dwell, {20, 20}, {20, 20}, 0, 80);
        dot.dwell_ms = 250;
        job.segments.push_back(dot);
        const std::string g2 = gcode(job, d);
        CHECK(has_line(g2, "G0 Z-1.5"));
        CHECK(has_line(g2, "M8"));
        CHECK(has_line(g2, "M9"));
        CHECK(has_line(g2, "M3 S800"));
        CHECK(has_line(g2, "G4 P0.25"));
    }
    SECTION("invalid jobs")
    {
        LaserJob bad = cut_job();
        bad.segments[1].to.x() = std::nan("");
        CHECK(gcode(bad, dev, {}, &err).empty());
        CHECK_FALSE(err.empty());
        LaserDevice r = dev;
        r.type = DeviceType::Ruida;
        CHECK(gcode(cut_job(), r, {}, &err).empty());
        LaserJob failed;
        failed.error = "Cancelled";
        CHECK(gcode(failed, dev, {}, &err).empty());
        CHECK(err == "Cancelled");
    }
}

TEST_CASE("A scan row with three runs writes one G1 per run plus dark stretches", "[LaserGCode]")
{
    LaserJob job;
    ScanLine sl;
    sl.start = {0, 5};
    sl.end   = {30, 5};
    sl.runs  = {{2, 8, 100}, {8, 12, 40}, {20, 25, 70}};   // two touching runs, a gap, a run
    job.scans.push_back(sl);
    Segment s = seg(Segment::Kind::Scan, sl.start, sl.end, 100, 100);
    s.scan = 0;
    job.segments.push_back(seg(Segment::Kind::Travel, {0, 0}, {0, 5}, 100));
    job.segments.push_back(s);
    LaserDevice dev;
    const std::vector<std::string> l = lines_of(gcode(job, dev));
    std::vector<std::string> g1;
    for (const std::string& x : l)
        if (x.rfind("G1", 0) == 0) g1.push_back(x);
    REQUIRE(g1.size() == 6);
    CHECK(g1[0] == "G1 X2 S0 F6000");   // overscan in, laser off
    CHECK(g1[1] == "G1 X8 S1000");
    CHECK(g1[2] == "G1 X12 S400");
    CHECK(g1[3] == "G1 X20 S0");        // gap
    CHECK(g1[4] == "G1 X25 S700");
    CHECK(g1[5] == "G1 X30 S0");        // overscan out
}

TEST_CASE("Marlin, Smoothie and the origin corner", "[LaserGCode]")
{
    LaserDevice dev;
    dev.type  = DeviceType::Marlin;
    dev.s_max = 255;
    SECTION("Marlin M3/M5 per lit stretch")
    {
        const std::string g = gcode(cut_job(), dev);
        CHECK(lines_of(g)[0] == "G21");
        CHECK(has_line(g, "M4 S128"));
        CHECK(has_line(g, "G1 X20 F1200"));
        CHECK(has_line(g, "M5"));
        CHECK_FALSE(has_line(g, "M2"));
    }
    SECTION("Marlin fan-laser mode")
    {
        dev.marlin_fan_laser = true;
        const std::string g = gcode(cut_job(), dev);
        CHECK(has_line(g, "M106 S128"));
        CHECK(has_line(g, "M107"));
        CHECK_FALSE(has_line(g, "M4 S128"));
    }
    SECTION("Marlin inline")
    {
        dev.marlin_inline = true;
        const std::string g = gcode(cut_job(), dev);
        CHECK(has_line(g, "M4 I"));
        CHECK(has_line(g, "G1 X20 S128 F1200"));
    }
    SECTION("Smoothie power as 0..1")
    {
        dev.type = DeviceType::Smoothie;
        const std::string g = gcode(cut_job(), dev);
        CHECK(has_line(g, "G1 X20 S0.5 F1200"));
        CHECK_FALSE(has_line(g, "M4 S0"));
    }
    SECTION("origin corner flips, rotary passes Y through")
    {
        LaserDevice d;   // GRBL 400 x 400
        d.origin_corner = OriginCorner::RearRight;
        const std::string g = gcode(cut_job(), d);
        CHECK(has_line(g, "G0 X390 Y390"));
        CHECK(has_line(g, "G1 X380 S200 F1200") == false);   // power is 50 %
        CHECK(has_line(g, "G1 X380 S500 F1200"));
        CHECK(to_machine({10, 20}, d, JobSettings::StartFrom::UserOrigin).isApprox(Vec2d(-10, -20)));
        CHECK(from_machine(to_machine({10, 20}, d, JobSettings::StartFrom::Absolute), d, JobSettings::StartFrom::Absolute)
                  .isApprox(Vec2d(10, 20)));
        d.rotary.enabled = true;
        const std::string r = gcode(cut_job(), d);
        CHECK(has_line(r, "G0 X390 Y10"));
    }
    SECTION("current position jobs are wrapped in G92")
    {
        LaserDevice d;
        LaserJob job = cut_job();
        job.start_from = JobSettings::StartFrom::CurrentPosition;
        const std::string g = gcode(job, d);
        CHECK(has_line(g, "G92 X0 Y0"));
        CHECK(has_line(g, "G92.1"));
    }
}

TEST_CASE("Framing traces the job bounds with the laser off", "[LaserGCode]")
{
    LaserDevice dev;
    const std::string g = frame_gcode(cut_job(), dev, FrameMode::Rect);
    CHECK(has_line(g, "G0 X10 Y10"));
    CHECK(has_line(g, "G1 X20 S0 F6000"));
    CHECK(has_line(g, "G1 Y20"));
    CHECK(has_line(g, "G1 X10"));
    CHECK(has_line(g, "G1 Y10"));
    CHECK_FALSE(g.find("S500") != std::string::npos);
    dev.frame_power_pct = 1;
    CHECK(has_line(frame_gcode(cut_job(), dev, FrameMode::Rect), "G1 X20 S10 F6000"));
    // Outline: the hull of the lit moves (a triangle here).
    const std::string o = frame_gcode(cut_job(), LaserDevice{}, FrameMode::Outline);
    CHECK(has_line(o, "G1 X20 S0 F6000"));
    CHECK_FALSE(has_line(o, "G1 X10 Y20"));
}

TEST_CASE("export_gcode plans and writes a document", "[LaserGCode]")
{
    LaserDocument doc;
    LaserShape r;
    r.type = ShapeType::Rect;
    r.width = r.height = 10;
    r.xform.translation() = Vec2d(20, 20);
    doc.add_shape(r);
    std::string err;
    const std::string g = export_gcode(doc, LaserDevice{}, {}, &err);
    CHECK(err.empty());
    CHECK_THAT(g, ContainsSubstring("G1 X25 S200 F6000"));
    doc.shapes.clear();
    CHECK(export_gcode(doc, LaserDevice{}, {}, &err).empty());
    CHECK_FALSE(err.empty());
}

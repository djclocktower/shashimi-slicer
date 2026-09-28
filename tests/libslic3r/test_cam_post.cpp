#include <catch2/catch_all.hpp>

#include "libslic3r/CAM/CAM.hpp"

#include <regex>
#include <sstream>

using namespace Slic3r;
using namespace Slic3r::CAM;

namespace {

Move mv(Move::Kind k, double x, double y, double z, double feed = 0, double a = 0)
{
    Move m;
    m.kind  = k;
    m.to    = Vec3d(x, y, z);
    m.feed  = feed;
    m.a_deg = a;
    return m;
}

bool has(const std::string& g, const std::string& s) { return g.find(s) != std::string::npos; }

int count_lines(const std::string& g, const std::string& re)
{
    std::istringstream in(g);
    std::regex         r(re);
    int                n = 0;
    for (std::string line; std::getline(in, line);)
        n += std::regex_search(line, r);
    return n;
}

// T1 6 mm flat: a square with a 270 deg arc; T2 5 mm drill: one peck-drilled hole.
CamDocument sample()
{
    CamDocument doc;
    CamTool t1;
    t1.number = 1; t1.name = "6 mm flat end mill"; t1.diameter = 6;
    CamTool t2;
    t2.number = 2; t2.name = "5 mm drill"; t2.type = ToolType::Drill; t2.diameter = 5;
    doc.add_tool(t1);
    doc.add_tool(t2);
    doc.add_setup(CamSetup{});

    CamOperation c = default_operation(OpType::Contour2D, &doc.tools[0]);
    c.name = "Contour1";
    doc.add_operation(c);
    Toolpath& p = doc.paths[0];
    p.moves = {mv(Move::Kind::Rapid, 0, 0, 10), mv(Move::Kind::Rapid, 0, 0, 5), mv(Move::Kind::Plunge, 0, 0, -1, 200),
               mv(Move::Kind::Feed, 10, 0, -1, 800)};
    Move arc = mv(Move::Kind::ArcCW, 20, 10, -1, 800);   // 270 deg clockwise around (10, 10)
    arc.center = Vec3d(10, 10, 0);
    p.moves.push_back(arc);
    p.moves.push_back(mv(Move::Kind::Retract, 20, 10, 10));

    CamOperation d = default_operation(OpType::Drill, &doc.tools[1]);
    d.name       = "Drill1";
    d.cycle      = DrillCycle::Peck;
    d.peck_depth = 2;
    doc.add_operation(d);
    Toolpath& q = doc.paths[1];
    q.moves = {mv(Move::Kind::Rapid, 30, 30, 10), mv(Move::Kind::Rapid, 30, 30, 2)};
    for (Move m : {mv(Move::Kind::Plunge, 30, 30, -2, 100), mv(Move::Kind::Retract, 30, 30, 2), mv(Move::Kind::Rapid, 30, 30, -1.5),
                   mv(Move::Kind::Plunge, 30, 30, -4, 100), mv(Move::Kind::Retract, 30, 30, 2)}) {
        m.cycle = 0;
        q.moves.push_back(m);
    }
    q.moves.push_back(mv(Move::Kind::Retract, 30, 30, 10));
    return doc;
}

PostOptions options(const char* machine)
{
    return default_post_options(find_machine(machine));
}

} // namespace

TEST_CASE("GRBL post: safe start, manual tool change, arcs, expanded drilling", "[CamPost]")
{
    const CamDocument doc = sample();
    std::string       err;
    const std::string g = post_process(doc, {0, 1}, options("GRBL router (Shapeoko/X-Carve class)"), &err);
    INFO(g);
    REQUIRE(err.empty());
    REQUIRE(has(g, "G21"));
    REQUIRE(has(g, "G90"));
    REQUIRE(count_lines(g, "^M3 S[0-9]+$") == 2);
    REQUIRE(has(g, "M0 (MSG, Change to T2 5 mm drill)"));
    REQUIRE_FALSE(has(g, "M6"));
    REQUIRE_FALSE(has(g, "G43"));
    REQUIRE(has(g, "M30"));
    REQUIRE(has(g, "(Estimated time:"));
    REQUIRE(has(g, "T1 D6mm 6 mm flat end mill"));
    // the 270 deg arc is split into two G2 moves with relative I/J
    REQUIRE(count_lines(g, "^G2 X[-0-9.]+ Y[-0-9.]+ I[-0-9.]+ J[-0-9.]+") == 2);
    REQUIRE(has(g, "I0 J10"));
    // no canned cycles: the pecks are plain moves
    REQUIRE_FALSE(has(g, "G83"));
    REQUIRE(count_lines(g, "Z-2 F100$") == 1);
    REQUIRE(count_lines(g, "Z-4$") == 1);
    REQUIRE(count_lines(g, "^(G0 )?Z-1.5$") == 1);   // modal G0 after the retract
}

TEST_CASE("LinuxCNC post: M6, tool length offset, canned cycles", "[CamPost]")
{
    const CamDocument doc = sample();
    const std::string g   = post_process(doc, {0, 1}, options("LinuxCNC mill"));
    INFO(g);
    REQUIRE(has(g, "G17 G21 G40 G49 G80 G90 G94"));
    REQUIRE(has(g, "G54"));
    REQUIRE(has(g, "T1 M6"));
    REQUIRE(has(g, "T2 M6"));
    REQUIRE(has(g, "G43 H2"));
    REQUIRE(has(g, "M8"));
    REQUIRE(has(g, "G99 G83 X30 Y30 Z-4 R2 Q2 F100"));
    REQUIRE(count_lines(g, "Z-1.5") == 0);   // collapsed into the cycle
    REQUIRE(has(g, "G80"));
    REQUIRE(count_lines(g, "^G[23] ") == 2);
    REQUIRE(has(g, "M30"));
}

TEST_CASE("Fanuc post: program number, line numbers, decimal points", "[CamPost]")
{
    const std::string g = post_process(sample(), {0, 1}, options("LinuxCNC mill") /*dialect below*/, nullptr);
    PostOptions o = options("LinuxCNC mill");
    o.machine.post = PostDialect::Fanuc;
    o = default_post_options(o.machine);
    const std::string f = post_process(sample(), {0, 1}, o);
    INFO(f);
    REQUIRE(f.rfind("%\n", 0) == 0);
    REQUIRE(has(f, "\nN10 O1000 (shashimi)"));
    REQUIRE(count_lines(f, "^N[0-9]+ ") > 10);
    REQUIRE(has(f, "X10. "));
    REQUIRE(has(f, "G91 G28 Z0"));
    REQUIRE(f.substr(f.size() - 2) == "%\n");
    REQUIRE(g != f);
}

TEST_CASE("Marlin post linearises arcs and never ends with M30", "[CamPost]")
{
    const std::string g = post_process(sample(), {0}, options("Generic 3-axis") /*Grbl*/);
    PostOptions o = options("Generic 3-axis");
    o.machine.post = PostDialect::Marlin;
    o = default_post_options(o.machine);
    REQUIRE_FALSE(o.arcs);
    const std::string m = post_process(sample(), {0}, o);
    INFO(m);
    REQUIRE(count_lines(m, "^G[23] ") == 0);
    REQUIRE(count_lines(m, "^G1 ") > 20);   // the 270 deg arc as short chords
    REQUIRE_FALSE(has(m, "M30"));
    REQUIRE(has(m, "; Operation: Contour1"));
    REQUIRE(has(g, "(Operation: Contour1)"));
}

TEST_CASE("Inch output converts every length", "[CamPost]")
{
    PostOptions o = options("Generic 3-axis");
    o.inch = true;
    const std::string g = post_process(sample(), {0}, o);
    INFO(g);
    REQUIRE(has(g, "G20"));
    REQUIRE_FALSE(has(g, "G21"));
    REQUIRE(has(g, "X0.3937"));   // 10 mm
    REQUIRE(has(g, "F31.5"));     // 800 mm/min
}

TEST_CASE("A axis words are unwrapped, optionally modulo, with inverse time", "[CamPost]")
{
    CamDocument doc = sample();
    doc.operations[0].type = OpType::RotaryFinish;
    doc.setups[0].a_index_deg = 350;
    Toolpath& p = doc.paths[0];
    p.moves = {mv(Move::Kind::Rapid, 0, 0, 20, 0, 350), mv(Move::Kind::Plunge, 0, 0, 10, 100, 350),
               mv(Move::Kind::Feed, 0, 0, 10, 500, 10), mv(Move::Kind::Feed, 5, 0, 10, 500, 10), mv(Move::Kind::Retract, 5, 0, 20, 0, 10)};
    PostOptions o = options("Generic 4-axis (A about X)");
    REQUIRE(o.inverse_time);
    std::string g = post_process(doc, {0}, o);
    INFO(g);
    REQUIRE(has(g, "G0 A350"));    // the setup's index position
    REQUIRE(has(g, "G0 Z20\nG0 A350"));   // Z up to the op's clearance before the part turns
    REQUIRE(has(g, "A370"));       // 350 -> 10 is +20 deg, not -340
    REQUIRE(count_lines(g, "(^| )A10( |$)") == 0);
    // inverse time: 20 deg at r = 10 is 3.49 mm at 500 mm/min -> F = 143.2 (1/min)
    REQUIRE(has(g, "G93"));
    REQUIRE(has(g, "A370 F143.239"));
    REQUIRE(has(g, "G94"));

    o.a_modulo     = true;
    o.inverse_time = false;
    g = post_process(doc, {0}, o);
    REQUIRE(count_lines(g, "(^| )A10( |$)") == 1);
    REQUIRE_FALSE(has(g, "G93"));
    // without inverse time a pure A move is metered in deg/min: 500 mm/min at r = 10 is 2864.8 deg/min
    REQUIRE(has(g, "A10 F2864.8"));

    // a 3-axis machine refuses A motion
    std::string err;
    REQUIRE(post_process(doc, {0}, options("Generic 3-axis"), &err).empty());
    REQUIRE(has(err, "A axis"));
}

TEST_CASE("A turns between setups only after Z is up", "[CamPost]")
{
    CamDocument doc = sample();
    CamSetup    s2;
    s2.name        = "Setup2";
    s2.a_index_deg = 90;
    doc.add_setup(s2);
    doc.operations[1].setup_index = 1;
    for (Move& m : doc.paths[1].moves) m.a_deg = 90;   // as generate_toolpath stamps an indexed setup
    const std::string g = post_process(doc, {0, 1}, options("Generic 4-axis (A about X)"));
    INFO(g);
    REQUIRE(has(g, "G0 Z10\nG0 A90"));   // Drill1's first rapid is at Z10
}

TEST_CASE("Tapping needs a synchronised control and a thread pitch", "[CamPost]")
{
    CamDocument doc = sample();
    doc.operations[1].cycle = DrillCycle::Tap;
    doc.tools[1].type       = ToolType::Tap;
    std::string err;
    for (const char* m : {"GRBL router (Shapeoko/X-Carve class)", "Generic 3-axis"}) {
        REQUIRE(post_process(doc, {0, 1}, options(m), &err).empty());
        REQUIRE(has(err, "synchronised tapping"));
    }
    doc.tools[1].thread_pitch = 0;
    REQUIRE(post_process(doc, {0, 1}, options("LinuxCNC mill"), &err).empty());
    REQUIRE(has(err, "thread pitch"));
    doc.tools[1].thread_pitch = 0.8;
    const std::string g = post_process(doc, {0, 1}, options("LinuxCNC mill"), &err);
    INFO(g);
    REQUIRE(err.empty());
    REQUIRE(has(g, "G33.1 Z-4 K0.8"));
}

TEST_CASE("Post refuses operations without a toolpath", "[CamPost]")
{
    CamDocument doc = sample();
    doc.paths[1].error = "Nothing to cut.";
    std::string err;
    REQUIRE(post_process(doc, {0, 1}, options("Generic 3-axis"), &err).empty());
    REQUIRE(has(err, "Drill1"));
    doc.operations[1].enabled = false;   // suppressed ops are skipped
    REQUIRE_FALSE(post_process(doc, {0, 1}, options("Generic 3-axis"), &err).empty());
}

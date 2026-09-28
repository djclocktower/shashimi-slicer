#include <catch2/catch_all.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <chrono>
#include <mutex>
#include <thread>

#include "slic3r/Utils/GrblStreamer.hpp"

using namespace Slic3r;
using namespace std::chrono_literals;

namespace {

// Polls `pred` for up to `ms`.
template<class Pred> bool wait_for(Pred pred, int ms = 5000)
{
    for (auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms); std::chrono::steady_clock::now() < end;
         std::this_thread::sleep_for(5ms))
        if (pred()) return true;
    return pred();
}

struct Rig {
    std::shared_ptr<GrblSimulator::Shared> sim = std::make_shared<GrblSimulator::Shared>();
    std::mutex                             m;
    std::vector<size_t>                    acked;     // lines_acked per progress report
    std::vector<std::string>               errors;
    std::atomic<int>                       done{0}, completed{0};
    GrblStreamer                           streamer;  // last: disconnects (and calls back) before the rest goes

    Rig(int lines_per_read = 2, const Laser::LaserDevice& device = {})
    {
        sim->lines_per_read = lines_per_read;
        GrblStreamer::Callbacks cb;
        cb.on_progress = [this](const JobProgress& p) { std::lock_guard<std::mutex> lk(m); acked.push_back(p.lines_acked); };
        cb.on_error    = [this](const std::string& t, int) { std::lock_guard<std::mutex> lk(m); errors.push_back(t); };
        cb.on_job_done = [this](bool ok, const std::string&) { ++done; completed += ok; };
        streamer.set_callbacks(cb);
        std::string err;
        REQUIRE(streamer.connect(std::make_unique<GrblSimulator>(sim), device, &err));
        if (device.type != Laser::DeviceType::Marlin)   // the simulator answers no M114
            REQUIRE(wait_for([&] { return streamer.status().state == GrblState::Idle; }));
    }
};

std::vector<std::string> job_lines(int n)
{
    std::vector<std::string> lines;
    for (int i = 0; i < n; ++i) lines.push_back("G1 X" + std::to_string(i % 50) + ".125 Y" + std::to_string(i % 37) + ".5 S250 F3000 ; c");
    return lines;
}

} // namespace

TEST_CASE("Status reports fill position, feed, overrides, buffer and pins", "[GrblStreamer]")
{
    GrblStatus st;
    REQUIRE(parse_grbl_status("<Idle|MPos:10.000,20.000,1.000|FS:0,0|WCO:5.000,5.000,0.000>", st));
    CHECK(st.state == GrblState::Idle);
    CHECK_THAT(st.wpos.x(), Catch::Matchers::WithinAbs(5, 1e-9));
    CHECK_THAT(st.wpos.y(), Catch::Matchers::WithinAbs(15, 1e-9));
    REQUIRE(parse_grbl_status("<Hold:1|WPos:1.5,2.5,0|Bf:15,128|FS:1200,300|Ov:110,100,90|A:SF|Pn:XZ>", st));
    CHECK(st.state == GrblState::Hold);
    CHECK(st.substate == 1);
    CHECK_THAT(st.mpos.x(), Catch::Matchers::WithinAbs(6.5, 1e-9));   // WCO kept from the previous report
    CHECK(st.feed == 1200);
    CHECK(st.spindle == 300);
    CHECK(st.ov_feed == 110);
    CHECK(st.ov_spindle == 90);
    CHECK(st.rx_free == 128);
    CHECK(st.pins == "XZ");
    CHECK(st.accessories == "SF");
    REQUIRE(parse_grbl_status("<Run|MPos:0,0,0|FS:0,0>", st));
    CHECK(st.pins.empty());
    CHECK_FALSE(parse_grbl_status("ok", st));
    CHECK(grbl_error_text(22).find("feed") != std::string::npos);
    CHECK(grbl_alarm_text(2).find("Soft limit") != std::string::npos);
}

TEST_CASE("A job streams in order without overfilling the 128-byte RX buffer", "[GrblStreamer]")
{
    Rig rig;
    const auto lines = job_lines(300);
    REQUIRE(rig.streamer.start(lines, 60));
    REQUIRE(wait_for([&] { return rig.done.load() > 0; }, 20000));
    CHECK(rig.completed == 1);
    CHECK(rig.sim->max_rx_bytes <= 127);
    CHECK(rig.sim->max_rx_bytes > 60);   // really streamed ahead, not ping-pong
    {
        std::lock_guard<std::mutex> lk(rig.sim->mutex);
        std::vector<std::string> job;
        for (const auto& l : rig.sim->received)
            if (l.rfind("G1", 0) == 0) job.push_back(l);
        REQUIRE(job.size() == 300);
        CHECK(job.front() == "G1 X0.125 Y0.5 S250 F3000");   // comment stripped
        CHECK(job.back() == "G1 X49.125 Y3.5 S250 F3000");
    }
    std::lock_guard<std::mutex> lk(rig.m);
    CHECK(std::is_sorted(rig.acked.begin(), rig.acked.end()));
    CHECK(rig.acked.back() == 300);
}

TEST_CASE("An error mid-job pauses it; resume continues and stop resets and unlocks", "[GrblStreamer]")
{
    Rig rig(1);
    rig.sim->fail_on_line = 50 + 1;   // + the $I sent on connect
    REQUIRE(rig.streamer.start(job_lines(200)));
    REQUIRE(wait_for([&] { return rig.streamer.progress().paused; }));
    {
        std::lock_guard<std::mutex> lk(rig.m);
        REQUIRE(rig.errors.size() == 1);
        CHECK(rig.errors[0] == grbl_error_text(20));
    }
    REQUIRE(wait_for([&] { return rig.streamer.status().state == GrblState::Hold; }));
    const size_t sent = rig.streamer.progress().lines_sent;
    std::this_thread::sleep_for(100ms);
    CHECK(rig.streamer.progress().lines_sent == sent);   // nothing streamed while paused

    rig.streamer.resume();
    REQUIRE(wait_for([&] { return rig.streamer.progress().lines_sent > sent; }));
    rig.streamer.pause();
    REQUIRE(wait_for([&] { return rig.streamer.status().state == GrblState::Hold; }));
    CHECK(rig.streamer.send_line("G0 X0") == false);   // console refused while a job runs

    rig.streamer.stop();
    REQUIRE(wait_for([&] { return rig.done.load() > 0; }));
    CHECK(rig.completed == 0);
    CHECK_FALSE(rig.streamer.progress().running);
    REQUIRE(wait_for([&] { return rig.streamer.status().state == GrblState::Idle; }));
    // $X went out after the reset.
    REQUIRE(wait_for([&] {
        std::lock_guard<std::mutex> lk(rig.sim->mutex);
        return !rig.sim->received.empty() && rig.sim->received.back() == "$X";
    }));
}

TEST_CASE("Console lines, settings, jog and set origin reach the controller", "[GrblStreamer]")
{
    Rig rig;
    REQUIRE(rig.streamer.send_line("$$"));
    REQUIRE(wait_for([&] { return rig.streamer.settings().count(130) == 1; }));
    CHECK(rig.streamer.settings().at(30) == "1000");
    CHECK(rig.streamer.firmware().find("Grbl 1.1h") != std::string::npos);
    rig.streamer.jog(10, -2.5, 0, 50);
    REQUIRE(wait_for([&] { return std::abs(rig.streamer.status().wpos.x() - 10) < 1e-6; }));
    rig.streamer.set_origin();
    REQUIRE(wait_for([&] { return std::abs(rig.streamer.status().wpos.x()) < 1e-6; }));
    CHECK_THAT(rig.streamer.status().mpos.x(), Catch::Matchers::WithinAbs(10, 1e-6));
    bool has_jog = false;
    for (const ConsoleLine& l : rig.streamer.console())
        has_jog |= l.dir == ConsoleLine::Dir::Tx && l.text == "$J=G91 G21 X10.000 Y-2.500 F3000";
    CHECK(has_jog);
    rig.streamer.disconnect();
    CHECK(rig.streamer.status().state == GrblState::Disconnected);
}

TEST_CASE("Disconnecting mid-job holds and resets the controller", "[GrblStreamer]")
{
    Rig rig(1);
    rig.sim->time_scale = 1.0;   // moves take real time, so the job is still running
    REQUIRE(rig.streamer.start(job_lines(200)));
    REQUIRE(wait_for([&] { return rig.streamer.progress().lines_acked > 5; }));
    CHECK(rig.sim->resets == 0);
    rig.streamer.disconnect();
    CHECK(rig.sim->resets == 1);   // the reset switches the laser off at once
}

TEST_CASE("Marlin pause switches the laser off and resume restores its power", "[GrblStreamer]")
{
    Laser::LaserDevice dev;
    dev.type = Laser::DeviceType::Marlin;
    Rig rig(1, dev);
    rig.sim->time_scale = 1.0;   // real move times: the job is still running when paused
    std::vector<std::string> lines{"M3 S100"};
    for (const std::string& l : job_lines(300)) lines.push_back(l);
    REQUIRE(rig.streamer.start(lines));
    REQUIRE(wait_for([&] { return rig.streamer.progress().lines_acked > 5; }));
    rig.streamer.pause();
    auto count = [&](const char* line) {
        std::lock_guard<std::mutex> lk(rig.sim->mutex);
        return std::count(rig.sim->received.begin(), rig.sim->received.end(), std::string(line));
    };
    REQUIRE(wait_for([&] { return count("M5") == 1; }));
    rig.streamer.resume();
    REQUIRE(wait_for([&] { return count("M3 S100") == 2; }));
    rig.streamer.stop();
    REQUIRE(wait_for([&] { return rig.done.load() > 0; }));
}

TEST_CASE("Fire arms the laser in G1 mode and stop clears the power", "[GrblStreamer]")
{
    Rig rig;
    auto received = [&](const char* line) {
        std::lock_guard<std::mutex> lk(rig.sim->mutex);
        return std::count(rig.sim->received.begin(), rig.sim->received.end(), std::string(line)) > 0;
    };
    rig.streamer.fire(10, 0);   // 10 % of S1000; travel 100 mm/s
    REQUIRE(wait_for([&] { return received("G1 F6000 M3 S100"); }));
    REQUIRE(wait_for([&] { return rig.streamer.status().spindle == 100; }));
    CHECK_FALSE(rig.streamer.start({"G1 X1 F100"}));   // no job while firing
    rig.streamer.stop_fire();
    REQUIRE(wait_for([&] { return received("M5 S0"); }));
    REQUIRE(wait_for([&] { return rig.streamer.status().spindle == 0; }));
}

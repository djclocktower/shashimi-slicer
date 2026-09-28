#include "GrblStreamer.hpp"
#include "Serial.hpp"

#include <boost/asio.hpp>
#include <boost/log/trivial.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <future>
#include <thread>

namespace Slic3r {

using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

namespace {

std::string trim(const std::string& s)
{
    size_t a = 0, b = s.size();
    while (a < b && std::isspace((unsigned char) s[a])) ++a;
    while (b > a && std::isspace((unsigned char) s[b - 1])) --b;
    return s.substr(a, b - a);
}

bool starts_with(const std::string& s, const char* prefix) { return s.rfind(prefix, 0) == 0; }

std::vector<double> parse_numbers(const std::string& s)
{
    std::vector<double> out;
    const char*         p = s.c_str();
    while (*p) {
        char*  end = nullptr;
        double v   = std::strtod(p, &end);
        if (end == p) break;
        out.push_back(v);
        p = end;
        if (*p == ',') ++p;
    }
    return out;
}

Vec3d to_vec3(const std::vector<double>& n)
{
    return {n.size() > 0 ? n[0] : 0., n.size() > 1 ? n[1] : 0., n.size() > 2 ? n[2] : 0.};
}

// A comment-free, trimmed G-code line ("" for blank/comment-only lines).
std::string strip_gcode(const std::string& line)
{
    std::string out;
    int         paren = 0;
    for (char c : line) {
        if (c == ';' && paren == 0) break;
        if (c == '(') { ++paren; continue; }
        if (c == ')' && paren > 0) { --paren; continue; }
        if (paren == 0 && c != '\r' && c != '\n') out += c;
    }
    return trim(out);
}

// Axis word values of a G-code line ("G1 X10 Y-2.5 F600" -> X=10, Y=-2.5, F=600).
bool word(const std::string& line, char letter, double& value)
{
    for (size_t i = 0; i < line.size(); ++i) {
        if (std::toupper((unsigned char) line[i]) != letter) continue;
        // A letter inside another word ("G10 L20" has no X): must follow a space, start, or a digit.
        if (i > 0 && std::isalpha((unsigned char) line[i - 1])) continue;
        char* end = nullptr;
        value     = std::strtod(line.c_str() + i + 1, &end);
        if (end != line.c_str() + i + 1) return true;
    }
    return false;
}

std::string fmt(const char* f, double v)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), f, v);
    return buf;
}

} // namespace

// ---- Parsers and tables ---------------------------------------------------------------------------

bool parse_grbl_status(const std::string& line_in, GrblStatus& st)
{
    const std::string line = trim(line_in);
    if (line.size() < 3 || line.front() != '<' || line.back() != '>') return false;
    std::vector<std::string> fields;
    {
        std::string body = line.substr(1, line.size() - 2);
        size_t      pos  = 0;
        for (;;) {
            size_t bar = body.find('|', pos);
            fields.push_back(body.substr(pos, bar == std::string::npos ? std::string::npos : bar - pos));
            if (bar == std::string::npos) break;
            pos = bar + 1;
        }
    }
    std::string state = fields[0];
    int         sub   = 0;
    if (size_t c = state.find(':'); c != std::string::npos) {
        sub = std::atoi(state.c_str() + c + 1);
        state.resize(c);
    }
    static const std::pair<const char*, GrblState> names[] = {
        {"Idle", GrblState::Idle},   {"Run", GrblState::Run},     {"Hold", GrblState::Hold},
        {"Jog", GrblState::Jog},     {"Alarm", GrblState::Alarm}, {"Door", GrblState::Door},
        {"Check", GrblState::Check}, {"Home", GrblState::Home},    {"Sleep", GrblState::Sleep}};
    GrblState s = GrblState::Unknown;
    for (const auto& n : names)
        if (state == n.first) s = n.second;
    st.state    = s;
    st.substate = sub;
    if (s != GrblState::Alarm) st.alarm = 0;

    bool  has_m = false, has_w = false, pins = false, acc = false;
    Vec3d m, w;
    for (size_t i = 1; i < fields.size(); ++i) {
        const std::string& f = fields[i];
        size_t             k = f.find(':');
        if (k == std::string::npos) continue;
        const std::string key = f.substr(0, k), val = f.substr(k + 1);
        const auto        n   = parse_numbers(val);
        if (key == "MPos") { m = to_vec3(n); has_m = true; }
        else if (key == "WPos") { w = to_vec3(n); has_w = true; }
        else if (key == "WCO") st.wco = to_vec3(n);
        else if (key == "FS") {
            if (n.size() > 0) st.feed = n[0];
            if (n.size() > 1) st.spindle = n[1];
        } else if (key == "F") {
            if (!n.empty()) st.feed = n[0];
        } else if (key == "Ov" && n.size() >= 3) {
            st.ov_feed = int(n[0]); st.ov_rapid = int(n[1]); st.ov_spindle = int(n[2]);
        } else if (key == "Bf" && n.size() >= 2) {
            st.planner_free = int(n[0]); st.rx_free = int(n[1]);
        } else if (key == "Pn") { st.pins = val; pins = true; }
        else if (key == "A") { st.accessories = val; acc = true; }
    }
    if (!pins) st.pins.clear();
    if (!acc) st.accessories.clear();
    if (has_m) { st.mpos = m; st.wpos = m - st.wco; }
    else if (has_w) { st.wpos = w; st.mpos = w + st.wco; }
    return true;
}

std::string grbl_error_text(int code)
{
    switch (code) {
    case 1: return "The controller did not understand a command (a G-code word is missing its letter).";
    case 2: return "A number in the G-code is badly formatted.";
    case 3: return "The controller does not know this '$' command.";
    case 4: return "A value that must be positive is negative.";
    case 5: return "Homing is not enabled on this controller ($22=0).";
    case 6: return "The step pulse time setting is too short (must be at least 3 microseconds).";
    case 7: return "The controller could not read its settings and restored the defaults.";
    case 8: return "This command only works while the machine is idle.";
    case 9: return "G-code is locked out: the machine is in alarm or jogging. Unlock ($X) or home first.";
    case 10: return "Soft limits need homing to be enabled.";
    case 11: return "A G-code line is too long for the controller.";
    case 12: return "A setting would make the motors step faster than the controller can.";
    case 13: return "The safety door is open.";
    case 14: return "A startup line or build info is too long to store.";
    case 15: return "Jog target is outside the machine's travel; the jog was ignored.";
    case 16: return "Invalid jog command.";
    case 17: return "Laser mode needs the PWM output to be enabled.";
    case 20: return "Unsupported or invalid G-code command.";
    case 21: return "Two commands from the same group are on one G-code line.";
    case 22: return "No feed rate (speed) was set before a cutting move.";
    case 23: return "A G-code command needs a whole number.";
    case 24: return "Two commands on one line both want the axis words.";
    case 25: return "A G-code word is repeated on one line.";
    case 26: return "A command needs axis words (X/Y/Z) but has none.";
    case 27: return "Invalid line number.";
    case 28: return "A G-code command is missing a required value.";
    case 29: return "Work coordinate system G59.x is not supported.";
    case 30: return "G53 only works with G0 and G1.";
    case 31: return "Axis words found where no command uses them.";
    case 32: return "An arc (G2/G3) needs at least one axis word in its plane.";
    case 33: return "The target of a move is invalid.";
    case 34: return "Arc radius error: the arc cannot be drawn with these values.";
    case 35: return "An arc (G2/G3) needs at least one offset word (I/J/K) in its plane.";
    case 36: return "Unused value words on a G-code line.";
    case 37: return "Dynamic tool length offset (G43.1) is not on the tool length axis.";
    case 38: return "Tool number is larger than the controller supports.";
    // grblHAL extras
    case 39: return "A value is out of range.";
    case 45: return "A limit switch is pressed: only homing is allowed until it is cleared.";
    case 46: return "Homing is required before this command. Home the machine ($H).";
    case 50: return "The emergency stop is engaged. Release it and reset.";
    case 51: return "A motor driver reported a fault.";
    case 52: return "A setting value is out of range.";
    case 53: return "This setting is disabled.";
    case 60: return "The SD card could not be mounted.";
    case 61: return "The SD card could not be read.";
    case 62: return "An SD card directory could not be opened.";
    case 63: return "SD card directory not found.";
    case 64: return "The SD card file is empty.";
    default: return "The controller reported error " + std::to_string(code) + " (see its documentation).";
    }
}

std::string grbl_alarm_text(int code)
{
    switch (code) {
    case 1: return "Hard limit hit: a limit switch triggered. The position is probably lost; home the machine again.";
    case 2: return "Soft limit: the job would go past the machine's travel. The position is kept; you can unlock.";
    case 3: return "Reset while moving: the position is probably lost. Home the machine again.";
    case 4: return "Probe fail: the probe was not in the expected state before probing.";
    case 5: return "Probe fail: the probe did not touch within the programmed travel.";
    case 6: return "Homing failed: the homing cycle was reset.";
    case 7: return "Homing failed: the safety door opened during homing.";
    case 8: return "Homing failed: pull-off did not clear the limit switch. Check wiring or increase pull-off.";
    case 9: return "Homing failed: no limit switch found within the search distance. Check wiring and max travel.";
    // grblHAL extras
    case 10: return "Emergency stop engaged. Release it, then reset and unlock.";
    case 11: return "Homing required: home the machine ($H) before running jobs.";
    case 12: return "A limit switch is engaged. Clear it before continuing.";
    case 13: return "Probe protection triggered. Clear it before continuing.";
    case 14: return "The spindle/laser did not report ready in time.";
    case 15: return "Homing failed: the second switch of a squared axis was not found.";
    case 16: return "The controller's power-on self test failed.";
    case 17: return "A motor driver reported a fault.";
    default: return "The controller is in alarm " + std::to_string(code) + " (see its documentation).";
    }
}

const char* grbl_state_name(GrblState state)
{
    switch (state) {
    case GrblState::Disconnected: return "Disconnected";
    case GrblState::Unknown: return "Connecting";
    case GrblState::Idle: return "Idle";
    case GrblState::Run: return "Run";
    case GrblState::Hold: return "Hold";
    case GrblState::Jog: return "Jog";
    case GrblState::Alarm: return "Alarm";
    case GrblState::Door: return "Door";
    case GrblState::Check: return "Check";
    case GrblState::Home: return "Home";
    case GrblState::Sleep: return "Sleep";
    }
    return "?";
}

// ---- Serial transport --------------------------------------------------------------------------

struct SerialTransport::Impl {
    Impl(std::string p, unsigned b) : port(std::move(p)), baud(b) {}
    std::string                    port;
    unsigned                       baud;
    boost::asio::io_context        io;
    std::unique_ptr<Utils::Serial> serial;
    bool                           lost{false};
};

SerialTransport::SerialTransport(std::string port, unsigned baud) : m_impl(std::make_unique<Impl>(std::move(port), baud)) {}
SerialTransport::~SerialTransport() { close(); }

bool SerialTransport::open(std::string* error)
{
    namespace sp = boost::asio;
    try {
        m_impl->serial = std::make_unique<Utils::Serial>(m_impl->io, m_impl->port, m_impl->baud);
        m_impl->serial->set_option(sp::serial_port_base::character_size(8));
        m_impl->serial->set_option(sp::serial_port_base::parity(sp::serial_port_base::parity::none));
        m_impl->serial->set_option(sp::serial_port_base::stop_bits(sp::serial_port_base::stop_bits::one));
        m_impl->serial->set_option(sp::serial_port_base::flow_control(sp::serial_port_base::flow_control::none));
    } catch (const std::exception& e) {
        m_impl->serial.reset();
        if (error)
            *error = "Could not open " + m_impl->port + ": " + e.what() +
                     ". Is another program (LightBurn, a slicer, a serial monitor) using the port?";
        return false;
    }
    m_impl->lost = false;
    return true;
}

void SerialTransport::close()
{
    if (m_impl->serial) {
        boost::system::error_code ec;
        m_impl->serial->close(ec);
        m_impl->serial.reset();
    }
}

bool SerialTransport::is_open() const { return m_impl->serial && m_impl->serial->is_open() && !m_impl->lost; }

bool SerialTransport::write(const std::string& bytes)
{
    if (!is_open()) return false;
    boost::system::error_code ec;
    boost::asio::write(*m_impl->serial, boost::asio::buffer(bytes), ec);
    if (ec) m_impl->lost = true;
    return !ec;
}

std::string SerialTransport::read_available(int timeout_ms)
{
    if (!is_open()) return {};
    char                      buf[1024];
    size_t                    got  = 0;
    bool                      done = false;
    boost::system::error_code ec;
    m_impl->serial->async_read_some(boost::asio::buffer(buf), [&](const boost::system::error_code& e, size_t n) {
        ec   = e;
        got  = n;
        done = true;
    });
    m_impl->io.restart();
    m_impl->io.run_for(milliseconds(timeout_ms));
    if (!done) {
        boost::system::error_code ignored;
        m_impl->serial->cancel(ignored);
        m_impl->io.restart();
        m_impl->io.run();   // runs the aborted handler
    }
    if (ec && ec != boost::asio::error::operation_aborted) m_impl->lost = true;
    return std::string(buf, got);
}

// ---- Simulator ----------------------------------------------------------------------------------

struct GrblSimulator::Impl {
    enum class S { Idle, Run, Hold, Alarm };
    std::shared_ptr<Shared> sh;
    bool                    open{false};
    std::string             rx, out;
    S                       state{S::Idle};
    Vec3d                   mpos{0, 0, 0}, wco{0, 0, 0};
    double                  feed{0}, spindle{0}, s_word{0};
    bool                    relative{false}, laser_on{false}, rapid{true};
    int                     line_no{0};
    Clock::time_point       busy_until{};
    // Timed mode (Shared::time_scale > 0): a 15-block planner executed at feed speed.
    struct Block {
        Vec3d  target;
        double seconds;
    };
    std::deque<Block>       planner;
    Clock::time_point       block_start{};

    bool timed() const { return sh->time_scale.load() > 0; }
    Vec3d planned_end() const { return planner.empty() ? mpos : planner.back().target; }
    // Retires finished blocks; the head position inside the current block is interpolated.
    Vec3d advance()
    {
        if (!timed() || state == S::Hold) return mpos;
        const Clock::time_point now = Clock::now();
        while (!planner.empty()) {
            const auto end = block_start + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(planner.front().seconds));
            if (now < end) {
                const double f = std::chrono::duration<double>(now - block_start).count() / std::max(1e-9, planner.front().seconds);
                return mpos + (planner.front().target - mpos) * std::clamp(f, 0., 1.);
            }
            mpos        = planner.front().target;
            block_start = end;
            planner.pop_front();
        }
        return mpos;
    }
    void status()
    {
        const Clock::time_point now = Clock::now();
        const Vec3d  pos = advance();
        const char* st = state == S::Hold ? "Hold:0" : state == S::Alarm ? "Alarm" :
                         (state == S::Run || now < busy_until || !planner.empty()) ? "Run" : "Idle";
        char buf[256];
        std::snprintf(buf, sizeof(buf), "<%s|MPos:%.3f,%.3f,%.3f|Bf:%d,%d|FS:%.0f,%.0f|WCO:%.3f,%.3f,%.3f>\r\n", st,
                      pos.x(), pos.y(), pos.z(), 15 - int(planner.size()), 128 - int(rx.size()), feed, laser_on ? s_word : 0.,
                      wco.x(), wco.y(), wco.z());
        out += buf;
    }
    void ok() { out += "ok\r\n"; }
    void move(const std::string& line, bool rel)
    {
        double v;
        const Vec3d from   = planned_end();
        Vec3d       target = rel ? from : Vec3d(from - wco);
        bool        any    = false;
        for (int a = 0; a < 3; ++a)
            if (word(line, "XYZ"[a], v)) {
                target[a] = rel ? target[a] + v : v;
                any       = true;
            }
        if (word(line, 'F', v)) feed = v;
        // Modal motion: G0 runs at the rapid rate ($110), G1/G2/G3 at F.
        for (size_t i = 0; i + 1 < line.size(); ++i)
            if ((line[i] == 'G' || line[i] == 'g') && (i == 0 || !std::isalpha((unsigned char) line[i - 1]))) {
                const double g = std::atof(line.c_str() + i + 1);
                if (g == 0) rapid = true;
                else if (g == 1 || g == 2 || g == 3) rapid = false;
            }
        if (!any) return;
        const Vec3d to = rel ? target : Vec3d(target + wco);
        if (timed()) {
            if (planner.empty()) block_start = Clock::now();
            const double speed = std::max(1., (rapid ? 6000. : feed) / 60.) * sh->time_scale.load();
            planner.push_back({to, (to - from).norm() / speed});
        } else {
            mpos       = to;
            busy_until = Clock::now() + milliseconds(20);
        }
    }
    void execute(const std::string& raw)
    {
        const std::string line = trim(raw);
        ++line_no;
        {
            std::lock_guard<std::mutex> lk(sh->mutex);
            sh->received.push_back(line);
        }
        if (sh->fail_on_line.load() == line_no) { out += "error:20\r\n"; return; }
        if (line.empty()) { ok(); return; }
        if (state == S::Alarm && line != "$X" && line != "$H" && line[0] != '$') { out += "error:9\r\n"; return; }
        if (line == "$X") { state = S::Idle; out += "[MSG:Caution: Unlocked]\r\n"; ok(); return; }
        if (line == "$H") { mpos = Vec3d::Zero(); state = S::Idle; ok(); return; }
        if (line == "$$") {
            static const char* settings[] = {"$0=10", "$1=25", "$2=0", "$3=0", "$4=0", "$5=0", "$6=0", "$10=1",
                "$11=0.010", "$12=0.002", "$13=0", "$20=0", "$21=0", "$22=1", "$23=0", "$24=25.000", "$25=500.000",
                "$26=250", "$27=1.000", "$30=1000", "$31=0", "$32=1", "$100=80.000", "$101=80.000", "$102=250.000",
                "$110=6000.000", "$111=6000.000", "$112=500.000", "$120=1000.000", "$121=1000.000", "$122=10.000",
                "$130=400.000", "$131=400.000", "$132=50.000"};
            for (const char* s : settings) out += std::string(s) + "\r\n";
            ok();
            return;
        }
        if (line == "$I") { out += "[VER:1.1h.20190825:Shashimi simulator]\r\n[OPT:VZL,15,128]\r\n"; ok(); return; }
        if (line == "$G") { out += "[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]\r\n"; ok(); return; }
        if (starts_with(line, "$J=")) {   // jogs run at their F and leave the modal state alone
            const bool r = rapid;
            rapid = false;
            move(line.substr(3), line.find("G91") != std::string::npos);
            rapid = r;
            ok();
            return;
        }
        if (line[0] == '$') { ok(); return; }
        if (starts_with(line, "G10") && line.find("L20") != std::string::npos) {
            double v;
            for (int a = 0; a < 3; ++a)
                if (word(line, "XYZ"[a], v)) wco[a] = mpos[a] - v;
            ok();
            return;
        }
        for (char ch : line)
            if (std::isalpha((unsigned char) ch) && std::string("GMXYZFSPIJKLNTgmxyzfspijklnt").find(ch) == std::string::npos) {
                out += "error:20\r\n";
                return;
            }
        if (line.find("G91") != std::string::npos) relative = true;
        if (line.find("G90") != std::string::npos) relative = false;
        double v;
        if (word(line, 'S', v)) s_word = v;
        if (line.find("M3") != std::string::npos || line.find("M4") != std::string::npos) laser_on = true;
        if (line.find("M5") != std::string::npos) laser_on = false;
        move(line, relative);
        ok();
    }
};

GrblSimulator::GrblSimulator(std::shared_ptr<Shared> shared) : m_impl(std::make_unique<Impl>()) { m_impl->sh = std::move(shared); }
GrblSimulator::~GrblSimulator() = default;

bool GrblSimulator::open(std::string*)
{
    m_impl->open = true;
    m_impl->out += "\r\nGrbl 1.1h ['$' for help]\r\n";
    return true;
}
void GrblSimulator::close() { m_impl->open = false; }
bool GrblSimulator::is_open() const { return m_impl->open; }

bool GrblSimulator::write(const std::string& bytes)
{
    Impl& s = *m_impl;
    if (!s.open) return false;
    for (char ch : bytes) {
        const uint8_t c = uint8_t(ch);
        if (c == GrblRt::Status) s.status();
        else if (c == GrblRt::FeedHold) {
            if (s.state != Impl::S::Alarm && s.state != Impl::S::Hold) {
                // The head stops where it is; the current block keeps what is left of its time.
                const Vec3d p = s.advance();
                if (!s.planner.empty())
                    s.planner.front().seconds = std::max(0., s.planner.front().seconds - std::chrono::duration<double>(Clock::now() - s.block_start).count());
                s.mpos  = p;
                s.state = Impl::S::Hold;
            }
        } else if (c == GrblRt::CycleStart) {
            if (s.state == Impl::S::Hold) {
                s.block_start = Clock::now();
                s.state       = Impl::S::Idle;
            }
        }
        else if (c == GrblRt::Reset) {
            const bool moving = s.state == Impl::S::Run || (s.state == Impl::S::Idle && (Clock::now() < s.busy_until || !s.planner.empty()));
            s.mpos = s.advance();
            s.planner.clear();
            s.rx.clear();
            s.laser_on = false;
            s.state    = moving ? Impl::S::Alarm : Impl::S::Idle;
            s.out += "\r\nGrbl 1.1h ['$' for help]\r\n";
            if (moving) s.out += "ALARM:3\r\n[MSG:'$H'|'$X' to unlock]\r\n";
        } else if (c >= 0x80) {
            // overrides / jog cancel: accepted, no effect
        } else {
            s.rx.push_back(ch);
            int n = int(s.rx.size());
            int prev = s.sh->max_rx_bytes.load();
            while (n > prev && !s.sh->max_rx_bytes.compare_exchange_weak(prev, n)) {}
        }
    }
    return true;
}

std::string GrblSimulator::read_available(int timeout_ms)
{
    Impl& s = *m_impl;
    if (s.state != Impl::S::Hold) {
        s.advance();
        for (int i = 0, n = std::max(1, s.sh->lines_per_read.load()); i < n && s.planner.size() < 15; ++i) {
            size_t nl = s.rx.find('\n');
            if (nl == std::string::npos) break;
            std::string line = s.rx.substr(0, nl);
            s.rx.erase(0, nl + 1);
            s.execute(line);
        }
        if (s.state == Impl::S::Run || s.state == Impl::S::Idle)
            s.state = s.rx.find('\n') != std::string::npos || !s.planner.empty() ? Impl::S::Run : Impl::S::Idle;
    }
    if (s.out.empty()) std::this_thread::sleep_for(milliseconds(std::min(timeout_ms, 5)));
    std::string r;
    r.swap(s.out);
    return r;
}

// ---- Streamer ------------------------------------------------------------------------------------

namespace {
constexpr size_t kRxBudget = 127;   // GRBL's 128-byte RX buffer, one byte of slack
constexpr long   kManual   = -1;    // Sent::job_line for console/manual lines
constexpr long   kPoll     = -2;    // internal status poll (Marlin M114): not logged
} // namespace

struct GrblStreamer::Impl {
    struct Sent {
        size_t len;
        long   job_line;
    };

    Callbacks         cb;
    std::thread       thread;
    std::atomic<bool> quit{false}, connected{false};

    mutable std::mutex         mx;   // guards everything down to the worker-only block
    GrblStatus                 status;
    std::deque<std::string>    manual;
    std::string                realtime;
    bool                       stop_request{false};
    std::vector<std::string>   job;
    size_t                     job_next{0};
    JobProgress                prog;
    Clock::time_point          job_start;
    double                     job_estimate{0};
    bool                       firing{false};
    std::deque<ConsoleLine>    console;
    std::map<int, std::string> settings;
    std::string                firmware;
    Laser::LaserDevice         device;

    // Worker only.
    std::unique_ptr<ITransport> tr;
    std::deque<Sent>            inflight;
    size_t                      inflight_bytes{0};
    std::string                 rx;
    Clock::time_point           last_poll{}, last_progress{}, all_acked_at{};
    bool                        banner_seen{false};
    std::string                 lost_reason;

    bool grbl_like() const { return device.type != Laser::DeviceType::Marlin; }
    bool ping_pong() const { return device.type == Laser::DeviceType::Marlin || device.type == Laser::DeviceType::Smoothie; }

    void log(ConsoleLine::Dir dir, const std::string& text)
    {
        ConsoleLine l{dir, std::chrono::system_clock::now(), text};
        {
            std::lock_guard<std::mutex> lk(mx);
            console.push_back(l);
            while (console.size() > kConsoleLines) console.pop_front();
        }
        if (cb.on_console) cb.on_console(l);
    }
    void info(const std::string& text) { log(ConsoleLine::Dir::Info, text); }

    bool write(const std::string& bytes)
    {
        if (tr->write(bytes)) return true;
        if (lost_reason.empty()) lost_reason = "The connection to the laser was lost (cable unplugged or controller powered off?).";
        return false;
    }

    JobProgress progress_now()
    {
        std::lock_guard<std::mutex> lk(mx);
        if (prog.running) {
            prog.elapsed_s = std::chrono::duration<double>(Clock::now() - job_start).count();
            const double left = prog.lines_total > 0 ? double(prog.lines_total - prog.lines_acked) / prog.lines_total : 0;
            if (job_estimate > 0) prog.eta_s = job_estimate * left;
            else if (prog.lines_acked > 0) prog.eta_s = prog.elapsed_s * double(prog.lines_total - prog.lines_acked) / prog.lines_acked;
            else prog.eta_s = 0;
        }
        return prog;
    }
    void report_progress()
    {
        last_progress = Clock::now();
        JobProgress p = progress_now();
        if (cb.on_progress) cb.on_progress(p);
    }

    void finish_job(bool completed, const std::string& reason)
    {
        {
            std::lock_guard<std::mutex> lk(mx);
            if (!prog.running) return;
        }
        JobProgress p = progress_now();
        {
            std::lock_guard<std::mutex> lk(mx);
            prog.running = prog.paused = false;
            p            = prog;
            job.clear();
            job_next = 0;
        }
        info(completed ? "Job finished." : "Job ended: " + reason);
        if (cb.on_progress) cb.on_progress(p);
        if (cb.on_job_done) cb.on_job_done(completed, reason);
    }

    void drop_inflight()
    {
        inflight.clear();
        inflight_bytes = 0;
    }

    void handle_line(const std::string& raw)
    {
        const std::string l = trim(raw);
        if (l.empty()) return;
        if (l.front() == '<') {
            GrblStatus copy;
            {
                std::lock_guard<std::mutex> lk(mx);
                parse_grbl_status(l, status);
                copy = status;
            }
            if (cb.on_status) cb.on_status(copy);
            return;
        }
        if (l == "ok" || starts_with(l, "error")) {
            Sent s{0, kManual};
            if (!inflight.empty()) {
                s = inflight.front();
                inflight.pop_front();
                inflight_bytes -= std::min(inflight_bytes, s.len);
            }
            if (l == "ok") {
                if (s.job_line >= 0) {
                    std::lock_guard<std::mutex> lk(mx);
                    ++prog.lines_acked;
                } else if (s.job_line == kManual)
                    log(ConsoleLine::Dir::Rx, l);
                return;
            }
            const size_t colon = l.find(':');
            const int    code  = colon == std::string::npos ? 0 : std::atoi(l.c_str() + colon + 1);
            const std::string text = grbl_error_text(code);
            log(ConsoleLine::Dir::Rx, l + "  (" + text + ")");
            if (s.job_line >= 0) {
                {
                    std::lock_guard<std::mutex> lk(mx);
                    ++prog.lines_acked;
                    prog.paused = true;
                }
                if (grbl_like()) write(std::string(1, char(GrblRt::FeedHold)));
                info("Job paused at line " + std::to_string(s.job_line + 1) + ". Resume to skip the line, or Stop.");
                if (cb.on_error) cb.on_error(text, int(s.job_line + 1));
                report_progress();
            } else if (cb.on_error)
                cb.on_error(text, -1);
            return;
        }
        if (starts_with(l, "ALARM:")) {
            const int code = std::atoi(l.c_str() + 6);
            GrblStatus copy;
            {
                std::lock_guard<std::mutex> lk(mx);
                status.alarm = code;
                status.state = GrblState::Alarm;
                copy         = status;
            }
            const std::string text = grbl_alarm_text(code);
            log(ConsoleLine::Dir::Rx, l + "  (" + text + ")");
            // The controller flushed its buffers: the lines in flight will never be acked.
            drop_inflight();
            if (cb.on_status) cb.on_status(copy);
            if (cb.on_error) cb.on_error(text, -1);
            finish_job(false, text);
            return;
        }
        if (starts_with(l, "Grbl") || starts_with(l, "GrblHAL") || starts_with(l, "Smoothie")) {
            banner_seen = true;
            {
                std::lock_guard<std::mutex> lk(mx);
                firmware = l;
            }
            drop_inflight();   // a reset (ours or the controller's) empties its RX buffer
            log(ConsoleLine::Dir::Rx, l);
            finish_job(false, "The controller reset.");
            return;
        }
        if (starts_with(l, "start") && !grbl_like()) banner_seen = true;   // Marlin boot
        if (starts_with(l, "[VER:")) {
            std::lock_guard<std::mutex> lk(mx);
            firmware += (firmware.empty() ? "" : " ") + l;
        }
        if (l.size() > 2 && l[0] == '$' && std::isdigit((unsigned char) l[1])) {
            const size_t eq = l.find('=');
            if (eq != std::string::npos) {
                std::lock_guard<std::mutex> lk(mx);
                settings[std::atoi(l.c_str() + 1)] = trim(l.substr(eq + 1));
            }
        }
        // Marlin M114: "X:10.00 Y:20.00 Z:0.00 E:0.00 Count X:..."
        if (!grbl_like() && starts_with(l, "X:")) {
            GrblStatus copy;
            {
                std::lock_guard<std::mutex> lk(mx);
                auto n = [&](char axis) {
                    size_t p = l.find(std::string(1, axis) + ":");
                    return p == std::string::npos ? 0. : std::atof(l.c_str() + p + 2);
                };
                status.wpos = status.mpos = Vec3d(n('X'), n('Y'), n('Z'));
                status.state = prog.running ? (prog.paused ? GrblState::Hold : GrblState::Run) : GrblState::Idle;
                copy         = status;
            }
            if (cb.on_status) cb.on_status(copy);
            if (!inflight.empty() && inflight.front().job_line == kPoll) return;   // our own poll: not logged
        }
        log(ConsoleLine::Dir::Rx, l);
    }

    void read_and_dispatch(int timeout_ms)
    {
        rx += tr->read_available(timeout_ms);
        size_t nl;
        while ((nl = rx.find_first_of("\n")) != std::string::npos) {
            std::string line = rx.substr(0, nl);
            rx.erase(0, nl + 1);
            handle_line(line);
        }
    }

    // Reads and dispatches for up to `ms`, polling status, until `done` returns true.
    void pump(int ms, const std::function<bool()>& done)
    {
        const auto end = Clock::now() + milliseconds(ms);
        auto       poll = Clock::now() - milliseconds(1000);
        while (Clock::now() < end && !quit && tr->is_open()) {
            if (grbl_like() && Clock::now() - poll >= milliseconds(100)) {
                write("?");
                poll = Clock::now();
            }
            read_and_dispatch(20);
            if (done && done()) return;
        }
    }

    void handshake()
    {
        info("Waiting for the controller to start...");
        banner_seen = false;
        pump(3000, [this] { return banner_seen; });
        if (!banner_seen && grbl_like()) {
            info("No greeting from the controller; sending a soft reset.");
            write(std::string(1, char(GrblRt::Reset)));
            pump(2000, [this] { return banner_seen; });
        }
        if (!banner_seen) info("The controller did not introduce itself; continuing anyway.");
        if (device.type == Laser::DeviceType::Grbl || device.type == Laser::DeviceType::GrblHal) {
            std::lock_guard<std::mutex> lk(mx);
            manual.push_back("$I");
        }
    }

    void do_stop()
    {
        info("Stopping.");
        if (grbl_like()) {
            write(std::string(1, char(GrblRt::FeedHold)));
            // Reset only once the hold has stopped the motion, so the position is kept.
            pump(1500, [this] {
                std::lock_guard<std::mutex> lk(mx);
                return (status.state == GrblState::Hold && status.substate == 0) || status.state == GrblState::Idle ||
                       status.state == GrblState::Alarm;
            });
            {
                std::lock_guard<std::mutex> lk(mx);
                manual.clear();
                job.clear();
                job_next = 0;
            }
            finish_job(false, "Stopped.");
            banner_seen = false;
            write(std::string(1, char(GrblRt::Reset)));
            drop_inflight();
            pump(2000, [this] { return banner_seen; });
            std::lock_guard<std::mutex> lk(mx);
            manual.push_back("$X");
            firing = false;
        } else {
            {
                std::lock_guard<std::mutex> lk(mx);
                manual.clear();
                job.clear();
                job_next = 0;
                firing   = false;
            }
            drop_inflight();
            write("M410\nM5\n");   // Marlin quick stop, laser off
            finish_job(false, "Stopped.");
        }
    }

    void fill()
    {
        for (;;) {
            std::string line;
            long        job_line = kManual;
            {
                std::lock_guard<std::mutex> lk(mx);
                if (!manual.empty()) line = manual.front();
                else if (prog.running && !prog.paused && job_next < job.size()) {
                    line     = job[job_next];
                    job_line = long(job_next);
                } else
                    break;
                const size_t len = line.size() + 1;
                if (!inflight.empty() && (ping_pong() || inflight_bytes + len > kRxBudget)) break;
                if (job_line == kManual) manual.pop_front();
                else prog.lines_sent = ++job_next;
            }
            if (!write(line + "\n")) return;
            inflight.push_back({line.size() + 1, job_line});
            inflight_bytes += line.size() + 1;
            if (job_line == kManual) log(ConsoleLine::Dir::Tx, line);
        }
    }

    void step()
    {
        std::string rt;
        bool        stop;
        {
            std::lock_guard<std::mutex> lk(mx);
            rt.swap(realtime);
            stop         = stop_request;
            stop_request = false;
        }
        if (!rt.empty() && grbl_like()) write(rt);
        if (stop) do_stop();

        const auto now = Clock::now();
        if (grbl_like()) {
            if (now - last_poll >= milliseconds(200)) {
                write("?");
                last_poll = now;
            }
        } else if (now - last_poll >= milliseconds(1000) && inflight.empty()) {
            bool idle;
            {
                std::lock_guard<std::mutex> lk(mx);
                idle = manual.empty() && !prog.running;
            }
            if (idle && write("M114\n")) {
                inflight.push_back({5, kPoll});
                inflight_bytes += 5;
            }
            last_poll = now;
        }

        fill();
        read_and_dispatch(inflight.empty() ? 20 : 5);

        // Job completion: every line acked, then (GRBL) the machine back to Idle after a later poll.
        bool running, all_acked, idle;
        {
            std::lock_guard<std::mutex> lk(mx);
            running   = prog.running;
            all_acked = running && job_next == job.size() && prog.lines_acked >= prog.lines_total;
            idle      = status.state == GrblState::Idle;
        }
        if (!running) return;
        if (all_acked) {
            if (all_acked_at == Clock::time_point{}) all_acked_at = Clock::now();
            if (!grbl_like() || (idle && Clock::now() - all_acked_at > milliseconds(450))) {
                all_acked_at = {};
                finish_job(true, "");
                return;
            }
        } else
            all_acked_at = {};
        if (Clock::now() - last_progress >= milliseconds(250)) report_progress();
    }

    void run(std::promise<std::string> opened)
    {
        std::string err;
        if (!tr->open(&err)) {
            opened.set_value(err.empty() ? "Could not open the connection." : err);
            return;
        }
        connected = true;
        {
            std::lock_guard<std::mutex> lk(mx);
            status       = GrblStatus{};
            status.state = GrblState::Unknown;
        }
        opened.set_value("");
        lost_reason.clear();
        rx.clear();
        drop_inflight();
        handshake();
        if (cb.on_connection) cb.on_connection(true, "");
        while (!quit && tr->is_open() && lost_reason.empty()) step();
        if (lost_reason.empty() && !tr->is_open())
            lost_reason = "The connection to the laser was lost (cable unplugged or controller powered off?).";
        {
            std::lock_guard<std::mutex> lk(mx);
            if (firing && lost_reason.empty()) tr->write("M5\n");
            firing = false;
        }
        finish_job(false, lost_reason.empty() ? "Disconnected." : lost_reason);
        tr->close();
        connected = false;
        GrblStatus copy;
        {
            std::lock_guard<std::mutex> lk(mx);
            status.state = GrblState::Disconnected;
            copy         = status;
            manual.clear();
            realtime.clear();
        }
        info(lost_reason.empty() ? "Disconnected." : lost_reason);
        if (cb.on_status) cb.on_status(copy);
        if (cb.on_connection) cb.on_connection(false, lost_reason);
    }

    void queue(const std::string& line)
    {
        std::lock_guard<std::mutex> lk(mx);
        manual.push_back(line);
    }
};

GrblStreamer::GrblStreamer() : m_impl(std::make_unique<Impl>()) {}
GrblStreamer::~GrblStreamer() { disconnect(); }

void GrblStreamer::set_callbacks(Callbacks callbacks)
{
    if (!is_connected() && !m_impl->thread.joinable()) m_impl->cb = std::move(callbacks);
}

bool GrblStreamer::connect(std::unique_ptr<ITransport> transport, const Laser::LaserDevice& device, std::string* error)
{
    disconnect();
    if (device.type == Laser::DeviceType::Ruida) {
        if (error) *error = "Ruida controllers are not supported yet.";
        return false;
    }
    Impl& d = *m_impl;
    d.tr     = std::move(transport);
    d.device = device;
    d.quit   = false;
    {
        std::lock_guard<std::mutex> lk(d.mx);
        d.settings.clear();
        d.firmware.clear();
        d.prog = JobProgress{};
    }
    std::promise<std::string> opened;
    auto                      result = opened.get_future();
    d.thread = std::thread([&d, p = std::move(opened)]() mutable { d.run(std::move(p)); });
    const std::string err = result.get();
    if (!err.empty()) {
        d.thread.join();
        d.tr.reset();
        if (error) *error = err;
        return false;
    }
    return true;
}

void GrblStreamer::disconnect()
{
    m_impl->quit = true;
    if (m_impl->thread.joinable()) m_impl->thread.join();
    m_impl->tr.reset();
}

bool GrblStreamer::is_connected() const { return m_impl->connected; }

GrblStatus GrblStreamer::status() const
{
    std::lock_guard<std::mutex> lk(m_impl->mx);
    return m_impl->status;
}

bool GrblStreamer::send_line(const std::string& line)
{
    const std::string l = trim(line);
    if (!is_connected() || l.empty()) return false;
    std::lock_guard<std::mutex> lk(m_impl->mx);
    if (m_impl->prog.running) return false;
    m_impl->manual.push_back(l);
    return true;
}

void GrblStreamer::send_realtime(uint8_t byte)
{
    if (!is_connected()) return;
    std::lock_guard<std::mutex> lk(m_impl->mx);
    m_impl->realtime.push_back(char(byte));
}

void GrblStreamer::home() { send_line(m_impl->grbl_like() ? "$H" : "G28 X Y"); }
void GrblStreamer::unlock()
{
    if (m_impl->grbl_like()) send_line("$X");
}

void GrblStreamer::jog(double dx, double dy, double dz, double speed_mm_s)
{
    std::string axes;
    if (dx != 0) axes += " X" + fmt("%.3f", dx);
    if (dy != 0) axes += " Y" + fmt("%.3f", dy);
    if (dz != 0) axes += " Z" + fmt("%.3f", dz);
    if (axes.empty()) return;
    const std::string f = " F" + fmt("%.0f", std::max(1., speed_mm_s * 60.));
    if (m_impl->grbl_like()) send_line("$J=G91 G21" + axes + f);
    else if (send_line("G91")) {
        send_line("G0" + axes + f);
        send_line("G90");
    }
}

void GrblStreamer::jog_cancel()
{
    if (m_impl->grbl_like()) send_realtime(GrblRt::JogCancel);
}

void GrblStreamer::set_origin() { send_line(m_impl->grbl_like() ? "G10 L20 P1 X0 Y0" : "G92 X0 Y0"); }

void GrblStreamer::fire(double power_pct, int ms)
{
    const double s = std::clamp(power_pct, 0., 100.) * m_impl->device.s_max / 100.;
    if (!send_line("M3 S" + fmt("%.0f", s))) return;
    if (ms > 0) {
        send_line("G4 P" + fmt("%.3f", ms / 1000.));
        send_line("M5");
    } else {
        std::lock_guard<std::mutex> lk(m_impl->mx);
        m_impl->firing = true;
    }
}

void GrblStreamer::stop_fire()
{
    {
        std::lock_guard<std::mutex> lk(m_impl->mx);
        m_impl->firing = false;
    }
    send_line("M5");
}

bool GrblStreamer::start(std::vector<std::string> lines, double estimated_s)
{
    std::vector<std::string> job;
    job.reserve(lines.size());
    for (const std::string& l : lines)
        if (std::string s = strip_gcode(l); !s.empty()) job.push_back(std::move(s));
    if (!is_connected() || job.empty()) return false;
    {
        std::lock_guard<std::mutex> lk(m_impl->mx);
        const GrblState st = m_impl->status.state;
        if (m_impl->prog.running || !m_impl->manual.empty()) return false;
        if (m_impl->grbl_like() ? st != GrblState::Idle : (st != GrblState::Idle && st != GrblState::Unknown)) return false;
        m_impl->prog              = JobProgress{};
        m_impl->prog.running      = true;
        m_impl->prog.lines_total  = job.size();
        m_impl->job               = std::move(job);
        m_impl->job_next          = 0;
        m_impl->job_start         = Clock::now();
        m_impl->job_estimate      = estimated_s;
    }
    m_impl->info("Job started: " + std::to_string(lines.size()) + " lines.");
    return true;
}

void GrblStreamer::pause()
{
    {
        std::lock_guard<std::mutex> lk(m_impl->mx);
        if (!m_impl->prog.running) return;
        m_impl->prog.paused = true;
    }
    send_realtime(GrblRt::FeedHold);
}

void GrblStreamer::resume()
{
    {
        std::lock_guard<std::mutex> lk(m_impl->mx);
        m_impl->prog.paused = false;
    }
    send_realtime(GrblRt::CycleStart);
}

void GrblStreamer::stop()
{
    if (!is_connected()) return;
    std::lock_guard<std::mutex> lk(m_impl->mx);
    m_impl->stop_request = true;
}

JobProgress GrblStreamer::progress() const
{
    std::lock_guard<std::mutex> lk(m_impl->mx);
    return m_impl->prog;
}

std::vector<ConsoleLine> GrblStreamer::console() const
{
    std::lock_guard<std::mutex> lk(m_impl->mx);
    return {m_impl->console.begin(), m_impl->console.end()};
}

void GrblStreamer::clear_console()
{
    std::lock_guard<std::mutex> lk(m_impl->mx);
    m_impl->console.clear();
}

std::map<int, std::string> GrblStreamer::settings() const
{
    std::lock_guard<std::mutex> lk(m_impl->mx);
    return m_impl->settings;
}

std::string GrblStreamer::firmware() const
{
    std::lock_guard<std::mutex> lk(m_impl->mx);
    return m_impl->firmware;
}

} // namespace Slic3r

#pragma once

// Laser controller streaming (GRBL 1.1 / grblHAL; Marlin and Smoothie through the same line/ok
// protocol). No wx: every callback runs on the streamer's own thread and the GUI marshals it with
// CallAfter.
//
// Protocol (GRBL): character counting against the 128-byte RX buffer (at most 127 bytes in flight),
// one `ok` / `error:N` per line; `?` status poll every 200 ms parsed into GrblStatus; realtime bytes
// bypass the queue. Marlin: no `?`, M114 polling; Smoothie: `?` like GRBL.

#include "libslic3r/Point.hpp"
#include "libslic3r/Laser/LaserTypes.hpp"

#include <chrono>
#include <cstdint>
#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace Slic3r {

// Byte transport. Implementations are used from the streamer thread only.
class ITransport {
public:
    virtual ~ITransport() = default;
    virtual bool        open(std::string* error) = 0;
    virtual void        close() = 0;
    virtual bool        is_open() const = 0;
    // All bytes or false (connection lost).
    virtual bool        write(const std::string& bytes) = 0;
    // Whatever arrived, waiting at most timeout_ms for the first byte; empty on timeout.
    virtual std::string read_available(int timeout_ms) = 0;
};

// Over Utils::Serial (src/slic3r/Utils/Serial.hpp); port list from Utils::scan_serial_ports().
// ponytail: serial only in v1; TCP (grblHAL telnet) / WebSocket (FluidNC, ESP3D) transports slot in
// behind ITransport when wanted.
class SerialTransport : public ITransport {
public:
    SerialTransport(std::string port, unsigned baud);
    ~SerialTransport() override;
    bool        open(std::string* error) override;
    void        close() override;
    bool        is_open() const override;
    bool        write(const std::string& bytes) override;
    std::string read_available(int timeout_ms) override;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

// A fake GRBL 1.1 controller behind ITransport, for the GUI's "Simulator" device and the tests:
// 128-byte RX buffer (overflow recorded), `ok` per line, `?` status, `!` `~` 0x18, $$ $I $X $H $J=,
// G0/G1 positions, G10 L20; error:20 for unknown words. Takes `lines_per_read` lines per
// read_available() call (none while held), so the streamer really has to wait for acks.
class GrblSimulator : public ITransport {
public:
    // Readable from any thread while the simulator runs.
    struct Shared {
        std::atomic<int> max_rx_bytes{0};    // largest RX buffer fill seen (GRBL: must stay <= 128)
        std::atomic<int> fail_on_line{0};    // > 0: answer error:20 to that line (1-based, all lines)
        std::atomic<int> lines_per_read{8};
        std::atomic<int> resets{0};          // soft resets (0x18) received
        // > 0: moves take distance / feed / time_scale (15-block planner, like the real thing);
        // 0: they complete at once.
        std::atomic<double> time_scale{0};
        std::mutex               mutex;
        std::vector<std::string> received;   // every line, in arrival order
    };
    explicit GrblSimulator(std::shared_ptr<Shared> shared = std::make_shared<Shared>());
    ~GrblSimulator() override;
    bool        open(std::string* error) override;
    void        close() override;
    bool        is_open() const override;
    bool        write(const std::string& bytes) override;
    std::string read_available(int timeout_ms) override;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

enum class GrblState { Disconnected, Unknown, Idle, Run, Hold, Jog, Alarm, Door, Check, Home, Sleep };

struct GrblStatus {
    GrblState   state{GrblState::Disconnected};
    int         substate{0};         // Hold:0/1, Door:0..3
    Vec3d       mpos{0, 0, 0};       // machine position; mpos = wpos + wco
    Vec3d       wpos{0, 0, 0};
    Vec3d       wco{0, 0, 0};        // last reported work coordinate offset
    double      feed{0};             // mm/min
    double      spindle{0};          // S
    int         ov_feed{100}, ov_rapid{100}, ov_spindle{100};   // %
    int         planner_free{-1}, rx_free{-1};                    // Bf:, -1 = not reported
    std::string pins;                // Pn: (cleared when a report has none)
    std::string accessories;         // A: (S/C spindle, F flood, M mist; cleared when absent)
    int         alarm{0};            // last ALARM:n, 0 = none
};

// Parses one `<...>` status report into `status` (fields not in the report are kept, wpos/mpos
// completed from wco). False when `line` is not a status report.
bool        parse_grbl_status(const std::string& line, GrblStatus& status);
// GRBL 1.1 tables, plain language ("Soft limit: the job would go past the machine's travel.").
std::string grbl_error_text(int code);
std::string grbl_alarm_text(int code);
const char* grbl_state_name(GrblState state);

// Realtime bytes (GRBL 1.1).
namespace GrblRt {
    constexpr uint8_t Status = '?', CycleStart = '~', FeedHold = '!', Reset = 0x18;
    constexpr uint8_t SafetyDoor = 0x84, JogCancel = 0x85;
    constexpr uint8_t FeedOv100 = 0x90, FeedOvPlus10 = 0x91, FeedOvMinus10 = 0x92, FeedOvPlus1 = 0x93, FeedOvMinus1 = 0x94;
    constexpr uint8_t RapidOv100 = 0x95, RapidOv50 = 0x96, RapidOv25 = 0x97;
    constexpr uint8_t PowerOv100 = 0x99, PowerOvPlus10 = 0x9A, PowerOvMinus10 = 0x9B, PowerOvPlus1 = 0x9C, PowerOvMinus1 = 0x9D;
    constexpr uint8_t SpindleStop = 0x9E, FloodToggle = 0xA0, MistToggle = 0xA1;
} // namespace GrblRt

struct ConsoleLine {
    enum class Dir { Tx, Rx, Info };
    Dir                                   dir{Dir::Info};
    std::chrono::system_clock::time_point time;
    std::string                           text;   // status polls and their replies are not logged
};

struct JobProgress {
    bool   running{false};
    bool   paused{false};
    size_t lines_total{0};
    size_t lines_sent{0};
    size_t lines_acked{0};
    double elapsed_s{0};
    double eta_s{0};             // from the ack rate; 0 = unknown
};

class GrblStreamer {
public:
    struct Callbacks {
        std::function<void(const GrblStatus&)>                    on_status;       // each poll reply
        std::function<void(const ConsoleLine&)>                   on_console;
        std::function<void(bool connected, const std::string& error)> on_connection;
        std::function<void(const JobProgress&)>                   on_progress;
        // Plain-language error/alarm text (already decoded); `line` = job line number or -1.
        std::function<void(const std::string& text, int line)>    on_error;
        // Job ended: completed, or stopped/failed with the reason.
        std::function<void(bool completed, const std::string& reason)> on_job_done;
    };

    GrblStreamer();
    ~GrblStreamer();   // disconnects
    GrblStreamer(const GrblStreamer&) = delete;
    GrblStreamer& operator=(const GrblStreamer&) = delete;

    // Callbacks may only be replaced while disconnected.
    void set_callbacks(Callbacks callbacks);

    // Takes the transport, opens it on the streamer thread, waits for the GRBL banner (soft reset if
    // none arrives) and starts polling. `device` gives the protocol (type) and s_max for fire().
    bool connect(std::unique_ptr<ITransport> transport, const Laser::LaserDevice& device, std::string* error = nullptr);
    void disconnect();
    bool is_connected() const;
    GrblStatus status() const;

    // ---- Commands (thread-safe) -----------------------------------------------------------------
    // Queued console/manual line; refused (false) while a job runs.
    bool send_line(const std::string& line);
    void send_realtime(uint8_t byte);
    void home();                                         // $H
    void unlock();                                       // $X
    void jog(double dx, double dy, double dz, double speed_mm_s);   // $J=G91 G21 X Y Z F
    void jog_cancel();                                   // 0x85
    void set_origin();                                   // G10 L20 P1 X0 Y0
    // Low-power pulse (0 ms = until stop_fire). GRBL: `G1 F.. M3 S..` (laser mode ignores M3 in G0
    // mode), G4 P.., `M5 S0`; Marlin: M3 S.. / M5.
    void fire(double power_pct, int ms);
    void stop_fire();

    // ---- Job control ----------------------------------------------------------------------------
    // Streams the lines (comments/blank stripped); false when not connected or not Idle. Framing is
    // a job too (Laser::frame_gcode). estimated_s > 0 (LaserJob::estimated_time_s): the ETA is that
    // estimate scaled by the lines left; otherwise it comes from the ack rate.
    bool        start(std::vector<std::string> lines, double estimated_s = 0);
    void        pause();     // feed hold
    void        resume();    // cycle start
    void        stop();      // feed hold, soft reset, $X; queue dropped
    JobProgress progress() const;

    // ---- Console ring buffer (last kConsoleLines lines) -------------------------------------------
    static constexpr size_t kConsoleLines = 2000;
    std::vector<ConsoleLine> console() const;
    void                     clear_console();

    // ---- Controller info ------------------------------------------------------------------------
    // `$N=value` replies seen so far (send_line("$$") refreshes them), keyed by N.
    std::map<int, std::string> settings() const;
    // Banner and `[VER:...]` text of the connected controller.
    std::string                firmware() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace Slic3r

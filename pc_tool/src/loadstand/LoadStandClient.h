#pragma once

// -----------------------------------------------------------------------------
// LoadStandClient — telemetry/command link to the loading-stand STM32 (the
// brake controller) over a plain TCP socket. The Nucleo is hosted on a
// Raspberry Pi; a small serial<->TCP bridge there (tools/uart_bridge.py) exposes
// the stand's UART console as a TCP port. This client connects to that port,
// requests the `DATA,...` stream, parses it into time-series buffers for the
// torque plot, and can send CLI commands (set_m, vt off, ...).
//
// DATA line format (firmware Cmd_StreamTask, hi-res):
//   DATA,<tick_ms>,<set_mA>,<I_mA>,<setM_mNm>,<M_mNm>,<duty%>,<dir>
// torque fields are milli-Nm (x1000); dir = -1/0/+1 (drive direction).
// -----------------------------------------------------------------------------

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#endif

namespace drivescope {

class LoadStandClient {
public:
    ~LoadStandClient();

    // Connect to the Pi TCP bridge (e.g. host="192.168.0.104", port=5555).
    // On success starts the reader thread and asks the stand to stream at 50 Hz.
    bool open(const std::string& host, int port);
    void close();
    bool isOpen() const { return opened_.load(); }
    std::string status();                       // thread-safe copy

    // Send a CLI line to the stand (a trailing '\r' is appended).
    bool sendLine(const std::string& line);

    // Plotting buffers + last values. Copied out under the data lock.
    struct Snapshot {
        std::vector<double> t;         // seconds since first sample
        std::vector<double> torque;    // measured torque [Nm]
        std::vector<double> setpoint;  // torque setpoint [Nm]
        std::vector<double> current;   // coil current [A]
        std::vector<double> duty;      // PWM duty [%]
        double lastTorque   = 0.0;
        double lastSetpoint = 0.0;
        double lastCurrent  = 0.0;
        double lastDuty     = 0.0;
        int    lastDir      = 0;       // -1 NEG / 0 ARMED / +1 POS
        size_t samples      = 0;
        std::string lastText;          // last non-DATA console line (status etc.)
    };
    void snapshot(Snapshot& out);
    void clearData();

    size_t maxPoints = 120000;         // ring cap (~40 min @50 Hz)

private:
    void readerLoop();
    void consumeRx();
    void pushSample(long ms, long set_mA, long I_mA,
                    long setM_mNm, long M_mNm, long duty, int dir);
    bool rawSend(const std::string& text);

    std::thread        reader_;
    std::atomic<bool>  stop_{false};
    std::atomic<bool>  opened_{false};
    std::mutex         ioMutex_;       // guards socket writes
    std::mutex         dataMutex_;     // guards status_ + buf_
    std::string        rx_;            // reader-thread accumulation buffer
    std::string        status_ = "closed";
    Snapshot           buf_;
    double             t0ms_ = -1.0;   // first tick_ms -> time origin

#ifdef _WIN32
    SOCKET sock_ = INVALID_SOCKET;
#endif
};

} // namespace drivescope

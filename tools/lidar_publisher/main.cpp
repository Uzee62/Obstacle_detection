// tools/lidar_publisher/main.cpp
//
// Slamtec RPLidar scan publisher — wraps the official C++ SDK and
// emits one full 360° revolution per "frame" on stdout in a simple
// text protocol consumed by `maritime_perception.sensors.lidar.driver`.
//
// Wire protocol (one frame):
//   SCAN <monotonic_ns>
//   <angle_deg> <distance_m> <quality>
//   <angle_deg> <distance_m> <quality>
//   ...
//   END
//
// Notes:
//   * Only valid measurements (distance > 0 AND quality > 0) are emitted,
//     mirroring the filtering that the previous direct-serial driver did.
//   * Timestamp is CLOCK_MONOTONIC nanoseconds — same clock Python's
//     time.monotonic_ns() uses, so downstream timestamps align.
//   * Stderr is forwarded to the Python logger as INFO; stdout is
//     scan data only — never log to stdout.
//   * Exit on SIGINT/SIGTERM or stdin EOF (Python closes stdin to
//     request a clean shutdown).

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;

static std::atomic<bool> g_running{true};

static void on_signal(int) { g_running.store(false); }

static uint64_t monotonic_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

// Returns true if stdin has hit EOF — Python uses this as our shutdown
// signal. Non-blocking poll, so it costs almost nothing per loop.
static bool stdin_closed() {
    char c;
    ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n == 0) return true;                       // EOF
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return true;
    return false;
}

int main(int argc, const char *argv[]) {
    if (argc < 3) {
        fprintf(stderr,
                "usage: %s <serial_port> <baudrate>\n"
                "example: %s /dev/ttyUSB0 1000000\n",
                argv[0], argv[0]);
        return 2;
    }
    const char* port     = argv[1];
    uint32_t    baudrate = (uint32_t)std::atoi(argv[2]);

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    // Make stdin non-blocking so we can poll for EOF without stalling.
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags != -1) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    sl_result res;

    ILidarDriver* drv = *createLidarDriver();
    if (!drv) {
        fprintf(stderr, "createLidarDriver failed\n");
        return 1;
    }

    IChannel* channel = *createSerialPortChannel(port, baudrate);
    if (SL_IS_FAIL((res = drv->connect(channel)))) {
        fprintf(stderr, "connect failed: 0x%08x\n", res);
        delete drv;
        return 1;
    }

    sl_lidar_response_device_info_t info;
    if (SL_IS_OK((res = drv->getDeviceInfo(info)))) {
        fprintf(stderr,
                "device: model=%u fw=%u.%02u hw=%u sn=",
                info.model,
                info.firmware_version >> 8,
                info.firmware_version & 0xFF,
                info.hardware_version);
        for (int i = 0; i < 16; ++i) fprintf(stderr, "%02X", info.serialnum[i]);
        fputc('\n', stderr);
    } else {
        fprintf(stderr, "getDeviceInfo failed (0x%08x); continuing\n", res);
    }

    sl_lidar_response_device_health_t health;
    if (SL_IS_OK((res = drv->getHealth(health)))) {
        fprintf(stderr, "health: status=%u err=0x%04x\n",
                health.status, health.error_code);
    } else {
        fprintf(stderr, "getHealth failed (0x%08x); continuing anyway\n", res);
    }

    drv->setMotorSpeed();                          // default RPM

    // The S2 motor needs ~1–2 s to reach operating speed after a fresh
    // port open. startScan will time out if it lands while the motor
    // is still spinning up — see error 0x80008002. Retry in-process so
    // we don't tear the port down and force a cold motor restart, and
    // call stop() between attempts to clear any half-started state in
    // the device's firmware.
    fprintf(stderr, "waiting for motor to reach speed...\n");  
    usleep(3000 * 1000);   // 3 sec to give motor time warm up before first attempt, which is usually enough to avoid a retry at all
    const int    START_RETRIES_MAX = 6;
    const useconds_t START_RETRY_GAP_US = 500 * 1000;   // 0.5 s
    for (int attempt = 1; ; ++attempt) {
        drv->stop();                               // clear any pending state
        usleep(50 * 1000);                         // brief settle
        res = drv->startScan(false, true);
        if (SL_IS_OK(res)) {
            fprintf(stderr, "scanning started (attempt %d)\n", attempt);
            break;
        }
        fprintf(stderr,
                "startScan failed: 0x%08x (attempt %d/%d)\n",
                res, attempt, START_RETRIES_MAX);
        if (attempt >= START_RETRIES_MAX) {
            drv->stop();
            drv->setMotorSpeed(0);
            drv->disconnect();
            delete drv;
            return 1;
        }
        usleep(START_RETRY_GAP_US);
    }

    static sl_lidar_response_measurement_node_hq_t nodes[8192];
    int consecutive_failures = 0;

    while (g_running.load()) {
        if (stdin_closed()) {
            fprintf(stderr, "stdin closed; shutting down\n");
            break;
        }

        size_t count = sizeof(nodes) / sizeof(nodes[0]);
        res = drv->grabScanDataHq(nodes, count);
        if (SL_IS_FAIL(res)) {
            fprintf(stderr, "grabScanDataHq failed: 0x%08x\n", res);
            // Drop the bad frame and let the SDK resync. Only give up if
            // failures pile up — a brief streak of timeouts under load
            // is normal on the S2 and shouldn't force a publisher restart
            // (which costs a cold motor spin-up).
            if (++consecutive_failures >= 10) {
                fprintf(stderr, "too many consecutive grab failures; exiting\n");
                break;
            }
            continue;
        }
        consecutive_failures = 0;
        drv->ascendScanData(nodes, count);

        const uint64_t ts = monotonic_ns();
        fprintf(stdout, "SCAN %llu\n", (unsigned long long)ts);
        for (size_t i = 0; i < count; ++i) {
            if (nodes[i].dist_mm_q2 == 0) continue;
            if (nodes[i].quality    == 0) continue;
            float angle_deg = (nodes[i].angle_z_q14 * 90.0f) / (1 << 14);
            float dist_m    = nodes[i].dist_mm_q2 / 4000.0f;
            fprintf(stdout, "%.3f %.4f %u\n",
                    angle_deg, dist_m, (unsigned)nodes[i].quality);
        }
        fputs("END\n", stdout);
        fflush(stdout);
    }

    drv->stop();
    drv->setMotorSpeed(0);
    drv->disconnect();
    delete drv;
    return 0;
}

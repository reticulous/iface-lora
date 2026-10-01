/**
 * sim-mesh's virtual radio under ESP-IDF's Linux host target: the services the
 * radio library needs, in this port's terms (esp_timer, a FreeRTOS critical
 * section, a FreeRTOS task over a non-blocking socket), and the board's clock
 * (hw-linux's hwlinux.h) on the library's node time.
 *
 * The library is sim-mesh's libsimradio-sx1262.so, linked by name and provided
 * by sim-mesh when it starts the station; only its public header (simradio.h)
 * is used here. Its own services are POSIX threads, which this port must not
 * run model callbacks on, so these are handed over before anything reaches
 * the library: from a constructor, before app_main.
 *
 * The rules of the port apply, and each one is a way it breaks:
 *
 * - no FreeRTOS task blocks in a host system call. The port only knows a task
 *   is blocked when it blocked on a FreeRTOS primitive; a task sitting in
 *   recv() is, to the scheduler, the running task. So the socket is
 *   non-blocking and the reader's only wait is select() with no timeout,
 *   which hw-linux interposes for a task: it blocks the task until the socket
 *   is readable;
 * - every task stack is at least 20 KB and has no core affinity: a task is a
 *   pthread with a real mapping, and there is one core;
 * - one critical section for everything. On this port it nests (a
 *   per-thread signal mask with a global count), which is the recursive lock
 *   the model needs, and contention between chips is nil.
 *
 * Logging goes to ESP-IDF's log under the tag `simradio`, which is where the
 * firmware's own logger reads everything else.
 */
#include "simradio.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

/* The board's (hw-linux's hwlinux.h): its identity, defined weak in lora.cpp
 * for a build without the board, and the two calls the clock makes back. */
extern "C" int hwLinuxNodeId(void);
extern "C" const char* hwLinuxBindAddr(void);
extern "C" const char* hwLinuxEtherAddr(void);
extern "C" void hwLinuxClockDue(void) __attribute__((weak));
extern "C" void hwLinuxClockMoved(void) __attribute__((weak));

namespace {

constexpr int      kMaxDatagram  = 4096;
constexpr int      kRecvBuffer   = 1 << 20;
constexpr uint32_t kTaskStack    = 32768;
constexpr int      kTaskPrio     = 6;      /* above a radio task at 5 */
constexpr int64_t  kNever        = INT64_MAX;

const char* const kTag = "simradio";

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* The host's monotonic clock, read past any time shim and never through
 * esp_timer, which reads the library. Zero at the first reading. */
int64_t nowUs()
{
    static int64_t origin = -1;
    struct timespec ts;
    syscall(SYS_clock_gettime, CLOCK_MONOTONIC, &ts);
    int64_t now = (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
    if (origin < 0) origin = now;
    return now - origin;
}

void* timerCreate(void (*cb)(void*), void* arg, const char* name)
{
    esp_timer_create_args_t a = {};
    a.callback = cb;
    a.arg = arg;
    a.name = name;
    esp_timer_handle_t h = nullptr;
    if (esp_timer_create(&a, &h) != ESP_OK) return nullptr;
    return h;
}

void timerStartOnce(void* timer, int64_t delayUs)
{
    if (!timer) return;
    auto h = (esp_timer_handle_t)timer;
    esp_timer_stop(h);                  /* restart, not "already running" */
    esp_timer_start_once(h, (uint64_t)(delayUs < 0 ? 0 : delayUs));
}

void timerStop(void* timer)
{
    if (timer) esp_timer_stop((esp_timer_handle_t)timer);
}

void lock() { portENTER_CRITICAL(&s_mux); }
void unlock() { portEXIT_CRITICAL(&s_mux); }

void logLine(int level, const char* fmt, ...)
{
    esp_log_level_t l = level <= SIMRADIO_LOG_ERROR ? ESP_LOG_ERROR
                      : level == SIMRADIO_LOG_WARN  ? ESP_LOG_WARN
                                                    : ESP_LOG_INFO;
    char line[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    ESP_LOG_LEVEL(l, kTag, "%s", line);     /* the prefix and the newline are the log's */
}

int udpOpen(const char* bindAddr, const char* dest)
{
    char host[96];
    snprintf(host, sizeof host, "%s", dest ? dest : "");
    char* colon = strrchr(host, ':');
    if (!colon) {
        logLine(SIMRADIO_LOG_ERROR, "ether: %s is not host:port", host);
        return -1;
    }
    *colon = '\0';
    int port = atoi(colon + 1);

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        logLine(SIMRADIO_LOG_ERROR, "ether: socket: %s", strerror(errno));
        return -1;
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    /* Room for a burst of the ether's messages: one the kernel drops is
     * recovered only by the library's resend. */
    int rcvbuf = kRecvBuffer;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof rcvbuf);

    /* Bound to an ephemeral port on this station's own address, so the ether
     * can tell one station's datagrams from another's by source alone. */
    struct sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = inet_addr(bindAddr);
    local.sin_port = 0;
    if (bind(fd, (struct sockaddr*)&local, sizeof local) != 0)
        logLine(SIMRADIO_LOG_WARN, "ether: bind %s: %s", bindAddr, strerror(errno));

    struct sockaddr_in peer = {};
    peer.sin_family = AF_INET;
    peer.sin_addr.s_addr = inet_addr(host);
    peer.sin_port = htons((uint16_t)port);
    if (connect(fd, (struct sockaddr*)&peer, sizeof peer) != 0) {
        logLine(SIMRADIO_LOG_ERROR, "ether: connect %s: %s", dest, strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}

struct Reader {
    int fd;
    void (*onDatagram)(const char*, size_t);
};

void readerTask(void* arg)
{
    Reader r = *(Reader*)arg;
    delete (Reader*)arg;
    static char buf[kMaxDatagram + 1];
    for (;;) {
        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(r.fd, &rd);
        if (select(r.fd + 1, &rd, nullptr, nullptr, nullptr) <= 0) continue;
        for (;;) {
            ssize_t n = recv(r.fd, buf, kMaxDatagram, MSG_DONTWAIT);
            if (n <= 0) break;
            buf[n] = '\0';
            r.onDatagram(buf, (size_t)n);
        }
    }
}

int spawnReader(int fd, void (*onDatagram)(const char*, size_t))
{
    auto* r = new Reader{fd, onDatagram};
    TaskHandle_t h = nullptr;
    if (xTaskCreatePinnedToCore(readerTask, "ether", kTaskStack, r, kTaskPrio, &h,
                                tskNO_AFFINITY) != pdPASS) {
        delete r;
        return -1;
    }
    return 0;
}

const struct simradio_services kServices = {
    nowUs,
    timerCreate,
    timerStartOnce,
    timerStop,
    lock,
    unlock,
    udpOpen,
    spawnReader,
    logLine,
};

__attribute__((constructor)) void handOver()
{
    simradio_set_services(&kServices);
}

/* esp_timer's next expiry, as a wake in node time. */
int s_timerWake = -1;

void timerDue(void*)
{
    if (hwLinuxClockDue) hwLinuxClockDue();
}

/* The next tick a task waits for, as a wake in node time. Reaching it does
 * nothing of its own: every move of T already brings the tick count up. */
int s_tickWake = -1;

void tickDue(void*) {}

void clockMoved()
{
    if (hwLinuxClockMoved) hwLinuxClockMoved();
}

/* The station's clock reads 0 at the whole second of node time the station
 * joined in. Every station's clock is then a whole number of seconds from
 * every other's, so what the board times on it — its tick above all — falls
 * on the same instants of T on every station, and those share a barrier. */
constexpr int64_t kSecondUs = 1000000;

int64_t clockZero()
{
    int64_t join = simradio_node_at_join();
    return join - join % kSecondUs;
}

int64_t toNode(int64_t us)
{
    return us == kNever ? kNever : us + clockZero();
}

}  // namespace

/* What the component's link names with `-u`, so this file is linked: nothing
 * else refers to it strongly — the board's references are weak — and the
 * linker leaves an archive member nobody asks for out, constructor and all. */
extern "C" void simradio_glue_linked(void) {}

/* ---- The board's clock (hw-linux's hwlinux.h) ----
 *
 * esp_timer, the board's tick and idle, and its bring-up reach the station's
 * clock through these. In a real-time run the clock is the host's and this
 * adds nothing; in a virtual one esp_timer counts node time from the ether's
 * welcome, its next expiry and the next tick a task waits for are wakes, the
 * tick count follows every move of T, and the idle task is what tells the
 * ether this station is idle. */

extern "C" int64_t hwLinuxClockUs(void)
{
    if (!simradio_virtual()) return nowUs();
    return simradio_joined() ? simradio_node_us() - clockZero() : 0;
}

extern "C" void hwLinuxClockWake(int64_t us)
{
    if (!simradio_virtual()) return;
    if (s_timerWake < 0) s_timerWake = simradio_wake_create(timerDue, nullptr);
    simradio_wake_at(s_timerWake, toNode(us));
}

extern "C" void hwLinuxClockTickAt(int64_t us)
{
    if (!simradio_virtual()) return;
    if (s_tickWake < 0) s_tickWake = simradio_wake_create(tickDue, nullptr);
    simradio_wake_at(s_tickWake, toNode(us));
}

extern "C" void hwLinuxClockIdle(void)
{
    simradio_idle();
}

/* A virtual run needs the ether before anything in the station waits on
 * time, since only the ether moves it: so the link opens here, at the
 * board's bring-up, rather than when the radio does. */
extern "C" int hwLinuxClockStart(void)
{
    if (!simradio_virtual()) return 0;
    simradio_on_advance(clockMoved);
    simradio_station_open(hwLinuxNodeId(), hwLinuxBindAddr(), hwLinuxEtherAddr());
    return 1;
}

/**
 * ether_task — see the header.
 *
 * The socket is non-blocking and the only wait is a select() with a one-tick
 * timeout, which IDF interposes for a FreeRTOS task: it polls, then sleeps on
 * a delay, so the scheduler sees a blocked task rather than a spinning one.
 */
#include "ether_task.h"

#include "virtual_sx126x.h"

#include "compat.h"
#include "log.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"

#include <arpa/inet.h>
#include <cJSON.h>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

/* The station's identity and its two addresses come from the board straddle.
 * Weak, so a build assembled without one still links and stays silent. */
extern "C" __attribute__((weak)) int hwLinuxNodeId(void) { return 1; }
extern "C" __attribute__((weak)) const char* hwLinuxBindAddr(void) { return "127.0.0.1"; }
extern "C" __attribute__((weak)) const char* hwLinuxEtherAddr(void) { return ""; }

namespace {

constexpr int  kMaxDatagram = 4096;
constexpr int  kTaskStack   = 32768;
constexpr int  kTaskPrio    = 6;      /* above the radio task, which runs at 5 */

int  s_fd = -1;
bool s_started = false;

void sendLine(const char* s, size_t n)
{
    if (s_fd < 0) return;
    ssize_t w = send(s_fd, s, n, MSG_DONTWAIT);
    (void)w;
}

size_t b64(const uint8_t* in, size_t n, char* out, size_t cap)
{
    size_t olen = 0;
    if (n == 0 || mbedtls_base64_encode((unsigned char*)out, cap, &olen, in, n) != 0) {
        out[0] = '\0';
        return 0;
    }
    return olen;
}

void appendState(char* p, size_t cap, size_t* at, const EtherState& s)
{
    *at += (size_t)snprintf(p + *at, cap - *at,
        "\"slot\":%d,\"freq\":%u,\"bw\":%u,\"sf\":%d,\"cr\":%d,\"sync\":%d,"
        "\"hdr\":\"%s\",\"crc\":%s,\"pre\":%d",
        s.slot, (unsigned)s.freqHz, (unsigned)s.bwHz, s.sf, s.cr, s.syncWord,
        s.hdrImplicit ? "implicit" : "explicit", s.crc ? "true" : "false", s.preamble);
}

/* ---- Inbound ---- */

int jsonInt(const cJSON* o, const char* k, int def)
{
    const cJSON* v = cJSON_GetObjectItemCaseSensitive(o, k);
    return cJSON_IsNumber(v) ? (int)v->valuedouble : def;
}

int64_t jsonI64(const cJSON* o, const char* k, int64_t def)
{
    const cJSON* v = cJSON_GetObjectItemCaseSensitive(o, k);
    return cJSON_IsNumber(v) ? (int64_t)v->valuedouble : def;
}

const char* jsonStr(const cJSON* o, const char* k, const char* def)
{
    const cJSON* v = cJSON_GetObjectItemCaseSensitive(o, k);
    return cJSON_IsString(v) && v->valuestring ? v->valuestring : def;
}

void handleMessage(const char* text, size_t len)
{
    cJSON* root = cJSON_ParseWithLength(text, len);
    if (!root) return;
    const char* type = jsonStr(root, "type", "");

    if (strcmp(type, "rx_begin") == 0) {
        VirtualSx126x* chip = virtualChip(jsonInt(root, "slot", 0));
        if (chip) {
            VirtualRxBegin f = {};
            f.id       = jsonInt(root, "id", 0);
            f.t0       = jsonI64(root, "t0", 0);
            f.tPre     = jsonI64(root, "t_pre", 0);
            f.tHdr     = jsonI64(root, "t_hdr", 0);
            f.tEnd     = jsonI64(root, "t_end", 0);
            f.levelDbm = jsonInt(root, "level", VirtualSx126x::kNoiseFloorDbm);
            chip->onRxBegin(f);
        }
    } else if (strcmp(type, "rx_end") == 0) {
        VirtualSx126x* chip = virtualChip(jsonInt(root, "slot", 0));
        if (chip) {
            uint8_t payload[256];
            size_t  plen = 0;
            const char* b = jsonStr(root, "payload", "");
            if (*b) {
                if (mbedtls_base64_decode(payload, sizeof payload, &plen,
                                          (const unsigned char*)b, strlen(b)) != 0)
                    plen = 0;
            }
            const char* verdict = jsonStr(root, "verdict", "clean");
            VirtualRxEnd f = {};
            f.id       = jsonInt(root, "id", 0);
            f.payload  = payload;
            f.len      = plen;
            f.crcOk    = strcmp(verdict, "crc") != 0 && strcmp(verdict, "hdr") != 0;
            f.headerOk = strcmp(verdict, "hdr") != 0;
            f.rssiDbm  = jsonInt(root, "rssi", -80);
            f.snrDb    = jsonInt(root, "snr", 10);
            chip->onRxEnd(f);
        }
    } else if (strcmp(type, "energy") == 0) {
        VirtualSx126x* chip = virtualChip(jsonInt(root, "slot", 0));
        if (chip) {
            VirtualRxBegin f = {};
            f.id       = jsonInt(root, "id", 0);
            f.t0       = jsonI64(root, "t0", 0);
            f.tEnd     = jsonI64(root, "t_end", 0);
            f.levelDbm = jsonInt(root, "level", VirtualSx126x::kNoiseFloorDbm);
            chip->onEnergy(f);
        }
    } else if (strcmp(type, "welcome") == 0) {
        info("ether: %s at t0 %lld", jsonStr(root, "mode", "real"),
             (long long)jsonI64(root, "t0", 0));
    }
    /* Anything else is a message this station does not understand. */
    cJSON_Delete(root);
}

void etherTaskFn(void*)
{
    char buf[kMaxDatagram + 1];
    for (;;) {
        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(s_fd, &rd);
        struct timeval tv = { 0, 1000 * portTICK_PERIOD_MS };
        if (select(s_fd + 1, &rd, nullptr, nullptr, &tv) <= 0) continue;
        for (;;) {
            ssize_t n = recv(s_fd, buf, kMaxDatagram, MSG_DONTWAIT);
            if (n <= 0) break;
            buf[n] = '\0';
            handleMessage(buf, (size_t)n);
        }
    }
}

}  // namespace

/* ---- Outbound ---- */

void etherPublishState(const EtherState& s)
{
    if (s_fd < 0) return;
    char line[512];
    size_t at = (size_t)snprintf(line, sizeof line,
        "{\"type\":\"state\",\"sid\":%d,\"t\":%lld,\"mode\":\"%s\",\"ready_at\":%lld,",
        hwLinuxNodeId(), (long long)esp_timer_get_time(), s.mode, (long long)s.readyAt);
    appendState(line, sizeof line, &at, s);
    at += (size_t)snprintf(line + at, sizeof line - at, "}");
    sendLine(line, at);
}

void etherPublishTx(const EtherTxFrame& f)
{
    if (s_fd < 0) return;
    char payload[400];
    b64(f.payload, f.len, payload, sizeof payload);

    char line[1024];
    size_t at = (size_t)snprintf(line, sizeof line,
        "{\"type\":\"tx\",\"sid\":%d,\"t\":%lld,\"id\":%d,"
        "\"t0\":%lld,\"t_pre\":%lld,\"t_hdr\":%lld,\"t_end\":%lld,\"power_dbm\":%d,",
        hwLinuxNodeId(), (long long)esp_timer_get_time(), f.id,
        (long long)f.t0, (long long)f.tPre, (long long)f.tHdr, (long long)f.tEnd,
        f.powerDbm);
    appendState(line, sizeof line, &at, f.state);
    at += (size_t)snprintf(line + at, sizeof line - at, ",\"payload\":\"%s\"}", payload);
    sendLine(line, at);
}

/* ---- Bring-up ---- */

void etherStart(void)
{
    if (s_started) return;
    s_started = true;

    const char* addr = hwLinuxEtherAddr();
    if (!addr || !*addr) {
        info("ether: no ether named — the radio transmits into nothing");
        return;
    }

    char host[96];
    snprintf(host, sizeof host, "%s", addr);
    char* colon = strrchr(host, ':');
    if (!colon) { err("ether: %s is not host:port", addr); return; }
    *colon = '\0';
    int port = atoi(colon + 1);

    s_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (s_fd < 0) { err("ether: socket: %s", strerror(errno)); return; }
    fcntl(s_fd, F_SETFL, fcntl(s_fd, F_GETFL, 0) | O_NONBLOCK);

    /* Bound to an ephemeral port on this station's own address, so the ether
     * can tell one station's datagrams from another's by source alone. */
    struct sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = inet_addr(hwLinuxBindAddr());
    local.sin_port = 0;
    if (bind(s_fd, (struct sockaddr*)&local, sizeof local) != 0)
        warn("ether: bind %s: %s", hwLinuxBindAddr(), strerror(errno));

    struct sockaddr_in peer = {};
    peer.sin_family = AF_INET;
    peer.sin_addr.s_addr = inet_addr(host);
    peer.sin_port = htons((uint16_t)port);
    if (connect(s_fd, (struct sockaddr*)&peer, sizeof peer) != 0) {
        err("ether: connect %s: %s", addr, strerror(errno));
        close(s_fd);
        s_fd = -1;
        return;
    }

    char hello[160];
    int n = snprintf(hello, sizeof hello,
        "{\"type\":\"hello\",\"sid\":%d,\"t\":%lld,\"slots\":[0]}",
        hwLinuxNodeId(), (long long)esp_timer_get_time());
    sendLine(hello, (size_t)n);

    if (!spawnTask(etherTaskFn, "ether", kTaskStack, nullptr, kTaskPrio, 0))
        err("ether: no task");
    else
        info("ether: %s, station %d", addr, hwLinuxNodeId());
}

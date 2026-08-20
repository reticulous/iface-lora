/**
 * lora_rnode — the RNode client endpoint: KISS decode/encode, command
 * execution against the ordinary config keys, the config echo, and the two
 * transports (serial claim + net TCP door). Protocol overview and the
 * client-side traps at RNODE_ITS_PORT in lora_priv.h.
 */
#include "lora_priv.h"

#if defined(CONFIG_LORA0_CS_PIN)

/* One session at a time, across every transport. */
RnodeState s_rnode;

/* ─────────────── RNode endpoint: implementation ───────────────
 * (protocol overview and the client-side traps at RNODE_ITS_PORT, top of file) */

/* The only TCP port a stock client can dial: TCPConnection.TARGET_PORT is
 * hardcoded client-side, so the setting is a switch, not a number. */
#define RNODE_TCP_PORT 7633

/* One switch per door. Serial and Bluetooth default ON — a dormant in-band-
 * triggered claim and an unadvertised GATT service cost nothing until a
 * client speaks — TCP defaults OFF: a listener on the network is a decision,
 * not a side effect. There is no master switch: an open door is what
 * "enabled" means, and a client can only arrive through an open one.
 * (`s.lora.rnode.ble` is read by rnode-ble, which owns that door.) */
static bool rnodeSerialOn(void) { return storageGetInt("s.lora.rnode.serial", 1) != 0; }
static bool rnodeTcpOn(void)    { return storageGetInt("s.lora.rnode.tcp",    0) != 0; }

static bool rnodeDoorOn(uint8_t door) {
    switch (door) {
    case RNODE_VIA_SERIAL: return rnodeSerialOn();
    case RNODE_VIA_TCP:    return rnodeTcpOn();
    case RNODE_VIA_BLE:    return storageGetInt("s.lora.rnode.ble", 1) != 0;
    }
    return false;
}

static int rnodeRadioIdx(void) {
    int i = storageGetInt("s.lora.rnode.radio", 0);
    return (i >= 0 && i < kNumRadios) ? i : 0;
}

/* KISS encode buffer — lora task only. Worst case is every payload byte
 * escaping to two, plus the delimiters and the command byte. */
static uint8_t s_rnodeTx[2 * RNS_MTU + 8];

/* Encode and send one whole command frame. Whole frame or nothing: the client
 * never flushes a command frame it has only part of, so a stream-mode short
 * write would leave it waiting for the rest forever and desynchronise
 * everything after it. */
static bool rnodeSendCmd(uint8_t cmd, const uint8_t* payload, size_t n) {
    if (s_rnode.handle < 0) return false;
    size_t o = 0;
    s_rnodeTx[o++] = KISS_FEND;
    s_rnodeTx[o++] = cmd;
    for (size_t i = 0; i < n && o + 3 <= sizeof(s_rnodeTx); i++) {
        uint8_t b = payload[i];
        if (b == KISS_FEND)      { s_rnodeTx[o++] = KISS_FESC; s_rnodeTx[o++] = KISS_TFEND; }
        else if (b == KISS_FESC) { s_rnodeTx[o++] = KISS_FESC; s_rnodeTx[o++] = KISS_TFESC; }
        else                     { s_rnodeTx[o++] = b; }
    }
    s_rnodeTx[o++] = KISS_FEND;
    if (itsSpacesAvailable(s_rnode.handle) < o) return false;
    return itsSend(s_rnode.handle, s_rnodeTx, o, 0) == o;
}

/* Release the client's transmit queue after one of its packets has finished
 * with the radio. Harmless with flow control off, mandatory with it on. The
 * payload byte is not optional: the client dispatches a command on its payload
 * bytes, so a frame with none is read and discarded. */
void rnodeSendReady(void) {
    uint8_t z = 0;
    rnodeSendCmd(RN_CMD_READY, &z, 1);
}

/* Radio or rnsd → client. All-or-nothing: the stat frames and the data frame
 * are space-checked together and skipped together, because a partial write in
 * the middle of a KISS frame corrupts the stream from there on. */
void rnodeForwardData(LoraRadio* r, const uint8_t* data, size_t len, bool withStats) {
    RnodeState& S = s_rnode;
    if (S.handle < 0 || r->idx != S.radio || len == 0 || len > RNS_MTU) return;
    size_t need = 2 * len + 3 + (withStats ? 10 : 0);
    if (itsSpacesAvailable(S.handle) < need) {
        warn("lora/%d rnode: client stream full, dropped %u B", r->idx, (unsigned)len);
        return;
    }
    if (withStats) {
        /* Both, in this order, ahead of the data frame: the client holds them
         * as sticky attributes applied to the NEXT data frame and cleared after
         * it, and its transport drops the SNR unless an RSSI came with it. */
        int rv = (int)lroundf(r->rssiLast) + RN_RSSI_OFFSET;
        uint8_t rb = (uint8_t)(rv < 0 ? 0 : rv > 255 ? 255 : rv);
        rnodeSendCmd(RN_CMD_STAT_RSSI, &rb, 1);
        int sv = (int)lroundf(r->snrLast * 4.0f);
        int8_t sb = (int8_t)(sv < -128 ? -128 : sv > 127 ? 127 : sv);
        rnodeSendCmd(RN_CMD_STAT_SNR, (const uint8_t*)&sb, 1);
    }
    rnodeSendCmd(RN_CMD_DATA, data, len);
}

/* A client command changed (or restated) the radio configuration. Arms the
 * coalesced apply and marks the echo owed — self-arming even when every write
 * was a no-op, so the client always gets its echo and never sits out the
 * validation window waiting for one that storage saw no reason to trigger. */
static void rnodeCfgTouched(void) {
    s_rnode.echoPend = true;
    cfgArm(LORA_CFG_COALESCE_MS);
}

/* Report the state that was just applied. bw / txpower / sf / state are the
 * echoes the client compares unconditionally — an absent one reads as a
 * mismatch; frequency is optional but must be within 100 Hz if present, which
 * is why it comes from the applied value rather than from what was asked; cr is
 * never validated. A mismatch costs a full reconnect every 5 s, forever, so
 * this runs from the apply pass and nowhere earlier. */
void rnodeEchoFlush(void) {
    RnodeState& S = s_rnode;
    if (S.handle < 0 || !S.echoPend) return;
    S.echoPend = false;
    LoraRadio* r = &s_radios[S.radio];
    char kb[48];

    auto be32 = [](uint8_t* o, uint32_t v) {
        o[0] = (uint8_t)(v >> 24); o[1] = (uint8_t)(v >> 16);
        o[2] = (uint8_t)(v >> 8);  o[3] = (uint8_t)v;
    };
    uint8_t b4[4];
    be32(b4, (uint32_t)storageGetInt(sk(kb, sizeof kb, S.radio, "frequency"), 0));
    rnodeSendCmd(RN_CMD_FREQUENCY, b4, 4);
    be32(b4, (uint32_t)r->cfgBwHz);
    rnodeSendCmd(RN_CMD_BANDWIDTH, b4, 4);
    uint8_t v = (uint8_t)r->cfgTxp;
    rnodeSendCmd(RN_CMD_TXPOWER, &v, 1);
    v = (uint8_t)r->cfgSf;
    rnodeSendCmd(RN_CMD_SF, &v, 1);
    v = (uint8_t)r->cfgCr;
    rnodeSendCmd(RN_CMD_CR, &v, 1);
    v = r->running ? RN_RADIO_ON : RN_RADIO_OFF;
    rnodeSendCmd(RN_CMD_RADIO_STATE, &v, 1);

    if (S.wantOn && !r->running) {
        /* The one error code the client turns into a clean teardown and retry.
         * 0x03/0x04 are unhandled ("Unknown hardware failure") and 0x21/0x22
         * crash its handler outright — see the note at RNODE_ITS_PORT. */
        uint8_t e = RN_ERROR_INITRADIO;
        rnodeSendCmd(RN_CMD_ERROR, &e, 1);
    }
}

void rnodeDropSession(void) {
    if (s_rnode.handle < 0) return;
    itsDisconnect(s_rnode.handle);      /* releases a claimed serial port too */
    s_rnode.handle   = -1;
    s_rnode.txLen    = 0;
    s_rnode.inLen    = s_rnode.inPos = 0;
    s_rnode.offPend  = false;
    s_rnode.echoPend = false;
    s_rnode.wantOn   = false;
}

/* ── command execution ── */

static void rnodeFrame(void) {
    RnodeState& S = s_rnode;
    char kb[48];

    switch (S.cmd) {

    /* Handshake. Answered every time it is asked, not once: over TCP the client
     * re-sends the whole four-command detect burst after 3.5 s of transmit idle
     * for the life of the connection, so these replies must be stateless and
     * repeatable. A CMD_DETECT frame carrying anything other than DETECT_RESP
     * actively clears the client's detected flag. */
    case RN_CMD_DETECT: {
        if (S.len < 1 || S.buf[0] != RN_DETECT_REQ) break;
        uint8_t resp = RN_DETECT_RESP;
        rnodeSendCmd(RN_CMD_DETECT, &resp, 1);
        break;
    }
    case RN_CMD_FW_VERSION: {
        uint8_t ver[2] = { RN_FW_MAJ, RN_FW_MIN };
        rnodeSendCmd(RN_CMD_FW_VERSION, ver, 2);
        break;
    }
    case RN_CMD_PLATFORM: {
        uint8_t p = RN_PLATFORM_AVR;
        rnodeSendCmd(RN_CMD_PLATFORM, &p, 1);
        break;
    }
    case RN_CMD_MCU: {
        uint8_t m = RN_MCU_1284P;
        rnodeSendCmd(RN_CMD_MCU, &m, 1);
        break;
    }

    /* Radio configuration. Executed by writing the ordinary config keys, so it
     * flows through the same path an operator's edit takes — and persists. */
    case RN_CMD_FREQUENCY: {
        if (S.len < 4) break;
        uint32_t hz = ((uint32_t)S.buf[0] << 24) | ((uint32_t)S.buf[1] << 16) |
                      ((uint32_t)S.buf[2] << 8)  |  (uint32_t)S.buf[3];
        if (hz < (uint32_t)LORA_FREQ_MIN_HZ || hz > (uint32_t)LORA_FREQ_MAX_HZ) {
            warn("lora/%d rnode: frequency %u Hz out of range, ignored", S.radio, (unsigned)hz);
            break;
        }
        storageBegin();
        storageSet(sk(kb, sizeof kb, S.radio, "frequency"), (int)hz);
        storageEnd();
        rnodeCfgTouched();
        break;
    }
    case RN_CMD_BANDWIDTH: {
        if (S.len < 4) break;
        uint32_t hz = ((uint32_t)S.buf[0] << 24) | ((uint32_t)S.buf[1] << 16) |
                      ((uint32_t)S.buf[2] << 8)  |  (uint32_t)S.buf[3];
        if (hz < (uint32_t)LORA_BW_MIN_HZ || hz > (uint32_t)LORA_BW_MAX_HZ) {
            warn("lora/%d rnode: bandwidth %u Hz out of range, ignored", S.radio, (unsigned)hz);
            break;
        }
        storageBegin();
        storageSet(sk(kb, sizeof kb, S.radio, "bandwidth"), (int)hz);
        storageEnd();
        rnodeCfgTouched();
        break;
    }
    case RN_CMD_TXPOWER: {
        if (S.len < 1) break;
        int txp = (int8_t)S.buf[0];
        if (txp > RNODE_TXP_MAX) {
            /* Clamped and echoed honestly. Such a client reads the echo as a
             * mismatch and re-dials every 5 s — a churn loop rather than a
             * single abort — which is the price of not lying to it. */
            warn("lora/%d rnode: tx power %d dBm above the %d dBm ceiling, clamped",
                 S.radio, txp, RNODE_TXP_MAX);
            txp = RNODE_TXP_MAX;
        }
        storageBegin();
        storageSet(sk(kb, sizeof kb, S.radio, "tx_power"), txp);
        storageEnd();
        rnodeCfgTouched();
        break;
    }
    case RN_CMD_SF: {
        if (S.len < 1) break;
        int sf = S.buf[0];
        if (sf < 5 || sf > 12) {
            warn("lora/%d rnode: spreading factor %d out of range, ignored", S.radio, sf);
            break;
        }
        storageBegin();
        storageSet(sk(kb, sizeof kb, S.radio, "spreading_factor"), sf);
        storageEnd();
        rnodeCfgTouched();
        break;
    }
    case RN_CMD_CR: {
        if (S.len < 1) break;
        int cr = S.buf[0];
        if (cr < 5 || cr > 8) {
            warn("lora/%d rnode: coding rate 4/%d out of range, ignored", S.radio, cr);
            break;
        }
        storageBegin();
        storageSet(sk(kb, sizeof kb, S.radio, "coding_rate"), cr);
        storageEnd();
        rnodeCfgTouched();
        break;
    }

    /* The configuration burst always ends with this, so pulling the apply
     * deadline in here puts the apply and its echo inside the client's
     * validation sleep (0.25 s on serial, 1.5 s on TCP). The burst still
     * coalesces: the earlier writes armed 300 ms and nothing has applied yet. */
    case RN_CMD_RADIO_STATE: {
        if (S.len < 1) break;
        if (S.buf[0] == RN_RADIO_ON) {
            S.offPend = false;
            S.wantOn  = true;
            storageBegin();
            storageSet(sk(kb, sizeof kb, S.radio, "enable"), 1);
            storageEnd();
            rnodeCfgTouched();
        } else if (S.buf[0] == RN_RADIO_OFF) {
            /* Deferred, not obeyed now. A clean client shutdown is OFF then
             * LEAVE then close, and taking the radio down for rnsd on every
             * such exit is not what "the host has left" means. LEAVE and
             * disconnect cancel it; a client that turns the radio off and stays
             * connected is honoured at the deadline. */
            S.offPend = true;
            S.wantOn  = false;
            rnodeCfgTouched();
        } else {
            S.echoPend = true;      /* ASK — report, change nothing */
        }
        cfgArm(0);
        break;
    }

    /* Parsed and echoed as zero, never enforced: airtime governance here is
     * LBT/APPC plus rnsd's announce cap. The client parses these echoes and
     * never validates them. */
    case RN_CMD_ST_ALOCK:
    case RN_CMD_LT_ALOCK: {
        uint8_t z[2] = { 0, 0 };
        rnodeSendCmd(S.cmd, z, 2);
        break;
    }

    case RN_CMD_LEAVE:
        S.offPend = false;
        S.txLen   = 0;
        rnodeDropSession();
        return;

    case RN_CMD_DATA:
        if (S.len > 0 && S.len <= RNS_MTU) {
            memcpy(S.txPkt, S.buf, S.len);
            S.txLen = S.len;
        }
        break;

    default:
        /* Everything else is ignored in silence. A client with a beacon
         * configured also injects unsolicited callsign data frames; those are
         * plain CMD_DATA and simply air. */
        break;
    }
}

/* ── KISS decoder ── */

static void rnodeByte(uint8_t b) {
    RnodeState& S = s_rnode;
    if (b == KISS_FEND) {
        /* A close is also an open — back-to-back frames share the delimiter —
         * and this is where the decoder resyncs. A freshly accepted socket can
         * carry a partial frame: the client's TCP layer buffers writes while
         * disconnected and flushes them on the next successful one. */
        if (S.inFrame && S.haveCmd && !S.overflow) rnodeFrame();
        S.inFrame = true;
        S.haveCmd = S.escape = S.overflow = false;
        S.len = 0;
        return;
    }
    if (!S.inFrame) return;                     /* pre-sync bytes: wait for a FEND */
    if (!S.haveCmd) { S.cmd = b; S.haveCmd = true; return; }
    if (S.escape) {
        S.escape = false;
        if (b == KISS_TFEND) b = KISS_FEND;
        else if (b == KISS_TFESC) b = KISS_FESC;
    } else if (b == KISS_FESC) {
        S.escape = true;
        return;
    }
    if (S.len < sizeof(S.buf)) S.buf[S.len++] = b;
    else                       S.overflow = true;   /* swallow the rest to the next FEND */
}

/* Client → decoder. The ITS ring is the inbound queue: while a decoded packet
 * is parked waiting for the channel this stops reading, so backpressure reaches
 * the client instead of a second packet overwriting the first. */
void rnodePump(void) {
    RnodeState& S = s_rnode;
    if (S.handle < 0) return;
    for (;;) {
        while (S.inPos < S.inLen) {
            if (S.txLen) return;                /* parked — the rest stays buffered */
            rnodeByte(S.inBuf[S.inPos++]);
            if (S.handle < 0) return;           /* CMD_LEAVE dropped the session */
        }
        if (S.txLen) return;
        S.inPos = S.inLen = 0;
        size_t n = itsRecv(S.handle, S.inBuf, sizeof(S.inBuf), 0);
        if (n == 0) return;
        S.inLen = n;
    }
}

/* ── transports ── */

/* Open or close the endpoint's two doors to match the settings. Called from the
 * coalesced apply pass, which is also what puts the net registration on this
 * task — where net requires it to originate. */
void rnodeApplyTransports(void) {
    int radio = rnodeRadioIdx();

    /* A client bound to a radio the settings no longer point at — or whose
     * own door has just been switched off — is holding a session that no
     * longer means anything. A session on another door rides through. */
    if (s_rnode.handle >= 0 && (s_rnode.radio != radio || !rnodeDoorOn(s_rnode.door)))
        rnodeDropSession();

#if CONFIG_SPANGAP_NET
    /* TCP, in the two steps net's endpoint model asks for: register once, then
     * drive the listener by writing the port — net's own s.net.* subscriber
     * opens and closes the socket from there. Port 7633 is the only port a
     * stock client can dial. */
    static bool registered = false;
    if (!registered) {
        net_port_msg_t reg = {};
        reg.itsPort     = RNODE_ITS_PORT;
        reg.tcpNoDelay  = 1;
        reg.keepAlive   = 1;
        reg.backlog     = 1;
        reg.defaultPort = 0;          /* never auto-open; gated by the key below */
        safeStrncpy(reg.nvsKey, "rnode_port", sizeof(reg.nvsKey));
        if (itsSendAux("net", NET_PORT_REG_PORT, &reg, sizeof(reg), pdMS_TO_TICKS(500)))
            registered = true;
        else
            warn("lora rnode: net endpoint registration failed");
    }
    if (registered) {
        int want = rnodeTcpOn() ? RNODE_TCP_PORT : 0;
        if (storageGetInt("s.net.rnode_port", -1) != want) storageSet("s.net.rnode_port", want);
    }
#endif

    /* Serial: the door rides the highest serial port that exists right now —
     * the console port on a one-port build (shared: the console keeps working
     * until a client's first KISS FEND takes it over), the spare CDC port
     * while `usb cdc` presents two. The claim is in-band triggered on that
     * FEND, so holding it costs nothing: no keystroke produces 0xC0, and
     * esptool auto-reset stays armed until a client actually attaches — which
     * is why this door can default on. The port count is republished on every
     * transport switch and re-arms this apply pass, which moves the claim. */
    static int have = -1;
    int ports = storageGetInt("sys.usb.serial_ports", 1);
    int want  = rnodeSerialOn() ? ports - 1 : -1;
    if (have >= 0 && have != want) { serialPortRelease(have); have = -1; }
    if (want >= 0 && have < 0) {
        static const uint8_t kissFend = 0xC0;
        if (serialPortClaim(want, TAG, RNODE_ITS_PORT, &kissFend, 1)) have = want;
    }
}

/* ── ITS server callbacks (lora task, via itsPoll) ── */

/* The three doors are told apart by the connect payload's length and nothing
 * else — the endpoint has no other channel to learn which transport handed it a
 * client. The sizes must therefore stay pairwise distinct; rnode_door_connect_t
 * is padded to 12 for exactly that reason, because net_connect_t is 8 bytes in
 * an IPv6-off build and the natural Bluetooth struct is 8 too. */
static_assert(sizeof(serial_handler_connect_t) != sizeof(rnode_door_connect_t),
              "serial and ble door payloads must differ in size");
#if CONFIG_SPANGAP_NET
static_assert(sizeof(net_connect_t) != sizeof(rnode_door_connect_t),
              "tcp and ble door payloads must differ in size");
static_assert(sizeof(net_connect_t) != sizeof(serial_handler_connect_t),
              "tcp and serial door payloads must differ in size");
#endif

int onRnodeConnect(int handle, const void* data, size_t len) {
    /* One session at a time, across every transport. Enforced here as well as
     * by the port's single handle, because the refusal is what keeps a serial
     * takeover from disturbing the console while a TCP client is attached. */
    if (s_rnode.handle >= 0 || s_stop) return -1;

    const char* via = "tcp";
    uint8_t door = RNODE_VIA_TCP;
    if (len == sizeof(serial_handler_connect_t)) {
        via  = "serial";
        door = RNODE_VIA_SERIAL;
    } else if (len == sizeof(rnode_door_connect_t)) {
        auto* d = (const rnode_door_connect_t*)data;
        if (d->magic != RNODE_DOOR_MAGIC) {
            warn("lora rnode: 12-byte connect payload without the door magic, refused");
            return -1;
        }
        via  = "ble";
        door = RNODE_VIA_BLE;
    }
    /* A client can only arrive through an open door (a closed one has no
     * listener, no claim, no advertisement) — this covers the race with a
     * door closing while the connect was in flight. */
    if (!rnodeDoorOn(door)) return -1;

    /* Fresh decoder for a fresh stream. Field by field rather than assigning a
     * RnodeState{} — the temporary would be over a kilobyte of task stack. */
    RnodeState& S = s_rnode;
    S.inFrame = S.haveCmd = S.escape = S.overflow = false;
    S.cmd = 0; S.len = 0;
    S.inLen = S.inPos = 0;
    S.txLen = 0;
    S.echoPend = S.offPend = S.wantOn = S.txAlternate = false;
    S.handle = handle;
    S.radio  = rnodeRadioIdx();
    S.door   = door;
    info("lora/%d rnode: client attached over %s", S.radio, via);
    return 0;
}

void onRnodeRecv(int /*handle*/, size_t /*bytesAvail*/) { rnodePump(); }

void onRnodeDisconnect(int /*ref*/) {
    if (s_rnode.handle < 0) return;
    info("lora/%d rnode: client detached", s_rnode.radio);
    s_rnode.handle   = -1;
    s_rnode.txLen    = 0;
    s_rnode.inLen    = s_rnode.inPos = 0;
    s_rnode.offPend  = false;           /* a clean detach leaves the radio up */
    s_rnode.echoPend = false;
    s_rnode.wantOn   = false;
}

/* Honour a radio-off the client asked for and stayed connected through. Runs at
 * the apply deadline, before the apply itself, so the write lands in the same
 * pass it gates. */
void rnodeSettleOff(void) {
    if (s_rnode.handle < 0 || !s_rnode.offPend) return;
    s_rnode.offPend = false;
    char kb[48];
    storageBegin();
    storageSet(sk(kb, sizeof kb, s_rnode.radio, "enable"), 0);
    storageEnd();
}

#endif  /* CONFIG_LORA0_CS_PIN */

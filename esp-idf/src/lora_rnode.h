#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

/* ─────────────── RNode endpoint ───────────────
 * A stock Reticulum RNodeInterface client attaches to this device as if it were
 * RNode hardware, over USB serial and/or RNode-over-TCP, and becomes the third
 * endpoint of the radio segment: a packet entering from any one of the radio,
 * rnsd and the client is presented to the other two. Radio commands from the
 * client are executed by writing the ordinary s.lora.<n>.* keys, so they take
 * the same path an operator's edit does — and persist, which is the point and
 * also the hazard the README warns about.
 *
 * Wire format is raw KISS on both transports. The client dials TCP port 7633
 * and nothing else: its config URI's host:port suffix is handed whole to
 * getaddrinfo as a hostname, so a port there does not override the constant.
 *
 * We report an AVR platform on purpose. Platform ESP32 arms the client's
 * "CMD_RESET seen while online → IOError" teardown and, with NRF52, unlocks its
 * framebuffer methods — which makes its own detach() emit a framebuffer-disable
 * frame. AVR sidesteps both. The firmware version has to clear the client's
 * 1.52 floor or it calls RNS.panic(), which is os._exit(255).
 *
 * Frames we must never send: CMD_STAT_RX / CMD_STAT_TX (the client's handler
 * calls ord() on an int and takes the interface offline), CMD_ERROR 0x03 / 0x04
 * (unhandled → "Unknown hardware failure"), and any spontaneous CMD_RESET. */

/* RNODE_ITS_PORT and the Bluetooth door's connect payload are the public half
 * of this endpoint — a transport needs them, the KISS opcodes below it does
 * not. They live in include/rnode_door.h, pulled in by lora_priv.h. */

/* KISS framing. */
#define KISS_FEND   0xC0
#define KISS_FESC   0xDB
#define KISS_TFEND  0xDC
#define KISS_TFESC  0xDD

/* Commands — RNS/Interfaces/RNodeInterface.py, class KISS. */
#define RN_CMD_DATA         0x00
#define RN_CMD_FREQUENCY    0x01
#define RN_CMD_BANDWIDTH    0x02
#define RN_CMD_TXPOWER      0x03
#define RN_CMD_SF           0x04
#define RN_CMD_CR           0x05
#define RN_CMD_RADIO_STATE  0x06
#define RN_CMD_DETECT       0x08
#define RN_CMD_LEAVE        0x0A
#define RN_CMD_ST_ALOCK     0x0B
#define RN_CMD_LT_ALOCK     0x0C
#define RN_CMD_READY        0x0F
#define RN_CMD_STAT_RSSI    0x23
#define RN_CMD_STAT_SNR     0x24
#define RN_CMD_PLATFORM     0x48
#define RN_CMD_MCU          0x49
#define RN_CMD_FW_VERSION   0x50
#define RN_CMD_ERROR        0x90

#define RN_DETECT_REQ       0x73
#define RN_DETECT_RESP      0x46
#define RN_RADIO_OFF        0x00
#define RN_RADIO_ON         0x01
#define RN_RADIO_ASK        0xFF
#define RN_ERROR_INITRADIO  0x01
#define RN_PLATFORM_AVR     0x90
#define RN_MCU_1284P        0x91   /* the AVR RNode's MCU; the client only records it */
#define RN_FW_MAJ           1
#define RN_FW_MIN           78
#define RN_RSSI_OFFSET      157    /* the client subtracts this from CMD_STAT_RSSI */
#define RNODE_TXP_MAX       22     /* chip ceiling; a higher request is clamped */

/* Signal stamped onto a packet injected into rnsd from the client. It never
 * crossed the air, so any real-looking reading would be a fiction: -10 dBm at
 * 10.0 dB SNR is top-of-scale "perfect local", impossible over the air, and so
 * unmistakable as an injection in every signal view. */
#define RNODE_INJ_RSSI    (-10)
#define RNODE_INJ_SNR10   100

/* One session at a time, across every transport. */
struct RnodeState {
    int      handle = -1;       /* ITS handle of the attached client; -1 = none */
    int      radio;             /* radio index this session is bound to */

    /* KISS decoder. */
    bool     inFrame, haveCmd, escape, overflow;
    uint8_t  cmd;
    uint8_t  buf[RNS_MTU + 8];
    size_t   len;

    /* Inbound carry: a frame can complete mid-chunk and park a packet, and the
     * bytes after it in the same read must not be lost. */
    uint8_t  inBuf[128];
    size_t   inLen, inPos;

    /* One decoded packet, waiting for the channel. While it is parked the pump
     * stops reading, so backpressure reaches the client rather than a second
     * packet overwriting the first. */
    uint8_t  txPkt[RNS_MTU];
    size_t   txLen;

    bool     echoPend;          /* the apply pass owes the client a config echo */
    bool     offPend;           /* RADIO_STATE OFF, deferred to the apply deadline */
    bool     wantOn;            /* the client asked for ON; a failed start is an error */
    bool     txAlternate;       /* fair share of the channel between rnsd and the client */
};

/* One session at a time, across every transport (lora_rnode.cpp). */
extern RnodeState s_rnode;

/* ─────────────── lora_rnode: the RNode client endpoint ─────────────── */
void rnodeForwardData(LoraRadio* r, const uint8_t* data, size_t len, bool withStats);
void rnodeSendReady(void);
void rnodePump(void);
void rnodeEchoFlush(void);
void rnodeApplyTransports(void);
void rnodeSettleOff(void);
void rnodeDropSession(void);
int  onRnodeConnect(int handle, const void* data, size_t len);
void onRnodeRecv(int handle, size_t bytesAvail);
void onRnodeDisconnect(int ref);

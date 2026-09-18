/**
 * ether_task — the station's one link to the virtual ether.
 *
 * One task and one UDP socket. Outbound: everything a chip model does that
 * anyone else could observe — the mode and carrier it sits on, and each frame
 * it transmits. Inbound: the frames that reach its antenna, applied to the
 * addressed slot's model.
 *
 * The messages are JSON, one per datagram, payloads base64, times in
 * microseconds on the sender's own clock and meaningful only against each
 * other. A station ignores a message it does not understand.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

/** What a receiver needs to know about this radio to decide whether a frame
 *  reaches it. */
struct EtherState {
    int         slot;
    const char* mode;        /* SLEEP | STDBY_RC | STDBY_XOSC | FS | TX | RX */
    int64_t     readyAt;     /* when the mode transition completes, µs */
    uint32_t    freqHz;
    uint32_t    bwHz;
    int         sf;
    int         cr;          /* the 4/N denominator, 5..8 */
    int         syncWord;
    bool        hdrImplicit;
    bool        crc;
    int         preamble;
};

/** A frame leaving this radio. */
struct EtherTxFrame {
    EtherState     state;
    int            id;
    int64_t        t0, tPre, tHdr, tEnd;
    int            powerDbm;
    const uint8_t* payload;
    size_t         len;
};

/** Bring the link up: the socket, the hello, and the task that drains it.
 *  Idempotent; a station with no ether named in its environment does nothing. */
void etherStart(void);

void etherPublishState(const EtherState& s);
void etherPublishTx(const EtherTxFrame& f);

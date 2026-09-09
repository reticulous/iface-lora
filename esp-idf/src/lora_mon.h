#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

/* ─────────────── LoRaMon: per-on-air-frame record ring ───────────────
 * One record per LoRa frame (so two per split RNS packet). Always recording
 * while a radio is up; the ring holds ~a busy hour. A LoRaMon viewer (web/LCD)
 * pulls the whole ring once over ITS (port LORAMON_PORT) and then follows live
 * frames via the ephemeral `lora.<n>.mon.*` storage keys. `dur_ms` is the
 * frame's computed time-on-air; `t_ms` its start on the monotonic ms clock. */
#define LORA_MON_CAP  4096          /* max published packet nodes per radio (FIFO backstop) */

/* Rolling one-hour airtime, 12 × 5-minute buckets per radio. The apps derive
 * every shorter window from the frame records themselves; only the hour — which
 * needs more history than a viewer may have been open for — is published. */
#define AIR_BUCKETS     12
#define AIR_BUCKET_MS   (5u * 60u * 1000u)

struct AirBucket { uint32_t absIdx; uint32_t rxMs, txMs; };

/* Telemetry state, one per radio, embedded in LoraRadio as `mon`. The
 * airtime rollups and drop counters are written by the radio task's recorder
 * (loraMonPush); the FIFO belongs to the interface task. Nothing outside
 * lora_mon touches any of it. */
struct LoraMonState {
    AirBucket  air[AIR_BUCKETS];     /* rolling one-hour airtime */
    Rolling1h  txAir[LORA_CH_MAX];   /* transmit seconds per channel, rolling hour */
    uint32_t*  pktMs;                /* FIFO of published packet start-ms (gp_alloc'd once) */
    uint16_t   pktCap, pktHead, pktCount;
    TickType_t rssiNext;             /* tick the next channel-RSSI sample is due */
    uint32_t   rssiDropped;          /* samples lost to a full interface queue */
    uint32_t   monDropped;           /* frame records lost to a full interface queue */
    uint32_t   dwellSince;           /* when the radio arrived on its current
                                      * channel; 0 = no stay in progress, take
                                      * the next pass as its start */
    bool       dwellWatched;         /* the watch state dwellSince belongs to.
                                      * A stay that began before the window
                                      * opened is not one this viewer may be
                                      * shown, and the edge has to be noticed on
                                      * the RADIO task — dwellSince is its
                                      * state, and the interface task clearing
                                      * it from under a span being closed is a
                                      * cross-task write for no reason. */
    /* The dwell node currently being extended, so a stay on one channel is one
     * growing record rather than one node per beat. Interface task only, like
     * the FIFO beside it. */
    uint32_t   dwellKeyMs;           /* 0 = nothing to extend */
    uint32_t   dwellEndMs;
    uint16_t   dwellDur;
    uint8_t    dwellCh;
    uint8_t    dwellTag[3];          /* the meeting whose slot this stay is, if
                                      * any — a change of peer ends the run */
};

/* ─────────────── lora_mon: telemetry ─────────────── */
/* What a frame IS, for the graph to say on demand. A code rather than a string:
 * every record is a storage node and there may be thousands of them, so the
 * name lives once in each viewer's own table (browser and LCD) and the record
 * carries a byte. Values are wire — a viewer decodes them — so they are
 * appended to, never renumbered.
 *
 *   0 unknown   1 HAIL  2 ANNOUNCE  3 GOT  4 READY
 *   5 END   6 BYE       7 RESEND     8 data      9 announce
 *  10 link req 11 proof    12 split     13 RNode */
enum : uint8_t {
    LMD_NONE = 0,
    LMD_HAIL, LMD_ANNOUNCE, LMD_GOT, LMD_READY,
    LMD_END, LMD_BYE, LMD_RESEND,
    LMD_RNS_DATA, LMD_RNS_ANNOUNCE, LMD_RNS_LINKREQ, LMD_RNS_PROOF,
    LMD_RNS_SPLIT, LMD_RNODE,
};

/* Classify one on-air frame. `type` is the LORA_PKT_* class the caller already
 * knows; `f` is the frame as it flew, framing byte included.
 *
 * Both this and loraMonTagOf read the Reticulum header, so both want the half of
 * a split packet that HAS one — the head. Nothing in a frame says which half it
 * is, so a recorder should call loraMonClassify below rather than either of
 * these: it is the thing that tracks halves. */
uint8_t loraMonDescribe(const uint8_t* f, size_t len, uint8_t type);

/* The three bytes of the address a frame was AIMED AT, or zeros where there are
 * none to take. The same prefix the protocol classifies on, so a record and a
 * log line agree about who a frame was for.
 *
 * A split packet's address lives in its head, which is the only half carrying a
 * header — pass that half, or call loraMonClassify, which knows which half it
 * has. */
void loraMonTagOf(const uint8_t* f, size_t len, uint8_t type, uint8_t out[3]);

/* What one recorded frame is and who it concerns, in one call — which is how a
 * caller should ask, since answering either needs to know which half of a split
 * the frame holds and nothing in the frame says. Tracks that itself, per
 * direction, so it is right for a train (buffered by its meeting, reassembled
 * only at close) as well as for plain traffic.
 *
 * `desc` is what this FRAME is — a tail is a `split`. `whole` is what the PACKET
 * is, which a tail inherits from its head; cast keys off that one. RADIO TASK. */
void loraMonClassify(LoraRadio* r, uint8_t dir, const uint8_t* f, size_t len,
                     uint8_t type, uint32_t now,
                     uint8_t* desc, uint8_t* whole, uint8_t tag[3]);

/* Who SENT a frame, where the frame says so, and false where it does not. Two
 * SUPE frames do. HAIL carries the sender's identity prefix precisely
 * because the node it names in its address field is the node being hailed, not
 * the one hailing: a received hail concerns the sender, since the address on it
 * is us. ANNOUNCE has no address field at all and its payload IS the sender's
 * identities, so without this an announcement — the frame that introduces a
 * node — is the one frame on the graph attributed to nobody. */
bool loraMonSenderOf(const uint8_t* f, size_t len, uint8_t type, uint8_t out[3]);

/* Whose traffic an address is: ours, or somebody else's. Never broadcast —
 * being aimed at everyone is a property of the frame, not of an address, and
 * the caller knows it from the description. An address that resolves to nobody
 * is OTHER: unknown is not the same as ours.
 *
 * Decided here rather than in the viewer: it turns on which addresses mean US,
 * which lives in the peer table and reaches no browser. */
enum : uint8_t {
    LMC_BCAST = 0,   /* aimed at everyone — an announce */
    LMC_US,          /* unicast, and the address is one of ours */
    LMC_OTHER,       /* unicast for somebody else, overheard */
    /* Ours too, but by a LINK identifier rather than by a destination or an
     * identity — split out because it is the one case where being unable to
     * name the far end is a fact and not a gap. A link request carries no
     * sender and its identify step is encrypted inside the session, so a link
     * DIALLED TO US has an anonymous far end for its whole life; a link we
     * dialled has the identifier filed on the peer's row, so it resolves and
     * this never shows. A viewer that finds no name behind one of these can say
     * *inbound link* instead of leaving six hex characters unexplained. */
    LMC_US_LINK,
};
uint8_t loraMonCastOf(LoraRadio* r, const uint8_t tag[3]);

void loraMonPush(LoraRadio* r, uint8_t dir, uint32_t t_ms, uint16_t dur_ms,
                 uint16_t bytes, int16_t rssi, int16_t snr10, int8_t txp,
                 uint8_t type, uint16_t wait_ms, uint16_t own_ms,
                 uint8_t desc, const uint8_t tag[3], uint8_t cast);
/* One listening span closed off: where the radio was and for how long. Called
 * on every retune and on the maintenance beat — see the note at the record. */
void loraMonDwell(LoraRadio* r, uint32_t now);
void publishStats(LoraRadio* r);
/* The `L<n>` status-bar pill for the whole medium — every slot's neighbours
 * added up, published while any slot is enabled. */
void publishPill(void);
void publishChannels(LoraRadio* r);
void publishState(LoraRadio* r, const char* state);
void rssiSamplePoll(LoraRadio* r);
bool loraMonOpen(void);          /* a LoRaMon viewer (web or LCD) is open */
void loraMonStart(void);
bool loraMonParked(void);
#if CONFIG_STRADDLE_LORAMON
/* The expiry FIFO, allocated once per radio. Only the recorder fills it, so
 * with no viewer straddle staged there is nothing to allocate and the call is
 * a no-op rather than a #if at the config-apply site. */
void loraMonInit(LoraRadio* r);
#else
inline void loraMonInit(LoraRadio*) {}
#endif

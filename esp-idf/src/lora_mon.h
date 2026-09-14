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
 * A code's NAME is the one the protocol that defined it uses — RNS's own
 * all-caps constants verbatim, our own protocol's frames under a SUPE_ prefix —
 * so the graph, a Reticulum log and the source all say the same word. `split`
 * and `RNode` are neither protocol's: they are facts about this interface's own
 * framing, and read lower case to say so.
 *
 *   0 unknown        1 SUPE_HAIL   2 SUPE_ANNOUNCE  3 SUPE_GOT  4 SUPE_READY
 *   5 SUPE_END       6 SUPE_BYE    7 SUPE_RESEND    8 DATA      9 ANNOUNCE
 *  10 LINKREQUEST   11 PROOF      12 split         13 RNode
 *
 * 8 through 11 are Reticulum's four packet-type bits and nothing more, which
 * is all this record used to say about a packet that was not ours. The header
 * carries a destination type and a context byte beside them, both in the clear
 * and both naming what the packet is *for*, so a path request read as `data`
 * and a path response — an announce answering one — as a broadcast announce.
 * Everything from 14 is that header read out: the context byte first, then the
 * destination type, with 8 through 11 left holding the cases none of the rest
 * claims. A name here is a fact off the wire, never a guess: what a packet
 * carries is encrypted and stays unsaid. */
enum : uint8_t {
    LMD_NONE = 0,
    LMD_HAIL, LMD_ANNOUNCE, LMD_GOT, LMD_READY,
    LMD_END, LMD_BYE, LMD_RESEND,
    LMD_RNS_DATA, LMD_RNS_ANNOUNCE, LMD_RNS_LINKREQ, LMD_RNS_PROOF,
    LMD_RNS_SPLIT, LMD_RNODE,
    /* 14 */ LMD_RNS_PATHREQ,      /* PLAIN, to rnstransport.path.request */
    /* 15 */ LMD_RNS_PATHRESP,     /* an announce answering one (ctx PATH_RESPONSE) */
    /* 16 */ LMD_RNS_TUNNEL,       /* PLAIN, to rnstransport.tunnel.synthesize */
    /* 17 */ LMD_RNS_PLAIN,        /* PLAIN to anything else — unencrypted by definition */
    /* 18 */ LMD_RNS_LRPROOF,      /* the proof that establishes a link */
    /* 19 */ LMD_RNS_LINKPROOF,    /* a packet proof inside an established link */
    /* 20 */ LMD_RNS_RESPROOF,
    /* 21 */ LMD_RNS_GROUP,
    /* 22 */ LMD_RNS_LINKDATA,     /* LINK dest, no context — the payload stream */
    /* 23 */ LMD_RNS_RESPART,
    /* 24 */ LMD_RNS_RESADV,
    /* 25 */ LMD_RNS_RESREQ,
    /* 26 */ LMD_RNS_RESHMU,
    /* 27 */ LMD_RNS_RESCANCEL,    /* initiator cancelled */
    /* 28 */ LMD_RNS_RESCANCEL_RX, /* receiver cancelled */
    /* 29 */ LMD_RNS_CACHEREQ,
    /* 30 */ LMD_RNS_REQUEST,
    /* 31 */ LMD_RNS_RESPONSE,
    /* 32 */ LMD_RNS_COMMAND,
    /* 33 */ LMD_RNS_CMDSTATUS,
    /* 34 */ LMD_RNS_CHANNEL,
    /* 35 */ LMD_RNS_KEEPALIVE,
    /* 36 */ LMD_RNS_LINKIDENT,
    /* 37 */ LMD_RNS_LINKCLOSE,
    /* 38 */ LMD_RNS_LINKRTT,
};

/* The detail fields, filled only while a viewer asks for them (§ the
 * `detailed` toggle). Each is a short string whose MEANING is given by the
 * record's `desc`: the viewer holds the table that says how to read them, the
 * same division of labour the desc codes themselves use.
 *
 *   `to`   — the packet's DESTINATION, which is not the address the record's
 *            `tag` carries. `tag` names the node this hop was addressed to (the
 *            relay, on a packet in transport), which is the neighbour a person
 *            watching the air is interacting with and what the peer pills want.
 *            `to` is where the packet is ultimately going, plus the hop count
 *            it has travelled: "3f2a11 h2". Empty where a frame names no
 *            destination at all.
 *   `subj` — what this packet is ABOUT, per desc: the hash a path request asks
 *            for, the packet hash a proof proves, the link id a link request
 *            creates, the aspect an announce serves, the far end of a link.
 *            Empty where the kind has nothing of its own to say.
 *   `hash` — this packet's own hash, six hex characters of it, which is the
 *            name a PROOF for it will carry. Its own field rather than a case
 *            of `subj` because a viewer needs it on the packets that have
 *            something else to say as well: a proof answers a link's data
 *            packet, and that packet's subject is already the far end it is
 *            with. With both, the two can be joined on screen.
 *
 * 24 characters each: enough for a six-hex address and a short qualifier, or an
 * aspect label like `nomadnetwork.node`. */
#define LORA_MON_EXT_MAX 25              /* 24 characters + NUL */
#define LORA_MON_HASH_MAX 7              /* six hex characters + NUL */
struct LoraMonExt {
    char to[LORA_MON_EXT_MAX];
    char subj[LORA_MON_EXT_MAX];
    char hash[LORA_MON_HASH_MAX];
};

/* Aimed at everyone, by what the frame IS. Being a broadcast is a property of
 * the packet's kind, not of an address lookup — a path request goes to a
 * well-known control address that names no node, and an announce's address is
 * the destination it announces — so this is what decides the audience, and the
 * colour that follows from it, on both directions.
 *
 * A list rather than a single code because Reticulum broadcasts in more than
 * one shape: an announce, the path request that asks for one, the path response
 * that answers it, and a tunnel synthesis. Missing one leaves that kind reading
 * as somebody else's unicast. */
bool loraMonIsBroadcast(uint8_t desc);

/* A proof of something, in any of the shapes Reticulum proves in: a packet, a
 * link request, a packet inside a link, a resource. They were one code until
 * the context byte was read, and everything that asks "is this a proof" wants
 * all four — a proof is addressed to the hash of what it proves rather than to
 * a node, which is the property the callers turn on. */
bool loraMonIsProof(uint8_t desc);

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
 * is, which a tail inherits from its head; cast keys off that one. RADIO TASK.
 *
 * `ext`, where given, is filled with the two detail fields — but only while a
 * viewer has asked for them; it comes back empty otherwise, and a split's tail
 * always comes back empty (it carries no header, and its head's answers belong
 * to the head's own record). */
void loraMonClassify(LoraRadio* r, uint8_t dir, const uint8_t* f, size_t len,
                     uint8_t type, uint32_t now,
                     uint8_t* desc, uint8_t* whole, uint8_t tag[3],
                     LoraMonExt* ext = nullptr);

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
                 uint8_t desc, const uint8_t tag[3], uint8_t cast,
                 const LoraMonExt* ext = nullptr);
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
/* Drop everything published about one peer-table slot — the neighbourhood
 * record and the measurements. Both publishers delete an empty slot on their
 * own beat, but neither beat is guaranteed to come: the neighbourhood one runs
 * only while a LoRaMon viewer is open, and the measurement one only when a
 * frame has moved. A node that was just forgotten must not go on being
 * described to every reader outside this binary until something happens to
 * arrive. */
void loraPeerPubForget(LoraRadio* r, int slot);
bool loraMonOpen(void);          /* a LoRaMon viewer (web or LCD) is open */
/* A viewer has the `detailed` toggle on, so records carry their two extra
 * fields. Separate from loraMonOpen because it is a separate appetite and a
 * separate cost: filling the fields reads further into every frame, and the
 * strings ride in every record node for as long as the ring holds it. */
bool loraMonDetail(void);
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

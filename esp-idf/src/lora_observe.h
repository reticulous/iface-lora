#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

/* RNS wire constants (see the header decode in loraTracePacket). */
#define NEI_PT_DATA     0
#define NEI_PT_ANNOUNCE 1
#define NEI_PT_LINKREQ  2
#define NEI_PT_PROOF    3
#define NEI_DT_SINGLE   0
#define NEI_DT_GROUP    1
#define NEI_DT_PLAIN    2
#define NEI_DT_LINK     3
/* The context byte, which rides in the clear beside the packet type and says
 * what the packet is FOR — resource machinery, link lifecycle, a request or its
 * response. RNS/Packet.py's context_types, with the names shortened. */
#define NEI_CTX_NONE      0x00
#define NEI_CTX_RESOURCE  0x01
#define NEI_CTX_RES_ADV   0x02
#define NEI_CTX_RES_REQ   0x03
#define NEI_CTX_RES_HMU   0x04
#define NEI_CTX_RES_PRF   0x05
#define NEI_CTX_RES_ICL   0x06
#define NEI_CTX_RES_RCL   0x07
#define NEI_CTX_CACHE_REQ 0x08
#define NEI_CTX_REQUEST   0x09
#define NEI_CTX_RESPONSE  0x0A
#define NEI_CTX_PATH_RESP 0x0B
#define NEI_CTX_COMMAND   0x0C
#define NEI_CTX_CMD_STAT  0x0D
#define NEI_CTX_CHANNEL   0x0E
#define NEI_CTX_KEEPALIVE 0xFA
#define NEI_CTX_LINKIDENT 0xFB
#define NEI_CTX_LINKCLOSE 0xFC
#define NEI_CTX_LINKPROOF 0xFD
#define NEI_CTX_LRRTT     0xFE
#define NEI_CTX_LRPROOF   0xFF
#define NEI_ECPUBSIZE   64          /* LR ephemeral keys; link_id hashes only these */
#define NEI_RATCHETSIZE 32          /* announce ratchet, present iff ctxflag */

/* Decoded RNS wire header (layout described at loraTracePacket in
 * lora_observe.cpp). */
struct RnsHdr {
    uint8_t        hops, ptype, dtype, ctx;
    bool           hdr2;
    /* Header byte 0 bit 0x20. On an announce it is the one thing that says
     * whether a ratchet sits between random_hash and the signature — the field
     * itself is unmarked. */
    bool           ctxflag;
    const uint8_t* transportId;   /* HEADER_2 only, else null */
    const uint8_t* dest;
    const uint8_t* data;
    size_t         dataLen;
};

/* ─────────────── lora_observe: Reticulum packet inspection ─────────────── */
bool        rnsParse(const uint8_t* p, size_t len, RnsHdr* h);
/** The truncated packet hash — what a proof is addressed to, and what a later
 *  proof NAMES. With `isLr` it is the link id instead: the same hash with any
 *  data past the 64-byte ephemeral keys dropped, which is the address every
 *  packet of that link is then sent to. Static scratch buffer inside: radio
 *  task only. */
void        rnsPacketHash(const RnsHdr* h, const uint8_t* p, size_t len,
                          bool isLr, uint8_t out[16]);
/* `fromPeer` is the node that provably transmitted this packet — a transaction
 * we granted names one — or LORAQ_PEER_NONE when it arrived in the clear and
 * the sender is whoever the frame's own contents imply. */
void        peersObserve(LoraRadio* r, const uint8_t* p, size_t len, bool isTx,
                       int16_t rssi, int16_t snr10, uint8_t txOrigin,
                       uint16_t fromPeer);
/** The aspect behind an announce's name hash, or null where this firmware does
 *  not speak it. Thin pass-through to rnsd's dictionary, which every medium
 *  shares — see rnsdAspectLabel. */
const char* rnsNameLabel(const uint8_t nameHash[10]);
void        loraHex(char* out, const uint8_t* d, size_t n);

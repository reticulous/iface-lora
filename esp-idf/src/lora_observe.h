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
#define NEI_DT_LINK     3
#define NEI_CTX_LRPROOF 0xFF
#define NEI_ECPUBSIZE   64          /* LR ephemeral keys; link_id hashes only these */

/* Decoded RNS wire header (layout described at loraTracePacket in
 * lora_observe.cpp). */
struct RnsHdr {
    uint8_t        hops, ptype, dtype, ctx;
    bool           hdr2;
    const uint8_t* transportId;   /* HEADER_2 only, else null */
    const uint8_t* dest;
    const uint8_t* data;
    size_t         dataLen;
};

/* ─────────────── lora_observe: Reticulum packet inspection ─────────────── */
bool        rnsParse(const uint8_t* p, size_t len, RnsHdr* h);
/* `fromPeer` is the node that provably transmitted this packet — a transaction
 * we granted names one — or LORAQ_PEER_NONE when it arrived in the clear and
 * the sender is whoever the frame's own contents imply. */
void        peersObserve(LoraRadio* r, const uint8_t* p, size_t len, bool isTx,
                       int16_t rssi, int16_t snr10, uint8_t txOrigin,
                       uint16_t fromPeer);
void        rnsNamesInit(void);
const char* rnsNameLabel(const uint8_t nameHash[10]);
void        loraHex(char* out, const uint8_t* d, size_t n);

#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

/* ─────────────── announce buffer ───────────────
 *
 * Every announce this node ORIGINATES (wire hops 0) is copied here as it goes
 * out, keyed by destination hash, replaced when that destination announces
 * again, dropped after an hour. `lora [<n>] a[nnounce]` repeats the lot, one at
 * a time through ordinary channel access, and then this node's own SUPE
 * announcement.
 *
 * **Nothing is held back and nothing is batched.** An announce airs when rnsd
 * hands it over. An earlier design buffered and swallowed them, replayed them
 * in back-to-back bunches on a beat, and followed each run with a transmit
 * power sweep so listeners could measure a floor against this node — all of
 * which is gone. It delayed somebody else's routing decision by up to an
 * announce interval, which plans/SUPE.md §9 rules out, and the sweep measured
 * something SUPE now gets continuously and for free: every frame a detour sends
 * states the power it went out at, so each one yields a path loss rather than a
 * bare reading (§7, §10).
 *
 * What is left is a copy and a command. */
#define ANN_MAX_ENTRIES   16                      /* bounds RAM *and* the manifest, hence the tail's airtime */
#define ANN_MAX_LEN       256                     /* an announce is ~167 B + app_data; longer is not buffered */
#define ANN_TTL_MS        (60u * 60u * 1000u)     /* an hour, replayed or not */
#define ANN_INTERVAL_DEF  30                      /* s.lora.<i>.announce_interval, minutes */
#define ANN_JITTER_PCT    10                      /* ± this much, so a fleet does not synchronise */

struct AnnRec {
    uint8_t  dest[16];
    uint16_t len;
    uint32_t ms;                 /* millis() when stored */
    bool     used;
    uint8_t  data[ANN_MAX_LEN];  /* the RNS packet verbatim — it is signed, so it
                                  * cannot be regenerated and must not be altered */
};
struct AnnBuf { AnnRec e[ANN_MAX_ENTRIES]; };

/* ─────────────── lora_bridge: rnsd registration + the packet paths ──────── */
uint8_t modeFromString(const char* s);
bool registerWithRnsd(LoraRadio* r);
void deregisterFromRnsd(LoraRadio* r);
void rearmRx(LoraRadio* r);
void startTxFrame(LoraRadio* r, int idx);
void beginTx(LoraRadio* r, const uint8_t* data, size_t len, uint8_t origin,
             bool fromBuffer = false, const int8_t* forcePwr = nullptr);
void serviceRadio(LoraRadio* r);
void drainOneOutbound(LoraRadio* r);
int  annCount(const LoraRadio* r);
int  annReplayStart(LoraRadio* r);
void queueFill(LoraRadio* r);
bool stageTx(LoraRadio* r, const uint8_t* data, size_t len, uint8_t origin,
             int8_t pwr);
bool fireStagedTx(LoraRadio* r);
void queueDiscardHead(LoraRadio* r);
void annInit(LoraRadio* r);

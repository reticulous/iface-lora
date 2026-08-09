/**
 * lora_chanplan — the regime's channel tables: which frequencies exist, what
 * a channel's frequency is, and the power cap a channel imposes.
 */
#include "lora_priv.h"

#if defined(CONFIG_LORA0_CS_PIN)

/* ─────────────── regimes ───────────────
 *
 * A regime is a numbered, versioned statement of what is permissible on air:
 * the channels, and per channel the airtime allowance, the transmission-length
 * ceilings and the power limit. It is the thing two nodes name when they agree
 * how to behave, so the number is the negotiation currency and the table is
 * resolved locally at each end.
 *
 * `s.lora.<n>.afa` IS the regime number, and it is the same number SUPE names:
 * the regime is the statement of what is permissible on which channels, so a
 * second key would be a second answer to one question.
 *
 * **0 means no agile channels, and it is also SUPE's regime 0.** Those are the
 * same thing read two ways rather than a contradiction: regime 0 has no channel
 * plan at all — its whole ladder is the spreading factors above the hailing one,
 * on the channel the network already hails on — so resolving 0 to an empty
 * channel set is correct under both readings. What changes with SUPE enabled is
 * that a node with `afa` at 0 detours by spreading factor alone; what does not
 * change is that it never leaves the hailing frequency. 1 is the EU 863-870 MHz
 * plan below. An unrecognised number resolves to no agile channels, which is the
 * safe reading of a value this firmware cannot understand.
 *
 * The airtime allowance is a seconds-per-window PAIR rather than a percentage,
 * because that is what makes the table portable across regulators: EU polite
 * spectrum access is 100 s per 3600 s, an EU duty cycle is 360 s per 3600 s,
 * and US frequency-hopping dwell is 0.4 s per 20 s. One field pair, three
 * regulatory shapes, and nothing downstream has to special-case any of them.
 *
 * Nothing enforces these yet — no transmission ever leaves channel 0. The table
 * exists now so that records, measurements and the UI are all written against
 * channel indices from the start rather than retrofitted later. */

/* Regime 1 — EU 863-870 MHz, nine 500 kHz channels on polite spectrum access
 * (100 s/h each), 25 mW e.r.p. Channel 0 is absent from this table: it is the
 * hailing channel and takes the radio's configured frequency and bandwidth,
 * because that is a user choice and not ours to fix. */
static const RegimeChan kRegime1[] = {
    { 863350000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 864050000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 864750000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 865450000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 866150000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 866850000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 867550000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 868250000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
    { 868950000, 500000, 100.0f, 3600, 1.0f, 4.0f, 14 },
};

/* The agile channels of a regime, or none at all for regime 0. */
const RegimeChan* regimeChans(uint8_t regime, int* count) {
    switch (regime) {
        case 1: *count = (int)(sizeof kRegime1 / sizeof kRegime1[0]); return kRegime1;
        default: *count = 0; return nullptr;
    }
}

/* ─────────────── channels, airtime and the transmit verdict ─────────────── */

uint32_t supeChanFreq(const LoraRadio* r, uint8_t chan) {
    if (chan == SUPE_CH_HAIL) return r->cfgFreqHz;
    int n = 0;
    const SupeChan* c = supeRegimeChans(r->afa, &n);
    if (!c || chan > n) return r->cfgFreqHz;
    return c[chan - 1].freqHz;
}

/* The most this radio may transmit at on a given channel. The hailing channel
 * is the operator's and takes `tx_power` as configured; a regime's own channels
 * carry the regime's limit, which is a **regulatory** figure — regime 1's 14 dBm
 * is 25 mW e.r.p. from EN 300 220 annex B, not a preference. Transmitting the
 * configured 22 dBm there is simply not allowed, whatever the link would enjoy. */
int8_t supeChanTxpCap(LoraRadio* r, uint8_t chan) {
    if (chan == SUPE_CH_HAIL) return r->cfgTxp;
    const SupeRegime* g = supeRegime(r->afa);
    if (!g || g->maxTxpDbm == SUPE_TXP_IFACE) return r->cfgTxp;
    return r->cfgTxp < g->maxTxpDbm ? r->cfgTxp : g->maxTxpDbm;
}

#endif  /* CONFIG_LORA0_CS_PIN */

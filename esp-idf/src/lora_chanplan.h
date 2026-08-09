#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

/* Regime channel entry — the table and lookup live in lora_chanplan.cpp. */
struct RegimeChan {
    uint32_t freqHz;
    uint32_t bwHz;
    float    airtimeMaxS;    /* transmit seconds allowed … */
    uint32_t airtimeWinS;    /* … in any window of this many seconds */
    float    maxTxS;         /* ceiling on one transmission */
    float    maxTxTxnS;      /* ceiling on a dialogue or polling sequence */
    int8_t   maxTxpDbm;
};

/* ─────────────── lora_chanplan: the regime's channel tables ─────────────── */
const RegimeChan* regimeChans(uint8_t regime, int* count);
uint32_t supeChanFreq(const LoraRadio* r, uint8_t chan);
int8_t   supeChanTxpCap(LoraRadio* r, uint8_t chan);

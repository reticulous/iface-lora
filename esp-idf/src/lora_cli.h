#pragma once
/* Included by lora_priv.h in dependency order; module code includes
 * lora_priv.h, not this file directly. */
struct LoraRadio;

/* ─────────────── lora_cli ─────────────── */
void cliLora(const char* args);
void manualTxPoll(LoraRadio* r);
/* Service a pending `lora forget` on the task that owns the peer table. */
void peersForgetPoll(LoraRadio* r);

#pragma once
/* Time-on-air, per Semtech AN1200.13 — the one copy of it.
 *
 * Symbol time Tsym = 2^SF / BW; the preamble runs (n+4.25) symbols and the
 * payload rounds up into whole symbols, with low-data-rate optimisation (DE)
 * engaged once a symbol exceeds 16 ms.
 *
 * It lives in a header, alone, because both halves of this straddle need it and
 * neither can reach the other: SUPE's portable core compiles with a plain g++
 * (see test/) so it cannot call into the driver, and the driver has to time a
 * frame in a build with no SUPE in it at all (CONFIG_LORA_NO_SUPE). A second
 * copy is how the radio-check sweep's airtime came to be over-stated by half —
 * once was enough. Header-only and free of ESP-IDF for the same reason.
 *
 * `implicitHeader` is NOT optional bookkeeping: a headerless frame drops the
 * 20-bit header from the payload term. `crc` likewise — a SUPE frame flies
 * without one, every frame the driver asks about carries one. */
#include <math.h>
#include <stdint.h>

static inline double loraToaSeconds(int sf, int bw_hz, int cr_denom, int preamble,
                                    int payload, bool implicitHeader, bool crc) {
    if (sf <= 0 || bw_hz <= 0 || cr_denom < 5 || cr_denom > 8) return 0.0;
    double tSym = (double)((uint32_t)1 << sf) / (double)bw_hz;
    int    de   = (tSym > 0.016) ? 1 : 0;
    int    cr   = cr_denom - 4;
    double num  = 8.0 * payload - 4.0 * sf + 28.0
                  + (crc ? 16.0 : 0.0)
                  - (implicitHeader ? 20.0 : 0.0);
    double den  = 4.0 * (sf - 2 * de);
    double payloadSym = 8.0 + fmax(ceil(num / den) * (cr + 4), 0.0);
    return (preamble + 4.25) * tSym + payloadSym * tSym;
}

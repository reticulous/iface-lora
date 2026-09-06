/* lora_rfcal.cpp — the RF calibration arithmetic. See lora_rfcal.h. */

#include "lora_rfcal.h"

#include <stdio.h>
#include <stdlib.h>

void rfCalReset(LoraRfCal* c, int lo, int hi)
{
    c->n = 0;
    c->grade = CAL_NONE;
    c->flatGainDb = 0;
    c->rxGainDb = 0;
    c->lo = (int8_t)lo;
    c->hi = (int8_t)hi;
}

const char* rfCalName(uint8_t grade)
{
    switch (grade) {
    case CAL_MEASURED:  return "measured";
    case CAL_DATASHEET: return "datasheet";
    default:            return "none";
    }
}

int8_t rfCalAntenna(const LoraRfCal* c, int8_t chipDbm)
{
    if (c->n < 2) return (int8_t)(chipDbm + c->flatGainDb);

    const int n = c->n;
    if (chipDbm <= c->chip[0])     return c->ant[0];
    if (chipDbm >= c->chip[n - 1]) return c->ant[n - 1];
    for (int i = 1; i < n; i++) {
        if (chipDbm > c->chip[i]) continue;
        const int x0 = c->chip[i - 1], x1 = c->chip[i];
        const int y0 = c->ant[i - 1],  y1 = c->ant[i];
        /* Rounded rather than truncated: half a dB either way is the whole
         * resolution the register and the wire have between them. */
        const int num = (y1 - y0) * (chipDbm - x0);
        const int den = x1 - x0;
        return (int8_t)(y0 + (num >= 0 ? (num + den / 2) / den
                                       : -((-num + den / 2) / den)));
    }
    return c->ant[n - 1];
}

int8_t rfCalChip(const LoraRfCal* c, int8_t antennaDbm)
{
    /* No range yet means rfCalReset has not run, so there is nothing to
     * convert against and the request stands. Nothing should reach here before
     * femInit; converting to register 0 if something did would be far worse
     * than passing the number through. */
    if (c->hi <= c->lo) return antennaDbm;

    int best = c->lo, bestAnt = rfCalAntenna(c, c->lo);
    for (int chip = c->lo; chip <= c->hi; chip++) {
        const int a = rfCalAntenna(c, (int8_t)chip);
        if (a >= antennaDbm) return (int8_t)chip;   /* ascending, so the lowest */
        if (a > bestAnt) { bestAnt = a; best = chip; }
    }
    return (int8_t)best;
}

int8_t rfCalMaxDbm(const LoraRfCal* c)
{
    int mx = rfCalAntenna(c, c->lo);
    for (int chip = c->lo + 1; chip <= c->hi; chip++) {
        const int a = rfCalAntenna(c, (int8_t)chip);
        if (a > mx) mx = a;
    }
    return (int8_t)mx;
}

int8_t rfCalMinDbm(const LoraRfCal* c)
{
    return rfCalAntenna(c, c->lo);
}

float rfCalRssi(const LoraRfCal* c, float chipRssi)
{
    return chipRssi - (float)c->rxGainDb;
}

/* ─────────────── spec parsing ─────────────── */

static bool wordIs(const char* p, size_t n, const char* word)
{
    size_t i = 0;
    for (; i < n && word[i]; i++) {
        char a = p[i], b = word[i];
        if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
        if (b >= 'A' && b <= 'Z') b = (char)(b - 'A' + 'a');
        if (a != b) return false;
    }
    return i == n && !word[i];
}

static const char* skipSpace(const char* p)
{
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

/* The next word at p, its length out through `len`. */
static const char* nextWord(const char* p, size_t* len)
{
    p = skipSpace(p);
    const char* s = p;
    while (*p && *p != ' ' && *p != '\t' && *p != ';') p++;
    *len = (size_t)(p - s);
    return s;
}

bool rfCalParse(LoraRfCal* c, const char* spec, const char* part,
                char* err, size_t errLen)
{
    if (err && errLen) err[0] = '\0';
    if (!spec || !*spec || !part) return false;

    for (const char* seg = spec; *seg; ) {
        size_t pn = 0, gn = 0;
        const char* name  = nextWord(seg, &pn);
        const char* grade = nextWord(name + pn, &gn);
        const char* pts   = grade + gn;
        /* Advance now, so every exit below lands on the next entry. */
        const char* next = pts;
        while (*next && *next != ';') next++;
        if (*next == ';') next++;

        if (!wordIs(name, pn, part)) { seg = next; continue; }

        const uint8_t g = wordIs(grade, gn, "measured")  ? CAL_MEASURED
                        : wordIs(grade, gn, "datasheet") ? CAL_DATASHEET
                        : CAL_NONE;
        if (g == CAL_NONE) {
            if (err) snprintf(err, errLen, "%s: grade must be measured or datasheet", part);
            return false;
        }

        int8_t chip[RF_CAL_MAX_POINTS], ant[RF_CAL_MAX_POINTS];
        int n = 0, last = -128;
        for (const char* p = skipSpace(pts); *p && *p != ';'; ) {
            char* end = nullptr;
            const long reg = strtol(p, &end, 10);
            if (end == p || *end != ':') break;
            p = end + 1;
            const long a = strtol(p, &end, 10);
            if (end == p) break;
            p = skipSpace(end);
            if (*p == ',') p = skipSpace(p + 1);

            if (n >= RF_CAL_MAX_POINTS) {
                if (err) snprintf(err, errLen, "%s: more than %d points",
                                  part, RF_CAL_MAX_POINTS);
                return false;
            }
            if (reg <= last || reg < -128 || reg > 127 || a < -128 || a > 127) {
                if (err) snprintf(err, errLen, "%s: bad point %ld:%ld", part, reg, a);
                return false;
            }
            chip[n] = (int8_t)reg;
            ant[n]  = (int8_t)a;
            last    = (int)reg;
            n++;
        }
        if (n < 2) {
            if (err) snprintf(err, errLen, "%s: needs two points or more", part);
            return false;
        }
        for (int i = 0; i < n; i++) { c->chip[i] = chip[i]; c->ant[i] = ant[i]; }
        c->n = (uint8_t)n;
        c->grade = g;
        return true;
    }
    return false;   /* this spec says nothing about this part, which is fine */
}

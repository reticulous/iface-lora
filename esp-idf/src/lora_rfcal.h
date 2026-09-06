/* ─────────────── lora_rfcal: the RF calibration itself ───────────────
 *
 * What a radio register setting actually puts on the antenna connector, and
 * back again. This is the arithmetic only — no GPIOs, no storage, no chip —
 * so lora_fem owns the hardware and this owns the numbers.
 *
 * Deliberately free of ESP-IDF, RadioLib and FreeRTOS, like supe.cpp and for
 * the same reason: test/ compiles it with a plain g++. If this file starts
 * needing an IDF header, something has leaked in that belongs in lora_fem.cpp.
 *
 * The model is a curve rather than a gain because neither half of the error is
 * constant: the chip's own set-versus-actual output drifts several dB at the
 * extremes, an amplifier compresses as it saturates, and a board may sit a
 * fixed pad between the two. A curve holds measured (register, connector)
 * points, straight lines between them and flat outside; with fewer than two
 * points the model falls back to a flat gain, which is identity when there is
 * no front end either.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

/* Points in one curve. The longest in the tree is the Heltec V4's nine. */
#define RF_CAL_MAX_POINTS  12

/* How well the numbers a radio reports are known. Carried in the LORAn_TX_CAL
 * entry itself, so provenance cannot drift away from the data it describes,
 * and published as lora.<n>.cal. */
enum LoraCalGrade : uint8_t {
    CAL_NONE = 0,   /* uncalibrated: the flat gain, or identity */
    CAL_DATASHEET,  /* derived from the part's published figures */
    CAL_MEASURED,   /* somebody put this board on an analyser */
};

struct LoraRfCal {
    int8_t   chip[RF_CAL_MAX_POINTS];   /* register dBm, strictly ascending */
    int8_t   ant[RF_CAL_MAX_POINTS];    /* connector dBm at that setting */
    uint8_t  n;             /* points; < 2 = the flat model */
    uint8_t  grade;         /* LoraCalGrade */
    int8_t   flatGainDb;    /* what the flat model adds (0 = identity) */
    int8_t   rxGainDb;      /* receive gain in front of the chip */
    int8_t   lo, hi;        /* register settings this port will accept */
};

/* Start from the flat model over a port's register range: no curve, no gain,
 * grade none. Everything else is layered on by the caller. */
void rfCalReset(LoraRfCal* c, int lo, int hi);

/* What a register setting puts on the connector. */
int8_t rfCalAntenna(const LoraRfCal* c, int8_t chipDbm);

/* The register setting for a wanted connector power: the LOWEST one that
 * reaches it, which on a curve that turns over near saturation is both the
 * cheapest and the strongest. A request nothing reaches lands on the best
 * setting there is. */
int8_t rfCalChip(const LoraRfCal* c, int8_t antennaDbm);

/* The ends of the range this board actually reaches: the curve's own maximum
 * — which is not the value at the top setting when the curve turns over — and
 * the output at the bottom setting, a floor an amplified board cannot go
 * under however low the radio is driven. */
int8_t rfCalMaxDbm(const LoraRfCal* c);
int8_t rfCalMinDbm(const LoraRfCal* c);

/* A level read at the chip, referred back to the connector. The receive gain
 * is the whole of it: gain in front raises signal and noise together, so SNR
 * needs no correction and must not be given one. */
float rfCalRssi(const LoraRfCal* c, float chipRssi);

/* Load the entry for `part` out of a LORAn_TX_CAL spec:
 *
 *     <part> <grade> <reg>:<ant>,<reg>:<ant>,... ; <part> <grade> ...
 *
 * Returns true when a curve was installed. False leaves the flat model intact
 * — a curve read wrong is worse than no curve at all — and, when the spec was
 * malformed rather than merely silent about this part, writes why into `err`.
 * A part the spec does not mention is not an error. */
bool rfCalParse(LoraRfCal* c, const char* spec, const char* part,
                char* err, size_t errLen);

const char* rfCalName(uint8_t grade);

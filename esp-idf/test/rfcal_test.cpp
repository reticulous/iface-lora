/* rfcal_test — the RF calibration arithmetic, on the host.
 *
 * This is the code that decides what a radio is actually driven at, on
 * hardware nobody can watch from here, so it is worth pinning down: the
 * inverse search on a curve that turns over, the interpolation, the range the
 * conversion reports, and a parser fed the kind of string a board writes by
 * hand. */

#include "../src/lora_rfcal.h"

#include <cstdio>
#include <cstring>

static int failures = 0;

static void ck(bool ok, const char* what, long got, long want)
{
    if (ok) return;
    printf("FAIL  %s: got %ld, want %ld\n", what, got, want);
    failures++;
}

static void eq(int got, int want, const char* what)
{
    ck(got == want, what, got, want);
}

/* The Heltec V4's GC1109 revision, measured. Note the turnover at the top:
 * register 20 gives more than 22 does. */
static const char* kHeltec =
    "gc1109 measured 1:7,5:12,10:20,12:23,14:24,16:25,18:27,20:28,22:27;"
    "kct8103l datasheet -9:6,12:27,22:27";

static LoraRfCal heltec(const char* part)
{
    LoraRfCal c;
    rfCalReset(&c, -9, 21);
    char err[80] = {0};
    if (!rfCalParse(&c, kHeltec, part, err, sizeof err))
        printf("FAIL  parse %s: %s\n", part, err[0] ? err : "no entry");
    return c;
}

static void testFlat()
{
    LoraRfCal c;
    rfCalReset(&c, -9, 21);
    eq(rfCalAntenna(&c, 14), 14, "flat: identity");
    eq(rfCalChip(&c, 14), 14, "flat: identity inverse");
    eq(rfCalMinDbm(&c), -9, "flat: floor is the register floor");
    eq(rfCalMaxDbm(&c), 21, "flat: ceiling is the register ceiling");
    eq(rfCalChip(&c, 30), 21, "flat: unreachable request tops out");
    eq(rfCalChip(&c, -40), -9, "flat: request under the floor lands on it");
    eq((int)rfCalRssi(&c, -100.0f), -100, "flat: rssi unchanged");

    c.flatGainDb = 13;
    eq(rfCalAntenna(&c, 0), 13, "declared: gain added");
    eq(rfCalChip(&c, 13), 0, "declared: gain removed");
    eq(rfCalMinDbm(&c), 4, "declared: floor lifts with the gain");
}

static void testCurve()
{
    const LoraRfCal c = heltec("gc1109");
    eq(c.grade, CAL_MEASURED, "grade travels with the data");

    /* Points land on themselves. */
    eq(rfCalAntenna(&c, 1), 7, "point 1");
    eq(rfCalAntenna(&c, 10), 20, "point 10");
    eq(rfCalAntenna(&c, 20), 28, "point 20");
    eq(rfCalAntenna(&c, 22), 27, "point 22 — below the peak");

    /* Between points, a straight line. 5:12 → 10:20 is 1.6 dB per step. */
    eq(rfCalAntenna(&c, 6), 14, "interpolated 6");
    eq(rfCalAntenna(&c, 8), 17, "interpolated 8");

    /* Outside, flat — which is the whole reason the floor is what it is. */
    eq(rfCalAntenna(&c, -9), 7, "below the lowest point, flat");
    eq(rfCalMinDbm(&c), 7, "an amplified board cannot transmit quietly");

    /* The ceiling is the curve's peak, not its last point, and the setting
     * that reaches it is the lowest one that does — never 22, which sits below
     * the peak while drawing the most current of any setting on the part.
     *
     * 19 rather than 20 because the curve is stored in whole dB: 20 measured
     * +27.7, which rounds to 28, so the straight line from 18 reaches 28 half
     * a step early. That is inside the measurement's own +-1 dB and it favours
     * the cheaper setting, which is the direction to be wrong in. */
    eq(rfCalMaxDbm(&c), 28, "ceiling is the peak");
    eq(rfCalChip(&c, 28), 19, "full power picks the lowest setting that reaches it");
    eq(rfCalChip(&c, 99), 19, "unreachable request lands on the peak, not the top");

    /* The case the board actually runs: its 27 dBm rating caps the ceiling,
     * and a full-power request then resolves to 18 — the setting the
     * measurements call the practical maximum. 22 is never chosen at all. */
    eq(rfCalChip(&c, 27), 18, "capped full power is register 18");

    /* Ordinary requests: the lowest setting that reaches, never one below. */
    eq(rfCalChip(&c, 20), 10, "exact point");
    eq(rfCalChip(&c, 21), 11, "between points, rounds up to reach");
    eq(rfCalChip(&c, -20), -9, "a request under the floor still lands on the floor");
}

static void testSecondPart()
{
    const LoraRfCal c = heltec("kct8103l");
    eq(c.grade, CAL_DATASHEET, "second entry keeps its own grade");
    eq(rfCalMinDbm(&c), 6, "datasheet floor");
    eq(rfCalAntenna(&c, 12), 27, "datasheet ceiling reached at 12");
    eq(rfCalChip(&c, 27), 12, "and 12 is the cheapest way to it");
    eq(rfCalMaxDbm(&c), 27, "saturated above that");
}

/* A radio whose calibration has not been built yet — the state a zeroed
 * LoraRadio is in before femInit — must pass powers through, not convert them
 * against a range of nothing. */
static void testUnset()
{
    LoraRfCal c = {};
    eq(rfCalChip(&c, 14), 14, "unset range: request stands");
    eq(rfCalChip(&c, -9), -9, "unset range: floor request stands");
}

static void testRxGain()
{
    LoraRfCal c;
    rfCalReset(&c, -9, 21);
    c.rxGainDb = 20;
    ck(rfCalRssi(&c, -12.0f) == -32.0f, "rssi referred to the connector",
       (long)rfCalRssi(&c, -12.0f), -32);
}

static void testParser()
{
    LoraRfCal c;
    char err[80];

    /* A part the spec is silent about is not an error, and leaves the flat
     * model in place. */
    rfCalReset(&c, -9, 21);
    err[0] = 'x';
    ck(!rfCalParse(&c, kHeltec, "none", err, sizeof err), "silent part: no curve", 1, 0);
    eq(c.n, 0, "silent part: flat model kept");
    eq(err[0], '\0', "silent part: no complaint");

    /* Malformed entries leave the radio uncalibrated rather than half so. */
    struct { const char* spec; const char* what; } bad[] = {
        { "gc1109 guessed 1:7,5:12",  "unknown grade" },
        { "gc1109 measured 5:12,1:7", "points out of order" },
        { "gc1109 measured 5:12",     "only one point" },
        { "gc1109 measured",          "no points" },
    };
    for (auto& b : bad) {
        rfCalReset(&c, -9, 21);
        err[0] = '\0';
        ck(!rfCalParse(&c, b.spec, "gc1109", err, sizeof err), b.what, 1, 0);
        eq(c.n, 0, b.what);
        ck(err[0] != '\0', "malformed entry explains itself", 0, 1);
    }

    /* Case and spacing are the board author's business, not ours. */
    rfCalReset(&c, -9, 21);
    ck(rfCalParse(&c, "GC1109 Measured 0:10, 10:20", "gc1109", err, sizeof err),
       "case-insensitive, spaces tolerated", 0, 1);
    eq(rfCalAntenna(&c, 5), 15, "and it interpolates");
}

int main()
{
    testFlat();
    testCurve();
    testSecondPart();
    testUnset();
    testRxGain();
    testParser();
    if (failures) { printf("%d failure(s)\n", failures); return 1; }
    printf("rfcal: ok\n");
    return 0;
}

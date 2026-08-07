/**
 * rolling — a one-hour running total kept in six ten-minute buckets.
 *
 * Private to iface-lora: transmit seconds per channel, and whatever else wants
 * an hour of history without a ring buffer per figure. Instances link
 * themselves into one list when constructed, so a single Rolling1h::shiftAll()
 * every ten minutes ages every total at once.
 *
 * Nothing here knows what is being summed — the units are the caller's.
 *
 * Instances must outlive the program: there is no unlink, so these belong in
 * statics, globals or long-lived structs, never on a stack or in anything that
 * is freed.
 */
#ifndef IFACE_LORA_ROLLING_H
#define IFACE_LORA_ROLLING_H

#include <stdint.h>

class Rolling1h {
public:
    static const int kBuckets       = 6;    /* 6 × 10 min = 1 h */
    static const int kBucketMinutes = 10;

    Rolling1h();

    /** Credit the current bucket. */
    void add(float v);

    /** Total over the last `minutes`, rounded down to whole buckets and
     *  clamped to [10, 60]. The newest bucket is still filling, so a total is
     *  "the last N minutes up to now" rather than a settled window — fine for
     *  comparing one of these against another, not for a fixed-window average. */
    float total(int minutes) const;

    void reset();

    /** Age every instance by one bucket. Call once per kBucketMinutes. */
    static void shiftAll();

private:
    float      _b[kBuckets];                /* _b[0] is current, oldest last */
    Rolling1h* _next;
    static Rolling1h* _head;
};

#endif

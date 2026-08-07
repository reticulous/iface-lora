#include "rolling.h"

/* Constant-initialised, so it is set before any instance's constructor can run
 * whatever the static init order turns out to be. */
Rolling1h* Rolling1h::_head = nullptr;

Rolling1h::Rolling1h() : _next(_head) {
    for (int i = 0; i < kBuckets; i++) _b[i] = 0.0f;
    _head = this;
}

void Rolling1h::add(float v) { _b[0] += v; }

void Rolling1h::reset() { for (int i = 0; i < kBuckets; i++) _b[i] = 0.0f; }

float Rolling1h::total(int minutes) const {
    int n = minutes / kBucketMinutes;
    if (n < 1)        n = 1;
    if (n > kBuckets) n = kBuckets;
    float s = 0.0f;
    for (int i = 0; i < n; i++) s += _b[i];
    return s;
}

void Rolling1h::shiftAll() {
    for (Rolling1h* p = _head; p; p = p->_next) {
        for (int i = kBuckets - 1; i > 0; i--) p->_b[i] = p->_b[i - 1];
        p->_b[0] = 0.0f;
    }
}

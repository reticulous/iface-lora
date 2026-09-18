/**
 * virtual_sx126x — an SX1262 that answers on a virtual bus.
 *
 * The driver above it is the one that runs on hardware, unchanged: it writes
 * the same opcodes, reads the same registers, waits on the same DIO1 line and
 * reads the same IRQ bits. What this supplies is the other end of the bus —
 * a command interpreter, a register file, a payload buffer, and the timing of
 * a frame, which is where a radio actually lives.
 *
 * A frame in flight is three instants: the end of its preamble, the end of its
 * header, and the end of the frame. The model schedules the receiver's
 * interrupts on those instants with esp_timer one-shots, so a driver that
 * disables its interrupt, drains, and re-enables sees exactly the edges it
 * sees on a board.
 *
 * What is between two radios is the ether (ether_task.cpp): the model hands it
 * every transmission and every change of mode or carrier, and is handed back
 * the frames that reach its antenna.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

/** A frame arriving at this receiver: when its stages land, relative to the
 *  `t0` the sender stamped, and how strongly it arrives. */
struct VirtualRxBegin {
    int     id;
    int64_t t0, tPre, tHdr, tEnd;   /* the sender's own microsecond stamps */
    int     levelDbm;
};

/** The same frame, finished: what it carried and how it came out. */
struct VirtualRxEnd {
    int            id;
    const uint8_t* payload;
    size_t         len;
    bool           crcOk;       /* false when the ether's verdict is not clean */
    bool           headerOk;    /* false when the header itself did not survive */
    int            rssiDbm;
    int            snrDb;
};

class VirtualSx126x {
public:
    explicit VirtualSx126x(int slot);

    /** One complete SPI frame: `out[0]` is the opcode, `in` is filled with the
     *  reply — the status byte until the data starts, then the data. */
    void transfer(const uint8_t* out, size_t len, uint8_t* in);

    /** The RST line's rising edge. */
    void reset();

    /** The DIO1 line as the model currently drives it. */
    bool dio1High() const;

    void onRxBegin(const VirtualRxBegin& f);
    void onRxEnd(const VirtualRxEnd& f);

    /** The noise floor this receiver reports when nothing is arriving. */
    static constexpr int kNoiseFloorDbm = -110;

    /* The state, and the timer callbacks that reach it. Defined in the
     * implementation file and opaque everywhere else. */
    struct Impl;
    Impl* d;
};

/** The model for a radio slot, created with the slot's HAL. */
VirtualSx126x* virtualChip(int slot);

/** How many slots this build has models for. */
int virtualChipCount(void);

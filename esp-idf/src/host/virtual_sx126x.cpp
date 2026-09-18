/**
 * virtual_sx126x — see the header.
 *
 * Everything that touches the model's state does so under one critical
 * section, because the radio task reaches it through the bus and the ether
 * task and the timer task reach it from the other side. The DIO1 line is
 * driven outside that section: raising it runs the driver's interrupt handler
 * on the calling task, and a handler ends in a yield.
 */
#include "virtual_sx126x.h"

#include "ether_task.h"
#include "../lora_toa.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <unistd.h>

/* ---- The SX126x command and register surface the driver uses ---- */

enum {
    CMD_RESET_STATS        = 0x00,
    CMD_CLEAR_IRQ_STATUS   = 0x02,
    CMD_CLEAR_DEVICE_ERR   = 0x07,
    CMD_SET_DIO_IRQ_PARAMS = 0x08,
    CMD_WRITE_REGISTER     = 0x0D,
    CMD_WRITE_BUFFER       = 0x0E,
    CMD_GET_STATS          = 0x10,
    CMD_GET_PACKET_TYPE    = 0x11,
    CMD_GET_IRQ_STATUS     = 0x12,
    CMD_GET_RX_BUF_STATUS  = 0x13,
    CMD_GET_PACKET_STATUS  = 0x14,
    CMD_GET_RSSI_INST      = 0x15,
    CMD_GET_DEVICE_ERRORS  = 0x17,
    CMD_READ_REGISTER      = 0x1D,
    CMD_READ_BUFFER        = 0x1E,
    CMD_SET_STANDBY        = 0x80,
    CMD_SET_RX             = 0x82,
    CMD_SET_TX             = 0x83,
    CMD_SET_SLEEP          = 0x84,
    CMD_SET_RF_FREQUENCY   = 0x86,
    CMD_SET_CAD_PARAMS     = 0x88,
    CMD_CALIBRATE          = 0x89,
    CMD_SET_PACKET_TYPE    = 0x8A,
    CMD_SET_MODULATION     = 0x8B,
    CMD_SET_PACKET_PARAMS  = 0x8C,
    CMD_SET_TX_PARAMS      = 0x8E,
    CMD_SET_BUFFER_BASE    = 0x8F,
    CMD_SET_RXTX_FALLBACK  = 0x93,
    CMD_SET_PA_CONFIG      = 0x95,
    CMD_SET_REGULATOR      = 0x96,
    CMD_SET_DIO3_TCXO      = 0x97,
    CMD_CALIBRATE_IMAGE    = 0x98,
    CMD_SET_DIO2_RF_SWITCH = 0x9D,
    CMD_STOP_TIMER_ON_PRE  = 0x9F,
    CMD_SET_LORA_SYMB_TO   = 0xA0,
    CMD_GET_STATUS         = 0xC0,
    CMD_SET_FS             = 0xC1,
    CMD_SET_CAD            = 0xC5,
};

enum {
    IRQ_TX_DONE           = 1u << 0,
    IRQ_RX_DONE           = 1u << 1,
    IRQ_PREAMBLE_DETECTED = 1u << 2,
    IRQ_SYNC_WORD_VALID   = 1u << 3,
    IRQ_HEADER_VALID      = 1u << 4,
    IRQ_HEADER_ERR        = 1u << 5,
    IRQ_CRC_ERR           = 1u << 6,
};

enum {
    REG_VERSION_STRING  = 0x0320,
    REG_IQ_CONFIG       = 0x0736,
    REG_SYNC_WORD_MSB   = 0x0740,
    REG_SYNC_WORD_LSB   = 0x0741,
    REG_SENSITIVITY     = 0x0889,
    REG_RX_GAIN         = 0x08AC,
    REG_TX_CLAMP        = 0x08D8,
    REG_OCP             = 0x08E7,
    REG_RTC_CTRL        = 0x0902,
    REG_EVENT_MASK      = 0x0944,
};

/* The status byte's mode field, and the one command-status value that is
 * neither an error nor silence. */
enum {
    ST_STDBY_RC   = 0x20,
    ST_STDBY_XOSC = 0x30,
    ST_FS         = 0x40,
    ST_RX         = 0x50,
    ST_TX         = 0x60,
    ST_DATA_AVAIL = 0x04,
    ST_CMD_INVALID = 0x08,
};

/* The LoRa bandwidth register codes, in hertz. */
static uint32_t bwFromCode(uint8_t code)
{
    switch (code) {
        case 0x00: return 7810;
        case 0x08: return 10420;
        case 0x01: return 15630;
        case 0x09: return 20830;
        case 0x02: return 31250;
        case 0x0A: return 41670;
        case 0x03: return 62500;
        case 0x04: return 125000;
        case 0x05: return 250000;
        case 0x06: return 500000;
        default:   return 125000;
    }
}

struct VirtualSx126x::Impl {
    int slot;

    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    /* Chip state */
    const char* mode = "STDBY_RC";
    uint8_t     modeBits = ST_STDBY_RC;
    uint8_t     fallbackBits = ST_STDBY_RC;
    const char* fallbackMode = "STDBY_RC";

    uint16_t irqStatus = 0;
    uint16_t irqMask = 0;
    uint16_t dio1Mask = 0;

    std::map<uint16_t, uint8_t> regs;
    uint8_t  buf[256] = {};
    uint8_t  txBase = 0, rxBase = 0;

    uint32_t freqHz = 869525000;
    uint32_t bwHz = 125000;
    int      sf = 8;
    int      cr = 5;
    int      preamble = 8;
    bool     hdrImplicit = false;
    bool     crcOn = true;
    uint8_t  payloadLen = 0;
    int      powerDbm = 14;

    uint8_t  rxLen = 0, rxPtr = 0;
    uint8_t  rssiPkt = 220, snrPkt = 40, sigRssiPkt = 220;
    int      inFlightLevel = 0;      /* 0 = nothing arriving */

    int      txId = 0;

    /* Every scheduled instant of a frame in flight, in or out. */
    esp_timer_handle_t tTxDone = nullptr;
    esp_timer_handle_t tPre = nullptr, tHdr = nullptr, tEnd = nullptr;
    VirtualRxEnd pendingEnd = {};
    uint8_t      pendingPayload[256] = {};
    size_t       pendingLen = 0;
    bool         pendingValid = false;

    uint8_t status() const { return (uint8_t)(modeBits | ST_DATA_AVAIL); }
};

namespace {

VirtualSx126x* s_chips[8] = {};
int            s_chipCount = 0;

/* The sync word as the ether matches on it: the two nibble-expanded register
 * bytes read back as the one 8-bit word the driver set. */
uint8_t syncWordOf(const VirtualSx126x::Impl& d)
{
    auto msb = d.regs.find(REG_SYNC_WORD_MSB);
    auto lsb = d.regs.find(REG_SYNC_WORD_LSB);
    uint8_t m = msb == d.regs.end() ? 0x14 : msb->second;
    uint8_t l = lsb == d.regs.end() ? 0x24 : lsb->second;
    return (uint8_t)((m & 0xF0) | ((l & 0xF0) >> 4));
}

void fillState(const VirtualSx126x::Impl& d, EtherState& s)
{
    s.slot        = d.slot;
    s.mode        = d.mode;
    s.readyAt     = esp_timer_get_time();   /* transitions are instantaneous here */
    s.freqHz      = d.freqHz;
    s.bwHz        = d.bwHz;
    s.sf          = d.sf;
    s.cr          = d.cr;
    s.syncWord    = syncWordOf(d);
    s.hdrImplicit = d.hdrImplicit;
    s.crc         = d.crcOn;
    s.preamble    = d.preamble;
}

}  // namespace

/* ---- Construction ---- */

VirtualSx126x::VirtualSx126x(int slot)
{
    d = new Impl();
    d->slot = slot;

    /* The datasheet reset values the driver read-modify-writes. Anything it
     * writes and reads back lands in the map on the way past; these are the
     * ones it reads before it has written them. */
    d->regs[REG_SENSITIVITY] = 0x94;
    d->regs[REG_TX_CLAMP]    = 0x18;
    d->regs[REG_IQ_CONFIG]   = 0x0D;
    d->regs[REG_RTC_CTRL]    = 0x00;
    d->regs[REG_EVENT_MASK]  = 0x00;
    d->regs[REG_RX_GAIN]     = 0x94;
    d->regs[REG_OCP]         = 0x38;
    d->regs[REG_SYNC_WORD_MSB] = 0x14;
    d->regs[REG_SYNC_WORD_LSB] = 0x24;

    /* The part answers for itself at the version register. An SX1262 reports
     * "SX1261" there — the two are the same silicon — and that is the string
     * the driver matches on. */
    static const char kVersion[16] = { 'S','X','1','2','6','1',' ','V','2','D',' ','2','D','0','2', 0 };
    for (int i = 0; i < 16; i++)
        d->regs[(uint16_t)(REG_VERSION_STRING + i)] = (uint8_t)kVersion[i];
}

VirtualSx126x* virtualChip(int slot)
{
    if (slot < 0 || slot >= (int)(sizeof(s_chips) / sizeof(s_chips[0]))) return nullptr;
    if (!s_chips[slot]) {
        s_chips[slot] = new VirtualSx126x(slot);
        if (slot + 1 > s_chipCount) s_chipCount = slot + 1;
    }
    return s_chips[slot];
}

int virtualChipCount(void) { return s_chipCount; }

/* ---- The DIO1 line ---- */

bool VirtualSx126x::dio1High() const
{
    return (d->irqStatus & d->dio1Mask) != 0;
}

/* Which pin this slot's DIO1 is, as the board wired it. */
int virtualHalDio1Pin(int slot);

/* Drive DIO1 to whatever the IRQ bits now say. Called with nothing held: the
 * shim runs the driver's handler from here, and a handler ends in a yield. */
static void applyDio1(int slot, bool high)
{
    int pin = virtualHalDio1Pin(slot);
    if (pin >= 0) gpio_shim_set_level(pin, high ? 1 : 0);
}

/* ---- Timed events ---- */

namespace {

struct TimerArg { VirtualSx126x* chip; int which; };

void armOnce(esp_timer_handle_t* h, esp_timer_cb_t cb, void* arg, int64_t delayUs)
{
    if (!*h) {
        esp_timer_create_args_t a = {};
        a.callback = cb;
        a.arg = arg;
        a.name = "sx126x";
        if (esp_timer_create(&a, h) != ESP_OK) return;
    }
    esp_timer_stop(*h);
    if (delayUs < 0) delayUs = 0;
    esp_timer_start_once(*h, (uint64_t)delayUs);
}

}  // namespace

/* ---- The command interpreter ---- */

void VirtualSx126x::reset()
{
    bool dio;
    portENTER_CRITICAL(&d->mux);
    d->mode = "STDBY_RC";
    d->modeBits = ST_STDBY_RC;
    d->irqStatus = 0;
    dio = false;
    portEXIT_CRITICAL(&d->mux);
    applyDio1(d->slot, dio);
}

static void setMode(VirtualSx126x::Impl* d, const char* mode, uint8_t bits)
{
    d->mode = mode;
    d->modeBits = bits;
}

/* TX_DONE lands at the end of the frame, and the chip falls back to whatever
 * SetRxTxFallbackMode named. */
static void txDoneCb(void* arg);
static void rxPreCb(void* arg);
static void rxHdrCb(void* arg);
static void rxEndCb(void* arg);

void VirtualSx126x::transfer(const uint8_t* out, size_t len, uint8_t* in)
{
    if (!out || !in || len == 0) return;

    const uint8_t op = out[0];
    bool     publishState = false;
    bool     publishTx = false;
    bool     dioAfter = false;
    EtherTxFrame frame = {};
    EtherState   snap = {};
    uint8_t  txPayload[256];

    portENTER_CRITICAL(&d->mux);

    /* Every byte of the reply is the status until the data starts. */
    memset(in, d->status(), len);

    switch (op) {
    case CMD_SET_SLEEP:
        setMode(d, "SLEEP", ST_STDBY_RC);
        publishState = true;
        break;

    case CMD_SET_STANDBY:
        if (len >= 2 && out[1] == 0x01) setMode(d, "STDBY_XOSC", ST_STDBY_XOSC);
        else                            setMode(d, "STDBY_RC", ST_STDBY_RC);
        publishState = true;
        break;

    case CMD_SET_FS:
        setMode(d, "FS", ST_FS);
        publishState = true;
        break;

    case CMD_SET_RX:
        setMode(d, "RX", ST_RX);
        publishState = true;
        break;

    case CMD_SET_TX: {
        setMode(d, "TX", ST_TX);
        double toa = loraToaSeconds(d->sf, (int)d->bwHz, d->cr + 4, d->preamble,
                                    d->payloadLen, d->hdrImplicit, d->crcOn);
        double tSym = (double)((uint32_t)1 << d->sf) / (double)d->bwHz;
        int64_t now = esp_timer_get_time();
        frame.state.slot = d->slot;
        fillState(*d, frame.state);
        frame.id   = ++d->txId;
        frame.t0   = now;
        frame.tPre = now + (int64_t)((d->preamble + 4.25) * tSym * 1e6);
        frame.tHdr = frame.tPre + (int64_t)(8.0 * tSym * 1e6);
        frame.tEnd = now + (int64_t)(toa * 1e6);
        frame.powerDbm = d->powerDbm;
        memcpy(txPayload, d->buf + d->txBase, d->payloadLen);
        frame.payload = txPayload;
        frame.len = d->payloadLen;
        publishTx = true;
        armOnce(&d->tTxDone, txDoneCb, this, frame.tEnd - now);
        break;
    }

    case CMD_SET_RF_FREQUENCY:
        if (len >= 5) {
            uint32_t frf = ((uint32_t)out[1] << 24) | ((uint32_t)out[2] << 16) |
                           ((uint32_t)out[3] << 8) | out[4];
            d->freqHz = (uint32_t)(((uint64_t)frf * 32000000ULL) >> 25);
            publishState = true;
        }
        break;

    case CMD_SET_MODULATION:
        if (len >= 4) {
            d->sf   = out[1];
            d->bwHz = bwFromCode(out[2]);
            d->cr   = out[3] + 4;          /* the register holds 4/(4+n) */
            if (d->cr < 5) d->cr = 5;
            if (d->cr > 8) d->cr = 8;
            publishState = true;
        }
        break;

    case CMD_SET_PACKET_PARAMS:
        if (len >= 7) {
            d->preamble    = ((int)out[1] << 8) | out[2];
            d->hdrImplicit = out[3] != 0;
            d->payloadLen  = out[4];
            d->crcOn       = out[5] != 0;
            publishState = true;
        }
        break;

    case CMD_SET_TX_PARAMS:
        if (len >= 2) d->powerDbm = (int8_t)out[1];
        break;

    case CMD_SET_BUFFER_BASE:
        if (len >= 3) { d->txBase = out[1]; d->rxBase = out[2]; }
        break;

    case CMD_WRITE_REGISTER:
        if (len >= 3) {
            uint16_t addr = ((uint16_t)out[1] << 8) | out[2];
            for (size_t i = 3; i < len; i++) d->regs[(uint16_t)(addr + (i - 3))] = out[i];
            publishState = true;   /* the sync word lives here */
        }
        break;

    case CMD_READ_REGISTER:
        if (len >= 4) {
            uint16_t addr = ((uint16_t)out[1] << 8) | out[2];
            for (size_t i = 4; i < len; i++) {
                auto it = d->regs.find((uint16_t)(addr + (i - 4)));
                in[i] = it == d->regs.end() ? 0x00 : it->second;
            }
        }
        break;

    case CMD_WRITE_BUFFER:
        if (len >= 2) {
            uint8_t off = out[1];
            for (size_t i = 2; i < len; i++) d->buf[(uint8_t)(off + (i - 2))] = out[i];
        }
        break;

    case CMD_READ_BUFFER:
        if (len >= 3) {
            uint8_t off = out[1];
            for (size_t i = 3; i < len; i++) in[i] = d->buf[(uint8_t)(off + (i - 3))];
        }
        break;

    case CMD_SET_DIO_IRQ_PARAMS:
        if (len >= 5) {
            d->irqMask  = (uint16_t)(((uint16_t)out[1] << 8) | out[2]);
            d->dio1Mask = (uint16_t)(((uint16_t)out[3] << 8) | out[4]);
        }
        break;

    case CMD_GET_IRQ_STATUS:
        if (len >= 4) {
            in[2] = (uint8_t)(d->irqStatus >> 8);
            in[3] = (uint8_t)(d->irqStatus & 0xFF);
        }
        break;

    case CMD_CLEAR_IRQ_STATUS:
        if (len >= 3) {
            uint16_t clear = (uint16_t)(((uint16_t)out[1] << 8) | out[2]);
            d->irqStatus &= (uint16_t)~clear;
        }
        break;

    case CMD_GET_RX_BUF_STATUS:
        if (len >= 4) { in[2] = d->rxLen; in[3] = d->rxPtr; }
        break;

    case CMD_GET_PACKET_STATUS:
        if (len >= 5) { in[2] = d->rssiPkt; in[3] = d->snrPkt; in[4] = d->sigRssiPkt; }
        break;

    case CMD_GET_RSSI_INST:
        if (len >= 3) {
            /* The chip's -x/2 encoding, and nothing at all outside RX. */
            int dbm = d->inFlightLevel ? d->inFlightLevel : VirtualSx126x::kNoiseFloorDbm;
            in[2] = strcmp(d->mode, "RX") == 0 ? (uint8_t)(-2 * dbm) : 0xFF;
        }
        break;

    case CMD_GET_PACKET_TYPE:
        if (len >= 3) in[2] = 0x01;      /* LoRa */
        break;

    case CMD_GET_STATUS:
        if (len >= 2) in[1] = d->status();
        break;

    case CMD_GET_DEVICE_ERRORS:
        if (len >= 4) { in[2] = 0; in[3] = 0; }
        break;

    case CMD_GET_STATS:
        for (size_t i = 2; i < len; i++) in[i] = 0;
        break;

    /* Accepted and without effect at this depth. */
    case CMD_SET_PACKET_TYPE:
    case CMD_SET_REGULATOR:
    case CMD_CALIBRATE:
    case CMD_CALIBRATE_IMAGE:
    case CMD_SET_PA_CONFIG:
    case CMD_SET_DIO2_RF_SWITCH:
    case CMD_SET_DIO3_TCXO:
    case CMD_STOP_TIMER_ON_PRE:
    case CMD_SET_CAD_PARAMS:
    case CMD_SET_CAD:
    case CMD_SET_LORA_SYMB_TO:
    case CMD_CLEAR_DEVICE_ERR:
    case CMD_RESET_STATS:
        break;

    case CMD_SET_RXTX_FALLBACK:
        if (len >= 2) {
            switch (out[1]) {
                case 0x40: d->fallbackMode = "FS";         d->fallbackBits = ST_FS; break;
                case 0x30: d->fallbackMode = "STDBY_XOSC"; d->fallbackBits = ST_STDBY_XOSC; break;
                default:   d->fallbackMode = "STDBY_RC";   d->fallbackBits = ST_STDBY_RC; break;
            }
        }
        break;

    default:
        memset(in, (uint8_t)(d->modeBits | ST_CMD_INVALID), len);
        break;
    }

    /* Leaving RX abandons whatever was arriving. */
    if (op == CMD_SET_STANDBY || op == CMD_SET_SLEEP || op == CMD_SET_FS || op == CMD_SET_TX) {
        d->inFlightLevel = 0;
        d->pendingValid = false;
        if (d->tPre) esp_timer_stop(d->tPre);
        if (d->tHdr) esp_timer_stop(d->tHdr);
        if (d->tEnd) esp_timer_stop(d->tEnd);
    }

    if (publishState) fillState(*d, snap);
    dioAfter = (d->irqStatus & d->dio1Mask) != 0;
    portEXIT_CRITICAL(&d->mux);

    if (publishTx)    etherPublishTx(frame);
    if (publishState) etherPublishState(snap);
    applyDio1(d->slot, dioAfter);
}

/* ---- Timer callbacks ---- */

static void raise(VirtualSx126x::Impl* d, uint16_t bits)
{
    bool dio;
    portENTER_CRITICAL(&d->mux);
    d->irqStatus |= bits;
    dio = (d->irqStatus & d->dio1Mask) != 0;
    portEXIT_CRITICAL(&d->mux);
    applyDio1(d->slot, dio);
}

static void txDoneCb(void* arg)
{
    auto* chip = (VirtualSx126x*)arg;
    auto* d = chip->d;
    EtherState s;
    portENTER_CRITICAL(&d->mux);
    setMode(d, d->fallbackMode, d->fallbackBits);
    fillState(*d, s);
    portEXIT_CRITICAL(&d->mux);
    etherPublishState(s);
    raise(d, IRQ_TX_DONE);
}

static void rxPreCb(void* arg)
{
    raise(((VirtualSx126x*)arg)->d, IRQ_PREAMBLE_DETECTED | IRQ_SYNC_WORD_VALID);
}

static void rxHdrCb(void* arg)
{
    raise(((VirtualSx126x*)arg)->d, IRQ_HEADER_VALID);
}

static void rxEndCb(void* arg)
{
    auto* chip = (VirtualSx126x*)arg;
    auto* d = chip->d;
    uint16_t bits = 0;
    portENTER_CRITICAL(&d->mux);
    if (d->pendingValid) {
        for (size_t i = 0; i < d->pendingLen; i++)
            d->buf[(uint8_t)(d->rxBase + i)] = d->pendingPayload[i];
        d->rxLen = (uint8_t)d->pendingLen;
        d->rxPtr = d->rxBase;
        d->rssiPkt    = (uint8_t)(-2 * d->pendingEnd.rssiDbm);
        d->sigRssiPkt = d->rssiPkt;
        d->snrPkt     = (uint8_t)(int8_t)(d->pendingEnd.snrDb * 4);
        bits = IRQ_RX_DONE;
        if (!d->pendingEnd.crcOk)   bits |= IRQ_CRC_ERR;
        if (!d->pendingEnd.headerOk) bits |= IRQ_HEADER_ERR;
        d->pendingValid = false;
    }
    d->inFlightLevel = 0;
    portEXIT_CRITICAL(&d->mux);
    if (bits) raise(d, bits);
}

/* ---- What the ether hands back ---- */

void VirtualSx126x::onRxBegin(const VirtualRxBegin& f)
{
    portENTER_CRITICAL(&d->mux);
    if (strcmp(d->mode, "RX") != 0) { portEXIT_CRITICAL(&d->mux); return; }
    d->inFlightLevel = f.levelDbm;
    portEXIT_CRITICAL(&d->mux);

    /* The sender's stamps are its own clock's; only the gaps between them mean
     * anything here, and they are measured from this instant. */
    armOnce(&d->tPre, rxPreCb, this, f.tPre - f.t0);
    armOnce(&d->tHdr, rxHdrCb, this, f.tHdr - f.t0);
}

void VirtualSx126x::onRxEnd(const VirtualRxEnd& f)
{
    portENTER_CRITICAL(&d->mux);
    if (strcmp(d->mode, "RX") != 0) { portEXIT_CRITICAL(&d->mux); return; }
    size_t n = f.len > sizeof(d->pendingPayload) ? sizeof(d->pendingPayload) : f.len;
    if (f.payload && n) memcpy(d->pendingPayload, f.payload, n);
    d->pendingLen = n;
    d->pendingEnd = f;
    d->pendingEnd.payload = nullptr;
    d->pendingValid = true;
    portEXIT_CRITICAL(&d->mux);

    rxEndCb(this);
}

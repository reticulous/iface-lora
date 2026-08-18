/**
 * rnode_door.h — the RNode endpoint's door contract.
 *
 * A stock Reticulum RNodeInterface client attaches to this device as if it
 * were RNode hardware and becomes the third endpoint of the radio segment. The
 * endpoint itself is private to iface-lora; what is public is the ITS port a
 * transport dials to hand it a client, and how a transport says which one it
 * is.
 *
 *   transport                        iface-lora
 *     |  itsConnect("lora", RNODE_ITS_PORT, <payload>, <len>) ---->
 *     |                                     accepted iff no session is
 *     |                                     attached and the endpoint is on
 *     |  <-------------------------------- raw KISS, both directions
 *     |  itsDisconnect                ---->  radio stays up for rnsd
 *
 * ONE SESSION, FIRST COME FIRST SERVED, ACROSS EVERY TRANSPORT. The port is
 * opened with maxHandles = 1, so a serial (or TCP) client attached first blocks
 * the Bluetooth door until it disconnects, and the other way round. There is
 * one RNode.
 *
 * THE CONNECT PAYLOAD'S LENGTH IS THE DISCRIMINATOR. The endpoint has no other
 * channel to learn which transport handed it a client on, so each transport's
 * payload must be a distinct size:
 *
 *   serial_handler_connect_t  (cli.h)          1 byte
 *   net_connect_t             (net.h)         28 bytes with CONFIG_LWIP_IPV6,
 *                                              8 bytes without it
 *   rnode_door_connect_t      (here)          12 bytes
 *
 * Twelve, not the eight a { magic, addr[6], type } struct would naturally be:
 * eight is exactly what net_connect_t collapses to in an IPv6-off build, and a
 * collision there would route a Bluetooth client through the TCP branch. The
 * magic is the belt to the length's braces. The static_asserts below are what
 * keeps the three sizes distinct as any of them changes.
 *
 * The KISS opcodes stay private to iface-lora: a transport moves bytes and
 * knows nothing about what they mean.
 */
#pragma once

#include <stdint.h>

/** ITS server port on the `lora` task. 'RN'. */
#define RNODE_ITS_PORT   0x524E

/** Connect payload for a Bluetooth client. */
typedef struct {
    uint8_t magic;        /* RNODE_DOOR_MAGIC */
    uint8_t peerAddr[6];  /* the central's address, NimBLE byte order */
    uint8_t addrType;     /* BLE_ADDR_PUBLIC / _RANDOM / … */
    uint8_t reserved[4];  /* pads to 12 — see the header comment */
} rnode_door_connect_t;

#define RNODE_DOOR_MAGIC 0xB7

#ifdef __cplusplus
static_assert(sizeof(rnode_door_connect_t) == 12,
              "the door's transport discriminator is this payload's size");
#endif

// Frame primitives for the link layer: the constants, the two block check
// characters, byte stuffing, and the builders that put a frame on the wire.
//
// Nothing here touches the serial port, a signal, or any state, so
// tests/selftest.c can exercise all of it without a cable. The statistics that
// used to live here moved to link_layer.c, next to the connection they count.

#ifndef FRAME_H
#define FRAME_H

#include <stdbool.h>
#include <stddef.h>

#include "link_layer.h" // MAX_PAYLOAD_SIZE (single source of truth)

// ---- Delimiting and transparency ----

#define FRAME_FLAG 0x7E          // frame delimiter
#define FRAME_ESCAPE 0x7D        // escape octet used by the stuffing mechanism
#define FRAME_ESCAPE_FLAG 0x5E   // 0x7D 0x5E stands for a stuffed 0x7E
#define FRAME_ESCAPE_ESCAPE 0x5D // 0x7D 0x5D stands for a stuffed 0x7D

// ---- Address field (A) ----

// A command carries the address of the side that sends it; a reply carries the
// address of the command it answers. So:
//
//   Tx -> Rx : I, SET, DISC .................... A = 0x03
//   Rx -> Tx : UA (answers SET), RR, REJ ....... A = 0x03
//   Rx -> Tx : DISC (answers the Tx DISC) ...... A = 0x01
//   Tx -> Rx : UA (answers the Rx DISC) ........ A = 0x01
//
// The closing handshake crosses over as a result: both ends use 0x01 for the
// last two frames. Do not derive A from "control == C_UA", or the receiver's UA
// gets 0x03 wrong and comes out as 0x01.
#define A_TX 0x03
#define A_RX 0x01

// ---- Control field (C) ----

#define C_SET 0x03  // connection request
#define C_DISC 0x0B // disconnection request
#define C_UA 0x07   // unnumbered acknowledgement

#define C_I0 0x00   // information frame, Ns = 0
#define C_I1 0x80   // information frame, Ns = 1

#define C_RR0 0xAA  // receive ready, Nr = 0 (next expected frame is I0)
#define C_RR1 0xAB  // receive ready, Nr = 1 (next expected frame is I1)

#define C_REJ0 0x54 // reject, Nr = 0
#define C_REJ1 0x55 // reject, Nr = 1

// Ns sits in bit 7 of an information frame, Nr in bit 0 of RR and REJ. The two
// encodings have nothing to do with each other, so keep the builders and the
// extractors on separate bit arithmetic.
#define C_I(ns) ((unsigned char)((ns) ? C_I1 : C_I0))
#define C_RR(ns) ((unsigned char)((ns) ? C_RR1 : C_RR0))
#define C_REJ(ns) ((unsigned char)((ns) ? C_REJ1 : C_REJ0))

// ---- Frame geometry ----

// Supervision frame: F A C BCC1 F
#define SUPERVISION_FRAME_SIZE 5
// F A C BCC1, common to every frame type
#define FRAME_HEADER_SIZE 4
// A frame with no data field still carries a BCC2
#define FRAME_BCC2_SIZE 1

// Worst case on the wire: header, every data byte escaped, an escaped BCC2,
// closing flag. frameBuildInfo refuses anything smaller, which is why its
// stuffing steps have no error path.
#define MAX_FRAME_SIZE (FRAME_HEADER_SIZE + 2 * MAX_PAYLOAD_SIZE + 2 * FRAME_BCC2_SIZE + 1)

// ---- Control field predicates and extractors ----

// Compares against the two legal values rather than testing a bit. RR0 (0xAA)
// has bit 0 clear, so a sloppy bit test reads it as a data frame.
bool frameCIsInfo(unsigned char control);

// True for an RR (or a REJ) whatever Nr it carries.
bool frameCIsRR(unsigned char control);
bool frameCIsREJ(unsigned char control);

// 0 or 1. Ns is bit 7 of an I frame; Nr is bit 0 of an RR or REJ.
unsigned char frameCGetNs(unsigned char control);
unsigned char frameCGetNr(unsigned char control);

// For the printf calls. Anything outside the table comes back as "unknown".
const char *frameControlName(unsigned char control);

// ---- Block check characters ----

// Even parity over the header: A XOR C.
unsigned char frameBcc1(unsigned char address, unsigned char control);

// Even parity over the data field. An empty field gives 0. A NULL pointer is
// only legal together with dataSize == 0.
unsigned char frameBcc2(const unsigned char *data, size_t dataSize);

// ---- Byte stuffing ----

// Copy inSize bytes into out, replacing 0x7E with 0x7D 0x5E and 0x7D with
// 0x7D 0x5D.
//
// Returns the number written, or -1 if the result does not fit in outCapacity
// (nothing is written in that case) or the arguments don't make sense.
int frameStuff(const unsigned char *in, size_t inSize, unsigned char *out, size_t outCapacity);

// ---- Frame construction ----

// Writes F A C BCC1 F. Returns SUPERVISION_FRAME_SIZE, or -1 if outCapacity is
// too small.
int frameBuildSupervision(unsigned char *out, size_t outCapacity,
                          unsigned char address, unsigned char control);

// Writes F A C BCC1 <stuffed packet || stuffed BCC2> F. Ns picks 0x00 or 0x80.
// packet may be NULL only with packetLen == 0.
//
// out has to hold MAX_FRAME_SIZE whatever the packet length. The bound is
// checked once up front so the frame goes out whole or not at all.
// Returns the wire length, or -1 on error.
int frameBuildInfo(unsigned char *out, size_t outCapacity,
                   unsigned char address, unsigned char ns,
                   const unsigned char *packet, size_t packetLen);

#endif // FRAME_H
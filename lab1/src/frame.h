// Serial data link protocol: frame primitives.
//
// Pure helpers shared by the link layer: protocol constants, the two block
// check characters, byte stuffing and the frame builders. Nothing in this
// module touches the serial port, waits on a signal or keeps state, so every
// function here can be exercised in isolation (see tests/selftest.c).
// Connection statistics live in link_layer.c, where the connection does.

#ifndef FRAME_H
#define FRAME_H

#include <stdbool.h>
#include <stddef.h>

#include "link_layer.h" // MAX_PAYLOAD_SIZE (single source of truth)

////////////////////////////////////////////////
// Delimiting and transparency
////////////////////////////////////////////////

#define FRAME_FLAG 0x7E          // frame delimiter
#define FRAME_ESCAPE 0x7D        // escape octet used by the stuffing mechanism
#define FRAME_ESCAPE_FLAG 0x5E   // 0x7D 0x5E stands for a stuffed 0x7E
#define FRAME_ESCAPE_ESCAPE 0x5D // 0x7D 0x5D stands for a stuffed 0x7D

////////////////////////////////////////////////
// Address field (A)
////////////////////////////////////////////////

// A command carries the address of the side that sends it; a reply carries the
// address of the command it answers. Concretely:
//
//   Tx -> Rx : I, SET, DISC .................... A = 0x03
//   Rx -> Tx : UA (answers SET), RR, REJ ....... A = 0x03
//   Rx -> Tx : DISC (answers the Tx DISC) ...... A = 0x01
//   Tx -> Rx : UA (answers the Rx DISC) ........ A = 0x01
//
// Note that the closing handshake therefore crosses over: both ends use 0x01
// for the last two frames. Do not derive A from "control == C_UA".
#define A_TX 0x03
#define A_RX 0x01

////////////////////////////////////////////////
// Control field (C)
////////////////////////////////////////////////

#define C_SET 0x03  // connection request
#define C_DISC 0x0B // disconnection request
#define C_UA 0x07   // unnumbered acknowledgement

#define C_I0 0x00   // information frame, Ns = 0
#define C_I1 0x80   // information frame, Ns = 1

#define C_RR0 0xAA  // receive ready, Nr = 0 (next expected frame is I0)
#define C_RR1 0xAB  // receive ready, Nr = 1 (next expected frame is I1)

#define C_REJ0 0x54 // reject, Nr = 0
#define C_REJ1 0x55 // reject, Nr = 1

// Ns lives in bit 7 of an information frame; Nr lives in bit 0 of RR and REJ.
// The two encodings are unrelated, so the builders and the extractors below
// must never share bit arithmetic.
#define C_I(ns) ((unsigned char)((ns) ? C_I1 : C_I0))
#define C_RR(ns) ((unsigned char)((ns) ? C_RR1 : C_RR0))
#define C_REJ(ns) ((unsigned char)((ns) ? C_REJ1 : C_REJ0))

////////////////////////////////////////////////
// Frame geometry
////////////////////////////////////////////////

// Supervision frame: F A C BCC1 F
#define SUPERVISION_FRAME_SIZE 5
// F A C BCC1, the part shared by every frame type
#define FRAME_HEADER_SIZE 4
// A frame with no data field still carries a BCC2
#define FRAME_BCC2_SIZE 1

// Worst case on the wire: the header, every data byte escaped, an escaped BCC2
// and the closing flag. frameBuildInfo refuses any buffer smaller than this,
// which is what makes its stuffing step infallible.
#define MAX_FRAME_SIZE (FRAME_HEADER_SIZE + 2 * MAX_PAYLOAD_SIZE + 2 * FRAME_BCC2_SIZE + 1)

////////////////////////////////////////////////
// Control field predicates and extractors
////////////////////////////////////////////////

// True for an information frame. Equality against the two legal control
// values: bit tests are wrong here, because RR0 (0xAA) has bit 0 clear and
// would otherwise be mistaken for a data frame.
bool frameCIsInfo(unsigned char control);

// True for an RR reply, whatever the value of Nr.
bool frameCIsRR(unsigned char control);

// True for a REJ reply, whatever the value of Nr.
bool frameCIsREJ(unsigned char control);

// Value of Ns carried by an information frame control field (bit 7): 0 or 1.
unsigned char frameCGetNs(unsigned char control);

// Value of Nr carried by an RR or REJ control field (bit 0, not bit 7): 0 or 1.
unsigned char frameCGetNr(unsigned char control);

// Printable name of a control field, for diagnostics only.
const char *frameControlName(unsigned char control);

////////////////////////////////////////////////
// Block check characters
////////////////////////////////////////////////

// BCC1: covers the header, i.e. the address and control fields.
unsigned char frameBcc1(unsigned char address, unsigned char control);

// BCC2: covers the data field. An empty data field yields 0.
// A NULL pointer is accepted only together with dataSize == 0.
unsigned char frameBcc2(const unsigned char *data, size_t dataSize);

////////////////////////////////////////////////
// Byte stuffing
////////////////////////////////////////////////

// Copy inSize bytes from "in" to "out", replacing 0x7E with 0x7D 0x5E and
// 0x7D with 0x7D 0x5D.
//
// Returns the number of bytes written to "out", or -1 if the result does not
// fit in outCapacity (nothing is written at all in that case), or if the
// arguments are inconsistent.
int frameStuff(const unsigned char *in, size_t inSize, unsigned char *out, size_t outCapacity);

////////////////////////////////////////////////
// Frame construction
////////////////////////////////////////////////

// Build the supervision frame F A C BCC1 F into "out".
// Returns SUPERVISION_FRAME_SIZE, or -1 if outCapacity is too small.
int frameBuildSupervision(unsigned char *out, size_t outCapacity,
                          unsigned char address, unsigned char control);

// Build the information frame F A C BCC1 <stuffed packet || stuffed BCC2> F.
// Ns selects 0x00 or 0x80. packet may be NULL only with packetLen == 0.
//
// "out" must be able to hold the worst case, MAX_FRAME_SIZE, whatever the
// actual packet length: the bound is checked once up front so that the frame
// is either written whole or not at all.
//
// Returns the number of wire bytes written, or -1 on error / small buffer.
int frameBuildInfo(unsigned char *out, size_t outCapacity,
                   unsigned char address, unsigned char ns,
                   const unsigned char *packet, size_t packetLen);

#endif // FRAME_H
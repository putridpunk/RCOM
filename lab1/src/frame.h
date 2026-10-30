// Serial data link protocol: frame primitives.
//
// Pure: no port, no signals, no state, no output. Connection statistics live in
// link_layer.c, where the connection does.

#ifndef FRAME_H
#define FRAME_H

#include <stdbool.h>
#include <stddef.h>

#include "link_layer.h" // MAX_PAYLOAD_SIZE

// Delimiting and transparency
#define FRAME_FLAG 0x7E
#define FRAME_ESCAPE 0x7D
#define FRAME_ESCAPE_FLAG 0x5E   // 0x7D 0x5E is a stuffed 0x7E
#define FRAME_ESCAPE_ESCAPE 0x5D // 0x7D 0x5D is a stuffed 0x7D

// Address field.
//
// A command carries the address of the side that sends it; a reply carries the
// address of the command it answers:
//
//   Tx -> Rx : I, SET, DISC .................... A = 0x03
//   Rx -> Tx : UA (answers SET), RR, REJ ....... A = 0x03
//   Rx -> Tx : DISC (answers the Tx DISC) ...... A = 0x01
//   Tx -> Rx : UA (answers the Rx DISC) ........ A = 0x01
//
// So the closing handshake crosses over: both ends use 0x01 for its last two
// frames. Do not derive A from "control == C_UA".
#define A_TX 0x03
#define A_RX 0x01

#define C_SET 0x03
#define C_DISC 0x0B
#define C_UA 0x07

#define C_I0 0x00
#define C_I1 0x80

#define C_RR0 0xAA  // Nr = 0
#define C_RR1 0xAB  // Nr = 1
#define C_REJ0 0x54 // Nr = 0
#define C_REJ1 0x55 // Nr = 1

// Ns is bit 7 of an information frame; Nr is bit 0 of RR and REJ. The two
// encodings are unrelated, so never share bit arithmetic between them.
#define C_I(ns) ((unsigned char)((ns) ? C_I1 : C_I0))
#define C_RR(ns) ((unsigned char)((ns) ? C_RR1 : C_RR0))
#define C_REJ(ns) ((unsigned char)((ns) ? C_REJ1 : C_REJ0))

// Frame geometry.
// supervision: F A C BCC1 F
#define SUPERVISION_FRAME_SIZE 5
// F A C BCC1, shared by every frame type
#define FRAME_HEADER_SIZE 4
#define FRAME_BCC2_SIZE 1

// Worst case on the wire: header, every data byte escaped, escaped BCC2, flag.
// frameBuildInfo refuses any smaller buffer, which is what makes its stuffing
// step infallible.
#define MAX_FRAME_SIZE (FRAME_HEADER_SIZE + 2 * MAX_PAYLOAD_SIZE + 2 * FRAME_BCC2_SIZE + 1)

// True for an information frame. Compare against the two legal values: bit
// tests are wrong here, because RR0 (0xAA) has bit 0 clear and would be taken
// for a data frame.
bool frameCIsInfo(unsigned char control);

// True for an RR / REJ reply, whatever the value of Nr.
bool frameCIsRR(unsigned char control);
bool frameCIsREJ(unsigned char control);

// Value (0 or 1), not a predicate: Ns of an I frame is bit 7, Nr of an RR or
// REJ is bit 0.
unsigned char frameCGetNs(unsigned char control);
unsigned char frameCGetNr(unsigned char control);

// Printable name of a control field, for diagnostics only.
const char *frameControlName(unsigned char control);

// Both checksums are always computed on the un-stuffed octets: before stuffing
// when transmitting, after de-stuffing when receiving.
unsigned char frameBcc1(unsigned char address, unsigned char control);

// NULL only valid with dataSize == 0.
unsigned char frameBcc2(const unsigned char *data, size_t dataSize);

// Bytes written, or -1 if the result does not fit -- nothing is written then.
int frameStuff(const unsigned char *in, size_t inSize, unsigned char *out, size_t outCapacity);

// SUPERVISION_FRAME_SIZE, or -1 if outCapacity is too small.
int frameBuildSupervision(unsigned char *out, size_t outCapacity,
                          unsigned char address, unsigned char control);

// outCapacity must be >= MAX_FRAME_SIZE: the worst case is checked once, so the
// frame is written whole or not at all.
int frameBuildInfo(unsigned char *out, size_t outCapacity,
                   unsigned char address, unsigned char ns,
                   const unsigned char *packet, size_t packetLen);

#endif // FRAME_H
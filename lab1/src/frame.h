// Serial data link protocol: frame primitives.
//
// Pure helpers shared by the link layer: protocol constants, the two block
// check characters, byte stuffing and the frame builders, plus the
// transmission statistics records printed when a connection is closed.
//
// Nothing in this module touches the serial port, waits on a signal or keeps
// global state, so every function here can be exercised in isolation.

#ifndef FRAME_H
#define FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <time.h>

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

// Worst case on the wire, with every data byte and the BCC2 escaped.
#define FRAME_MAX_WIRE_SIZE (FRAME_HEADER_SIZE + 2 * MAX_PAYLOAD_SIZE + 2 * FRAME_BCC2_SIZE + 1)

// Buffer size that always holds a complete frame, whatever its contents.
#define MAX_FRAME_SIZE (2 * MAX_PAYLOAD_SIZE + 8)

#if MAX_FRAME_SIZE < FRAME_MAX_WIRE_SIZE
#error "MAX_FRAME_SIZE is too small to hold a fully escaped frame"
#endif

////////////////////////////////////////////////
// Control field predicates
////////////////////////////////////////////////

// True for an information frame. Equality against the two legal control
// values: bit tests are wrong here, because RR0 (0xAA) has bit 0 clear and
// would otherwise be mistaken for a data frame.
bool frameCIsInfo(unsigned char control);

// True for an RR reply, whatever the value of Nr.
bool frameCIsRR(unsigned char control);

// True for a REJ reply, whatever the value of Nr.
bool frameCIsREJ(unsigned char control);

// Ns carried by an information frame control field (bit 7).
bool frameCGetNs(unsigned char control);

// Nr carried by a RR or REJ control field (bit 0, not bit 7).
bool frameCGetNr(unsigned char control);

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
// Returns the number of wire bytes written, or -1 on error / small buffer.
int frameBuildInfo(unsigned char *out, size_t outCapacity,
                   unsigned char address, unsigned char ns,
                   const unsigned char *packet, size_t packetLen);

////////////////////////////////////////////////
// Transmission statistics
////////////////////////////////////////////////

// Counters gathered while a connection is open, printed on close.
typedef struct
{
    // Supervision frames, by type
    unsigned long setSent;
    unsigned long setReceived;
    unsigned long uaSent;
    unsigned long uaReceived;
    unsigned long discSent;
    unsigned long discReceived;
    unsigned long rrSent;
    unsigned long rrReceived;
    unsigned long rejSent;
    unsigned long rejReceived;

    // Totals, including supervision frames of an unrecognised control field
    unsigned long supervisionSent;
    unsigned long supervisionReceived;

    // Information frames transmitted / delivered to the application
    unsigned long infoSent;
    unsigned long infoReceived;

    // Error and recovery events
    unsigned long retransmissions;
    unsigned long timeouts;
    unsigned long bcc1Errors;
    unsigned long bcc2Errors;
    unsigned long duplicateFrames;
    unsigned long addressErrors;

    // Useful payload carried by information frames. Counted once per packet
    // queued for transmission, not once per retry, so payloadBytesSent equals
    // the number of file bytes handed to the link layer.
    unsigned long payloadBytesSent;
    unsigned long payloadBytesReceived;

    // Set by linkStatsReset; used to report the elapsed time
    struct timespec startTime;
    bool started;
} LinkStats;

// Zero every counter and start the clock. Safe to call with NULL.
void linkStatsReset(LinkStats *stats);

// Record one supervision frame of the given control field. Safe with NULL.
void linkStatsCountSupervisionSent(LinkStats *stats, unsigned char control);
void linkStatsCountSupervisionReceived(LinkStats *stats, unsigned char control);

// Seconds elapsed since linkStatsReset, or 0 if the clock never started.
double linkStatsElapsedSeconds(const LinkStats *stats);

// Human readable summary on stdout. Safe with NULL.
void linkStatsPrint(const LinkStats *stats);

#endif // FRAME_H
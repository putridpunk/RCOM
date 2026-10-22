// Serial data link protocol: frame primitives.
//
// Pure helpers shared by the link layer: protocol constants, the two block
// check characters, byte stuffing and the supervision frame builder, plus the
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

// Frames sent by the transmitter, and replies sent by the receiver.
#define A_TX 0x03
// Frames sent by the receiver, and replies sent by the transmitter.
#define A_RX 0x01

////////////////////////////////////////////////
// Control field (C)
////////////////////////////////////////////////

#define C_SET 0x03  // connection request
#define C_DISC 0x0B // disconnection request
#define C_UA 0x07   // unnumbered acknowledgement
#define C_RR(ns) ((unsigned char)(0x05 | ((ns) ? 0x80 : 0x00)))  // receive ready
#define C_REJ(ns) ((unsigned char)(0x01 | ((ns) ? 0x80 : 0x00))) // reject
#define C_I(ns) ((unsigned char)((ns) ? 0x40 : 0x00))            // information

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

// True for an information frame (Ns in bit 6). Supervision and reply frames
// all have bit 0 set, so bit 0 alone separates the two classes.
bool frameCIsInfo(unsigned char control);

// True for a RR reply, whatever the value of Nr.
bool frameCIsRR(unsigned char control);

// True for a REJ reply, whatever the value of Nr.
bool frameCIsREJ(unsigned char control);

// Ns carried by an information frame control field.
bool frameCGetNs(unsigned char control);

// Nr carried by a RR or REJ control field.
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
// fit in outCapacity (nothing is ever written past the capacity), or if the
// arguments are inconsistent.
int frameStuff(const unsigned char *in, size_t inSize, unsigned char *out, size_t outCapacity);

////////////////////////////////////////////////
// Frame construction
////////////////////////////////////////////////

// Build the supervision frame F A C BCC1 F into "out".
// Returns SUPERVISION_FRAME_SIZE, or -1 if outCapacity is too small.
int frameBuildSupervision(unsigned char *out, size_t outCapacity,
                          unsigned char address, unsigned char control);

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

    // Information frames
    unsigned long infoSent;
    unsigned long infoReceived;

    // Error and recovery events
    unsigned long retransmissions;
    unsigned long timeouts;
    unsigned long bcc1Errors;
    unsigned long bcc2Errors;
    unsigned long duplicateFrames;

    // Useful payload actually carried by information frames
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
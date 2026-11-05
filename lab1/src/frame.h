#ifndef FRAME_H
#define FRAME_H

#include <stdbool.h>
#include <stddef.h>

#include "link_layer.h" // MAX_PAYLOAD_SIZE

#define FRAME_FLAG 0x7E
#define FRAME_ESCAPE 0x7D
#define FRAME_ESCAPE_FLAG 0x5E   // 7D 5E -> a stuffed 7E
#define FRAME_ESCAPE_ESCAPE 0x5D // 7D 5D -> a stuffed 7D

// A field. a command carries the sender's address, a reply carries the address
// of the command its answering. so tx sends 0x03 for everything except its UA,
// and rx sends 0x03 for everything except its DISC.
#define A_TX 0x03
#define A_RX 0x01

#define C_SET 0x03
#define C_DISC 0x0B
#define C_UA 0x07

#define C_I0 0x00 // I frame, Ns=0
#define C_I1 0x80 // I frame, Ns=1

#define C_RR0 0xAA // rr, nr=0
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55

// Ns is bit 7 of an I frame but Nr is bit 0 of an RR/REJ.
#define C_I(ns) ((unsigned char)((ns) ? C_I1 : C_I0))
#define C_RR(ns) ((unsigned char)((ns) ? C_RR1 : C_RR0))
#define C_REJ(ns) ((unsigned char)((ns) ? C_REJ1 : C_REJ0))

#define SUPERVISION_FRAME_SIZE 5 // F A C BCC1 F
#define FRAME_HEADER_SIZE 4      // F A C BCC1
#define FRAME_BCC2_SIZE 1        // even an empty data field carries one

// worst case: header, every payload byte escaped, an escaped BCC2, closing flag
#define MAX_FRAME_SIZE (FRAME_HEADER_SIZE + 2 * MAX_PAYLOAD_SIZE + 2 * FRAME_BCC2_SIZE + 1)

// compares against the two legal values instead of testing bit 7. RR0 is 0xAA
// and its bit 0 is clear, so a bit test reads it as a data frame
bool frameCIsInfo(unsigned char control);

bool frameCIsRR(unsigned char control);
bool frameCIsREJ(unsigned char control);

// 0 or 1
unsigned char frameCGetNs(unsigned char control);
unsigned char frameCGetNr(unsigned char control);

// for the printfs. anything not in the table comes back "unknown"
const char *frameControlName(unsigned char control);

// even parity over the header: A ^ C
unsigned char frameBcc1(unsigned char address, unsigned char control);

// even parity over the data. empty field gives 0. NULL is only ok with size 0
unsigned char frameBcc2(const unsigned char *data, size_t dataSize);

// 7E -> 7D 5E and 7D -> 7D 5D. returns bytes written, or -1 if it doesn't fit,
// in which case out stays untouched
int frameStuff(const unsigned char *in, size_t inSize, unsigned char *out, size_t outCapacity);

// writes F A C BCC1 F. returns 5 or -1
int frameBuildSupervision(unsigned char *out, size_t outCapacity,
                          unsigned char address, unsigned char control);

// writes F A C BCC1 <stuffed packet, stuffed BCC2> F. needs MAX_FRAME_SIZE of
// room whatever the packet length, so it either writes the whole frame or
// nothing. returns the wire length or -1
int frameBuildInfo(unsigned char *out, size_t outCapacity,
                   unsigned char address, unsigned char ns,
                   const unsigned char *packet, size_t packetLen);

#endif // FRAME_H
// Serial data link protocol: frame primitives (implementation).
//
// Everything here is pure: no serial port, no signals, no mutable state.
// The only side effect is the printf done by linkStatsPrint.

#include "frame.h"

#include <stdio.h>
#include <string.h>

////////////////////////////////////////////////
// Internal helpers
////////////////////////////////////////////////

static int isSpecial(unsigned char byte)
{
    return byte == FRAME_FLAG || byte == FRAME_ESCAPE;
}

// Number of bytes "in" occupies once stuffed.
static size_t stuffedSize(const unsigned char *in, size_t inSize)
{
    size_t total = 0;

    for (size_t i = 0; i < inSize; i++)
        total += isSpecial(in[i]) ? 2u : 1u;

    return total;
}

////////////////////////////////////////////////
// Control field predicates
////////////////////////////////////////////////

bool frameCIsInfo(unsigned char control)
{
    return control == C_I0 || control == C_I1;
}

bool frameCIsRR(unsigned char control)
{
    return control == C_RR0 || control == C_RR1;
}

bool frameCIsREJ(unsigned char control)
{
    return control == C_REJ0 || control == C_REJ1;
}

bool frameCGetNs(unsigned char control)
{
    return (control & 0x80) != 0;
}

bool frameCGetNr(unsigned char control)
{
    // 0xAA/0xAB and 0x54/0x55 differ in bit 0 only. Bit 7 is always set for
    // RR and clear for REJ, so using it here would return a constant.
    return (control & 0x01) != 0;
}

////////////////////////////////////////////////
// Block check characters
////////////////////////////////////////////////

unsigned char frameBcc1(unsigned char address, unsigned char control)
{
    return (unsigned char)(address ^ control);
}

unsigned char frameBcc2(const unsigned char *data, size_t dataSize)
{
    unsigned char bcc = 0;

    if (data == NULL)
        return 0;

    for (size_t i = 0; i < dataSize; i++)
        bcc ^= data[i];

    return bcc;
}

////////////////////////////////////////////////
// Byte stuffing
////////////////////////////////////////////////

int frameStuff(const unsigned char *in, size_t inSize, unsigned char *out, size_t outCapacity)
{
    size_t written = 0;

    if (out == NULL)
        return -1;

    if (in == NULL && inSize > 0)
        return -1;

    if (inSize == 0)
        return 0;

    // Capacity is checked before a single byte is written, so a failure never
    // leaves a half-stuffed buffer behind.
    if (stuffedSize(in, inSize) > outCapacity)
        return -1;

    for (size_t i = 0; i < inSize; i++)
    {
        switch (in[i])
        {
        case FRAME_FLAG:
            out[written++] = FRAME_ESCAPE;
            out[written++] = FRAME_ESCAPE_FLAG;
            break;

        case FRAME_ESCAPE:
            out[written++] = FRAME_ESCAPE;
            out[written++] = FRAME_ESCAPE_ESCAPE;
            break;

        default:
            out[written++] = in[i];
            break;
        }
    }

    return (int)written;
}

////////////////////////////////////////////////
// Frame construction
////////////////////////////////////////////////

int frameBuildSupervision(unsigned char *out, size_t outCapacity,
                          unsigned char address, unsigned char control)
{
    if (out == NULL || outCapacity < SUPERVISION_FRAME_SIZE)
        return -1;

    // For every control value in use the BCC1 result is never 0x7E or 0x7D,
    // so the header of a supervision frame never needs stuffing.
    out[0] = FRAME_FLAG;
    out[1] = address;
    out[2] = control;
    out[3] = frameBcc1(address, control);
    out[4] = FRAME_FLAG;

    return SUPERVISION_FRAME_SIZE;
}

int frameBuildInfo(unsigned char *out, size_t outCapacity,
                   unsigned char address, unsigned char ns,
                   const unsigned char *packet, size_t packetLen)
{
    unsigned char control;
    unsigned char bcc1;
    unsigned char bcc2;
    size_t needed;
    size_t written;
    int stuffed;

    if (out == NULL)
        return -1;

    if (packet == NULL && packetLen > 0)
        return -1;

    if (packetLen > MAX_PAYLOAD_SIZE)
        return -1;

    control = C_I(ns);
    bcc1 = frameBcc1(address, control);

    // Both checksums are computed on the un-stuffed bytes.
    bcc2 = frameBcc2(packet, packetLen);

    needed = FRAME_HEADER_SIZE + stuffedSize(packet, packetLen) +
             stuffedSize(&bcc2, 1) + 1;

    if (needed > outCapacity)
        return -1;

    written = 0;
    out[written++] = FRAME_FLAG;
    out[written++] = address;
    out[written++] = control;
    out[written++] = bcc1;

    stuffed = frameStuff(packet, packetLen, out + written, outCapacity - written);
    if (stuffed < 0)
        return -1;
    written += (size_t)stuffed;

    stuffed = frameStuff(&bcc2, 1, out + written, outCapacity - written);
    if (stuffed < 0)
        return -1;
    written += (size_t)stuffed;

    out[written++] = FRAME_FLAG;

    return (int)written;
}

////////////////////////////////////////////////
// Transmission statistics
////////////////////////////////////////////////

void linkStatsReset(LinkStats *stats)
{
    if (stats == NULL)
        return;

    memset(stats, 0, sizeof(*stats));

    if (clock_gettime(CLOCK_MONOTONIC, &stats->startTime) == 0)
        stats->started = true;
    else
        stats->started = false;
}

void linkStatsCountSupervisionSent(LinkStats *stats, unsigned char control)
{
    if (stats == NULL)
        return;

    stats->supervisionSent++;

    switch (control)
    {
    case C_SET:
        stats->setSent++;
        break;
    case C_UA:
        stats->uaSent++;
        break;
    case C_DISC:
        stats->discSent++;
        break;
    case C_RR0:
    case C_RR1:
        stats->rrSent++;
        break;
    case C_REJ0:
    case C_REJ1:
        stats->rejSent++;
        break;
    default:
        break;
    }
}

void linkStatsCountSupervisionReceived(LinkStats *stats, unsigned char control)
{
    if (stats == NULL)
        return;

    stats->supervisionReceived++;

    switch (control)
    {
    case C_SET:
        stats->setReceived++;
        break;
    case C_UA:
        stats->uaReceived++;
        break;
    case C_DISC:
        stats->discReceived++;
        break;
    case C_RR0:
    case C_RR1:
        stats->rrReceived++;
        break;
    case C_REJ0:
    case C_REJ1:
        stats->rejReceived++;
        break;
    default:
        break;
    }
}

double linkStatsElapsedSeconds(const LinkStats *stats)
{
    struct timespec now;

    if (stats == NULL || stats->started == false)
        return 0.0;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
        return 0.0;

    return (double)(now.tv_sec - stats->startTime.tv_sec) +
           (double)(now.tv_nsec - stats->startTime.tv_nsec) / 1.0e9;
}

void linkStatsPrint(const LinkStats *stats)
{
    double elapsed;
    double totalPayload;

    if (stats == NULL)
        return;

    elapsed = linkStatsElapsedSeconds(stats);
    totalPayload = (double)stats->payloadBytesSent + (double)stats->payloadBytesReceived;

    printf("\n--- Link layer statistics ---\n");
    printf("  elapsed ................ %8.3f s\n", elapsed);

    printf("  SET   sent/received .... %8lu / %lu\n", stats->setSent, stats->setReceived);
    printf("  UA    sent/received .... %8lu / %lu\n", stats->uaSent, stats->uaReceived);
    printf("  DISC  sent/received .... %8lu / %lu\n", stats->discSent, stats->discReceived);
    printf("  RR    sent/received .... %8lu / %lu\n", stats->rrSent, stats->rrReceived);
    printf("  REJ   sent/received .... %8lu / %lu\n", stats->rejSent, stats->rejReceived);

    printf("  I     sent/received .... %8lu / %lu\n", stats->infoSent, stats->infoReceived);
    printf("  payload sent/received .. %8lu / %lu bytes\n",
           stats->payloadBytesSent, stats->payloadBytesReceived);

    printf("  retransmissions ........ %8lu\n", stats->retransmissions);
    printf("  timeouts ............... %8lu\n", stats->timeouts);
    printf("  BCC1 errors ............ %8lu\n", stats->bcc1Errors);
    printf("  BCC2 errors ............ %8lu\n", stats->bcc2Errors);
    printf("  duplicate frames ....... %8lu\n", stats->duplicateFrames);
    printf("  wrong address/class .... %8lu\n", stats->addressErrors);

    if (elapsed > 0.0)
        printf("  throughput ............. %8.1f B/s\n", totalPayload / elapsed);

    printf("-----------------------------\n");
}
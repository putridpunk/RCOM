#include "frame.h"

#include <stddef.h>

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

unsigned char frameCGetNs(unsigned char control)
{
    return (unsigned char)((control & 0x80) ? 1u : 0u);
}

unsigned char frameCGetNr(unsigned char control)
{
    return (unsigned char)(control & 0x01);
}

const char *frameControlName(unsigned char control)
{
    switch (control)
    {
    case C_SET:
        return "SET";
    case C_DISC:
        return "DISC";
    case C_UA:
        return "UA";
    case C_I0:
        return "I0";
    case C_I1:
        return "I1";
    case C_RR0:
        return "RR0";
    case C_RR1:
        return "RR1";
    case C_REJ0:
        return "REJ0";
    case C_REJ1:
        return "REJ1";
    default:
        return "unknown";
    }
}

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

int frameStuff(const unsigned char *in, size_t inSize, unsigned char *out, size_t outCapacity)
{
    size_t written = 0;

    if (out == NULL)
        return -1;

    if (in == NULL && inSize > 0)
        return -1;

    if (inSize == 0)
        return 0;

    // work out the size first, otherwise we could half fill out and then have
    // to bail
    size_t needed = 0;

    for (size_t i = 0; i < inSize; i++)
        needed += (in[i] == FRAME_FLAG || in[i] == FRAME_ESCAPE) ? 2u : 1u;

    if (needed > outCapacity)
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

int frameBuildSupervision(unsigned char *out, size_t outCapacity,
                          unsigned char address, unsigned char control)
{
    if (out == NULL || outCapacity < SUPERVISION_FRAME_SIZE)
        return -1;

    // nothing to stuff here, none of our A/C pairs give a BCC1 of 7E or 7D
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
    if (out == NULL)
        return -1;

    if (packet == NULL && packetLen > 0)
        return -1;

    if (packetLen > MAX_PAYLOAD_SIZE)
        return -1;

    // one capacity check up front. after this the two frameStuff calls below
    // can't fail, so we don't need to handle them
    if (outCapacity < MAX_FRAME_SIZE)
        return -1;

    unsigned char control = C_I(ns);
    unsigned char bcc2 = frameBcc2(packet, packetLen);
    size_t written = 0;

    out[written++] = FRAME_FLAG;
    out[written++] = address;
    out[written++] = control;
    out[written++] = frameBcc1(address, control);

    // both checksums go on the wire stuffed, but they were computed on the
    // bytes before stuffing
    written += (size_t)frameStuff(packet, packetLen, out + written, outCapacity - written);
    written += (size_t)frameStuff(&bcc2, 1, out + written, outCapacity - written);

    out[written++] = FRAME_FLAG;

    return (int)written;
}
// Application layer protocol implementation.
//
// START: C=1 | TLVs   DATA: C=2 | L2 | L1 | K bytes   END: C=3 | TLVs
// TLV 0 = size (4B big-endian), TLV 1 = name (0..255 raw); unknown types skipped.
//
// An RR means the link took the frame, not that the receiver wrote it to disk.

#include "application_layer.h"
#include "link_layer.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define APP_C_START 1
#define APP_C_DATA 2
#define APP_C_END 3

#define TLV_TYPE_SIZE 0
#define TLV_TYPE_NAME 1

#define SIZE_VALUE_LEN 4
#define MAX_NAME_LEN 255
#define MAX_METADATA_SIZE (2 + SIZE_VALUE_LEN + 2 + MAX_NAME_LEN)

// the size TLV is four octets
#define MAX_FILE_SIZE 0xFFFFFFFFUL

// Override with -DAPP_DATA_CHUNK=... to measure efficiency with smaller frames;
// the receiver reads K from L2/L1, so both ends stay compatible.
#ifndef APP_DATA_CHUNK
#define APP_DATA_CHUNK (MAX_PAYLOAD_SIZE - 3)
#endif

_Static_assert(APP_DATA_CHUNK >= 1 && APP_DATA_CHUNK + 3 <= MAX_PAYLOAD_SIZE,
               "APP_DATA_CHUNK must leave room for the packet header");
_Static_assert(MAX_METADATA_SIZE + 3 <= MAX_PAYLOAD_SIZE,
               "START/END metadata must fit in one packet");

// Must outlast the transmitter's own budget for one frame (33 s worst case),
// otherwise both ends abort at the same instant. 12 * 4 s.
#define RX_SILENT_ROUNDS 12

static size_t buildPacket(unsigned char *out, size_t capacity, unsigned char type,
                          const unsigned char *payload, size_t payloadLen)
{
    if (out == NULL || payloadLen > 0xFFFF)
        return 0;

    if (payload == NULL && payloadLen > 0)
        return 0;

    if (payloadLen + 3 > MAX_PAYLOAD_SIZE || payloadLen + 3 > capacity)
        return 0;

    out[0] = type;
    out[1] = (unsigned char)((payloadLen >> 8) & 0xFF); // L2, high byte
    out[2] = (unsigned char)(payloadLen & 0xFF);        // L1, low byte

    if (payloadLen > 0)
        memcpy(out + 3, payload, payloadLen);

    return payloadLen + 3;
}

static int parsePacket(const unsigned char *packet, size_t packetLen, size_t *payloadLen)
{
    size_t declared;

    if (packet == NULL || payloadLen == NULL || packetLen < 3)
        return -1;

    declared = 256u * (size_t)packet[1] + (size_t)packet[2];

    if (declared + 3 != packetLen)
        return -1;

    *payloadLen = declared;
    return (int)packet[0];
}

static size_t buildMetadata(unsigned char *out, size_t capacity,
                            unsigned long fileSize, const char *filename)
{
    size_t written = 0;
    size_t nameLen = (filename != NULL) ? strlen(filename) : 0;

    if (out == NULL || fileSize > MAX_FILE_SIZE)
        return 0;

    // refuse rather than drop the name silently: START and END would still agree
    if (nameLen > MAX_NAME_LEN)
        return 0;

    if (capacity < 2 + SIZE_VALUE_LEN + ((nameLen > 0) ? 2 + nameLen : 0))
        return 0;

    out[written++] = TLV_TYPE_SIZE;
    out[written++] = SIZE_VALUE_LEN;
    out[written++] = (unsigned char)((fileSize >> 24) & 0xFF);
    out[written++] = (unsigned char)((fileSize >> 16) & 0xFF);
    out[written++] = (unsigned char)((fileSize >> 8) & 0xFF);
    out[written++] = (unsigned char)(fileSize & 0xFF);

    if (nameLen > 0)
    {
        out[written++] = TLV_TYPE_NAME;
        out[written++] = (unsigned char)nameLen;
        memcpy(out + written, filename, nameLen);
        written += nameLen;
    }

    return written;
}

// Unknown types are skipped by their length, as the specification allows.
static int parseSize(const unsigned char *tlv, size_t len, unsigned long *fileSize)
{
    size_t pos = 0;
    bool haveSize = false;

    if (tlv == NULL || fileSize == NULL)
        return -1;

    *fileSize = 0;

    while (pos < len)
    {
        unsigned char type;
        size_t tlvLen;

        if (pos + 2 > len)
            return -1;

        type = tlv[pos];
        tlvLen = tlv[pos + 1];
        pos += 2;

        if (tlvLen > len - pos)
            return -1;

        if (type == TLV_TYPE_SIZE)
        {
            if (haveSize || tlvLen != SIZE_VALUE_LEN)
                return -1;

            *fileSize = ((unsigned long)tlv[pos] << 24) |
                        ((unsigned long)tlv[pos + 1] << 16) |
                        ((unsigned long)tlv[pos + 2] << 8) |
                        (unsigned long)tlv[pos + 3];

            haveSize = true;
        }

        pos += tlvLen;
    }

    return haveSize ? 0 : -1;
}

static int sendPacket(const unsigned char *packet, size_t packetLen)
{
    if (packet == NULL || packetLen == 0 || packetLen > MAX_PAYLOAD_SIZE)
    {
        fprintf(stderr, "A packet of %lu bytes does not fit in an information field.\n",
                (unsigned long)packetLen);
        return -1;
    }

    if (llSend(packet, (int)packetLen) < 0)
    {
        fprintf(stderr, "The link layer failed to send a %lu-byte packet.\n",
                (unsigned long)packetLen);
        return -1;
    }

    return 0;
}

static void applicationTx(const LinkLayer *params, const char *filename)
{
    FILE *file = fopen(filename, "rb");

    if (file == NULL)
    {
        perror(filename);
        return;
    }

    if (fseek(file, 0, SEEK_END) != 0)
    {
        perror("fseek");
        fclose(file);
        return;
    }

    const long fileSize = ftell(file);

    if (fileSize < 0)
    {
        perror("ftell");
        fclose(file);
        return;
    }

    if ((unsigned long)fileSize > MAX_FILE_SIZE)
    {
        fprintf(stderr, "File %s is too large (max %lu bytes).\n",
                filename, MAX_FILE_SIZE);
        fclose(file);
        return;
    }

    rewind(file);

    // one metadata block, two packet types
    unsigned char metadata[MAX_METADATA_SIZE];
    const size_t metadataLen = buildMetadata(metadata, sizeof(metadata),
                                             (unsigned long)fileSize, filename);

    if (metadataLen == 0)
    {
        fprintf(stderr, "Cannot announce %s: the name does not fit in %d characters.\n",
                filename, MAX_NAME_LEN);
        fclose(file);
        return;
    }

    unsigned char startPacket[MAX_PAYLOAD_SIZE];
    unsigned char endPacket[MAX_PAYLOAD_SIZE];
    const size_t startLen = buildPacket(startPacket, sizeof(startPacket),
                                       APP_C_START, metadata, metadataLen);
    const size_t endLen = buildPacket(endPacket, sizeof(endPacket),
                                     APP_C_END, metadata, metadataLen);

    if (startLen == 0 || endLen == 0)
    {
        fprintf(stderr, "Could not build the START/END packets.\n");
        fclose(file);
        return;
    }

    if (llOpenTx(*params) != 0)
    {
        fprintf(stderr, "Transfer of %s aborted: connection not established.\n", filename);
        fclose(file);
        return;
    }

    bool ok = (sendPacket(startPacket, startLen) == 0);

    unsigned char chunk[APP_DATA_CHUNK];
    size_t readBytes;

    while (ok && (readBytes = fread(chunk, 1, sizeof(chunk), file)) > 0)
        ok = (sendPacket(chunk, readBytes) == 0);

    if (ok && ferror(file))
    {
        fprintf(stderr, "Read error on %s.\n", filename);
        ok = false;
    }

    if (ok && sendPacket(endPacket, endLen) != 0)
        ok = false;

    fclose(file);
    llCloseTx();

    if (ok)
        printf("File %s sent successfully (%ld bytes).\n", filename, fileSize);
    else
        fprintf(stderr, "Transfer of %s failed.\n", filename);
}

static void applicationRx(const LinkLayer *params, const char *filename)
{
    if (llOpenRx(*params) != 0)
        return;

    unsigned char *packet = malloc(MAX_PAYLOAD_SIZE);

    if (packet == NULL)
    {
        fprintf(stderr, "Out of memory.\n");
        llCloseRx();
        return;
    }

    FILE *file = NULL;
    unsigned long declaredSize = 0;
    unsigned long receivedSize = 0;
    unsigned char startMetadata[MAX_METADATA_SIZE];
    size_t startMetadataLen = 0;
    bool sawStart = false;
    int idleCount = 0;
    const char *failure = NULL;

    for (;;)
    {
        int length = llReceive(packet);

        if (length < 0)
        {
            failure = "link layer receive failed";
            break;
        }

        if (length == 0)
        {
            // bounded, so a vanished transmitter cannot hang the receiver
            if (++idleCount > RX_SILENT_ROUNDS)
            {
                failure = "no data for too long";
                break;
            }
            continue;
        }

        idleCount = 0;

        size_t payloadLen = 0;
        int type = parsePacket(packet, (size_t)length, &payloadLen);

        if (type < 0)
        {
            failure = "malformed application packet";
            break;
        }

        if (type == APP_C_START)
        {
            if (sawStart)
            {
                failure = "duplicate START";
                break;
            }

            if (payloadLen > sizeof(startMetadata))
            {
                failure = "oversized START metadata";
                break;
            }

            if (parseSize(packet + 3, payloadLen, &declaredSize) != 0)
            {
                failure = "malformed START metadata";
                break;
            }

            // the output path is ours, never the one on the wire
            file = fopen(filename, "wb");

            if (file == NULL)
            {
                perror(filename);
                failure = "cannot create the output file";
                break;
            }

            // verbatim, so END can be compared octet for octet
            memcpy(startMetadata, packet + 3, payloadLen);
            startMetadataLen = payloadLen;
            sawStart = true;
            printf("Receiving %s (%lu bytes).\n", filename, declaredSize);
        }
        else if (type == APP_C_DATA)
        {
            if (!sawStart)
            {
                failure = "DATA outside the START..END window";
                break;
            }

            if (receivedSize + payloadLen > declaredSize)
            {
                failure = "more bytes than declared in START";
                break;
            }

            if (payloadLen > 0 &&
                fwrite(packet + 3, 1, payloadLen, file) != payloadLen)
            {
                perror("fwrite");
                failure = "write error";
                break;
            }

            receivedSize += (unsigned long)payloadLen;
        }
        else if (type == APP_C_END)
        {
            if (!sawStart)
            {
                failure = "END before START";
                break;
            }

            if (payloadLen != startMetadataLen ||
                memcmp(packet + 3, startMetadata, startMetadataLen) != 0)
            {
                failure = "END does not repeat START";
                break;
            }

            if (receivedSize != declaredSize)
            {
                failure = "received byte count does not match the declared size";
                break;
            }

            break;
        }
        else
        {
            failure = "unknown packet type";
            break;
        }
    }

    if (file != NULL)
        fclose(file);

    if (failure != NULL && sawStart)
    {
        // a truncated file reads as a near-miss in "diff -s ... || exit 0"
        remove(filename);
        fprintf(stderr, "Partial file %s removed.\n", filename);
    }

    free(packet);
    llCloseRx();

    if (failure != NULL)
        fprintf(stderr, "Transfer failed: %s.\n", failure);
    else
        printf("File %s received successfully (%lu bytes).\n", filename, receivedSize);
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // line-buffered so piped demo transcripts arrive line by line
    setvbuf(stdout, NULL, _IOLBF, 0);

    LinkLayer params;

    memset(&params, 0, sizeof(params));
    params.baudRate = baudRate;
    params.nRetransmissions = nTries;
    params.timeout = timeout;

    int written = snprintf(params.serialPort, sizeof(params.serialPort), "%s", serialPort);

    if (written < 0 || (size_t)written >= sizeof(params.serialPort))
    {
        fprintf(stderr, "Serial port name too long (max %lu characters).\n",
                (unsigned long)sizeof(params.serialPort) - 1);
        return;
    }

    if (strcmp(role, "tx") == 0)
        applicationTx(&params, filename);
    else if (strcmp(role, "rx") == 0)
        applicationRx(&params, filename);
    else
        fprintf(stderr, "Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
}
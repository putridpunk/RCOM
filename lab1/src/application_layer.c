//
// Application layer protocol implementation.
//
// Packet formats, as defined by the lab specification:
//
//   START : C=1 | TLV parameters
//   DATA  : C=2 | L2 | L1 | K payload bytes,  K = 256*L2 + L1
//   END   : C=3 | the same TLV parameters sent in START
//
// TLV encoding used here (agreed convention, documented so both ends match):
//   type 0, file size : length 4, value is an unsigned 32-bit big-endian count
//   type 1, file name : length 0..255, value is the raw name, no terminator
//   type 0 comes first. Unknown types are skipped by their length.
//
// Layering: this module only builds and parses whole packets. It writes the K
// fragment bytes of a DATA packet and nothing else -- no packet header, no TLV,
// no checksum. An RR returned by llSend means the link accepted the frame, not
// that the receiver managed to write it to disk.

#include "application_layer.h"
#include "link_layer.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

////////////////////////////////////////////////
// Protocol constants
////////////////////////////////////////////////

#define APP_C_START 1
#define APP_C_DATA 2
#define APP_C_END 3

#define TLV_TYPE_SIZE 0
#define TLV_TYPE_NAME 1

#define SIZE_VALUE_LEN 4
#define MAX_NAME_LEN 255
// Worst case metadata: size TLV (2 + 4) followed by a full length name TLV.
#define MAX_METADATA_SIZE (2 + SIZE_VALUE_LEN + 2 + MAX_NAME_LEN)

#define MAX_FILE_SIZE 0xFFFFFFFFUL

// A packet carries at most MAX_PAYLOAD_SIZE bytes, three of which are the
// C, L2 and L1 header fields.
#define DATA_CHUNK (MAX_PAYLOAD_SIZE - 3)

////////////////////////////////////////////////
// Packet construction and parsing
////////////////////////////////////////////////

// Build "C | L2 | L1 | payload". Returns the packet length, or 0 on error.
static size_t buildPacket(unsigned char *out, size_t capacity, unsigned char type,
                          const unsigned char *payload, size_t payloadLen)
{
    if (out == NULL)
        return 0;

    if (payloadLen > 0xFFFF)
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

// Validate "C | L2 | L1 | payload". Returns the type, or -1 if the declared
// length does not match the packet length exactly.
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

// Parse the TLV list of a START or END packet.
// Returns 0 on success, -1 on any structural problem or a missing size.
static int parseTlvs(const unsigned char *tlv, size_t len, unsigned long *fileSize,
                     const unsigned char **name, size_t *nameLen)
{
    size_t pos = 0;
    bool haveSize = false;

    if (tlv == NULL || fileSize == NULL || name == NULL || nameLen == NULL)
        return -1;

    *fileSize = 0;
    *name = NULL;
    *nameLen = 0;

    while (pos < len)
    {
        unsigned char type;
        unsigned char tlvLen;

        if (pos + 2 > len)
            return -1;

        type = tlv[pos];
        tlvLen = tlv[pos + 1];
        pos += 2;

        if (pos + tlvLen > len)
            return -1;

        if (type == TLV_TYPE_SIZE)
        {
            if (haveSize == true || tlvLen != SIZE_VALUE_LEN)
                return -1;

            *fileSize = ((unsigned long)tlv[pos] << 24) |
                        ((unsigned long)tlv[pos + 1] << 16) |
                        ((unsigned long)tlv[pos + 2] << 8) |
                        ((unsigned long)tlv[pos + 3]);

            haveSize = true;
        }
        else if (type == TLV_TYPE_NAME)
        {
            if (tlvLen > MAX_NAME_LEN)
                return -1;

            *name = tlv + pos;
            *nameLen = tlvLen;
        }

        // Unknown types fall through here and are skipped by their length.
        pos += tlvLen;
    }

    return haveSize ? 0 : -1;
}

// Build the START/END metadata block. Returns its length, or 0 on error.
static size_t buildMetadata(unsigned char *out, size_t capacity,
                            unsigned long fileSize, const char *filename)
{
    size_t written = 0;
    size_t nameLen = 0;

    if (out == NULL || fileSize > MAX_FILE_SIZE)
        return 0;

    if (capacity < 2 + SIZE_VALUE_LEN)
        return 0;

    out[written++] = TLV_TYPE_SIZE;
    out[written++] = SIZE_VALUE_LEN;
    out[written++] = (unsigned char)((fileSize >> 24) & 0xFF);
    out[written++] = (unsigned char)((fileSize >> 16) & 0xFF);
    out[written++] = (unsigned char)((fileSize >> 8) & 0xFF);
    out[written++] = (unsigned char)(fileSize & 0xFF);

    // strlen is used on a file name, never on binary payload.
    if (filename != NULL)
        nameLen = strlen(filename);

    if (nameLen > 0 && nameLen <= MAX_NAME_LEN)
    {
        if (written + 2 + nameLen > capacity)
            return 0;

        out[written++] = TLV_TYPE_NAME;
        out[written++] = (unsigned char)nameLen;
        memcpy(out + written, filename, nameLen);
        written += nameLen;
    }

    return written;
}

////////////////////////////////////////////////
// Transmitter
////////////////////////////////////////////////

// Build one packet and hand it to the link layer.
static int sendPacket(unsigned char type, const unsigned char *payload, size_t payloadLen)
{
    unsigned char packet[MAX_PAYLOAD_SIZE];
    size_t packetLen = buildPacket(packet, sizeof(packet), type, payload, payloadLen);

    if (packetLen == 0)
    {
        fprintf(stderr, "Payload of %lu bytes does not fit in a packet.\n",
                (unsigned long)payloadLen);
        return -1;
    }

    if (llSend(packet, (int)packetLen) < 0)
    {
        fprintf(stderr, "Link layer failed to send a type %u packet.\n",
                (unsigned int)type);
        return -1;
    }

    return 0;
}

static void applicationTx(const LinkLayer *params, const char *filename)
{
    FILE *file = fopen(filename, "rb");

    if (file == NULL)
    {
        perror("fopen");
        return;
    }

    if (fseek(file, 0, SEEK_END) != 0)
    {
        perror("fseek");
        fclose(file);
        return;
    }

    long fileSize = ftell(file);

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

    unsigned char metadata[MAX_METADATA_SIZE];
    size_t metadataLen = buildMetadata(metadata, sizeof(metadata),
                                       (unsigned long)fileSize, filename);

    if (metadataLen == 0)
    {
        fprintf(stderr, "Could not build the START/END metadata.\n");
        fclose(file);
        return;
    }

    if (llOpenTx(*params) != 0)
    {
        fclose(file);
        return;
    }

    int ok = TRUE;

    if (sendPacket(APP_C_START, metadata, metadataLen) != 0)
        ok = FALSE;

    unsigned char chunk[DATA_CHUNK];
    size_t readBytes = 0;

    while (ok == TRUE && (readBytes = fread(chunk, 1, sizeof(chunk), file)) > 0)
    {
        if (sendPacket(APP_C_DATA, chunk, readBytes) != 0)
            ok = FALSE;
    }

    if (ok == TRUE && ferror(file))
    {
        fprintf(stderr, "Read error on %s.\n", filename);
        ok = FALSE;
    }

    // END carries exactly the metadata START carried.
    if (ok == TRUE && sendPacket(APP_C_END, metadata, metadataLen) != 0)
        ok = FALSE;

    fclose(file);
    llCloseTx();

    if (ok == TRUE)
        printf("File %s sent successfully (%ld bytes).\n", filename, fileSize);
    else
        fprintf(stderr, "Transfer of %s failed.\n", filename);
}

////////////////////////////////////////////////
// Receiver
////////////////////////////////////////////////

static void applicationRx(const LinkLayer *params, const char *filename)
{
    const int idleBudget = params->nRetransmissions + 1;

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
    unsigned char startName[MAX_NAME_LEN + 1];
    size_t startNameLen = 0;
    bool sawStart = false;
    bool sawEnd = false;
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
            // Nothing arrived within one timeout interval. Bounded, so a
            // vanished transmitter cannot hang the receiver forever.
            if (++idleCount > idleBudget)
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
            unsigned long size = 0;
            const unsigned char *name = NULL;
            size_t nameLen = 0;

            if (sawStart == true)
            {
                failure = "duplicate START";
                break;
            }

            if (parseTlvs(packet + 3, payloadLen, &size, &name, &nameLen) != 0)
            {
                failure = "malformed START metadata";
                break;
            }

            declaredSize = size;
            startNameLen = nameLen;

            if (nameLen > 0)
                memcpy(startName, name, nameLen);

            file = fopen(filename, "wb");

            if (file == NULL)
            {
                perror("fopen");
                failure = "cannot create the output file";
                break;
            }

            sawStart = true;
            printf("Receiving %s (%lu bytes).\n", filename, declaredSize);
        }
        else if (type == APP_C_DATA)
        {
            if (sawStart == false || sawEnd == true)
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
            unsigned long size = 0;
            const unsigned char *name = NULL;
            size_t nameLen = 0;

            if (sawStart == false)
            {
                failure = "END before START";
                break;
            }

            if (parseTlvs(packet + 3, payloadLen, &size, &name, &nameLen) != 0)
            {
                failure = "malformed END metadata";
                break;
            }

            if (size != declaredSize)
            {
                failure = "START and END disagree on the file size";
                break;
            }

            if (nameLen != startNameLen ||
                (nameLen > 0 && memcmp(name, startName, nameLen) != 0))
            {
                failure = "START and END disagree on the file name";
                break;
            }

            if (receivedSize != declaredSize)
            {
                failure = "received byte count does not match the declared size";
                break;
            }

            sawEnd = true;
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

    free(packet);
    llCloseRx();

    if (failure != NULL)
        fprintf(stderr, "Transfer failed: %s.\n", failure);
    else if (sawEnd == true)
        printf("File %s received successfully (%lu bytes).\n", filename, receivedSize);
    else
        fprintf(stderr, "Transfer failed: the transfer never reached END.\n");
}

////////////////////////////////////////////////
// Entry point
////////////////////////////////////////////////

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
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
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
}
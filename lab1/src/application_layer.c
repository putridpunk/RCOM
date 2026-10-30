// packets:
//
//   START: C=1 | TLVs
//   DATA : C=2 | L2 | L1 | data     size = 256*L2 + L1
//   END  : C=3 | same TLVs as START
//
// TLVs: type 0 is the size, 4 bytes big endian. type 1 is the name, 0..255 raw
// bytes with no \0. size goes first, unknown types get skipped by their length.
//

#include "application_layer.h"
#include "link_layer.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// packet types
#define APP_C_START 1
#define APP_C_DATA 2
#define APP_C_END 3

#define TLV_TYPE_SIZE 0
#define TLV_TYPE_NAME 1

#define SIZE_VALUE_LEN 4
#define MAX_NAME_LEN 255 // one byte of length, so this is the ceiling
// size TLV (2+4) plus a name TLV at full length
#define MAX_METADATA_SIZE (2 + SIZE_VALUE_LEN + 2 + MAX_NAME_LEN)

// the size is 4 octets so the file has to fit in 32 bits
#define MAX_FILE_SIZE 0xFFFFFFFFUL

// how much file data goes in each DATA packet. 3 of the payload bytes are the
// C/L2/L1 header.
//
// build with -DAPP_DATA_CHUNK=... for the efficiency measurements. rx reads K
// back out of L2/L1 so both ends stay compatible without changing anything else
#ifndef APP_DATA_CHUNK
#define APP_DATA_CHUNK (MAX_PAYLOAD_SIZE - 3)
#endif

_Static_assert(APP_DATA_CHUNK >= 1 && APP_DATA_CHUNK + 3 <= MAX_PAYLOAD_SIZE,
               "APP_DATA_CHUNK must leave room for the packet header");
_Static_assert(MAX_METADATA_SIZE + 3 <= MAX_PAYLOAD_SIZE,
               "START/END metadata must fit in one packet");

// how many empty llReceive rounds rx puts up with before giving up. has to be
// longer than tx's own budget for a single frame, else tx gives up first and rx
// sits there forever. 12*4 = 48s vs ~24s worst case for tx.
// arbitrary, just needs the margin
#define RX_SILENT_ROUNDS 12

// C | L2 | L1 | payload. returns the packet length, 0 on error
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
    out[1] = (unsigned char)((payloadLen >> 8) & 0xFF); // L2 = high byte
    out[2] = (unsigned char)(payloadLen & 0xFF);        // L1 = low byte

    if (payloadLen > 0)
        memcpy(out + 3, payload, payloadLen);

    return payloadLen + 3;
}

// returns the type, or -1 if L2/L1 doesn't match what we actually read
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

// builds the TLV block, START and END both use this. returns its length, 0 if
// it doesn't fit
static size_t buildMetadata(unsigned char *out, size_t capacity,
                            unsigned long fileSize, const char *filename)
{
    size_t written = 0;
    size_t nameLen = (filename != NULL) ? strlen(filename) : 0;

    if (out == NULL || fileSize > MAX_FILE_SIZE)
        return 0;

    // can't announce a name this long, one byte of length. refusing is the only
    // option, if we just left the name out START and END would still agree
    // with each other and nothing downstream would notice
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

// walks the START TLVs and gets the size out. 0 on success, -1 on anything
// wrong (bad structure, no size, two sizes)
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

// hand a finished packet to the link layer. careful: an RR here means the frame
// left tx, not that rx wrote anything to disk
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

    // one metadata block, then START and END are the same bytes with a
    // different C
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
            // empty round, count them. bounded so a tx that dies doesn't leave
            // us hanging forever
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

            // path comes from OUR argv. whatever name is on the wire gets
            // checked (END vs START) but never used to open anything
            file = fopen(filename, "wb");

            if (file == NULL)
            {
                perror(filename);
                failure = "cannot create the output file";
                break;
            }

            // kept exactly as it came so END can be memcmp'd against it, no
            // field by field comparison
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

            // spec says END repeats the info in START so thats what we check,
            // literally the same bytes
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

            break; // only way out of the loop that isn't a failure
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
        // delete the leftover file.
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
    // line buffered
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
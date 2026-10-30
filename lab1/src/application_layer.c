// Application layer protocol implementation.
//
// START: C=1 | TLVs   DATA: C=2 | L2 | L1 | K bytes   END: C=3 | TLVs
// TLV 0 = size (4B big-endian), TLV 1 = name (0..255 raw); unknown types
// are skipped by their length.
//
// START and END are built from one metadata block, so END repeats START octet
// for octet and the receiver can check exactly that with memcmp instead of
// comparing field by field and trusting that it enumerated them all.
//
// The receiver takes the output path from its own command line, never from the
// wire: the name TLV is carried and verified, not obeyed.
//
// Layering: this module only builds and parses whole packets, and writes the K
// fragment bytes of a DATA packet and nothing else -- no frame header, no TLV,
// no checksum. An RR means the link accepted the frame, not that the receiver
// wrote it to disk.

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

// The size TLV is four octets, so a file must fit in 32 bits.
#define MAX_FILE_SIZE 0xFFFFFFFFUL

// Fragment carried by one DATA packet. A packet holds at most MAX_PAYLOAD_SIZE
// bytes, three of which are the C, L2 and L1 fields.
//
// Override with -DAPP_DATA_CHUNK=... to measure efficiency with smaller
// information frames: the receiver reads K from L2/L1 of every packet, so both
// ends stay compatible without any other change.
#ifndef APP_DATA_CHUNK
#define APP_DATA_CHUNK (MAX_PAYLOAD_SIZE - 3)
#endif

_Static_assert(APP_DATA_CHUNK >= 1 && APP_DATA_CHUNK + 3 <= MAX_PAYLOAD_SIZE,
               "APP_DATA_CHUNK must leave room for the packet header");
_Static_assert(MAX_METADATA_SIZE + 3 <= MAX_PAYLOAD_SIZE,
               "START/END metadata must fit in one packet");

// Consecutive silent timeouts the receiver tolerates between packets.
//
// It must strictly outlast the transmitter's own budget for one frame,
// (nRetransmissions + 1) * (timeout + one frame time): 4 * (4 + 1.05) = 20 s at
// 9600 baud and 4 * (4 + 2.10) = 24 s at 4800, the slowest rate at which a full
// frame still fits inside the timeout. Twelve rounds of four seconds is safely
// above both, so the transmitter is always the side that declares a transfer
// failed and the receiver never reports a different reason for the same event.
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

// The declared length must match the packet length exactly: that is the
// integrity check the specification asks of the two L octets.
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

    // A name that does not fit its one-octet length field cannot be announced.
    // Refuse the transfer rather than dropping the name silently: START and END
    // would still agree with each other, so nothing downstream would notice.
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

// The only parameter this receiver needs from START is the size; it validates
// END by comparing the whole block, which also covers parameters it does not
// understand. Unknown types are skipped by their length, as the spec allows.
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

    // One metadata block, two packet types: END is START with a different C.
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
        // This is the whole transcript of a transfer attempted with no receiver.
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
            // Nothing arrived within one timeout interval. Bounded, so a
            // vanished transmitter cannot hang the receiver forever.
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

            file = fopen(filename, "wb");

            if (file == NULL)
            {
                perror(filename);
                failure = "cannot create the output file";
                break;
            }

            // Kept verbatim, so END can be compared against it octet for octet.
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

            // "END repeats the information in START", checked as what it is.
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

            break; // the only clean exit from this loop
        }
        else
        {
            failure = "unknown packet type";
            break;
        }
    }

    // fclose is where a write to a full disk finally fails, and it is the last
    // chance to notice: without this the run would report success on a file
    // that was never completely written.
    if (file != NULL && fclose(file) != 0)
    {
        perror("fclose");
        if (failure == NULL)
            failure = "the output file could not be closed";
    }

    if (failure != NULL && sawStart)
    {
        // Never leave a truncated file behind. "make check_files" is
        // "diff -s ... || exit 0", so a half-written file reads as a near-miss
        // in the demo and hides the real cause of the failure.
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
    // Best effort, and the only way to get line-buffered output without editing
    // main.c: strictly, setvbuf must be called before the stream is used and
    // main has already printed its banner, but glibc accepts it and a demo
    // transcript through tee is worth more than the letter of 7.19.5.6.
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
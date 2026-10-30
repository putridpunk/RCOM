// Link layer protocol implementation.
//
// supervision: F | A | C | BCC1 | F
// information: F | A | C | BCC1 | packet | BCC2 | F, both stuffed
// Stop-and-Wait, alternating Ns.

#include "link_layer.h"
#include "serial_port.h"
#include "alarm_sigaction.h"
#include "frame.h"

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

typedef enum
{
    ROLE_NONE = 0,
    ROLE_TX,
    ROLE_RX
} LinkRole;

typedef enum
{
    RX_OK_SUPERVISION,
    RX_OK_DATA,
    RX_IGNORED,   // malformed, truncated or oversized
    RX_BAD_BCC2,
    RX_TIMEOUT,
    RX_IO_ERROR
} RxResult;

typedef enum
{
    ST_HUNT_FLAG,
    ST_READ_A,
    ST_READ_C,
    ST_READ_BCC1,
    ST_READ_SUPERVISION_F,
    ST_READ_DATA
} RxState;

typedef enum
{
    ACKED,
    REJECTED,
    ACK_TIMEOUT,
    ACK_ERROR
} AckResult;

typedef struct
{
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

    unsigned long infoSent;
    unsigned long infoReceived;

    unsigned long retransmissions;
    unsigned long timeouts;
    unsigned long bcc1Errors;
    unsigned long bcc2Errors;
    unsigned long duplicateFrames;
    unsigned long addressErrors;

    // counted once per packet, not once per retry
    unsigned long payloadBytesSent;
    unsigned long payloadBytesReceived;

    struct timespec startTime;
    bool started;
} LinkStats;

static LinkStats linkStats;

static LinkRole currentRole = ROLE_NONE;
static LinkLayer linkParams;

static uint8_t txNs = 0;
static uint8_t rxExpected = 0;

// one byte past MAX_PAYLOAD_SIZE: the trailing BCC2
static unsigned char rxBody[MAX_PAYLOAD_SIZE + FRAME_BCC2_SIZE];

// Write exactly "length" bytes, looping over short writes.
static int writeAll(const unsigned char *buf, size_t length)
{
    size_t written = 0;

    if (buf == NULL)
        return -1;

    while (written < length)
    {
        size_t remaining = length - written;
        int chunk = (remaining > (size_t)INT_MAX) ? INT_MAX : (int)remaining;
        int result = writeBytesSerialPort(buf + written, chunk);

        if (result < 0)
        {
            if (errno == EINTR)
                continue;

            perror("writeBytesSerialPort");
            return -1;
        }

        if (result == 0)
        {
            fprintf(stderr, "writeBytesSerialPort made no progress.\n");
            return -1;
        }

        written += (size_t)result;
    }

    return 0;
}

static void statsReset(void)
{
    memset(&linkStats, 0, sizeof(linkStats));

    linkStats.started = (clock_gettime(CLOCK_MONOTONIC, &linkStats.startTime) == 0);
}

static void statsCountSupervisionSent(unsigned char control)
{
    switch (control)
    {
    case C_SET:
        linkStats.setSent++;
        break;
    case C_UA:
        linkStats.uaSent++;
        break;
    case C_DISC:
        linkStats.discSent++;
        break;
    case C_RR0:
    case C_RR1:
        linkStats.rrSent++;
        break;
    case C_REJ0:
    case C_REJ1:
        linkStats.rejSent++;
        break;
    default:
        break;
    }
}

static void statsCountSupervisionReceived(unsigned char control)
{
    switch (control)
    {
    case C_SET:
        linkStats.setReceived++;
        break;
    case C_UA:
        linkStats.uaReceived++;
        break;
    case C_DISC:
        linkStats.discReceived++;
        break;
    case C_RR0:
    case C_RR1:
        linkStats.rrReceived++;
        break;
    case C_REJ0:
    case C_REJ1:
        linkStats.rejReceived++;
        break;
    default:
        break;
    }
}

static double statsElapsedSeconds(void)
{
    struct timespec now;

    if (!linkStats.started)
        return 0.0;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
        return 0.0;

    return (double)(now.tv_sec - linkStats.startTime.tv_sec) +
           (double)(now.tv_nsec - linkStats.startTime.tv_nsec) / 1.0e9;
}

static void statsPrint(void)
{
    const double elapsed = statsElapsedSeconds();
    const double payload = (double)linkStats.payloadBytesSent +
                           (double)linkStats.payloadBytesReceived;

    printf("\n--- Link layer statistics ---\n");
    printf("  elapsed ................ %8.3f s\n", elapsed);

    printf("  SET   sent/received .... %8lu / %lu\n", linkStats.setSent, linkStats.setReceived);
    printf("  UA    sent/received .... %8lu / %lu\n", linkStats.uaSent, linkStats.uaReceived);
    printf("  DISC  sent/received .... %8lu / %lu\n", linkStats.discSent, linkStats.discReceived);
    printf("  RR    sent/received .... %8lu / %lu\n", linkStats.rrSent, linkStats.rrReceived);
    printf("  REJ   sent/received .... %8lu / %lu\n", linkStats.rejSent, linkStats.rejReceived);

    printf("  I     sent/received .... %8lu / %lu\n", linkStats.infoSent, linkStats.infoReceived);
    printf("  payload sent/received .. %8lu / %lu bytes\n",
           linkStats.payloadBytesSent, linkStats.payloadBytesReceived);

    printf("  retransmissions ........ %8lu\n", linkStats.retransmissions);
    printf("  timeouts ............... %8lu\n", linkStats.timeouts);
    printf("  BCC1 errors ............ %8lu\n", linkStats.bcc1Errors);
    printf("  BCC2 errors ............ %8lu\n", linkStats.bcc2Errors);
    printf("  duplicate frames ....... %8lu\n", linkStats.duplicateFrames);
    printf("  wrong address/class .... %8lu\n", linkStats.addressErrors);

    if (elapsed > 0.0)
    {
        printf("  throughput ............. %8.1f B/s\n", payload / elapsed);

        // S = R / C; ten bits on the wire per octet (8-N-1)
        if (linkParams.baudRate > 0)
            printf("  efficiency ............. %8.2f %%\n",
                   100.0 * payload * 10.0 / (elapsed * (double)linkParams.baudRate));
    }

    printf("-----------------------------\n");
}

// commands carry our address, replies carry the peer's
static unsigned char supervisionAddr(unsigned char control)
{
    if (currentRole == ROLE_TX)
        return (control == C_UA) ? A_RX : A_TX;

    if (currentRole == ROLE_RX)
        return (control == C_DISC) ? A_RX : A_TX;

    return A_TX;
}

static int sendSupervision(unsigned char control)
{
    unsigned char frame[SUPERVISION_FRAME_SIZE];
    unsigned char address = supervisionAddr(control);

    int built = frameBuildSupervision(frame, sizeof(frame), address, control);
    if (built != SUPERVISION_FRAME_SIZE)
        return -1;

    if (writeAll(frame, (size_t)built) != 0)
        return -1;

    statsCountSupervisionSent(control);
    return 0;
}

// a lost RR/REJ is recovered by the peer's timeout, so this is never fatal
static void sendReply(unsigned char control)
{
    if (sendSupervision(control) != 0)
        fprintf(stderr, "Could not send supervision frame %s.\n",
                frameControlName(control));
}

static bool addressAccepted(unsigned char address, unsigned char control)
{
    if (currentRole == ROLE_RX)
    {
        if (address == A_TX)
            return control == C_SET || control == C_DISC ||
                   control == C_I0 || control == C_I1;

        // the UA answering the DISC we send during close
        if (address == A_RX)
            return control == C_UA;

        return false;
    }

    if (currentRole == ROLE_TX)
    {
        if (address == A_TX)
            return control == C_UA || control == C_RR0 || control == C_RR1 ||
                   control == C_REJ0 || control == C_REJ1;

        // the DISC answering ours, and its retransmissions
        if (address == A_RX)
            return control == C_DISC;

        return false;
    }

    return false;
}

// packet/*packetLen are filled on RX_OK_DATA only. *controlOut is set for
// supervision frames and for RX_BAD_BCC2, so the caller can send the right REJ.
static RxResult rxReadFrame(unsigned char *packet, size_t packetCapacity,
                            int *packetLen, unsigned char *controlOut)
{
    RxState state = ST_HUNT_FLAG;

    unsigned char address = 0;
    unsigned char control = 0;

    size_t filled = 0;
    bool overflow = false;
    bool escape = false;

    if (packet == NULL || packetLen == NULL || controlOut == NULL)
        return RX_IO_ERROR;

    if (packetCapacity < MAX_PAYLOAD_SIZE)
        return RX_IO_ERROR;

    *packetLen = 0;
    *controlOut = 0;

    for (;;)
    {
        unsigned char byte = 0;
        int result = readByteSerialPort(&byte);

        if (result < 0)
        {
            if (errno == EINTR)
            {
                // only our handler clears the flag, so cleared means our alarm
                if (!isAlarmActive())
                    return RX_TIMEOUT;

                continue;
            }

            perror("readByteSerialPort");
            return RX_IO_ERROR;
        }

        // VMIN is 1 in serial_port.c, so a zero read is EOF, not "nothing yet"
        if (result == 0)
        {
            fprintf(stderr, "Serial port reported end of file.\n");
            return RX_IO_ERROR;
        }

        switch (state)
        {
        case ST_HUNT_FLAG:
            if (byte == FRAME_FLAG)
                state = ST_READ_A;
            break;

        case ST_READ_A:
            // several opening flags allowed
            if (byte == FRAME_FLAG)
                break;

            address = byte;
            state = ST_READ_C;
            break;

        case ST_READ_C:
            if (byte == FRAME_FLAG)
            {
                state = ST_READ_A;
                break;
            }

            control = byte;
            state = ST_READ_BCC1;
            break;

        case ST_READ_BCC1:
            if (byte == FRAME_FLAG)
            {
                state = ST_READ_A;
                break;
            }

            if (byte != frameBcc1(address, control))
            {
                // damaged header: silent discard, recovered by timeout
                linkStats.bcc1Errors++;
                state = ST_HUNT_FLAG;
                break;
            }

            if (!addressAccepted(address, control))
            {
                linkStats.addressErrors++;
                state = ST_HUNT_FLAG;
                break;
            }

            if (frameCIsInfo(control))
            {
                filled = 0;
                overflow = false;
                escape = false;
                state = ST_READ_DATA;
            }
            else
            {
                state = ST_READ_SUPERVISION_F;
            }
            break;

        case ST_READ_SUPERVISION_F:
            if (byte != FRAME_FLAG)
            {
                // malformed: resync on the next flag
                state = ST_HUNT_FLAG;
                break;
            }

            *controlOut = control;
            statsCountSupervisionReceived(control);
            return RX_OK_SUPERVISION;

        case ST_READ_DATA:
            if (escape)
            {
                escape = false;

                if (byte == FRAME_ESCAPE_FLAG)
                    byte = FRAME_FLAG;
                else if (byte == FRAME_ESCAPE_ESCAPE)
                    byte = FRAME_ESCAPE;
                else
                {
                    state = ST_HUNT_FLAG;
                    break;
                }

                if (filled < sizeof(rxBody))
                    rxBody[filled++] = byte;
                else
                    overflow = true;
                break;
            }

            if (byte == FRAME_ESCAPE)
            {
                escape = true;
                break;
            }

            if (byte != FRAME_FLAG)
            {
                if (filled < sizeof(rxBody))
                    rxBody[filled++] = byte;
                else
                    overflow = true;
                break;
            }

            // closing flag; the last de-stuffed byte is the BCC2
            state = ST_HUNT_FLAG;

            if (overflow || filled < 1)
                return RX_IGNORED;

            *controlOut = control;

            if (frameBcc2(rxBody, filled - 1) != rxBody[filled - 1])
            {
                linkStats.bcc2Errors++;
                return RX_BAD_BCC2;
            }

            *packetLen = (int)(filled - 1);

            if (filled > 1)
                memcpy(packet, rxBody, filled - 1);

            return RX_OK_DATA;
        }
    }
}

// One wait for every supervision exchange. A duplicate is still acked: dropping
// a retransmitted END would burn the peer's whole retry budget.
//
// 1 and *controlOut on success, 0 on timeout, -1 on I/O error.
static int waitControl(unsigned char *controlOut)
{
    unsigned char packet[MAX_PAYLOAD_SIZE];
    int length = 0;
    unsigned char control = 0;

    for (;;)
    {
        RxResult result = rxReadFrame(packet, sizeof(packet), &length, &control);

        if (result == RX_TIMEOUT)
            return 0;

        if (result == RX_IO_ERROR)
            return -1;

        if (result == RX_OK_SUPERVISION)
        {
            *controlOut = control;
            return 1;
        }

        // the transmitter never accepts data
        if (currentRole != ROLE_RX)
            continue;

        if (frameCGetNs(control) != rxExpected)
        {
            linkStats.duplicateFrames++;
            sendReply(C_RR(rxExpected));
            continue;
        }

        if (result == RX_BAD_BCC2)
        {
            sendReply(C_REJ(rxExpected));
            continue;
        }

        // a new frame belongs to llReceive(); the peer's timeout repeats it
    }
}

// Send a supervision frame and wait for the reply that answers it, retrying up
// to nRetransmissions extra times. 0 on success, -1 when every attempt failed.
static int sendAndWait(unsigned char command, unsigned char expected)
{
    const int attempts = linkParams.nRetransmissions + 1;

    for (int attempt = 1; attempt <= attempts; attempt++)
    {
        unsigned char control = 0;
        int result;

        if (sendSupervision(command) != 0)
            return -1;

        if (attempt > 1)
            linkStats.retransmissions++;

        startAlarm((unsigned)linkParams.timeout);

        do
        {
            result = waitControl(&control);
        } while (result == 1 && control != expected);

        stopAlarm();

        if (result < 0)
            return -1;

        if (result == 1)
            return 0;

        linkStats.timeouts++;
        printf("Timeout waiting for %s (attempt %d of %d).\n",
               frameControlName(expected), attempt, attempts);
    }

    return -1;
}

// RR with Nr != Ns acks; REJ with Nr == Ns rejects. The rest is stale.
static AckResult waitAck(unsigned char ns)
{
    for (;;)
    {
        unsigned char control = 0;
        int result = waitControl(&control);

        if (result < 0)
            return ACK_ERROR;

        if (result == 0)
        {
            linkStats.timeouts++;
            return ACK_TIMEOUT;
        }

        if (frameCIsRR(control))
        {
            if (frameCGetNr(control) != ns)
                return ACKED;

            continue;
        }

        if (frameCIsREJ(control))
        {
            if (frameCGetNr(control) == ns)
                return REJECTED;

            continue;
        }
    }
}

int llOpenTx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened (transmitter).\n", llParameters.serialPort);

    if (setupAlarmHandler() == -1)
    {
        perror("setupAlarmHandler");
        closeSerialPort();
        return -1;
    }

    if (llParameters.timeout <= 0)
    {
        fprintf(stderr, "Invalid timeout (%d s).\n", llParameters.timeout);
        closeSerialPort();
        return -1;
    }

    linkParams = llParameters;
    currentRole = ROLE_TX;
    txNs = 0;
    rxExpected = 0;
    stopAlarm();
    statsReset();

    // nRetransmissions counts retries after the first attempt
    if (sendAndWait(C_SET, C_UA) != 0)
    {
        closeSerialPort();
        currentRole = ROLE_NONE;
        fprintf(stderr, "Connection failed: no UA after %d attempts.\n",
                llParameters.nRetransmissions + 1);
        return -1;
    }

    printf("UA received. Connection established.\n");
    return 0;
}

// Bounded: (nRetransmissions + 1) rounds.
int llOpenRx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened (receiver).\n", llParameters.serialPort);

    // the close handshake arms an alarm in both roles, so install it here too
    if (setupAlarmHandler() == -1)
    {
        perror("setupAlarmHandler");
        closeSerialPort();
        return -1;
    }

    if (llParameters.timeout <= 0)
    {
        fprintf(stderr, "Invalid timeout (%d s).\n", llParameters.timeout);
        closeSerialPort();
        return -1;
    }

    linkParams = llParameters;
    currentRole = ROLE_RX;
    txNs = 0;
    rxExpected = 0;
    stopAlarm();
    statsReset();

    const int rounds = llParameters.nRetransmissions + 1;

    for (int round = 1; round <= rounds; round++)
    {
        unsigned char control = 0;
        int result;

        startAlarm((unsigned)llParameters.timeout);

        do
        {
            result = waitControl(&control);
        } while (result == 1 && control != C_SET);

        stopAlarm();

        if (result < 0)
            break;

        if (result == 1)
        {
            printf("SET received. Sending UA.\n");

            if (sendSupervision(C_UA) != 0)
                break;

            return 0;
        }

        linkStats.timeouts++;
        printf("Timeout waiting for SET (round %d of %d).\n", round, rounds);
    }

    closeSerialPort();
    currentRole = ROLE_NONE;
    fprintf(stderr, "Connection failed: no SET received.\n");
    return -1;
}

int llSend(const unsigned char *buf, int bufSize)
{
    if (buf == NULL || bufSize <= 0 || bufSize > MAX_PAYLOAD_SIZE)
        return -1;

    if (currentRole != ROLE_TX || linkParams.timeout <= 0)
        return -1;

    unsigned char wire[MAX_FRAME_SIZE];
    const int attempts = linkParams.nRetransmissions + 1;

    for (int attempt = 1; attempt <= attempts; attempt++)
    {
        int built = frameBuildInfo(wire, sizeof(wire), A_TX, txNs, buf, (size_t)bufSize);

        if (built < 0)
        {
            fprintf(stderr, "Could not build the information frame.\n");
            return -1;
        }

        if (attempt > 1)
            linkStats.retransmissions++;

        if (writeAll(wire, (size_t)built) != 0)
            return -1;

        linkStats.infoSent++;
        if (attempt == 1)
            linkStats.payloadBytesSent += (unsigned long)bufSize;

        startAlarm((unsigned)linkParams.timeout);
        AckResult ack = waitAck(txNs);
        stopAlarm();

        if (ack == ACKED)
        {
            txNs ^= 1u;
            return bufSize;
        }

        if (ack == ACK_ERROR)
            return -1;

        // timeout or REJ: same number, same bytes
    }

    fprintf(stderr, "llSend: no acknowledgement after %d attempts.\n", attempts);
    return -1;
}

int llReceive(unsigned char *buf)
{
    if (buf == NULL || currentRole != ROLE_RX || linkParams.timeout <= 0)
        return -1;

    for (;;)
    {
        unsigned char control = 0;
        int length = 0;

        startAlarm((unsigned)linkParams.timeout);
        RxResult result = rxReadFrame(buf, MAX_PAYLOAD_SIZE, &length, &control);
        stopAlarm();

        if (result == RX_TIMEOUT)
            return 0;

        if (result == RX_IO_ERROR)
            return -1;

        if (result == RX_IGNORED)
            continue;

        if (result == RX_OK_SUPERVISION)
        {
            if (control == C_SET)
            {
                // we lost the UA; re-acking is idempotent
                sendReply(C_UA);
                continue;
            }

            if (control == C_DISC)
            {
                // the transmitter aborted; let the close handshake answer it
                fprintf(stderr, "DISC received during the transfer; "
                                "the transmitter has aborted.\n");
                return -1;
            }

            continue;
        }

        if (frameCGetNs(control) != rxExpected)
        {
            linkStats.duplicateFrames++;
            sendReply(C_RR(rxExpected));
            continue;
        }

        if (result == RX_BAD_BCC2)
        {
            sendReply(C_REJ(rxExpected));
            continue;
        }

        rxExpected ^= 1u;
        sendReply(C_RR(rxExpected));

        linkStats.infoReceived++;
        linkStats.payloadBytesReceived += (unsigned long)length;

        return length;
    }
}

int llCloseTx(void)
{
    if (currentRole != ROLE_TX || linkParams.timeout <= 0)
        return -1;

    int status = 0;

    stopAlarm();

    if (sendAndWait(C_DISC, C_DISC) != 0)
    {
        fprintf(stderr, "Close handshake failed: no DISC from the receiver.\n");
        status = -1;
    }
    else if (sendSupervision(C_UA) != 0)
    {
        status = -1;
    }
    else
    {
        // if our UA was lost the peer repeats DISC: answer each, stop on the
        // first quiet window so noise cannot extend this forever
        unsigned char control = 0;
        const int windows = linkParams.nRetransmissions + 1;

        for (int i = 0; i < windows; i++)
        {
            startAlarm((unsigned)linkParams.timeout);
            int result = waitControl(&control);
            stopAlarm();

            if (result != 1)
                break;

            if (control == C_DISC && sendSupervision(C_UA) != 0)
                break;
        }
    }

    closeSerialPort();
    statsPrint();
    currentRole = ROLE_NONE;

    if (status == 0)
        printf("Serial port closed (transmitter).\n");

    return status;
}

int llCloseRx(void)
{
    if (currentRole != ROLE_RX || linkParams.timeout <= 0)
        return -1;

    int status = 0;
    bool gotDisc = false;
    const int rounds = linkParams.nRetransmissions + 1;

    stopAlarm();

    // waitControl keeps acking a retransmitted END, so a lost RR for the last
    // packet costs the transmitter one retry instead of its whole budget
    for (int round = 1; round <= rounds; round++)
    {
        unsigned char control = 0;
        int result;

        startAlarm((unsigned)linkParams.timeout);

        do
        {
            result = waitControl(&control);
        } while (result == 1 && control != C_DISC);

        stopAlarm();

        if (result < 0)
        {
            fprintf(stderr, "Close handshake failed: serial port error.\n");
            status = -1;
            break;
        }

        if (result == 1)
        {
            gotDisc = true;
            break;
        }

        linkStats.timeouts++;
        printf("Timeout waiting for DISC (round %d of %d).\n", round, rounds);
    }

    if (status == 0 && !gotDisc)
        fprintf(stderr, "Close handshake failed: no DISC from the transmitter.\n");

    if (status == 0 && gotDisc && sendAndWait(C_DISC, C_UA) != 0)
    {
        fprintf(stderr, "Close handshake failed: no UA from the transmitter.\n");
        status = -1;
    }

    closeSerialPort();
    statsPrint();
    currentRole = ROLE_NONE;

    if (status == 0)
        printf("Serial port closed (receiver).\n");

    return status;
}
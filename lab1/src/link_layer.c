// Link layer protocol implementation.
//
// Frames on the wire, before stuffing:
//   supervision : F | A | C | BCC1 | F
//   information : F | A | C | BCC1 | application packet | BCC2 | F
//
// Stop-and-Wait with modulo-2 numbering. Only the transmitter sends information
// frames; an RR or REJ carries Nr, the next information frame the receiver
// expects. Ns advances only when an RR carrying Nr = !Ns arrives, so every retry
// repeats the same number and the same bytes.
//
// Every supervision exchange here has the same shape: send a frame, arm the
// alarm, wait for the reply that answers it, retry a bounded number of times.
// That shape is implemented once, in sendAndWait() and waitAck(); everything
// else is a few lines around them.
//
// Layering: the application packet is opaque. This file never looks at its
// header and never modifies it.

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

// Outcome of one attempt at reading a frame from the serial port.
typedef enum
{
    RX_OK_SUPERVISION,
    RX_OK_DATA,
    RX_IGNORED, // malformed, truncated or oversized
    RX_BAD_BCC2,
    RX_TIMEOUT,
    RX_IO_ERROR
} RxResult;

// States of the frame reception machine.
typedef enum
{
    ST_HUNT_FLAG,
    ST_READ_A,
    ST_READ_C,
    ST_READ_BCC1,
    ST_READ_SUPERVISION_F,
    ST_READ_DATA
} RxState;

// Verdict on an information frame that has just been transmitted.
typedef enum
{
    ACKED,
    REJECTED,
    ACK_TIMEOUT,
    ACK_ERROR
} AckResult;

// Counters for one connection, which is why they live here and not in the pure
// frame module.
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

    // Counted once per packet, not once per retry.
    unsigned long payloadBytesSent;
    unsigned long payloadBytesReceived;

    struct timespec startTime;
    bool started;

    // Seconds from connection established to the last packet, filled by
    // statsFreeze() when the close handshake begins.
    double elapsed;
} LinkStats;

static LinkStats linkStats;

static LinkRole currentRole = ROLE_NONE;
static LinkLayer linkParams;

static uint8_t txNs = 0;
static uint8_t rxExpected = 0;

// De-stuffed body of the frame being received: the packet plus its trailing
// BCC2, one octet longer than MAX_PAYLOAD_SIZE for exactly that reason.
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

// Stop the clock when the data phase ends. Without this the reported time, and
// so the throughput and the efficiency, would also contain the close handshake
// and the transmitter's final linger -- four seconds of a thirty second
// transfer, which is exactly the number the efficiency plot is about.
static void statsFreeze(void)
{
    struct timespec now;

    if (!linkStats.started || linkStats.elapsed > 0.0)
        return;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
        return;

    linkStats.elapsed = (double)(now.tv_sec - linkStats.startTime.tv_sec) +
                        (double)(now.tv_nsec - linkStats.startTime.tv_nsec) / 1.0e9;
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

static void statsPrint(void)
{
    const double elapsed = linkStats.elapsed;
    const double payload = (double)linkStats.payloadBytesSent +
                           (double)linkStats.payloadBytesReceived;

    // Two terminals are usually on screen during the demo; say which one this is.
    printf("\n--- Link layer statistics (%s) ---\n",
           currentRole == ROLE_TX ? "transmitter" : "receiver");
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

        // S = R / C: R is the useful payload rate in bit/s, C the channel bit
        // rate. Ten bits go on the wire per octet (8-N-1).
        if (linkParams.baudRate > 0)
            printf("  efficiency ............. %8.2f %%\n",
                   100.0 * payload * 10.0 / (elapsed * (double)linkParams.baudRate));
    }

    printf("-----------------------------\n");
}

// A command carries our own address, a reply carries the peer's, so only two of
// the frames we emit are addressed to the other side.
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

// An RR or REJ that is lost is recovered by the peer's own timeout, so a write
// failure here is reported but never fatal.
static void sendReply(unsigned char control)
{
    if (sendSupervision(control) != 0)
        fprintf(stderr, "Could not send supervision frame %s.\n",
                frameControlName(control));
}

// Which (address, control) pairs may this role accept? Anything else is a frame
// for the other direction, or noise that happened to pass BCC1.
static bool addressAccepted(unsigned char address, unsigned char control)
{
    if (currentRole == ROLE_RX)
    {
        if (address == A_TX)
            return control == C_SET || control == C_DISC ||
                   control == C_I0 || control == C_I1;

        // The UA answering the DISC the receiver sends during close.
        if (address == A_RX)
            return control == C_UA;

        return false;
    }

    if (currentRole == ROLE_TX)
    {
        if (address == A_TX)
            return control == C_UA || control == C_RR0 || control == C_RR1 ||
                   control == C_REJ0 || control == C_REJ1;

        // The DISC answering ours, and its retransmissions.
        if (address == A_RX)
            return control == C_DISC;

        return false;
    }

    return false;
}

// Read one frame, byte by byte, and classify it.
//
// The control field is examined at BCC1 time because it decides the geometry:
// supervision frames end at the next flag, information frames carry a
// de-stuffed data field followed by a BCC2 and then the flag.
//
// "packet" and *packetLen are filled on RX_OK_DATA only; *controlOut is filled
// for supervision frames and for RX_BAD_BCC2, so the caller can send the right
// REJ. It is the caller's job to size "packet" to MAX_PAYLOAD_SIZE.
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

    // A smaller buffer would turn every full-size information frame into
    // RX_IGNORED, which at the receiver is indistinguishable from a dead line.
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
                // Only our handler clears the flag, so a cleared flag means our
                // own alarm fired. Any other signal leaves it set and must not
                // fake a timeout: keep reading inside the same attempt.
                if (!isAlarmActive())
                    return RX_TIMEOUT;

                continue;
            }

            perror("readByteSerialPort");
            return RX_IO_ERROR;
        }

        // serial_port.c blocks with VMIN = 1, so a zero-length read is not
        // "nothing yet": it is end of file. Spinning on it would hang the
        // program at 100% CPU until it is killed.
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
            // A frame may be preceded by several flags. This is also what
            // resynchronises us after an error one frame early: the closing
            // flag of the damaged frame is taken as the next opening flag and
            // then ignored here.
            if (byte == FRAME_FLAG)
                break;

            address = byte;
            state = ST_READ_C;
            break;

        case ST_READ_C:
            // A flag where the control field should be opens a new frame.
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
                // Damaged header: silent discard, recovery by timeout. Nothing
                // is sent, exactly as the specification requires.
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
                // Malformed supervision frame: resynchronise on the next flag.
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
                    // 0x7D not followed by a valid escape code.
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

            // Closing flag. The last de-stuffed byte is the BCC2.
            state = ST_HUNT_FLAG;

            if (overflow || filled < 1 + FRAME_BCC2_SIZE)
            {
                // Oversized, or a header with no BCC2 at all, or a data field
                // with no packet in it. The last case is a frame llSend never
                // produces (an empty buffer is rejected there), and refusing it
                // here is what keeps "llReceive returned 0" meaning exactly one
                // thing: the timeout expired.
                return RX_IGNORED;
            }

            *controlOut = control;

            if (frameBcc2(rxBody, filled - 1) != rxBody[filled - 1])
            {
                linkStats.bcc2Errors++;
                return RX_BAD_BCC2;
            }

            *packetLen = (int)(filled - 1);

            memcpy(packet, rxBody, filled - 1);

            return RX_OK_DATA;
        }
    }
}

// Wait, inside the current alarm window, for a supervision frame.
//
// Returns 1 and stores the control field, 0 if the window expired, -1 on an
// unrecoverable I/O error.
//
// Information frames are not what the callers of this function are waiting for,
// with one exception that matters: at the receiver a duplicate must still be
// acknowledged. Dropping it would burn the transmitter's whole retry budget on
// data it has already delivered -- which is exactly what happens to a
// retransmitted END whose RR was lost. One wait function for every supervision
// exchange is what keeps that rule in a single place.
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

        // Below: information frames. The transmitter never accepts data.
        if (currentRole != ROLE_RX)
            continue;

        if (frameCGetNs(control) != rxExpected)
        {
            // Duplicate: discard the payload whatever its BCC2 says, and repeat
            // the RR for the frame we are still expecting.
            linkStats.duplicateFrames++;
            sendReply(C_RR(rxExpected));
        }
        else if (result == RX_BAD_BCC2)
        {
            // New frame, damaged payload: ask for it again.
            sendReply(C_REJ(rxExpected));
        }
        else
        {
            // A new, undamaged frame: only llReceive() accepts new data. It is
            // not acknowledged here, so the peer repeats it on its own timeout.
            continue;
        }

        // A signal can be delivered while we are writing the acknowledgement
        // rather than while blocked in read(), and the handler clears the flag
        // when that happens. The window is then over, so report it instead of
        // going back to a blocking read with no timer armed.
        if (!isAlarmActive())
            return 0;
    }
}

// Send a supervision frame and wait for the reply that answers it, repeating the
// frame up to nRetransmissions extra times.
// Returns 0 once the expected reply arrives, -1 when every attempt failed.
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

        // Anything that is not the expected reply is ignored inside the same
        // window: a stale RR, a leftover UA, a repeated DISC.
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

// Verify rr is the acknowledgement of the frame we are sending, and rej is the
// request to send it again. Both are a single comparison because Nr is one bit:
//   RR accepted our frame iff its Nr is the number after ours.
//   REJ rejects our frame iff its Nr is exactly ours.
// "Nr != ns" would give the same answer while Nr is one of two values, but it
// stops being true the moment the numbering window grows, and it reads as if
// any old RR acknowledged anything.
static AckResult waitAck(unsigned char ns)
{
    const unsigned char next = (unsigned char)(ns ^ 1u);

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
            if (frameCGetNr(control) == next)
                return ACKED;

            continue; // stale RR: the receiver is still behind us
        }

        if (frameCIsREJ(control))
        {
            if (frameCGetNr(control) == ns)
                return REJECTED;

            continue; // stale REJ
        }

        // A leftover UA or an unexpected DISC: keep listening.
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

    // nRetransmissions counts retries after the first attempt.
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

// Bounded, by requirement: (nRetransmissions + 1) timeout rounds.
int llOpenRx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened (receiver).\n", llParameters.serialPort);

    // Installed here as well as in llOpenTx: the close handshake arms an alarm
    // in both roles, so a handler must exist before the first alarm(2) call.
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
    int built = frameBuildInfo(wire, sizeof(wire), A_TX, txNs, buf, (size_t)bufSize);

    if (built < 0)
    {
        fprintf(stderr, "Could not build the information frame.\n");
        return -1;
    }

    const int attempts = linkParams.nRetransmissions + 1;

    // The frame is built once, before the loop, so a retry provably puts the
    // same bytes on the wire: same Ns, same packet, same BCC2, same stuffing.
    for (int attempt = 1; attempt <= attempts; attempt++)
    {
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

        if (ack == REJECTED)
            printf("REJ received: sending frame Ns=%u again.\n", (unsigned)txNs);
        else
            printf("Timeout waiting for RR (attempt %d of %d).\n", attempt, attempts);
    }

    fprintf(stderr, "llSend: no acknowledgement after %d attempts.\n", attempts);
    return -1;
}

// Deliver one application packet.
//
// Returns the packet length on success, 0 if nothing was delivered within one
// timeout interval (call again), -1 on an unrecoverable error. The length is
// never 0 on success: rxReadFrame refuses an information frame with an empty
// data field.
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
                // The transmitter lost our UA and is repeating SET.
                // Re-acknowledging is idempotent. It is not a duplicate
                // information frame, so it must not touch that counter: the
                // repeat shows up as "SET received = 2".
                sendReply(C_UA);
                continue;
            }

            if (control == C_DISC)
            {
                // The transmitter gave up mid-transfer. Stop receiving; the
                // application runs the close handshake, which answers the next
                // DISC the transmitter sends.
                fprintf(stderr, "DISC received during the transfer; "
                                "the transmitter has aborted.\n");
                return -1;
            }

            continue;
        }

        // An information frame, possibly with a damaged data field.
        const unsigned char ns = frameCGetNs(control);

        if (ns != rxExpected)
        {
            // Duplicate: discard the payload whatever its BCC2 says, and repeat
            // the RR for the frame we are still expecting.
            linkStats.duplicateFrames++;
            printf("Duplicate frame Ns=%u: payload dropped, RR%u repeated.\n",
                   (unsigned)ns, (unsigned)rxExpected);
            sendReply(C_RR(rxExpected));
            continue;
        }

        if (result == RX_BAD_BCC2)
        {
            // New frame, damaged payload: ask for it again, expectation
            // unchanged.
            printf("BCC2 error on frame Ns=%u: sending REJ%u.\n",
                   (unsigned)ns, (unsigned)rxExpected);
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

// Close the transmitter: DISC -> DISC -> UA.
int llCloseTx(void)
{
    if (currentRole != ROLE_TX || linkParams.timeout <= 0)
        return -1;

    int status = 0;

    statsFreeze();
    stopAlarm();

    if (sendAndWait(C_DISC, C_DISC) != 0)
    {
        fprintf(stderr, "Close handshake failed: no DISC from the receiver.\n");
        status = -1;
    }
    else if (sendSupervision(C_UA) != 0)
    {
        fprintf(stderr, "Close handshake failed: could not answer the receiver's DISC.\n");
        status = -1;
    }
    else
    {
        // Linger for up to (nRetransmissions + 1) windows. If that UA was lost
        // the receiver repeats its DISC when its own timeout expires, and each
        // repeat must be answered. The first quiet window ends the linger, so
        // the normal case costs exactly one timeout interval.
        unsigned char control = 0;
        const int windows = linkParams.nRetransmissions + 1;

        for (int i = 0; i < windows; i++)
        {
            startAlarm((unsigned)linkParams.timeout);
            int result = waitControl(&control);
            stopAlarm();

            if (result != 1)
                break; // one quiet window is enough

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

// Close the receiver: wait for DISC, answer DISC, wait for UA.
int llCloseRx(void)
{
    if (currentRole != ROLE_RX || linkParams.timeout <= 0)
        return -1;

    int status = 0;
    bool gotDisc = false;
    const int rounds = linkParams.nRetransmissions + 1;

    statsFreeze();
    stopAlarm();

    // waitControl() keeps acknowledging a retransmitted END, so an RR lost at
    // the very last packet costs the transmitter one retry, not its whole
    // budget.
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

    // The receiver answers with a DISC of its own, carrying A = 0x01, and waits
    // for the transmitter's UA -- also 0x01.
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
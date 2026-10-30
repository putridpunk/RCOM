// what actually goes on the wire, before stuffing:
//   supervision: F | A | C | BCC1 | F
//   information: F | A | C | BCC1 | app packet | BCC2 | F
//
// stop and wait. Ns goes 0,1,0,1... and an RR or REJ carries Nr = the frame rx
// wants next. tx only moves Ns when an RR comes back with a different Nr, so a
// retry sends the same number and the same bytes.
//
// TODO: efficiency numbers in statsPrint are rough, redo for the report

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

// what came out of one attempt at reading a frame
typedef enum
{
    RX_OK_SUPERVISION, // got a supervision frame
    RX_OK_DATA,        // got an I frame and its BCC2 checked out
    RX_IGNORED,        // malformed/truncated/too long, threw it away
    RX_BAD_BCC2,       // I frame but the data field failed its check
    RX_TIMEOUT,        // our alarm went off
    RX_IO_ERROR        // port died
} RxResult;

typedef enum
{
    ST_HUNT_FLAG,          // looking for the opening delimiter
    ST_READ_A,
    ST_READ_C,
    ST_READ_BCC1,
    ST_READ_SUPERVISION_F, // waiting for the closing flag
    ST_READ_DATA           // unstuffing until the closing flag
} RxState;

typedef enum
{
    ACKED,       // rx took it, bump Ns
    REJECTED,    // rx wants it again
    ACK_TIMEOUT, // nothing came back
    ACK_ERROR    // port died
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

    // counted once when the packet is queued, not once per retry, so this
    // matches the number of file bytes we handed to the link layer
    unsigned long payloadBytesSent;
    unsigned long payloadBytesReceived;

    struct timespec startTime;
    bool started;
} LinkStats;

static LinkStats linkStats;

static LinkRole currentRole = ROLE_NONE;
static LinkLayer linkParams;

static uint8_t txNs = 0;       // Ns of the next I frame (tx side)
static uint8_t rxExpected = 0; // next Ns rx will pass up (rx side)

// unstuffed body of the frame we're reading: the packet plus its BCC2
static unsigned char rxBody[MAX_PAYLOAD_SIZE + FRAME_BCC2_SIZE];

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
            // EINTR = our alarm went off mid write, doesn't mean the write
            // failed, just do it again
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

// TODO: these two are basically the same function twice, could do it with a
// table but it's not worth the bother at this point
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

        // efficiency point (report)
        if (linkParams.baudRate > 0)
            printf("  efficiency ............. %8.2f %%\n",
                   100.0 * payload * 10.0 / (elapsed * (double)linkParams.baudRate));
    }

    printf("-----------------------------\n");
}

// commands carry our own address, replies carry the peer's. the only two we
// send that are replies: tx's UA and rx's DISC
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

// best effort, if the write fails we just say so and carry on. the peer's
// timeout is what recovers, not ours
static void sendReply(unsigned char control)
{
    if (sendSupervision(control) != 0)
        fprintf(stderr, "Could not send supervision frame %s.\n",
                frameControlName(control));
}

// which (A,C) pairs this role accepts. anything else is either meant for the
// other direction or noise that happened to pass BCC1
static bool addressAccepted(unsigned char address, unsigned char control)
{
    if (currentRole == ROLE_RX)
    {
        if (address == A_TX)
            return control == C_SET || control == C_DISC ||
                   control == C_I0 || control == C_I1;

        // the UA that answers the DISC we send when closing
        if (address == A_RX)
            return control == C_UA;

        return false;
    }

    if (currentRole == ROLE_TX)
    {
        if (address == A_TX)
            return control == C_UA || control == C_RR0 || control == C_RR1 ||
                   control == C_REJ0 || control == C_REJ1;

        // the DISC rx answers with, and its retransmissions
        if (address == A_RX)
            return control == C_DISC;

        return false;
    }

    return false;
}

// reads one frame byte by byte and says what it was. hunt for the flag, take A,
// C and BCC1, then C decides the rest: supervision ends at the next flag, an I
// frame has an unstuffed data field, then BCC2, then the flag.
//
// packet and *packetLen only get filled on RX_OK_DATA. on RX_BAD_BCC2 we drop
// the payload but still hand back C so the caller knows which REJ to send
static RxResult rxReadFrame(unsigned char *packet, size_t packetCapacity,
                            int *packetLen, unsigned char *controlOut)
{
    RxState state = ST_HUNT_FLAG;

    unsigned char address = 0;
    unsigned char control = 0;

    size_t filled = 0;     // unstuffed bytes sitting in rxBody
    bool overflow = false; // data field didn't fit in rxBody
    bool escape = false;   // the byte before was the escape

    if (packet == NULL || packetLen == NULL || controlOut == NULL)
        return RX_IO_ERROR;

    // if the buffer is too small for a full frame we'd silently turn everything
    // into RX_IGNORED and it'd look like a dead line
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
                // our handler is the only thing that clears the flag, so a
                // cleared flag means it was our alarm. anything else
                // interrupted us, keep reading in the same window
                if (!isAlarmActive())
                    return RX_TIMEOUT;

                continue;
            }

            perror("readByteSerialPort");
            return RX_IO_ERROR;
        }

        // serial_port.c blocks with VMIN=1 so 0 bytes here is EOF
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
            // a bunch of flags in a row is legal, keep skipping
            if (byte == FRAME_FLAG)
                break;

            address = byte;
            state = ST_READ_C;
            break;

        case ST_READ_C:
            // flag where C should be, so this is a new frame
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
                // header is damaged, drop it quietly. the peer's timeout sorts
                // it out
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
                // not a flag where it should be, give up and resync
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
                    // 7D followed by something that isn't a valid escape
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

            // closing flag. last unstuffed byte is the BCC2
            state = ST_HUNT_FLAG;

            if (overflow || filled < 1)
            {
                // too long, or a header with no BCC2 at all
                return RX_IGNORED;
            }

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

// waits for a supervision frame inside the alarm window we already armed.
// 1 + the control field, 0 if the alarm fired first, -1 on an I/O error.
//
// I frames aren't what the callers want here, except at rx where a duplicate
// still needs an RR. if we dropped it, tx would burn its whole retry budget on
// data that already arrived, and that's what happens to a retransmitted END when
// its RR gets lost
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

        // everything from here down is an I frame. tx never accepts data so
        // this is rx's problem only
        if (currentRole != ROLE_RX)
            continue;

        if (frameCGetNs(control) != rxExpected)
        {
            // duplicate, bin the payload whatever its BCC2 says and repeat the
            // RR for the one we're still waiting for
            sendReply(C_RR(rxExpected));
            continue;
        }

        if (result == RX_BAD_BCC2)
        {
            // new frame but the payload is damaged, ask again
            sendReply(C_REJ(rxExpected));
            continue;
        }

        // new and undamaged, leave it alone here. only llReceive() passes data
        // up to the app
    }
}

// send a supervision frame and wait for the reply that answers it, with up to
// nRetransmissions extra goes. 0 once we get it, -1 when we've run out
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

        // ignore anything that isn't the reply we asked for, in the same
        // window. stale RR, leftover UA, repeated DISC, all the same to us
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

// what happened to the I frame we just sent
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

        // RR with Nr != Ns means rx moved on and took it. REJ with Nr == Ns
        // means send it again. anything else is stale and doesn't end the
        // window
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

        // leftover UA or a DISC we didn't ask for
    }
}

// open the port as tx and do the SET/UA handshake
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

    // nTries counts retries AFTER the first go, so nTries=3 means at most 4 SET
    // frames on the wire
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

// open the port as rx and wait for SET. the spec says bounded by
// nRetransmissions+1 rounds, with main.c that's 4 rounds of 4s so tx has 16s
// from when we start
int llOpenRx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened (receiver).\n", llParameters.serialPort);

    // needed here too, the close handshake arms an alarm on both sides and the
    // handler has to exist before the first alarm(2)
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

// put one packet in an I frame and wait for the ack. bufSize on success, -1 on
// error or when we've used up the retries
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

        // timeout or a REJ, both end up back here with the same Ns so we just
        // rebuild and send it again
    }

    fprintf(stderr, "llSend: no acknowledgement after %d attempts.\n", attempts);
    return -1;
}

// pass one packet up to the app. returns the length, or 0 if nothing arrived in
// one timeout (call again), or -1 on a real error
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
                // tx missed our UA, send it again. harmless, and its not a
                // duplicate I frame so duplicateFrames stays where it is
                sendReply(C_UA);
                continue;
            }

            if (control == C_DISC)
            {
                // tx gave up half way through. stop receiving and let the app
                // run the close handshake, which will answer the DISC it sends
                // again
                fprintf(stderr, "DISC received during the transfer; "
                                "the transmitter has aborted.\n");
                return -1;
            }

            continue;
        }

        // an I frame, maybe with a damaged payload
        if (frameCGetNs(control) != rxExpected)
        {
            linkStats.duplicateFrames++;
            sendReply(C_RR(rxExpected));
            continue;
        }

        if (result == RX_BAD_BCC2)
        {
            // new frame, bad payload, ask for it again. expectation doesn't
            // move
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

// close tx: DISC -> DISC -> UA
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
        // the UA carries the address of the DISC we just got
        status = -1;
    }
    else
    {
        // hang around for up to nTries+1 windows. if our UA got lost rx will
        // repeat its DISC when its own timeout expires and each repeat needs an
        // answer. first quiet window ends this so normally it's just one
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

// close rx: wait for DISC, answer DISC, wait for UA
int llCloseRx(void)
{
    if (currentRole != ROLE_RX || linkParams.timeout <= 0)
        return -1;

    int status = 0;
    bool gotDisc = false;
    const int rounds = linkParams.nRetransmissions + 1;

    stopAlarm();

    // 1. wait for tx's DISC. waitControl keeps acking a retransmitted END, so a
    //    lost RR for the last packet only costs tx one retry instead of its
    //    whole budget
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

    // 2. answer with DISC (A = 0x01) and wait for tx's UA
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
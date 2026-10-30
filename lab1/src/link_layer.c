// Link layer implementation.
//
// On the wire, before stuffing:
//   supervision : F | A | C | BCC1 | F
//   information : F | A | C | BCC1 | application packet | BCC2 | F
//
// Only the transmitter sends information frames. Stop-and-Wait with Ns
// alternating 0/1; an RR or REJ carries Nr, the frame the receiver expects next.
// Ns moves only on an RR carrying a different Nr, so a retry repeats the same
// number and the same bytes.
//
// Every supervision exchange in here is the same three steps: send, arm the
// alarm, wait for the reply. That's sendAndWait() and waitAck() below. The rest
// of the file is a few lines wrapped around them.

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

// ---- Connection state ----

typedef enum
{
    ROLE_NONE = 0,
    ROLE_TX,
    ROLE_RX
} LinkRole;

// Outcome of one attempt at reading a frame from the serial port.
typedef enum
{
    RX_OK_SUPERVISION, // a complete, valid supervision frame
    RX_OK_DATA,        // a complete information frame with a valid BCC2
    RX_IGNORED,        // malformed, truncated or oversized frame; discarded
    RX_BAD_BCC2,       // information frame whose data field failed its check
    RX_TIMEOUT,        // the read was interrupted by our own alarm
    RX_IO_ERROR        // unrecoverable error on the serial port
} RxResult;

// States of the frame reception machine.
typedef enum
{
    ST_HUNT_FLAG,          // look for the opening delimiter
    ST_READ_A,             // address field
    ST_READ_C,             // control field
    ST_READ_BCC1,          // header check character
    ST_READ_SUPERVISION_F, // closing flag of a supervision frame
    ST_READ_DATA           // de-stuffed data field, up to the closing flag
} RxState;

// Verdict on an information frame that has just been transmitted.
typedef enum
{
    ACKED,       // the receiver took it: advance Ns
    REJECTED,    // the receiver wants it again: retransmit now
    ACK_TIMEOUT, // no reply inside the window: retransmit
    ACK_ERROR    // the serial port failed
} AckResult;

// Counters for one connection, which is why they live here rather than in the
// pure frame module.
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

    // Payload counted once per packet queued, not once per retry, so
    // payloadBytesSent matches the number of file bytes handed to the link
    // layer.
    unsigned long payloadBytesSent;
    unsigned long payloadBytesReceived;

    struct timespec startTime;
    bool started;
} LinkStats;

static LinkStats linkStats;

static LinkRole currentRole = ROLE_NONE;
static LinkLayer linkParams;

static uint8_t txNs = 0;       // Ns of the next information frame (Tx side)
static uint8_t rxExpected = 0; // next Ns the receiver will deliver (Rx side)

// De-stuffed body of the frame in progress: the application packet plus its
// trailing BCC2, which is the one extra byte.
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

// ---- Statistics ----

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

        // S = R/C for the efficiency plot in the report. Ten bits per octet go
        // on the wire (8-N-1), so R in bit/s is payload * 10 / elapsed.
        if (linkParams.baudRate > 0)
            printf("  efficiency ............. %8.2f %%\n",
                   100.0 * payload * 10.0 / (elapsed * (double)linkParams.baudRate));
    }

    printf("-----------------------------\n");
}

// ---- Supervision frames ----

// A command carries our own address, a reply carries the address of the command
// it answers. Only two frames we send are replies: the transmitter's UA and the
// receiver's DISC.
static unsigned char supervisionAddr(unsigned char control)
{
    if (currentRole == ROLE_TX)
        return (control == C_UA) ? A_RX : A_TX;

    if (currentRole == ROLE_RX)
        return (control == C_DISC) ? A_RX : A_TX;

    return A_TX;
}

// Returns 0 on success, -1 on failure.
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

// An RR, a REJ or a repeated UA. If the write fails we say so and carry on: the
// peer's timeout is what recovers, not ours.
static void sendReply(unsigned char control)
{
    if (sendSupervision(control) != 0)
        fprintf(stderr, "Could not send supervision frame %s.\n",
                frameControlName(control));
}

// ---- Frame reception ----

// Which (address, control) pairs this role is allowed to accept. Anything else
// belongs to the other direction, or is noise that happened to pass BCC1.
static bool addressAccepted(unsigned char address, unsigned char control)
{
    if (currentRole == ROLE_RX)
    {
        if (address == A_TX)
            return control == C_SET || control == C_DISC ||
                   control == C_I0 || control == C_I1;

        // The UA that answers the DISC the receiver sends during close.
        if (address == A_RX)
            return control == C_UA;

        return false;
    }

    if (currentRole == ROLE_TX)
    {
        if (address == A_TX)
            return control == C_UA || control == C_RR0 || control == C_RR1 ||
                   control == C_REJ0 || control == C_REJ1;

        // The DISC the receiver answers with, and its retransmissions.
        if (address == A_RX)
            return control == C_DISC;

        return false;
    }

    return false;
}

// Read one frame, byte by byte, and classify it.
//
// Hunt for the opening flag, then take A, C and BCC1. The filtering happens at
// BCC1 because C decides what follows: a supervision frame ends at the next
// flag, an information frame has a de-stuffed data field, then a BCC2, then the
// flag.
//
// packet and *packetLen are filled on RX_OK_DATA only. On RX_BAD_BCC2 the
// payload is dropped but *controlOut still gets the control field, so the caller
// can pick the right REJ.
static RxResult rxReadFrame(unsigned char *packet, size_t packetCapacity,
                            int *packetLen, unsigned char *controlOut)
{
    RxState state = ST_HUNT_FLAG;

    unsigned char address = 0;
    unsigned char control = 0;

    size_t filled = 0;     // de-stuffed bytes sitting in rxBody
    bool overflow = false; // the data field didn't fit in rxBody
    bool escape = false;   // the previous byte was the escape octet

    if (packet == NULL || packetLen == NULL || controlOut == NULL)
        return RX_IO_ERROR;

    // A smaller buffer turns full-size frames into RX_IGNORED, which at the
    // receiver looks the same as a dead line.
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
                // Our handler is the only thing that clears the flag, so a
                // cleared flag means our own alarm fired. Anything else
                // interrupted us and we keep reading inside the same window.
                if (!isAlarmActive())
                    return RX_TIMEOUT;

                continue;
            }

            perror("readByteSerialPort");
            return RX_IO_ERROR;
        }

        // serial_port.c blocks with VMIN = 1, so a zero-length read is end of
        // file, not "nothing yet". Spinning on it would pin the CPU until
        // someone kills the process.
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
            // Several opening flags in a row are legal.
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
                // Damaged header: drop it silently, the peer's timeout recovers.
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
                // Malformed supervision frame: resync on the next flag.
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
                    // 0x7D with nothing valid after it.
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

            if (overflow || filled < 1)
            {
                // Oversized, or a header with no BCC2 at all.
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

// ---- Retry primitives ----

// Wait for a supervision frame inside the current alarm window.
//
// Returns 1 with the control field, 0 if the alarm fired first, -1 on an I/O
// error.
//
// Information frames aren't what the callers are after, with one exception: at
// the receiver a duplicate still has to be acknowledged. Dropping it burns the
// sender's whole retry budget on data that already arrived. That is exactly
// what happens to a retransmitted END when its RR goes missing.
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

        // Everything below is an information frame. The transmitter never
        // accepts data, so this branch is the receiver's business only.
        if (currentRole != ROLE_RX)
            continue;

        if (frameCGetNs(control) != rxExpected)
        {
            // Duplicate. Throw the payload away whatever its BCC2 says, and
            // repeat the RR for the frame we're still expecting.
            sendReply(C_RR(rxExpected));
            continue;
        }

        if (result == RX_BAD_BCC2)
        {
            // New frame with a damaged payload: ask for it again.
            sendReply(C_REJ(rxExpected));
            continue;
        }

        // A new, undamaged frame is left alone here. Only llReceive() hands data
        // to the application. The transmitter's timeout brings the frame back,
        // and the close handshake sees the repeat.
    }
}

// Send a supervision frame and wait for the reply that answers it, retrying up
// to nRetransmissions extra times. Returns 0 once the expected reply arrives,
// -1 when every attempt ran out.
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

        // Anything that isn't the expected reply gets ignored inside the same
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

// Verdict on the information frame just sent.
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

        // Modulo-2 numbering turns both cases into one comparison:
        //   RR with Nr != Ns  -> the receiver moved on, frame accepted
        //   REJ with Nr == Ns -> the receiver wants it again
        // Anything else is stale and doesn't end the window.
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

        // A leftover UA, or a DISC we didn't ask for. Keep listening.
    }
}

// ---- LLOPEN ----

// Open the port as transmitter and run the SET/UA handshake.
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

    // nRetransmissions counts retries after the first try, so nTries = 3 means
    // at most four SET frames on the wire.
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

// Open the port as receiver and wait for SET.
//
// Bounded by requirement: (nRetransmissions + 1) rounds. With main.c's values
// that's four rounds of four seconds, so the transmitter has a sixteen second
// window to show up after we start.
int llOpenRx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened (receiver).\n", llParameters.serialPort);

    // Also installed here: the close handshake arms an alarm in both roles, and
    // a handler has to exist before the first alarm(2) call.
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

// ---- LLSEND ----

// Send one packet inside an information frame and wait for the acknowledgement.
// Returns bufSize on success, -1 on error or when the retries run out.
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

        // ACK_TIMEOUT and REJECTED both land here: same number, same bytes.
    }

    fprintf(stderr, "llSend: no acknowledgement after %d attempts.\n", attempts);
    return -1;
}

// ---- LLRECEIVE ----

// Hand one packet to the application.
//
// Returns the packet length, or 0 if nothing arrived within one timeout interval
// (call again), or -1 on an unrecoverable error.
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
                // The transmitter missed our UA and is repeating SET. Sending it
                // again is harmless. This is not a duplicate information frame,
                // so duplicateFrames stays put; the repeat shows up as a second
                // "SET received" instead.
                sendReply(C_UA);
                continue;
            }

            if (control == C_DISC)
            {
                // The transmitter gave up mid-transfer. Stop receiving and let
                // the application run the close handshake, which answers the
                // DISC it sends again.
                fprintf(stderr, "DISC received during the transfer; "
                                "the transmitter has aborted.\n");
                return -1;
            }

            continue;
        }

        // An information frame, possibly with a damaged data field.
        if (frameCGetNs(control) != rxExpected)
        {
            // Duplicate. Drop the payload whatever its BCC2 says, and repeat the
            // RR for the frame we're still expecting.
            linkStats.duplicateFrames++;
            sendReply(C_RR(rxExpected));
            continue;
        }

        if (result == RX_BAD_BCC2)
        {
            // New frame, damaged payload: ask for it again, expectation
            // unchanged.
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

// ---- LLCLOSE ----

// Close the transmitter: DISC -> DISC -> UA.
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
        // The UA carries the address of the DISC we just received.
        status = -1;
    }
    else
    {
        // Linger for up to (nRetransmissions + 1) windows. If our UA was lost
        // the receiver repeats its DISC when its own timeout expires, and every
        // repeat needs an answer. The first quiet window ends this, so the
        // normal case costs exactly one interval.
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

// Close the receiver: wait for DISC, answer DISC, wait for UA.
int llCloseRx(void)
{
    if (currentRole != ROLE_RX || linkParams.timeout <= 0)
        return -1;

    int status = 0;
    bool gotDisc = false;
    const int rounds = linkParams.nRetransmissions + 1;

    stopAlarm();

    // 1. Wait for the transmitter's DISC. waitControl() keeps acknowledging a
    //    retransmitted END, so a lost RR for the last packet costs the
    //    transmitter one retry instead of its whole budget.
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

    // 2. Answer with DISC (A = 0x01) and wait for the transmitter's UA.
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
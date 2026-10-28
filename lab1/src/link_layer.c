// Link layer protocol implementation.
//
// Frames on the wire, before stuffing:
//   supervision : F | A | C | BCC1 | F
//   information : F | A | C | BCC1 | application packet | BCC2 | F
//
// Only the transmitter sends information frames. The transfer is
// Stop-and-Wait with alternating Ns (0/1); an RR or REJ carries Nr, the next
// information frame the receiver expects. Ns advances only when an RR with
// Nr == !Ns arrives, so every retry repeats the same number and the same
// packet bytes.

#include "link_layer.h"
#include "serial_port.h"
#include "alarm_sigaction.h"
#include "frame.h"

#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// After its final UA the transmitter keeps the port open for one timeout
// interval so that a repeated DISC (the receiver never saw that UA) can be
// answered. Set to 0 to skip it; closing then costs no extra time but a lost
// final UA leaves the receiver waiting for its full retry budget.
#define TX_CLOSE_LINGER 1

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
    RX_TIMEOUT,        // the read was interrupted by the alarm
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

static LinkStats linkStats;

static LinkRole currentRole = ROLE_NONE;
static LinkLayer linkParams;

static uint8_t txNs = 0;       // Ns of the next information frame (Tx side)
static uint8_t rxExpected = 0; // next Ns the receiver expects to deliver (Rx side)

// De-stuffed body of the frame being received: the application packet plus its
// trailing BCC2. One byte larger than MAX_PAYLOAD_SIZE for exactly that reason.
static unsigned char rxBody[MAX_PAYLOAD_SIZE + FRAME_BCC2_SIZE];

////////////////////////////////////////////////
// Serial I/O helpers
////////////////////////////////////////////////

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

////////////////////////////////////////////////
// Supervision frames
////////////////////////////////////////////////

// Address of a supervision frame, by who sends it and what it is.
static unsigned char supervisionAddr(LinkRole sender, unsigned char control)
{
    if (sender == ROLE_TX)
    {
        // Tx sends SET and DISC with A_TX; its UA answers the receiver's DISC,
        // so it carries the address of that DISC.
        return (control == C_UA) ? A_RX : A_TX;
    }

    if (sender == ROLE_RX)
    {
        // Rx sends UA, RR and REJ with A_TX; its DISC answers the
        // transmitter's DISC, so it carries the address of that DISC.
        return (control == C_DISC) ? A_RX : A_TX;
    }

    return A_TX;
}

// Send a supervision frame. Returns 0 on success, -1 on failure.
static int sendSupervision(unsigned char control)
{
    unsigned char frame[SUPERVISION_FRAME_SIZE];
    unsigned char address = supervisionAddr(currentRole, control);

    int built = frameBuildSupervision(frame, sizeof(frame), address, control);
    if (built != SUPERVISION_FRAME_SIZE)
        return -1;

    if (writeAll(frame, (size_t)built) != 0)
        return -1;

    linkStatsCountSupervisionSent(&linkStats, control);
    return 0;
}

// Send a supervision frame that is part of normal operation (an RR, a REJ, a
// repeated UA). A write failure here is reported but never fatal: the peer's
// own timeout drives recovery.
static void sendReply(unsigned char control)
{
    if (sendSupervision(control) != 0)
        fprintf(stderr, "Could not send supervision frame 0x%02X.\n",
                (unsigned int)control);
}

////////////////////////////////////////////////
// LLCONNECT
////////////////////////////////////////////////

// Which (address, control) pairs may this role accept? Anything else is a
// frame for the other direction, or noise that happened to pass its BCC1.
static int addressAccepted(unsigned char address, unsigned char control)
{
    if (currentRole == ROLE_RX)
    {
        if (address == A_TX)
            return control == C_SET || control == C_DISC ||
                   control == C_I0 || control == C_I1;

        // The UA that answers the DISC the receiver sent during close.
        if (address == A_RX)
            return control == C_UA;

        return FALSE;
    }

    if (currentRole == ROLE_TX)
    {
        if (address == A_TX)
            return control == C_UA || control == C_RR0 || control == C_RR1 ||
                   control == C_REJ0 || control == C_REJ1;

        // The DISC the receiver answers with, and its retransmissions.
        if (address == A_RX)
            return control == C_DISC;

        return FALSE;
    }

    return FALSE;
}

////////////////////////////////////////////////
// Frame reception
////////////////////////////////////////////////

// Read one frame from the serial port, byte by byte, and classify it.
//
// The machine hunts an opening flag, then consumes address, control and BCC1.
// It validates and filters at BCC1 time, because the control field decides the
// frame geometry: supervision frames end at the next flag, information frames
// carry a de-stuffed data field followed by a BCC2 and then the flag.
//
// On RX_OK_DATA the application packet is copied to "packet" if it fits in
// "packetCapacity"; "*packetLen" receives its length. On RX_BAD_BCC2 the packet
// is not copied but "*packetLen" and the control field are still reported, so
// the caller can send the right REJ. "*addressOut" and "*controlOut" are set
// for every supervision and data frame.
static RxResult rxReadFrame(unsigned char *packet, size_t packetCapacity,
                            int *packetLen, unsigned char *addressOut,
                            unsigned char *controlOut)
{
    RxState state = ST_HUNT_FLAG;

    unsigned char address = 0;
    unsigned char control = 0;

    size_t filled = 0;    // de-stuffed bytes held in rxBody
    int overflow = FALSE; // the data field did not fit in rxBody
    int escape = FALSE;   // the previous byte was the escape octet

    if (packet == NULL || packetLen == NULL || addressOut == NULL ||
        controlOut == NULL || packetCapacity < 3)
        return RX_IO_ERROR;

    *packetLen = 0;
    *addressOut = 0;
    *controlOut = 0;

    for (;;)
    {
        unsigned char byte = 0;
        int result = readByteSerialPort(&byte);

        if (result < 0)
        {
            if (errno == EINTR)
            {
                // The alarm handler clears the flag; a signal from any other
                // source leaves it set and must not fake a timeout.
                if (isAlarmActive() == FALSE)
                    return RX_TIMEOUT;

                continue;
            }

            perror("readByteSerialPort");
            return RX_IO_ERROR;
        }

        // No byte was consumed: keep the parser state and wait for the next
        // one. Never re-use the previous "byte" value.
        if (result == 0)
            continue;

        switch (state)
        {
        case ST_HUNT_FLAG:
            if (byte == FRAME_FLAG)
                state = ST_READ_A;
            break;

        case ST_READ_A:
            // Several opening flags may precede a frame.
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
                // Damaged header: silent discard, recovery by timeout.
                linkStats.bcc1Errors++;
                state = ST_HUNT_FLAG;
                break;
            }

            if (addressAccepted(address, control) == FALSE)
            {
                linkStats.addressErrors++;
                state = ST_HUNT_FLAG;
                break;
            }

            if (frameCIsInfo(control))
            {
                filled = 0;
                overflow = FALSE;
                escape = FALSE;
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

            packet[0] = FRAME_FLAG;
            packet[1] = address;
            packet[2] = control;
            packet[3] = frameBcc1(address, control);
            packet[4] = FRAME_FLAG;

            *packetLen = SUPERVISION_FRAME_SIZE;
            *addressOut = address;
            *controlOut = control;

            linkStatsCountSupervisionReceived(&linkStats, control);
            return RX_OK_SUPERVISION;

        case ST_READ_DATA:
            if (escape)
            {
                escape = FALSE;

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
                    overflow = TRUE;
                break;
            }

            if (byte == FRAME_ESCAPE)
            {
                escape = TRUE;
                break;
            }

            if (byte != FRAME_FLAG)
            {
                if (filled < sizeof(rxBody))
                    rxBody[filled++] = byte;
                else
                    overflow = TRUE;
                break;
            }

            // Closing flag. The last de-stuffed byte is the BCC2.
            state = ST_HUNT_FLAG;

            if (overflow == TRUE || filled < 1)
            {
                // Oversized, or a header with no BCC2 at all.
                return RX_IGNORED;
            }

            *addressOut = address;
            *controlOut = control;
            *packetLen = (int)(filled - 1);

            if (frameBcc2(rxBody, filled - 1) != rxBody[filled - 1])
            {
                linkStats.bcc2Errors++;
                return RX_BAD_BCC2;
            }

            if ((size_t)(filled - 1) > packetCapacity)
                return RX_IGNORED;

            if (filled > 1)
                memcpy(packet, rxBody, filled - 1);

            return RX_OK_DATA;
        }
    }
}

// Wait for any supervision frame. Returns 1 and stores the control field on
// success, 0 if the alarm fired first, -1 on an unrecoverable I/O error.
static int waitSupervisionControl(unsigned char *controlOut)
{
    unsigned char frame[SUPERVISION_FRAME_SIZE];

    for (;;)
    {
        int length = 0;
        unsigned char address = 0;

        RxResult result = rxReadFrame(frame, sizeof(frame), &length, &address, controlOut);

        switch (result)
        {
        case RX_OK_SUPERVISION:
            return 1;

        case RX_TIMEOUT:
            return 0;

        case RX_IO_ERROR:
            return -1;

        default:
            // Data frames, rejected payloads and junk are not our business
            // here; keep waiting inside the same attempt.
            break;
        }
    }
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////

// Open the serial port as transmitter and perform the SET/UA handshake.
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

    linkParams = llParameters;
    currentRole = ROLE_TX;
    txNs = 0;
    rxExpected = 0;
    linkStatsReset(&linkStats);
    resetAlarmCount();

    if (llParameters.timeout <= 0)
    {
        fprintf(stderr, "Invalid timeout (%d s).\n", llParameters.timeout);
        closeSerialPort();
        currentRole = ROLE_NONE;
        return -1;
    }

    // nRetransmissions counts retries after the first attempt: nTries = 3
    // means at most four SET frames on the wire.
    const int totalAttempts = llParameters.nRetransmissions + 1;

    for (int attempt = 1; attempt <= totalAttempts; attempt++)
    {
        unsigned char control = 0;

        if (sendSupervision(C_SET) != 0)
        {
            stopAlarm();
            closeSerialPort();
            currentRole = ROLE_NONE;
            return -1;
        }

        if (attempt > 1)
            linkStats.retransmissions++;

        startAlarm((unsigned)llParameters.timeout);

        int timedOut = FALSE;
        int failed = FALSE;

        for (;;)
        {
            int result = waitSupervisionControl(&control);

            if (result < 0)
            {
                failed = TRUE;
                break;
            }

            if (result == 0)
            {
                linkStats.timeouts++;
                timedOut = TRUE;
                break;
            }

            if (control == C_UA)
            {
                stopAlarm();
                printf("UA received. Connection established.\n");
                return 0;
            }

            // A stray DISC or a second UA: keep waiting inside this attempt.
        }

        stopAlarm();

        if (failed == TRUE)
        {
            closeSerialPort();
            currentRole = ROLE_NONE;
            return -1;
        }

        if (timedOut == TRUE)
            printf("Timeout waiting for UA (attempt %d of %d).\n", attempt, totalAttempts);
    }

    closeSerialPort();
    currentRole = ROLE_NONE;
    fprintf(stderr, "Maximum number of retransmissions reached. Connection failed.\n");
    return -1;
}

// Open the serial port as receiver and wait for a SET frame.
//
// The wait is bounded, by requirement: (nRetransmissions + 1) timeout rounds.
// With the values in main.c that is four rounds of four seconds, so the
// transmitter has a sixteen second window to show up after the receiver
// starts.
int llOpenRx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened (receiver).\n", llParameters.serialPort);

    // Installed here as well as in llOpenTx: the close handshake arms an alarm
    // in both roles, and a handler must exist before the first alarm(2) call.
    if (setupAlarmHandler() == -1)
    {
        perror("setupAlarmHandler");
        closeSerialPort();
        return -1;
    }

    linkParams = llParameters;
    currentRole = ROLE_RX;
    txNs = 0;
    rxExpected = 0;
    stopAlarm();
    resetAlarmCount();
    linkStatsReset(&linkStats);

    if (llParameters.timeout <= 0)
    {
        fprintf(stderr, "Invalid timeout (%d s).\n", llParameters.timeout);
        closeSerialPort();
        currentRole = ROLE_NONE;
        return -1;
    }

    const int totalRounds = llParameters.nRetransmissions + 1;
    int round = 1;

    startAlarm((unsigned)llParameters.timeout);

    while (round <= totalRounds)
    {
        unsigned char control = 0;
        int result = waitSupervisionControl(&control);

        if (result < 0)
        {
            stopAlarm();
            closeSerialPort();
            currentRole = ROLE_NONE;
            return -1;
        }

        if (result == 0)
        {
            linkStats.timeouts++;
            printf("Timeout waiting for SET (round %d of %d).\n", round, totalRounds);
            round++;

            if (round <= totalRounds)
                startAlarm((unsigned)llParameters.timeout);

            continue;
        }

        if (control == C_SET)
        {
            printf("SET received. Sending UA.\n");

            if (sendSupervision(C_UA) != 0)
            {
                stopAlarm();
                closeSerialPort();
                currentRole = ROLE_NONE;
                return -1;
            }

            stopAlarm();
            return 0;
        }

        // A DISC from a transmitter that gave up, or a stray UA. Wait for SET
        // without spending one of the rounds.
        printf("Ignoring supervision frame 0x%02X while waiting for SET.\n",
               (unsigned int)control);
    }

    stopAlarm();
    closeSerialPort();
    currentRole = ROLE_NONE;
    fprintf(stderr, "No SET received. Connection failed.\n");
    return -1;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////

// Send one application packet inside an information frame and wait for the
// acknowledgement.
//
// Returns bufSize on success, -1 on error or exhausted retries.
int llSend(const unsigned char *buf, int bufSize)
{
    if (buf == NULL || bufSize <= 0 || bufSize > MAX_PAYLOAD_SIZE)
        return -1;

    if (currentRole != ROLE_TX || linkParams.timeout <= 0)
        return -1;

    unsigned char wire[MAX_FRAME_SIZE];
    const int totalAttempts = linkParams.nRetransmissions + 1;

    for (int attempt = 1; attempt <= totalAttempts; attempt++)
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

        int failed = FALSE;

        for (;;)
        {
            unsigned char control = 0;
            int result = waitSupervisionControl(&control);

            if (result < 0)
            {
                failed = TRUE;
                break;
            }

            if (result == 0)
            {
                linkStats.timeouts++;
                break; // retransmit the same frame
            }

            if (frameCIsRR(control))
            {
                unsigned char nr = frameCGetNr(control) ? 1u : 0u;

                if (nr == (unsigned char)(txNs ^ 1u))
                {
                    stopAlarm();
                    txNs = (uint8_t)(txNs ^ 1u);
                    return bufSize;
                }

                // Stale RR: the receiver is still waiting for the frame we
                // already sent. Nothing to do but keep listening.
                continue;
            }

            if (frameCIsREJ(control))
            {
                unsigned char nr = frameCGetNr(control) ? 1u : 0u;

                if (nr == txNs)
                    break; // retransmit immediately

                continue;
            }

            // UA left over from the open handshake, or anything else.
        }

        stopAlarm();

        if (failed == TRUE)
            return -1;
    }

    fprintf(stderr, "llSend: no acknowledgement after %d attempts.\n", totalAttempts);
    return -1;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////

// Deliver one application packet.
//
// Returns the packet length on success, 0 if nothing was delivered within one
// timeout interval (call again), -1 on an unrecoverable error.
int llReceive(unsigned char *buf)
{
    if (buf == NULL || currentRole != ROLE_RX || linkParams.timeout <= 0)
        return -1;

    for (;;)
    {
        unsigned char address = 0;
        unsigned char control = 0;
        int length = 0;

        startAlarm((unsigned)linkParams.timeout);
        RxResult result = rxReadFrame(buf, MAX_PAYLOAD_SIZE, &length, &address, &control);
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
                // The transmitter lost our UA and is re-sending SET.
                // Re-acknowledging is idempotent.
                linkStats.duplicateFrames++;
                sendReply(C_UA);
                continue;
            }

            if (control == C_DISC)
            {
                printf("DISC received while receiving data; ignored.\n");
                continue;
            }

            continue;
        }

        // An information frame, possibly with a damaged payload.
        unsigned char ns = frameCGetNs(control) ? 1u : 0u;

        if (ns != rxExpected)
        {
            // Duplicate: discard the payload whatever its BCC2 says and repeat
            // the RR for the frame we are still expecting.
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

        if (length < 0 || length > MAX_PAYLOAD_SIZE)
            continue;

        rxExpected = (uint8_t)(rxExpected ^ 1u);
        sendReply(C_RR(rxExpected));

        linkStats.infoReceived++;
        linkStats.payloadBytesReceived += (unsigned long)length;

        return length;
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////

// Close the transmitter: DISC -> DISC -> UA.
int llCloseTx(void)
{
    stopAlarm();

    if (currentRole != ROLE_TX || linkParams.timeout <= 0)
        return -1;

    const int totalAttempts = linkParams.nRetransmissions + 1;
    unsigned char control = 0;
    int gotDisc = FALSE;
    int status = 0;

    // 1. Send DISC and wait for the receiver's DISC.
    for (int attempt = 1; attempt <= totalAttempts && gotDisc == FALSE; attempt++)
    {
        if (sendSupervision(C_DISC) != 0)
        {
            status = -1;
            break;
        }

        if (attempt > 1)
            linkStats.retransmissions++;

        startAlarm((unsigned)linkParams.timeout);

        for (;;)
        {
            int result = waitSupervisionControl(&control);

            if (result < 0)
            {
                status = -1;
                break;
            }

            if (result == 0)
            {
                linkStats.timeouts++;
                break;
            }

            if (control == C_DISC)
            {
                gotDisc = TRUE;
                break;
            }
        }

        stopAlarm();

        if (status < 0)
            break;
    }

    if (status < 0 || gotDisc == FALSE)
    {
        stopAlarm();
        if (closeSerialPort() < 0)
            perror("closeSerialPort");
        linkStatsPrint(&linkStats);
        currentRole = ROLE_NONE;
        fprintf(stderr, "Close handshake failed: no DISC from the receiver.\n");
        return -1;
    }

    // 2. Answer with UA carrying the address of the DISC we are answering.
    if (sendSupervision(C_UA) != 0)
        status = -1;

#if TX_CLOSE_LINGER
    // 3. Stay around for one timeout interval. If that UA was lost the
    //    receiver re-sends DISC when its own timeout expires, and each DISC
    //    seen here restarts the window, so a peer that keeps retrying keeps
    //    being answered.
    if (status == 0)
    {
        int resends = 0;

        for (;;)
        {
            startAlarm((unsigned)linkParams.timeout);
            int result = waitSupervisionControl(&control);
            stopAlarm();

            if (result <= 0)
                break; // timeout, or a port error we cannot recover from

            if (control != C_DISC)
                continue;

            if (resends >= linkParams.nRetransmissions)
                break;

            resends++;

            if (sendSupervision(C_UA) != 0)
                break;
        }
    }
#endif

    stopAlarm();

    if (closeSerialPort() < 0)
        perror("closeSerialPort");

    linkStatsPrint(&linkStats);
    currentRole = ROLE_NONE;
    printf("Serial port closed (transmitter).\n");

    return (status == 0) ? 0 : -1;
}

// Close the receiver: wait for DISC, answer DISC, wait for UA.
int llCloseRx(void)
{
    stopAlarm();

    if (currentRole != ROLE_RX || linkParams.timeout <= 0)
        return -1;

    const int totalAttempts = linkParams.nRetransmissions + 1;
    unsigned char control = 0;
    int gotDisc = FALSE;
    int gotUa = FALSE;
    int status = 0;

    // 1. Wait for the transmitter's DISC.
    for (int attempt = 1; attempt <= totalAttempts && gotDisc == FALSE; attempt++)
    {
        startAlarm((unsigned)linkParams.timeout);

        for (;;)
        {
            int result = waitSupervisionControl(&control);

            if (result < 0)
            {
                status = -1;
                break;
            }

            if (result == 0)
            {
                linkStats.timeouts++;
                break;
            }

            if (control == C_DISC)
            {
                gotDisc = TRUE;
                break;
            }
        }

        stopAlarm();

        if (status < 0)
            break;
    }

    if (status < 0 || gotDisc == FALSE)
    {
        stopAlarm();
        if (closeSerialPort() < 0)
            perror("closeSerialPort");
        linkStatsPrint(&linkStats);
        currentRole = ROLE_NONE;
        fprintf(stderr, "Close handshake failed: no DISC from the transmitter.\n");
        return -1;
    }

    // 2. Answer with DISC (A=0x01) and wait for the transmitter's UA.
    for (int attempt = 1; attempt <= totalAttempts && gotUa == FALSE; attempt++)
    {
        if (sendSupervision(C_DISC) != 0)
        {
            status = -1;
            break;
        }

        if (attempt > 1)
            linkStats.retransmissions++;

        startAlarm((unsigned)linkParams.timeout);

        for (;;)
        {
            int result = waitSupervisionControl(&control);

            if (result < 0)
            {
                status = -1;
                break;
            }

            if (result == 0)
            {
                linkStats.timeouts++;
                break;
            }

            if (control == C_UA)
            {
                gotUa = TRUE;
                break;
            }

            if (control == C_DISC)
            {
                // The transmitter re-sent its DISC, so our DISC was probably
                // lost. Answer again on the next attempt.
                break;
            }
        }

        stopAlarm();

        if (status < 0)
            break;
    }

    stopAlarm();

    if (closeSerialPort() < 0)
        perror("closeSerialPort");

    linkStatsPrint(&linkStats);
    currentRole = ROLE_NONE;

    if (status < 0 || gotUa == FALSE)
    {
        fprintf(stderr, "Close handshake failed: no UA from the transmitter.\n");
        return -1;
    }

    printf("Serial port closed (receiver).\n");
    return 0;
}
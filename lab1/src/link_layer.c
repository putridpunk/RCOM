// Link layer protocol implementation
// This module implements the connection establishment and
// supervision frame handling required by the serial protocol.

#include "link_layer.h"
#include "serial_port.h"
#include "alarm_sigaction.h"
#include "frame.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// Counters for the current connection. Filled in as frames are processed;
// L8 wires them into the close path, which prints them.
static LinkStats linkStats;

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

// States of the frame reception machine (slides 19-21).
typedef enum
{
    ST_HUNT_FLAG,          // look for the opening delimiter
    ST_READ_A,             // address field
    ST_READ_C,             // control field
    ST_READ_BCC1,          // header check character
    ST_READ_SUPERVISION_F, // closing flag of a supervision frame
    ST_READ_DATA           // de-stuffed data field, up to the closing flag
} RxState;

// The address a frame with the given control field must carry.
static unsigned char supervisionAddress(unsigned char control)
{
    return (control == C_UA) ? A_RX : A_TX;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
// Validate a 5-byte supervision frame in the format: F A C BCC1 F.
static int isValidSupervisionFrame(const unsigned char *frame, int frameSize, unsigned char expectedControl)
{
    if (frameSize != SUPERVISION_FRAME_SIZE)
        return FALSE;

    if (frame[0] != FRAME_FLAG || frame[4] != FRAME_FLAG)
        return FALSE;

    if (frame[1] != supervisionAddress(expectedControl) || frame[2] != expectedControl)
        return FALSE;

    return (frameBcc1(frame[1], frame[2]) == frame[3]);
}

// Send a supervision frame. Returns 0 on success, -1 on failure.
static int sendSupervisionFrame(unsigned char address, unsigned char control)
{
    unsigned char frame[SUPERVISION_FRAME_SIZE];

    int built = frameBuildSupervision(frame, sizeof(frame), address, control);
    if (built != SUPERVISION_FRAME_SIZE)
        return -1;

    if (writeBytesSerialPort(frame, built) != built)
        return -1;

    linkStatsCountSupervisionSent(&linkStats, control);
    return 0;
}

// Open the serial port as transmitter and perform the SET/UA handshake.
int llOpenTx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    if (setupAlarmHandler() == -1)
    {
        perror("setupAlarmHandler");
        closeSerialPort();
        return -1;
    }

    linkStatsReset(&linkStats);
    resetAlarmCount();

    // nRetransmissions counts retries after the first attempt.
    const int maxAttempts = llParameters.nRetransmissions + 1;

    for (int attempt = 1; attempt <= maxAttempts; attempt++)
    {
        unsigned char packet[SUPERVISION_FRAME_SIZE] = {0};

        if (sendSupervisionFrame(A_TX, C_SET) != 0)
        {
            perror("writeBytesSerialPort");
            stopAlarm();
            closeSerialPort();
            return -1;
        }

        startAlarm((unsigned)llParameters.timeout);

        // Block in llReceive until a frame arrives, a fatal I/O error occurs,
        // or SIGALRM clears the alarm flag. llReceive returns 0 for anything
        // that is not a supervision frame, so junk and damaged frames are
        // skipped inside the same attempt instead of burning a retry.
        for (;;)
        {
            int received = llReceive(packet);

            if (received < 0)
            {
                stopAlarm();
                closeSerialPort();
                return -1;
            }

            if (received == SUPERVISION_FRAME_SIZE && isValidSupervisionFrame(packet, received, C_UA))
            {
                stopAlarm();
                printf("UA received. Connection established.\n");
                return 0;
            }

            if (isAlarmActive() == FALSE)
            {
                linkStats.timeouts++;
                printf("Timeout waiting for UA. Attempt %d of %d.\n", attempt, maxAttempts);
                break;
            }
        }
    }

    stopAlarm();
    closeSerialPort();
    fprintf(stderr, "Maximum number of retransmissions reached. Connection failed.\n");
    return -1;
}

// Open the serial port as receiver and wait for a SET frame from the transmitter.
int llOpenRx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    linkStatsReset(&linkStats);

    for (;;)
    {
        unsigned char packet[SUPERVISION_FRAME_SIZE] = {0};

        int received = llReceive(packet);

        if (received < 0)
        {
            fprintf(stderr, "Error while waiting for SET.\n");
            closeSerialPort();
            return -1;
        }

        if (received == SUPERVISION_FRAME_SIZE && isValidSupervisionFrame(packet, received, C_SET))
        {
            printf("SET received. Sending UA.\n");

            if (sendSupervisionFrame(A_RX, C_UA) != 0)
            {
                perror("writeBytesSerialPort");
                closeSerialPort();
                return -1;
            }

            return 0;
        }
    }
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
// Send raw bytes over the open serial link.
int llSend(const unsigned char *buf, int bufSize)
{
    if (buf == NULL || bufSize <= 0)
    {
        return -1;
    }

    return writeBytesSerialPort(buf, bufSize);
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
// Read one frame from the serial port, byte by byte, and classify it.
//
// The machine hunts an opening flag, then consumes address, control and BCC1.
// The control field decides which kind of frame it is: supervision frames end
// at the next flag, information frames carry a de-stuffed data field followed
// by a BCC2 before the closing flag.
//
// "capacity" is the space available in "buf". A data frame that does not fit
// is discarded rather than truncated. "*frameSize" receives the number of
// bytes stored: SUPERVISION_FRAME_SIZE for a supervision frame, the length of
// the data field for an information frame.
static RxResult rxReadFrame(unsigned char *buf, size_t capacity, int *frameSize)
{
    if (buf == NULL || frameSize == NULL || capacity < (size_t)SUPERVISION_FRAME_SIZE)
        return RX_IO_ERROR;

    RxState state = ST_HUNT_FLAG;

    unsigned char address = 0;
    unsigned char control = 0;

    size_t filled = 0;      // de-stuffed bytes held in "buf"
    int overflow = FALSE;   // the data field did not fit in "buf"
    int escape = FALSE;     // the previous byte was the escape octet

    for (;;)
    {
        unsigned char byte = 0;
        int r = readByteSerialPort(&byte);

        if (r < 0)
        {
            // SIGALRM (or any other signal) interrupts the blocking read; that
            // is the timeout mechanism, not an error.
            if (errno == EINTR)
                return RX_TIMEOUT;

            perror("readByteSerialPort");
            return RX_IO_ERROR;
        }

        // The port is configured with VMIN = 1, so a successful read always
        // returns one byte. A zero means the port went away underneath us.
        if (r == 0)
        {
            fprintf(stderr, "readByteSerialPort returned 0 on a blocking port.\n");
            return RX_IO_ERROR;
        }

        switch (state)
        {
        case ST_HUNT_FLAG:
            if (byte == FRAME_FLAG)
                state = ST_READ_A;
            break;

        case ST_READ_A:
            if (byte == FRAME_FLAG)
            {
                // One or more flags may precede a frame; stay here.
                break;
            }

            if (byte == A_TX || byte == A_RX)
            {
                address = byte;
                state = ST_READ_C;
            }
            else
            {
                state = ST_HUNT_FLAG;
            }
            break;

        case ST_READ_C:
            if (byte == FRAME_FLAG)
            {
                // A flag where the control field should be opens a new frame.
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
                // A frame with a damaged header is dropped without any action
                // and without consuming a retransmission attempt.
                linkStats.bcc1Errors++;
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

            buf[0] = FRAME_FLAG;
            buf[1] = address;
            buf[2] = control;
            buf[3] = frameBcc1(address, control);
            buf[4] = FRAME_FLAG;
            *frameSize = SUPERVISION_FRAME_SIZE;
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

                if (filled < capacity)
                    buf[filled++] = byte;
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
                if (filled < capacity)
                    buf[filled++] = byte;
                else
                    overflow = TRUE;
                break;
            }

            // Closing flag: the last de-stuffed byte is the BCC2.
            if (overflow)
                return RX_IGNORED;

            if (filled < 1)
                return RX_IGNORED;

            if (frameBcc2(buf, filled - 1) != buf[filled - 1])
            {
                linkStats.bcc2Errors++;
                return RX_BAD_BCC2;
            }

            *frameSize = (int)(filled - 1);
            return RX_OK_DATA;
        }
    }
}

// Receive one frame from the serial port.
// Returns the frame size on success, 0 if the call was interrupted or the
// frame was discarded, or -1 on a fatal error.
int llReceive(unsigned char *packet)
{
    if (packet == NULL)
    {
        return -1;
    }

    // L2 callers only ever pass a supervision-frame buffer; information
    // frames are therefore discarded by the capacity check rather than
    // overflowing it. L6 gives llReceive the MAX_PAYLOAD_SIZE contract.
    int frameSize = 0;
    RxResult result = rxReadFrame(packet, SUPERVISION_FRAME_SIZE, &frameSize);

    switch (result)
    {
    case RX_OK_SUPERVISION:
        linkStatsCountSupervisionReceived(&linkStats, packet[2]);
        return frameSize;

    case RX_OK_DATA:
    case RX_BAD_BCC2:
    case RX_IGNORED:
    case RX_TIMEOUT:
        return 0;

    case RX_IO_ERROR:
    default:
        return -1;
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
// Close the transmitter port after the communication is complete.
int llCloseTx(void)
{
    stopAlarm();

    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port closed (transmitter).\n");
    return 0;
}

// Close the receiver port after the communication is complete.
int llCloseRx(void)
{
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port closed (receiver).\n");
    return 0;
}
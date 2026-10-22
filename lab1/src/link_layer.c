//
//
// Link layer protocol implementation
// This module implements the connection establishment and
// supervision frame handling required by the serial protocol.

#include "link_layer.h"
#include "serial_port.h"
#include "alarm_sigaction.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// Supervision frame: F A C BCC1 F
#define SUPERVISION_FRAME_SIZE 5

#define FRAME_FLAG 0x7E // frame delimiter
#define A_COMMAND  0x03 // address used by the connection initiator
#define A_REPLY    0x01 // address used by the connection responder
#define C_SET      0x03 // connection request (command)
#define C_UA       0x07 // unnumbered acknowledgement (reply)

// Bytes accumulated while hunting for a supervision frame. Kept at file scope
// so it can be reset when a connection is opened.
static unsigned char rxWindow[SUPERVISION_FRAME_SIZE];
static int rxFilled = 0;

static void resetReceiver(void)
{
    rxFilled = 0;
}

// The address a frame with the given control field must carry.
static unsigned char supervisionAddress(unsigned char control)
{
    return (control == C_UA) ? A_REPLY : A_COMMAND;
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

    return ((frame[1] ^ frame[2]) == frame[3]);
}

// Send a supervision frame. Returns 0 on success, -1 on failure.
static int sendSupervisionFrame(unsigned char address, unsigned char control)
{
    unsigned char frame[SUPERVISION_FRAME_SIZE];

    frame[0] = FRAME_FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = address ^ control;
    frame[4] = FRAME_FLAG;

    if (writeBytesSerialPort(frame, SUPERVISION_FRAME_SIZE) != SUPERVISION_FRAME_SIZE)
        return -1;

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

    resetReceiver();
    resetAlarmCount();

    // nRetransmissions counts retries after the first attempt.
    const int maxAttempts = llParameters.nRetransmissions + 1;

    for (int attempt = 1; attempt <= maxAttempts; attempt++)
    {
        unsigned char packet[SUPERVISION_FRAME_SIZE] = {0};

        if (sendSupervisionFrame(A_COMMAND, C_SET) != 0)
        {
            perror("writeBytesSerialPort");
            stopAlarm();
            closeSerialPort();
            return -1;
        }

        startAlarm((unsigned)llParameters.timeout);

        // Block in llReceive until a frame arrives, a fatal I/O error occurs,
        // or SIGALRM clears the alarm flag. A read interrupted by SIGALRM
        // yields 0, which is the only timeout signal available here.
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

    resetReceiver();

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

            if (sendSupervisionFrame(A_REPLY, C_UA) != 0)
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
// Receive one supervision frame from the serial port.
// Returns the frame size on success, 0 if the call was interrupted before a
// complete frame was assembled, or -1 on a fatal error.
int llReceive(unsigned char *packet)
{
    if (packet == NULL)
    {
        return -1;
    }

    for (;;)
    {
        unsigned char byte = 0;
        int r = readByteSerialPort(&byte);

        if (r < 0)
        {
            // SIGALRM (or any other signal) interrupts the blocking read; that
            // is the timeout mechanism, not an error.
            if (errno == EINTR)
            {
                return 0;
            }
            perror("readByteSerialPort");
            return -1;
        }

        if (r == 0)
        {
            continue;
        }

        // Hunt for the opening flag before starting to collect bytes.
        if (rxFilled == 0 && byte != FRAME_FLAG)
        {
            continue;
        }

        rxWindow[rxFilled++] = byte;

        if (rxFilled < SUPERVISION_FRAME_SIZE)
        {
            continue;
        }

        if (rxWindow[0] == FRAME_FLAG && rxWindow[SUPERVISION_FRAME_SIZE - 1] == FRAME_FLAG)
        {
            memcpy(packet, rxWindow, SUPERVISION_FRAME_SIZE);
            rxFilled = 0;
            return SUPERVISION_FRAME_SIZE;
        }

        // Not properly delimited: drop the first byte and keep scanning, so a
        // valid frame starting inside the window is not thrown away.
        memmove(rxWindow, rxWindow + 1, SUPERVISION_FRAME_SIZE - 1);
        rxFilled = SUPERVISION_FRAME_SIZE - 1;
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
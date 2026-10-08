// RCOM 2026/2027
//
// Link layer protocol implementation
// This module implements the connection establishment and
// supervision frame handling required by the serial protocol.

#include "link_layer.h"
#include "serial_port.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

int alarmEnabled = FALSE;
int alarmCount = 0;

void alarmHandler(int signal)
{
    (void)signal;
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d received\n", alarmCount);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
// Validate a 5-byte supervision frame in the format: F A C BCC1 F.
// The expected control field depends on whether we are checking for SET (0x03)
// or UA (0x07). For these frames, BCC1 = A ^ C.
static int isValidSupervisionFrame(const unsigned char *frame, int frameSize, unsigned char expectedControl)
{
    if (frameSize != 5)
        return FALSE;

    if (frame[0] != 0x7E || frame[4] != 0x7E)
        return FALSE;

    if (frame[1] != 0x03 || frame[2] != expectedControl)
        return FALSE;

    return ((frame[1] ^ frame[2]) == frame[3]);
}

// Send a supervision frame with the given control byte.
static int sendSupervisionFrame(unsigned char control)
{
    unsigned char frame[5] = {0};
    frame[0] = 0x7E;
    frame[1] = 0x03;
    frame[2] = control;
    frame[3] = frame[1] ^ frame[2];
    frame[4] = 0x7E;

    return writeBytesSerialPort(frame, 5);
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

    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        return -1;
    }

    int retries = 0;
    while (retries <= llParameters.nRetransmissions)
    {
        unsigned char packet[5] = {0};

        alarm(0);
        alarmEnabled = FALSE;

        if (sendSupervisionFrame(0x03) < 0)
        {
            perror("writeBytesSerialPort");
            closeSerialPort();
            return -1;
        }

        alarm(llParameters.timeout);
        alarmEnabled = TRUE;

        int received = llReceive(packet);
        if (received == 5 && isValidSupervisionFrame(packet, received, 0x07))
        {
            alarm(0);
            alarmEnabled = FALSE;
            printf("UA received. Connection established.\n");
            return 0;
        }

        if (alarmEnabled == FALSE)
        {
            printf("Timeout waiting for UA. Retransmitting SET.\n");
            retries++;
            if (retries > llParameters.nRetransmissions)
            {
                alarm(0);
                closeSerialPort();
                fprintf(stderr, "Maximum number of retransmissions reached.\n");
                return -1;
            }
            continue;
        }

        retries++;
    }

    closeSerialPort();
    fprintf(stderr, "Failed to establish connection.\n");
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

    while (1)
    {
        unsigned char packet[5] = {0};
        int received = llReceive(packet);

        if (received == 5 && isValidSupervisionFrame(packet, received, 0x03))
        {
            printf("SET received. Sending UA.\n");
            if (sendSupervisionFrame(0x07) < 0)
            {
                perror("writeBytesSerialPort");
                closeSerialPort();
                return -1;
            }
            return 0;
        }

        if (received < 0)
        {
            fprintf(stderr, "Error while waiting for SET.\n");
            closeSerialPort();
            return -1;
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
        return 0;
    }

    return writeBytesSerialPort(buf, bufSize);
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
// Receive one valid supervision frame from the serial port.
int llReceive(unsigned char *packet)
{
    if (packet == NULL)
    {
        return -1;
    }

    unsigned char buf[5] = {0};
    int idx = 0;

    while (1)
    {
        unsigned char byte = 0;
        int r = readByteSerialPort(&byte);

        if (r < 0)
        {
            if (errno == EINTR)
            {
                return -1;
            }
            perror("readByteSerialPort");
            return -1;
        }

        if (r == 0)
        {
            continue;
        }

        if (idx == 0 && byte != 0x7E)
        {
            continue;
        }

        buf[idx++] = byte;

        if (idx == 5)
        {
            if (buf[0] == 0x7E && buf[4] == 0x7E && buf[1] == 0x03 &&
                ((buf[1] ^ buf[2]) == buf[3]))
            {
                for (int i = 0; i < 5; i++)
                    packet[i] = buf[i];
                return 5;
            }

            idx = 0;
        }
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
// Close the transmitter port after the communication is complete.
int llCloseTx()
{
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port closed (transmitter).\n");
    return 0;
}

// Close the receiver port after the communication is complete.
int llCloseRx()
{
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port closed (receiver).\n");
    return 0;
}

// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

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
int llOpenTx(LinkLayer llParameters)
{
    // ----------------------------------------------------
    // This example code shows how to open the serial port and send a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

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
        exit(1);
    }

    int retries = 0;
    while (retries <= llParameters.nRetransmissions)
    {
        unsigned char frame[5] = {0};
        unsigned char packet[5] = {0};

        frame[0] = 0x7E;
        frame[1] = 0x03;
        frame[2] = 0x03;
        frame[3] = 0x03 ^ 0x03;
        frame[4] = 0x7E;

        alarm(0);
        alarmEnabled = FALSE;

        int bytes = writeBytesSerialPort(frame, 5);
        printf("%d bytes written to serial port\n", bytes);

        alarm(llParameters.timeout);
        alarmEnabled = TRUE;

        int received = llReceive(packet);
        if (received == 5 && packet[0] == 0x7E && packet[4] == 0x7E && packet[1] == 0x03 && packet[2] == 0x07 && ((packet[1] ^ packet[2]) == packet[3]))
        {
            alarm(0);
            alarmEnabled = FALSE;
            printf("UA received. Connection established.\n");
            break;
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

    // Wait until all bytes have been written to the serial port
    sleep(1);

    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);

    return 0;
}

int llOpenRx(LinkLayer llParameters)
{
    // ----------------------------------------------------
    // This example code shows how to open the serial port and receive a string.
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Serial port %s opened\n", llParameters.serialPort);

    // Read from serial port until the 'z' char is received.

    // NOTE: This while() cycle is a simple example showing how to read from the serial port.
    // It must be changed in order to respect the specifications of the protocol indicated in the Lab guide.

    // TODO: Save the received bytes in a buffer array and print it at the end of the program.
    volatile int STOP = FALSE;
    int nBytesBuf = 0;

    while (STOP == FALSE)
    {
        // Read one byte from serial port.
        // NOTE: You must check how many bytes were actually read by reading the return value.
        // In this example, we assume that the byte is always read, which may not be true.
        unsigned char byte;
        int bytes = readByteSerialPort(&byte);
        nBytesBuf += bytes;

        printf("Byte received: %c\n", byte);

        if (byte == 'z')
        {
            printf("Received 'z' char. Stop reading from serial port.\n");
            STOP = TRUE;
        }
    }

    printf("Total bytes received: %d\n", nBytesBuf);

    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        return -1;
    }

    printf("Serial port %s closed\n", llParameters.serialPort);

    return 0;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    (void)buf;
    (void)bufSize;

    // Build a supervision frame for connection establishment (SET)
    // Frame format: |F|A|C|BCC1|F|
    // FLAG: 0x7E
    // A: 0x03 for frames sent by the Sender
    // C (SET): 0x03
    // BCC1: A ^ C

    unsigned char frame[5];
    const unsigned char FLAG = 0x7E;
    const unsigned char A = 0x03; // Sender address
    const unsigned char C = 0x03; // SET
    const unsigned char BCC1 = A ^ C;

    frame[0] = FLAG;
    frame[1] = A;
    frame[2] = C;
    frame[3] = BCC1;
    frame[4] = FLAG;

    // Debug: print frame bytes in hexadecimal
    for (int i = 0; i < 5; i++)
    {
        // Use cast to unsigned int to be compatible with older compilers
        printf("frame[%d] = 0x%02X\n", i, (unsigned int)(frame[i] & 0xFF));
    }

    // Send the 5-byte supervision frame
    int written = writeBytesSerialPort(frame, 5);
    if (written < 0)
    {
        perror("writeBytesSerialPort");
        return -1;
    }

    return written;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    // Read a 5-byte supervision frame and validate it.
    // Expected frame format: |F|A|C|BCC1|F|
    // FLAG: 0x7E

    const unsigned char FLAG = 0x7E;
    unsigned char buf[5];
    int idx = 0;
    int state = 0;

    while (1)
    {
        unsigned char byte = 0;
        int r = readByteSerialPort(&byte);
        if (r < 0)
        {
            perror("readByteSerialPort");
            return -1;
        }
        else if (r == 0)
        {
            // No byte available (should not happen with blocking reads), continue
            continue;
        }

        switch (state)
        {
            case 0: // START
                if (byte == FLAG)
                {
                    buf[idx++] = byte;
                    state = 1;
                }
                break;

            case 1: // FLAG_RCV
                if (byte == 0x03)
                {
                    buf[idx++] = byte;
                    state = 2;
                }
                else if (byte == FLAG)
                {
                    idx = 1;
                    buf[0] = FLAG;
                }
                else
                {
                    idx = 0;
                    state = 0;
                }
                break;

            case 2: // A_RCV
                if (byte == 0x03)
                {
                    buf[idx++] = byte;
                    state = 3;
                }
                else
                {
                    idx = 0;
                    state = 0;
                }
                break;

            case 3: // C_RCV
                buf[idx++] = byte;
                state = 4;
                break;

            case 4: // BCC_RCV
                if (byte == FLAG)
                {
                    buf[idx++] = byte;

                    if (idx == 5 &&
                        buf[0] == FLAG &&
                        buf[4] == FLAG &&
                        buf[1] == 0x03 &&
                        buf[2] == 0x03 &&
                        ((buf[1] ^ buf[2]) == buf[3]))
                    {
                        for (int i = 0; i < 5; i++)
                            packet[i] = buf[i];

                        return 5;
                    }

                    idx = 0;
                    state = 0;
                }
                else
                {
                    idx = 0;
                    state = 0;
                }
                break;
        }
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    // TODO: Implement this function

    return 0;
}

int llCloseRx()
{
    // TODO: Implement this function

    return 0;
}

// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

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

    // Create string to send
    unsigned char buf[BUF_SIZE] = {0};

    for (int i = 0; i < BUF_SIZE; i++)
    {
        buf[i] = 'a' + i % 26;
    }

    // In non-canonical mode, '\n' does not end the writing.
    // Test this condition by placing a '\n' in the middle of the buffer.
    // The whole buffer must be sent even with the '\n'.
    buf[5] = '\n';

    int bytes = writeBytesSerialPort(buf, BUF_SIZE);
    printf("%d bytes written to serial port\n", bytes);

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

    while (idx < 5)
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

        // Store byte
        buf[idx++] = byte;
    }

    // Debug: print received bytes in hexadecimal
    for (int i = 0; i < 5; i++)
    {
        printf("recv[%d] = 0x%02X\n", i, (unsigned int)(buf[i] & 0xFF));
    }

    // Basic validation: first and last must be FLAG
    if (buf[0] != FLAG || buf[4] != FLAG)
    {
        fprintf(stderr, "Invalid supervision frame: missing FLAG\n");
        return -1;
    }

    // Validate BCC1 (A ^ C)
    unsigned char A = buf[1];
    unsigned char C = buf[2];
    unsigned char BCC1 = buf[3];

    // Debug: print header fields
    printf("A = 0x%02X, C = 0x%02X, BCC1 = 0x%02X\n", (unsigned int)(A & 0xFF), (unsigned int)(C & 0xFF), (unsigned int)(BCC1 & 0xFF));

    if ((A ^ C) != BCC1)
    {
        fprintf(stderr, "Invalid supervision frame: BCC1 mismatch\n");
        return -1;
    }

    // Copy validated frame into packet (5 bytes)
    for (int i = 0; i < 5; i++)
        packet[i] = buf[i];

    return 5;
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

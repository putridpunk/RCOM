// 
//
// Application layer protocol implementation
// This layer sends or receives the file metadata and payload after
// the link-layer handshake has been completed.

#include "application_layer.h"
#include "link_layer.h"
#include "serial_port.h"

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Length field sent ahead of the payload.
#define SIZE_FIELD_LEN 4

// Largest value the 4-byte length field can carry.
#define MAX_FILE_SIZE 0xFFFFFFFFL

// Read exactly 'length' bytes from the serial port.
// Returns the number of bytes read, or -1 on error.
static long readExactly(unsigned char *buffer, long length)
{
    long total = 0;

    while (total < length)
    {
        int bytesRead = readByteSerialPort(buffer + total);

        if (bytesRead < 0)
        {
            if (errno == EINTR)
                continue;
            perror("readByteSerialPort");
            return -1;
        }

        // VMIN is 1, so a successful read returns one byte; 0 cannot happen
        // unless the port was closed underneath us.
        if (bytesRead == 0)
            continue;

        total += bytesRead;
    }

    return total;
}

// Write exactly 'length' bytes to the serial port.
// Returns the number of bytes written, or -1 on error.
static long writeExactly(const unsigned char *buffer, long length)
{
    long total = 0;

    while (total < length)
    {
        long remaining = length - total;
        int chunk = (remaining > (long)INT_MAX) ? INT_MAX : (int)remaining;

        int bytesWritten = writeBytesSerialPort(buffer + total, chunk);

        if (bytesWritten < 0)
        {
            if (errno == EINTR)
                continue;
            perror("writeBytesSerialPort");
            return -1;
        }

        if (bytesWritten == 0)
        {
            // A short write that makes no progress would spin forever.
            fprintf(stderr, "writeBytesSerialPort made no progress.\n");
            return -1;
        }

        total += bytesWritten;
    }

    return total;
}

// Main application entry point. The role selects whether the process acts as
// a sender or a receiver. After the connection setup, the file size and bytes
// are exchanged over the established serial link.
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };

    int written = snprintf(llParameters.serialPort, sizeof(llParameters.serialPort), "%s", serialPort);
    if (written < 0 || written >= (int)sizeof(llParameters.serialPort))
    {
        fprintf(stderr, "Serial port name too long (max %zu characters).\n",
                sizeof(llParameters.serialPort) - 1);
        return;
    }

    if (strcmp(role, "tx") == 0)
    {
        FILE *file = fopen(filename, "rb");
        if (file == NULL)
        {
            perror("fopen");
            return;
        }

        if (llOpenTx(llParameters) != 0)
        {
            fclose(file);
            return;
        }

        if (fseek(file, 0, SEEK_END) != 0)
        {
            perror("fseek");
            fclose(file);
            llCloseTx();
            return;
        }

        long fileSize = ftell(file);
        if (fileSize < 0 || fileSize > MAX_FILE_SIZE)
        {
            fprintf(stderr, "File size out of range (0..%ld bytes).\n", MAX_FILE_SIZE);
            fclose(file);
            llCloseTx();
            return;
        }
        rewind(file);

        unsigned char sizeField[SIZE_FIELD_LEN] = {
            (unsigned char)(fileSize & 0xFF),
            (unsigned char)((fileSize >> 8) & 0xFF),
            (unsigned char)((fileSize >> 16) & 0xFF),
            (unsigned char)((fileSize >> 24) & 0xFF)
        };

        if (writeExactly(sizeField, SIZE_FIELD_LEN) != SIZE_FIELD_LEN)
        {
            fprintf(stderr, "Failed to send file size.\n");
            fclose(file);
            llCloseTx();
            return;
        }

        unsigned char *buffer = malloc(fileSize == 0 ? 1 : (size_t)fileSize);
        if (buffer == NULL)
        {
            fprintf(stderr, "Memory allocation failed.\n");
            fclose(file);
            llCloseTx();
            return;
        }

        size_t bytesRead = fread(buffer, 1, (size_t)fileSize, file);
        if (bytesRead != (size_t)fileSize)
        {
            fprintf(stderr, "Failed to read full file.\n");
            free(buffer);
            fclose(file);
            llCloseTx();
            return;
        }

        if (writeExactly(buffer, fileSize) != fileSize)
        {
            fprintf(stderr, "Failed to send file data.\n");
            free(buffer);
            fclose(file);
            llCloseTx();
            return;
        }

        free(buffer);
        fclose(file);
        printf("File %s sent successfully (%ld bytes).\n", filename, fileSize);
        llCloseTx();
    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(llParameters) != 0)
            return;

        unsigned char sizeField[SIZE_FIELD_LEN] = {0};
        if (readExactly(sizeField, SIZE_FIELD_LEN) != SIZE_FIELD_LEN)
        {
            fprintf(stderr, "Failed to receive file size.\n");
            llCloseRx();
            return;
        }

        // Decode through unsigned long: a size with the top bit set would
        // otherwise become negative on 32-bit long.
        long fileSize = (long)((unsigned long)sizeField[0] |
                               ((unsigned long)sizeField[1] << 8) |
                               ((unsigned long)sizeField[2] << 16) |
                               ((unsigned long)sizeField[3] << 24));

        if (fileSize < 0 || fileSize > MAX_FILE_SIZE)
        {
            fprintf(stderr, "Received file size out of range.\n");
            llCloseRx();
            return;
        }

        unsigned char *buffer = malloc(fileSize == 0 ? 1 : (size_t)fileSize);
        if (buffer == NULL)
        {
            fprintf(stderr, "Memory allocation failed.\n");
            llCloseRx();
            return;
        }

        if (fileSize > 0 && readExactly(buffer, fileSize) != fileSize)
        {
            fprintf(stderr, "Failed to receive file data.\n");
            free(buffer);
            llCloseRx();
            return;
        }

        FILE *file = fopen(filename, "wb");
        if (file == NULL)
        {
            perror("fopen");
            free(buffer);
            llCloseRx();
            return;
        }

        if (fileSize > 0 && fwrite(buffer, 1, (size_t)fileSize, file) != (size_t)fileSize)
        {
            perror("fwrite");
            free(buffer);
            fclose(file);
            llCloseRx();
            return;
        }

        free(buffer);
        fclose(file);
        printf("File %s received successfully (%ld bytes).\n", filename, fileSize);
        llCloseRx();
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
        return;
    }
}
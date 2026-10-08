// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include "serial_port.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int readExactly(unsigned char *buffer, int length)
{
    int total = 0;

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
        if (bytesRead == 0)
            continue;

        total += bytesRead;
    }

    return total;
}

static int writeExactly(const unsigned char *buffer, int length)
{
    int total = 0;

    while (total < length)
    {
        int bytesWritten = writeBytesSerialPort(buffer + total, length - total);
        if (bytesWritten < 0)
        {
            if (errno == EINTR)
                continue;
            perror("writeBytesSerialPort");
            return -1;
        }
        if (bytesWritten == 0)
            continue;

        total += bytesWritten;
    }

    return total;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    strcpy(llParameters.serialPort, serialPort);

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
        if (fileSize < 0)
        {
            perror("ftell");
            fclose(file);
            llCloseTx();
            return;
        }
        rewind(file);

        unsigned char sizeField[4] = {
            (unsigned char)(fileSize & 0xFF),
            (unsigned char)((fileSize >> 8) & 0xFF),
            (unsigned char)((fileSize >> 16) & 0xFF),
            (unsigned char)((fileSize >> 24) & 0xFF)
        };

        if (writeExactly(sizeField, 4) != 4)
        {
            fprintf(stderr, "Failed to send file size.\n");
            fclose(file);
            llCloseTx();
            return;
        }

        unsigned char *buffer = malloc((size_t)fileSize == 0 ? 1 : (size_t)fileSize);
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

        if (writeExactly(buffer, (int)bytesRead) != (int)bytesRead)
        {
            fprintf(stderr, "Failed to send file data.\n");
            free(buffer);
            fclose(file);
            llCloseTx();
            return;
        }

        free(buffer);
        fclose(file);
        llCloseTx();
    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(llParameters) != 0)
            return;

        unsigned char sizeField[4] = {0};
        if (readExactly(sizeField, 4) != 4)
        {
            fprintf(stderr, "Failed to receive file size.\n");
            llCloseRx();
            return;
        }

        long fileSize = (long)sizeField[0] |
                        ((long)sizeField[1] << 8) |
                        ((long)sizeField[2] << 16) |
                        ((long)sizeField[3] << 24);

        unsigned char *buffer = malloc((size_t)fileSize == 0 ? 1 : (size_t)fileSize);
        if (buffer == NULL)
        {
            fprintf(stderr, "Memory allocation failed.\n");
            llCloseRx();
            return;
        }

        if (fileSize > 0 && readExactly(buffer, (int)fileSize) != fileSize)
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
        printf("File %s received successfully.\n", filename);
        llCloseRx();
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
        return;
    }
}

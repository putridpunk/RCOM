// Standalone checks for the pure frame helpers.
//
// Build and run (from lab1/):
//   gcc -Wall -I src src/frame.c tests/selftest.c -o tests/selftest
//   ./tests/selftest

#include <stdio.h>
#include <string.h>

#include "frame.h"

static int failures = 0;

#define CHECK(condition)                                              \
    do                                                                \
    {                                                                 \
        if (!(condition))                                             \
        {                                                             \
            printf("FAIL line %d: %s\n", __LINE__, #condition);       \
            failures++;                                               \
        }                                                             \
    } while (0)

int main(void)
{
    // Control field predicates and Ns / Nr extraction.
    CHECK(frameCIsInfo(C_I0) == true);
    CHECK(frameCIsInfo(C_I1) == true);
    CHECK(frameCIsInfo(C_RR0) == false);
    CHECK(frameCIsRR(C_RR0) == true);
    CHECK(frameCIsRR(C_RR1) == true);
    CHECK(frameCIsRR(C_REJ0) == false);
    CHECK(frameCIsREJ(C_REJ1) == true);
    CHECK(frameCGetNs(C_I0) == 0);
    CHECK(frameCGetNs(C_I1) == 1);
    CHECK(frameCGetNr(C_RR0) == 0);
    CHECK(frameCGetNr(C_RR1) == 1);
    CHECK(frameCGetNr(C_REJ0) == 0);
    CHECK(frameCGetNr(C_REJ1) == 1);

    // BCC1 over the header.
    CHECK(frameBcc1(A_TX, C_SET) == 0x00);
    CHECK(frameBcc1(A_TX, C_UA) == 0x04);
    CHECK(frameBcc1(A_TX, C_DISC) == 0x08);
    CHECK(frameBcc1(A_RX, C_DISC) == 0x0A);

    // BCC2 over the data field.
    unsigned char payload[4] = {0x10, 0x20, 0x7E, 0x7D};
    CHECK(frameBcc2(payload, 0) == 0x00);
    CHECK(frameBcc2(payload, 4) == (unsigned char)(0x10 ^ 0x20 ^ 0x7E ^ 0x7D));

    // Stuffing, and the refusal to write into a buffer that is too small.
    unsigned char in[4] = {0x7E, 0x7D, 0x00, 0xFF};
    unsigned char out[16];
    int stuffed = frameStuff(in, sizeof(in), out, sizeof(out));

    CHECK(stuffed == 6);
    CHECK(out[0] == 0x7D && out[1] == 0x5E);
    CHECK(out[2] == 0x7D && out[3] == 0x5D);
    CHECK(out[4] == 0x00 && out[5] == 0xFF);

    unsigned char tooSmall[5];
    CHECK(frameStuff(in, sizeof(in), tooSmall, sizeof(tooSmall)) == -1);

    // Supervision frame: F A C BCC1 F.
    unsigned char supervision[SUPERVISION_FRAME_SIZE];
    CHECK(frameBuildSupervision(supervision, sizeof(supervision), A_TX, C_SET) == 5);
    CHECK(supervision[0] == 0x7E);
    CHECK(supervision[1] == 0x03);
    CHECK(supervision[2] == 0x03);
    CHECK(supervision[3] == 0x00);
    CHECK(supervision[4] == 0x7E);
    CHECK(frameBuildSupervision(supervision, 4, A_TX, C_SET) == -1);

    // Information frame with a payload that needs no escaping.
    unsigned char packet[3] = {2, 0, 1};
    unsigned char wire[MAX_FRAME_SIZE];
    int wireLen = frameBuildInfo(wire, sizeof(wire), A_TX, 1, packet, sizeof(packet));

    CHECK(wireLen == 9);
    CHECK(wire[0] == 0x7E);
    CHECK(wire[1] == A_TX);
    CHECK(wire[2] == C_I1);
    CHECK(wire[3] == frameBcc1(A_TX, C_I1));
    CHECK(wire[4] == 2 && wire[5] == 0 && wire[6] == 1);
    CHECK(wire[7] == frameBcc2(packet, sizeof(packet)));
    CHECK(wire[8] == 0x7E);

    // Information frame whose payload and BCC2 are all escape candidates.
    unsigned char nasty[2] = {0x7E, 0x7D};
    wireLen = frameBuildInfo(wire, sizeof(wire), A_TX, 0, nasty, sizeof(nasty));

    unsigned char expectedBcc2 = frameBcc2(nasty, sizeof(nasty));
    // 4 header bytes + 2 + 2 stuffed payload bytes + 1 or 2 stuffed BCC2 + flag.
    int expectedLen = 4 + 4 + ((expectedBcc2 == 0x7E || expectedBcc2 == 0x7D) ? 2 : 1) + 1;
    CHECK(wireLen == expectedLen);
    CHECK(wire[wireLen - 1] == 0x7E);
    // No unescaped flag may appear inside the frame body.
    for (int i = 1; i < wireLen - 1; i++)
    {
        if (wire[i] == 0x7E)
        {
            printf("FAIL line %d: unescaped flag at offset %d\n", __LINE__, i);
            failures++;
        }
    }

    if (failures == 0)
        printf("selftest: all checks passed\n");
    else
        printf("selftest: %d check(s) failed\n", failures);

    return (failures == 0) ? 0 : 1;
}
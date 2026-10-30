// Standalone checks for the pure frame helpers.
//
//   gcc -Wall -Wextra -I src src/frame.c tests/selftest.c -o tests/selftest
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

// Independent of the receiver's state machine: an escape mechanism and its
// inverse are two programs, and testing one against the other is the only way
// to catch a pair that agrees with itself.
static int destuff(const unsigned char *in, size_t inSize,
                   unsigned char *out, size_t capacity)
{
    size_t written = 0;

    for (size_t i = 0; i < inSize; i++)
    {
        unsigned char byte = in[i];

        if (byte == FRAME_ESCAPE)
        {
            if (i + 1 >= inSize)
                return -1;

            unsigned char next = in[++i];

            if (next == FRAME_ESCAPE_FLAG)
                byte = FRAME_FLAG;
            else if (next == FRAME_ESCAPE_ESCAPE)
                byte = FRAME_ESCAPE;
            else
                return -1;
        }

        if (written >= capacity)
            return -1;

        out[written++] = byte;
    }

    return (int)written;
}

int main(void)
{
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

    CHECK(frameBcc1(A_TX, C_SET) == 0x00);
    CHECK(frameBcc1(A_TX, C_UA) == 0x04);
    CHECK(frameBcc1(A_TX, C_DISC) == 0x08);
    CHECK(frameBcc1(A_RX, C_DISC) == 0x0A);

    // the no-stuffing claim frameBuildSupervision relies on, over the whole table
    const unsigned char addresses[] = {A_TX, A_RX};
    const unsigned char controls[] = {C_SET, C_DISC, C_UA, C_I0, C_I1,
                                      C_RR0, C_RR1, C_REJ0, C_REJ1};

    for (size_t a = 0; a < sizeof(addresses); a++)
    {
        for (size_t c = 0; c < sizeof(controls); c++)
        {
            CHECK(frameBcc1(addresses[a], controls[c]) != FRAME_FLAG);
            CHECK(frameBcc1(addresses[a], controls[c]) != FRAME_ESCAPE);
        }
    }

    unsigned char payload[4] = {0x10, 0x20, 0x7E, 0x7D};
    CHECK(frameBcc2(payload, 0) == 0x00);
    CHECK(frameBcc2(payload, 4) == (unsigned char)(0x10 ^ 0x20 ^ 0x7E ^ 0x7D));
    CHECK(frameBcc2(NULL, 0) == 0x00);

    unsigned char in[4] = {0x7E, 0x7D, 0x00, 0xFF};
    unsigned char out[16];
    int stuffed = frameStuff(in, sizeof(in), out, sizeof(out));

    CHECK(stuffed == 6);
    CHECK(out[0] == 0x7D && out[1] == 0x5E);
    CHECK(out[2] == 0x7D && out[3] == 0x5D);
    CHECK(out[4] == 0x00 && out[5] == 0xFF);

    unsigned char tooSmall[5];
    CHECK(frameStuff(in, sizeof(in), tooSmall, sizeof(tooSmall)) == -1);

    unsigned char supervision[SUPERVISION_FRAME_SIZE];
    CHECK(frameBuildSupervision(supervision, sizeof(supervision), A_TX, C_SET) == 5);
    CHECK(supervision[0] == 0x7E);
    CHECK(supervision[1] == 0x03);
    CHECK(supervision[2] == 0x03);
    CHECK(supervision[3] == 0x00);
    CHECK(supervision[4] == 0x7E);
    CHECK(frameBuildSupervision(supervision, 4, A_TX, C_SET) == -1);

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

    // refused even though this packet would fit the smaller buffer
    unsigned char narrow[MAX_FRAME_SIZE - 1];
    CHECK(frameBuildInfo(narrow, sizeof(narrow), A_TX, 0, packet, sizeof(packet)) == -1);

    unsigned char nasty[2] = {0x7E, 0x7D};
    wireLen = frameBuildInfo(wire, sizeof(wire), A_TX, 0, nasty, sizeof(nasty));

    unsigned char expectedBcc2 = frameBcc2(nasty, sizeof(nasty));
    int expectedLen = 4 + 4 + ((expectedBcc2 == 0x7E || expectedBcc2 == 0x7D) ? 2 : 1) + 1;
    CHECK(wireLen == expectedLen);
    CHECK(wire[wireLen - 1] == 0x7E);

    // round trip over every byte value
    unsigned char everything[256];
    for (int i = 0; i < 256; i++)
        everything[i] = (unsigned char)i;

    wireLen = frameBuildInfo(wire, sizeof(wire), A_TX, 0, everything, sizeof(everything));
    CHECK(wireLen > 0);
    CHECK(wire[0] == FRAME_FLAG);
    CHECK(wire[wireLen - 1] == FRAME_FLAG);

    unsigned char body[2 * 256 + 2];
    int bodyLen = destuff(wire + FRAME_HEADER_SIZE,
                          (size_t)wireLen - FRAME_HEADER_SIZE - 1,
                          body, sizeof(body));

    CHECK(bodyLen == (int)sizeof(everything) + 1);

    if (bodyLen == (int)sizeof(everything) + 1)
    {
        CHECK(memcmp(body, everything, sizeof(everything)) == 0);
        CHECK(body[sizeof(everything)] == frameBcc2(everything, sizeof(everything)));
    }

    for (int i = FRAME_HEADER_SIZE; i < wireLen - 1; i++)
        CHECK(wire[i] != FRAME_FLAG);

    if (failures == 0)
        printf("selftest: all checks passed\n");
    else
        printf("selftest: %d check(s) failed\n", failures);

    return (failures == 0) ? 0 : 1;
}
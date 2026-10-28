#include "alarm_sigaction.h"

#include <signal.h>
#include <string.h>

// Both flags are written by the handler and read by the protocol code, so they
// must be sig_atomic_t. The handler only touches these: write(2) and alarm(2)
// are async-signal-safe, printf(3) is not.
static volatile sig_atomic_t alarmEnabled = 0;
static volatile sig_atomic_t alarmCount = 0;

static void alarmHandler(int sig)
{
    (void)sig;
    alarmEnabled = 0;
    alarmCount++;
}

int setupAlarmHandler(void)
{
    struct sigaction act;

    memset(&act, 0, sizeof(act));
    act.sa_handler = alarmHandler;
    sigemptyset(&act.sa_mask);
    // SA_RESTART is deliberately left unset: the blocking read() must return
    // EINTR so that the protocol code can notice the timeout.
    act.sa_flags = 0;

    if (sigaction(SIGALRM, &act, NULL) == -1)
        return -1;

    return 0;
}

void startAlarm(unsigned seconds)
{
    // Arm the flag before the timer, so the handler always finds it set.
    alarmEnabled = 1;
    alarm(seconds);
}

void stopAlarm(void)
{
    alarm(0);
    alarmEnabled = 0;
}

void resetAlarmCount(void) { alarmCount = 0; }

bool isAlarmActive(void) { return alarmEnabled != 0; }

int getAlarmCount(void) { return (int)alarmCount; }
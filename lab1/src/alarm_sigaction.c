#include "alarm_sigaction.h"

#include <signal.h>
#include <string.h>
#include <unistd.h> // alarm(2)

// the handler writes this and the protocol reads it -> sig_atomic_t
// (no printf in the handler, it's not async-signal-safe)
static volatile sig_atomic_t alarmEnabled = 0;

static void alarmHandler(int sig)
{
    (void)sig;
    alarmEnabled = 0;
}

int setupAlarmHandler(void)
{
    struct sigaction act;

    memset(&act, 0, sizeof(act));
    act.sa_handler = alarmHandler;
    sigemptyset(&act.sa_mask);
    // NO SA_RESTART. without it the blocked read() comes back with EINTR when
    // the alarm fires, which is the only way the protocol ever sees a timeout
    act.sa_flags = 0;

    if (sigaction(SIGALRM, &act, NULL) == -1)
        return -1;

    return 0;
}

void startAlarm(unsigned seconds)
{
    alarmEnabled = 1;
    alarm(seconds);
}

void stopAlarm(void)
{
    alarm(0); // 0 cancels a pending alarm
    alarmEnabled = 0;
}

bool isAlarmActive(void) { return alarmEnabled != 0; }
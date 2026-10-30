#include "alarm_sigaction.h"

#include <signal.h>
#include <string.h>
#include <unistd.h> // alarm(2)

// The handler writes it and the protocol reads it, so sig_atomic_t. Nothing
// else is shared with the handler: printf(3) isn't async-signal-safe.
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
    // No SA_RESTART on purpose. The blocking read() has to come back with
    // EINTR, or the protocol never notices the timeout.
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
    alarm(0);
    alarmEnabled = 0;
}

bool isAlarmActive(void) { return alarmEnabled != 0; }
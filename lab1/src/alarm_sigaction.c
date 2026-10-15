// Alarm example using sigaction.
// This example shows how to configure an alarm using the sigaction function.
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]
//              Rui Prior [rcprior@fc.up.pt]

#include "alarm_sigaction.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static volatile int alarmEnabled = 0;
static int alarmCount = 0;

static void alarmHandler(int sig) {
    (void)sig;
    alarmEnabled = 0;
    alarmCount++;
    printf("Alarm #%d received\n", alarmCount);
}

int setupAlarmHandler(void) {
    struct sigaction act = {0};
    act.sa_handler = alarmHandler;
    act.sa_flags = 0; 
    if (sigaction(SIGALRM, &act, NULL) == -1)
        return -1;
    return 0;
}

void startAlarm(unsigned seconds) {
    alarm(seconds);
    alarmEnabled = 1;
}

void stopAlarm(void) {
    alarm(0);
    alarmEnabled = 0;
}

void resetAlarmCount(void) { alarmCount = 0; }
bool isAlarmActive(void) { return alarmEnabled != 0; }
int  getAlarmCount(void) { return alarmCount; }
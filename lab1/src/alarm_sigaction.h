#ifndef ALARM_SIGACTION_H
#define ALARM_SIGACTION_H

#include <stdbool.h>

// Install the SIGALRM handler. Must be called before the first startAlarm().
// Returns 0 on success, -1 on failure.
int setupAlarmHandler(void);

// Arm the timer for "seconds". The flag is set before the timer, so the handler
// always finds it set when it fires.
void startAlarm(unsigned seconds);

// Cancel a pending timer and clear the flag.
void stopAlarm(void);

// True while the alarm armed by startAlarm() has not fired yet. The protocol
// code uses it to tell "our own timeout interrupted the read" from "something
// else interrupted the read".
bool isAlarmActive(void);

#endif // ALARM_SIGACTION_H
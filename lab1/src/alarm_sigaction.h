#ifndef ALARM_SIGACTION_H
#define ALARM_SIGACTION_H

#include <stdbool.h>

// Installs the SIGALRM handler. Has to run before the first startAlarm().
// Returns 0 on success.
int setupAlarmHandler(void);

// Arms the timer. The flag goes up before the timer does, so the handler always
// finds it set.
void startAlarm(unsigned seconds);

// Cancels a pending timer and drops the flag.
void stopAlarm(void);

// True while the alarm armed by startAlarm() hasn't fired yet. The protocol
// uses it to tell our own timeout apart from some other signal interrupting a
// read.
bool isAlarmActive(void);

#endif // ALARM_SIGACTION_H
#ifndef ALARM_SIGACTION_H
#define ALARM_SIGACTION_H

#include <stdbool.h>

// Must be installed before the first startAlarm().
int setupAlarmHandler(void);

void startAlarm(unsigned seconds);

void stopAlarm(void);

// Tells our own timeout apart from any other signal interrupting a read.
bool isAlarmActive(void);

#endif // ALARM_SIGACTION_H
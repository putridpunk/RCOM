#ifndef ALARM_SIGACTION_H
#define ALARM_SIGACTION_H

#include <stdbool.h>

// installs the SIGALRM handler, has to run before the first startAlarm()
int setupAlarmHandler(void);

// arm it. flag goes up first so the handler can't miss it
void startAlarm(unsigned seconds);

// cancel a pending one
void stopAlarm(void);

// true while our alarm hasn't fired yet. this is how we tell our own timeout
// apart from any other signal that interrupts read()
bool isAlarmActive(void);

#endif // ALARM_SIGACTION_H
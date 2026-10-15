#ifndef ALARM_SIGACTION_H
#define ALARM_SIGACTION_H

#include <stdbool.h>

int setupAlarmHandler(void);
void startAlarm(unsigned seconds);
void stopAlarm(void);
void resetAlarmCount(void);
bool isAlarmActive(void);
int getAlarmCount(void);

#endif
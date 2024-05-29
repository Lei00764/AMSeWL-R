#ifndef SERVO_TASK_H
#define SERVO_TASK_H
#include "struct_typedef.h"

typedef enum{
cover_off,
cover_on,
}covermoni;

extern void servo_task(void const * argument);

#endif

#ifndef __TASK_USER_H__
#define __TASK_USER_H__

#include <stdint.h>

#define EVENT_FIFO_MAX 16
#define EVENT_SUBSCRIBE_MAX 4
#define TASK_EVENT_MAX 1

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _EVENT_TYPE {
  EVENT_TYPE_TICK_1MS,

  EVENT_TYPE_MAX,
} EVENT_TYPE;

typedef enum _TASK_ID {
  TASK_ID_MAIN_LOOP,
  TASK_ID_TIMER_1MS,

  TASK_ID_MAX,
} TASK_ID;

void *task_list_get(uint32_t *num);

#ifdef __cplusplus
}
#endif

#endif

#ifndef __TIMER_USER_H__
#define __TIMER_USER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _TIMER_TICK {
  TICK_1MS = 1,
  TICK_5MS = 5,
  TICK_25MS = 25,
  TICK_500MS = 500,
  TICK_1S = 1000,
  TICK_5S = 5000,
} TIMER_TICK;

void *timer_list_get(uint32_t *num);

#ifdef __cplusplus
}
#endif

#endif

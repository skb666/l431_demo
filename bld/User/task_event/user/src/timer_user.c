#include "common.h"
#include "timer.h"

static TIMER s_timer_list[] = {
    {
        .event_type = EVENT_TYPE_TICK_1MS,
        .priority = 0,
        .reload = TICK_1MS,
        .tick = TICK_1MS,
        .times = -1,
    },
};
const static uint32_t s_timer_list_size = ARRAY_SIZE(s_timer_list);

void *timer_list_get(uint32_t *num) {
  *num = s_timer_list_size;
  return s_timer_list;
}

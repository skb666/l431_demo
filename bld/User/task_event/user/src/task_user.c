#include "common.h"
#include "task.h"

extern void timer_1ms_init(void);
extern void timer_1ms_handle(TASK *task);
extern void main_loop_init(void);
extern void main_loop_handle(TASK *task);

static TASK s_task_list[] = {
    {
        .id = TASK_ID_MAIN_LOOP,
        .times = -1,
        .delay = 0,
        .init = main_loop_init,
        .handle = main_loop_handle,
    },
    {
        .id = TASK_ID_TIMER_1MS,
        .times = -1,
        .delay = 0,
        .init = timer_1ms_init,
        .handle = timer_1ms_handle,
    },
};
const static uint32_t s_task_list_size = ARRAY_SIZE(s_task_list);

void *task_list_get(uint32_t *num) {
  *num = s_task_list_size;
  return s_task_list;
}

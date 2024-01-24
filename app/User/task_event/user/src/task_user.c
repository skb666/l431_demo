#include "common.h"
#include "task.h"

extern void debug_print_init(void);
extern void debug_print_handle(TASK *task);
extern void key_scan_init(void);
extern void key_scan_handle(TASK *task);
extern void timer_1ms_init(void);
extern void timer_1ms_handle(TASK *task);
extern void main_loop_init(void);
extern void main_loop_handle(TASK *task);
extern void led_blink_init(void);
extern void led_blink_handle(TASK *task);

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
    {
        .id = TASK_ID_DEBUG_PRINT,
        .times = -1,
        .delay = 0,
        .init = debug_print_init,
        .handle = debug_print_handle,
    },
    {
        .id = TASK_ID_KEY_SCAN,
        .times = -1,
        .delay = 1,
        .init = key_scan_init,
        .handle = key_scan_handle,
    },
    {
        .id = TASK_ID_LED_BLINK,
        .times = -1,
        .delay = 2,
        .init = led_blink_init,
        .handle = led_blink_handle,
    },
};
const static uint32_t s_task_list_size = ARRAY_SIZE(s_task_list);

void *task_list_get(uint32_t *num) {
  *num = s_task_list_size;
  return s_task_list;
}

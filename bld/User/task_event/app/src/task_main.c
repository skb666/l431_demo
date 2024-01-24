#include <stdio.h>

#include "device.h"
#include "main.h"
#include "task.h"
#include "timer.h"

typedef enum {
  FRAME_TYPE_DEBUG = 0,
  FRAME_TYPE_UPDATE_DATA = 1,
  FRAME_TYPE_UPDATE_STATUS = 2,
  FRAME_TYPE_REBOOT = 3,
  FRAME_TYPE_MAX,
} FRAME_TYPE;

extern void update_status_get(frame_parse_t *frame);
extern void update_frame_parse(frame_parse_t *frame);
extern void update_pkg_process(void);

static void task_event_process(TASK *task, void (*callback)(EVENT *)) {
  int8_t err;
  EVENT *ev;

  if (event_empty(&task->events)) {
    return;
  }

  while (event_count(&task->events)) {
    err = event_peek(&task->events, &ev);
    if (err) {
      return;
    }

    callback(ev);

    event_pop_only(&task->events);
    task_update_times(task);
  }
}

/* 1ms 周期任务 */
void timer_1ms_init(void) {
  task_event_subscribe(EVENT_TYPE_TICK_1MS, TASK_ID_TIMER_1MS);
}

static void timer_1ms_cb(EVENT *ev) {
  switch (ev->type) {
    case EVENT_TYPE_TICK_1MS: {
      // 串口发送
      uart_tx_poll(DEV_USART1);
    } break;
    default: {
    } break;
  }
}

void timer_1ms_handle(TASK *task) {
  task_event_process(task, timer_1ms_cb);
}

/* 循环任务 */
static void print_frame_usart1(frame_parse_t *frame) {
  uart_puts(DEV_USART1, frame->data, frame->length);
}

static void system_ctrl_reboot(frame_parse_t *frame) {
  uart_printf(DEV_USART1, "system_ctrl_reboot\r\n");
  NVIC_SystemReset();
}

void main_loop_init(void) {
  frame_parse_register(DEV_USART1, FRAME_TYPE_DEBUG, print_frame_usart1);
  frame_parse_register(DEV_USART1, FRAME_TYPE_UPDATE_DATA, update_frame_parse);
  frame_parse_register(DEV_USART1, FRAME_TYPE_UPDATE_STATUS, update_status_get);
  frame_parse_register(DEV_USART1, FRAME_TYPE_REBOOT, system_ctrl_reboot);
}

void main_loop_handle(TASK *task) {
  // USART1 帧解析
  uart_frame_parse(DEV_USART1);

  // 固件升级
  update_pkg_process();

  task_update_times(task);
}

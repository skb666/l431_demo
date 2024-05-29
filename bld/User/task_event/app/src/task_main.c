#include <stdio.h>

#include "common.h"
#include "i2c_slave.h"
#include "main.h"
#include "param.h"
#include "task.h"
#include "timer.h"
#include "uart_device.h"
#include "update.h"

typedef enum {
  FRAME_TYPE_DEBUG = 0x00,
  FRAME_TYPE_SYSTEM_CTRL = 0xF0,
  FRAME_TYPE_UPDATE_DATA = 0xF1,
  FRAME_TYPE_UPDATE_STATUS = 0xF2,
  FRAME_TYPE_MAX = FUNC_LIST_MAX,
} FRAME_TYPE;

extern void system_ctrl_frame_parse(frame_parse_t *frame);
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
      // i2c 异常检测
      i2c_abnormal_check(300);
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
static void print_frame_usart(frame_parse_t *frame) {
  uart_puts_wait(frame->dev_type, frame->data, frame->length);
}

static void system_ctrl_check(void) {
  SYS_PARAM *sys = sys_param_get();
  BOOT_PARAM param;

  switch (sys->ctrl.system) {
    case SYSTEM_CTRL_REBOOT: {
      printf_dbg("SYSTEM_CTRL_REBOOT\r\n");
      LL_mDelay(500);
      NVIC_SystemReset();
    } break;
    case SYSTEM_CTRL_BOOT_APP: {
      boot_param_get(&param);

      if ((param.app_status == STATUS_NORM) || (param.app_status == STATUS_NONE)) {
        param.update_needed = 0;
        param.app_status = STATUS_BOOT;
        if (boot_param_update(&param)) {
          Error_Handler();
        }
      } else {
        sys->ctrl.system = SYSTEM_CTRL_NONE;
      }

      printf_dbg("SYSTEM_CTRL_BOOT_APP\r\n");
      boot_param_check(0);
    } break;
    default: {
    } break;
  }
}

void main_loop_init(void) {
  frame_parse_register(DEV_USART1, FRAME_TYPE_DEBUG, print_frame_usart);
  frame_parse_register(DEV_USART1, FRAME_TYPE_SYSTEM_CTRL, system_ctrl_frame_parse);
  frame_parse_register(DEV_USART1, FRAME_TYPE_UPDATE_DATA, update_frame_parse);
  frame_parse_register(DEV_USART1, FRAME_TYPE_UPDATE_STATUS, update_status_get);
}

void main_loop_handle(TASK *task) {
  // USART1 帧解析
  uart_frame_parse(DEV_USART1);

  // 固件升级
  update_pkg_process();

  // 系统控制
  system_ctrl_check();

  task_update_times(task);
}

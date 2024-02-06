#include <stdio.h>

#include "combo_key.h"
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
  FRAME_TYPE_I2C_WRITE = 0xE1,
  FRAME_TYPE_I2C_READ = 0xE2,
  FRAME_TYPE_SYSTEM_CTRL = 0xF0,
  FRAME_TYPE_UPDATE_DATA = 0xF1,
  FRAME_TYPE_UPDATE_STATUS = 0xF2,
  FRAME_TYPE_MAX = FUNC_LIST_MAX,
} FRAME_TYPE;

extern void system_ctrl_frame_parse(frame_parse_t *frame);
extern void update_status_get(frame_parse_t *frame);
extern void update_frame_parse(frame_parse_t *frame);
extern void update_pkg_process(void);
extern void uart_frame_i2c_write(frame_parse_t *frame);
extern void uart_frame_i2c_read(frame_parse_t *frame);

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

/* 调试信息打印 */
void debug_print_init(void) {
  // task_event_subscribe(EVENT_TYPE_KEY_PRESS, TASK_ID_DEBUG_PRINT);
  task_event_subscribe(EVENT_TYPE_KEY_RELEASE, TASK_ID_DEBUG_PRINT);
  task_event_subscribe(EVENT_TYPE_KEY_LONG_PRESS, TASK_ID_DEBUG_PRINT);
  // task_event_subscribe(EVENT_TYPE_KEY_LONG_RELEASE, TASK_ID_DEBUG_PRINT);
  // task_event_subscribe(EVENT_TYPE_KEY_COMBO, TASK_ID_DEBUG_PRINT);
  task_event_subscribe(EVENT_TYPE_KEY_COMBO_RELEASE, TASK_ID_DEBUG_PRINT);
}

static void debug_print_cb(EVENT *ev) {
  switch (ev->type) {
    case EVENT_TYPE_KEY_PRESS: {
      printf_dbg("[KEY]: PRESS\r\n");
    } break;
    case EVENT_TYPE_KEY_RELEASE: {
      printf_dbg("[KEY]: RELEASE\r\n");
    } break;
    case EVENT_TYPE_KEY_LONG_PRESS: {
      printf_dbg("[KEY]: LONG_PRESS %hu\r\n", (size_t)ev->custom_data);
    } break;
    case EVENT_TYPE_KEY_LONG_RELEASE: {
      printf_dbg("[KEY]: LONG_RELEASE %hu\r\n", (size_t)ev->custom_data);
    } break;
    case EVENT_TYPE_KEY_COMBO: {
      printf_dbg("[KEY]: COMBO %hu\r\n", (size_t)ev->custom_data);
    } break;
    case EVENT_TYPE_KEY_COMBO_RELEASE: {
      printf_dbg("[KEY]: COMBO_RELEASE %hu\r\n", (size_t)ev->custom_data);
    } break;
    default: {
    } break;
  }
}

void debug_print_handle(TASK *task) {
  task_event_process(task, debug_print_cb);
}

/* 按键扫描 */
static KEY_VALUE getKey(void) {
  if (LL_GPIO_IsInputPinSet(KEY_GPIO_Port, KEY_Pin) == 1) {
    return K_PRESS;
  } else {
    return K_RELEASE;
  }
}

void key_scan_init(void) {
  task_event_subscribe(EVENT_TYPE_TICK_5MS, TASK_ID_KEY_SCAN);
  key_register(0, getKey, NULL, 4, 75, 200);
}

static void key_scan_cb(EVENT *ev) {
  KEY *key_list;
  int num;

  switch (ev->type) {
    case EVENT_TYPE_TICK_5MS: {
      key_list = key_list_get(&num);
      for (int i = 0; i < num; ++i) {
        switch (combo_key_event_check(&key_list[i])) {
          case KE_PRESS: {
            task_event_publish(EVENT_TYPE_KEY_PRESS, NULL, 0);
          } break;
          case KE_RELEASE: {
            task_event_publish(EVENT_TYPE_KEY_RELEASE, NULL, 0);
          } break;
          case KE_LONG_PRESS: {
            task_event_publish(EVENT_TYPE_KEY_LONG_PRESS, (void *)(size_t)key_combo_release_count(&key_list[i]), 0);
          } break;
          case KE_LONG_RELEASE: {
            task_event_publish(EVENT_TYPE_KEY_LONG_RELEASE, (void *)(size_t)key_combo_release_count(&key_list[i]), 0);
          } break;
          case KE_COMBO: {
            task_event_publish(EVENT_TYPE_KEY_COMBO, (void *)(size_t)key_combo_press_count(&key_list[i]), 0);
          } break;
          case KE_COMBO_RELEASE: {
            task_event_publish(EVENT_TYPE_KEY_COMBO_RELEASE, (void *)(size_t)key_combo_press_count(&key_list[i]), 0);
          } break;
          default: {
          } break;
        }
      }
      task_delay_ms(TASK_ID_KEY_SCAN, 5);
    } break;
    default: {
    } break;
  }
}

void key_scan_handle(TASK *task) {
  task_event_process(task, key_scan_cb);
}

/* LED 闪烁 */
void led_blink_init(void) {
  task_event_subscribe(EVENT_TYPE_TICK_500MS, TASK_ID_LED_BLINK);
}

static void led_blink_cb(EVENT *ev) {
  switch (ev->type) {
    case EVENT_TYPE_TICK_500MS: {
      LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      task_delay_ms(TASK_ID_LED_BLINK, 500);
    } break;
    default: {
    } break;
  }
}

void led_blink_handle(TASK *task) {
  task_event_process(task, led_blink_cb);
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
static void print_frame_usart1(frame_parse_t *frame) {
  uart_puts(DEV_USART1, frame->data, frame->length);
}

static void system_ctrl_check(void) {
  SYS_PARAM *sys = sys_param_get();
  BOOT_PARAM param;

  switch (sys->ctrl.system) {
    case SYSTEM_CTRL_REBOOT: {
      uart_printf(DEV_USART1, "SYSTEM_CTRL_REBOOT\r\n");
      LL_mDelay(500);
      NVIC_SystemReset();
    } break;
    case SYSTEM_CTRL_UPDATE_START: {
      boot_param_get(&param);

      param.update_needed = 1;
      if (boot_param_update(&param)) {
        Error_Handler();
      }

      uart_printf(DEV_USART1, "SYSTEM_CTRL_UPDATE_START\r\n");
      NVIC_SystemReset();
    } break;
    default: {
    } break;
  }
}

void main_loop_init(void) {
  frame_parse_register(DEV_USART1, FRAME_TYPE_DEBUG, print_frame_usart1);
  frame_parse_register(DEV_USART1, FRAME_TYPE_I2C_WRITE, uart_frame_i2c_write);
  frame_parse_register(DEV_USART1, FRAME_TYPE_I2C_READ, uart_frame_i2c_read);
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

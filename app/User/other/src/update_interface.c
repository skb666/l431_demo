#include <stdio.h>
#include <string.h>

#include "common.h"
#include "device.h"
#include "main.h"
#include "param.h"
#include "ring_fifo.h"
#include "update.h"

extern UPDATE_PKG g_update_pkg;
ring_define_static(uint8_t, update_data_buf, (sizeof(uint16_t) + sizeof(PKG_DATA) + UPDATE_PACKAGE_MAX_SIZE), 0);

void update_status_get(frame_parse_t *frame) {
  SYS_PARAM *sys = sys_param_get();
  uint16_t update_status = sys->ctrl.update.status;
  if (frame->byte_order) {
    change_byte_order(&update_status, sizeof(update_status));
  }
  uart_puts(DEV_USART1, (uint8_t *)&update_status, sizeof(update_status));
}

void update_frame_parse(frame_parse_t *frame) {
  SYS_PARAM *sys = sys_param_get();

  if (sys->ctrl.update.need_process) {
    return;
  }

  if (frame->length < 2) {
    return;
  }

  /* 获取升级包数据 */
  disable_global_irq();
  ring_reset(&update_data_buf);
  ring_push_mult(&update_data_buf, frame->data, frame->length);
  enable_global_irq();

  /* 获取升级包类型 */
  ring_pop_mult(&update_data_buf, &g_update_pkg.type, sizeof(g_update_pkg.type));
  if (frame->byte_order) {
    change_byte_order(&g_update_pkg.type, sizeof(g_update_pkg.type));
  }

  switch (g_update_pkg.type) {
    case PKG_TYPE_INIT:
    case PKG_TYPE_FINISH: {
      g_update_pkg.head = NULL;
    } break;
    case PKG_TYPE_HEAD: {
      if (ring_size(&update_data_buf) < sizeof(PKG_HEAD)) {
        return;
      }
      g_update_pkg.head = (PKG_HEAD *)ring_peek(&update_data_buf);
      if (frame->byte_order) {
        change_byte_order(&g_update_pkg.head->file_crc, sizeof(g_update_pkg.head->file_crc));
        change_byte_order(&g_update_pkg.head->file_size_real, sizeof(g_update_pkg.head->file_size_real));
        change_byte_order(&g_update_pkg.head->data_size_one, sizeof(g_update_pkg.head->data_size_one));
        change_byte_order(&g_update_pkg.head->pkg_num_total, sizeof(g_update_pkg.head->pkg_num_total));
      }
    } break;
    case PKG_TYPE_DATA: {
      if (ring_size(&update_data_buf) < sizeof(PKG_DATA)) {
        return;
      }
      g_update_pkg.data = (PKG_DATA *)ring_peek(&update_data_buf);
      if (frame->byte_order) {
        change_byte_order(&g_update_pkg.data->pkg_crc, sizeof(g_update_pkg.data->pkg_crc));
        change_byte_order(&g_update_pkg.data->pkg_num, sizeof(g_update_pkg.data->pkg_num));
        change_byte_order(&g_update_pkg.data->data_len, sizeof(g_update_pkg.data->data_len));
      }
    } break;
    default: {
      return;
    } break;
  }

  /* 升级包准备好后置位 */
  disable_global_irq();
  sys->ctrl.update.need_process = 1;
  enable_global_irq();
}

void system_ctrl_reboot(frame_parse_t *frame) {
  uart_printf(DEV_USART1, "system_ctrl_reboot\r\n");
  NVIC_SystemReset();
}

void update_start(frame_parse_t *frame) {
  BOOT_PARAM param;

  boot_param_get(&param);

  param.update_needed = 1;
  if (boot_param_update(&param)) {
    Error_Handler();
  }

  NVIC_SystemReset();
}

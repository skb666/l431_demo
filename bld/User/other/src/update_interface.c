#include <stdio.h>
#include <string.h>

#include "common.h"
#include "main.h"
#include "param.h"
#include "uart_device.h"
#include "update.h"

extern UPDATE_PKG g_update_pkg;

void update_status_get(frame_parse_t *frame) {
  SYS_PARAM *sys = sys_param_get();
  uint16_t update_status = sys->ctrl.update.status;
  if (frame->byte_order) {
    change_byte_order(&update_status, sizeof(update_status));
  }
  uart_puts_wait(frame->dev_type, (uint8_t *)&update_status, sizeof(update_status));
}

void update_frame_parse(frame_parse_t *frame) {
  SYS_PARAM *sys = sys_param_get();
  uint8_t *frame_data;
  uint16_t frame_length;

  if (sys->ctrl.update.need_process) {
    return;
  }

  if (frame->length < sizeof(g_update_pkg.type)) {
    return;
  }

  /* 获取升级包信息 */
  frame_data = frame->data;
  frame_length = frame->length;

  /* 获取升级包类型 */
  memcpy(&g_update_pkg.type, frame_data, sizeof(g_update_pkg.type));
  frame_data += sizeof(g_update_pkg.type);
  frame_length -= sizeof(g_update_pkg.type);
  if (frame->byte_order) {
    change_byte_order(&g_update_pkg.type, sizeof(g_update_pkg.type));
  }

  switch (g_update_pkg.type) {
    case PKG_TYPE_INIT:
    case PKG_TYPE_FINISH: {
      memset(&g_update_pkg.data, 0xFF, sizeof(PKG_DATA));
    } break;
    case PKG_TYPE_HEAD: {
      if (frame_length != sizeof(PKG_HEAD)) {
        return;
      }
      memset(&g_update_pkg.data, 0xFF, sizeof(PKG_DATA));
      memcpy(&g_update_pkg.head, frame_data, frame_length);
      if (frame->byte_order) {
        change_byte_order(&g_update_pkg.head.partition_type, sizeof(g_update_pkg.head.partition_type));
        change_byte_order(&g_update_pkg.head.file_crc, sizeof(g_update_pkg.head.file_crc));
        change_byte_order(&g_update_pkg.head.file_size_real, sizeof(g_update_pkg.head.file_size_real));
        change_byte_order(&g_update_pkg.head.data_size_one, sizeof(g_update_pkg.head.data_size_one));
        change_byte_order(&g_update_pkg.head.pkg_num_total, sizeof(g_update_pkg.head.pkg_num_total));
      }
    } break;
    case PKG_TYPE_DATA: {
      if ((frame_length < sizeof(PKG_DATA) - UPDATE_PACKAGE_MAX_SIZE) || (frame_length > sizeof(PKG_DATA))) {
        return;
      }
      memset(&g_update_pkg.data, 0xFF, sizeof(PKG_DATA));
      memcpy(&g_update_pkg.data, frame_data, frame_length);
      if (frame->byte_order) {
        change_byte_order(&g_update_pkg.data.pkg_crc, sizeof(g_update_pkg.data.pkg_crc));
        change_byte_order(&g_update_pkg.data.pkg_num, sizeof(g_update_pkg.data.pkg_num));
        change_byte_order(&g_update_pkg.data.data_len, sizeof(g_update_pkg.data.data_len));
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

void system_ctrl_frame_parse(frame_parse_t *frame) {
  SYS_PARAM *sys = sys_param_get();
  uint16_t value;

  if (frame->length < sizeof(value)) {
    return;
  }

  memcpy(&value, frame->data, sizeof(value));
  if (frame->byte_order) {
    change_byte_order(&value, sizeof(value));
  }

  sys->ctrl.system = value;
}

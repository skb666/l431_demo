#include "i2c_protocol.h"

#include "common.h"
#include "device.h"
#include "i2c_slave.h"
#include "main.h"
#include "param.h"
#include "update.h"

#define I2C_PUT_NUM(Type, Value)                         \
  do {                                                   \
    Type __value = Value;                                \
    change_byte_order(&__value, sizeof(Type));           \
    i2c_slave_tx_put((uint8_t *)&__value, sizeof(Type)); \
  } while (0)

#define I2C_GET_NUM(Type, Value)                \
  do {                                          \
    uint8_t __data;                             \
    Value = 0;                                  \
    for (size_t i = 0; i < sizeof(Type); ++i) { \
      if (i2c_slave_rx_get(&__data, 1)) {       \
        Value <<= 8;                            \
        Value |= __data;                        \
      }                                         \
    }                                           \
  } while (0)

extern UPDATE_PKG g_update_pkg;

void reg_read_cb_version(void) {
  I2C_PUT_NUM(uint16_t, MCU_SOFTWARE_VERSION);
}

void reg_read_cb_system_ctrl(void) {
  SYS_PARAM *sys = sys_param_get();

  I2C_PUT_NUM(uint16_t, sys->ctrl.system);
}

void reg_write_cb_system_ctrl(void) {
  SYS_PARAM *sys = sys_param_get();
  uint16_t value;

  if (i2c_slave_rx_size() < sizeof(uint16_t)) {
    return;
  }

  I2C_GET_NUM(uint16_t, value);
  sys->ctrl.system = value;
}

void reg_write_cb_update_data(void) {
  SYS_PARAM *sys = sys_param_get();

  if (sys->ctrl.update.need_process) {
    return;
  }

  if (i2c_slave_rx_size() < sizeof(g_update_pkg.type)) {
    return;
  }

  /* 获取升级包类型 */
  I2C_GET_NUM(uint16_t, g_update_pkg.type);

  switch (g_update_pkg.type) {
    case PKG_TYPE_INIT:
    case PKG_TYPE_FINISH: {
      memset(&g_update_pkg.data, 0xFF, sizeof(PKG_DATA));
    } break;
    case PKG_TYPE_HEAD: {
      if (i2c_slave_rx_size() != sizeof(PKG_HEAD)) {
        return;
      }
      memset(&g_update_pkg.data, 0xFF, sizeof(PKG_DATA));
      i2c_slave_rx_get((uint8_t *)&g_update_pkg.head, i2c_slave_rx_size());
      change_byte_order(&g_update_pkg.head.file_crc, sizeof(g_update_pkg.head.file_crc));
      change_byte_order(&g_update_pkg.head.file_size_real, sizeof(g_update_pkg.head.file_size_real));
      change_byte_order(&g_update_pkg.head.data_size_one, sizeof(g_update_pkg.head.data_size_one));
      change_byte_order(&g_update_pkg.head.pkg_num_total, sizeof(g_update_pkg.head.pkg_num_total));
    } break;
    case PKG_TYPE_DATA: {
      if ((i2c_slave_rx_size() < sizeof(PKG_DATA) - UPDATE_PACKAGE_MAX_SIZE) || (i2c_slave_rx_size() > sizeof(PKG_DATA))) {
        return;
      }
      memset(&g_update_pkg.data, 0xFF, sizeof(PKG_DATA));
      i2c_slave_rx_get((uint8_t *)&g_update_pkg.data, i2c_slave_rx_size());
      change_byte_order(&g_update_pkg.data.pkg_crc, sizeof(g_update_pkg.data.pkg_crc));
      change_byte_order(&g_update_pkg.data.pkg_num, sizeof(g_update_pkg.data.pkg_num));
      change_byte_order(&g_update_pkg.data.data_len, sizeof(g_update_pkg.data.data_len));
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

void reg_read_cb_update_status(void) {
  SYS_PARAM *sys = sys_param_get();

  I2C_PUT_NUM(uint16_t, sys->ctrl.update.status);
}

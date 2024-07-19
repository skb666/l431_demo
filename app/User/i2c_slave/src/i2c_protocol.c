#include "i2c_protocol.h"

#include <string.h>

#include "common.h"
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

static GPIO_TypeDef *s_GPIOx = NULL;
static uint32_t s_Pin;

void reg_read_cb_version(void) {
  SYS_PARAM *sys = sys_param_get();

  I2C_PUT_NUM(uint16_t, sys->ctrl.update.version);
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
    case PKG_TYPE_INIT: {
      if (i2c_slave_rx_size() != sizeof(PKG_INIT)) {
        return;
      }
      memset(&g_update_pkg.data, 0xFF, sizeof(PKG_DATA));
      i2c_slave_rx_get((uint8_t *)&g_update_pkg.init, i2c_slave_rx_size());
      change_byte_order(&g_update_pkg.init.partition_type, sizeof(g_update_pkg.init.partition_type));
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
    case PKG_TYPE_FINISH: {
      memset(&g_update_pkg.data, 0xFF, sizeof(PKG_DATA));
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

void reg_write_cb_gpio_ctrl(void) {
  uint16_t value;

  if (i2c_slave_rx_size() < sizeof(uint16_t)) {
    return;
  }

  I2C_GET_NUM(uint16_t, value);

  switch ((value >> 12) & 0xF) {
    case 0x0: {
      s_GPIOx = GPIOA;
    } break;
    case 0x1: {
      s_GPIOx = GPIOB;
    } break;
    case 0x2: {
      s_GPIOx = GPIOC;
    } break;
    case 0x3: {
      s_GPIOx = GPIOD;
    } break;
    case 0x7: {
      s_GPIOx = GPIOH;
    } break;
    default: {
      s_GPIOx = NULL;
    }
  }

  switch ((value >> 8) & 0xF) {
    case 0x0: {
      s_Pin = LL_GPIO_PIN_0;
    } break;
    case 0x1: {
      s_Pin = LL_GPIO_PIN_1;
    } break;
    case 0x2: {
      s_Pin = LL_GPIO_PIN_2;
    } break;
    case 0x3: {
      s_Pin = LL_GPIO_PIN_3;
    } break;
    case 0x4: {
      s_Pin = LL_GPIO_PIN_4;
    } break;
    case 0x5: {
      s_Pin = LL_GPIO_PIN_5;
    } break;
    case 0x6: {
      s_Pin = LL_GPIO_PIN_6;
    } break;
    case 0x7: {
      s_Pin = LL_GPIO_PIN_7;
    } break;
    case 0x8: {
      s_Pin = LL_GPIO_PIN_8;
    } break;
    case 0x9: {
      s_Pin = LL_GPIO_PIN_9;
    } break;
    case 0xa: {
      s_Pin = LL_GPIO_PIN_10;
    } break;
    case 0xb: {
      s_Pin = LL_GPIO_PIN_11;
    } break;
    case 0xc: {
      s_Pin = LL_GPIO_PIN_12;
    } break;
    case 0xd: {
      s_Pin = LL_GPIO_PIN_13;
    } break;
    case 0xe: {
      s_Pin = LL_GPIO_PIN_14;
    } break;
    case 0xf: {
      s_Pin = LL_GPIO_PIN_15;
    } break;
    default: {
      s_Pin = 0;
    }
  }

  switch (value & 0xFF) {
    case 0: {
      if (LL_GPIO_MODE_OUTPUT == LL_GPIO_GetPinMode(s_GPIOx, s_Pin)) {
        LL_GPIO_ResetOutputPin(s_GPIOx, s_Pin);
      }
    } break;
    case 1: {
      if (LL_GPIO_MODE_OUTPUT == LL_GPIO_GetPinMode(s_GPIOx, s_Pin)) {
        LL_GPIO_SetOutputPin(s_GPIOx, s_Pin);
      }
    } break;
    default: {
    }
  }
}

void reg_read_cb_gpio_ctrl(void) {
  uint16_t gpio_level = 0xFFFF;

  if ((s_GPIOx != NULL) && (s_Pin != 0)) {
    if (LL_GPIO_MODE_INPUT == LL_GPIO_GetPinMode(s_GPIOx, s_Pin)) {
      gpio_level = (uint16_t)LL_GPIO_IsInputPinSet(s_GPIOx, s_Pin);
    } else if (LL_GPIO_MODE_OUTPUT == LL_GPIO_GetPinMode(s_GPIOx, s_Pin)) {
      gpio_level = (uint16_t)LL_GPIO_IsOutputPinSet(s_GPIOx, s_Pin);
    }
  }

  I2C_PUT_NUM(uint16_t, gpio_level);
}

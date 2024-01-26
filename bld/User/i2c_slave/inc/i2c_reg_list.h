#ifndef __I2C_REG_LIST_H__
#define __I2C_REG_LIST_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  REG_VERSION = 0x0000,
  REG_SYSTEM_CTRL = 0x0001,
  REG_UPDATE_DATA = 0xFF01,
  REG_UPDATE_STATUS = 0xFF02,
  REG_MAX = 0xFFFF,
} REG_NAME;

void *reg_list_get(uint32_t *num);

#ifdef __cplusplus
}
#endif

#endif

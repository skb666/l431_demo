#ifndef __PARAM_H__
#define __PARAM_H__

#include <stdint.h>

/* VERSION: 1.1.1.0 */
#define MCU_SOFTWARE_VERSION (0x1110)

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  SYSTEM_CTRL_NONE = 0,
  SYSTEM_CTRL_REBOOT = 1,
  SYSTEM_CTRL_BOOT_APP = 2,
  SYSTEM_CTRL_UPDATE_START = 3,
} SYSTEM_CTRL;

typedef struct {
  uint16_t need_process;
  uint16_t stage;
  uint16_t status;
} UPDATE_CTRL;

typedef struct {
  uint16_t system;
  UPDATE_CTRL update;
} SYS_CTRL;

typedef struct {
  SYS_CTRL ctrl;
} SYS_PARAM;

SYS_PARAM *sys_param_get(void);
void sys_param_init(void);

#ifdef __cplusplus
}
#endif

#endif

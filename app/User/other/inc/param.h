#ifndef __PARAM_H__
#define __PARAM_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint16_t need_process;
  uint16_t stage;
  uint16_t status;
} UPDATE_CTRL;

typedef struct {
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

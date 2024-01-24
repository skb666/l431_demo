#include "param.h"

#include <string.h>

static SYS_PARAM sys_param = {0};

SYS_PARAM *sys_param_get(void) {
  return &sys_param;
}

void sys_param_init(void) {
  memset(&sys_param, 0, sizeof(SYS_PARAM));

  sys_param.ctrl.update.need_process = 0;
  sys_param.ctrl.update.stage = 0;
  sys_param.ctrl.update.status = 0x0FFF;
}

#include "param.h"

#include <string.h>

const static struct __attribute__((aligned(2))) {
  uint16_t support : 4;
  uint16_t type : 4;
  uint16_t version : 4;
  uint16_t partition : 4;
} update_version = {
    .partition = VERSION_PARTITION,
    .version = VERSION_IAP,
    .type = VERSION_UPDATE_TYPE,
    .support = VERSION_SUPPORT,
};

static SYS_PARAM sys_param = {0};

SYS_PARAM *sys_param_get(void) {
  return &sys_param;
}

void sys_param_init(void) {
  memset(&sys_param, 0, sizeof(SYS_PARAM));

  sys_param.ctrl.update.version = *(uint16_t *)&update_version;
  sys_param.ctrl.update.need_process = 0;
  sys_param.ctrl.update.stage = 0;
  sys_param.ctrl.update.status = 0x0FFF;
}

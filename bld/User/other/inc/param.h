#ifndef __PARAM_H__
#define __PARAM_H__

#include <stdint.h>

#define VERSION_IAP (1)
#define VERSION_SUPPORT (0)

#ifdef PROGRAM_BLD
#define VERSION_PARTITION (0)
#else
#define VERSION_PARTITION (1)
#endif

#if defined(USING_UPDATE_OVERWRITE)
#define VERSION_UPDATE_TYPE (0)
#elif defined(USING_UPDATE_BACKUP_OVERWRITE)
#define VERSION_UPDATE_TYPE (1)
#elif defined(USING_UPDATE_BACKUP_IN_BLD)
#define VERSION_UPDATE_TYPE (2)
#elif defined(USING_UPDATE_BACKUP_IN_APP)
#define VERSION_UPDATE_TYPE (3)
#else
#define VERSION_UPDATE_TYPE (0)
#endif

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
  uint16_t version;
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

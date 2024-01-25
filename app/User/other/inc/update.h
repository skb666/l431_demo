#ifndef __UPDATE_H__
#define __UPDATE_H__

#include <stdint.h>

#include "onchip_flash.h"

#define ADDR_BASE_BLD (STMFLASH_BASE)
#define ADDR_BASE_PARAM (ADDR_FLASH_PAGE_16)
#define ADDR_BASE_PARAM_BAK (ADDR_FLASH_PAGE_17)
#define ADDR_BASE_APP (ADDR_FLASH_PAGE_18)
#define ADDR_BASE_APP_BAK (ADDR_FLASH_PAGE_73)

#define PART_SIZE_BLD ((ADDR_BASE_PARAM) - (ADDR_BASE_BLD))          // 32KB
#define PART_SIZE_PARAM ((ADDR_BASE_PARAM_BAK) - (ADDR_BASE_PARAM))  // 2KB
#define PART_SIZE_APP ((ADDR_BASE_APP_BAK) - (ADDR_BASE_APP))        // 110KB

#define UPDATE_PACKAGE_MAX_SIZE 1024

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  STATUS_NONE = 0,  // APP 不存在
  STATUS_RECV,      // APP 接收中
  STATUS_LOAD,      // APP 载入中
  STATUS_BOOT,      // APP 可引导
  STATUS_NORM,      // APP 可运行
} APP_STATUS;

typedef struct {
  uint32_t update_needed;  // 更新标记
  uint32_t app_status;     // APP 的状态
  uint32_t back_to_app;    // 是否允许回到 APP
  uint32_t crc_val;        // 引导参数的 CRC 校验值
} BOOT_PARAM;

typedef enum {
  PKG_TYPE_INIT = 0x1000,
  PKG_TYPE_FINISH = 0x0FFE,
  PKG_TYPE_HEAD = 0x0001,
  PKG_TYPE_DATA = 0x0002,
} PKG_TYPE;

typedef struct {
  uint32_t file_crc;        // 升级文件的 CRC 值
  uint32_t file_size_real;  // 升级文件实际大小
  uint16_t data_size_one;   // 单次传输的升级数据大小
  uint16_t pkg_num_total;   // 升级包数量
} PKG_HEAD;

typedef struct {
  uint32_t pkg_crc;   // 升级数据 CRC
  uint16_t pkg_num;   // 当前包号
  uint16_t data_len;  // 携带的数据长度
  uint8_t data[];     // 升级数据
} PKG_DATA;

typedef struct {
  uint16_t type;
  union {
    PKG_HEAD *head;
    PKG_DATA *data;
  };
} UPDATE_PKG;

int8_t boot_param_update(BOOT_PARAM *param);
void boot_param_get(BOOT_PARAM *pdata);
void boot_param_check(void);

#ifdef __cplusplus
}
#endif

#endif

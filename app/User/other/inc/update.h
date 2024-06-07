#ifndef __UPDATE_H__
#define __UPDATE_H__

#include <stdint.h>

#include "onchip_flash.h"

#if defined(USING_UPDATE_BACKUP_OVERWRITE) || defined(USING_UPDATE_BACKUP_IN_BLD) || defined(USING_UPDATE_BACKUP_IN_APP)
#ifndef UPDATE_SUPPORT_BACKUP
#define UPDATE_SUPPORT_BACKUP
#endif
#endif

#define ADDR_BASE_BLD (STMFLASH_BASE)
#define ADDR_BASE_PARAM (ADDR_FLASH_PAGE_16)
#define ADDR_BASE_PARAM_BAK (ADDR_FLASH_PAGE_17)
#define ADDR_BASE_APP (ADDR_FLASH_PAGE_18)
#ifdef UPDATE_SUPPORT_BACKUP
#define ADDR_BASE_BACKUP (ADDR_FLASH_PAGE_71)
#endif
#define ADDR_BASE_CUSTOM (ADDR_FLASH_PAGE_124)

#define PART_SIZE_BLD ((ADDR_BASE_PARAM) - (ADDR_BASE_BLD))          // 32KB
#define PART_SIZE_PARAM ((ADDR_BASE_PARAM_BAK) - (ADDR_BASE_PARAM))  // 2KB
#ifdef UPDATE_SUPPORT_BACKUP
#define PART_SIZE_APP ((ADDR_BASE_BACKUP) - (ADDR_BASE_APP))  // 106KB
#else
#define PART_SIZE_APP ((ADDR_BASE_CUSTOM) - (ADDR_BASE_APP))  // 212KB
#endif
#define PART_SIZE_CUSTOM ((STMFLASH_END) - (ADDR_BASE_CUSTOM))

#define UPDATE_PACKAGE_MAX_SIZE 1024

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  UPDATE_OVERWRITE,  // 覆盖升级
#ifdef UPDATE_SUPPORT_BACKUP
  UPDATE_BACKUP,  // 备份升级
#endif
} UPDATE_TYPE;

typedef enum {
  STATUS_NONE = 0,  // APP 不存在
  STATUS_RECV,      // APP 接收中
  STATUS_LOAD,      // APP 载入中
  STATUS_BOOT,      // APP 可引导
  STATUS_NORM,      // APP 可运行
} APP_STATUS;

typedef struct {
  volatile uint32_t update_needed;  // 更新标记
  volatile uint32_t app_status;     // APP 的状态
  volatile uint32_t back_to_app;    // 是否允许回到 APP
  volatile uint32_t crc_val_v1;     // 引导参数的 CRC 校验值
} __attribute__((aligned(FLASH_DATA_ALIGN))) BOOT_PARAM_V1;

typedef struct {
  volatile uint32_t update_needed;  // 更新标记
  volatile uint32_t app_status;     // APP 的状态
  volatile uint32_t back_to_app;    // 是否允许回到 APP
  volatile uint32_t crc_val_v1;     // 引导参数的 CRC 校验值
  volatile uint32_t version;        // 升级程序版本
  volatile uint32_t update_type;    // 升级方案
  volatile uint32_t from_app;       // 是否从 APP 跳转
  volatile uint32_t crc_val_v2;     // 引导参数的 CRC 校验值
} __attribute__((aligned(FLASH_DATA_ALIGN))) BOOT_PARAM;

typedef enum {
  PARTITION_APP = 0xFFFFFFFF,
  PARTITION_BLD = 0xA5A5A5A5,
} PARTITION_TYPE;

typedef enum {
  PKG_TYPE_INIT = 0x1000,
  PKG_TYPE_HEAD = 0x0001,
  PKG_TYPE_DATA = 0x0002,
  PKG_TYPE_FINISH = 0x0FFE,
} PKG_TYPE;

typedef struct {
  uint32_t partition_type;  // 分区类型
} PKG_INIT;

typedef struct {
  uint32_t file_crc;        // 升级文件的 CRC 值
  uint32_t file_size_real;  // 升级文件实际大小
  uint16_t data_size_one;   // 单次传输的升级数据大小
  uint16_t pkg_num_total;   // 升级包数量
} PKG_HEAD;

typedef struct {
  uint32_t pkg_crc;                       // 升级数据 CRC
  uint16_t pkg_num;                       // 当前包号
  uint16_t data_len;                      // 携带的数据长度
  uint8_t data[UPDATE_PACKAGE_MAX_SIZE];  // 升级数据
} PKG_DATA;

typedef struct {
  uint16_t type;
  union {
    PKG_INIT init;
    PKG_HEAD head;
    PKG_DATA data;
  };
} UPDATE_PKG;

#ifndef PROGRAM_BLD
void boot_to_bld(uint32_t boot_addr);
#endif

int8_t boot_param_update(BOOT_PARAM *param);
int8_t boot_param_bak_update(BOOT_PARAM *param);
int8_t boot_param_get_with_check(BOOT_PARAM *pdata);
void boot_param_get(BOOT_PARAM *pdata);
void boot_param_check(uint8_t with_check);

#ifdef __cplusplus
}
#endif

#endif

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

#ifdef __cplusplus
extern "C" {
#endif

void boot_param_check(void);

#ifdef __cplusplus
}
#endif

#endif

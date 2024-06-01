#include "onchip_flash.h"

#include "main.h"

#define FLASH_WAITETIME 50000  // FLASH等待超时时间

static inline uint32_t STMFLASH_GetPage(uint32_t Addr) {
  uint32_t page = 0;

  page = (Addr - STMFLASH_BASE) / FLASH_PAGE_SIZE;

  return page;
}

int8_t STMFLASH_Read(const uint32_t ReadAddr, FLASH_DATA_TYPE *pBuffer, uint32_t Num) {
  uint32_t addrx = 0;
  uint32_t addr_end = 0;

  addrx = ReadAddr;
  addr_end = ReadAddr + Num * FLASH_DATA_ALIGN;

  if (addrx < STMFLASH_BASE || addr_end > STMFLASH_END || addrx % FLASH_DATA_ALIGN) {
    return -1;  // 非法地址
  }

  for (uint32_t i = 0; i < Num; i++) {
    pBuffer[i] = STMFLASH_ReadDoubleWord(addrx);
    addrx += FLASH_DATA_ALIGN;
  }

  return 0;
}

int8_t STMFLASH_Write(uint32_t WriteAddr, FLASH_DATA_TYPE *pBuffer, uint32_t Num) {
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t addrx = 0;
  uint32_t addr_end = 0;

  addrx = WriteAddr;               // 写入的起始地址
  addr_end = WriteAddr + Num * FLASH_DATA_ALIGN;  // 写入的结束地址

  if (addrx < STMFLASH_BASE || addr_end > STMFLASH_END || addrx % FLASH_DATA_ALIGN) {
    return -1;  // 非法地址
  }

  HAL_FLASH_Unlock();  // 解锁

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PROGERR | FLASH_FLAG_BSY);

  while (addrx < addr_end) {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addrx, *pBuffer);
    if (status != HAL_OK) {
      break;
    }
    addrx += FLASH_DATA_ALIGN;
    pBuffer++;
  }

  HAL_FLASH_Lock();  // 上锁

  if (status != HAL_OK) {
    return -2;
  }

  return 0;
}

int8_t STMFLASH_Erase(uint32_t addrx, uint32_t length, uint32_t retry) {
  uint32_t PageError = STMFLASH_GetPage(addrx);
  FLASH_EraseInitTypeDef FlashEraseInit;
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t addr_end = addrx + length;

  if (addrx < STMFLASH_BASE || addr_end > STMFLASH_END) {
    return -1;  // 非法地址
  }

  while (addrx < addr_end) {
    if (STMFLASH_ReadDoubleWord(addrx) != 0xFFFFFFFFFFFFFFFF) {
      status = HAL_ERROR;
      break;
    }
    addrx += 8;
  }

  if (status == HAL_OK) {
    return 0;
  }

  HAL_FLASH_Unlock();  // 解锁

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PROGERR | FLASH_FLAG_BSY);

  do {
    FlashEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;  // 擦除类型，页擦除
    FlashEraseInit.Banks = FLASH_BANK_1;
    FlashEraseInit.Page = PageError;                                  // 要擦除的页
    FlashEraseInit.NbPages = STMFLASH_GetPage(addr_end) - PageError;  // 擦除页数

    status = HAL_FLASHEx_Erase(&FlashEraseInit, &PageError);
  } while ((status != HAL_OK) && (retry--));

  HAL_FLASH_Lock();  // 上锁

  if (status != HAL_OK) {
    return -2;
  }

  return 0;
}

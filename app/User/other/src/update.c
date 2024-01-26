#include "update.h"

#include <stdio.h>
#include <string.h>

#include "common.h"
#include "crc.h"
#include "crc32_mpeg2.h"
#include "main.h"
#include "onchip_flash.h"
#include "param.h"

typedef enum {
  UPDATE_WAITTING = 0,
  UPDATE_BEGIN,
  UPDATE_TRANSMIT,
  UPDATE_FINISH,
} UPDATE_STAGE;

typedef enum {
  ERRNO_SUCC = 0,       // 无错误
  ERRNO_PROC = 7,       // 处理中
  ERRNO_CHECK_CRC = 6,  // 校验错误
  ERRNO_DATA_SIZE = 5,  // 数据长度错误
  ERRNO_PKG_NUM = 4,    // 包号错误
  ERRNO_FLASH_WR = 3,   // Flash 写入错误
  ERRNO_RES = 2,        // 预留错误码
  ERRNO_UNKNOW = 1,     // 未知错误
} UPDATE_ERRNO;

typedef struct {
  uint16_t pkg_num : 12;
  uint16_t running : 1;
  uint16_t errno : 3;
} UPDATE_STATUS;

typedef struct {
  CRC32_MPEG2 crc;          // crc 计算器
  BOOT_PARAM boot_param;    // boot 参数
  uint32_t file_crc;        // 升级文件的 CRC 值
  uint32_t file_size_real;  // 升级文件实际大小
  uint16_t data_size_one;   // 单次传输的升级数据大小
  uint16_t pkg_num_total;   // 升级包数量
  uint16_t process_num;     // 已处理包号
  uint32_t recv_len;        // 已接收数据长度
  uint32_t recv_crc;        // 已接收数据的 CRC
} UPDATE_INFO;

typedef void (*pFunction)(void);
__IO uint32_t MspAddress;
__IO uint32_t JumpAddress;
__IO pFunction JumpToApplication;

UPDATE_PKG g_update_pkg;
static UPDATE_INFO s_update_info;

static BOOT_PARAM boot_param_default = {
    .update_needed = 0,
    .app_status = STATUS_BOOT,
    .back_to_app = 0,
};
const uint32_t boot_param_size64 = (sizeof(BOOT_PARAM) >> 3) + !!(sizeof(BOOT_PARAM) % 8);
const uint32_t boot_param_crcdatalen = sizeof(boot_param_default) - sizeof(boot_param_default.crc_val);

static inline void crc_reset(void) {
  /* Change CRC peripheral state */
  (&hcrc)->State = HAL_CRC_STATE_BUSY;

  /* Reset CRC Calculation Unit (hcrc->Instance->INIT is
   *  written in hcrc->Instance->DR) */
  __HAL_CRC_DR_RESET(&hcrc);

  /* Change CRC peripheral state */
  (&hcrc)->State = HAL_CRC_STATE_READY;
}

static uint32_t param_crc_calc(const BOOT_PARAM *param) {
  uint32_t crc = 0;

  crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)param, boot_param_crcdatalen);

  return crc;
}

static int8_t boot_param_save(uint32_t addr, BOOT_PARAM *param) {
  int8_t err = 0;

  param->crc_val = param_crc_calc(param);

  disable_global_irq();
  err = STMFLASH_Erase(addr, PART_SIZE_PARAM, 1);
  enable_global_irq();
  if (err) {
    printf_dbg("ERROR: Flash Erase\r\n");
    return -1;
  }

  disable_global_irq();
  err = STMFLASH_Write(addr, (uint64_t *)param, boot_param_size64);
  enable_global_irq();
  if (err) {
    printf_dbg("ERROR: Flash Write\r\n");
    return -2;
  }

  return 0;
}

int8_t boot_param_update(BOOT_PARAM *param) {
  int8_t err = 0;

  err = boot_param_save(ADDR_BASE_PARAM, param);
  if (err) {
    return err;
  }

  err = boot_param_save(ADDR_BASE_PARAM_BAK, param);
  if (err) {
    return err;
  }

  return 0;
}

void boot_param_get(BOOT_PARAM *pdata) {
  (void)STMFLASH_Read(ADDR_BASE_PARAM, (uint64_t *)pdata, boot_param_size64);
}

static int8_t boot_param_get_with_check(BOOT_PARAM *pdata) {
  BOOT_PARAM param, param_bak;

  (void)STMFLASH_Read(ADDR_BASE_PARAM, (uint64_t *)&param, boot_param_size64);
  (void)STMFLASH_Read(ADDR_BASE_PARAM_BAK, (uint64_t *)&param_bak, boot_param_size64);

  if (param_crc_calc(&param) == param.crc_val) {
    printf_dbg("boot param checked Ok\r\n");
    if (param_crc_calc(&param_bak) == param_bak.crc_val) {
      printf_dbg("boot param backup checked Ok\r\n");
      if (memcmp(&param, &param_bak, sizeof(BOOT_PARAM)) != 0) {
        printf_dbg("boot param main sector and backup sector data are different, update bakup sector data\r\n");
        if (boot_param_save(ADDR_BASE_PARAM_BAK, &param)) {
          return -1;
        }
      } else {
        printf_dbg("boot param main sector and backup sector data are the same\r\n");
      }
    } else {
      printf_dbg("boot param backup checked Fail, update backup sector data\r\n");
      if (boot_param_save(ADDR_BASE_PARAM_BAK, &param)) {
        return -1;
      }
    }
    memcpy(pdata, &param, sizeof(BOOT_PARAM));
  } else {
    printf_dbg("boot param checked Fail\r\n");
    if (param_crc_calc(&param_bak) == param_bak.crc_val) {
      printf_dbg("boot param backup checked Ok\r\n");
      printf_dbg("update main sector data\r\n");
      if (boot_param_save(ADDR_BASE_PARAM, &param_bak)) {
        return -1;
      }
      memcpy(pdata, &param_bak, sizeof(BOOT_PARAM));
    } else {
      printf_dbg("boot param backup checked Fail\r\n");
      printf_dbg("restore defaults\r\n");
      if (boot_param_update(&boot_param_default)) {
        return -1;
      }
      memcpy(pdata, &boot_param_default, sizeof(BOOT_PARAM));
    }
  }

  return 0;
}

static int8_t load_app_from_backup(void) {
  uint64_t buf[256];
  int8_t err = 0;
  uint32_t addr_write = ADDR_BASE_APP;
  uint32_t addr_read = ADDR_BASE_APP_BAK;

  disable_global_irq();
  err = STMFLASH_Erase(ADDR_BASE_APP, PART_SIZE_APP, 1);
  enable_global_irq();
  if (err) {
    return err;
  }

  while (addr_write < ADDR_BASE_APP_BAK) {
    (void)STMFLASH_Read(addr_read, buf, sizeof(buf) >> 3);
    disable_global_irq();
    err = STMFLASH_Write(addr_write, buf, sizeof(buf) >> 3);
    enable_global_irq();
    if (!err) {
      addr_read += sizeof(buf);
      addr_write += sizeof(buf);
    }
  }

  return 0;
}

static inline __attribute__((always_inline)) void boot_to_app(uint32_t boot_addr) {
  MspAddress = STMFLASH_ReadWord(boot_addr);
  JumpAddress = STMFLASH_ReadWord(boot_addr + 4);
  JumpToApplication = (pFunction)JumpAddress;

  if ((MspAddress & 0xFFFFC000) != 0x10000000 && (MspAddress & 0xFFFF0000) != 0x20000000) {
    return;
  }

  LL_SYSTICK_DisableIT();
  LL_USART_Disable(USART1);
  LL_USART_DisableIT_IDLE(USART1);
  LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_4);
  LL_USART_DeInit(USART1);
  LL_DMA_DeInit(DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_DeInit(DMA1, LL_DMA_CHANNEL_4);
  HAL_CRC_DeInit(&hcrc);
  HAL_DeInit();

  SCB->VTOR = boot_addr;

  __set_CONTROL(0);
  __set_PSP(MspAddress);
  __set_MSP(MspAddress);
  JumpToApplication();
}

void boot_param_check(uint8_t with_check) {
  BOOT_PARAM param;

  if (with_check) {
    if (boot_param_get_with_check(&param)) {
      Error_Handler();
    }
  } else {
    boot_param_get(&param);
  }

  if (SCB->VTOR == ADDR_BASE_APP) {
    param.update_needed = 0;
    param.app_status = STATUS_NORM;
    param.back_to_app = 1;
    if (boot_param_update(&param)) {
      Error_Handler();
    }
    printf_dbg("running in app\r\n");
    return;
  } else {
    printf_dbg("running in bld\r\n");
  }

  if (!param.update_needed) {
    switch (param.app_status) {
      /* 默认状态，尝试引导 */
      case STATUS_BOOT: {
        param.update_needed = 1;
        param.back_to_app = 0;
        if (boot_param_update(&param)) {
          Error_Handler();
        }
      } break;
      /* 正常状态，正常引导 */
      case STATUS_NORM: {
      } break;
      /* 异常状态，进入升级 */
      case STATUS_NONE:
      case STATUS_RECV:
      case STATUS_LOAD:
      default: {
        param.update_needed = 1;
        param.app_status = STATUS_NONE;
        param.back_to_app = 0;
        if (boot_param_update(&param)) {
          Error_Handler();
        }
        return;
      } break;
    }
  } else {
    switch (param.app_status) {
      /* 升级过程中断电 */
      case STATUS_RECV: {
        if (param.back_to_app) {
          /* 引导升级前的 APP */
          param.app_status = STATUS_BOOT;
          if (boot_param_update(&param)) {
            Error_Handler();
          }
        } else {
          /* 进入升级 */
          param.app_status = STATUS_NONE;
          if (boot_param_update(&param)) {
            Error_Handler();
          }
          return;
        }
      } break;
      /* 加载程序时断电，程序加载后引导 */
      case STATUS_LOAD: {
        if (load_app_from_backup()) {
          Error_Handler();
        }
        param.app_status = STATUS_BOOT;
        param.back_to_app = 0;
        if (boot_param_update(&param)) {
          Error_Handler();
        }
      } break;
      /* 正常状态，进入升级 */
      case STATUS_NORM: {
        return;
      } break;
      /* 异常状态，进入升级 */
      case STATUS_BOOT:
      case STATUS_NONE:
      default: {
        param.app_status = STATUS_NONE;
        param.back_to_app = 0;
        if (boot_param_update(&param)) {
          Error_Handler();
        }
        return;
      } break;
    }
  }

  /* 引导到 APP */
  boot_to_app(ADDR_BASE_APP);
  printf_dbg("ERROR: boot to app\r\n");

  /* 引导失败，进入升级 */
  param.update_needed = 1;
  param.app_status = STATUS_NONE;
  param.back_to_app = 0;
  if (boot_param_update(&param)) {
    Error_Handler();
  }
}

static void update_init(SYS_PARAM *sys) {
  int8_t err = 0;
  UPDATE_STATUS status;

  sys->ctrl.update.stage = UPDATE_BEGIN;
  sys->ctrl.update.status = 0x1FFF;
  /* 清零升级过程数据 */
  memset(&s_update_info, 0, sizeof(s_update_info));
  (void)CRC_OPT(crc32_mpeg2, init)(&s_update_info.crc);
  boot_param_get(&s_update_info.boot_param);
  /* 擦除 APP 接收区域 */
  disable_global_irq();
  err = STMFLASH_Erase(ADDR_BASE_APP_BAK, PART_SIZE_APP, 1);
  enable_global_irq();
  if (err) {
    /* 理论上不应该发生 */
    printf_dbg("![impossible] update_init\r\n");
    memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
    status.errno = ERRNO_FLASH_WR;
    memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
  }
}

static void update_reset(SYS_PARAM *sys) {
  sys->ctrl.update.stage = UPDATE_WAITTING;
  sys->ctrl.update.status = 0x0FFF;
}

void update_pkg_process(void) {
  SYS_PARAM *sys = sys_param_get();
  uint32_t pkg_crc;
  int8_t err = 0;
  UPDATE_STATUS status;
  uint32_t msp_addr;

  if (!sys->ctrl.update.need_process) {
    return;
  }

  switch (sys->ctrl.update.stage) {
    case UPDATE_WAITTING: {
      switch (g_update_pkg.type) {
        case PKG_TYPE_INIT: {
          update_init(sys);
        } break;
        case PKG_TYPE_HEAD:
        case PKG_TYPE_DATA:
        case PKG_TYPE_FINISH:
        default: {
          update_reset(sys);
        } break;
      }
    } break;
    case UPDATE_BEGIN: {
      switch (g_update_pkg.type) {
        case PKG_TYPE_INIT: {
          update_init(sys);
        } break;
        case PKG_TYPE_HEAD: {
          sys->ctrl.update.status = 0x1000;
          memcpy(&s_update_info.file_crc, &g_update_pkg.head.file_crc, sizeof(s_update_info.file_crc));
          memcpy(&s_update_info.file_size_real, &g_update_pkg.head.file_size_real, sizeof(s_update_info.file_size_real));
          memcpy(&s_update_info.data_size_one, &g_update_pkg.head.data_size_one, sizeof(s_update_info.data_size_one));
          memcpy(&s_update_info.pkg_num_total, &g_update_pkg.head.pkg_num_total, sizeof(s_update_info.pkg_num_total));
          /* 检查升级包数量 */
          if (s_update_info.pkg_num_total >= 0xFFE) {
            memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
            status.errno = ERRNO_PKG_NUM;
            memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
            break;
          }
          /* 检查数据长度 */
          if ((s_update_info.file_size_real > PART_SIZE_APP) ||
              (s_update_info.data_size_one > UPDATE_PACKAGE_MAX_SIZE) ||
              (s_update_info.data_size_one % 2)) {
            memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
            status.errno = ERRNO_DATA_SIZE;
            memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
            break;
          }
          /* 更新引导参数 */
          s_update_info.boot_param.update_needed = 1;
          s_update_info.boot_param.app_status = STATUS_RECV;
          if (boot_param_update(&s_update_info.boot_param)) {
            /* 理论上不应该发生 */
            printf_dbg("![impossible] PKG_TYPE_HEAD\r\n");
            memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
            status.errno = ERRNO_FLASH_WR;
            memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
            break;
          }
          sys->ctrl.update.stage = UPDATE_TRANSMIT;
        } break;
        case PKG_TYPE_DATA:
        case PKG_TYPE_FINISH:
        default: {
          update_reset(sys);
        } break;
      }
    } break;
    case UPDATE_TRANSMIT: {
      switch (g_update_pkg.type) {
        case PKG_TYPE_INIT: {
          update_init(sys);
        } break;
        case PKG_TYPE_DATA: {
          memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
          status.errno = ERRNO_PROC;
          status.pkg_num = s_update_info.process_num;
          memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
          /* 检查升级包号 */
          if ((g_update_pkg.data.pkg_num <= s_update_info.pkg_num_total) && (g_update_pkg.data.pkg_num != s_update_info.process_num + 1)) {
            memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
            status.errno = ERRNO_PKG_NUM;
            memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
            break;
          }
          /* 检查数据长度 */
          if ((g_update_pkg.data.data_len > s_update_info.data_size_one) ||
              ((g_update_pkg.data.data_len != s_update_info.data_size_one) && (g_update_pkg.data.pkg_num < s_update_info.pkg_num_total))) {
            memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
            status.errno = ERRNO_DATA_SIZE;
            memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
            break;
          }
          /* 计算数据 CRC */
          pkg_crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)g_update_pkg.data.data, g_update_pkg.data.data_len);
          if (pkg_crc != g_update_pkg.data.pkg_crc) {
            memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
            status.errno = ERRNO_CHECK_CRC;
            memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
            break;
          }
          /* 向 Flash 写入数据 */
          disable_global_irq();
          err = STMFLASH_Write((ADDR_BASE_APP_BAK + s_update_info.recv_len),
              (uint64_t *)g_update_pkg.data.data,
              ((g_update_pkg.data.data_len >> 3) + !!(g_update_pkg.data.data_len % 8)));
          enable_global_irq();
          if (err) {
            /* 理论上不应该发生 */
            printf_dbg("![impossible] PKG_TYPE_DATA\r\n");
            memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
            status.errno = ERRNO_FLASH_WR;
            memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
            update_reset(sys);
            break;
          }
          /* 更新升级信息 */
          s_update_info.recv_len += g_update_pkg.data.data_len;
          s_update_info.recv_crc = CRC_OPT(crc32_mpeg2, accum)(&s_update_info.crc, g_update_pkg.data.data, g_update_pkg.data.data_len);
          /* 数据包接收成功 */
          s_update_info.process_num += 1;
          memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
          status.errno = ERRNO_SUCC;
          status.pkg_num = s_update_info.process_num;
          memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
        } break;
        case PKG_TYPE_FINISH: {
          sys->ctrl.update.status = 0xEFFE;
          /* 验证文件长度 */
          if (s_update_info.file_size_real != s_update_info.recv_len) {
            memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
            status.errno = ERRNO_DATA_SIZE;
            memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
            break;
          }
          /* 校验文件 CRC */
          if (s_update_info.file_crc != CRC_OPT(crc32_mpeg2, get)(&s_update_info.crc)) {
            memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
            status.errno = ERRNO_CHECK_CRC;
            memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
            break;
          }
          /* 校验升级包正确性 */
          msp_addr = STMFLASH_ReadWord(ADDR_BASE_APP_BAK);
          if ((msp_addr & 0xFFFFC000) != 0x10000000 && (msp_addr & 0xFFFF0000) != 0x20000000) {
            memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
            status.errno = ERRNO_UNKNOW;
            memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
            break;
          }
          /* 更新引导参数 */
          s_update_info.boot_param.update_needed = 1;
          s_update_info.boot_param.back_to_app = 0;
          s_update_info.boot_param.app_status = STATUS_LOAD;
          if (boot_param_update(&s_update_info.boot_param)) {
            /* 理论上不应该发生 */
            printf_dbg("![impossible] PKG_TYPE_FINISH\r\n");
            memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
            status.errno = ERRNO_FLASH_WR;
            memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
            sys->ctrl.update.stage = UPDATE_FINISH;
            break;
          }
#if 0
          /* 加载 APP 到运行区 */
          if (load_app_from_backup()) {
            /* 理论上不应该发生 */
            printf_dbg("![impossible] PKG_TYPE_FINISH\r\n");
            memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
            status.errno = ERRNO_FLASH_WR;
            memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
            sys->ctrl.update.stage = UPDATE_FINISH;
            break;
          }
          /* 更新引导参数 */
          s_update_info.boot_param.update_needed = 0;
          s_update_info.boot_param.app_status = STATUS_BOOT;
          if (boot_param_update(&s_update_info.boot_param)) {
            /* 理论上不应该发生 */
            printf_dbg("![impossible] PKG_TYPE_FINISH\r\n");
            memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
            status.errno = ERRNO_FLASH_WR;
            memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
            sys->ctrl.update.stage = UPDATE_FINISH;
            break;
          }
#endif
          /* 升级成功 */
          memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
          status.errno = ERRNO_SUCC;
          memcpy(&sys->ctrl.update.status, &status, sizeof(UPDATE_STATUS));
          sys->ctrl.update.stage = UPDATE_FINISH;
        } break;
        case PKG_TYPE_HEAD:
        default: {
          update_reset(sys);
        } break;
      }
    } break;
    case UPDATE_FINISH: {
      switch (g_update_pkg.type) {
        case PKG_TYPE_INIT: {
          /* 升级失败，等待复位 */
          memcpy(&status, &sys->ctrl.update.status, sizeof(UPDATE_STATUS));
          if (status.errno != ERRNO_SUCC) {
            update_init(sys);
          }
        } break;
        case PKG_TYPE_HEAD:
        case PKG_TYPE_DATA:
        case PKG_TYPE_FINISH:
        default: {
          /* 升级成功，等待设备重启，不再进行升级 */
        } break;
      }
    } break;
    default: {
      Error_Handler();
    } break;
  }

  /* 升级包处理完成后复位 */
  disable_global_irq();
  sys->ctrl.update.need_process = 0;
  enable_global_irq();
}

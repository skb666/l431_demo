  
  
# STM32 备份升级
  
## 升级方案详解
  
### 前言
  
IAP (in-application programming) 又叫“应用程序内编程”，对于大多数基于闪存的系统，一个重要的要求是能够在最终产品中安装固件进行更新。  
STM32微控制器可以运行用户特定的固件来对微控制器中嵌入的闪存执行IAP。  
由于不限制通信接口协议等，只要能通过任意通信接口拿到新版固件包数据（bin文件），就能自己升级固件。另外通过添加外部无线模块（WIFI、蓝牙、4G模块）或者使用U盘、TF卡等外部存储设备还可以做OTA升级。
  
**解决了什么问题？**
  
1. 在升级 APP 时不再执行其它业务流程，无法进行后台升级
2. BLD 程序烧录后只能通过再次烧录来更新 BLD
3. 升级完成后会立即重启，用户无法控制
4. 协议不统一，不同外设接口走不同的升级协议
5. 升级过程不统一，跳转升级与应用内升级完成后需要的等待启动时间不一致
  
**存在的不完善的地方**
  
本项目为演示验证以下讲述的方案，使用了尽可能简化的流程，尽量使其易移植、易修改，所以未涉及固件存储安全、固件传输安全等相关的内容。
  
若需要，大家可以自己移植 ymodem 协议用于接收 bin、添加加解密算法用于保证存储安全等等 :)
  
### FLASH 分区设计
  
| 扇区  |  起始地址  |    大小 | 名称      | 成分         |
| :---: | :--------: | ------: | :-------- | :----------- |
|   0   | 0x08000000 |  0x8000 | bld       | BLD          |
|   1   | 0x08008000 |   0x800 | param     | 引导参数     |
|   2   | 0x08008800 |   0x800 | param_bak | 参数备份     |
|   3   | 0x08009000 | 0x1B800 | app       | APP          |
|   4   | 0x08024800 | 0x1B800 | backup    | 升级固件暂存 |
  
```c
/* FLASH 扇区的起始地址 */
#define STMFLASH_BASE (0x08000000UL) /*!< FLASH(up to 256 KB) base address */
#define STMFLASH_END (0x08040000UL)  /*!< FLASH END address                */
  
/* 功能分区定义 */
#define ADDR_BASE_BLD (STMFLASH_BASE)
#define ADDR_BASE_PARAM (ADDR_FLASH_PAGE_16)
#define ADDR_BASE_PARAM_BAK (ADDR_FLASH_PAGE_17)
#define ADDR_BASE_APP (ADDR_FLASH_PAGE_18)
#define ADDR_BASE_BACKUP (ADDR_FLASH_PAGE_73)
  
#define PART_SIZE_BLD ((ADDR_BASE_PARAM) - (ADDR_BASE_BLD))          // 32KB
#define PART_SIZE_PARAM ((ADDR_BASE_PARAM_BAK) - (ADDR_BASE_PARAM))  // 2KB
#define PART_SIZE_APP ((ADDR_BASE_BACKUP) - (ADDR_BASE_APP))         // 110KB
```
  
### 引导参数设计
  
**状态类型**
  
```c
typedef enum {
  STATUS_NONE = 0,  // APP 不存在
  STATUS_RECV,      // APP 接收中
  STATUS_LOAD,      // APP 载入中
  STATUS_BOOT,      // APP 可引导
  STATUS_NORM,      // APP 可运行
} APP_STATUS;
```
  
**存储结构**
  
```c
typedef struct {
  uint32_t update_needed;  // 更新标记
  uint32_t app_status;     // APP 的状态
  uint32_t back_to_app;    // 是否允许回到 APP
  uint32_t crc_val;        // 引导参数的 CRC 校验值
} BOOT_PARAM;
```
  
**默认参数**
  
```c
static BOOT_PARAM boot_param_default = {
    .update_needed = 0,
    .app_status = STATUS_BOOT,
    .back_to_app = 0,
};
```
  
**参数读写与更新**
  
参数校验时不包括存储结构中的 CRC 值，通过调用 HAL 库的 CRC 计算函数实现：
  
```c
static uint32_t param_crc_calc(const BOOT_PARAM *param) {
  uint32_t crc = 0;
  
  crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)param, boot_param_crcdatalen);
  
  return crc;
}
```
  
**参数擦除**
  
```c
static int8_t boot_param_erase(uint32_t addr) {
  int8_t err = 0;
  
  disable_global_irq();
  err = STMFLASH_Erase(addr, PART_SIZE_PARAM, 1);
  enable_global_irq();
  if (err) {
    return err;
  }
  
  return 0;
}
```
  
**参数固化**
  
```c
static int8_t boot_param_save(uint32_t addr, BOOT_PARAM *param) {
  int8_t err = 0;
  
  param->crc_val = param_crc_calc(param);
  
  disable_global_irq();
  err = STMFLASH_Write(addr, (uint64_t *)param, boot_param_size64);
  enable_global_irq();
  if (err) {
    return err;
  }
  
  return 0;
}
```
  
**参数更新**
  
```c
int8_t boot_param_update(BOOT_PARAM *param) {
  int8_t err = 0;
  
  err = boot_param_erase(ADDR_BASE_PARAM);
  if (err) {
    printf_dbg("ERROR: Flash Erase\r\n");
    return err;
  }
  
  err = boot_param_save(ADDR_BASE_PARAM, param);
  if (err) {
    printf_dbg("ERROR: Flash Write\r\n");
    return err;
  }
  
  return 0;
}
  
int8_t boot_param_bak_update(BOOT_PARAM *param) {
  int8_t err = 0;
  
  err = boot_param_erase(ADDR_BASE_PARAM_BAK);
  if (err) {
    printf_dbg("ERROR: Flash Erase\r\n");
    return err;
  }
  
  err = boot_param_save(ADDR_BASE_PARAM_BAK, param);
  if (err) {
    printf_dbg("ERROR: Flash Write\r\n");
    return err;
  }
  
  return 0;
}
```
  
**参数获取**
  
```c
void boot_param_get(BOOT_PARAM *pdata) {
  (void)STMFLASH_Read(ADDR_BASE_PARAM, (uint64_t *)pdata, boot_param_size64);
}
```
  
#### 引导参数校验与修正
  

![](/assets/other/9e0bf638ef74184250e742a6ab9721970.png?0.5875066329583529)  
  
主要逻辑实现如下：
  
```c
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
        if (boot_param_bak_update(&param)) {
          return -1;
        }
      } else {
        printf_dbg("boot param main sector and backup sector data are the same\r\n");
      }
    } else {
      printf_dbg("boot param backup checked Fail, update backup sector data\r\n");
      if (boot_param_bak_update(&param)) {
        return -1;
      }
    }
    memcpy(pdata, &param, sizeof(BOOT_PARAM));
  } else {
    printf_dbg("boot param checked Fail\r\n");
    if (param_crc_calc(&param_bak) == param_bak.crc_val) {
      printf_dbg("boot param backup checked Ok\r\n");
      printf_dbg("update main sector data\r\n");
      if (boot_param_update(&param_bak)) {
        return -1;
      }
      memcpy(pdata, &param_bak, sizeof(BOOT_PARAM));
    } else {
      printf_dbg("boot param backup checked Fail\r\n");
      printf_dbg("restore defaults\r\n");
      if (boot_param_update(&boot_param_default)) {
        return -1;
      }
      if (boot_param_bak_update(&boot_param_default)) {
        return -1;
      }
      memcpy(pdata, &boot_param_default, sizeof(BOOT_PARAM));
    }
  }
  
  return 0;
}
```
  
#### 引导程序流程
  
1. 判断当前运行是否运行在 APP
2. 若运行在 APP，则清除升级标志；标记 APP 状态为 STATUS_NORM；置位可回退 APP 标志后退出运行 APP 逻辑
3. 检查升级标志是否被置位
4. 升级标志未被置位：
    + *STATUS_BOOT*：置位升级标志；清除可回退 APP 标志；尝试引导到 APP
    + *STATUS_NORM*：正常引导到 APP
    + *STATUS_NONE*、*STATUS_RECV*、*STATUS_LOAD*：置位升级标志；清除可回退 APP 标志；标记 APP 状态为 STATUS_NONE；退出等待升级
5. 升级标志被置位：
    + *STATUS_RECV*：若可回退 APP 则标记 APP 状态为 STATUS_BOOT，尝试引导 APP，否则标记 APP 状态为 STATUS_NONE，退出等待升级
    + *STATUS_LOAD*：从升级固件暂存区加载覆盖 APP；标记 APP 状态为 STATUS_BOOT；清除可回退 APP 标志；尝试引导 APP
    + *STATUS_NORM*：退出后正常进入等待升级
    + *STATUS_BOOT*、*STATUS_NONE*：标记 APP 状态为 STATUS_NONE；清除可回退 APP 标志；退出等待升级
6. 若引导 APP 失败则置位升级标志；清除可回退 APP 标志；标记 APP 状态为 STATUS_NONE；退出等待升级
  
**固件加载覆盖**
  
单次拷贝 2KB 数据
  
```c
#ifdef PROGRAM_BLD
static int8_t load_app_from_backup(void) {
  uint64_t buf[256];
  int8_t err = 0;
  uint32_t addr_read = ADDR_BASE_BACKUP;
  uint32_t addr_write = ADDR_BASE_APP;
  uint32_t load_size = PART_SIZE_APP;
  uint32_t addr_write_end = addr_write + load_size;
  
  disable_global_irq();
  err = STMFLASH_Erase(addr_write, load_size, 1);
  enable_global_irq();
  if (err) {
    return err;
  }
  
  while (addr_write < addr_write_end) {
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
#else
static int8_t load_bld_from_backup(void) {
  uint64_t buf[256];
  int8_t err = 0;
  uint32_t addr_read = ADDR_BASE_BACKUP;
  uint32_t addr_write = ADDR_BASE_BLD;
  uint32_t load_size = PART_SIZE_BLD;
  uint32_t addr_write_end = addr_write + load_size;
  
  disable_global_irq();
  err = STMFLASH_Erase(addr_write, load_size, 1);
  enable_global_irq();
  if (err) {
    return err;
  }
  
  while (addr_write < addr_write_end) {
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
#endif
```
  
在 APP 内进行 BLD 程序覆盖；在 BLD 内进行 APP 程序覆盖
  
**分区跳转**
  
![iap](/assets/image/iap.png )
  
1. 在跳转前需要关闭所有的中断，然后反初始化用到的外设
2. 从 APP 分区起始地址取中断向量表中栈顶指针地址
3. 栈顶指针地址偏移 4 字节，在中断向量表中取程序入口 Reset_Handler 的函数地址
4. 检测栈顶指针地址是否合法，不合法则退出引导，进入升级
5. 设置 PSP、MSP 指针指向中断向量表的栈顶地址保存的地址
6. 跳转到程序入口，执行 APP
  
```c
typedef void (*pFunction)(void);
__IO uint32_t MspAddress;
__IO uint32_t JumpAddress;
__IO pFunction JumpToApplication;
  
#ifdef PROGRAM_BLD
static inline __attribute__((always_inline)) void boot_to_app(uint32_t boot_addr) {
  MspAddress = STMFLASH_ReadWord(boot_addr);
  JumpAddress = STMFLASH_ReadWord(boot_addr + 4);
  JumpToApplication = (pFunction)JumpAddress;
  
  if ((MspAddress & 0xFFFF8000) != 0x10000000 && (MspAddress & 0xFFFF0000) != 0x20000000) {
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
  LL_I2C_Disable(I2C1);
  LL_I2C_DisableIT_ADDR(I2C1);
  LL_I2C_DisableIT_NACK(I2C1);
  LL_I2C_DisableIT_ERR(I2C1);
  LL_I2C_DisableIT_STOP(I2C1);
  LL_I2C_DisableIT_RX(I2C1);
  LL_I2C_DisableIT_TX(I2C1);
  LL_I2C_DeInit(I2C1);
  HAL_CRC_DeInit(&hcrc);
  HAL_DeInit();
  
  SCB->VTOR = boot_addr;
  
  __set_CONTROL(0);
  __set_PSP(MspAddress);
  __set_MSP(MspAddress);
  JumpToApplication();
}
#endif
```
  
上面对于栈顶指针地址的检测，第一个范围为 RAM1，第二个范围为 RAM2
  
### 固件升级流程
  
#### 升级包类型、结构设计
  
**升级的分区类型**
  
```c
typedef enum {
  PARTITION_APP = 0xFFFFFFFF,
  PARTITION_BLD = 0xA5A5A5A5,
} PARTITION_TYPE;
```
  
**升级包类型**
  
```c
typedef enum {
  PKG_TYPE_INIT = 0x1000,
  PKG_TYPE_FINISH = 0x0FFE,
  PKG_TYPE_HEAD = 0x0001,
  PKG_TYPE_DATA = 0x0002,
} PKG_TYPE;
```
  
**固件头部信息包结构**
  
```c
typedef struct {
  uint32_t partition_type;  // 分区类型
  uint32_t file_crc;        // 升级文件的 CRC 值
  uint32_t file_size_real;  // 升级文件实际大小
  uint16_t data_size_one;   // 单次传输的升级数据大小
  uint16_t pkg_num_total;   // 升级包数量
} PKG_HEAD;
```
  
**固件数据包结构**
  
```c
typedef struct {
  uint32_t pkg_crc;                       // 升级数据 CRC
  uint16_t pkg_num;                       // 当前包号
  uint16_t data_len;                      // 携带的数据长度
  uint8_t data[UPDATE_PACKAGE_MAX_SIZE];  // 升级数据
} PKG_DATA;
```
#### 升级参数设计
  
**升级阶段**
  
```c
typedef enum {
  UPDATE_WAITTING = 0,
  UPDATE_BEGIN,
  UPDATE_TRANSMIT,
  UPDATE_FINISH,
} UPDATE_STAGE;
```
  
**错误码**
  
```c
typedef enum {
  ERRNO_SUCC = 0,       // 无错误
  ERRNO_PROC = 7,       // 处理中
  ERRNO_CHECK_CRC = 6,  // 校验错误
  ERRNO_DATA_SIZE = 5,  // 数据长度错误
  ERRNO_PKG_NUM = 4,    // 包号错误
  ERRNO_FLASH_WR = 3,   // Flash 写入错误
  ERRNO_PARTITION = 2,  // 分区类型错误
  ERRNO_UNKNOW = 1,     // 未知错误
} UPDATE_ERRNO;
```
  
**外部可获取的升级状态**
  
```c
typedef struct {
  uint16_t pkg_num : 12;
  uint16_t running : 1;
  uint16_t errno : 3;
} UPDATE_STATUS;
```
  
**升级信息结构**
  
```c
typedef struct {
  CRC32_MPEG2 crc;          // crc 计算器
  BOOT_PARAM boot_param;    // boot 参数
  uint32_t partition_type;  // 分区类型
  uint32_t file_crc;        // 升级文件的 CRC 值
  uint32_t file_size_real;  // 升级文件实际大小
  uint16_t data_size_one;   // 单次传输的升级数据大小
  uint16_t pkg_num_total;   // 升级包数量
  uint16_t process_num;     // 已处理包号
  uint32_t recv_len;        // 已接收数据长度
  uint32_t recv_crc;        // 已接收数据的 CRC
} UPDATE_INFO;
```
  
**内部升级控制结构**
  
```c
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
```
  
**对外升级包接口**
  
```c
typedef struct {
  uint16_t type;
  union {
    PKG_HEAD head;
    PKG_DATA data;
  };
} UPDATE_PKG;
```
  
#### 升级流程
  
1. 判断是否有包要处理，无包需要处理则直接退出
2. 根据当前升级阶段，判断升级包类型：
    + *UPDATE_WAITTING*
        + *PKG_TYPE_INIT*：升级阶段进入 UPDATE_BEGIN；升级状态置为 0x1FFF；擦除升级固件暂存区 Flash
        + *其它*：升级阶段回到 UPDATE_WAITTING；升级状态置为 0x0FFF
    + *UPDATE_BEGIN*
        + *PKG_TYPE_INIT*：升级阶段进入 UPDATE_BEGIN；升级状态置为 0x1FFF；擦除升级固件暂存区 Flash
        + *PKG_TYPE_HEAD*：升级状态置为 0x1000；检查升级分区类型、升级包数量、数据长度；检查失败则置位对应的升级状态错误码，否则更新引导参数：置位升级标志、标记 APP 状态为 STATUS_RECV，升级阶段进入 UPDATE_TRANSMIT
        + *其它*：升级阶段回到 UPDATE_WAITTING；升级状态置为 0x0FFF
    + *UPDATE_TRANSMIT*
        + *PKG_TYPE_INIT*：升级阶段进入 UPDATE_BEGIN；升级状态置为 0x1FFF；擦除升级固件暂存区 Flash
        + *PKG_TYPE_DATA*：置位升级状态中包号为已处理的包号，错误码置为处理中；检查升级包号是否为已处理包号的下一个包、数据长度、数据 CRC；检查失败则置位对应的升级状态错误码，否则写接收数据到升级包暂存区；更新升级信息，已处理包号 +1；置位升级状态错误码为成功，包号为已处理的包号
        + *PKG_TYPE_FINISH*：升级状态置为 0xEFFE；校验文件长度、文件 CRC、暂存区升级包是否可被引导；校验失败则置位对应的升级状态错误码，否则根据升级分区类型，更新引导参数或加载覆盖 BLD 程序；置位升级状态错误码为成功；升级阶段进入 UPDATE_FINISH
        + *其它*：升级阶段回到 UPDATE_WAITTING；升级状态置为 0x0FFF
    + *UPDATE_FINISH*
        + *PKG_TYPE_INIT*：判断升级状态；若升级失败则升级阶段进入 UPDATE_BEGIN，升级状态置为 0x1FFF，擦除升级固件暂存区 Flash
        + *其它*：等待设备重启
3. 清除包处理标志
  
校验使用的 CRC 算法为 [crc32_mpeg2](https://github.com/skb666/libcrc/tree/main/libcrc/crc32_mpeg2 )
  
#### 固件传输流程
  

![](/assets/other/9e0bf638ef74184250e742a6ab9721971.png?0.26278042362380205)  
  
中间 errno 获取到的值为 0 表示正常；若不为 0，根据错误码执行对应的操作
  
### 通过串口升级
  
#### 串口帧接收及处理
  
串口的接收与发送基于 [稳定 DMA 串口通信](/uart.md )
  
#### 串口帧注册
  
```c
typedef enum {
  FRAME_TYPE_DEBUG = 0x00,
  FRAME_TYPE_SYSTEM_CTRL = 0xF0,
  FRAME_TYPE_UPDATE_DATA = 0xF1,
  FRAME_TYPE_UPDATE_STATUS = 0xF2,
  FRAME_TYPE_MAX = FUNC_LIST_MAX,
} FRAME_TYPE;
```
  
对于固件接收及升级功能，我们在主函数进死循环前，要对 “系统控制”、“升级包数据传输”、“升级状态获取” 等 id 进行注册：
  
```c
frame_parse_register(DEV_USART1, FRAME_TYPE_SYSTEM_CTRL, system_ctrl_frame_parse);
frame_parse_register(DEV_USART1, FRAME_TYPE_UPDATE_DATA, update_frame_parse);
frame_parse_register(DEV_USART1, FRAME_TYPE_UPDATE_STATUS, update_status_get);
```
  
#### 串口升级接口
  
**系统控制**
  
```c
/* 主函数内循环执行 */
static void system_ctrl_check(void) {
  SYS_PARAM *sys = sys_param_get();
  BOOT_PARAM param;
  
  switch (sys->ctrl.system) {
    case SYSTEM_CTRL_REBOOT: {
      uart_printf(DEV_USART1, "SYSTEM_CTRL_REBOOT\r\n");
      LL_mDelay(500);
      NVIC_SystemReset();
    } break;
    case SYSTEM_CTRL_UPDATE_START: {
      boot_param_get(&param);
  
      param.update_needed = 1;
      if (boot_param_update(&param)) {
        Error_Handler();
      }
  
      uart_printf(DEV_USART1, "SYSTEM_CTRL_UPDATE_START\r\n");
      NVIC_SystemReset();
    } break;
    default: {
    } break;
  }
}
  
void system_ctrl_frame_parse(frame_parse_t *frame) {
  SYS_PARAM *sys = sys_param_get();
  uint16_t value;
  
  if (frame->length < sizeof(value)) {
    return;
  }
  
  memcpy(&value, frame->data, sizeof(value));
  if (frame->byte_order) {
    change_byte_order(&value, sizeof(value));
  }
  
  sys->ctrl.system = value;
}
```
  
**升级状态获取**
  
```c
void update_status_get(frame_parse_t *frame) {
  SYS_PARAM *sys = sys_param_get();
  uint16_t update_status = sys->ctrl.update.status;
  if (frame->byte_order) {
    change_byte_order(&update_status, sizeof(update_status));
  }
  uart_puts(DEV_USART1, (uint8_t *)&update_status, sizeof(update_status));
}
```
  
**升级包传输**
  
1. 检查是否有包未处理，有则忽略当前包
2. 获取当前升级包类型
3. 根据升级包类型，给结构体赋值
4. 置位升级包处理标志
  
```c
extern UPDATE_PKG g_update_pkg;
  
void update_frame_parse(frame_parse_t *frame) {
  SYS_PARAM *sys = sys_param_get();
  uint8_t *frame_data;
  uint16_t frame_length;
  
  if (sys->ctrl.update.need_process) {
    return;
  }
  
  if (frame->length < sizeof(g_update_pkg.type)) {
    return;
  }
  
  /* 获取升级包信息 */
  frame_data = frame->data;
  frame_length = frame->length;
  
  /* 获取升级包类型 */
  memcpy(&g_update_pkg.type, frame_data, sizeof(g_update_pkg.type));
  frame_data += sizeof(g_update_pkg.type);
  frame_length -= sizeof(g_update_pkg.type);
  if (frame->byte_order) {
    change_byte_order(&g_update_pkg.type, sizeof(g_update_pkg.type));
  }
  
  switch (g_update_pkg.type) {
    case PKG_TYPE_INIT:
    case PKG_TYPE_FINISH: {
      memset(&g_update_pkg.data, 0xFF, sizeof(PKG_DATA));
    } break;
    case PKG_TYPE_HEAD: {
      if (frame_length != sizeof(PKG_HEAD)) {
        return;
      }
      memset(&g_update_pkg.data, 0xFF, sizeof(PKG_DATA));
      memcpy(&g_update_pkg.head, frame_data, frame_length);
      if (frame->byte_order) {
        change_byte_order(&g_update_pkg.head.partition_type, sizeof(g_update_pkg.head.partition_type));
        change_byte_order(&g_update_pkg.head.file_crc, sizeof(g_update_pkg.head.file_crc));
        change_byte_order(&g_update_pkg.head.file_size_real, sizeof(g_update_pkg.head.file_size_real));
        change_byte_order(&g_update_pkg.head.data_size_one, sizeof(g_update_pkg.head.data_size_one));
        change_byte_order(&g_update_pkg.head.pkg_num_total, sizeof(g_update_pkg.head.pkg_num_total));
      }
    } break;
    case PKG_TYPE_DATA: {
      if ((frame_length < sizeof(PKG_DATA) - UPDATE_PACKAGE_MAX_SIZE) || (frame_length > sizeof(PKG_DATA))) {
        return;
      }
      memset(&g_update_pkg.data, 0xFF, sizeof(PKG_DATA));
      memcpy(&g_update_pkg.data, frame_data, frame_length);
      if (frame->byte_order) {
        change_byte_order(&g_update_pkg.data.pkg_crc, sizeof(g_update_pkg.data.pkg_crc));
        change_byte_order(&g_update_pkg.data.pkg_num, sizeof(g_update_pkg.data.pkg_num));
        change_byte_order(&g_update_pkg.data.data_len, sizeof(g_update_pkg.data.data_len));
      }
    } break;
    default: {
      return;
    } break;
  }
  
  /* 升级包准备好后置位 */
  disable_global_irq();
  sys->ctrl.update.need_process = 1;
  enable_global_irq();
}
```
  
### 通过 i2c 升级
  
#### i2c 从机实现
  
i2c 从机实现基于 [i2c 从机实现](/i2c_slave.md )
  
#### 相关寄存器及处理函数注册
  
```c
typedef enum {
  REG_VERSION = 0x0000,
  REG_SYSTEM_CTRL = 0xFF00,
  REG_UPDATE_DATA = 0xFF01,
  REG_UPDATE_STATUS = 0xFF02,
  REG_MAX = 0xFFFF,
} REG_NAME;
  
static REG_T s_reg_list[] = {
    {REG_VERSION, REG_RO, reg_read_cb_version, NULL},
    {REG_SYSTEM_CTRL, REG_RW, reg_read_cb_system_ctrl, reg_write_cb_system_ctrl},
    {REG_UPDATE_DATA, REG_RW, reg_read_cb_update_status, reg_write_cb_update_data},
    {REG_UPDATE_STATUS, REG_RO, reg_read_cb_update_status, NULL},
};
```
  
#### i2c 升级接口
  
**系统控制**
  
```c
/* 主函数内循环执行 */
static void system_ctrl_check(void) {
  SYS_PARAM *sys = sys_param_get();
  BOOT_PARAM param;
  
  switch (sys->ctrl.system) {
    case SYSTEM_CTRL_REBOOT: {
      uart_printf(DEV_USART1, "SYSTEM_CTRL_REBOOT\r\n");
      LL_mDelay(500);
      NVIC_SystemReset();
    } break;
    case SYSTEM_CTRL_UPDATE_START: {
      boot_param_get(&param);
  
      param.update_needed = 1;
      if (boot_param_update(&param)) {
        Error_Handler();
      }
  
      uart_printf(DEV_USART1, "SYSTEM_CTRL_UPDATE_START\r\n");
      NVIC_SystemReset();
    } break;
    default: {
    } break;
  }
}
  
void reg_write_cb_system_ctrl(void) {
  SYS_PARAM *sys = sys_param_get();
  uint16_t value;
  
  if (i2c_slave_rx_size() < sizeof(uint16_t)) {
    return;
  }
  
  I2C_GET_NUM(uint16_t, value);
  sys->ctrl.system = value;
}
```
  
**升级状态获取**
  
```c
void reg_read_cb_update_status(void) {
  SYS_PARAM *sys = sys_param_get();
  
  I2C_PUT_NUM(uint16_t, sys->ctrl.update.status);
}
```
  
**升级包传输**
  
1. 检查是否有包未处理，有则忽略当前包
2. 获取当前升级包类型
3. 根据升级包类型，给结构体赋值
4. 置位升级包处理标志
  
```c
extern UPDATE_PKG g_update_pkg;
  
void reg_write_cb_update_data(void) {
  SYS_PARAM *sys = sys_param_get();
  
  if (sys->ctrl.update.need_process) {
    return;
  }
  
  if (i2c_slave_rx_size() < sizeof(g_update_pkg.type)) {
    return;
  }
  
  /* 获取升级包类型 */
  I2C_GET_NUM(uint16_t, g_update_pkg.type);
  
  switch (g_update_pkg.type) {
    case PKG_TYPE_INIT:
    case PKG_TYPE_FINISH: {
      memset(&g_update_pkg.data, 0xFF, sizeof(PKG_DATA));
    } break;
    case PKG_TYPE_HEAD: {
      if (i2c_slave_rx_size() != sizeof(PKG_HEAD)) {
        return;
      }
      memset(&g_update_pkg.data, 0xFF, sizeof(PKG_DATA));
      i2c_slave_rx_get((uint8_t *)&g_update_pkg.head, i2c_slave_rx_size());
      change_byte_order(&g_update_pkg.head.partition_type, sizeof(g_update_pkg.head.partition_type));
      change_byte_order(&g_update_pkg.head.file_crc, sizeof(g_update_pkg.head.file_crc));
      change_byte_order(&g_update_pkg.head.file_size_real, sizeof(g_update_pkg.head.file_size_real));
      change_byte_order(&g_update_pkg.head.data_size_one, sizeof(g_update_pkg.head.data_size_one));
      change_byte_order(&g_update_pkg.head.pkg_num_total, sizeof(g_update_pkg.head.pkg_num_total));
    } break;
    case PKG_TYPE_DATA: {
      if ((i2c_slave_rx_size() < sizeof(PKG_DATA) - UPDATE_PACKAGE_MAX_SIZE) || (i2c_slave_rx_size() > sizeof(PKG_DATA))) {
        return;
      }
      memset(&g_update_pkg.data, 0xFF, sizeof(PKG_DATA));
      i2c_slave_rx_get((uint8_t *)&g_update_pkg.data, i2c_slave_rx_size());
      change_byte_order(&g_update_pkg.data.pkg_crc, sizeof(g_update_pkg.data.pkg_crc));
      change_byte_order(&g_update_pkg.data.pkg_num, sizeof(g_update_pkg.data.pkg_num));
      change_byte_order(&g_update_pkg.data.data_len, sizeof(g_update_pkg.data.data_len));
    } break;
    default: {
      return;
    } break;
  }
  
  /* 升级包准备好后置位 */
  disable_global_irq();
  sys->ctrl.update.need_process = 1;
  enable_global_irq();
}
```
  
### 升级固件发送
  
这里使用 python 实现串口发送固件的功能，依赖 `pyserial` 三方库实现的串口收发功能，同一套代码可以方便的在 linux 或 windows 平台执行。
  
在固件传输前，先定义一个 `uint16_t` 类型的变量，赋值 `0xFFFE`，取出该变量第一个字节，作为 ID 发送给 MCU，来调整发送帧的字节序。
  
固件发送时，采用分包的策略发送，每个包携带的固件数据长度可以按情况进行自定义，长度为 8 的倍数；默认按照每包 1024 字节固件数据来发。
  
具体代码实现见 [mySerial.py](/tools/mySerial.py )、[update.py](/tools/update.py )、[update_i2c.py](https://github.com/skb666/stm32f3discovery_demo/blob/main/tools/update_i2c.py )
  
### 内存映射设置
  
通过修改链接脚本或对应的内存映射文件，使编译生成的目标文件内符号映射到正确的 FLASH 地址，还可用于限制生成目标的大小
  
**bld**
  
Makefile 链接脚本：[bld/STM32L431RCTx_FLASH.ld](/bld/STM32L431RCTx_FLASH.ld )  
emStudio 内存映射：[bld/emStudio/STM32L431RCTx_MemoryMap.xml](/bld/emStudio/STM32L431RCTx_MemoryMap.xml )
  
**app**
  
Makefile 链接脚本：[app/STM32L431RCTx_FLASH.ld](/app/STM32L431RCTx_FLASH.ld )  
emStudio 内存映射：[app/emStudio/STM32L431RCTx_MemoryMap.xml](/app/emStudio/STM32L431RCTx_MemoryMap.xml )
  
## 参考链接
  
1. [STM32CubeIDE IAP原理讲解，及UART双APP交替升级IAP实现](https://blog.csdn.net/sudaroot/article/details/106932736 )
2. [IAP技术原理](https://www.yii666.com/article/519050.html )
  
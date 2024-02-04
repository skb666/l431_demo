  
  
# i2c 从机实现
  
**多字节写时序**
  
<table>
    <tr>
        <td style="text-align:center;">主机</td>
        <td style="text-align:center;">START</td>
        <td style="text-align:center;">设备地址 + W</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">寄存器地址高8位</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">寄存器地址低8位</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">发送数据</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">......</td>
        <td style="text-align:center;">发送数据</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">STOP</td>
    </tr>
    <tr>
        <td style="text-align:center;">从机</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">ACK</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">ACK</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">ACK</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">ACK</td>
        <td style="text-align:center;">......</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">ACK</td>
        <td style="text-align:center;"> </td>
    </tr>
</table>
  
**多字节读时序**
  
<table>
    <tr>
        <td style="text-align:center;">主机</td>
        <td style="text-align:center;">START</td>
        <td style="text-align:center;">设备地址 + W</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">寄存器地址高8位</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">寄存器地址低8位</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">START</td>
        <td style="text-align:center;">设备地址 + R</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">ACK</td>
        <td style="text-align:center;">......</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">NACK</td>
        <td style="text-align:center;">STOP</td>
    </tr>
    <tr>
        <td style="text-align:center;">从机</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">ACK</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">ACK</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">ACK</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">ACK</td>
        <td style="text-align:center;">发送数据</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;">......</td>
        <td style="text-align:center;">发送数据</td>
        <td style="text-align:center;"> </td>
        <td style="text-align:center;"> </td>
    </tr>
</table>
  
## 设备状态信息结构
  
```c
typedef struct {
  I2C_SLAVE_STATUS status;
  uint16_t reg_addr_size;
  REG_ADDR_TYPE reg_addr;
  RING_FIFO rx_ring;
  RING_FIFO tx_ring;
} i2c_device_t;
```
  
相关枚举及宏定义：
  
```c
#define I2C_SLAVE_TYPE I2C1
  
#define I2C_SLAVE_ADDRESS (0x3c << 1)
  
#define I2C_RX_RING_SIZE 1280
#define I2C_TX_RING_SIZE 16
  
typedef enum {
  I2C_STATUS_REG = 0,
  I2C_STATUS_DATA,
} I2C_SLAVE_STATUS;
```
  
## 内部寄存器
  
**寄存器信息结构**
  
```c
typedef struct {
  REG_ADDR_TYPE addr;
  REG_ATTRIB attrib;
  void (*read_cb)(void);
  void (*write_cb)(void);
} REG_T;
```
  
相关枚举及宏定义：
  
```c
#ifndef REG_ADDR_SIZE
#define REG_ADDR_SIZE 2
#endif
  
#if (REG_ADDR_SIZE == 1)
#define REG_ADDR_TYPE uint8_t
#elif (REG_ADDR_SIZE == 2)
#define REG_ADDR_TYPE uint16_t
#elif (NUM_BITS == 4)
#define REG_ADDR_TYPE uint32_t
#elif (NUM_BITS == 8)
#define REG_ADDR_TYPE uint64_t
#endif
  
typedef enum {
  REG_RO = 0,
  REG_RW,
} REG_ATTRIB;
```
  
**寄存器列表**
  
```c
static REG_T s_reg_list[] = {
    {REG_VERSION, REG_RO, reg_read_cb_version, NULL},
};
const static uint32_t s_reg_list_size = ARRAY_SIZE(s_reg_list);
  
void *reg_list_get(uint32_t *num) {
  *num = s_reg_list_size;
  return s_reg_list;
}
```
  
每个寄存器可绑定指定的读写处理函数用于回调，在该列表内向下添加
  
对应的处理函数统一放于 [i2c_protocol.c](/bld/User/i2c_slave/src/i2c_protocol.c )
  
**寄存器排序及查找**
  
```c
static int reg_addr_cmp(const void *reg_a, const void *reg_b) {
  if (((REG_T *)reg_a)->addr > ((REG_T *)reg_b)->addr) {
    return 1;
  } else if (((REG_T *)reg_a)->addr < ((REG_T *)reg_b)->addr) {
    return -1;
  } else {
    return 0;
  }
}
  
static int reg_find_cmp(const void *addr, const void *reg) {
  if (*(REG_ADDR_TYPE *)addr > ((REG_T *)reg)->addr) {
    return 1;
  } else if (*(REG_ADDR_TYPE *)addr < ((REG_T *)reg)->addr) {
    return -1;
  } else {
    return 0;
  }
}
  
static void reg_list_init(void) {
  REG_T *reg_list;
  uint32_t list_size;
  
  reg_list = reg_list_get(&list_size);
  
  qsort(reg_list, list_size, sizeof(REG_T), reg_addr_cmp);
}
  
static REG_T *reg_addr_find(REG_ADDR_TYPE addr) {
  REG_T *reg, *reg_list;
  uint32_t list_size;
  
  reg_list = reg_list_get(&list_size);
  
  reg = bsearch(&addr, reg_list, list_size, sizeof(REG_T), reg_find_cmp);
  
  return reg;
}
```
  
## 从机初始化配置
  
**收发数据缓存**
  
使用之前设计的通用环形队列模块 [RING_FIFO](https://github.com/skb666/RING_FIFO ) 定义两个 FIFO 作为数据缓冲区。
  
```c
static uint8_t __i2c_rx_ring_data[I2C_RX_RING_SIZE];
static uint8_t __i2c_tx_ring_data[I2C_TX_RING_SIZE];
```
  
**初始化步骤**
  
1. 设置从机地址
2. 使能 ADDR、NACK、ERR、STOP 中断
3. 初始化收发数据缓存
4. 将寄存器列表根据寄存器地址进行排序，方便后续寻址
  
```c
void i2c_slave_config(void) {
  LL_I2C_Disable(I2C_SLAVE_TYPE);
  LL_I2C_SetOwnAddress1(I2C_SLAVE_TYPE, I2C_SLAVE_ADDRESS, LL_I2C_OWNADDRESS1_7BIT);
  LL_I2C_EnableOwnAddress1(I2C_SLAVE_TYPE);
  LL_I2C_Enable(I2C_SLAVE_TYPE);
  
  LL_I2C_EnableIT_ADDR(I2C_SLAVE_TYPE);
  LL_I2C_EnableIT_NACK(I2C_SLAVE_TYPE);
  LL_I2C_EnableIT_ERR(I2C_SLAVE_TYPE);
  LL_I2C_EnableIT_STOP(I2C_SLAVE_TYPE);
  
  /* INIT */
  i2c_dev.reg_addr_size = REG_ADDR_SIZE;
  i2c_dev.rx_ring = (RING_FIFO){
      .buffer = __i2c_rx_ring_data,
      .capacity = I2C_RX_RING_SIZE,
      .element_size = sizeof(__i2c_rx_ring_data[0]),
      .cover = 0,
      .head = 0,
      .tail = 0,
      .size = 0,
  };
  i2c_dev.tx_ring = (RING_FIFO){
      .buffer = __i2c_tx_ring_data,
      .capacity = I2C_TX_RING_SIZE,
      .element_size = sizeof(__i2c_rx_ring_data[0]),
      .cover = 0,
      .head = 0,
      .tail = 0,
      .size = 0,
  };
  
  reg_list_init();
}
```
  
**收发缓冲区数据读写接口**
  
```c
uint16_t i2c_slave_tx_get(uint8_t *buf, uint16_t size) {
  uint16_t ok = 0;
  
  if (buf == NULL) {
    return 0;
  }
  
  if (ring_is_empty(&i2c_dev.tx_ring)) {
    return 0;
  }
  
  disable_global_irq();
  ok = ring_pop_mult(&i2c_dev.tx_ring, buf, size);
  enable_global_irq();
  
  return ok;
}
  
uint16_t i2c_slave_tx_put(const uint8_t *buf, uint16_t size) {
  uint16_t ok = 0;
  
  if (buf == NULL) {
    return 0;
  }
  
  disable_global_irq();
  ok = ring_push_mult(&i2c_dev.tx_ring, buf, size);
  enable_global_irq();
  
  return ok;
}
  
uint16_t i2c_slave_rx_get(uint8_t *buf, uint16_t size) {
  uint16_t ok = 0;
  
  if (buf == NULL) {
    return 0;
  }
  
  if (ring_is_empty(&i2c_dev.rx_ring)) {
    return 0;
  }
  
  disable_global_irq();
  ok = ring_pop_mult(&i2c_dev.rx_ring, buf, size);
  enable_global_irq();
  
  return ok;
}
  
uint16_t i2c_slave_rx_put(const uint8_t *buf, uint16_t size) {
  uint16_t ok = 0;
  
  if (buf == NULL) {
    return 0;
  }
  
  disable_global_irq();
  ok = ring_push_mult(&i2c_dev.rx_ring, buf, size);
  enable_global_irq();
  
  return ok;
}
  
uint16_t i2c_slave_tx_size(void) {
  return ring_size(&i2c_dev.tx_ring);
}
  
uint16_t i2c_slave_rx_size(void) {
  return ring_size(&i2c_dev.rx_ring);
}
```
  
## 从机中断服务回调
  
**从机状态复原、重置数据收发缓冲区**
  
```c
static void i2c_dev_reset(void) {
  i2c_dev.status = I2C_STATUS_REG;
  
  disable_global_irq();
  ring_reset(&i2c_dev.rx_ring);
  ring_reset(&i2c_dev.tx_ring);
  enable_global_irq();
}
```
  
**数据接收**
  
1. RXNE 中断标志置位时，将接收到的数据放入接收缓冲区
2. 若当前设备处于寄存器寻址状态，读出寄存器地址，并切换到数据接收状态
  
```c
static void i2c_reception_cb(void) {
  uint8_t data;
  
  data = LL_I2C_ReceiveData8(I2C_SLAVE_TYPE);
  i2c_slave_rx_put(&data, 1);
  
  if (i2c_dev.status == I2C_STATUS_REG) {
    if (i2c_slave_rx_size() >= i2c_dev.reg_addr_size) {
      i2c_dev.reg_addr = 0;
      for (uint16_t i = 0; i < i2c_dev.reg_addr_size; ++i) {
        i2c_slave_rx_get(&data, 1);
        i2c_dev.reg_addr <<= 8;
        i2c_dev.reg_addr |= data;
      }
      i2c_dev.status = I2C_STATUS_DATA;
    }
  }
}
```
  
**读数据准备**
  
地址匹配中断标志置位，并且数据方向为读取方向时，根据寄存器地址，调用读回调函数，将要被读取的数据放入发送缓冲区
  
```c
static void i2c_prepare_data(void) {
  REG_T *reg;
  
  reg = reg_addr_find(i2c_dev.reg_addr);
  if (!reg) {
    return;
  }
  
  if (reg->read_cb) {
    reg->read_cb();
  }
}
```
  
**数据发送**
  
当数据发送缓冲区有数据时取出发送，无数据时发送 0xFF
  
```c
static void i2c_transmit_cb(void) {
  uint8_t data;
  
  if (i2c_slave_tx_get(&data, 1)) {
    LL_I2C_TransmitData8(I2C_SLAVE_TYPE, data);
  } else {
    LL_I2C_TransmitData8(I2C_SLAVE_TYPE, 0xFF);
  }
}
```
  
**数据接收完成处理**
  
当数据接收缓冲区不为空时，根据寄存器地址，调用写回调函数，读取缓存区内数据并处理
  
```c
static void i2c_complete_cb(void) {
  REG_T *reg;
  
  if (i2c_slave_rx_size()) {
    reg = reg_addr_find(i2c_dev.reg_addr);
    if (!reg) {
      return;
    }
  
    if ((reg->attrib != REG_RO) && reg->write_cb) {
      reg->write_cb();
    }
  }
}
```
  
**中断服务回调**
  
```c
void i2c_ev_isr(void) {
  /* Check ADDR flag value in ISR register */
  if (LL_I2C_IsActiveFlag_ADDR(I2C_SLAVE_TYPE)) {
    /* Verify the Address Match with the OWN Slave address */
    if (LL_I2C_GetAddressMatchCode(I2C_SLAVE_TYPE) == I2C_SLAVE_ADDRESS) {
      /* Call function Slave Reset Callback */
      i2c_dev_reset();
  
      /* Verify the transfer direction, a write direction, Slave enters receiver mode */
      if (LL_I2C_GetTransferDirection(I2C_SLAVE_TYPE) == LL_I2C_DIRECTION_WRITE) {
        /* Clear ADDR flag value in ISR register */
        LL_I2C_ClearFlag_ADDR(I2C_SLAVE_TYPE);
  
        /* Enable Receive Interrupt */
        LL_I2C_EnableIT_RX(I2C_SLAVE_TYPE);
      } else if (LL_I2C_GetTransferDirection(I2C_SLAVE_TYPE) == LL_I2C_DIRECTION_READ) {
        /* Clear ADDR flag value in ISR register */
        LL_I2C_ClearFlag_ADDR(I2C_SLAVE_TYPE);
  
        /* Call function Slave Prepare Data Callback */
        i2c_prepare_data();
  
        /* Enable Transmit Interrupt */
        LL_I2C_EnableIT_TX(I2C_SLAVE_TYPE);
      }
    } else {
      /* Clear ADDR flag value in ISR register */
      LL_I2C_ClearFlag_ADDR(I2C_SLAVE_TYPE);
  
      /* Call Error function */
      Error_Handler();
    }
  }
  /* Check NACK flag value in ISR register */
  else if (LL_I2C_IsActiveFlag_NACK(I2C_SLAVE_TYPE)) {
    /* End of Transfer */
    LL_I2C_ClearFlag_NACK(I2C_SLAVE_TYPE);
  }
  /* Check RXNE flag value in ISR register */
  else if (LL_I2C_IsActiveFlag_RXNE(I2C_SLAVE_TYPE)) {
    /* Call function Slave Reception Callback */
    i2c_reception_cb();
  }
  /* Check TXIS flag value in ISR register */
  else if (LL_I2C_IsActiveFlag_TXIS(I2C_SLAVE_TYPE)) {
    /* Call function Slave Ready to Transmit Callback */
    i2c_transmit_cb();
  }
  /* Check STOP flag value in ISR register */
  else if (LL_I2C_IsActiveFlag_STOP(I2C_SLAVE_TYPE)) {
    /* End of Transfer */
    LL_I2C_ClearFlag_STOP(I2C_SLAVE_TYPE);
  
    /* Check TXE flag value in ISR register */
    if (!LL_I2C_IsActiveFlag_TXE(I2C_SLAVE_TYPE)) {
      /* Flush TX buffer */
      LL_I2C_ClearFlag_TXE(I2C_SLAVE_TYPE);
    }
  
    if (LL_I2C_GetTransferDirection(I2C_SLAVE_TYPE) == LL_I2C_DIRECTION_WRITE) {
      /* Call function Slave Complete Callback */
      i2c_complete_cb();
    }
  }
  /* Check TXE flag value in ISR register */
  else if (!LL_I2C_IsActiveFlag_TXE(I2C_SLAVE_TYPE)) {
    /* Do nothing */
    /* This Flag will be set by hardware when the TXDR register is empty */
    /* If needed, use LL_I2C_ClearFlag_TXE() interface to flush the TXDR register  */
  } else {
    /* Call Error function */
    Error_Handler();
  }
}
```
  
## 异常检测及恢复
  
从机长时间处于 BUSY 状态时，重新使能 I2C
  
```c
void i2c_abnormal_check(uint16_t timeout) {
  static uint16_t i2c_busy_time = 0;
  
  if (LL_I2C_IsActiveFlag_BUSY(I2C_SLAVE_TYPE)) {
    i2c_busy_time += 1;
  } else {
    i2c_busy_time = 0;
  }
  
  if (i2c_busy_time >= timeout) {
    i2c_busy_time = 0;
    LL_I2C_Disable(I2C_SLAVE_TYPE);
    LL_I2C_Enable(I2C_SLAVE_TYPE);
  }
}
```
  
可在定时器内 1ms 周期调用 `i2c_abnormal_check(300);`
  
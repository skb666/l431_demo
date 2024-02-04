  
  
# 稳定 DMA 串口通信
  
## 前言
  
直接存储器访问（Direct Memory Access），简称DMA。DMA是CPU一个用于数据从一个地址空间到另一地址空间“搬运”（拷贝）的组件，数据拷贝过程不需CPU干预，数据拷贝结束则通知CPU处理。因此，大量数据拷贝时，使用DMA可以释放CPU资源。DMA数据拷贝过程，典型的有：
  
+ 内存—>内存，内存间拷贝
+ 外设—>内存，如uart、spi、i2c等总线接收数据过程
+ 内存—>外设，如uart、spi、i2c等总线发送数据过程
  
**串口有必要使用 DMA 吗**
  
串口(uart)是一种低速的串行异步通信，适用于低速通信场景，通常使用的波特率小于或等于115200bps。对于小于或者等于115200bps波特率的，而且数据量不大的通信场景，一般没必要使用DMA，或者说使用DMA并未能充分发挥出DMA的作用。  
对于数量大，或者波特率提高时，必须使用DMA以释放CPU资源，因为高波特率可能带来这样的问题：
  
+ 对于发送，使用循环发送，可能阻塞线程，需要消耗大量CPU资源“搬运”数据，浪费CPU
+ 对于发送，使用中断发送，不会阻塞线程，但需浪费大量中断资源，CPU频繁响应中断；以115200bps波特率，1s传输11520字节，大约69us需响应一次中断，如波特率再提高，将消耗更多CPU资源
+ 对于接收，如仍采用传统的中断模式接收，同样会因为频繁中断导致消耗大量CPU资源
  
因此，高波特率场景下，串口非常有必要使用DMA。
  
## 方案设计
  
关于STM32串口使用DMA，不乏一些开发板例程及网络上一些博主的使用教程。使用步骤、流程、配置基本大同小异，正确性也没什么毛病，但都是一些基本的Demo例子，作为学习过程没问题；实际项目使用缺乏严谨性，数据量大时可能导致数据异常。
  
**整体框图**
  
![global](/assets/image/global.png )
  
基础代码使用 STM32CubeMX 生成，配置如下
  
**串口基础参数**
  
![uart](/assets/image/uart.png )
  
**DMA 通道配置**
  
![dma](/assets/image/dma.png )
  
**中断优先级**
  
![nvic](/assets/image/nvic.png )
  
**串口状态存储结构**
  
```c
typedef struct {
  volatile uint16_t status; /* 发送状态 */
  uint16_t last_dmarx_size; /* dma上一次接收数据大小 */
  RING_FIFO rx_ring;
  RING_FIFO tx_ring;
  void (*rx_monitor)(uint8_t *, uint16_t);
  void (*tx_monitor)(uint8_t *, uint16_t);
} uart_device_t;
```
  
### 数据流缓存
  
#### DMA 收发缓存
  
```c
_RAM_DATA static uint8_t uart_dmarx_buf[DEV_NUM][UART_DMARX_BUF_SIZE];
_RAM_DATA static uint8_t uart_dmatx_buf[DEV_NUM][UART_DMATX_BUF_SIZE];
```
  
#### CPU 处理缓存
  
使用之前设计的通用环形队列模块 [RING_FIFO](https://github.com/skb666/RING_FIFO ) 定义两个 FIFO 作为 CPU 处理部分的缓冲。
  
```c
static uint8_t __uart_rx_ring_data[DEV_NUM][UART_RX_RING_SIZE];
static uint8_t __uart_tx_ring_data[DEV_NUM][UART_TX_RING_SIZE];
```
  
### 串口 DMA 接收
  
#### 接收总体流程
  

![](/assets/other/bac20c535e54e4eb7d42dcb81e1751cb0.png?0.7293600700300915)  
  
#### 接收关键配置
  
```c
/* USART_RX DMA */
LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5,
    LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_RECEIVE),
    (uint32_t)uart_dmarx_buf[DEV_USART1],
    LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5));
LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, UART_DMARX_BUF_SIZE);
  
  
LL_DMA_ClearFlag_HT5(DMA1);
LL_DMA_ClearFlag_TC5(DMA1);
LL_DMA_ClearFlag_TE5(DMA1);
  
LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_5);
LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
  
LL_USART_EnableDMAReq_RX(USART1);
LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
  
LL_USART_EnableIT_IDLE(USART1);
  
/* INIT */
uart_dev[dev_type].rx_ring = (RING_FIFO){
    .buffer = __uart_rx_ring_data[dev_type],
    .capacity = UART_RX_RING_SIZE,
    .element_size = sizeof(__uart_rx_ring_data[dev_type][0]),
    .cover = 1,
    .head = 0,
    .tail = 0,
    .size = 0,
};
  
// 默认按大端方式传输数据
rx_frame[dev_type].byte_order = 1;
rx_frame[dev_type].data = __frame_data[dev_type];
```
  
1. 初始化串口
2. 使能串口 DMA 接收模式，使能串口空闲中断
3. 配置 DMA 参数，使能 DMA 通道 buffer 半传输中断、传输完成中断
  
很多串口 DMA 模式接收的教程、例子，基本是使用了 **空闲中断** + **DMA传输完成中断** 来接收数据。实质上这是存在风险的，当 DMA 传输数据完成，CPU 介入开始拷贝 DMA 通道 buffer 数据，如果此时串口继续有数据进来，DMA 继续搬运数据到 buffer，就有可能将数据覆盖，因为 DMA 数据搬运是不受 CPU 控制的，即使你关闭了 CPU 中断。  
严谨的做法需要做双 buffer，CPU 和 DMA 各自一块内存交替访问，即是 **乒乓缓存**，处理流程步骤应该是这样:
  
1. DMA 将数据搬运完成 buffer 的前一半时，产生 **DMA半传输中断**，CPU 来拷贝 buffer 前半部分数据
2. DMA 继续将数据搬运到 buffer 的后半部分，与 CPU 拷贝 buffer 前半部数据不会冲突
3. buffer 后半部分数据搬运完成，触发 **DMA传输完成中断**，CPU 来拷贝 buf 后半部分数据
4. 执行完第三步，DMA 返回执行第一步，一直循环
  
UART DMA 模式接收配置代码如上，与其他外设使用 DMA 的配置基本一致，留意关键配置：
  
+ 串口接收，DMA 通道工作模式设为连续模式
+ 使能 DMA 通道接收 buffer 半满中断、溢满（传输完成）中断
+ 启动 DMA 通道前清空相关状态标识，防止首次传输错乱数据
  
#### 数据接收处理
  
数据传输过程是随机的，数据大小也是不定的，存在几类情况：
  
+ 数据刚好是 DMA 接收 buffer 的整数倍，这是理想的状态
+ 数据量小于 DMA 接收 buffer 或者小于接收 buffer 的一半，此时会触发串口空闲中断
  
因此，我们需根据 **DMA 通道 buffer 大小**、**DMA 通道 buffer 剩余空间大小**、**上一次接收的总数据大小** 来计算当前接收的数据大小。
  
**DMA 通道 buffer 溢满场景**
  
接收数据大小 = DMA 通道 buffer 大小 - 上一次接收的总数据大小
  
```c
/**
 * @brief  串口dma接收完成中断处理
 * @param
 * @retval
 */
void uart_dmarx_done_isr(DEV_TYPE dev_type) {
  uint16_t recv_size;
  
  recv_size = UART_DMARX_BUF_SIZE - uart_dev[dev_type].last_dmarx_size;
  
  disable_global_irq();
  ring_push_mult(&uart_dev[dev_type].rx_ring, &uart_dmarx_buf[dev_type][uart_dev[dev_type].last_dmarx_size], recv_size);
  enable_global_irq();
  
  if (uart_dev[dev_type].rx_monitor) {
    uart_dev[dev_type].rx_monitor(&uart_dmarx_buf[dev_type][uart_dev[dev_type].last_dmarx_size], recv_size);
  }
  
  uart_dev[dev_type].last_dmarx_size = 0;
}
```
  
**DMA 通道 buffer 半满场景**
  
接收数据大小 = DMA 通道接收总数据大小 - 上一次接收的总数据大小
  
DMA 通道接收总数据大小 = DMA 通道 buffer 大小 - DMA 通道 buffer 剩余空间大小
  
```c
/**
 * @brief  串口dma接收部分数据中断处理
 * @param
 * @retval
 */
void uart_dmarx_part_done_isr(DEV_TYPE dev_type) {
  uint16_t recv_total_size;
  uint16_t recv_size;
  
  switch (dev_type) {
    case DEV_USART1: {
      recv_total_size = UART_DMARX_BUF_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
    } break;
    default: {
      return;
    } break;
  }
  recv_size = recv_total_size - uart_dev[dev_type].last_dmarx_size;
  
  disable_global_irq();
  ring_push_mult(&uart_dev[dev_type].rx_ring, &uart_dmarx_buf[dev_type][uart_dev[dev_type].last_dmarx_size], recv_size);
  enable_global_irq();
  
  if (uart_dev[dev_type].rx_monitor) {
    uart_dev[dev_type].rx_monitor(&uart_dmarx_buf[dev_type][uart_dev[dev_type].last_dmarx_size], recv_size);
  }
  
  uart_dev[dev_type].last_dmarx_size = recv_total_size;
}
```
  
**串口空闲中断场景计算**
  
串口空闲中断场景的接收数据计算与 **DMA 通道 buffer 半满场景** 计算方式是一样的
  
> 串口空闲中断处理函数，除了将数据拷贝到串口接收fifo中，还可以增加特殊处理，如作为串口数据传输完成标识、不定长度数据处理等等。
  
**接收数据偏移**
  
将有效数据拷贝到 fifo 中，除了需知道有效数据大小外，还需知道数据存储于 DMA 接收 buffer 的偏移地址。有效数据偏移地址只需记录上一次接收的总大小即可，在 DMA 通道 buffer 溢满中断处理函数中将数据偏移地址清零，因为下一次数据将从 buffer 的开头存储。
  
**应用读取串口数据方法**
  
经过前面的处理步骤，已将串口数据拷贝至接收 fifo，应用程序任务只需从 fifo 获取数据进行处理。前提是，处理效率必须大于 DMA 接收搬运数据的效率，否则导致数据丢失或者被覆盖处理。
  
### 串口 DMA 发送
  
#### 发送总体流程
  

![](/assets/other/bac20c535e54e4eb7d42dcb81e1751cb1.png?0.6695467683981755)  
  
#### 发送关键配置
  
```c
/* USART_TX DMA */
LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4,
    (uint32_t)uart_dmatx_buf[DEV_USART1],
    LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT),
    LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));
  
LL_DMA_ClearFlag_TC4(DMA1);
LL_DMA_ClearFlag_TE4(DMA1);
  
LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
  
LL_USART_EnableDMAReq_TX(USART1);
  
/* INIT */
uart_dev[dev_type].tx_ring = (RING_FIFO){
    .buffer = __uart_tx_ring_data[dev_type],
    .capacity = UART_TX_RING_SIZE,
    .element_size = sizeof(__uart_tx_ring_data[dev_type][0]),
    .cover = 0,
    .head = 0,
    .tail = 0,
    .size = 0,
};
```
  
1. 初始化串口
2. 使能串口 DMA 发送模式
3. 配置 DMA 发送通道，这一步无需在初始化时设置，有数据需要发送时才配置使能 DMA 发送通道
  
UART DMA 模式发送配置代码如上，与其他外设使用 DMA 的配置基本一致，留意关键配置：
  
+ 串口发送时，DMA 通道工作模式设为单次模式（正常模式），每次需要发送数据时重新配置 DMA
+ 使能 DMA 通道传输完成中断，利用该中断信息处理一些必要的任务，如清空发送状态、启动下一次传输
+ 启动 DMA 通道前清空相关状态标识，防止首次传输错乱数据
  
#### 数据发送处理
  
串口待发送数据存于发送 fifo 中，发送处理函数需要做的的任务就是循环查询发送 fifo 是否存在数据，如存在则将该数据拷贝到 DMA 发送 buffer 中，然后启动 DMA 传输。前提是需要等待上一次 DMA 传输完毕，即是 DMA 接收到 DMA 传输完成中断信号 `DMA_IT_TC`
  
**DMA 传输完成中断处理**
  
```c
/**
 * @brief  串口dma发送完成中断处理
 * @param
 * @retval
 */
void uart_dmatx_done_isr(DEV_TYPE dev_type) {
  uart_dev[dev_type].status = 0; /* DMA发送空闲 */
}
```
  
此处清空DMA发送状态标识
  
**串口发送处理**
  
发送状态标识，必须先置为 `发送状态` ，然后启动 DMA 传输。如果步骤反过来，在传输数据量少时，DMA 传输时间短，`DMA_IT_TC` 中断可能比 `发送状态标识置位` 先执行，导致程序误判DMA 一直处理发送状态（发送标识无法被清除）。
  
```c
void uart_tx_poll(DEV_TYPE dev_type) {
  uint16_t size = 0;
  
  if (uart_dev[dev_type].status) {
    return;
  }
  
  if (ring_is_empty(&uart_dev[dev_type].tx_ring)) {
    return;
  }
  
  disable_global_irq();
  size = ring_pop_mult(&uart_dev[dev_type].tx_ring, uart_dmatx_buf[dev_type], UART_DMATX_BUF_SIZE);
  enable_global_irq();
  
  if (uart_dev[dev_type].tx_monitor) {
    uart_dev[dev_type].tx_monitor(uart_dmatx_buf[dev_type], size);
  }
  
  uart_dev[dev_type].status = 1;
  
  switch (dev_type) {
    case DEV_USART1: {
      LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
      LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, size);
      LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
    } break;
    default: {
    } break;
  }
}
```
  
**每次拷贝多少数据量到 DMA 发送 buffer**
  
关于这个问题，与具体应用场景有关，遵循的原则就是：只要发送 fifo 的数据量大于等于 DMA 发送 buffer 的大小，就应该填满 DMA 发送 buffer，然后启动 DMA 传输，这样才能充分发挥会 DMA 性能。因此，需兼顾每次 DMA 传输的效率和串口数据流实时性，参考着几类实现：
  
+ 周期查询发送 fifo 数据，启动 DMA 传输，充分利用 DMA 发送效率，但可能降低串口数据流实时性
+ 实时查询发送 fifo 数据，加上超时处理，理想的方法
+ 在 DMA 传输完成中断中处理，保证实时连续数据流
  
这里各缓存数组大小设置如下
  
```c
#define UART_RX_RING_SIZE 256
#define UART_TX_RING_SIZE 256
#define UART_DMARX_BUF_SIZE 64
#define UART_DMATX_BUF_SIZE 64
```
  
**外部串口读写接口**
  
```c
void uart_wait_tx(DEV_TYPE dev_type, uint32_t timeout) {
  if (!uart_dev[dev_type].status) {
    return;
  }
  while (uart_dev[dev_type].status) {
    if (LL_SYSTICK_IsActiveCounterFlag()) {
      if (timeout-- == 0) {
        return;
      }
    }
  }
}
  
uint16_t uart_read(DEV_TYPE dev_type, uint8_t *buf, uint16_t size) {
  uint16_t ok = 0;
  
  if (buf == NULL) {
    return 0;
  }
  
  if (ring_is_empty(&uart_dev[dev_type].rx_ring)) {
    return 0;
  }
  
  disable_global_irq();
  ok = ring_pop_mult(&uart_dev[dev_type].rx_ring, buf, size);
  enable_global_irq();
  
  return ok;
}
  
uint16_t uart_write(DEV_TYPE dev_type, const uint8_t *buf, uint16_t size) {
  uint16_t ok = 0;
  
  if (buf == NULL) {
    return 0;
  }
  
  disable_global_irq();
  ok = ring_push_mult(&uart_dev[dev_type].tx_ring, buf, size);
  enable_global_irq();
  
  return ok;
}
  
void uart_printf(DEV_TYPE dev_type, const char *format, ...) {
  va_list args;
  uint32_t length;
  uint16_t success = 0;
  uint8_t *pbuf;
  
  va_start(args, format);
  length = vsnprintf((char *)print_buf, UART_TX_RING_SIZE, (char *)format, args);
  va_end(args);
  
  pbuf = print_buf;
  
  do {
    success = uart_write(dev_type, pbuf, length);
  
    // if (success == length) {
    //   return;
    // }
  
    uart_tx_poll(dev_type);
    uart_wait_tx(dev_type, 50);
    pbuf += success;
    length -= success;
  } while (length);
}
  
void uart_puts(DEV_TYPE dev_type, uint8_t *buf, uint16_t len) {
  uint16_t success = 0;
  uint8_t *pbuf;
  
  pbuf = buf;
  
  do {
    success = uart_write(dev_type, pbuf, len);
  
    // if (success == len) {
    //   return;
    // }
  
    uart_tx_poll(dev_type);
    uart_wait_tx(dev_type, 50);
    pbuf += success;
    len -= success;
  } while (len);
}
```
  
### 串口帧接收及处理
  
#### 串口帧结构
  
|   HEAD    |       ID        |        LENGTH        |        DATA        |
| :-------: | :-------------: | :------------------: | :----------------: |
| 0x55 0xAA | 帧类型 (1 Byte) | 数据长度 n (2 Bytes) | 数据内容 (n Bytes) |
  
相关宏及类型定义：
  
```c
#define FRAME_DATA_LEN_MAX 1280
#define FRAME_HEAD1 0x55
#define FRAME_HEAD2 0xAA
  
typedef enum {
  DEV_USART1,
  DEV_NUM,
} DEV_TYPE;
  
typedef enum {
  PARSE_STAT_HEAD1 = 0,
  PARSE_STAT_HEAD2,
  PARSE_STAT_ID,
  PARSE_STAT_LENGTH,
  PARSE_STAT_DATA,
} FRAME_PARSE_STAT;
  
typedef struct {
  uint8_t status;
  uint8_t id;
  uint8_t byte_order;
  uint16_t length;
  uint16_t recv_size;
  uint8_t *data;
} frame_parse_t;
```
  
#### 帧处理函数注册
  
```c
#define FUNC_LIST_MAX 0xFE
  
static void (*func_list[DEV_NUM][FUNC_LIST_MAX])(frame_parse_t *);
  
int8_t frame_parse_register(uint8_t index, void (*func)(frame_parse_t *)) {
  if (func == 0 || index >= FUNC_LIST_MAX) {
    return -1;
  }
  
  if (func_list[index] == 0) {
    func_list[index] = func;
    return 0;
  } else {
    return -1;
  }
}
```
  
外部定义帧处理函数，原型为 `void (*func)(frame_parse_t *)`，函数接收一个 `帧指针类型` 的参数，无返回值。
  
本函数将 id 与 对应的处理函数关联，使串口完成一帧的接收后，可以根据 id 执行对应的处理函数。
  
#### 接收状态机
  

![](/assets/other/bac20c535e54e4eb7d42dcb81e1751cb2.png?0.6843867716497238)  
  
**ID 的特殊用途**
  
将 `0xFF` 与 `0xFE` 保留，用于修改接收数据的字节序。
  
系统默认小端接收，当 length 完成接收时，若帧字节序为大端，则需要翻转 length。
  
**完整代码**
  
```c
void uart_frame_parse(DEV_TYPE dev_type) {
  uint16_t size = 0;
  uint8_t rx;
  
  switch (rx_frame[dev_type].status) {
    case PARSE_STAT_HEAD1: {
      size = uart_read(dev_type, &rx, 1);
      if (size) {
        if (rx == FRAME_HEAD1) {
          rx_frame[dev_type].status = PARSE_STAT_HEAD2;
        }
      }
    } break;
    case PARSE_STAT_HEAD2: {
      size = uart_read(dev_type, &rx, 1);
      if (size) {
        if (rx == FRAME_HEAD2) {
          rx_frame[dev_type].status = PARSE_STAT_ID;
        } else {
          rx_frame[dev_type].status = PARSE_STAT_HEAD1;
        }
      }
    } break;
    case PARSE_STAT_ID: {
      size = uart_read(dev_type, &rx_frame[dev_type].id, 1);
      if (size) {
        if (rx_frame[dev_type].id == 0xFF) {
          rx_frame[dev_type].byte_order = 1;
          rx_frame[dev_type].status = PARSE_STAT_HEAD1;
        } else if (rx_frame[dev_type].id == 0xFE) {
          rx_frame[dev_type].byte_order = 0;
          rx_frame[dev_type].status = PARSE_STAT_HEAD1;
        } else if (rx_frame[dev_type].id < FUNC_LIST_MAX && func_list[dev_type][rx_frame[dev_type].id]) {
          rx_frame[dev_type].status = PARSE_STAT_LENGTH;
        } else {
          rx_frame[dev_type].status = PARSE_STAT_HEAD1;
          // Error_Handler();
        }
      }
    } break;
    case PARSE_STAT_LENGTH: {
      size = uart_read(dev_type, (uint8_t *)&rx_frame[dev_type].length + rx_frame[dev_type].recv_size, sizeof(rx_frame[dev_type].length) - rx_frame[dev_type].recv_size);
      if (size) {
        rx_frame[dev_type].recv_size += size;
      }
      if (rx_frame[dev_type].recv_size >= sizeof(rx_frame[dev_type].length)) {
        if (rx_frame[dev_type].byte_order) {
          change_byte_order(&rx_frame[dev_type].length, sizeof(rx_frame[dev_type].length));
        }
        if (rx_frame[dev_type].length > FRAME_DATA_LEN_MAX) {
          printf_dbg("frame length error!!! (%hu)\n", rx_frame[dev_type].length);
          rx_frame[dev_type].status = PARSE_STAT_HEAD1;
        } else {
          rx_frame[dev_type].status = PARSE_STAT_DATA;
        }
        rx_frame[dev_type].recv_size = 0;
      }
    } break;
    case PARSE_STAT_DATA: {
      size = uart_read(dev_type, rx_frame[dev_type].data + rx_frame[dev_type].recv_size, rx_frame[dev_type].length - rx_frame[dev_type].recv_size);
      if (size) {
        rx_frame[dev_type].recv_size += size;
      }
      if (rx_frame[dev_type].recv_size >= rx_frame[dev_type].length) {
        rx_frame[dev_type].status = PARSE_STAT_HEAD1;
        rx_frame[dev_type].recv_size = 0;
        func_list[dev_type][rx_frame[dev_type].id](&rx_frame[dev_type]);
      }
    } break;
    default: {
      printf_dbg("frame status error!!!\n");
    } break;
  }
}
```
  
## 参考链接
  
1. [一个严谨的STM32串口DMA发送&接收（1.5Mbps波特率）机制](https://blog.csdn.net/qq_20553613/article/details/108367512 )
2. [STM32H7xx 串口DMA发送&接收（LL库）](https://blog.csdn.net/qq_20553613/article/details/125108990 )
  
#include "device.h"

#include <stdarg.h>

#include "common.h"
#include "main.h"
#include "ring_fifo.h"

typedef enum {
  PARSE_STAT_HEAD1 = 0,
  PARSE_STAT_HEAD2,
  PARSE_STAT_ID,
  PARSE_STAT_LENGTH,
  PARSE_STAT_DATA,
} FRAME_PARSE_STAT;

typedef struct {
  volatile uint16_t status; /* 发送状态 */
  uint16_t last_dmarx_size; /* dma上一次接收数据大小 */
  RING_FIFO rx_ring;
  RING_FIFO tx_ring;
  void (*rx_monitor)(uint8_t *, uint16_t);
  void (*tx_monitor)(uint8_t *, uint16_t);
} uart_device_t;

_CCM_DATA static uint8_t print_buf[UART_TX_RING_SIZE];

static void (*func_list[DEV_NUM][FUNC_LIST_MAX])(frame_parse_t *);

_RAM_DATA static uint8_t uart_dmarx_buf[DEV_NUM][UART_DMARX_BUF_SIZE];
_RAM_DATA static uint8_t uart_dmatx_buf[DEV_NUM][UART_DMATX_BUF_SIZE];

static uint8_t __uart_rx_ring_data[DEV_NUM][UART_RX_RING_SIZE];
static uint8_t __uart_tx_ring_data[DEV_NUM][UART_TX_RING_SIZE];

static uint8_t __frame_data[DEV_NUM][FRAME_DATA_LEN_MAX];

_CCM_DATA static uart_device_t uart_dev[DEV_NUM];
_CCM_DATA static frame_parse_t rx_frame[DEV_NUM];

void uart_config(DEV_TYPE dev_type) {
  switch (dev_type) {
    case DEV_USART1: {
      /* USART_RX DMA */
      LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5,
          LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_RECEIVE),
          (uint32_t)uart_dmarx_buf[DEV_USART1],
          LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5));
      LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, UART_DMARX_BUF_SIZE);

      /* USART_TX DMA */
      LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4,
          (uint32_t)uart_dmatx_buf[DEV_USART1],
          LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT),
          LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));

      LL_DMA_ClearFlag_HT5(DMA1);
      LL_DMA_ClearFlag_TC5(DMA1);
      LL_DMA_ClearFlag_TE5(DMA1);

      LL_DMA_ClearFlag_TC4(DMA1);
      LL_DMA_ClearFlag_TE4(DMA1);

      LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_5);
      LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
      LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);

      LL_USART_EnableDMAReq_RX(USART1);
      LL_USART_EnableDMAReq_TX(USART1);
      LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);

      LL_USART_EnableIT_IDLE(USART1);
    } break;
    default: {
      return;
    } break;
  }

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
  uart_dev[dev_type].tx_ring = (RING_FIFO){
      .buffer = __uart_tx_ring_data[dev_type],
      .capacity = UART_TX_RING_SIZE,
      .element_size = sizeof(__uart_tx_ring_data[dev_type][0]),
      .cover = 0,
      .head = 0,
      .tail = 0,
      .size = 0,
  };

  // 默认按大端方式传输数据
  rx_frame[dev_type].byte_order = 1;
  rx_frame[dev_type].data = __frame_data[dev_type];
}

void uart_set_rx_monitor(DEV_TYPE dev_type, void (*monitor)(uint8_t *, uint16_t)) {
  if (monitor) {
    uart_dev[dev_type].rx_monitor = monitor;
  }
}

void uart_set_tx_monitor(DEV_TYPE dev_type, void (*monitor)(uint8_t *, uint16_t)) {
  if (monitor) {
    uart_dev[dev_type].tx_monitor = monitor;
  }
}

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

/**
 * @brief  串口dma发送完成中断处理
 * @param
 * @retval
 */
void uart_dmatx_done_isr(DEV_TYPE dev_type) {
  uart_dev[dev_type].status = 0; /* DMA发送空闲 */
}

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

int8_t frame_parse_register(DEV_TYPE dev_type, uint8_t index, void (*func)(frame_parse_t *)) {
  if (func == 0 || index >= FUNC_LIST_MAX) {
    return -1;
  }

  if (func_list[dev_type][index] == 0) {
    func_list[dev_type][index] = func;
    return 0;
  } else {
    return -1;
  }
}

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

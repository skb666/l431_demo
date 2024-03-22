#ifndef __UART_DEVICE_H__
#define __UART_DEVICE_H__

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define UART_RX_RING_SIZE 256
#define UART_TX_RING_SIZE 256
#define UART_DMARX_BUF_SIZE 64
#define UART_DMATX_BUF_SIZE 64

#define FRAME_DATA_LEN_MAX 1280
#define FRAME_HEAD1 0x55
#define FRAME_HEAD2 0xAA
#define FUNC_LIST_MAX 0xFE

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  DEV_USART1,
  DEV_NUM,
} DEV_TYPE;

typedef struct {
  DEV_TYPE dev_type;
  uint8_t status;
  uint8_t id;
  uint8_t byte_order;
  uint16_t length;
  uint16_t recv_size;
  uint8_t *data;
} frame_parse_t;

void uart_config(DEV_TYPE dev_type);
void uart_dmarx_done_isr(DEV_TYPE dev_type);
void uart_dmarx_part_done_isr(DEV_TYPE dev_type);
void uart_dmatx_done_isr(DEV_TYPE dev_type);

void uart_set_rx_monitor(DEV_TYPE dev_type, void (*monitor)(uint8_t *, uint16_t));
void uart_set_tx_monitor(DEV_TYPE dev_type, void (*monitor)(uint8_t *, uint16_t));

int8_t frame_parse_register(DEV_TYPE dev_type, uint8_t index, void (*func)(frame_parse_t *));
void uart_frame_parse(DEV_TYPE dev_type);

void uart_wait_tx(DEV_TYPE dev_type, uint32_t timeout);
void uart_tx_poll(DEV_TYPE dev_type);
uint16_t uart_read(DEV_TYPE dev_type, uint8_t *buf, uint16_t size);
uint16_t uart_write(DEV_TYPE dev_type, const uint8_t *buf, uint16_t size);

void uart_printf_wait(DEV_TYPE dev_type, const char *format, ...);
void uart_puts_wait(DEV_TYPE dev_type, uint8_t *buf, uint16_t len);
void uart_printf(DEV_TYPE dev_type, const char *format, ...);
void uart_puts(DEV_TYPE dev_type, uint8_t *buf, uint16_t len);
void uart_printf_block(DEV_TYPE dev_type, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif

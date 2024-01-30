#ifndef __I2C_SLAVE_H__
#define __I2C_SLAVE_H__

#include <stdint.h>

#define I2C_TYPE I2C1

#define I2C_SLAVE_ADDRESS (0x3c << 1)

#define I2C_RX_RING_SIZE 1280
#define I2C_TX_RING_SIZE 16

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

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  REG_RO = 0,
  REG_RW,
} REG_ATTRIB;

typedef struct {
  REG_ADDR_TYPE addr;
  REG_ATTRIB attrib;
  void (*read_cb)(void);
  void (*write_cb)(void);
} REG_T;

void i2c_slave_config(void);
void i2c_ev_isr(void);
void i2c_abnormal_check(uint16_t timeout);

uint16_t i2c_slave_tx_get(uint8_t *buf, uint16_t size);
uint16_t i2c_slave_tx_put(const uint8_t *buf, uint16_t size);
uint16_t i2c_slave_rx_get(uint8_t *buf, uint16_t size);
uint16_t i2c_slave_rx_put(const uint8_t *buf, uint16_t size);
uint16_t i2c_slave_tx_size(void);
uint16_t i2c_slave_rx_size(void);

#ifdef __cplusplus
}
#endif

#endif

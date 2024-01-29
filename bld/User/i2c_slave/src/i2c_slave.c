#include "i2c_slave.h"

#include <stdlib.h>

#include "common.h"
#include "i2c_protocol.h"
#include "i2c_reg_list.h"
#include "main.h"
#include "ring_fifo.h"

typedef enum {
  I2C_STATUS_REG = 0,
  I2C_STATUS_DATA,
} I2C_SLAVE_STATUS;

typedef struct {
  I2C_SLAVE_STATUS status;
  uint16_t reg_addr_size;
  REG_ADDR_TYPE reg_addr;
  RING_FIFO rx_ring;
  RING_FIFO tx_ring;
} i2c_device_t;

static uint8_t __i2c_rx_ring_data[I2C_RX_RING_SIZE];
static uint8_t __i2c_tx_ring_data[I2C_TX_RING_SIZE];

_CCM_DATA static i2c_device_t i2c_dev;

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

void i2c_slave_config(void) {
  LL_I2C_Disable(I2C_TYPE);
  LL_I2C_SetOwnAddress1(I2C_TYPE, I2C_SLAVE_ADDRESS, LL_I2C_OWNADDRESS1_7BIT);
  LL_I2C_EnableOwnAddress1(I2C_TYPE);
  LL_I2C_Enable(I2C_TYPE);

  LL_I2C_EnableIT_ADDR(I2C_TYPE);
  LL_I2C_EnableIT_NACK(I2C_TYPE);
  LL_I2C_EnableIT_ERR(I2C_TYPE);
  LL_I2C_EnableIT_STOP(I2C_TYPE);

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

static void i2c_dev_reset(void) {
  i2c_dev.status = I2C_STATUS_REG;

  disable_global_irq();
  ring_reset(&i2c_dev.rx_ring);
  ring_reset(&i2c_dev.tx_ring);
  enable_global_irq();
}

static void i2c_reception_cb(void) {
  uint8_t data;

  data = LL_I2C_ReceiveData8(I2C_TYPE);
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

static void i2c_transmit_cb(void) {
  uint8_t data;

  if (i2c_slave_tx_get(&data, 1)) {
    LL_I2C_TransmitData8(I2C_TYPE, data);
  } else {
    LL_I2C_TransmitData8(I2C_TYPE, 0xFF);
  }
}

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

void i2c_ev_isr(void) {
  /* Check ADDR flag value in ISR register */
  if (LL_I2C_IsActiveFlag_ADDR(I2C_TYPE)) {
    /* Verify the Address Match with the OWN Slave address */
    if (LL_I2C_GetAddressMatchCode(I2C_TYPE) == I2C_SLAVE_ADDRESS) {
      /* Call function Slave Reset Callback */
      i2c_dev_reset();

      /* Verify the transfer direction, a write direction, Slave enters receiver mode */
      if (LL_I2C_GetTransferDirection(I2C_TYPE) == LL_I2C_DIRECTION_WRITE) {
        /* Clear ADDR flag value in ISR register */
        LL_I2C_ClearFlag_ADDR(I2C_TYPE);

        /* Enable Receive Interrupt */
        LL_I2C_EnableIT_RX(I2C_TYPE);
      } else if (LL_I2C_GetTransferDirection(I2C_TYPE) == LL_I2C_DIRECTION_READ) {
        /* Clear ADDR flag value in ISR register */
        LL_I2C_ClearFlag_ADDR(I2C_TYPE);

        /* Call function Slave Prepare Data Callback */
        i2c_prepare_data();

        /* Enable Transmit Interrupt */
        LL_I2C_EnableIT_TX(I2C_TYPE);
      }
    } else {
      /* Clear ADDR flag value in ISR register */
      LL_I2C_ClearFlag_ADDR(I2C_TYPE);

      /* Call Error function */
      Error_Handler();
    }
  }
  /* Check NACK flag value in ISR register */
  else if (LL_I2C_IsActiveFlag_NACK(I2C_TYPE)) {
    /* End of Transfer */
    LL_I2C_ClearFlag_NACK(I2C_TYPE);
  }
  /* Check RXNE flag value in ISR register */
  else if (LL_I2C_IsActiveFlag_RXNE(I2C_TYPE)) {
    /* Call function Slave Reception Callback */
    i2c_reception_cb();
  }
  /* Check TXIS flag value in ISR register */
  else if (LL_I2C_IsActiveFlag_TXIS(I2C_TYPE)) {
    /* Call function Slave Ready to Transmit Callback */
    i2c_transmit_cb();
  }
  /* Check STOP flag value in ISR register */
  else if (LL_I2C_IsActiveFlag_STOP(I2C_TYPE)) {
    /* End of Transfer */
    LL_I2C_ClearFlag_STOP(I2C_TYPE);

    /* Check TXE flag value in ISR register */
    if (!LL_I2C_IsActiveFlag_TXE(I2C_TYPE)) {
      /* Flush TX buffer */
      LL_I2C_ClearFlag_TXE(I2C_TYPE);
    }

    /* Call function Slave Complete Callback */
    i2c_complete_cb();
  }
  /* Check TXE flag value in ISR register */
  else if (!LL_I2C_IsActiveFlag_TXE(I2C_TYPE)) {
    /* Do nothing */
    /* This Flag will be set by hardware when the TXDR register is empty */
    /* If needed, use LL_I2C_ClearFlag_TXE() interface to flush the TXDR register  */
  } else {
    /* Call Error function */
    Error_Handler();
  }
}

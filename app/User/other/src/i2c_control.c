#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_i2c.h"
#include "common.h"
#include "main.h"
#include "uart_device.h"

static i2c_dev *sw_i2c;
static uint8_t i2c_buf[1280];
static uint16_t i2c_buf_size = 0;

static inline void i2c_master_write_reg(uint16_t DevAddress, uint16_t RegAddr, uint8_t *pData, uint16_t Size) {
  if (!sw_i2c) {
    sw_i2c = i2c_obj_find(I2C0_NAME);
    if (!sw_i2c) {
      return;
    }
  }

  i2c_write_multi_byte_16bit(sw_i2c, DevAddress, RegAddr, pData, Size);
}

static inline void i2c_master_read_reg(uint16_t DevAddress, uint16_t RegAddr, uint8_t *pData, uint16_t Size) {
  if (!sw_i2c) {
    sw_i2c = i2c_obj_find(I2C0_NAME);
    if (!sw_i2c) {
      return;
    }
  }

  i2c_read_multi_byte_16bit(sw_i2c, DevAddress, RegAddr, pData, Size);
}

void uart_frame_i2c_write(frame_parse_t *frame) {
  uint8_t *frame_data;
  uint16_t frame_length;
  uint16_t dev_addr;
  uint16_t reg_addr;

  if (frame->length < sizeof(dev_addr) + sizeof(reg_addr)) {
    return;
  }

  frame_data = frame->data;
  frame_length = frame->length;

  /* 获取从机地址 */
  memcpy(&dev_addr, frame_data, sizeof(dev_addr));
  frame_data += sizeof(dev_addr);
  frame_length -= sizeof(dev_addr);
  if (frame->byte_order) {
    change_byte_order(&dev_addr, sizeof(dev_addr));
  }

  /* 获取寄存器地址 */
  memcpy(&reg_addr, frame_data, sizeof(reg_addr));
  frame_data += sizeof(reg_addr);
  frame_length -= sizeof(reg_addr);
  if (frame->byte_order) {
    change_byte_order(&reg_addr, sizeof(reg_addr));
  }

  /* 发送寄存器地址和寄存器值 */
  memcpy(i2c_buf, frame_data, frame_length);
  i2c_buf_size = frame_length;
  i2c_master_write_reg(dev_addr, reg_addr, i2c_buf, i2c_buf_size);
}

void uart_frame_i2c_read(frame_parse_t *frame) {
  uint8_t *frame_data;
  uint16_t frame_length;
  uint16_t dev_addr;
  uint16_t reg_addr;
  uint16_t data_len;

  if (frame->length < sizeof(dev_addr) + sizeof(reg_addr) + sizeof(data_len)) {
    return;
  }

  frame_data = frame->data;
  frame_length = frame->length;

  /* 获取从机地址 */
  memcpy(&dev_addr, frame_data, sizeof(dev_addr));
  frame_data += sizeof(dev_addr);
  frame_length -= sizeof(dev_addr);
  if (frame->byte_order) {
    change_byte_order(&dev_addr, sizeof(dev_addr));
  }

  /* 获取寄存器地址 */
  memcpy(&reg_addr, frame_data, sizeof(reg_addr));
  frame_data += sizeof(reg_addr);
  frame_length -= sizeof(reg_addr);
  if (frame->byte_order) {
    change_byte_order(&reg_addr, sizeof(reg_addr));
  }

  /* 获取数据长度 */
  memcpy(&data_len, frame_data, sizeof(data_len));
  frame_data += sizeof(data_len);
  frame_length -= sizeof(data_len);
  if (frame->byte_order) {
    change_byte_order(&data_len, sizeof(data_len));
  }

  /* 读取寄存器值 */
  i2c_buf_size = data_len;
  i2c_master_read_reg(dev_addr, reg_addr, i2c_buf, i2c_buf_size);

  uart_puts(frame->dev_type, i2c_buf, i2c_buf_size);
}

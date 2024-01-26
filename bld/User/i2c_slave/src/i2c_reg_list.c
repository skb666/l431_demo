#include "i2c_reg_list.h"

#include "common.h"
#include "i2c_protocol.h"
#include "i2c_slave.h"

static REG_T s_reg_list[] = {
    {REG_VERSION, REG_RO, reg_read_cb_version, NULL},
    {REG_SYSTEM_CTRL, REG_RW, reg_read_cb_system_ctrl, reg_write_cb_system_ctrl},
    {REG_UPDATE_DATA, REG_RW, reg_read_cb_update_status, reg_write_cb_update_data},
    {REG_UPDATE_STATUS, REG_RO, reg_read_cb_update_status, NULL},
};
const static uint32_t s_reg_list_size = ARRAY_SIZE(s_reg_list);

void *reg_list_get(uint32_t *num) {
  *num = s_reg_list_size;
  return s_reg_list;
}

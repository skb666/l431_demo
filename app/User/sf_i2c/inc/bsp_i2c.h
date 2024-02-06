#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__

#include "sf_i2c.h"

#define I2C0_NAME (char*)"I2C0"

#ifdef __cplusplus
extern "C" {
#endif

void i2c_master_init(void);

#ifdef __cplusplus
}
#endif

#endif

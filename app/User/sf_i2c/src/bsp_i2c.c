#include "bsp_i2c.h"

#include "main.h"

/* Private function ----------------------------------------------------------*/
__STATIC_INLINE void i2c0_sda_pin_out_low(void);
__STATIC_INLINE void i2c0_sda_pin_out_high(void);
__STATIC_INLINE void i2c0_scl_pin_out_low(void);
__STATIC_INLINE void i2c0_scl_pin_out_high(void);
__STATIC_INLINE uint8_t i2c0_sda_pin_read_level(void);
__STATIC_INLINE void i2c0_sda_pin_dir_input(void);
__STATIC_INLINE void i2c0_sda_pin_dir_output(void);

/* Private variables ---------------------------------------------------------*/
// 定义iic1驱动对象
static i2c_dev i2c0_dev = {
    .name = I2C0_NAME,
    .speed = 40,
    .port.sda_pin_out_low = i2c0_sda_pin_out_low,
    .port.sda_pin_out_high = i2c0_sda_pin_out_high,
    .port.scl_pin_out_low = i2c0_scl_pin_out_low,
    .port.scl_pin_out_high = i2c0_scl_pin_out_high,
    .port.sda_pin_read_level = i2c0_sda_pin_read_level,
    .port.sda_pin_dir_input = i2c0_sda_pin_dir_input,
    .port.sda_pin_dir_output = i2c0_sda_pin_dir_output,
};

/*! Set i2c sda pin low level */
__STATIC_INLINE void i2c0_sda_pin_out_low(void) {
  LL_GPIO_ResetOutputPin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin);
}

/*! Set i2c sda pin high level */
__STATIC_INLINE void i2c0_sda_pin_out_high(void) {
  LL_GPIO_SetOutputPin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin);
}

/*! Set i2c scl pin low level */
__STATIC_INLINE void i2c0_scl_pin_out_low(void) {
  LL_GPIO_ResetOutputPin(SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin);
}

/*! Set i2c scl pin high level */
__STATIC_INLINE void i2c0_scl_pin_out_high(void) {
  LL_GPIO_SetOutputPin(SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin);
}

/*! Read i2c sda pin level */
__STATIC_INLINE uint8_t i2c0_sda_pin_read_level(void) {
  return LL_GPIO_IsInputPinSet(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin);
}

/*! Switch i2c sda pin dir input */
__STATIC_INLINE void i2c0_sda_pin_dir_input(void) {
  LL_GPIO_SetPinMode(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin, LL_GPIO_MODE_INPUT);
}

/*! Switch i2c sda pin dir output */
__STATIC_INLINE void i2c0_sda_pin_dir_output(void) {
  LL_GPIO_SetPinMode(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin, LL_GPIO_MODE_OUTPUT);
}

/**
 * @brief  Initialization board i2c0 interface
 * @param  none
 * @return none
 */
void i2c_master_init(void) {
  /*! i2c software layer initialization */
  i2c_init(&i2c0_dev);
}

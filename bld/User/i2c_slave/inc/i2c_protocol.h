#ifndef __I2C_PROTOCOL_H__
#define __I2C_PROTOCOL_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* register read callback */
void reg_read_cb_version(void);
void reg_read_cb_system_ctrl(void);
void reg_read_cb_update_status(void);

/* register write callback */
void reg_write_cb_system_ctrl(void);
void reg_write_cb_update_data(void);

#ifdef __cplusplus
}
#endif

#endif

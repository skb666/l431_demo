#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include <stdio.h>

#define _CCM_DATA
#define _RAM_DATA

#define ARRAY_SIZE(array_name) ((int)(sizeof(array_name) / sizeof(array_name[0])))

#ifdef DEBUG
#define printf_dbg(fmt, args...) printf(fmt, ##args)
#else
#define printf_dbg(fmt, args...)
#endif

#ifdef __cplusplus
extern "C" {
#endif

void reset_global_irq(void);
void disable_global_irq(void);
void enable_global_irq(void);

void change_byte_order(void *ptr, size_t size);

#ifdef __cplusplus
}
#endif

#endif

#include "common.h"

#include "main.h"

static uint32_t s_irq_count = 0;

void reset_global_irq(void) {
  s_irq_count = 0;
}

void disable_global_irq(void) {
  __disable_irq();
  s_irq_count++;
  return;
}

void enable_global_irq(void) {
  if (s_irq_count != 0) {
    s_irq_count--;
  }

  if (0 == s_irq_count) {
    __enable_irq();
  }
  return;
}

void change_byte_order(void *ptr, size_t size) {
  uint8_t tmp;
  uint8_t *addr = (uint8_t *)ptr;
  size_t i, imax = size / 2;

  for (i = 0; i < imax; ++i) {
    tmp = addr[i];
    addr[i] = addr[size - i - 1];
    addr[size - i - 1] = tmp;
  }
}

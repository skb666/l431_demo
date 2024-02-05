#ifndef __COMBO_KEY_H__
#define __COMBO_KEY_H__

#include <stdint.h>

#define KEY_NUM_MAX 1

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  K_RELEASE,
  K_PRESS,
} KEY_VALUE;

typedef enum {
  KS_NONE = 0,
  KS_RELEASE,
  KS_SHAKE,
  KS_PRESS,
} KEY_STATUS;

typedef enum {
  KE_NONE = 0,
  KE_ERROR,
  KE_PRESS,
  KE_RELEASE,
  KE_LONG_PRESS,
  KE_LONG_RELEASE,
  KE_COMBO,
  KE_COMBO_RELEASE,
} KEY_EVENT;

typedef struct {
  KEY_STATUS status;      // 内部按键状态
  KEY_EVENT event;        // 按键事件
  uint16_t id;            // 按键 ID
  uint16_t valid;         // 电平有效时间
  uint16_t ageing;        // 老化时间
  uint16_t long_press;    // 长按判断时间
  uint16_t press_cnt;     // 内部按下计数
  uint16_t release_cnt;   // 内部释放计数
  uint16_t press_time;    // 按键按下次数
  uint16_t release_time;  // 按键松开次数
  KEY_VALUE (*get)(void); // 按键状态获取回调
  void *custom_data;      // 按键额外附带信息
} KEY;

KEY *key_list_get(int *num);
KEY *key_find_by_id(int id);
KEY_EVENT key_event_get(KEY *key);
int key_combo_press_count(KEY *key);
int key_combo_release_count(KEY *key);

int8_t key_register(uint16_t id, KEY_VALUE (*get)(void), void *custom_data, uint16_t valid, uint16_t ageing, uint16_t long_press);
KEY_EVENT combo_key_event_check(KEY *key);

#ifdef __cplusplus
}
#endif

#endif

#include "combo_key.h"

#include <stdio.h>

typedef struct {
  KEY key[KEY_NUM_MAX];
  int num;
} KEY_LIST;

static KEY_LIST key_list;

KEY *key_list_get(int *num) {
  *num = key_list.num;
  return key_list.key;
}

KEY *key_find_by_id(int id) {
  KEY *key = NULL;

  for (int i = 0; i < key_list.num; ++i) {
    if (id == key_list.key[i].id) {
      key = &key_list.key[i];
      break;
    }
  }

  return key;
}

KEY_EVENT key_event_get(KEY *key) {
  if (key == NULL) {
    return KE_ERROR;
  }
  return key->event;
}

int key_combo_press_count(KEY *key) {
  if (key == NULL) {
    return -1;
  }
  return key->press_time;
}

int key_combo_release_count(KEY *key) {
  if (key == NULL) {
    return -1;
  }
  return key->release_time;
}

int8_t key_register(uint16_t id, KEY_VALUE (*get)(void), void *custom_data, uint16_t valid, uint16_t ageing, uint16_t long_press) {
  KEY *key = NULL;

  key = key_find_by_id(id);
  if (key != NULL) {
    if (key->press_cnt >= key->long_press) {
      key->press_cnt = long_press;
    }
    key->enable = 1;
    key->valid = valid;
    key->ageing = ageing;
    key->long_press = long_press;
    key->get = get;
    key->custom_data = custom_data;
  } else {
    if (key_list.num >= KEY_NUM_MAX) {
      return -1;
    }

    key_list.key[key_list.num].status = KS_NONE;
    key_list.key[key_list.num].event = KE_NONE;
    key_list.key[key_list.num].id = id;
    key_list.key[key_list.num].enable = 1;
    key_list.key[key_list.num].valid = valid;
    key_list.key[key_list.num].ageing = ageing;
    key_list.key[key_list.num].long_press = long_press;
    key_list.key[key_list.num].press_cnt = 0;
    key_list.key[key_list.num].release_cnt = 0;
    key_list.key[key_list.num].press_time = 0;
    key_list.key[key_list.num].release_time = 0;
    key_list.key[key_list.num].get = get;
    key_list.key[key_list.num].custom_data = custom_data;
    key_list.num += 1;
  }

  return 0;
}

int8_t key_unregister(uint16_t id) {
  KEY *key = NULL;

  key = key_find_by_id(id);
  if (key == NULL) {
    return -1;
  }

  key->enable = 0;

  return 0;
}

KEY_EVENT combo_key_event_check(KEY *key) {
  if (key == NULL) {
    return KE_ERROR;
  }

  if (!key->enable) {
    return KE_NONE;
  }

  key->event = KE_NONE;
  switch (key->status) {
    case KS_NONE: {
      if (key->get() == K_PRESS) {
        key->press_cnt = 0;
        key->release_cnt = 0;
        key->press_time = 0;
        key->release_time = 0;
        key->status = KS_SHAKE;
      }
    } break;
    case KS_SHAKE: {
      if (key->get() == K_PRESS) {
        key->press_cnt += 1;
        if (key->press_cnt > key->valid) {
          key->press_time += 1;
          key->status = KS_PRESS;
          if (key->press_time == 1) {
            key->event = KE_PRESS;
          } else if (key->press_time > 1) {
            key->event = KE_COMBO;
          }
        }
      } else {
        if (key->release_time == 1) {
          key->event = KE_RELEASE;
        } else if (key->release_time > 1) {
          key->event = KE_COMBO_RELEASE;
        }
        key->status = KS_NONE;
      }
    } break;
    case KS_PRESS: {
      if (key->get() == K_PRESS) {
        key->press_cnt += 1;
        if (key->press_cnt == key->long_press) {
          key->event = KE_LONG_PRESS;
        }
      } else {
        key->release_time += 1;
        key->status = KS_RELEASE;
      }
    } break;
    case KS_RELEASE: {
      if (key->press_cnt >= key->long_press) {
        key->event = KE_LONG_RELEASE;
        key->status = KS_NONE;
        break;
      }
      if (key->get() == K_PRESS) {
        key->press_cnt = 0;
        key->release_cnt = 0;
        key->status = KS_SHAKE;
      } else {
        key->release_cnt += 1;
        if (key->release_cnt >= key->ageing) {
          if (key->release_time == 1) {
            key->event = KE_RELEASE;
          } else {
            key->event = KE_COMBO_RELEASE;
          }
          key->status = KS_NONE;
        }
      }
    } break;
  }

  return key->event;
}

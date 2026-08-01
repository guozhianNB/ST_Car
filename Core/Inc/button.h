#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  BUTTON_LEVEL_UP = 0,
  BUTTON_LEVEL_DOWN,
  BUTTON_COUNT
} ButtonId;

typedef enum {
  BUTTON_EVENT_NONE = 0,
  BUTTON_EVENT_SHORT,
  BUTTON_EVENT_LONG
} ButtonEvent;

void Button_Init(uint32_t now_ms);
void Button_Update(uint32_t now_ms);
ButtonEvent Button_TakeEvent(ButtonId id);
bool Button_IsRawPressed(ButtonId id);
bool Button_IsPressed(ButtonId id);
uint32_t Button_HeldMs(ButtonId id, uint32_t now_ms);
void Button_DiscardEvents(void);

#endif /* BUTTON_H */

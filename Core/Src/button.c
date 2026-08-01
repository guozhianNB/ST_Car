#include "button.h"
#include "app_config.h"
#include "main.h"
#include <stdbool.h>

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  bool raw_pressed;
  bool stable_pressed;
  bool armed;
  bool long_reported;
  uint32_t raw_change_ms;
  uint32_t press_start_ms;
  ButtonEvent event;
} ButtonState;

static ButtonState buttons[BUTTON_COUNT];

static bool ReadPressed(const ButtonState *button)
{
  return HAL_GPIO_ReadPin(button->port, button->pin) == GPIO_PIN_RESET;
}

void Button_Init(uint32_t now_ms)
{
  buttons[BUTTON_LEVEL_UP].port = LEVEL_UP_BUTTON_GPIO_Port;
  buttons[BUTTON_LEVEL_UP].pin = LEVEL_UP_BUTTON_Pin;
  buttons[BUTTON_LEVEL_DOWN].port = LEVEL_DOWN_BUTTON_GPIO_Port;
  buttons[BUTTON_LEVEL_DOWN].pin = LEVEL_DOWN_BUTTON_Pin;

  for (unsigned i = 0U; i < BUTTON_COUNT; ++i) {
    bool pressed = ReadPressed(&buttons[i]);
    buttons[i].raw_pressed = pressed;
    buttons[i].stable_pressed = pressed;
    buttons[i].armed = !pressed;
    buttons[i].long_reported = false;
    buttons[i].raw_change_ms = now_ms;
    buttons[i].press_start_ms = now_ms;
    buttons[i].event = BUTTON_EVENT_NONE;
  }
}

void Button_Update(uint32_t now_ms)
{
  for (unsigned i = 0U; i < BUTTON_COUNT; ++i) {
    ButtonState *button = &buttons[i];
    bool pressed = ReadPressed(button);
    if (pressed != button->raw_pressed) {
      button->raw_pressed = pressed;
      button->raw_change_ms = now_ms;
    }
    if ((pressed != button->stable_pressed) &&
        ((now_ms - button->raw_change_ms) >= APP_BUTTON_DEBOUNCE_MS)) {
      button->stable_pressed = pressed;
      if (pressed) {
        button->press_start_ms = now_ms;
        button->long_reported = false;
      } else {
        if (button->armed && !button->long_reported &&
            (button->event == BUTTON_EVENT_NONE)) {
          button->event = BUTTON_EVENT_SHORT;
        }
        button->armed = true;
      }
    }
    if (button->stable_pressed && button->armed &&
        !button->long_reported &&
        ((now_ms - button->press_start_ms) >= APP_BUTTON_LONG_PRESS_MS)) {
      if (button->event == BUTTON_EVENT_NONE) {
        button->event = BUTTON_EVENT_LONG;
      }
      button->long_reported = true;
    }
  }
}

ButtonEvent Button_TakeEvent(ButtonId id)
{
  ButtonEvent event;
  if ((unsigned)id >= BUTTON_COUNT) return BUTTON_EVENT_NONE;
  event = buttons[id].event;
  buttons[id].event = BUTTON_EVENT_NONE;
  return event;
}

bool Button_IsRawPressed(ButtonId id)
{
  if ((unsigned)id >= BUTTON_COUNT) return false;
  return buttons[id].raw_pressed;
}

bool Button_IsPressed(ButtonId id)
{
  if ((unsigned)id >= BUTTON_COUNT) return false;
  return buttons[id].stable_pressed;
}

uint32_t Button_HeldMs(ButtonId id, uint32_t now_ms)
{
  if (((unsigned)id >= BUTTON_COUNT) || !buttons[id].stable_pressed) return 0U;
  return now_ms - buttons[id].press_start_ms;
}

void Button_DiscardEvents(void)
{
  for (unsigned i = 0U; i < BUTTON_COUNT; ++i) {
    buttons[i].event = BUTTON_EVENT_NONE;
  }
}

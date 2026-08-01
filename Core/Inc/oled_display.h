#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

void OledDisplay_Init(uint32_t now_ms);
void OledDisplay_Service(uint32_t now_ms);
bool OledDisplay_IsPresent(void);
uint8_t OledDisplay_GetAddress(void);

#endif /* OLED_DISPLAY_H */

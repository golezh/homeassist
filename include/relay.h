#ifndef RELAY_H
#define RELAY_H

#include <stdint.h>

// 1 = relay ON by output HIGH
// 0 = relay ON by output LOW
#define RELAY_ACTIVE_HIGH 0

void relay_init(void);
void relay_on(uint8_t index);
void relay_off(uint8_t index);
void relay_all_off(void);
void relay_only(uint8_t index);

uint8_t relay_count(void);

#endif // RELAY_H
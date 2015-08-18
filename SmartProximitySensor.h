#pragma once

extern uint16_t brightnessLeft;
extern uint16_t brightnessRight;
extern uint8_t okLeft;
extern uint8_t okRight;
extern bool objectSeen;

extern Zumo32U4LCD lcd;

void senseInit();
void senseReset();
void sense();
void senseTest();

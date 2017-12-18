#ifndef MBED_SHIELD_H_
#define MBED_SHIELD_H_

#include "avr_core.h"

#define RGB_BLUE    GPIOB,0
#define RGB_GREEN   GPIOB,1
#define RGB_RED     GPIOD,5

#define JOY_UP      GPIOA,2
#define JOY_DOWN    GPIOA,3
#define JOY_LEFT    GPIOA,4
#define JOY_RIGHT   GPIOA,5
#define JOY_PUSH    GPIOD,4


uint8_t readJoystick();

#endif /* MBED_SHIELD_H_ */

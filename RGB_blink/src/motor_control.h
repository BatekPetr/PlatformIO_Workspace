#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <avr_core.h>

void Init_Tim0_PWM_2CH();
void Set_Tim0_PWM_2CH(int dutyCh1, int dutyCh2);

#endif //MOTOR_CONTROL_H

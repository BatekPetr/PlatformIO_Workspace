#ifndef AVR_CORE_H
#define AVR_CORE_H

#include <avr/io.h>     // Includes all the definition of register port etc
#include <avr/interrupt.h>
#include <stdbool.h>

#ifndef F_CPU
  #define F_CPU 16000000UL      //Need to include it before <util/delay.h>
#endif                                       //Change 16000000 with your crystal freq. In my case its 16 MHz

typedef enum {
  GPIOA,
  GPIOB,
  GPIOC,
  GPIOD
} ePorts;

typedef enum {
  ioPortOutput,     // output
  ioPortInputPU,    // input pull-up
  ioPortInputTS     // input Tri-State
} eIoPortModes;

typedef enum {
  Tim0,
  Tim1,
  Tim2
} eTimers;


bool Avr_SetPinGPIO(ePorts port, int bitnum, eIoPortModes mode);
void GPIOToggle(ePorts port, int bitnum);
bool GPIORead(ePorts port, int bitnum);
void GPIOWrite(ePorts port, int bitnum, bool state);

bool Avr_SetTimer(eTimers tim, float sec);
bool Avr_InitPWM(eTimers tim, float period, float duty );
bool Avr_SetPWM(eTimers tim, float duty );


#endif //AVR_CORE_H

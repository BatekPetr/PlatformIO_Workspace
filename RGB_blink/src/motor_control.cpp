#include"motor_control.h"

/*
  Initialize 8-bit counter for PWM generation
  Frequency of PWM is fixed at: 31 250 Hz
*/
void Init_Tim0_PWM_2CH()
{
  // initialize timer0 in Phase Correct PWM (mode 1)
  // with TOP counter value 255
  TCCR0A = 0;
  TCCR0B &= ~(1<<WGM02);
  TCCR0A |= (1<<WGM00);

  // Set Compare Output Modes for OC0A and OC0B PWM registers (pins)
  TCCR0A |= (1<<COM0A1); // non-inverting mode
  TCCR0A |= (1<<COM0B1); // non-inverting mode

  // Set inital duty cycle to 0
  OCR0A = 0;
  OCR0B = 0;

  // Set prescaler to 256 -> 16Mhz/256 = 62.5kHz
  // In Phase correct mode (counter counts up and than down)
  // It results in PWM frequency of 31 250 Hz
  // Non-zero value in TCCR0B_CS[] also starts the Timer 0
  TCCR0B = 0;
  TCCR0B |=  (1 << CS02);
}

/*
  Sets duty cycle value into the Output Compare Pins of Timer
  For Tim0 in 2Channel mode values are in range <0,255>
*/
void Set_Tim0_PWM_2CH(int dutyCh1, int dutyCh2)
{
  OCR0A = dutyCh1;
  OCR0B = dutyCh2;
}

#include <Arduino.h>
#include <avr_core.h>     // Includes all the definition of register port etc
#include <util/delay.h>    //includes delay functions delay_ms and delay_us
#include "mbed_shield.h"


volatile int sec0, sec1, sec2 = 0;

//uncomment this line if using a Common Anode LED
//#define COMMON_ANODE


ISR (TIMER0_COMPA_vect)  // timer0 output compare interrupt
{
    sec0++;
    TIFR0 |= (1<<OCF0A);  // clear interrupt flag
}


ISR (TIMER1_COMPA_vect)
{
  //secA++;
  sec1++;

  TIFR1 |= (1 << OCF1A);    // clear interrupt flag
}

/*
ISR (TIMER2_COMPA_vect)
{
  sec2++;
  if (sec2%1000 == 0)
  {
    sec2 = 0;
    GPIOToggle(RGB_RED);
  }

}
*/


void setup()
{
  Avr_SetPinGPIO(RGB_RED, ioPortOutput);
  Avr_SetPinGPIO(RGB_GREEN, ioPortOutput);
  Avr_SetPinGPIO(RGB_BLUE, ioPortOutput);

  // Joystick Initialization
  Avr_SetPinGPIO(JOY_UP, ioPortInputTS);
  Avr_SetPinGPIO(JOY_DOWN, ioPortInputTS);
  Avr_SetPinGPIO(JOY_LEFT, ioPortInputTS);
  Avr_SetPinGPIO(JOY_RIGHT, ioPortInputTS);
  Avr_SetPinGPIO(JOY_PUSH, ioPortInputTS);


  GPIOWrite(RGB_RED, 1);
  GPIOWrite(RGB_GREEN, 1);
  GPIOWrite(RGB_BLUE, 1);

  //Avr_SetTimer(Tim0, 0.001);
  //Avr_SetTimer(Tim1, 0.001);

  //Avr_SetPWM(Tim0,254);
  //Avr_SetPWM(Tim1,254);

  Serial.begin(9600);
}

void loop()
{
  uint8_t joy_combination = readJoystick();

    switch(joy_combination)
    {
      case 0b1:
        GPIOWrite(RGB_RED, 0);
        break;
      case 0b10:
        GPIOWrite(RGB_BLUE, 0);
        break;
      case 0b100:
        GPIOWrite(RGB_GREEN, 0);
        break;
      case 0b1000:
        GPIOWrite(RGB_GREEN, 0);
        GPIOWrite(RGB_RED, 0);
        break;
      case 0b10000:
        GPIOWrite(RGB_BLUE, 0);
        GPIOWrite(RGB_RED, 0);
        break;
      default:
        GPIOWrite(RGB_RED, 1);
        GPIOWrite(RGB_GREEN, 1);
        GPIOWrite(RGB_BLUE, 1);
        break;
    }
/*
  if (sec0%1000 == 1)
  {
    GPIOToggle(RGB_GREEN);
    //Serial.println(TCNT0);
  }
*/
  /*
  setColor(0, 1, 0);  // green
  _delay_ms(1000);
  setColor(0, 0, 1);  // blue
  _delay_ms(1000);
  setColor(1, 1, 0);  // yellow
  _delay_ms(1000);
  setColor(0, 1, 1);  // aqua
  _delay_ms(1000);
  setColor(1, 1, 1);  // white
  _delay_ms(1000);
  setColor(0, 0, 0);  // off
  _delay_ms(1000);
 */
}

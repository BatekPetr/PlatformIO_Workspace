#include <Arduino.h>
#include <avr_core.h>     // Includes all the definition of register port etc
#include <util/delay.h>    //includes delay functions delay_ms and delay_us
#include "mbed_shield.h"
#include "motor_control.h"
//#include <TimerOne.h>
//#include <avr/interrupt.h>


volatile int sec0, sec1, sec2 = 0;

//uncomment this line if using a Common Anode LED
//#define COMMON_ANODE

/*
ISR (TIMER0_COMPA_vect)  // timer0 output compare interrupt
{
  sec0++;
  TIFR0 |= (1<<OCF0A);  // clear interrupt flag

  if (sec0%1000 == 0)
  {
    sec0 = 0;
    GPIOToggle(RGB_BLUE);
    //Serial.println(TCNT0);
  }
}

ISR (TIMER1_COMPA_vect)
{
  GPIOToggle(RGB_BLUE);
  TIFR1 |= (1 << OCF1A);    // clear interrupt flag
}


ISR (TIMER1_CAPT_vect)
{
  GPIOToggle(RGB_BLUE);
  TIFR1 |= (1 << ICF1);    // clear interrupt flag
}

// TIMER1 overflow interrupt service routine
// called whenever TCNT1 overflows

ISR(TIMER1_OVF_vect)
{
    GPIOToggle(RGB_BLUE);
}
*/
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
  //Avr_SetTimer(Tim1, 1);

  //Timer1.initialize(1000000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  //Timer1.attachInterrupt( timerIsr ); // attach the service routine here

  //Avr_InitPWM(Tim0,0.001, 0);
  //Avr_SetPWM(Tim0, 0);
  //Avr_InitPWM(Tim1,0.001, 0.1);

  Init_Tim0_PWM_2CH();
  Set_Tim0_PWM_2CH(0, 255);
  //Serial.begin(9600);
}
float duty = 0.1;
int dir = 1;
void loop()
{
  /*
  if (dir)
  {
    duty = duty + 0.02;
  }
  else
  {
    duty = duty - 0.02;
  }

  Set_Tim0_PWM_2CH(0, duty);
  Avr_SetPWM(Tim1, duty);

  if (duty >= 0.3)
    dir = 0;
  else if (duty <=0)
    dir = 1;

  _delay_ms(500);
*/
  /*
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

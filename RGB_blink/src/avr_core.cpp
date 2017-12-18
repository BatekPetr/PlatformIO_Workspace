#include <avr_core.h>

// GPIO functions

bool Avr_SetPinGPIO(ePorts port, int bitnum, eIoPortModes mode)
{
  switch( (int)mode)
  {
    case (int)ioPortOutput:
      switch( port)
      {
        case GPIOA:
          //DDRA |= 1 << bitnum;        // set pin mode
          //PORTA &= (~(1 << bitnum));  // set output of port to 0
          break;
        case GPIOB:
          DDRB |= 1 << bitnum;        // set pin mode
          PORTB &= (~(1 << bitnum));  // set output of port to 0
          break;
        case GPIOC:
          DDRC |= 1 << bitnum;        // set pin mode
          PORTC &= (~(1 << bitnum));  // set output of port to 0
          break;
        case GPIOD:
          DDRD |= 1 << bitnum;        // set pin mode
          PORTD &= (~(1 << bitnum));  // set output of port to 0
          break;
        default:
          return false;               // Unknown port
      }
      break;
    case (int)ioPortInputPU:
      switch( port)
      {
        case GPIOB:
          DDRB &= ~(1 << bitnum);        // set pin mode to Input
          PORTB |= 1 << bitnum;       // enable pull up
          break;
        case GPIOC:
          DDRC &= ~(1 << bitnum);        // set pin mode to Input
          PORTC |= 1 << bitnum;       // enable pull up
          break;
        case GPIOD:
          DDRD &= ~(1 << bitnum);        // set pin mode to Input
          PORTD |= 1 << bitnum;       // enable pull up
          break;
        default:
          return false;               // Unknown port
      }
      break;
    case (int)ioPortInputTS:
      switch( port)
      {
        case GPIOB:
          DDRB &= ~(1 << bitnum);        // set pin mode to Input
          PORTB &= ~(1 << bitnum);       // make input TriStated
          break;
        case GPIOC:
          DDRC &= ~(1 << bitnum);        // set pin mode to Input
          PORTC &= ~(1 << bitnum);       // make input TriStated
          break;
        case GPIOD:
          DDRD &= ~(1 << bitnum);        // set pin mode to Input
          PORTD &= ~(1 << bitnum);       // make input TriStated
          break;
        default:
          return false;               // Unknown port
      }
      break;
    default:
      return false;                    // Unknown mode
  }
}

void GPIOToggle(ePorts port, int bitnum)
{
  switch( port)
  {
    case GPIOB:
      PORTB ^= (1 << bitnum);
      break;
    case GPIOC:
      PORTC ^= (1 << bitnum);
      break;
    case GPIOD:
      PORTD ^= (1 << bitnum);
      break;
  }
}

bool GPIORead(ePorts port, int bitnum)
{
  switch( port)
  {
    case GPIOA:
      //return (PINA & (1 << bitnum)) != 0;
      return 0;
      break;
    case GPIOB:
      return (PINB & (1 << bitnum)) != 0;
      break;
    case GPIOC:
      return (PINC & (1 << bitnum)) != 0;
      break;
    case GPIOD:
      return (PIND & (1 << bitnum)) != 0;
      break;
  }
}

void GPIOWrite(ePorts port, int bitnum, bool state)
{
  if(state)
  {
    switch( port)
    {
      case GPIOB:
        PORTB |= (1 << bitnum);
        break;
      case GPIOC:
        PORTC |= (1 << bitnum);
        break;
      case GPIOD:
        PORTD |= (1 << bitnum);
        break;
    }
  }
  else
  {
    switch( port)
    {
      case GPIOB:
        PORTB &= ~(1 << bitnum);
        break;
      case GPIOC:
        PORTC &= ~(1 << bitnum);
        break;
      case GPIOD:
        PORTD &= ~(1 << bitnum);
        break;
    }
  }

}

bool Avr_SetTimer(eTimers tim, float sec)
{
  int presc;
  switch(tim)
  {
    case(Tim0):   // 8bit timer
      TCCR0A |= (1 << WGM01); // Set the Timer Mode to CTC (Clear Timer on Compare)

      presc = 64;          // prescaler needs to be changed accorging to value in CS02 to CS00 registers
      OCR0A = ( (F_CPU/presc) * sec ) - 1; // Set the value that you want to count to

      TIMSK0 |= (1 << OCIE0A);    //Set the ISR COMPA vect

      TCCR0B |= (1 << CS01) | (1 << CS00);  // set prescaler to 64 and start the timer
      break;

    case(Tim1):   // (supposed to be 16bit timer) seems like it is 8 bit as well
      TCCR1B |= (1 << WGM12);  // Mode 4, CTC on OCR1A

      presc = 64;
      OCR1A = ( (F_CPU*sec/presc) ) - 1; // Set the value that you want to count to

      TIMSK1 |= (1 << OCIE1A);   //Set interrupt on compare match

      TCCR1B = (1 << CS11) | (1 << CS10); // set prescaler to 64 and start the timer
      break;

    case(Tim2):   // 8bit timer
      TCCR2A |= (1 << WGM21); // Set the Timer Mode to CTC (Clear Timer on Compare)

      presc = 64;          // prescaler needs to be changed accorging to value in CS02 to CS00 registers
      OCR2A = ( (F_CPU/presc) * sec ) - 1; // Set the value that you want to count to

      TIMSK2 |= (1 << OCIE2A);    //Set the ISR COMPA vect

      TCCR2B |= (1 << CS22);  // set prescaler to 64 and start the timer
      break;

    default:
      return false;
      break;
  }

  sei();         //enable interrupts
  return true;
}

bool Avr_SetPWM(eTimers tim, float duty )
{
  switch(tim)
  {
    case(Tim0):   // 8bit timer
      // initialize timer0 in Fast PWM mode
      TCCR0A |= (1<<WGM01)|(1<<WGM00);

      TCCR0A |= (1<<COM0B1); // NON-inverting mode
      TCCR0B |=  (1 << CS00);   //Start Timer0, with No prescaling

      OCR0B = duty;
      break;

    case(Tim1):
      // initialize timer1 in Fast PWM mode, with TOP in ICR1
      TCCR1A |= (1<<WGM10);
      TCCR1B |= (1<<WGM12);

      TCCR1A |= (1<<COM1A1); // NON-inverting mode

      ICR1 = 255;
      OCR1A = duty;

      TCCR1B |=  (1 << CS10);   //Start Timer0, with NO prescaling

      break;
    }

    return true;
}
/*
void Duty( uint8_t percentage, uint16_t ICR1_value)
{
	//percentage =  (percentage > 100 ? 100 : (percentage < 0 ? 0 : percentage));
	uint16_t OCR = percentage; //(uint16_t)(((uint32_t)percentage * (uint32_t)ICR1_value)/100) ;    // Set pwm percent of pwm period

	OCR1AH = OCR >> 8;
	OCR1AL = OCR & 0xFF;
}

void FrequencyPWM(uint8_t frequency, uint8_t percentage)
{
  uint8_t divider = 1;
	uint16_t TOP = F_CPU/(divider*frequency) - 1;
	ICR1H = 0;
	ICR1L = 0xFF;
	Duty(percentage, TOP);
}
*/

/*
 ESC.cpp - Interrupt driven Electronic Speed Controller library for Arduino using 16 bit timers
 Based on the Servo library written by Michael Margolis
 Copyright (c) 2016 James Jackson.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/* 
  
  An ESC is activated by creating an instance of the ESC class passing the desired pin to the attach() method.
  The ESCs are pulsed in the background using the value most recently written using the write() method

  Note that analogWrite of PWM on pins associated with the timer are disabled when the first ESC is attached.
  Timers are seized as needed in groups of 12 ESCs - 24 ESCs use two timers, 48 ESCs will use four.
  The sequence used to sieze timers is defined in timers.h

  The methods are:

   ESC - Class for manipulating electronic speed controllers for brushless DC motors connected to Arduino pins.

   attach(pin )  - Attaches an ESC to an i/o pin.
   attach(pin, min, max, freq) - Attaches to a pin setting min and max values in microseconds, and the update rate in Hz
   default min is 1000, max is 2000  
 
   write()     - Sets the normalized ESC throttle (between 0.0 and 1.0).
   writeMicroseconds() - Sets the pulse width in microseconds 
   read()      - Gets the last written normalized ESC throttle. 
   arm()       - Prevents Motors from not spinning by sending a higher default PWM
   disarm()    - Prvenets Motors from spinning by preventing higher PWM until armed
   readMicroseconds()   - Gets the last written ESC pulse width in microseconds.
   attached()  - Returns true if there is an ESC attached. 
   detach()    - Stops an attached ESC from pulsing its i/o pin. 
 */

#include <avr/interrupt.h>
#include <Arduino.h> 

#include "ESC.h"

#define usToTicks(_us)    (( clockCyclesPerMicrosecond()* _us) / 8)     // converts microseconds to tick (assumes prescale of 8)  // 12 Aug 2009
#define ticksToUs(_ticks) (( (unsigned)_ticks * 8)/ clockCyclesPerMicrosecond() ) // converts from ticks back to microseconds


#define TRIM_DURATION       2                               // compensation ticks to trim adjust for digitalWrite delays // 12 August 2009

//#define NBR_TIMERS        (MAX_ESCs / ESCS_PER_TIMER)

static ESC_t ESC_array[MAX_ESCs];                          // static array of servo structures
static volatile int8_t Channel[_Nbr_16timers ];             // counter for the servo being pulsed for each timer (or -1 if refresh interval)

uint8_t ServoCount = 0;                                     // the total number of attached ESC_array


// convenience macros
#define ESC_INDEX_TO_TIMER(_ESC_nb) ((timer16_Sequence_t)(_ESC_nb / ESCS_PER_TIMER)) // returns the timer controlling this servo
#define ESC_INDEX_TO_CHANNEL(_ESC_nb) (_ESC_nb % ESCS_PER_TIMER)       // returns the index of the servo on this timer
#define ESC_INDEX(_timer,_channel)  ((_timer*ESCS_PER_TIMER) + _channel)     // macro to access servo index by timer and channel
#define SERVO(_timer,_channel)  (ESC_array[ESC_INDEX(_timer,_channel)])            // macro to access servo class by timer and channel

// #define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)  // minimum value in uS for this servo
// #define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)  // maximum value in uS for this servo 

/************ static functions common to all instances ***********************/

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn, volatile uint16_t* OCRnA)
{
  if( Channel[timer] < 0 )
    *TCNTn = 0; // channel set to -1 indicated that refresh interval completed so reset the timer 
  else{
    if( ESC_INDEX(timer,Channel[timer]) < ServoCount && SERVO(timer,Channel[timer]).Pin.isActive == true )  
      digitalWrite( SERVO(timer,Channel[timer]).Pin.nbr,LOW); // pulse this channel low if activated   
  }

  Channel[timer]++;    // increment to the next channel
  if( ESC_INDEX(timer,Channel[timer]) < ServoCount && Channel[timer] < ESCS_PER_TIMER) {
    *OCRnA = *TCNTn + SERVO(timer,Channel[timer]).ticks;
    if(SERVO(timer,Channel[timer]).Pin.isActive == true)     // check if activated
      digitalWrite( SERVO(timer,Channel[timer]).Pin.nbr,HIGH); // its an active channel so pulse it high   
  }  
  else { 
    // finished all channels so wait for the refresh period to expire before starting over 
    if( ((unsigned)*TCNTn) + 4 < usToTicks(REFRESH_INTERVAL) )  // allow a few ticks to ensure the next OCR1A not missed
      *OCRnA = (unsigned int)usToTicks(REFRESH_INTERVAL);  
    else 
      *OCRnA = *TCNTn + 4;  // at least REFRESH_INTERVAL has elapsed
    Channel[timer] = -1; // this will get incremented at the end of the refresh period to start again at the first channel
  }
}

#ifndef WIRING // Wiring pre-defines signal handlers so don't define any if compiling for the Wiring platform
// Interrupt handlers for Arduino 
#if defined(_useTimer1)
ISR(TIMER1_COMPA_vect) 
{ 
  handle_interrupts(_timer1, &TCNT1, &OCR1A); 
}
#endif

#if defined(_useTimer3)
ISR(TIMER3_COMPA_vect) 
{ 
  handle_interrupts(_timer3, &TCNT3, &OCR3A); 
}
#endif

#if defined(_useTimer4)
ISR(TIMER4_COMPA_vect) 
{
  handle_interrupts(_timer4, &TCNT4, &OCR4A); 
}
#endif

#if defined(_useTimer5)
ISR(TIMER5_COMPA_vect) 
{
  handle_interrupts(_timer5, &TCNT5, &OCR5A); 
}
#endif

#elif defined WIRING
// Interrupt handlers for Wiring 
#if defined(_useTimer1)
void Timer1Service() 
{ 
  handle_interrupts(_timer1, &TCNT1, &OCR1A); 
}
#endif
#if defined(_useTimer3)
void Timer3Service() 
{ 
  handle_interrupts(_timer3, &TCNT3, &OCR3A); 
}
#endif
#endif


static void initISR(timer16_Sequence_t timer)
{  
#if defined (_useTimer1)
  if(timer == _timer1) {
    TCCR1A = 0;             // normal counting mode 
    TCCR1B = _BV(CS11);     // set prescaler of 8 
    TCNT1 = 0;              // clear the timer count 
#if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
    TIFR |= _BV(OCF1A);      // clear any pending interrupts; 
    TIMSK |=  _BV(OCIE1A) ;  // enable the output compare interrupt  
#else
    // here if not ATmega8 or ATmega128
    TIFR1 |= _BV(OCF1A);     // clear any pending interrupts; 
    TIMSK1 |=  _BV(OCIE1A) ; // enable the output compare interrupt 
#endif    
#if defined(WIRING)       
    timerAttach(TIMER1OUTCOMPAREA_INT, Timer1Service); 
#endif	
  } 
#endif  

#if defined (_useTimer3)
  if(timer == _timer3) {
    TCCR3A = 0;             // normal counting mode 
    TCCR3B = _BV(CS31);     // set prescaler of 8  
    TCNT3 = 0;              // clear the timer count 
#if defined(__AVR_ATmega128__)
    TIFR |= _BV(OCF3A);     // clear any pending interrupts;   
	ETIMSK |= _BV(OCIE3A);  // enable the output compare interrupt     
#else  
    TIFR3 = _BV(OCF3A);     // clear any pending interrupts; 
    TIMSK3 =  _BV(OCIE3A) ; // enable the output compare interrupt      
#endif
#if defined(WIRING)    
    timerAttach(TIMER3OUTCOMPAREA_INT, Timer3Service);  // for Wiring platform only	
#endif  
  }
#endif

#if defined (_useTimer4)
  if(timer == _timer4) {
    TCCR4A = 0;             // normal counting mode 
    TCCR4B = _BV(CS41);     // set prescaler of 8  
    TCNT4 = 0;              // clear the timer count 
    TIFR4 = _BV(OCF4A);     // clear any pending interrupts; 
    TIMSK4 =  _BV(OCIE4A) ; // enable the output compare interrupt
  }    
#endif

#if defined (_useTimer5)
  if(timer == _timer5) {
    TCCR5A = 0;             // normal counting mode 
    TCCR5B = _BV(CS51);     // set prescaler of 8  
    TCNT5 = 0;              // clear the timer count 
    TIFR5 = _BV(OCF5A);     // clear any pending interrupts; 
    TIMSK5 =  _BV(OCIE5A) ; // enable the output compare interrupt      
  }
#endif
} 

static void finISR(timer16_Sequence_t timer)
{
    //disable use of the given timer
#if defined WIRING   // Wiring
  if(timer == _timer1) {
    #if defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
    TIMSK1 &=  ~_BV(OCIE1A) ;  // disable timer 1 output compare interrupt
    #else 
    TIMSK &=  ~_BV(OCIE1A) ;  // disable timer 1 output compare interrupt   
    #endif
    timerDetach(TIMER1OUTCOMPAREA_INT); 
  }
  else if(timer == _timer3) {     
    #if defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
    TIMSK3 &= ~_BV(OCIE3A);    // disable the timer3 output compare A interrupt
    #else
    ETIMSK &= ~_BV(OCIE3A);    // disable the timer3 output compare A interrupt
    #endif
    timerDetach(TIMER3OUTCOMPAREA_INT);
  }
#else
    //For arduino - in future: call here to a currently undefined function to reset the timer
#endif
}

static boolean isTimerActive(timer16_Sequence_t timer)
{
  // returns true if any servo is active on this timer
  for(uint8_t channel=0; channel < ESCS_PER_TIMER; channel++) {
    if(SERVO(timer,channel).Pin.isActive == true)
      return true;
  }
  return false;
}


/****************** end of static functions ******************************/

ESC::ESC()
{
  if( ServoCount < MAX_ESCs) {
    this->ESCIndex = ServoCount++;                    // assign a servo index to this instance
	ESC_array[this->ESCIndex].ticks = usToTicks(DISARMED_PULSE_WIDTH);   // store default values  - 12 Aug 2009
  }
  else
    this->ESCIndex = INVALID_SERVO ;  // too many ESC_array 
}

uint8_t ESC::attach(int pin)
{
  if(this->ESCIndex < MAX_ESCs ) {
    pinMode( pin, OUTPUT) ;                                   // set servo pin to output
    ESC_array[this->ESCIndex].Pin.nbr = pin;  

    // initialize the timer if it has not already been initialized 
    timer16_Sequence_t timer = ESC_INDEX_TO_TIMER(ESCIndex);
    if(isTimerActive(timer) == false)
      initISR(timer);    
    ESC_array[this->ESCIndex].Pin.isActive = true;  // this must be set after the check for isTimerActive
    this->armed = false;
  } 
  return this->ESCIndex ;
}

void ESC::detach()  
{
  ESC_array[this->ESCIndex].Pin.isActive = false;  
  timer16_Sequence_t timer = ESC_INDEX_TO_TIMER(ESCIndex);
  if(isTimerActive(timer) == false) {
    finISR(timer);
  }
}

void ESC::write(float value)
{  
  int us(DISARMED_PULSE_WIDTH);
  if(this->armed){
    if(value > 1.0){
      value = 1.0;    
    }else if(value < 0.0){
      value = 0.0;
    }
    us = value*(MAX_PULSE_WIDTH - ARMED_PULSE_WIDTH) + ARMED_PULSE_WIDTH;
  }
  this->writeMicroseconds(us);
}

void ESC::writeMicroseconds(int value)
{
  // calculate and store the values for the given channel
  byte channel = this->ESCIndex;
  if(channel < MAX_ESCs)   // ensure channel is valid
  {  
    if (this->armed){
      if(value < ARMED_PULSE_WIDTH){
        value = ARMED_PULSE_WIDTH;
      }else if(value > MAX_PULSE_WIDTH)
      value = MAX_PULSE_WIDTH;  
    }else{
      value = DISARMED_PULSE_WIDTH;
    } 
    
  	value = value - TRIM_DURATION;
    value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead - 12 Aug 2009

    uint8_t oldSREG = SREG;
    cli();
    ESC_array[channel].ticks = value;  
    SREG = oldSREG;   
  } 
}

bool ESC::attached()
{
  return ESC_array[this->ESCIndex].Pin.isActive ;
}

void ESC::arm()
{
  this->armed = true;
  this->writeMicroseconds(ARMED_PULSE_WIDTH);
}

void ESC::disarm()
{
  this->armed = false;
  this->writeMicroseconds(DISARMED_PULSE_WIDTH);
}

bool ESC::isArmed()
{
  return this->armed;
}

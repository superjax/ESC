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
  A PWM signal is sent at 400 Hz to the ESC using the value most recently written using the write() method

  Note that analogWrite of PWM on pins associated with the timer are disabled when the first ESC is attached.
  Timers are seized as needed in groups of 12 ESCs - 24 ESCs use two timers, 48 ESCs will use four.
  The sequence used to sieze timers is defined in timers.h

  The methods are:

   ESC - Class for manipulating electronic speed controllers for brushless DC motors connected to Arduino pins.

   attach(pin )  - Attaches an ESC to an i/o pin.
   attach(pin, min, max) - Attaches to a pin setting min and max values in microseconds
   default min is 1000, max is 2000  
 
   write()     - Sets the normalized ESC throttle (between 0.0 and 1.0).
   writeMicroseconds() - Sets the pulse width in microseconds 
   read()      - Gets the last written normalized ESC throttle. 
   readMicroseconds()   - Gets the last written ESC pulse width in microseconds.
   attached()  - Returns true if there is an ESC attached. 
   detach()    - Stops an attached ESC from pulsing its i/o pin. 
 */

#ifndef Servo_h
#define Servo_h

#include <inttypes.h>

/* 
 * Defines for 16 bit timers used with  Servo library 
 *
 * If _useTimerX is defined then TimerX is a 16 bit timer on the curent board
 * timer16_Sequence_t enumerates the sequence that the timers should be allocated
 * _Nbr_16timers indicates how many 16 bit timers are available.
 *
 */

// Say which 16 bit timers can be used and in what order
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define _useTimer5
#define _useTimer1 
#define _useTimer3
#define _useTimer4 
typedef enum { _timer5, _timer1, _timer3, _timer4, _Nbr_16timers } timer16_Sequence_t ;

#elif defined(__AVR_ATmega32U4__)  
#define _useTimer1 
typedef enum { _timer1, _Nbr_16timers } timer16_Sequence_t ;

#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
#define _useTimer3
#define _useTimer1
typedef enum { _timer3, _timer1, _Nbr_16timers } timer16_Sequence_t ;

#elif defined(__AVR_ATmega128__) ||defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
#define _useTimer3
#define _useTimer1
typedef enum { _timer3, _timer1, _Nbr_16timers } timer16_Sequence_t ;

#else  // everything else
#define _useTimer1
typedef enum { _timer1, _Nbr_16timers } timer16_Sequence_t ;                  
#endif

#define ESC_VERSION           1      // software version of this library

#define MIN_PULSE_WIDTH       1000     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH       2000     // the longest pulse sent to a servo 
#define DISARMED_PULSE_WIDTH  1000     // default pulse width when servo is attached
#define ARMED_PULSE_WIDTH     1100     // default pulse width when servo is attached
#define REFRESH_INTERVAL      2500     // minumim time to refresh servos in microseconds (default 400 Hz)

#define ESCS_PER_TIMER       12     // the maximum number of servos controlled by one timer 
#define MAX_ESCs   (_Nbr_16timers  * ESCS_PER_TIMER)

#define INVALID_SERVO         255     // flag indicating an invalid servo index

typedef struct  {
  uint8_t nbr        :6 ;             // a pin number from 0 to 63
  uint8_t isActive   :1 ;             // true if this channel is enabled, pin not pulsed if false 
} ESCPin_t   ;  

typedef struct {
  ESCPin_t Pin;
  unsigned int ticks;
} ESC_t;

class ESC
{
public:
  ESC();
  uint8_t attach(int pin);           // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  void detach();
  void write(float value);             // Sets the normalized ESC throttle (between 0.0 and 1.0).
  void writeMicroseconds(int value); // Write pulse width in microseconds 
  bool attached();                   // return true if this ESC is attached, otherwise false 
  void arm();                        // arms the ESC
  void disarm();                     // disarms the ESC
  bool isArmed();                    // returns true if ESC is armed, false if not

private:
   uint8_t ESCIndex;                 // index into the channel data for this ESC
   bool armed;                       // A flag to know if the ESC is armed
};

#endif

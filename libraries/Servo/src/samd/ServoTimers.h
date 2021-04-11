/*
  Copyright (c) 2015 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

/*
 * Defines for 16 bit timers used with  Servo library
 *
 * If _useTimerX is defined then TimerX is a 16 bit timer on the current board
 * timer16_Sequence_t enumerates the sequence that the timers should be allocated
 * _Nbr_16timers indicates how many 16 bit timers are available.
 */

#ifndef __SERVO_TIMERS_H__
#define __SERVO_TIMERS_H__


/**
 * SAMD Only definitions
 * ---------------------
 */

// processor list taken from here: 
// https://github.com/arduino/ArduinoModule-CMSIS-Atmel/blob/master/CMSIS-Atmel/CMSIS/Device/ATMEL/samd21/include/samd21.h
#if defined(__SAMD21E16A__)  || defined(__ATSAMD21E16A__)  \
||  defined(__SAMD21E17A__)  || defined(__ATSAMD21E17A__)  \
||  defined(__SAMD21E18A__)  || defined(__ATSAMD21E18A__)  \
||  defined(__SAMD21G15A__)  || defined(__ATSAMD21G15A__)  \
||  defined(__SAMD21G16A__)  || defined(__ATSAMD21G16A__)  \
||  defined(__SAMD21G17A__)  || defined(__ATSAMD21G17A__)  \
||  defined(__SAMD21G17AU__) || defined(__ATSAMD21G17AU__) \
||  defined(__SAMD21G18A__)  || defined(__ATSAMD21G18A__)  \
||  defined(__SAMD21G18AU__) || defined(__ATSAMD21G18AU__) \
||  defined(__SAMD21J15A__)  || defined(__ATSAMD21J15A__)  \
||  defined(__SAMD21J16A__)  || defined(__ATSAMD21J16A__)  \
||  defined(__SAMD21J16AC__) || defined(__ATSAMD21J16AC__) \
||  defined(__SAMD21J17A__)  || defined(__ATSAMD21J17A__)  \
||  defined(__SAMD21J17AC__) || defined(__ATSAMD21J17AC__) \
||  defined(__SAMD21J18A__)  || defined(__ATSAMD21J18A__)  \
||  defined(__SAMD21J18AC__) || defined(__ATSAMD21J18AC__)
  #if !defined(__SAMD21__)
    #define __SAMD21__
  #endif
#endif

/*
 * Note:
 * 
 * TC4 and TC5 use the same peripheral clock. If we configure the clock
 * for one of these, we WILL configure the clock for the other. This means
 * that stuff on *both* TC instances will probably break when using this
 * library (most notably PWM with analogWrite(), no matter if we actually 
 * use both TC instances or only one.
 */

// For SAMD:
#define _useTimer1
#define _useTimer2   // <- TODO do not activate until the code in Servo.cpp has been changed in order
                       //         to manage more than one channel per timer on the SAMD architecture

#if defined(__SAMD21__)
    #if defined (_useTimer1)
        #define TIMER1_TC                 TC4          // TC instance to use. Pointer of type Tc.
        #define TIMER1_IRQn               TC4_IRQn     // Interrupt line for this TC instance.
        #define TIMER1_HANDLER            TC4_Handler  // ISR for this TC instance
        #define TIMER1_GCLK_ID            TC4_GCLK_ID  // index of peripheral for GCLK
    #endif
    #if defined (_useTimer2)
        #define TIMER2_TC                 TC5          // TC instance to use. Pointer of type Tc.
        #define TIMER2_IRQn               TC5_IRQn     // Interrupt line for this TC instance.
        #define TIMER2_HANDLER            TC5_Handler  // ISR for this TC instance
        #define TIMER2_GCLK_ID            TC5_GCLK_ID  // index of peripheral for GCLK
    #endif
#elif defined(__SAMD51__) 
    #if defined (_useTimer1)
        #define TIMER1_TC                 TC5          // TC instance to use. Pointer of type Tc.
        #define TIMER1_IRQn               TC5_IRQn     // Interrupt line for this TC instance.
        #define TIMER1_HANDLER            TC5_Handler  // ISR for this TC instance
        #define TIMER1_GCLK_ID            TC5_GCLK_ID  // index of peripheral for GCLK
    #endif  
    #if defined (_useTimer2)
        #define TIMER2_TC                 TC4          // TC instance to use. Pointer of type Tc.
        #define TIMER2_IRQn               TC4_IRQn     // Interrupt line for this TC instance.
        #define TIMER2_HANDLER            TC4_Handler  // ISR for this TC instance
        #define TIMER2_GCLK_ID            TC4_GCLK_ID  // index of peripheral for GCLK
    #endif
#endif

typedef enum {
#if defined (_useTimer1)
    _timer1,
#endif
#if defined (_useTimer2)
    _timer2,
#endif
    _Nbr_16timers } timer16_Sequence_t;

typedef enum {
    _cc0,
    _cc1,
    _Nbr_CC_registers } CC_register_t;

#endif   // __SERVO_TIMERS_H__

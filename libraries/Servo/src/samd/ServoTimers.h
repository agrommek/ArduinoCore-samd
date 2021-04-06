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

// taken from here: 
// https://github.com/arduino/ArduinoModule-CMSIS-Atmel/blob/46ab1021146152a64caf1ddbb837d8181b8faa35/CMSIS-Atmel/CMSIS/Device/ATMEL/samd21/include/samd21.h
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

// For SAMD:
#define _useTimer1
//#define _useTimer2   // <- TODO do not activate until the code in Servo.cpp has been changed in order
                       //         to manage more than one channel per timer on the SAMD architecture

#if defined(__SAMD51__) 
  #if defined (_useTimer1)
    #define TC_FOR_TIMER1             TC1
    #define CHANNEL_FOR_TIMER1        0
    #define INTENSET_BIT_FOR_TIMER_1  TC_INTENSET_MC0
    #define INTENCLR_BIT_FOR_TIMER_1  TC_INTENCLR_MC0
    #define INTFLAG_BIT_FOR_TIMER_1   TC_INTFLAG_MC0
    #define ID_TC_FOR_TIMER1          ID_TC1
    #define IRQn_FOR_TIMER1           TC1_IRQn
    #define HANDLER_FOR_TIMER1        TC1_Handler
    #define GCM_FOR_TIMER_1           TC1_GCLK_ID
  #endif

  #if defined (_useTimer2)
    #define TC_FOR_TIMER2             TC1
    #define CHANNEL_FOR_TIMER2        1
    #define INTENSET_BIT_FOR_TIMER_2  TC_INTENSET_MC1
    #define INTENCLR_BIT_FOR_TIMER_2  TC_INTENCLR_MC1
    #define INTFLAG_BIT_FOR_TIMER_2   TC_INTFLAG_MC1
    #define ID_TC_FOR_TIMER2          ID_TC1
    #define IRQn_FOR_TIMER2           TC1_IRQn
    #define HANDLER_FOR_TIMER2        TC1_Handler
    #define GCM_FOR_TIMER_2           TC1_GCLK_ID
  #endif
#else
  #if defined (_useTimer1)
    #define TC_FOR_TIMER1             TC4
    #define CHANNEL_FOR_TIMER1        0
    #define INTENSET_BIT_FOR_TIMER_1  TC_INTENSET_MC0
    #define INTENCLR_BIT_FOR_TIMER_1  TC_INTENCLR_MC0
    #define INTFLAG_BIT_FOR_TIMER_1   TC_INTFLAG_MC0
    #define ID_TC_FOR_TIMER1          ID_TC4
    #define IRQn_FOR_TIMER1           TC4_IRQn
    #define HANDLER_FOR_TIMER1        TC4_Handler
    #define GCM_FOR_TIMER_1           GCM_TC4_TC5
  #endif

  #if defined (_useTimer2)
    #define TC_FOR_TIMER2             TC4
    #define CHANNEL_FOR_TIMER2        1
    #define INTENSET_BIT_FOR_TIMER_2  TC_INTENSET_MC1
    #define INTENCLR_BIT_FOR_TIMER_2  TC_INTENCLR_MC1
    #define ID_TC_FOR_TIMER2          ID_TC4
    #define IRQn_FOR_TIMER2           TC4_IRQn
    #define HANDLER_FOR_TIMER2        TC4_Handler
    #define GCM_FOR_TIMER_2           GCM_TC4_TC5
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

#endif   // __SERVO_TIMERS_H__

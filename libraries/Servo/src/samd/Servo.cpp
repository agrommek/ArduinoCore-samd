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

#if defined(ARDUINO_ARCH_SAMD)

#include <Arduino.h>
#include <Servo.h>


// use different prescalers depending on F_CPU (avoid overflowing 16-bit counter)
// Note: There are no prescalers of size 32 or 128 for TC instances.
#if(F_CPU >  197000000) // between 197 MHz and ~7.7 GHz (more than any SAMD MCU can do)
  #define GCLK_PRESCALER (256)
  #define TC_CTRLA_PRESCALER_USED TC_CTRLA_PRESCALER_DIV256
#elif(F_CPU > 49000000) // between 49 MHz and 197 MHz (--> SAMD51@120MHz)
  #define GCLK_PRESCALER (64)
  #define TC_CTRLA_PRESCALER_USED TC_CTRLA_PRESCALER_DIV64
#elif(F_CPU > 24000000) // between 24 MHz and 49 MHz (--> SAMD21@48MHz)
  #define GCLK_PRESCALER (16)
  #define TC_CTRLA_PRESCALER_USED TC_CTRLA_PRESCALER_DIV16
#elif(F_CPU > 12000000) // between 12 MHz and 24 MHz
  #define GCLK_PRESCALER (8)
  #define TC_CTRLA_PRESCALER_USED TC_CTRLA_PRESCALER_DIV8
#elif(F_CPU >  6000000) // between 6 MHz and 12 MHz
  #define GCLK_PRESCALER (4)
  #define TC_CTRLA_PRESCALER_USED TC_CTRLA_PRESCALER_DIV4
#elif(F_CPU >  3000000) // between 3 MHz and 6 MHz
  #define GCLK_PRESCALER (2)
  #define TC_CTRLA_PRESCALER_USED TC_CTRLA_PRESCALER_DIV2
#elif(F_CPU <= 3000000) // less than 3 MHz
  #define GCLK_PRESCALER (1)
  #define TC_CTRLA_PRESCALER_USED TC_CTRLA_PRESCALER_DIV1
#else
  #error unsupported CPU clock frequency
#endif

// macro to convert microseconds to ticks
#define usToTicks(_us)    ((clockCyclesPerMicrosecond() * _us) / (GCLK_PRESCALER))
// macro to convert ticks back to microseconds
#define ticksToUs(_ticks) (((unsigned) _ticks * (GCLK_PRESCALER)) / clockCyclesPerMicrosecond())

// wrap around the counter after this number of tick corresponding PWM
// frequency of 50 Hz <--> a refresh interval of 20 ms
#define WRAPAROUND_TICKS  (usToTicks(REFRESH_INTERVAL))

// compensation ticks to trim adjust for digitalWrite delays
#define TRIM_DURATION  5

// static array of servo structures
static servo_t servos[MAX_SERVOS];

// the total number of attached servos
uint8_t ServoCount = 0;

// index for the servo being pulsed for each timer (or -1 if refresh interval)
static volatile int8_t currentServoIndex[_Nbr_16timers];

// convenience macros
#define SERVO_INDEX_TO_TIMER(_servo_nbr) ((timer16_Sequence_t)(_servo_nbr / SERVOS_PER_TIMER))   // returns the timer controlling this servo
#define SERVO_INDEX_TO_CHANNEL(_servo_nbr) (_servo_nbr % SERVOS_PER_TIMER)                       // returns the index of the servo on this timer
#define SERVO_INDEX(_timer,_channel)  ((_timer*SERVOS_PER_TIMER) + _channel)                     // macro to access servo index by timer and channel
#define SERVO(_timer,_channel)  (servos[SERVO_INDEX(_timer,_channel)])                           // macro to access servo class by timer and channel

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)   // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)   // maximum value in uS for this servo

// Referenced in SAMD21 code only, no harm in defining regardless
#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);

/************ static functions common to all instances ***********************/

void Servo_Handler(timer16_Sequence_t timer, Tc *pTc, uint8_t channel, uint8_t intFlag);
#if defined (_useTimer1)
void HANDLER_FOR_TIMER1(void) {
    Servo_Handler(_timer1, TC_FOR_TIMER1, CHANNEL_FOR_TIMER1, INTFLAG_BIT_FOR_TIMER_1);
}
#endif
#if defined (_useTimer2)
void HANDLER_FOR_TIMER2(void) {
    Servo_Handler(_timer2, TC_FOR_TIMER2, CHANNEL_FOR_TIMER2, INTFLAG_BIT_FOR_TIMER_2);
}
#endif

void Servo_Handler(timer16_Sequence_t timer, Tc *tc, uint8_t channel, uint8_t intFlag)
{
    if (currentServoIndex[timer] < 0) {
        tc->COUNT16.COUNT.reg = (uint16_t) 0;
#if defined(__SAMD21__)
        WAIT_TC16_REGS_SYNC(tc)
#elif defined(__SAMD51__)
        while(tc->COUNT16.SYNCBUSY.bit.COUNT);
#endif
    } else {
        if (SERVO_INDEX(timer, currentServoIndex[timer]) < ServoCount && SERVO(timer, currentServoIndex[timer]).Pin.isActive == true) {
            digitalWrite(SERVO(timer, currentServoIndex[timer]).Pin.nbr, LOW);   // pulse this channel low if activated
        }
    }

    // Select the next servo controlled by this timer
    currentServoIndex[timer]++;

    if (SERVO_INDEX(timer, currentServoIndex[timer]) < ServoCount && currentServoIndex[timer] < SERVOS_PER_TIMER) {
        if (SERVO(timer, currentServoIndex[timer]).Pin.isActive == true) {   // check if activated
            digitalWrite(SERVO(timer, currentServoIndex[timer]).Pin.nbr, HIGH);   // it's an active channel so pulse it high
        }

        // Get the counter value
#if defined(__SAMD51__)
        // Note from datasheet: Prior to any read access, this register must be synchronized by user by writing the according TC
        // Command value to the Control B Set register (CTRLBSET.CMD=READSYNC)
        while (tc->COUNT16.SYNCBUSY.bit.CTRLB);
        tc->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_READSYNC_Val;
        while (tc->COUNT16.SYNCBUSY.bit.CTRLB);
#endif
        uint16_t tcCounterValue = tc->COUNT16.COUNT.reg;
#if defined(__SAMD21__)
        WAIT_TC16_REGS_SYNC(tc)
#elif defined(__SAMD51__)
        while(tc->COUNT16.SYNCBUSY.bit.COUNT);
#endif

        tc->COUNT16.CC[channel].reg = (uint16_t) (tcCounterValue + SERVO(timer, currentServoIndex[timer]).ticks);
#if defined(__SAMD21__)
        WAIT_TC16_REGS_SYNC(tc)
#elif defined(__SAMD51__)
        if(channel == 0) {
            while(tc->COUNT16.SYNCBUSY.bit.CC0);
        } else if(channel == 1) {
            while(tc->COUNT16.SYNCBUSY.bit.CC1);
        }
#endif
    }
    else {
        // finished all channels so wait for the refresh period to expire before starting over

        // Get the counter value
        uint16_t tcCounterValue = tc->COUNT16.COUNT.reg;
#if defined(__SAMD21__)
        WAIT_TC16_REGS_SYNC(tc)
#elif defined(__SAMD51__)
        while(tc->COUNT16.SYNCBUSY.bit.COUNT);
#endif

        if (tcCounterValue + 4UL < usToTicks(REFRESH_INTERVAL)) {   // allow a few ticks to ensure the next OCR1A not missed
            tc->COUNT16.CC[channel].reg = (uint16_t) usToTicks(REFRESH_INTERVAL);
        }
        else {
            tc->COUNT16.CC[channel].reg = (uint16_t) (tcCounterValue + 4UL);   // at least REFRESH_INTERVAL has elapsed
        }
#if defined(__SAMD21__)
        WAIT_TC16_REGS_SYNC(tc)
#elif defined(__SAMD51__)
        if(channel == 0) {
            while(tc->COUNT16.SYNCBUSY.bit.CC0);
        } else if(channel == 1) {
            while(tc->COUNT16.SYNCBUSY.bit.CC1);
        }
#endif

        currentServoIndex[timer] = -1;   // this will get incremented at the end of the refresh period to start again at the first channel
    }

    // Clear the interrupt
    tc->COUNT16.INTFLAG.reg = intFlag;
}

static inline void resetTC (Tc* TCx)
{
    // Disable TCx
    TCx->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
#if defined(__SAMD21__)
    WAIT_TC16_REGS_SYNC(TCx)
#elif defined(__SAMD51__)
    while(TCx->COUNT16.SYNCBUSY.bit.ENABLE);
#endif

    // Reset TCx
    TCx->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
#if defined(__SAMD21__)
    WAIT_TC16_REGS_SYNC(TCx)
#elif defined(__SAMD51__)
    while(TCx->COUNT16.SYNCBUSY.bit.SWRST);
#endif
    while (TCx->COUNT16.CTRLA.bit.SWRST);
}

static void _initISR(Tc *tc, uint8_t channel, uint32_t id, IRQn_Type irqn, uint8_t gcmForTimer, uint8_t intEnableBit)
{
    (void)id;
    // Select GCLK0 as timer/counter input clock source
#if defined(__SAMD21__)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(gcmForTimer));
    while (GCLK->STATUS.bit.SYNCBUSY);
#elif defined(__SAMD51__)
    int idx = gcmForTimer;           // see datasheet Table 14-9
    GCLK->PCHCTRL[idx].bit.GEN  = 0; // Select GCLK0 as periph clock source
    GCLK->PCHCTRL[idx].bit.CHEN = 1; // Enable peripheral
    while(!GCLK->PCHCTRL[idx].bit.CHEN);
#endif

    // Reset the timer
    // TODO this is not the right thing to do if more than one channel per timer is used by the Servo library
    resetTC(tc);

    // Set timer counter mode to 16 bits
    tc->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

#if defined(__SAMD21__) // for SAMD21
    // Set timer counter mode as normal PWM
    tc->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_NPWM;
    // Set the prescaler factor to GCLK_TC/16.  At nominal 48MHz GCLK_TC this is 3000 ticks per millisecond
    tc->COUNT16.CTRLA.reg |= (uint16_t) TC_CTRLA_PRESCALER_USED;
#elif defined(__SAMD51__)
    // Set timer counter mode as normal PWM
    tc->COUNT16.WAVE.bit.WAVEGEN = TCC_WAVE_WAVEGEN_NPWM_Val;
    // Set the prescaler factor to 64 or 128 depending on FCPU
    // (avoid overflowing 16-bit clock counter)
 #if(F_CPU > 200000000)
    tc->COUNT16.CTRLA.bit.PRESCALER = TCC_CTRLA_PRESCALER_DIV128_Val;
 #else
    // At 120-200 MHz GCLK this is 1875-3125 ticks per millisecond
    tc->COUNT16.CTRLA.bit.PRESCALER = TCC_CTRLA_PRESCALER_DIV64_Val;
 #endif
#endif

    // Count up
    tc->COUNT16.CTRLBCLR.bit.DIR = 1;
#if defined(__SAMD21__)
    WAIT_TC16_REGS_SYNC(tc)
#elif defined(__SAMD51__)
    while(tc->COUNT16.SYNCBUSY.bit.CTRLB);
#endif

    // First interrupt request after 1 ms
    tc->COUNT16.CC[channel].reg = (uint16_t) usToTicks(1000UL);
#if defined(__SAMD21__)
    WAIT_TC16_REGS_SYNC(tc)
#elif defined(__SAMD51__)
    if(channel == 0) {
        while(tc->COUNT16.SYNCBUSY.bit.CC0);
    } else if(channel == 1) {
        while(tc->COUNT16.SYNCBUSY.bit.CC1);
    }
#endif

    // Configure interrupt request
    // TODO this should be changed if more than one channel per timer is used by the Servo library
    NVIC_DisableIRQ(irqn);
    NVIC_ClearPendingIRQ(irqn);
    NVIC_SetPriority(irqn, 0);
    NVIC_EnableIRQ(irqn);

    // Enable the match channel interrupt request
    tc->COUNT16.INTENSET.reg = intEnableBit;

    // Enable the timer and start it
    tc->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
#if defined(__SAMD21__)
    WAIT_TC16_REGS_SYNC(tc)
#elif defined(__SAMD51__)
    while(tc->COUNT16.SYNCBUSY.bit.ENABLE);
#endif
}

static void initISR(timer16_Sequence_t timer)
{
#if defined (_useTimer1)
    if (timer == _timer1)
        _initISR(TC_FOR_TIMER1, CHANNEL_FOR_TIMER1, ID_TC_FOR_TIMER1, IRQn_FOR_TIMER1, GCM_FOR_TIMER_1, INTENSET_BIT_FOR_TIMER_1);
#endif
#if defined (_useTimer2)
    if (timer == _timer2)
        _initISR(TC_FOR_TIMER2, CHANNEL_FOR_TIMER2, ID_TC_FOR_TIMER2, IRQn_FOR_TIMER2, GCM_FOR_TIMER_2, INTENSET_BIT_FOR_TIMER_2);
#endif
}

static void finISR(timer16_Sequence_t timer)
{
  (void)timer;
#if defined (_useTimer1)
    // Disable the match channel interrupt request
    TC_FOR_TIMER1->COUNT16.INTENCLR.reg = INTENCLR_BIT_FOR_TIMER_1;
#endif
#if defined (_useTimer2)
    // Disable the match channel interrupt request
    TC_FOR_TIMER2->COUNT16.INTENCLR.reg = INTENCLR_BIT_FOR_TIMER_2;
#endif
}

static boolean isTimerActive(timer16_Sequence_t timer)
{
  // returns true if any servo is active on this timer
  for(uint8_t channel=0; channel < SERVOS_PER_TIMER; channel++) {
    if(SERVO(timer,channel).Pin.isActive == true)
      return true;
  }
  return false;
}

/****************** end of static functions ******************************/

Servo::Servo()
{
  if (ServoCount < MAX_SERVOS) {
    this->servoIndex = ServoCount++;                    // assign a servo index to this instance
    servos[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);   // store default values
  } else {
    this->servoIndex = INVALID_SERVO;  // too many servos
  }
}

uint8_t Servo::attach(int pin)
{
  return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t Servo::attach(int pin, int min, int max)
{
  timer16_Sequence_t timer;

  if (this->servoIndex < MAX_SERVOS) {
    pinMode(pin, OUTPUT);                                   // set servo pin to output
    servos[this->servoIndex].Pin.nbr = pin;
    // todo min/max check: abs(min - MIN_PULSE_WIDTH) /4 < 128
    this->min  = (MIN_PULSE_WIDTH - min)/4; //resolution of min/max is 4 uS
    this->max  = (MAX_PULSE_WIDTH - max)/4;
    // initialize the timer if it has not already been initialized
    timer = SERVO_INDEX_TO_TIMER(servoIndex);
    if (isTimerActive(timer) == false) {
      initISR(timer);
    }
    servos[this->servoIndex].Pin.isActive = true;  // this must be set after the check for isTimerActive
  }
  return this->servoIndex;
}

void Servo::detach()
{
  timer16_Sequence_t timer;

  servos[this->servoIndex].Pin.isActive = false;
  timer = SERVO_INDEX_TO_TIMER(servoIndex);
  if(isTimerActive(timer) == false) {
    finISR(timer);
  }
}

void Servo::write(int value)
{
  // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
  if (value < MIN_PULSE_WIDTH)
  {
    if (value < 0)
      value = 0;
    else if (value > 180)
      value = 180;

    value = map(value, 0, 180, SERVO_MIN(), SERVO_MAX());
  }
  writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value)
{
  // calculate and store the values for the given channel
  byte channel = this->servoIndex;
  if( (channel < MAX_SERVOS) )   // ensure channel is valid
  {
    if (value < SERVO_MIN())          // ensure pulse width is valid
      value = SERVO_MIN();
    else if (value > SERVO_MAX())
      value = SERVO_MAX();

    value = value - TRIM_DURATION;
    value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead
    servos[channel].ticks = value;
  }
}

int Servo::read() // return the value as degrees
{
  return map(readMicroseconds()+1, SERVO_MIN(), SERVO_MAX(), 0, 180);
}

int Servo::readMicroseconds()
{
  unsigned int pulsewidth;
  if (this->servoIndex != INVALID_SERVO)
    pulsewidth = ticksToUs(servos[this->servoIndex].ticks)  + TRIM_DURATION;
  else
    pulsewidth  = 0;

  return pulsewidth;
}

bool Servo::attached()
{
  return servos[this->servoIndex].Pin.isActive;
}

#endif // ARDUINO_ARCH_SAMD

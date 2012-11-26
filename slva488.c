/*
 * slva488.c
 *
 *  Created on: Oct 1, 2012
 *      Author: csdexter
 */

#include <stdbool.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>


#define DIR _BV(PINB3)

typedef enum {
  STOP,
  ACCEL,
  DECEL,
  NOACC
} TAccelState;

static TAccelState AccelerationState;
static uint32_t ExecutedSteps, NumberOfSteps, StepsToStop, StepPosition;
static uint16_t AccelerationTime, AccelerationIncrease, ATBCount,
    StepperSpeedTMR, DecelerationRate, ActualStepperSpeed, DesiredStepperSpeed,
    StartStepperSpeed;
static uint8_t StepPulseWidth;
static volatile uint16_t TimerA0Count, tmpAccelerationTime;
static volatile bool shouldSleep;

void AccelTimeCompute(unsigned int AccelDecelRate) {
  if(AccelDecelRate <= 4000) {
    AccelerationTime = 4000/AccelDecelRate;
    AccelerationIncrease = 1;
  } else {
    AccelerationTime = 1;
    AccelerationIncrease = AccelDecelRate/4000;
  }
  tmpAccelerationTime = AccelerationTime;
}

//Timer1_A0 == Timer1, OCRA
//OCR1A, acceleration
//will replace with overflow since we don't do the PWM stuff
ISR(TIMER1_COMPA_vect) {
  if(++TimerA0Count == ATBCount) {
    TimerA0Count = 0;
    if(tmpAccelerationTime == 0) {
      tmpAccelerationTime = AccelerationTime;
      shouldSleep = false;
    } else shouldSleep = true;
    tmpAccelerationTime--;
  }
}

//Timer0_A1 == Timer0, OCRB+OCRC+OVF
//OCR0B = set step pulse
//OCR0A = reset step pulse
ISR(TIMER0_COMPB_vect) {
  OCR0A = OCR0B + StepPulseWidth; //2 us at 16 MHz
  OCR0B += StepperSpeedTMR;
  ExecutedSteps++;
  if(ExecutedSteps == NumberOfSteps) {
    TCCR0B &= ~(_BV(CS02) | _BV(CS01) | _BV(CS00)); //Stop counting
    TIMSK0 &= ~(_BV(OCIE0A) | _BV(OCIE0B)); //Disable interrupts
  } else if(ExecutedSteps == StepsToStop) {
    AccelTimeCompute(DecelerationRate);
    AccelerationState = STOP;
    TIMSK1 |= _BV(OCIE1A); //ENABLE 250 us coordinator interrupt on OCR1A
  }

  if(PINB & DIR) StepPosition++;
  else StepPosition--;

  //Set STEP pin here
}

ISR(TIMER0_COMPA_vect) {
  //Reset STEP pin here
  //Investigate using AVR builtin functionality to reset pin and do away with interrupt
}

/*
        case START_STEPPER_ADDR:
          NumberOfSteps = (ParametersTable[NUMBER_OF_STEPS3_ADDR] << 8) + ParametersTable[NUMBER_OF_STEPS2_ADDR])
          NumberOfSteps *= 65536;
          NumberOfSteps += (ParametersTable[NUMBER_OF_STEPS1_ADDR] << 8) + ParametersTable[NUMBER_OF_STEPS0_ADDR];
          ExecutedSteps = 0;
          StepsToStop = (ParametersTable[STEPS_TO_STOP3_ADDR] << 8) + ParametersTable[STEPS_TO_STOP2_ADDR];
          StepsToStop *= 65536;
          StepsToStop += (ParametersTable[STEPS_TO_STOP1_ADDR] << 8) + ParametersTable[STEPS_TO_STOP0_ADDR];
          StartStepperSpeed = (ParametersTable[START_SPEED1_ADDR] << 8) + ParametersTable[START_SPEED0_ADDR];
          DesiredStepperSpeed = (ParametersTable[DESIRED_SPEED1_ADDR] << 8) + ParametersTable[DESIRED_SPEED0_ADDR];
          AccelerationRate = (ParametersTable[ACCEL1_ADDR] << 8) + ParametersTable[ACCEL0_ADDR];
          DecelerationRate = (ParametersTable[DECEL1_ADDR] << 8) + ParametersTable[DECEL0_ADDR];
          switch(PARAMETER) {
            case START_ACCEL: // Start stepper motor at Start speed and until reaching desired speed
              AccelerationState = ACCEL;
              ActualStepperSpeed = StartStepperSpeed;
              AccelTimeCompute(AccelerationRate);
              TA1CCTL0 |= CCIE; //ENABLE 250 us coordinator interrupt on TA1.0
              break;
            case START_STEPPER: // Start stepper motor at start speed. Run at this rate.
              AccelerationState = NOACC;
              ActualStepperSpeed = StartStepperSpeed;
              break;
            case STOP_STEPPER: // Stop motor from current speed, through a deceleration profile and until reaching Start Speed. Then stop and disable stepping engine
              AccelTimeCompute(DecelerationRate);
              AccelerationState = STOP;
              TA1CCTL0 |= CCIE; //ENABLE 250 us coordinator interrupt on TA1.0
              break;
            case CHANGE_SPEED: // Modify current speed up or down to Desired Speed.
              if(DesiredStepperSpeed >= ActualStepperSpeed) {
                AccelTimeCompute(AccelerationRate);
                AccelerationState = ACCEL;
                TA1CCTL0 |= CCIE; //ENABLE 250 us coordinator interrupt on TA1.0
              } else {
                AccelTimeCompute(DecelerationRate);
                AccelerationState = DECEL;
                TA1CCTL0 |= CCIE; //ENABLE 250 us coordinator interrupt on TA1.0
              }
              break;
          }
          if(SpeedCompute(ActualStepperSpeed)) {
            P3OUT &= ~(nENABLE);
            P1OUT |= NSLEEP;
            TA0CCR2 += StepperSpeedTMR;
            //OCR0B += StepperSpeedTMR;
            TA0CCR0 = TACCR2 + StepPulseWidth; //2 us at 16 MHz
            //OCR0A = OCR0B + StepPulseWidth;
            TA0CCTL2 &= ~CCIFG;
            //TIFR0 |= _BV(OCF0B) | _BV(OCF0A) | _BV(TOV0);
            TA0CCTL2 |= (CCIE + TA0_OUTMOD2_CONF);
            //TIMSK0 = _BV(OCIE0B) | _BV(OCIE0A);
            //TCCR0[AB] setup comes here
          }
          break;
*/

#define PRESCALER_COUNT 5
const uint16_t prescalers[PRESCALER_COUNT][2] = {
    {1, _BV(CS10)},
    {8, _BV(CS11)},
    {64, _BV(CS11) | _BV(CS10)},
    {256, _BV(CS12)},
    {1024, _BV(CS12) | _BV(CS10)}};
#define COMPARE_MAX 0x10000UL

//Redo with AVR prescalers
void SpeedCompute(uint16_t freq) {
  uint32_t period = F_CPU / freq;
  uint8_t i;
  StepperSpeedTMR = 0;

  for(i = 0; i < PRESCALER_COUNT; i++) {
    if(period < COMPARE_MAX * prescalers[i][0]) {
      StepperSpeedTMR = (period / prescalers[i][0]) - 1;
      break;
    }
  }
  if(!StepperSpeedTMR) StepperSpeedTMR = 0xFFFF;

  OCR0B += StepperSpeedTMR;
  TCCR0B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)) | prescalers[i][1];
}

void AccelDecel(void) {
  switch(AccelerationState) {
    case NOACC: break;
    case ACCEL:
      ActualStepperSpeed += AccelerationIncrease;
      if(ActualStepperSpeed >= DesiredStepperSpeed) {
        ActualStepperSpeed = DesiredStepperSpeed;
        AccelerationState = NOACC;
        TIMSK1 |= _BV(OCIE1A);
      }
      SpeedCompute(ActualStepperSpeed);
      break;
    case DECEL:
      ActualStepperSpeed -= AccelerationIncrease;
      if(ActualStepperSpeed <= DesiredStepperSpeed) {
        ActualStepperSpeed = DesiredStepperSpeed;
        AccelerationState = NOACC;
        TIMSK1 |= _BV(OCIE1A);
      }
      SpeedCompute(ActualStepperSpeed);
      break;
    case STOP:
      ActualStepperSpeed -= AccelerationIncrease;
      if(ActualStepperSpeed <= StartStepperSpeed) {
        ActualStepperSpeed = StartStepperSpeed;
        AccelerationState = NOACC;
        TIMSK1 |= _BV(OCIE1A);
        ExecutedSteps = NumberOfSteps - 1;
      }
      SpeedCompute(ActualStepperSpeed);
      break;
  }
}

int main(void) {
  sei();
  //Do timer setup here
  //Do port pin setup here
  while(1) {
    if(!shouldSleep) {
      AccelDecel();
      cli();
      shouldSleep = true;
      sei();
    }
  }

  return 0; //Never reached
}

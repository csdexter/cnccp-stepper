/*
 * main.c - Boilerplate AVR main/startup code
 *
 * This is the Stepper MCU firmware, it receives movement commands from the
 * main MCU via SPI and drives stepper motor drivers using the the DIR/STEP
 * interface.
 *
 *  Created on: Nov 21, 2012
 *      Author: csdexter
 */

#include <stdbool.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "boilerplate.h"
#include "protocol.h"
#include "spi.h"
#include "stepper.h"


volatile bool NewCommand, LimitInvert;
volatile TSPIBuffer SPIBuf;
volatile uint16_t MaxAcceleration, MaxSpeed, StartSpeed;
volatile TInterruptFlags InterruptFlags, InterruptCauses;
TMoveBufferEntryEx MoveBuffer[2], MoveValues;
uint16_t StepPulseWidth, AccelTimeConstant;
uint8_t BufferSize;


ISR(PCINT0_vect) {
  if(PINA & _BV(PORTA3)) {
    NewCommand = true;
    spi_enable(SPI_OFF);
  } else {
    spi_enable(SPI_ON);
    spi_start(&SPIBuf.Data[0], &SPIBuf.Data[2]);
  }
}

ISR(INT0_vect) {
  if((PINB & _BV(PORTB2)) ^ LimitInvert) {
    SetFlagAndAssertInterrupt(_BV(MOVE_INTERRUPT_LIMITTRIP));
    if(InterruptFlags.flags.AutoLimit) {
      /* StartBraking */
      /* FlushMoveBuffer */
    }
  } else SetFlagAndAssertInterrupt(_BV(MOVE_INTERRUPT_LIMITCLEAR));
}

void SPI_hook(bool when) {
  if(when == SPI_HOOK_BEFORE) {
    SPIBuf.Data[2] = SPIBuf.Data[1];
    SPIBuf.Data[1] = SPIBuf.Data[0];
  } else spi_start(&SPIBuf.Data[0], &SPIBuf.Data[2]);
}

void SetFlagAndAssertInterrupt(uint8_t flag) {
  InterruptCauses.value |= flag;
  if(InterruptFlags.flags.GlobalEnable && (InterruptFlags.value & flag))
    DDRA |= _BV(PORTA2);
}

void ClearFlagsAndReleaseInterrupt(void) {
  InterruptCauses.value &= 0x01;
  DDRA &= ~_BV(PORTA2);
}

void init(void) {
  /* Setup GPIO ports */
  /* *_LIMIT# on PB2, input and external interrupt */
  /*   NOP, DDRB already contains "0" for input mode */
  MCUCR |= _BV(ISC00);
  /* *_DIR on PB1, output */
  DDRB = _BV(PORTB1);
  /* *_STEP on PA7, output */
  DDRA = _BV(PORTA7);
  /* MOSI, MISO and SCK configured by SPI */
  spi_configure(SPI_INT_ENABLE, SPI_ON, NULL, SPI_MASTER, NULL, SPI_PHASE_LEADING, NULL);
  spi_hook = SPI_hook;
  /* CS_STEP# on PA3, input and pin change interrupt */
  /*  NOP, DDRA already contains "0" for input mode */
  PCMSK0 = _BV(PCINT3);
  GIFR |= _BV(INTF0) | _BV(PCIF0); /* Avoid spurious interrupts on startup */
  GIMSK |= _BV(INT0) | _BV(PCIE0);
  /* INT# on PA2, output, open-collector */
  /*  NOP, DDRA already contains "0" for High-Z and PORTA already contains "0" for open-drain */

  /* Initialize state */
  NewCommand = false;
  LimitInvert = true;
  BufferSize = 0;

  /* And off we go */
  sei();
}

int main(void) {
  init();

  while(true) {
    if(NewCommand) {
      NewCommand = false;
      switch(SPIBuf.AsFields.CommandByte.Command) {
        case STEPPER_COMMAND_KINETIC:
          if(SPIBuf.AsFields.CommandByte.WriteMode)
            if(SPIBuf.AsFields.CommandByte.DataMode)
              MaxSpeed = SPIBuf.AsFields.ArgumentWord.AsWord.AsWord;
            else
              MaxAcceleration = SPIBuf.AsFields.ArgumentWord.AsWord.AsWord;
          else
            if(SPIBuf.AsFields.CommandByte.DataMode)
              SPIBuf.AsFields.ArgumentWord.AsWord.AsWord = MaxSpeed;
            else
              SPIBuf.AsFields.ArgumentWord.AsWord.AsWord = MaxAcceleration;
          break;
        case STEPPER_COMMAND_STATIC:
          if(SPIBuf.AsFields.CommandByte.WriteMode)
            if(SPIBuf.AsFields.CommandByte.DataMode)
              StartSpeed = SPIBuf.AsFields.ArgumentWord.AsWord.AsWord;
            else /* NOP */;
          else
            if(SPIBuf.AsFields.CommandByte.DataMode)
              SPIBuf.AsFields.ArgumentWord.AsWord.AsWord = StartSpeed;
            else /* NOP */;
          break;
        case STEPPER_COMMAND_MOVEMENT:
          if(SPIBuf.AsFields.CommandByte.WriteMode)
            if(SPIBuf.AsFields.CommandByte.DataMode)
              switch(SPIBuf.AsFields.ArgumentWord.AsValue.ValueType) {
                case MOVE_VALUE_SPEED:
                  if(!SPIBuf.AsFields.ArgumentWord.AsValue.ValueIndex)
                    if(!SPIBuf.AsFields.ArgumentWord.AsValue.ValueSlice)
                      MoveValues.linear.TargetSpeed = SPIBuf.AsFields.ArgumentWord.AsValue.Value;
                    else
                      MoveValues.linear.TargetSpeed |= (uint16_t)SPIBuf.AsFields.ArgumentWord.AsValue.Value << 12;
                  else /* NOP */;
                  break;
                case MOVE_VALUE_STEPS:
                  if(!SPIBuf.AsFields.ArgumentWord.AsValue.ValueIndex)
                    if(!SPIBuf.AsFields.ArgumentWord.AsValue.ValueSlice)
                      MoveValues.linear.StepCount = SPIBuf.AsFields.ArgumentWord.AsValue.Value;
                    else
                      MoveValues.linear.StepCount |= (uint32_t)SPIBuf.AsFields.ArgumentWord.AsValue.Value << 12;
                  else /* NOP */;
                  break;
                case MOVE_VALUE_RADIUSERROR:
                  if(!SPIBuf.AsFields.ArgumentWord.AsValue.ValueIndex)
                    if(!SPIBuf.AsFields.ArgumentWord.AsValue.ValueSlice)
                      MoveValues.circular.Radius = SPIBuf.AsFields.ArgumentWord.AsValue.Value;
                    else
                      MoveValues.circular.Radius |= (uint32_t)SPIBuf.AsFields.ArgumentWord.AsValue.Value << 12;
                  else
                    if(!SPIBuf.AsFields.ArgumentWord.AsValue.ValueSlice)
                      MoveValues.circular.Error = SPIBuf.AsFields.ArgumentWord.AsValue.Value;
                    else
                      /* using uint32_t as a container for int32_t to make sure the sign doesn't get messed with */
                      MoveValues.circular.Error |= (uint32_t)SPIBuf.AsFields.ArgumentWord.AsValue.Value << 12;
                  break;
                case MOVE_VALUE_CENTER:
                  if(!SPIBuf.AsFields.ArgumentWord.AsValue.ValueIndex)
                    if(!SPIBuf.AsFields.ArgumentWord.AsValue.ValueSlice)
                      MoveValues.circular.CenterX = SPIBuf.AsFields.ArgumentWord.AsValue.Value;
                    else
                      MoveValues.circular.CenterX |= (uint32_t)SPIBuf.AsFields.ArgumentWord.AsValue.Value << 12;
                  else
                    if(!SPIBuf.AsFields.ArgumentWord.AsValue.ValueSlice)
                      MoveValues.circular.CenterY = SPIBuf.AsFields.ArgumentWord.AsValue.Value;
                    else
                      MoveValues.circular.CenterY |= (uint32_t)SPIBuf.AsFields.ArgumentWord.AsValue.Value << 12;
                  break;
              }
            else {
              MoveValues.linear.flags.incoming.value = SPIBuf.AsFields.ArgumentWord.AsWord.AsBytes.LowByte;
              switch(MoveValues.linear.flags.incoming.flags.BufferIndex) {
                case MOVE_BUFFER_ENQUEUE:
                  if(BufferSize < 2) {
                    MoveBuffer[BufferSize] = MoveValues;
                    BufferSize++;
                  };
                  break;
                case MOVE_BUFFER_OVERRIDE_CURRENT:
                  MoveBuffer[0] = MoveValues;
                  break;
                case MOVE_BUFFER_OVERRIDE_NEXT:
                  MoveBuffer[1] = MoveValues;
                  break;
              }
            }
          else
            if(SPIBuf.AsFields.CommandByte.DataMode)
              SPIBuf.AsFields.ArgumentWord.AsWord.AsWord = (MoveBuffer[0].linear.StepCount >> 12) & 0xFFFFU;
            else
              SPIBuf.AsFields.ArgumentWord.AsWord.AsBytes.LowByte = MoveBuffer[0].linear.flags.ongoing.value;
          break;
          break;
        case STEPPER_COMMAND_INTERRUPT:
          if(SPIBuf.AsFields.CommandByte.WriteMode)
            if(SPIBuf.AsFields.CommandByte.DataMode)
              LimitInvert = SPIBuf.AsFields.ArgumentWord.AsWord.AsBytes.LowByte & MOVE_LIMIT_INVERT;
            else
              InterruptFlags.value = SPIBuf.AsFields.ArgumentWord.AsWord.AsBytes.LowByte;
          else
            if(SPIBuf.AsFields.CommandByte.DataMode)
              SPIBuf.AsFields.ArgumentWord.AsWord.AsBytes.LowByte = InterruptCauses.value;
            else
              SPIBuf.AsFields.ArgumentWord.AsWord.AsBytes.LowByte = InterruptFlags.value;
          break;
        default: /* NOP or invalid command received: ignore */
          break;
      }
    }
  }

  return 0;
}

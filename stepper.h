/*
 * stepper.h - types and prototypes used throughout the Stepper MCU code
 *
 *  Created on: Nov 25, 2012
 *      Author: csdexter
 */

#ifndef STEPPER_H_
#define STEPPER_H_

#include <stdint.h>

#include "protocol.h"


#define MOVE_TYPE_LINEAR 0x00
#define MOVE_TYPE_CIRCULAR_X 0x01
#define MOVE_TYPE_CIRCULAR_Y 0x02
#define MOVE_TYPE_NOP 0x03
#define MOVE_BUFFER_ENQUEUE 0x00
#define MOVE_BUFFER_OVERRIDE_CURRENT 0x01
#define MOVE_BUFFER_OVERRIDE_NEXT 0x02
#define MOVE_VALUE_SPEED 0x00
#define MOVE_VALUE_STEPS 0x01
#define MOVE_VALUE_RADIUSERROR 0x02
#define MOVE_VALUE_CENTER 0x03
#define MOVE_INTERRUPT_NEXT 6
#define MOVE_INTERRUPT_COMPLETE 5
#define MOVE_INTERRUPT_LIMITTRIP 4
#define MOVE_INTERRUPT_LIMITCLEAR 3
#define MOVE_LIMIT_INVERT 0x80

#define STEPPER_COMMAND_KINETIC 0x01
#define STEPPER_COMMAND_STATIC 0x02
#define STEPPER_COMMAND_MOVEMENT 0x03
#define STEPPER_COMMAND_INTERRUPT 0x04

typedef union {
  struct {
    uint8_t MoveType:2;
    uint8_t Positive:1;
    uint8_t BufferIndex:2;
    uint8_t OpenMove:1;
    uint8_t AutoStart:1;
    uint8_t CancelMove:1;
  } flags;
  uint8_t value;
} TMoveFlagsIncoming;

typedef struct {
  struct {
    uint8_t MoveType:2;
    uint8_t Positive:1;
    uint8_t LastMove:1;
    uint8_t Reserved1:1;
    uint8_t OpenMove:1;
    uint8_t Running:1;
    uint8_t Reserved2:1;
  } flags;
  uint8_t value;
} TMoveFlagsOngoing;

typedef struct {
  union {
    TMoveFlagsIncoming incoming;
    TMoveFlagsOngoing ongoing;
  } flags;
  uint16_t TargetSpeed;
  uint32_t StepCount;
} TMoveBufferEntry;

typedef struct {
  uint32_t Radius;
  int32_t Error;
  uint32_t CenterX;
  uint32_t CenterY;
} TCircularMoveExtras;

typedef struct {
  TMoveBufferEntry linear;
  TCircularMoveExtras circular;
} TMoveBufferEntryEx;

typedef union {
  uint8_t value;
  struct {
    uint8_t GlobalEnable:1;
    uint8_t NoNextMove:1;
    uint8_t MoveComplete:1;
    uint8_t LimitTrip:1;
    uint8_t LimitClear:1;
    uint8_t Reserved:2;
    uint8_t AutoLimit:1;
  } flags;
} TInterruptFlags;

typedef struct {
  uint8_t ValueType:2;
  uint8_t ValueIndex:1;
  uint8_t ValueSlice:1;
  uint16_t Value:12;
} TMoveValue;

typedef union {
  struct {
    union {
      TWireWord AsWord;
      TMoveValue AsValue;
    } ArgumentWord;
    TCommandByte CommandByte;
  } AsFields;
  uint8_t Data[3];
} TSPIBuffer;

/* Sets given bit in InterruptCauses bitmap and asserts INT# line if given bit
 * is set in InterruptFlags and global interrupts are enabled */
void SetFlagAndAssertInterrupt(uint8_t flag);
/* Called whenever the Read Last Interrupt Cause command is issued */
void ClearFlagsAndReleaseInterrupt(void);


#endif /* STEPPER_H_ */

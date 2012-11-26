/*
 * protocol.h - types pertaining to the secondary MCU wire protocol
 *
 *  Created on: Nov 25, 2012
 *      Author: csdexter
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stdint.h>


typedef struct {
  uint8_t WriteMode:1;
  uint8_t DataMode:1;
  uint8_t Command:6;
} TCommandByte;

typedef union {
  uint16_t AsWord;
  struct {
    uint8_t HighByte;
    uint8_t LowByte;
  } AsBytes;
} TWireWord;

#endif /* PROTOCOL_H_ */

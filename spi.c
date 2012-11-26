/*
 * spi.c - SPI HAL for AVR
 *
 *  Created on: Nov 18, 2012
 *      Author: csdexter
 */

#include <stdbool.h>
#include <stdint.h>

#include <avr/interrupt.h>

#include "spi.h"


/* private variables */
static volatile uint8_t *SPI_Rx_Buf;

/* public API */
void(*spi_hook)(bool);

void spi_start(volatile uint8_t *rx, volatile uint8_t *tx) {
  SPI_Rx_Buf = rx;
  if(tx) spi_write(*tx);
  else spi_write(0x00); /* Start the transaction */
}

ISR(SPI_INTERRUPT_NAME) {
  uint8_t data;

  data = spi_read(); /* Perform the read anyway and earliest to clear receive complete status */
  spi_int_ack();

  if(SPI_Rx_Buf) {
    if(spi_hook) spi_hook(SPI_HOOK_BEFORE);
    *SPI_Rx_Buf = data;
  }
  if(spi_hook) spi_hook(SPI_HOOK_AFTER);
}

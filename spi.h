/*
 * spi.h - SPI HAL for AVR
 *
 *  Created on: Oct 20, 2012
 *      Author: csdexter
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdbool.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/power.h>


#define SPI_POWER_ON false
#define SPI_POWER_OFF true
#define SPI_INT_ENABLE true
#define SPI_INT_DISABLE false
#define SPI_ON true
#define SPI_OFF false
#define SPI_MASTER true
#define SPI_SLAVE false
#define SPI_PHASE_LEADING false
#define SPI_PHASE_TRAILING true
#define SPI_HOOK_BEFORE false
#define SPI_HOOK_AFTER true

#define spi_port(bMode) \
    if(bMode == SPI_MASTER) { \
      MOSI_DDR |= _BV(MOSI_BIT); \
      SCK_DDR |= _BV(SCK_DDR); \
      MISO_DDR &= ~_BV(MISO_BIT); \
      \
    } else { \
      MISO_DDR |= _BV(MISO_BIT); \
      MOSI_DDR &= ~_BV(MOSI_BIT); \
      SCK_DDR &= ~_BV(SCK_BIT); \
    }

/* Starts SPI transaction: *tx gets sent out and *rx gets overwritten with
 * received data.
 * Call with NULL to signal you don't care about that side of the transaction.
 * NOTE: if running in master mode, this will actually start sending the data,
 *       otherwise it just prepares it for when the master will start SCK */
void spi_start(volatile uint8_t *rx, volatile uint8_t *tx);
/* Callback function, called after each byte is received, before and after the
 * byte is stored in *rx. First invocation (before) is with false, second
 * (after) true. First invocation only occurs if rx is not NULL.
 * WARNING: this will be called in interrupt context */
extern void(*spi_hook)(bool);

#if defined(__AVR_ATmega328P__) /* Native SPI interface */
  #define SPI_LSB_FIRST true
  #define SPI_MSB_FIRST false
  #define SPI_LITTLE_ENDIAN SPI_LSB_FIRST
  #define SPI_BIG_ENDIAN SPI_MSB_FIRST
  #define SPI_POLARITY_NEGATIVE true
  #define SPI_POLARITY_POSITIVE false
  #define SPI_SCK_IDLE_LOW SPI_POLARITY_POSITIVE
  #define SPI_SCK_IDLE_HIGH SPI_POLARITY_NEGATIVE
  #define SPI_SPEED_2X 0x04
  #define SPI_SPEED_4 0x00
  #define SPI_SPEED_16 _BV(SPR0)
  #define SPI_SPEED_64 _BV(SPR1)
  #define SPI_SPEED_128 (_BV(SPR1) | _BV(SPR0))

  #define spi_power(bState) \
      if(bState) power_spi_enable(); \
      else power_spi_disable()
  #define spi_interrupt(bState) SPCR &= ~_BV(SPIE) | (bState << SPIE)
  #define spi_enable(bState) SPCR &= ~_BV(SPE) | (bState << SPE)
  #define spi_endianness(bState) SPCR &= ~_BV(DORD) | (bState << DORD)
  #define spi_mode(bState) SPCR &= ~_BV(MSTR) | (bState << MSTR)
  #define spi_polarity(bState) SPCR &= ~_BV(CPOL) | (bState << CPOL)
  #define spi_phase(bState) SPCR &= ~_BV(CPHA) | (bState << CPHA)
  #define spi_speed(ucFlags) \
      SPCR &= ~(_BV(SPR1) | _BV(SPR0)) | (ucFlags & (_BV(SPR1) | _BV(SPR0))); \
      SPSR &= ~(_BV(SPI2X)) | (((bool)(ucFlags & SPI_SPEED_2X)) << SPI2X)
  #define spi_read() SPDR
  #define spi_write(ucValue) SPDR = ucValue
  #define spi_int_ack() /* the native SPI module auto-acks on interrupt */

  #define spi_configure(bInterrupt, bEnable, bEndian, bMode, bPolarity, bPhase, \
      ucSpeed) \
      spi_power(!bEnable); \
      spi_port(bMode); \
      SPCR = (bInterrupt << SPIE) | (bEnable << SPE) | (bEndian << DORD) | \
          (bMode << MSTR) | (bPolarity << CPOL) | (bPhase << CPHA) | \
          (ucSpeed & (_BV(SPR1) | _BV(SPR0))); \
      SPSR &= ~(_BV(SPI2X)) | (((bool)(ucSpeed & SPI_SPEED_2X)) << SPI2X)

  #define SPI_INTERRUPT_NAME SPI_STC_vect
#elif defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny2313A__) /* USI in SPI mode */
  /* avr-libc is lame enough to not have an orthogonal implementation of these constants */
  #if defined(__AVR_ATtiny84__)
    #define MOSI_DDR DDRA
    #define MISO_DDR DDRA
    #define SCK_DDR DDRA
    #define MOSI_BIT PORTA6
    #define MISO_BIT PORTA5
    #define SCK_BIT PORTA4
  #elif defined(__AVR_ATtiny2313A__)
    #define USIBR _SFR_IO8(0x00)
    #define power_usi_enable() (PRR &= ~_BV(PRUSI))
    #define power_usi_disable() (PRR |= _BV(PRUSI))
  #endif

  #define spi_power(bState) \
      if(bState) power_usi_enable(); \
      else power_usi_disable()
  #define spi_interrupt(bState) USICR &= ~_BV(USIOIE) | (bState << USIOIE)
  #define spi_enable(bState) USICR &= ~(_BV(USIWM0) | _BV(USIWM1)) | (bState << USIWM0)
  #define spi_endianness(bState) /* tiny84 is MSB first only */
  #define spi_mode(bState) USICR &= ~(_BV(USICS0) | _BV(USICS1)) | ((bState << USICS0) | (!bState << USICS1))
  #define spi_polarity(bState) /* tiny84 does CPOL = 0 only */
  #define spi_phase(bState) USICR &= ~_BV(USICS0) | (bState << USICS0)
  #define spi_speed(ucFlags) /* USI has no internal clock */
  #define spi_read() USIBR
  #define spi_write(ucValue) USIDR = ucValue
  #define spi_int_ack() USISR = _BV(USIOIF)

  #define spi_configure(bInterrupt, bEnable, bEndian, bMode, bPolarity, bPhase, \
      ucSpeed) \
      spi_power(!bEnable); \
      spi_port(bMode); \
      USICR = (bInterrupt << USIOIE) | (bEnable << USIWM0) | \
          ((bMode << USICS0) | (!bMode << USICS1)) | (bPhase << USICS0);

  #if defined(__AVR_ATtiny84__)
    #define SPI_INTERRUPT_NAME USI_OVF_vect
  #elif defined(__AVR_ATtiny2313A__)
    #define SPI_INTERRUPT_NAME USI_OVERFLOW_vect
  #endif
#endif

#endif /* SPI_H_ */

#ifndef _MGPIO_466_H_INCLUDED_
#define _MGPIO_466_H_INCLUDED_

#include <stdint.h>

/*
** this is the address of that various registers in the GPIO expander.
** Note that the actual address depends on the setting on the BANK bit.
** read how to set the bank bit and note the changes it makes in use of
** device.
*/
#define LED_REG 0x01
#define SW_REG 0x02
#define INT_REG 0x03

void    gpioInit(void);
void    writeByte(uint8_t address, uint8_t byte);
uint8_t readByte(uint8_t address);


#endif // _MGPIO_466_H_INCLUDED_

#ifndef _SPI_466_H_INCLUDED_
#define _SPI_466_H_INCLUDED_
#include <stdint.h>

#define LOW (0)
#define HIGH (1)

#ifdef FEATURE_BITBANG_SPI
#define CS_PIN   16  /* GPIO17 */
#define CLK_PIN  17  /* GPIO18 */
#define MOSI_PIN 18  /* GPIO19 */
#define MISO_PIN 19  /* GPIO16 */
#endif


void    regInit(void);
uint8_t transfer( uint8_t outData);


#endif  // _SPI_466_H_INCLUDED_

#include <stdint.h>

/*
** Okay. This header is called `pinout.h`, and that's not actually true.
** What's actually true is that this header file is the shared definitions
** that the master and slave pico's will use.
** I directly connect the same GPIO pins from master to slave, just with 
** opposite directionality. For me, it's easier, and the pins are defined
** here as for consistancy, but so too will be functions.
*/

#define READ_NIB        0xA0
#define WRITE_NIB       0xB0
#define WATCHDOG_NIB    0xC0
#define LED_REG_ADDR    0x01
#define SW_REG_ADDR     0x02
#define INT_REG_ADDR    0x03

#define LED_RED         0x01
#define LED_GREEN       0x02
#define LED_BLUE        0x04

#define SW1_ST          0x01
#define SW2_ST          0x02
#define SW1_INTR        0x04
#define SW2_INTR        0x08

#define LOW (0)
#define HIGH (1)

void    gpioInit(void);

/* ******************************************************************
    THIS IS THE PICO SPECIFIC SECTION. IT'S FOR THE PINS I USE
****************************************************************** */
/* ALL ONE SIDE OF PICO BOARD */
const uint8_t CS        = 16;   /* PICO PIN 21 */
const uint8_t CLK       = 17;   /* PICO PIN 22 */
//            GND                  PICO PIN 23      - DON'T USE
const uint8_t SO        = 19;   /* PICO PIN 25 */
const uint8_t SI        = 18;   /* PICO PIN 24 */
const uint8_t WATCHDOG  = 22;   /* PICO PIN 29 */
const uint8_t LED       = 25;   /*                  - NOT A PIN (SORT OF) */
//            RUN                  PICO PIN 30      - CAN'T USE
const uint8_t INTR      = 26;   /* PICO PIN 31 */

/* ALL OTHER SIDE OF PICO BOARD */
/* These switches are used only be slave pico */
const uint8_t SW1       = 14;   /* PICO PIN 19 */
const uint8_t SW2       = 15;   /* PICO PIN 20 */
//            GND               /* PICO PIN 18 */
const uint8_t LEDR_PIN  = 11;   /* PICO PIN 15 */
const uint8_t LEDG_PIN  = 12;   /* PICO PIN 16 */
const uint8_t LEDB_PIN  = 13;   /* PICO PIN 17 */
/* ******************************************************************
    END OF PICO PIN SECTION
****************************************************************** */


/* QUEUE EDGE DECTECTION - SPECIFIED BY MILER */
const uint8_t CS_HIGH   = 0x00;
const uint8_t CS_LOW    = 0x01;
/* Where is 0x02? */
const uint8_t CLK_HIGH  = 0x03;
const uint8_t CLK_LOW   = 0x04;

void pinTest() {
    printf("CS = %d\tCLK = %d\tMOSI = %d\n", gpio_get(CS), \
                                             gpio_get(CLK), \
                                             gpio_get(SI));
}

//void    regInit(void);

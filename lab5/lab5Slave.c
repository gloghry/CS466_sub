/**
 * @brief CS466 Lab4 SPI Bit-Bang
 * 
 * Copyright (c) 2022 Washington State University.
 */

#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "mGpio.h"
#include "mSpi.h"
#include "myAssert.h"

const uint8_t LED_PIN = 25;
const uint8_t SW1_PIN = 14;
const uint8_t SW2_PIN = 15;

const uint8_t INT_B = 13;

#define DEBOUNCE_TIMER 1


static volatile uint32_t lastTime = 0;
static volatile bool lastState = true;


static SemaphoreHandle_t _intBtn = NULL;

/* Core functions that put/get values to/from their respective pins */
void setMOSI(bool val) { gpio_put(MOSI_PIN, val); }
void setSCK(bool val) { gpio_put(CLK_PIN, val); }
uint8_t getMISO(void) { return gpio_get(MISO_PIN); }
void setCS(bool val) { gpio_put(CS_PIN, val); }

static bool debounce(uint gpio) {
    uint32_t time = to_ms_since_boot(get_absolute_time());
    bool state = gpio_get(gpio);

    if (state != lastState) {
        lastTime = time;
        lastState = state;
        return false;
    } else if ((time - lastTime) > DEBOUNCE_TIMER) {
        lastTime = time;
        return true;
    } else return false;
}

void interrupt_from_b(uint gpio, uint32_t event) {
    if (debounce(gpio)) xSemaphoreGiveFromISR(_intBtn, NULL);
}

void int_handler(void *notUsed) {
    while (true) {
        xSemaphoreTake(_intBtn, portMAX_DELAY);
        printf("Interrupt triggered by expander...\tClearling interrupt line\n");
        mGpioWriteByte(INTFB, 0x01);
    }
}


/* INIT THE **PICO** GPIO PINS AND SET THEIR CORRECT DIR
   I KEPT GETTING THIS MIXED UP WITH EXPANDER GPIO */
void mSpiInit(void) {
    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);

    gpio_init(CLK_PIN);
    gpio_set_dir(CLK_PIN, GPIO_OUT);

    gpio_init(MOSI_PIN);
    gpio_set_dir(MOSI_PIN, GPIO_OUT);

    gpio_init(MISO_PIN);
    gpio_set_dir(MISO_PIN, GPIO_IN);

    gpio_init(INT_B);
    gpio_pull_up(INT_B);
    gpio_set_dir(INT_B, GPIO_IN);
    gpio_set_irq_enabled_with_callback(INT_B, GPIO_IRQ_EDGE_FALL, true, &interrupt_from_b);

    setCS(HIGH);

}

/* INIT THE DIR OF GPIO A AND B FOR THE EXPANDER
   AND CHECK VALUE SET */
void mGpioInit(void) {
    uint8_t defvalA = mGpioReadByte(IODIRA); 
    uint8_t defvalB = mGpioReadByte(IODIRB);

    uint8_t setvalA = 0x55, setvalB = 0x56;

    mGpioWriteByte(IODIRA, setvalA);
    mGpioWriteByte(IODIRB, setvalB);
    uint8_t retvalA = mGpioReadByte(IODIRA);
    uint8_t retvalB = mGpioReadByte(IODIRB);

    myAssert(defvalA == 0xff);
    myAssert(defvalB == 0xff);
    myAssert(setvalA == retvalA);
    myAssert(setvalB == retvalB);

    mGpioWriteByte(IODIRA, 0x00);
    mGpioWriteByte(IODIRB, 0x08);

    mGpioWriteByte(GPINTENB, 0x08);
    mGpioWriteByte(IOCONB, 0x02);


    mGpioWriteByte(GPPUB, 0x08);
    mGpioWriteByte(DEFVALB, 0x08);
    mGpioWriteByte(INTCONB, 0x08);
    

    sleep_ms(1);

//    printf("val0: 0x%02x\nval1: 0x%02x\n", val0, val1);
}

/* CODE PROVIDED BY MILLER AS PART OF
   SPI Interfacing document. Copied from there,
   small change from 'out' to outData' to match prototype */
uint8_t mSpiTransfer(uint8_t outData) {
    uint8_t count, in = 0;
/*    
    gpio_put(CS_PIN, 1);
    gpio_put(CLK_PIN, 1);
    gpio_put(MOSI_PIN, 1);
    printf("%d\t%d\t%d\t%d\n", gpio_get(CS_PIN), gpio_get(CLK_PIN), gpio_get(MOSI_PIN), gpio_get(MISO_PIN));
*/

    setSCK(LOW);
    for (count = 0; count < 8; count++) {
        in <<= 1;
        setMOSI(outData & 0x80);
        sleep_us(1);
        //printf("%d\t%d\t%d\t%d\n", gpio_get(CS_PIN), gpio_get(CLK_PIN), gpio_get(MOSI_PIN), gpio_get(MISO_PIN));
        setSCK(HIGH);
        sleep_us(1);
        in += getMISO();
        sleep_us(1);
        setSCK(LOW);
        outData <<= 1;
    }
    setMOSI(0);
    sleep_us(1);

    return (in);
}

void mGpioWriteByte(uint8_t addr, uint8_t byte) {
    uint8_t preWrite = 0x40;        // OK THE READ/WRITE VAL
                                    // TOOK ME FOREVER TO FIND.
                                    // 010000 {0:1} 0 - write
                                    //              1 - read
    setCS(LOW);                     // indicate transfer
    mSpiTransfer(preWrite);         // indicate write - see above
    mSpiTransfer(addr);             // indicate address
    mSpiTransfer(byte);             // transfer byte
    setCS(HIGH);                    // indicate transfer complete
    sleep_ms(5);

    return;
}

uint8_t mGpioReadByte(uint8_t addr) {
    uint8_t val, preRead = 0x41;
    
    setCS(LOW);
    mSpiTransfer(preRead);
    mSpiTransfer(addr);
    val = mSpiTransfer(0);
    setCS(HIGH);

    return (val);
}


void hardware_init(void)
{
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    mSpiInit();
    mGpioInit();
}
//
// gpioVerifyReadWrite()
//
// This is the main function of a task that I'm using to verify that 
// my GPIO and SPI functionality is working correctly.  It will be retired 
// as I move on to actual GPIO-Expander Functionality.
//
void gpioVerifyReadWrite(void * notUsed)
{   
    const uint32_t queryDelayMs = 500;  // ms
    uint8_t regValue;
    uint8_t count=0;

    vTaskDelay(5000);

    while (true) 
    {
        mGpioWriteByte(IODIRB, count++);
        regValue = mGpioReadByte(IODIRB);
        printf("IODIRB: 0x%02x,  ", regValue);

        regValue = mGpioReadByte(IODIRA);
        printf("IODIRA: 0x%02x,  ", regValue);

        regValue = mGpioReadByte(IPOLA);
        printf("IPOLA: 0x%02x\n", regValue);

        vTaskDelay(queryDelayMs);
    }
}

// IODIRB: 0x00,  IODIRA: 0xff,  IPOLA: 0x00
// IODIRB: 0x01,  IODIRA: 0xff,  IPOLA: 0x00
// IODIRB: 0x02,  IODIRA: 0xff,  IPOLA: 0x00
// IODIRB: 0x03,  IODIRA: 0xff,  IPOLA: 0x00
// IODIRB: 0x04,  IODIRA: 0xff,  IPOLA: 0x00
// IODIRB: 0x05,  IODIRA: 0xff,  IPOLA: 0x00
// IODIRB: 0x06,  IODIRA: 0xff,  IPOLA: 0x00
// etc....

void ledCtrl(uint8_t val) {
    if((mGpioReadByte(GPIOB) & 0x08) == LOW) {    
        mGpioWriteByte(GPIOA, HIGH);
    } else {
        mGpioWriteByte(GPIOA, val);
    }
}
    

void heartbeat(void * notUsed)
{   
    const uint32_t heartbeatDelay = 200;  // ms

    while (true) 
    {
        
        gpio_put(LED_PIN, 1);
        /*
        gpio_put(CS_PIN, 1);
        gpio_put(CLK_PIN, 1);
        gpio_put(MOSI_PIN, 1);
        */
        ledCtrl(0xFF); 
        vTaskDelay(heartbeatDelay);
        gpio_put(LED_PIN, 0);
        ledCtrl(0x00);
        /*
        gpio_put(CS_PIN, 0);
        gpio_put(CLK_PIN, 0);
        gpio_put(MOSI_PIN, 0);
        */
        vTaskDelay(heartbeatDelay);

        printf("lab4 Tick\n");
    }
}

int main()
{
    stdio_init_all();
    printf("lab4 Hello!\n");

    hardware_init();

    _intBtn = xSemaphoreCreateBinary();

    xTaskCreate(heartbeat, "LED_Task", 256, NULL, 1, NULL);
    //xTaskCreate(gpioVerifyReadWrite, "GPIO_Task", 256, NULL, 2, NULL); 
    xTaskCreate(int_handler, "Interrupt Handler", 256, NULL, 2, NULL);

    vTaskStartScheduler();

    while(1){};
}

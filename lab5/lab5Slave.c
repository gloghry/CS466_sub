/**
 * @brief CS466 Lab5 SLAVE VER
 * 
 * Copyright (c) 2022 Washington State University.
 */

#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "pinout.h"
#include "myAssert.h"

#define DEBUG 1

volatile uint8_t LED_REG;
volatile uint8_t SW_REG;
volatile uint8_t INT_REG;
volatile bool transferState = false;

QueueHandle_t queue= NULL;

// PINS INCLUDE SO, SI, CLK, CS, SW1, SW2
// NO XX_PIN STYLE NAMING - ONLY CORE NAME

/* Core functions that put/get values to/from their respective pins */
/* PINS DEFINED IN "pinout.h" */
uint8_t getMOSI(void) { return gpio_get(SI); }
uint8_t getCLK(void) { return gpio_get(CLK); }
void setMISO(bool val) { gpio_put(SO, val); }
uint8_t getCS(void) { return gpio_get(CS); }

void setLED_REG(uint8_t val) { LED_REG = val; }
void setSW_REG(uint8_t val) { SW_REG = val; }
void setINT_REG(uint8_t val) { INT_REG = val; }

uint8_t getLED_REG(void) { return LED_REG; }
uint8_t getSW_REG(void) { return SW_REG; }
uint8_t getINT_REG(void) { return INT_REG; }


/* I don't have a single RGB, so we use 3 pico pins for R, G, B */
void setRED(bool val) { gpio_put(LEDR_PIN, val); }
void setGREEN(bool val) { gpio_put(LEDG_PIN, val); }
void setBLUE(bool val) { gpio_put(LEDB_PIN, val); }


void externRGB(uint8_t slct) {
    if (slct & LED_RED) setRED(HIGH);
    else setRED(LOW);

    if (slct & LED_GREEN) setGREEN(HIGH);
    else setGREEN(LOW);

    if (slct & LED_BLUE) setBLUE(HIGH);
    else setBLUE(LOW);
}

void writeREG(uint8_t REG, uint8_t slct) {
    REG &= 0x0F;                    // Mask upper 4 bits, lower 4 bits is addr
                                    // Upper 4 bits indicate read/write
    slct &= 0x0F;                   // whole byte could be used
                                    // But all that's ever used is a nibble
    if (REG & LED_REG_ADDR) {
        setLED_REG(slct);
        externRGB(slct);
    } else if (REG & SW_REG_ADDR) setSW_REG(slct); 
    else if (REG & INT_REG_ADDR) setINT_REG(slct);
    else printf("Not a valid register from master\n");
}

uint8_t readREG(uint8_t REG) {
    REG &= 0x0F;
    if (REG & LED_REG_ADDR) return (getLED_REG()) ;
    else if (REG & SW_REG_ADDR) return (getSW_REG());
    else if (REG & INT_REG_ADDR) return (getINT_REG());
    else {
        printf("Not a valid register from master\n");
        return 0;
    }
}
 

/* DESIGNED TO BE AN INTERNAL DEBUGGING FOR RGB
   PIN WRITES, AS WELL AS SETTING THE REGISTER.
   THIS PROGRAM IS *VERY* SLOW, AND SHOULD NOT NORMALLY
   BE CALLED, UNLESS IN A DEBUGGING STATE. */
static void REGWriteFlashTest(void) {
    sleep_ms(1000);
    writeREG(LED_REG_ADDR, LED_RED);
    sleep_ms(100);
    myAssert(LED_RED == readREG(LED_REG_ADDR));
    printf("LED REG shows 0x%02x\n", readREG(LED_REG_ADDR));

    writeREG(LED_REG_ADDR, LED_GREEN);
    sleep_ms(100);
    myAssert(LED_GREEN == readREG(LED_REG_ADDR));
    printf("LED REG shows 0x%02x\n", readREG(LED_REG_ADDR));

    writeREG(LED_REG_ADDR, LED_BLUE);
    sleep_ms(100);
    myAssert(LED_BLUE == readREG(LED_REG_ADDR));
    printf("LED REG shows 0x%02x\n", readREG(LED_REG_ADDR));

    writeREG(LED_REG_ADDR, 0);
    sleep_ms(100);
    myAssert(0x00 == readREG(LED_REG_ADDR));
    printf("LED REG shows 0x%02x\n", readREG(LED_REG_ADDR));

    for (int i = 0; i < 3; i++) {
        writeREG(LED_REG_ADDR, (LED_RED | LED_GREEN | LED_BLUE));
        sleep_ms(100);
        writeREG(LED_REG_ADDR, 0);
        sleep_ms(100);
    }
}

void int_handler(uint gpio, uint32_t event) {
    uint8_t inter;
    if (gpio == CS) {
        if (event == GPIO_IRQ_EDGE_RISE) {
            inter = CS_HIGH;
            xQueueSendFromISR(queue, (void *) &inter, 0);
        }
        if (event == GPIO_IRQ_EDGE_FALL) {
            inter = CS_LOW;
            xQueueSendFromISR(queue, (void *) &inter, 0);
        }
    }
    if (gpio == CLK) {
        if (event == GPIO_IRQ_EDGE_RISE) {
            inter = CLK_HIGH;
            xQueueSendFromISR(queue, (void *) &inter, 0);
        }
        if (event == GPIO_IRQ_EDGE_FALL) {
            inter = CLK_LOW;
            xQueueSendFromISR(queue, (void *) &inter, 0);
        }
    }
}

volatile uint8_t count, bitBuff, addrBuff, dataBuff, readWriteBuff;
volatile bool firstByte = true;
void queue_handler(void *empty) {
    uint8_t buff;
    while(1) {
        xQueueReceive(queue, &buff, portMAX_DELAY);
        
        switch(buff) {
            case CS_HIGH:
                if (DEBUG) printf("Chip select high incoming\n");
                /* INDICATE END OF TRANSFER OF BITS */
                transferState = false;
                count = bitBuff = addrBuff = dataBuff = readWriteBuff= 0;
                break;

            case CS_LOW:
                if (DEBUG) printf("Chip select low incoming\n");
                /* INDICATE START OF TRANSFER OF BITS */
                transferState = true;
                count = bitBuff = addrBuff = dataBuff = 0;
                break;

            case CLK_HIGH:
                if (DEBUG) printf("High clock tick\n");
                /* CHECK TO SEE IF IN A TRANSFER STATE OR NOT
                   WE MAY BE WRITTEN TO INCORRECTLY
                   HIGH CLOCK TICK WE WRITE A BIT */
                if (!transferState) printf("Err\n");
                break;

            case CLK_LOW:
                if (DEBUG) printf("Low clock tick\n");
                /* CHECK TO SEE IF IN A TRANSFER STATE OR NOT
                   WE MAY BE WRITTEN TO INCORRECTLY
                   LOW CLOCK TICK WE READ A BIT */
                count++;
                if (firstByte) {
                    if (count == 7) {
                        readWriteBuff = (0xF0 & bitBuff);
                        addrBuff = (0x0F & bitBuff);
                        count = 0;
                        firstByte = false;
                    }
                 } else {
                    if (count == 7) {
                        dataBuff = (0x0F & bitBuff);
                        if (readWriteBuff & WRITE_NIB) writeREG(addrBuff, dataBuff);
                    }
                }
                break;

            default:
                printf("Err\n");
                break;
        }
    }
}

/* init is opposite to master
   IN   CS, CLK, SI
   OUT  SO, WATCHDOG, INTR
*/ 
void gpioInit(void) {
    gpio_init(CS);
    gpio_init(CLK);
    gpio_init(SI);
    gpio_init(SO);
    gpio_init(WATCHDOG);
    gpio_init(INTR);
    gpio_init(LEDR_PIN);
    gpio_init(LEDG_PIN);
    gpio_init(LEDB_PIN);

    /* OUTPUT PINS */
    gpio_set_dir(SO, GPIO_OUT);         // MASTER IN, SLAVE OUT
    gpio_set_dir(WATCHDOG, GPIO_OUT);   // WATCHDOG TRIGGERED BY SLAVE
    gpio_set_dir(INTR, GPIO_OUT);       // INTERRUPT TRIGGERED BY SLAVE

    gpio_set_dir(LEDR_PIN, GPIO_OUT);
    gpio_set_dir(LEDG_PIN, GPIO_OUT);
    gpio_set_dir(LEDB_PIN, GPIO_OUT);

    /* INPUT PINS */
    gpio_set_dir(SI, GPIO_IN);          // MASTER OUT, SLAVE IN
    gpio_set_dir(CS, GPIO_IN);          // CHIP SELECT CNTRLD BY MASTER
    gpio_set_dir(CLK, GPIO_IN);         // CLK CNTRLED BY MASTER

    gpio_pull_down(CS);
    gpio_pull_down(CLK);
}


void hardware_init(void) {
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    gpioInit();

    if (DEBUG) REGWriteFlashTest();
}


void heartbeat(void * notUsed) {   
    const uint32_t heartbeatDelay = 1000;  // ms

    while (true) 
    {
        gpio_put(LED, 1);
        pinTest();
        vTaskDelay(heartbeatDelay);

        gpio_put(LED, 0);
        pinTest();
        vTaskDelay(heartbeatDelay);

        printf("lab5 Slave Tick\n");
    }
}


int main() {
    stdio_init_all();
    printf("lab5 Hello!\n");

    hardware_init();

    queue = xQueueCreate(100, sizeof( uint8_t ));
    if (DEBUG) printf("Queue pointer: %p\n", queue);


    xTaskCreate(heartbeat, "LED_Task", 256, NULL, 1, NULL);
    // xTaskCreate(int_handler, "Interrupt Handler", 256, NULL, 2, NULL);

    xTaskCreate(queue_handler, "QUEUE_HANDLER", 256, NULL, 2, NULL);

    /* IMPORTANT TO MOVE THIS FROM GPIOINIT FUNC
       THESE WERE FIRING BEFORE THE QUEUE WAS PROPERLY MADE
       CAUSING WEIRD BEHAVIOR. MOVED HERE AFTER QUEUE CREATION,
       BEFORE STARTSCHEDULER(); */
    gpio_set_irq_enabled_with_callback(CS, (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE), true, &int_handler);
    gpio_set_irq_enabled_with_callback(CLK, (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE), true, &int_handler);

    vTaskStartScheduler();

    while(1){};
}


/*
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
*/


//
// gpioVerifyReadWrite()
//
// This is the main function of a task that I'm using to verify that 
// my GPIO and SPI functionality is working correctly.  It will be retired 
// as I move on to actual GPIO-Expander Functionality.
//
/*
void gpioVerifyReadWrite(void * notUsed) {   
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
*/
// IODIRB: 0x00,  IODIRA: 0xff,  IPOLA: 0x00
// IODIRB: 0x01,  IODIRA: 0xff,  IPOLA: 0x00
// IODIRB: 0x02,  IODIRA: 0xff,  IPOLA: 0x00
// IODIRB: 0x03,  IODIRA: 0xff,  IPOLA: 0x00
// IODIRB: 0x04,  IODIRA: 0xff,  IPOLA: 0x00
// IODIRB: 0x05,  IODIRA: 0xff,  IPOLA: 0x00
// IODIRB: 0x06,  IODIRA: 0xff,  IPOLA: 0x00
// etc....
/*
void ledCtrl(uint8_t val) {
    if((mGpioReadByte(GPIOB) & 0x08) == LOW) {    
        mGpioWriteByte(GPIOA, HIGH);
    } else {
        mGpioWriteByte(GPIOA, val);
    }
}
*/

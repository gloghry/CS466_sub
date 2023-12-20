/**
 * @brief CS466 Lab5 MASTER VER
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

// PINS INCLUDE SO, SI, CLK, CS, SW1, SW2
// NO XX_PIN STYLE NAMING - ONLY CORE NAME

/* Core functions that put/get values to/from their respective pins */
void setCS(bool val) { gpio_put(CS, val); }
void setCLK(bool val) { gpio_put(CLK, val); }
void setMOSI(bool val) { gpio_put(SI, val); }
uint8_t getMISO(void) { return gpio_get(SO); }


/* INIT PICO PINS INTO GPIO */
void gpioInit(void) {
    gpio_init(CS);
    gpio_init(CLK);
    gpio_init(SI);
    gpio_init(SO);
    gpio_init(WATCHDOG);
    gpio_init(INTR);

    /* OUTPUT PINS */
    gpio_set_dir(SI, GPIO_OUT);         // MASTER OUT, SLAVE IN
    gpio_set_dir(CS, GPIO_OUT);         // CHIP SELECT CNTRLD BY MASTER
    gpio_set_dir(CLK, GPIO_OUT);        // CLK CNTRLD BY MASTER

    /* INPUT PINS */
    gpio_set_dir(SO, GPIO_IN);          // MASTER IN, SLAVE OUT
    gpio_set_dir(WATCHDOG, GPIO_IN);    // WATCHDOG TRIGGERED TO MASTER
    gpio_set_dir(INTR, GPIO_IN);        // INTERRUPT SIGNAL FROM SLAVE

    gpio_pull_down(WATCHDOG);
    gpio_pull_down(INTR);
}

void hardware_init(void) {
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    gpioInit();
}

/* masterTransfer should work the same as
   lab4. We will still use CS trigger to signal
   transfer. Will have to play with CLK. */
uint8_t masterTransfer(uint8_t outData) { 
    uint8_t count, in = 0;

    setCLK(LOW);
    for (count = 0; count < 8; count++) {
        in <<= 1;
        setMOSI(outData & 0x80);
        sleep_us(1);
        setCLK(HIGH);
        sleep_us(1);
        in += getMISO();
        sleep_us(1);
        setCLK(LOW);
        outData <<= 1;
    }
    setMOSI(0);
    sleep_us(1);

    return (in);
}

void masterWriteByte(uint8_t addr, uint8_t byte) {
                                    // 0xA0 for read
                                    // 0xB0 for write
    addr &= 0x0F;                   // Mask out top nibble
    addr |= WRITE_NIB;              // and substitute my own
    setCS(LOW);                     // indicate transfer
    masterTransfer(addr);           // transfer addr includes write nibble
                                    // and addr nibble
    masterTransfer(byte);           // transfer byte
    setCS(HIGH);                    // indicate transfer complete
    sleep_ms(5);

    return;
}

uint8_t masterReadByte(uint8_t addr) {
    uint8_t val;                    // top nibble for read write
                                    // 0xA0 for read
                                    // 0xB0 for write
    addr &= 0x0F;                   // Mask out top nibble
    addr |= READ_NIB;               // and substitue my own
    setCS(LOW);                     // indicate transfer
    masterTransfer(addr);           // transfer addr includes read nibble
    val = masterTransfer(0);        // Send nothing, get something
    setCS(HIGH);                    // indicate transfer complete

    return (val);
}

void masterTestWatchdog(void) {
    setCS(LOW);
    masterWriteByte(WATCHDOG_NIB, 0);
    setCS(HIGH);

    return;    
}

void setOutputPinsHigh(void) {
    setCS(HIGH);
    sleep_ms(1);
    setCLK(HIGH);
    sleep_ms(1);
    setMOSI(HIGH);
    sleep_ms(1);
}

void setOutputPinsLow(void) {
    setCS(LOW);
    sleep_ms(1);
    setCLK(LOW);
    sleep_ms(1);
    setMOSI(LOW);
    sleep_ms(1);
}

void writeRED(void) { masterWriteByte(LED_REG_ADDR, LED_RED); }

void heartbeat(void * notUsed)
{   
    const uint32_t heartbeatDelay = 1000;  // ms

    while (true) 
    {
        gpio_put(LED, 1);
        writeRED();
        pinTest();
        vTaskDelay(heartbeatDelay);
        gpio_put(LED, 0);
        pinTest();
        vTaskDelay(heartbeatDelay);

        printf("lab5 Master Tick\n");
    }
}

int main() {
    stdio_init_all();
    printf("lab5 Hello!\n");

    hardware_init();

    xTaskCreate(heartbeat, "LED_Task", 256, NULL, 1, NULL);

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

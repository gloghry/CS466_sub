/**
 * @brief CS466 Lab1 Blink proigram based on pico blink example
 * 
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <time.h>
#include <myAssert.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

// used for me internally to test w printf
#define DEBUG 0
enum task{consumer_t, producer1_t, producer2_t};  // I will forget this, probably

typedef struct {
    int priority;
    char *name;
    QueueHandle_t theQueue;
} property_t;

const uint8_t LED_PIN = 25;
const uint8_t SW1_PIN = 17;
const uint8_t SW2_PIN = 16;

/* invert GPIO switches, we pullup */
bool sw1_press(void) {
    return !gpio_get(SW1_PIN);
}

bool sw2_press(void) {
    return !gpio_get(SW2_PIN);
}

uint32_t heartbeatDelay = 500;  // ms
uint32_t tickCount = 0;

TaskHandle_t hb_task = NULL;

void hardware_init(void) {
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(SW1_PIN);
    gpio_pull_up(SW1_PIN);
    gpio_set_dir(SW1_PIN, GPIO_IN);

    gpio_init(SW2_PIN);
    gpio_pull_up(SW2_PIN);
    gpio_set_dir(SW2_PIN, GPIO_IN);
}

/* Core Heartbeat Function.
   50% duty cycle.
   in ms 
   Moved from outside of heartbeat()
   because others may want to use it */
void hb_ms(uint32_t ms) {
    gpio_put(LED_PIN, 1);
    vTaskDelay(ms);
    gpio_put(LED_PIN, 0);
    vTaskDelay(ms);
}

void hb_hz(uint32_t hz) {
    uint32_t ms = (1000/hz);
    ms /= 2;
    hb_ms(ms);
}

void heartbeat(void *notUsed) {   
    while (true) {
        hb_ms(heartbeatDelay);
        tickCount++;
        printf("tick: %d,\tSwitch 1 is %s,\tSwitch 2 is %s\n", \
            tickCount, \
            gpio_get(SW1_PIN)?"NOT PRESSED":"PRESSED", \
            gpio_get(SW2_PIN)?"NOT PRESSED":"PRESSED");
        //assert(tickCount < 5); // only used in part 5, bit of a pain otherwise
        //myAssert(tickCount < 5); // only used in part 5, bit of a pain otherwise
           
    }
}

void consumer(void *taskPointer) {
    // task pointer is the pointer given from void *
    // needs to be recast, and this is the easiest way I know
    property_t *taskProperties = (property_t*) taskPointer;
    QueueHandle_t queue = taskProperties[consumer_t].theQueue;
    uint32_t buff;  // buff doesn't do anything other than signal
    while(1) {
        xQueueReceive(queue, &buff, portMAX_DELAY);
        if (DEBUG) printf("Message received from %d\n", buff);
        if (sw1_press() && buff == producer1_t) { /* BUTTONS HELD FOR TOO LONG WILL CAUSE THE QUEUE
                              TO FILL AND ASSERT WILL TRIGGER.
                              THIS HAPPENS AFTER ~2s WITH CURRENT SETUP. */
            if (DEBUG) printf("SW1 pressed, message from p1 - flashing\n");
            for (int i = 0; i < 5; i++) hb_ms(25);
        }
        if (sw2_press() && buff == producer2_t) { /* BUTTONS HELD FOR TOO LONG WILL CAUSE THE QUEUE
                              TO FILL AND ASSERT WILL TRIGGER.
                              THIS HAPPENS AFTER ~2s WITH CURRENT SETUP. */
            if (DEBUG) printf("SW1 pressed, message from p1 - flashing\n");
            for (int i = 0; i < 4; i++) hb_ms(35);
        }
    }
}

void producer1(void *taskPointer) {
    property_t *taskProperties = (property_t*) taskPointer;
    QueueHandle_t queue = taskProperties[consumer_t].theQueue;
    uint32_t buff = producer1_t, wait;
    while(1) {
        wait = (rand() % 10) * 10;
        vTaskDelay(wait);
        xQueueSend(queue, (void*) &buff, 0);
        if (DEBUG) printf("Message sent\n");
        myAssert(uxQueueSpacesAvailable(queue) != 0);

/* Section for producer flashing is for my testing
        if (sw2_press()) {      // ASSERT WON'T TRIGGER HERE,
                                // BUT THE PROGRAM WILL SLOW DOWN SENDING MESSAGES 
            if (DEBUG) printf("SW2 pressed, flashing\n");
            for (int i = 0; i < 6; i++) hb_ms(35);
        }
*/
    }
}

void producer2(void *taskPointer) {
    property_t *taskProperties = (property_t*) taskPointer;
    QueueHandle_t queue = taskProperties[consumer_t].theQueue;
    uint32_t buff = producer2_t, wait;
    while(1) {
        wait = (rand() % 10) * 10;
        vTaskDelay(wait);
        xQueueSend(queue, (void*) &buff, 0);
        if (DEBUG) printf("Message sent\n");
        myAssert(uxQueueSpacesAvailable(queue) != 0);

/* Section for producer flashing is for my testing
        if (sw2_press()) {      // ASSERT WON'T TRIGGER HERE,
                                // BUT THE PROGRAM WILL SLOW DOWN SENDING MESSAGES
            if (DEBUG) printf("SW2 pressed, flashing\n");
            for (int i = 0; i < 6; i++) hb_ms(35);
        }
*/
    }
}

int main() {
    pid_t pid = getpid();   // use pid to seed program. "more" random
    srand(pid);
    stdio_init_all();
    printf("lab3 Hello!\nRandom number seed is %d", pid);
    hardware_init();

    xTaskCreate(heartbeat, "LED_Task", 256, NULL, 0, &hb_task);
    QueueHandle_t qHandle = xQueueCreate(20, sizeof( uint32_t ));
    myAssert(qHandle != NULL);

    property_t taskProperties[] = {
        {consumer_t, "Consumer", qHandle},
        {producer1_t, "Producer1", qHandle},
        {producer2_t, "Producer", qHandle}
    };

    xTaskCreate(consumer, "Consumer", 256, (void*) taskProperties, 1, NULL);
    xTaskCreate(producer1, "Producer1", 256, (void*) taskProperties, 2, NULL);
    xTaskCreate(producer2, "Producer2", 256, (void*) taskProperties, 2, NULL);
 
    vTaskStartScheduler();

    while(1){};
}

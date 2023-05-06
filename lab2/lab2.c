/**
 * @brief CS466 Lab1 Blink proigram based on pico blink example
 * 
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <time.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

const uint8_t LED_PIN = 25;
const uint8_t SW1_PIN = 17;
/* SW2 CREATED DURING PT 7 */
const uint8_t SW2_PIN = 16;

uint32_t heartbeatDelay = 500;  // ms

static SemaphoreHandle_t _semBtn1 = NULL;
static SemaphoreHandle_t _semBtn2 = NULL;
static SemaphoreHandle_t _semBoth = NULL;

volatile bool sw1_state = false;
volatile bool sw2_state = false;

volatile absolute_time_t last_interrupt;
volatile absolute_time_t current;

TaskHandle_t hb_task = NULL;
TaskHandle_t sw1_task = NULL;
TaskHandle_t sw2_task = NULL;

void gpio_int_callback(uint gpio, uint32_t events_unused) {
    printf("switch callback: GPIO ISR %d\n", gpio);

    if (gpio == SW1_PIN) {
        /* REMOVED AS PER PART 6 IN LAB 2 */
        //heartbeatDelay /= 2;
        xSemaphoreGiveFromISR(_semBtn1, NULL);
    }

    if (gpio == SW2_PIN) {
        xSemaphoreGiveFromISR(_semBtn2, NULL);
    }

    if (heartbeatDelay < 50) heartbeatDelay = 500;
}

void hardware_init(void) {
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(SW1_PIN);
    gpio_pull_up(SW1_PIN);
    gpio_set_dir(SW1_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_int_callback);

    gpio_init(SW2_PIN);
    gpio_pull_up(SW2_PIN);
    gpio_set_dir(SW2_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(SW2_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_int_callback);

    last_interrupt = current = get_absolute_time();
}

/* Core Heartbeat Function.
   50% duty cycle.
   in ms                    */
void hb_core(uint32_t ms) {
    printf("hb-tick: %d\n", ms);
    gpio_put(LED_PIN, 1);
    vTaskDelay(ms);
    gpio_put(LED_PIN, 0);
    vTaskDelay(ms);
}

#if 1
void sw1_handler(void * notUsed) {
    while (true) {
        xSemaphoreTake( _semBtn1, portMAX_DELAY);
        printf("sw1 Semaphore taken..\n");
    }
}
#endif

/* INTERRUPT FOR SW2 CODE FOR PT 7 */
#if 1
void sw2_handler(void *notUsed) {
    while(true) {
        xSemaphoreTake(_semBtn2, portMAX_DELAY);
        printf("sw2 Semaphore taken...\n");
    }
}
#endif

/* MY NEW INTERRUPT HANDLERS 
   PART 8 IN LAB 2           */
#if 1
void new_sw1_handler(void *notUsed) {
    while(true) {
        xSemaphoreTake(_semBtn1, portMAX_DELAY);
        gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_FALL, false, &gpio_int_callback);
        last_interrupt = get_absolute_time();
        vTaskDelay(25);
        gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_int_callback);
        sw1_state = true;
        if (sw2_state) xSemaphoreGiveFromISR(_semBoth, NULL);
        uint32_t sw1_hb = 33;
        printf("sw2 Semaphore taken, hb set to %d\n", sw1_hb);
        for (int i = 0; i < 10; i++) {
            hb_core(sw1_hb);
        }
        sw1_state = false;
    }
}

void new_sw2_handler(void *notUsed) {
    while(true) {
        xSemaphoreTake(_semBtn2, portMAX_DELAY);
        gpio_set_irq_enabled_with_callback(SW2_PIN, GPIO_IRQ_EDGE_FALL, false, &gpio_int_callback);
        last_interrupt = get_absolute_time();
        vTaskDelay(25);
        gpio_set_irq_enabled_with_callback(SW2_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_int_callback);
        vTaskDelay(25);
        sw2_state = true;
        if (sw1_state) xSemaphoreGiveFromISR(_semBoth, NULL);
        uint32_t sw2_hb = 38;
        printf("sw2 Semaphore taken, hb set to %d\n", sw2_hb);
        for (int i = 0; i < 10; i++) {
            hb_core(sw2_hb);
        }
        sw2_state = false;
    }
}
#endif

void both_handler(void *notUsed) {
    uint32_t both_hb = 100;
    while(true) {
        xSemaphoreTake(_semBoth, portMAX_DELAY);
        last_interrupt = get_absolute_time();

        vTaskSuspend(hb_task);
        vTaskSuspend(sw1_task);
        vTaskSuspend(sw2_task);

        while(!gpio_get(SW1_PIN) && !gpio_get(SW2_PIN)) hb_core(both_hb);

        vTaskResume(hb_task);
        vTaskResume(sw1_task);
        vTaskResume(sw2_task);
    }
    sw2_state = false;
    sw2_state = false;
}

void remind() {
    printf("60s elapsed since last button press.\n");
    for(int i = 0; i < 50; i++) hb_core(25);
    last_interrupt = get_absolute_time();
    current = get_absolute_time();
}

void heartbeat(void * notUsed) {   
    while (true) {
        current = get_absolute_time();
        if (current - last_interrupt > (60 * 1000000)) remind();
        hb_core(heartbeatDelay);
    }
}
 

int main() {
    stdio_init_all();
    printf("lab2 Hello!\n");
    hardware_init();

    _semBtn1 = xSemaphoreCreateBinary();
    _semBtn2 = xSemaphoreCreateBinary();
    _semBoth = xSemaphoreCreateBinary();


    xTaskCreate(heartbeat, "LED_Task", 256, NULL, 1, &hb_task);
    xTaskCreate(sw1_handler, "SW1_Task", 256, NULL, 2, NULL);
    xTaskCreate(sw2_handler, "SW2_Task", 256, NULL, 2, NULL);   // task created pt 7

    xTaskCreate(new_sw1_handler, "SW1_Task", 256, NULL, 3, &sw1_task);   // task created pt 8
    xTaskCreate(new_sw2_handler, "SW2_Task", 256, NULL, 3, &sw2_task);   // task created pt 8
    
    xTaskCreate(both_handler, "Both_Task", 256, NULL, 4, NULL); // Needed to finish pt 9


    vTaskStartScheduler();

    while(1){};
}

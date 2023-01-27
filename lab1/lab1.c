/**
 * @brief CS466 Lab1 Blink proigram based on pico blink example
 * 
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

#define LED_PIN 25
#define GPIO_18 18
#define SW1 17
#define SW2 16

#include "pico/stdlib.h"

void my_gpio_init(void) {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    /* New code starts here */
    gpio_init(GPIO_18);
    gpio_set_dir(GPIO_18, GPIO_OUT);

    gpio_init(SW1);
    gpio_set_dir(SW1, GPIO_IN);
    gpio_pull_up(SW1);

    gpio_init(SW2);
    gpio_set_dir(SW2, GPIO_IN);
    gpio_pull_up(SW2);
    /* end of new code added to init */
}

void led_control(bool isOn) {
    gpio_put(LED_PIN, isOn);
    gpio_put(GPIO_18, isOn);
}

bool isPressed(uint button) { return (!gpio_get(button)); }

int main() {
    my_gpio_init();

    while (true) {
#if 0   /* ORIGINAL CODE GIVEN */
        gpio_put(LED_PIN, 1);       
        gpio_put(LED_PIN, 0);
#endif

#if 0   /* PROBLEM 10 */
        led_control(1);
        sleep_ms(100);
        led_control(0);
        sleep_ms(900);
#endif

#if 0   /* PROBLEM 11 */
        led_control(gpio_get(SW1));
        led_control(gpio_get(SW2));
#endif

#if 1   /* PROBLEM 12 */
        if (isPressed(SW1)) {
            for (int i = 0; i < 20; i++) {
                /* UNKONWN REQUIREMENT
                 * Both buttons pressed while in the middle of one loop
                 * Breaking and moving onto both to both buttons function
                 */
                if (isPressed(SW2)) break;

                /* Want 15Hz blink, best approx */
                led_control(1);
                sleep_ms(7);                // 100/15 rounded
                led_control(0);
                sleep_ms(60);               // 900/15
            }
            /* Weird logic, if switch 1 held and 
             * switch 2 not pressed do nothing.
             * Required to act as a trigger.
             * KNOWN BUG: If held for about 2s will still trigger again
             *            but if held for about 1s or 5s it won't.
             *            Not sure why.
             */
            while (isPressed(SW1) && !isPressed(SW2));
        }

        if (isPressed(SW2)) {
            for (int i = 0; i < 10; i++) {
                /* UNKONWN REQUIREMENT
                 * Both buttons pressed while in the middle of one loop
                 * Breaking and moving onto both to both buttons function
                 */
                if (isPressed(SW1)) break;

                /* Want 13Hz blink, best approx */
                led_control(1);
                sleep_ms(8);                // 100/13 rounded
                led_control(0);
                sleep_ms(69);               // 900/13
            }
            /* Weird logic, if switch 2 held and 
             * switch 1 not pressed do nothing.
             * Required to act as a trigger.
             * KNOWN BUG: If held for about 2s will still trigger again
             *            but if held for about 1s or 5s it won't.
             *            Not sure why.
             */
            while (isPressed(SW2) && !isPressed(SW1));
        }   

        while (isPressed(SW1) && isPressed(SW2)) {
            /* Want 5Hz blink when both switches pressed */
            led_control(1);
            sleep_ms(20);
            led_control(0);
            sleep_ms(180);
        }
        
        while (!isPressed(SW1) && !isPressed(SW2)) {
            /* Want 1Hz blink when no switches pressed */
            led_control(1);
            sleep_ms(5);
            led_control(0);
            sleep_ms(995);
        }
#endif
    }
}

/* board.c - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"

#include "iot_button.h"
#include "board.h"

#define TAG "BOARD"

#define BUTTON_IO_NUM           0
#define BUTTON_ACTIVE_LEVEL     0
#define LED_R GPIO_NUM_3
#define LED_G GPIO_NUM_3
#define LED_B GPIO_NUM_3


struct _led_state led_state[3] = {
    { LED_OFF, LED_OFF, LED_R, "red"   },
    { LED_OFF, LED_OFF, LED_G, "green" },  // Updated green LED pin to GPIO3
    { LED_OFF, LED_OFF, LED_B, "blue"  },
};

void board_led_operation(uint8_t pin, uint8_t onoff)
{
    for (int i = 0; i < 3; i++) {
        if (led_state[i].pin != pin) {
            continue;
        }
        if (onoff == led_state[i].previous) {
            ESP_LOGW(TAG, "led %s is already %s",
                     led_state[i].name, (onoff ? "on" : "off"));
            return;
        }
        gpio_set_level(pin, onoff);
        led_state[i].previous = onoff;
        return;
    }

    ESP_LOGE(TAG, "LED is not found!");
}

static void board_led_init(void)
{
    for (int i = 0; i < 3; i++) {
        printf("Initializing LED %d with pin %d\n", i, led_state[i].pin);
        
        gpio_reset_pin(led_state[i].pin);
        printf("Pin %d reset\n", led_state[i].pin);

        gpio_set_direction(led_state[i].pin, GPIO_MODE_DEF_OUTPUT);
        printf("Set pin %d to output\n", led_state[i].pin);

        gpio_set_level(led_state[i].pin, LED_OFF);
        printf("Set pin %d level to %d (LED_OFF)\n", led_state[i].pin, LED_OFF);

        led_state[i].previous = LED_OFF;
        printf("LED %d previous state set to %d (LED_OFF)\n", i, LED_OFF);
    }
    printf("LED initialization complete\n");
}
static void button_tap_cb(void* arg)
{
    ESP_LOGI(TAG, "Button was tapped: %s", (char *)arg);
    // You can add any other functionality you need here
    printf("Button tap callback invoked.\n");
}

static void board_button_init(void)
{
    button_handle_t btn_handle = iot_button_create(BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, button_tap_cb, "RELEASE");
    }
}

void board_init(void)
{
    board_led_init();
    board_button_init();
}

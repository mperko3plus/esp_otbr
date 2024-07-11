#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "example";

#define BLINK_GPIO CONFIG_BLINK_GPIO

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void set_red_led(void)
{
    /* Set the LED pixel using RGB values (255, 0, 0) for red color */
    led_strip_set_pixel(led_strip, 0, 255, 0, 0);
    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1,
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    led_strip_clear(led_strip);
}

void app_main(void)
{
    configure_led();

    ESP_LOGI(TAG, "Turning the LED RED!");
    set_red_led();

    /* Keep the task running indefinitely */
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}

#elif CONFIG_BLINK_LED_GPIO

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void app_main(void)
{
    configure_led();

    ESP_LOGI(TAG, "Turning the LED ON!");
    gpio_set_level(BLINK_GPIO, 1); // Assuming 1 turns the LED on

    /* Keep the task running indefinitely */
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}

#else
#error "unsupported LED type"
#endif

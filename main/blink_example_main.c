/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include <driver/i2c_master.h>
#include "i2cmaster.h"
#include "tca9534.h"
#include "tca9535.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 50, 0, 0);
        led_strip_set_pixel(led_strip, 1, 0, 50, 0);
        led_strip_set_pixel(led_strip, 2, 0, 0, 50);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 3, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
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
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

void init_pin_out(uint8_t pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

void set_pin(uint8_t pin, bool state) {
    gpio_set_level(pin, state);
}

volatile const uint8_t n_pins[28] = 
{
    1, 2, 3, 4, 5, 6, 7, 
    8, 9, 10, 11, 12, 13, 14, 
    15, 16, 17, 18, 21, 38, 39, 
    40, 41, 42, 43, 44, 47, 48
};

#define DIO16_I2C_ADDR  0x20
#define DI8_I2C_ADDR    0x38
#define DO8_I2C_ADDR    0x39

i2c_master_bus_handle_t bus_handle;
i2c_master_bus_handle_t *p_bus_handle;

void print_val(char* name, uint8_t bit_num, uint32_t val) {
    char str[32];
    for (int bit = 0; bit < bit_num; bit++) {
        str[bit] = val & (1 << bit) ? '1' : '0';
    }
    str[bit_num] = '\0';

    ESP_LOGI(TAG, "%s %s", name, str);
}

void app_main(void)
{
    configure_led();
    
    i2c_master_bus_handle_t *bus_handle;
    bus_handle = i2cmaster_init(0);

    if (!i2cmaster_test(bus_handle, DIO16_I2C_ADDR))
        ESP_LOGE(TAG, "DIO16 module not found!");
    if (!i2cmaster_test(bus_handle, DI8_I2C_ADDR))
        ESP_LOGE(TAG, "DI8-ISO module not found!");
    if (!i2cmaster_test(bus_handle, DO8_I2C_ADDR))
        ESP_LOGE(TAG, "DO8 module not found!");

    i2c_master_dev_handle_t* dio16_handle = tca9535_init(bus_handle, DIO16_I2C_ADDR, 0xFFFF, 0x0000);
    i2c_master_dev_handle_t* di8_handle = tca9534_init(bus_handle, DI8_I2C_ADDR, 0x00, 0x00);
    i2c_master_dev_handle_t* do8_handle = tca9534_init(bus_handle, DO8_I2C_ADDR, 0xFF, 0x00);

    uint8_t cntr = 0;
    uint8_t val_out8 = 0;
    uint16_t val_out16 = 0;
        
    while (1) {
        blink_led();
        s_led_state = !s_led_state;

        if (di8_handle) {
            uint8_t di8;
            if (tca9534_read(di8_handle, &di8))
                print_val("DI8", 8, di8);
        }

        if (do8_handle)
            tca9534_write(do8_handle, 1 << (cntr & 0x07));
    
        if (dio16_handle)
            tca9535_write(dio16_handle, 1 << (15 - cntr));

        if (++cntr == 16)
            cntr = 0;

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}



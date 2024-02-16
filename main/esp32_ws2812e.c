#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "led_strip.h"

#include "driver/touch_pad.h"
#include "soc/rtc_periph.h"

/* Max number of leds. */
#define LED_NUM 60

#define TOUCH_THRESH_NO_USE 0
#define TOUCHPAD_FILTER_TOUCH_PERIOD 10

#define PIN_LED_STRIPE GPIO_NUM_13
#define PIN_TOUCH TOUCH_PAD_NUM6

#define PIN_R_DT GPIO_NUM_27
#define PIN_R_CLK GPIO_NUM_26

#define PIN_G_DT GPIO_NUM_25
#define PIN_G_CLK GPIO_NUM_33

#define PIN_B_DT GPIO_NUM_32
#define PIN_B_CLK GPIO_NUM_35

#define PIN_BRIGHTNESS_DT GPIO_NUM_34
#define PIN_BRIGHTNESS_CLK GPIO_NUM_39

/* It might be a good idea to split the colours in to different arrays and use
 * memset for better performance. */
typedef struct led_color {
    volatile int16_t r;
    volatile int16_t g;
    volatile int16_t b;
} led_color;

static led_strip_handle_t led_strip;
static led_color led[LED_NUM];
static volatile bool led_power_status;
static volatile bool dt = false;
gpio_num_t pins[][2] = {
    { PIN_R_CLK, PIN_R_DT },
    { PIN_G_CLK, PIN_G_DT },
    { PIN_B_CLK, PIN_B_DT },
    { PIN_BRIGHTNESS_CLK, PIN_BRIGHTNESS_DT },
};

void led_power()
{
    led_power_status = !led_power_status;
    if (led_power_status) {
        for (int i = 0; i < LED_NUM; i++) {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, led[i].r, led[i].g, led[i].b));
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        led_strip_refresh(led_strip);
    } else {
        for (int i = LED_NUM - 1; i >= 0; i--) {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        ESP_ERROR_CHECK(led_strip_clear(led_strip));
    }
    ESP_ERROR_CHECK(touch_pad_intr_enable());
    vTaskDelete(NULL);
}

void touch_interrupt(void *_arg)
{
    static bool skip = false;

    ESP_ERROR_CHECK(touch_pad_intr_disable());
    ESP_ERROR_CHECK(touch_pad_clear_status());

    if (skip) {
        skip = false;
        ESP_ERROR_CHECK(touch_pad_intr_enable());
        return;
    }
    skip = true;

    xTaskCreate(led_power, "led_power", 4096, NULL, 5, NULL);
}

void led_update()
{
    int r,g,b;
    for (int i = 0; i < LED_NUM; i++) {
        r = led[i].r > 0 ? led[i].r : 0;
        g = led[i].g > 0 ? led[i].g : 0;
        b = led[i].b > 0 ? led[i].b : 0;
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, r, g, b));
    }
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    printf("(%d, %d, %d)\n", led[0].r,led[0].g,led[0].b);
}

void rotary_colour(void *_arg)
{
    int v = *((int *) _arg);
    int vv = ((int *) _arg)[1];

    if (dt) {
        switch (v) {
            case PIN_R_CLK:
                for (int i = 0; i < LED_NUM; i++) {
                    led[i].r -= 5;
                }
                break;
            case PIN_G_CLK:
                for (int i = 0; i < LED_NUM; i++) {
                    led[i].g -= 5;
                }
                break;
            case PIN_B_CLK:
                for (int i = 0; i < LED_NUM; i++) {
                    led[i].b -= 5;
                }
                break;
            case PIN_BRIGHTNESS_CLK:
                for (int i = 0; i < LED_NUM; i++) {
                    led[i].r -= 1;
                    led[i].g -= 1;
                    led[i].b -= 1;
                }
                break;
        }
    } else {
        switch (v) {
            case PIN_R_CLK:
                for (int i = 0; i < LED_NUM; i++) {
                    led[i].r += 5;
                }
                break;
            case PIN_G_CLK:
                for (int i = 0; i < LED_NUM; i++) {
                    led[i].g += 5;
                }
                break;
            case PIN_B_CLK:
                for (int i = 0; i < LED_NUM; i++) {
                    led[i].b += 5;
                }
                break;
            case PIN_BRIGHTNESS_CLK:
                for (int i = 0; i < LED_NUM; i++) {
                    led[i].r += 1;
                    led[i].g += 1;
                    led[i].b += 1;
                }
                break;
        }
    }
    led_update();

    vTaskDelay(80 / portTICK_PERIOD_MS);
    dt = false;
    ESP_ERROR_CHECK(gpio_intr_enable(v));
    ESP_ERROR_CHECK(gpio_intr_enable(vv));
    vTaskDelete(NULL);
}

void rotary_interrupt(void *_arg)
{
    int p = *((int *)_arg);

    ESP_ERROR_CHECK(gpio_intr_disable(p));
    switch (p) {
        case PIN_R_CLK:
            ESP_ERROR_CHECK(gpio_intr_disable(PIN_R_DT));
            xTaskCreate(rotary_colour, "rotary_colour", 4096, _arg, 5, NULL);
            break;
        case PIN_G_CLK:
            ESP_ERROR_CHECK(gpio_intr_disable(PIN_G_DT));
            xTaskCreate(rotary_colour, "rotary_colour", 4096, _arg, 5, NULL);
            break;
        case PIN_B_CLK:
            ESP_ERROR_CHECK(gpio_intr_disable(PIN_B_DT));
            xTaskCreate(rotary_colour, "rotary_colour", 4096, _arg, 5, NULL);
            break;
        case PIN_BRIGHTNESS_CLK:
            ESP_ERROR_CHECK(gpio_intr_disable(PIN_BRIGHTNESS_DT));
            xTaskCreate(rotary_colour, "rotary_colour", 4096, _arg, 5, NULL);
            break;
        default:
            dt = true;
            break;
    }
}

void touchpad_init()
{
    uint16_t touch_value;
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V));
    ESP_ERROR_CHECK(touch_pad_config(PIN_TOUCH, TOUCH_THRESH_NO_USE));
    ESP_ERROR_CHECK(touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD));
    ESP_ERROR_CHECK(touch_pad_read_filtered(PIN_TOUCH, &touch_value));
    ESP_ERROR_CHECK(touch_pad_set_thresh(PIN_TOUCH, touch_value * 2 / 3));
    ESP_ERROR_CHECK(touch_pad_isr_register(touch_interrupt, NULL));
    ESP_ERROR_CHECK(touch_pad_intr_enable());
}

void led_stripe_init()
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = PIN_LED_STRIPE,
        .max_leds = LED_NUM,
    };
    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT,
        .flags.with_dma = true,
        .spi_bus = SPI2_HOST,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    for (int i = 0; i < LED_NUM; i++)
        led[i].r = led[i].g = led[i].b = 50;
}

void rotary_init()
{
    size_t pins_arr_len = 4;
    int i,j;

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    for (i = 0; i < pins_arr_len; i++) {
        for (j = 0; j < 2; j++) {
            ESP_ERROR_CHECK(gpio_set_direction(pins[i][j], GPIO_MODE_INPUT));
            ESP_ERROR_CHECK(gpio_set_intr_type(pins[i][j], GPIO_INTR_NEGEDGE));
            ESP_ERROR_CHECK(gpio_intr_enable(pins[i][j]));
            ESP_ERROR_CHECK(gpio_isr_handler_add(pins[i][j], rotary_interrupt, &pins[i][j]));
        }
    }
}

void app_main(void)
{
    touchpad_init();
    led_stripe_init();
    rotary_init();
}

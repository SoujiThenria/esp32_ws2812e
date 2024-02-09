#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "led_strip.h"

#include "driver/touch_pad.h"
#include "soc/rtc_periph.h"

#define TOUCH_PIN_ON_OFF 5
#define LED_PIN 13
#define LED_NUM 60
#define TOUCH_THRESH_NO_USE   (0)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)

bool led_status = false;
static led_strip_handle_t led_strip;

void led_start()
{
    led_strip_refresh(led_strip);
    led_strip_clear(led_strip);
    for (int i = 0; i < LED_NUM; i++) {
        led_strip_set_pixel(led_strip, i, 200, 200, 200);
        led_strip_refresh(led_strip);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    for (int i = 0; i < 3; i++) {
        for (int i = 0; i < LED_NUM; i++)
            led_strip_set_pixel(led_strip, i, 10, 10, 10);
        led_strip_refresh(led_strip);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        for (int i = 0; i < LED_NUM; i++)
            led_strip_set_pixel(led_strip, i, 200, 200, 200);
        led_strip_refresh(led_strip);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    led_strip_refresh(led_strip);

    // vTaskDelete(NULL);
}

void led_stop()
{
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);
    // vTaskDelete(NULL);
}

void interrupt_on(void *_arg)
{
    touch_pad_intr_disable();
    touch_pad_clear_status();
    if (!led_status) {
        led_status = true;
        xTaskCreate(&led_start, "led_start", 4096, NULL, 5, NULL);
    }
}

void interrupt_off(void *_arg)
{
    touch_pad_intr_disable();
    touch_pad_clear_status();
    if (led_status) {
        led_status = false;
        xTaskCreate(&led_stop, "led_stop", 4096, NULL, 5, NULL);
    }
}

void touchpad_init()
{
    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    touch_pad_config(TOUCH_PIN_ON_OFF, TOUCH_THRESH_NO_USE);
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);

    uint16_t touch_value;
    touch_pad_read_filtered(TOUCH_PIN_ON_OFF, &touch_value);
    touch_pad_set_thresh(TOUCH_PIN_ON_OFF, touch_value * 2 / 3);

    touch_pad_isr_register(interrupt_on, NULL);
    touch_pad_intr_enable();
}

void led_stripe_init()
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_PIN,
        .max_leds = LED_NUM,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    led_strip_clear(led_strip);
}

void app_main(void)
{
    touchpad_init();
    led_stripe_init();

    printf("ready");
    for (;;) {
        if (led_status) {
            printf("change hadler to off\n");
            touch_pad_isr_deregister(interrupt_on, NULL);
            touch_pad_isr_register(interrupt_off, NULL);
        } else {
            printf("change hadler to on\n");
            touch_pad_isr_deregister(interrupt_off, NULL);
            touch_pad_isr_register(interrupt_on, NULL);
        }
        printf("led_status: %s\n", led_status ? "true" : "false");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        touch_pad_intr_enable();
    }
}

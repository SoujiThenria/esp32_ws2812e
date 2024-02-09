#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "led_strip.h"

#include "driver/touch_pad.h"
#include "soc/rtc_periph.h"

#define TOUCH_PIN_POWER 5
#define TOUCH_PIN_COLOUR 6
#define LED_PIN 13
#define LED_NUM 60
#define TOUCH_THRESH_NO_USE   (0)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)

typedef struct led_color {
    int r;
    int g;
    int b;
} led_color;

static led_strip_handle_t led_strip;
static bool power_status = false;
static led_color leds[LED_NUM];

void led_power()
{
    power_status = !power_status;
    if (power_status) {
        for (int i = 0; i < LED_NUM; i++) {
            leds[i].r = leds[i].g = leds[i].b = 200;
            led_strip_set_pixel(led_strip, i, 200, 200, 200);
            led_strip_refresh(led_strip);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        led_strip_refresh(led_strip);
    } else {
        for (int i = LED_NUM - 1; i >= 0; i--) {
            leds[i].r = leds[i].g = leds[i].b = 0;
            led_strip_set_pixel(led_strip, i, 0, 0, 0);
            led_strip_refresh(led_strip);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        led_strip_clear(led_strip);
    }
    touch_pad_intr_enable();
    vTaskDelete(NULL);
}

void led_colour_change()
{
    if (!power_status)
        goto end;

    static bool count_down = true;
    if ((leds[0].r - 10) <= 0) {
        count_down = false;
    } else if ((leds[0].r + 10) > 255) {
        count_down = true;
    }

    if (count_down) {
        for (int i = 0; i < LED_NUM; i++) {
            leds[i].r = leds[i].g = leds[i].b = leds[i].r - 10;
            led_strip_set_pixel(led_strip, i, leds[i].r, leds[i].g, leds[i].b);
        }
    }
    else {
        for (int i = 0; i < LED_NUM; i++) {
            leds[i].r = leds[i].g = leds[i].b = leds[i].r + 10;
            led_strip_set_pixel(led_strip, i, leds[i].r, leds[i].g, leds[i].b);
        }
    }
    led_strip_refresh(led_strip);

end:
    touch_pad_intr_enable();
    vTaskDelete(NULL);
}

void touch_interrupt(void *_arg)
{
    touch_pad_intr_disable();
    uint32_t pad_intr = touch_pad_get_status();
    // set flag to off
    static uint32_t interrupt_count = (0 << 0);
    touch_pad_clear_status();

    /* The touch results into two interrupts, consume the second one */
    // check if flag is on
    if (interrupt_count & (1 << 0)) {
        // toggle flag
        interrupt_count ^= (1 << 0);
        touch_pad_intr_enable();
        return;
    }
    // set flag (I could also toggle, but no)
    interrupt_count |= (1 << 0);

    if ((pad_intr >> TOUCH_PIN_POWER) & 0x01) {
        xTaskCreate(led_power, "led_power", 4096, NULL, 5, NULL);
    }
    if ((pad_intr >> TOUCH_PIN_COLOUR) & 0x01) {
        xTaskCreate(led_colour_change, "led_colour_change", 4096, NULL, 5, NULL);
    }
}

void touchpad_init()
{
    uint16_t touch_value;
    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    touch_pad_config(TOUCH_PIN_POWER, TOUCH_THRESH_NO_USE);
    touch_pad_config(TOUCH_PIN_COLOUR, TOUCH_THRESH_NO_USE);
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
    touch_pad_read_filtered(TOUCH_PIN_POWER, &touch_value);
    touch_pad_set_thresh(TOUCH_PIN_POWER, touch_value * 2 / 3);
    touch_pad_read_filtered(TOUCH_PIN_COLOUR, &touch_value);
    touch_pad_set_thresh(TOUCH_PIN_COLOUR, touch_value * 2 / 3);
    touch_pad_isr_register(touch_interrupt, NULL);
    touch_pad_intr_enable();
}

void led_stripe_init()
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_PIN,
        .max_leds = LED_NUM,
    };
    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT,
        .flags.with_dma = true,
        .spi_bus = SPI2_HOST,
    };
    led_strip_new_spi_device(&strip_config, &spi_config, &led_strip);

    led_strip_clear(led_strip);
}

void app_main(void)
{
    touchpad_init();
    led_stripe_init();
}

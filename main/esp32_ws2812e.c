#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "led_strip.h"

#include "driver/touch_pad.h"
#include "soc/rtc_periph.h"

#define LED_NUM 60
#define TOUCH_THRESH_NO_USE   (0)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)

#define PIN_TOUCH TOUCH_PAD_NUM6
#define PIN_LED_STRIPE GPIO_NUM_13
#define PIN_R_DT GPIO_NUM_27
#define PIN_R_CLK GPIO_NUM_26
#define PIN_G_DT GPIO_NUM_25
#define PIN_G_CLK GPIO_NUM_33
#define PIN_B_DT GPIO_NUM_32
#define PIN_B_CLK GPIO_NUM_35

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

// void led_colour_change()
// {
//     if (!power_status)
//         goto end;
// 
//     static bool count_down = true;
//     if ((leds[0].r - 10) <= 0) {
//         count_down = false;
//     } else if ((leds[0].r + 10) > 255) {
//         count_down = true;
//     }
// 
//     if (count_down) {
//         for (int i = 0; i < LED_NUM; i++) {
//             leds[i].r = leds[i].g = leds[i].b = leds[i].r - 10;
//             led_strip_set_pixel(led_strip, i, leds[i].r, leds[i].g, leds[i].b);
//         }
//     }
//     else {
//         for (int i = 0; i < LED_NUM; i++) {
//             leds[i].r = leds[i].g = leds[i].b = leds[i].r + 10;
//             led_strip_set_pixel(led_strip, i, leds[i].r, leds[i].g, leds[i].b);
//         }
//     }
//     led_strip_refresh(led_strip);
// 
// end:
//     touch_pad_intr_enable();
//     vTaskDelete(NULL);
// }

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

    if ((pad_intr >> PIN_TOUCH) & 0x01) {
        xTaskCreate(led_power, "led_power", 4096, NULL, 5, NULL);
    }
    // if ((pad_intr >> TOUCH_PIN_COLOUR) & 0x01) {
    //     xTaskCreate(led_colour_change, "led_colour_change", 4096, NULL, 5, NULL);
    // }
}

void touchpad_init()
{
    uint16_t touch_value;
    touch_pad_init();

    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    touch_pad_config(PIN_TOUCH, TOUCH_THRESH_NO_USE);
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
    touch_pad_read_filtered(PIN_TOUCH, &touch_value);
    touch_pad_set_thresh(PIN_TOUCH, touch_value * 2 / 3);
    touch_pad_isr_register(touch_interrupt, NULL);
    touch_pad_intr_enable();
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
    led_strip_new_spi_device(&strip_config, &spi_config, &led_strip);

    led_strip_clear(led_strip);
}

bool dt = false;
void rotary_colour(void *_arg)
{
    int v = *((int *) _arg);


    if (dt) {
        switch (v) {
            case PIN_R_CLK:
                for (int i = 0; i < LED_NUM; i++) {
                    leds[i].r -= 10;
                    led_strip_set_pixel(led_strip, i, leds[i].r, leds[i].g, leds[i].b);
                }
                break;
            case PIN_G_CLK:
                for (int i = 0; i < LED_NUM; i++) {
                    leds[i].g -= 10;
                    led_strip_set_pixel(led_strip, i, leds[i].r, leds[i].g, leds[i].b);
                }
                break;
            case PIN_B_CLK:
                for (int i = 0; i < LED_NUM; i++) {
                    leds[i].b -= 10;
                    led_strip_set_pixel(led_strip, i, leds[i].r, leds[i].g, leds[i].b);
                }
                break;
        }
    } else {
        switch (v) {
            case PIN_R_CLK:
                for (int i = 0; i < LED_NUM; i++) {
                    leds[i].r += 10;
                    led_strip_set_pixel(led_strip, i, leds[i].r, leds[i].g, leds[i].b);
                }
                break;
            case PIN_G_CLK:
                for (int i = 0; i < LED_NUM; i++) {
                    leds[i].g += 10;
                    led_strip_set_pixel(led_strip, i, leds[i].r, leds[i].g, leds[i].b);
                }
                break;
            case PIN_B_CLK:
                for (int i = 0; i < LED_NUM; i++) {
                    leds[i].b += 10;
                    led_strip_set_pixel(led_strip, i, leds[i].r, leds[i].g, leds[i].b);
                }
                break;
        }
    }
    led_strip_refresh(led_strip);

    printf("(%d, %d, %d)\n", leds[0].r,leds[0].g,leds[0].b);

    vTaskDelay(50 / portTICK_PERIOD_MS);
    dt = false;
    gpio_intr_enable(v);
    gpio_intr_enable(PIN_R_DT);
    gpio_intr_enable(PIN_G_DT);
    gpio_intr_enable(PIN_B_DT);
    vTaskDelete(NULL);
}

void rotary_interrupt(void *_arg)
{
    int p = *((int *)_arg);

    gpio_intr_disable(p);
    switch (p) {
        case PIN_R_CLK:
            gpio_intr_disable(PIN_R_DT);
            xTaskCreate(rotary_colour, "rotary_colour", 4096, _arg, 5, NULL);
            break;
        case PIN_G_CLK:
            gpio_intr_disable(PIN_G_DT);
            xTaskCreate(rotary_colour, "rotary_colour", 4096, _arg, 5, NULL);
            break;
        case PIN_B_CLK:
            gpio_intr_disable(PIN_B_DT);
            xTaskCreate(rotary_colour, "rotary_colour", 4096, _arg, 5, NULL);
            break;
        case PIN_R_DT: case PIN_G_DT: case PIN_B_DT:
            dt = true;
            break;
    }
}

gpio_num_t pins[] = {
    PIN_R_CLK,
    PIN_G_CLK,
    PIN_B_CLK,
    PIN_R_DT,
    PIN_G_DT,
    PIN_B_DT,
};

void rotary_init()
{
    size_t pins_arr_len = 6;
    int i;

    gpio_install_isr_service(0);

    for (i = 0; i < pins_arr_len; i++) {
        gpio_set_direction(pins[i], GPIO_MODE_INPUT);
        gpio_set_intr_type(pins[i], GPIO_INTR_NEGEDGE);
        gpio_intr_enable(pins[i]);
        gpio_isr_handler_add(pins[i], rotary_interrupt, &pins[i]);
    }
}

void app_main(void)
{
    touchpad_init();
    led_stripe_init();
    rotary_init();
}

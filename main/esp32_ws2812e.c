#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/touch_pad.h"
#include "soc/rtc_periph.h"

#define TOUCH_PIN_ON_OFF 5
#define TOUCH_THRESH_NO_USE   (0)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)

bool led_status;

void interrupt_on(void *_arg)
{
    touch_pad_clear_status();
    led_status = true;
}

void interrupt_off(void *_arg)
{
    touch_pad_clear_status();
    led_status = false;
}

void app_main(void)
{
    led_status = false;

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

    for (;;) {
        if (led_status) {
            touch_pad_isr_deregister(interrupt_on, NULL);
            touch_pad_isr_register(interrupt_off, NULL);
        } else {
            touch_pad_isr_deregister(interrupt_off, NULL);
            touch_pad_isr_register(interrupt_on, NULL);
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

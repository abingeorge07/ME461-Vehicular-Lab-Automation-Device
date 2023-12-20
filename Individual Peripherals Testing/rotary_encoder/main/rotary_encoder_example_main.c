// /*
//  * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
//  *
//  * SPDX-License-Identifier: CC0-1.0
//  */

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "esp_log.h"
// #include "driver/pulse_cnt.h"
// //#include "gptimer.h"

// static const char *TAG = "example";

// #define EXAMPLE_PCNT_HIGH_LIMIT 100
// #define EXAMPLE_PCNT_LOW_LIMIT  -100

// #define EXAMPLE_EC11_GPIO_A 34
// #define EXAMPLE_EC11_GPIO_B -2




// /////////////////////////

// static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
// {
//     BaseType_t high_task_wakeup;
//     QueueHandle_t queue = (QueueHandle_t)user_ctx;
//     // send event data to queue, from this interrupt callback
//     xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
//     return (high_task_wakeup == pdTRUE);
// }

// void app_main(void)
// {
//     ESP_LOGI(TAG, "install pcnt unit");
//     pcnt_unit_config_t unit_config = {
//         .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
//         .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
//     };
//     pcnt_unit_handle_t pcnt_unit = NULL;
//     ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

//     ESP_LOGI(TAG, "set glitch filter");
//     pcnt_glitch_filter_config_t filter_config = {
//         .max_glitch_ns = 1000,
//     };
//     ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

//     ESP_LOGI(TAG, "install pcnt channels");
//     pcnt_chan_config_t chan_a_config = {
//         .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
//         .level_gpio_num = EXAMPLE_EC11_GPIO_B,
//     };
//     pcnt_channel_handle_t pcnt_chan_a = NULL;
//     ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
//     pcnt_chan_config_t chan_b_config = {
//         .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
//         .level_gpio_num = EXAMPLE_EC11_GPIO_A,
//     };
//     pcnt_channel_handle_t pcnt_chan_b = NULL;
//     ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

//     ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
//     ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
//     //ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
//     // ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
//     //ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

//     ESP_LOGI(TAG, "add watch points and register callbacks");
//     int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, -50, 0, 50, EXAMPLE_PCNT_HIGH_LIMIT};
//     for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
//         ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
//     }
//     pcnt_event_callbacks_t cbs = {
//         .on_reach = example_pcnt_on_reach,
//     };
//     QueueHandle_t queue = xQueueCreate(10, sizeof(int));
//     ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

//     ESP_LOGI(TAG, "enable pcnt unit");
//     ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
//     ESP_LOGI(TAG, "clear pcnt unit");
//     ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
//     ESP_LOGI(TAG, "start pcnt unit");
//     ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

//     // Report counter value
//     int pulse_count = 0;
//     int event_count = 0;
//     while (1) {
//         if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000))) {
//             ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
//         } else {
//             ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
//             ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
//         }
//     }
// }




/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gptimer.h"
#include <stdio.h>
// #include "gptimer.h"

static const char *TAG = "example";

#define EXAMPLE_PCNT_HIGH_LIMIT 100
#define EXAMPLE_PCNT_LOW_LIMIT  -100

#define EXAMPLE_EC11_GPIO_A 39
#define EXAMPLE_EC11_GPIO_B -1

#define alarmInterrupt 1000000   //us
#define duration (alarmInterrupt/1000000)

#define PULSE_PER_REV 24
#define CIRCUMFERENCE 62.0



typedef struct {
    uint64_t event_count;
} example_queue_element_t;



//////////////////// global variables
example_queue_element_t ele;
QueueHandle_t queue;
gptimer_handle_t gptimer;
gptimer_config_t timer_config;
gptimer_event_callbacks_t cbs;
int interrupt = 0;
int tot_time = 0;
int event_count;
int pulse_count;
int prev_pulse = 0;

pcnt_unit_handle_t pcnt_unit = NULL;




////////////////////////// TIMER
static bool IRAM_ATTR example_timer_on_alarm_cb_v1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;
  
    // Retrieve count value and send to queue
    example_queue_element_t ele = {
        .event_count = edata->count_value
    };

    interrupt = 1;
    xQueueSendFromISR(queue, &ele, NULL);
    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}


void alarm_init(){    
    queue = xQueueCreate(10, sizeof(example_queue_element_t));
    if (!queue) {
        ESP_LOGE(TAG, "Creating queue failed");
        return;
    }

    ESP_LOGI(TAG, "Create timer handle");
    gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = example_timer_on_alarm_cb_v1,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, queue));

    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    ESP_LOGI(TAG, "Start timer, stop it at alarm event");
    gptimer_alarm_config_t alarm_config1 = {
        .reload_count = 0,
        .alarm_count = alarmInterrupt, 
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

}







///////////////////////// Pulse count

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}


void pulseCount(){

    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
        .level_gpio_num = EXAMPLE_EC11_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
        .level_gpio_num = EXAMPLE_EC11_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    // ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    // ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    // ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, -50, 0, 50, EXAMPLE_PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
                    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    // Report counter value
    pulse_count = 0;
    event_count = 0;
    /*
    while (1) {
        if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000))) {
            // ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
            printf("something went wrong in pulseCount function\n");
        } else {
            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
            // ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
        }
    }
    */

}

void convert2speed(){

    while(1){
        if(interrupt){

            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
            ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
           
            float dist = ((float)pulse_count/PULSE_PER_REV) * CIRCUMFERENCE;
            float speed = dist/duration; // cm/s

            tot_time += duration;

            printf("Time: %d Total Pulse: %d Inst Speed: %.3f\n", tot_time, pulse_count, speed);
            interrupt = 0;

        }

        vTaskDelay(10 / portTICK_PERIOD_MS);   
    }

}

void app_main(void)
{
    
    alarm_init();    
    pulseCount();
     //xTaskCreate(pulseCount, "pulseCount", 4096, NULL, 5, NULL);
    xTaskCreate(convert2speed, "convert2speed", 4096, NULL, 5, NULL);


}


#include <stdio.h>
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_vfs_dev.h"	

static const char *TAG = "example";
mcpwm_cmpr_handle_t comparator = NULL;

int pwm_init = 3000;



#define GPIO_PWM0A_OUT 15   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 33   //Set GPIO 16 as PWM0B
#define PWM_MOTOR_GPIO  32

// GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        35000    // 20000 ticks, 20ms

int buttonState = 0;         
int count_value =0;
int prestate =0;

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");

    gpio_reset_pin(GPIO_PWM0A_OUT);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_PWM0A_OUT, GPIO_MODE_OUTPUT);

    gpio_reset_pin(GPIO_PWM0B_OUT);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_PWM0B_OUT, GPIO_MODE_OUTPUT);

}

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void brushed_motor_forward(void)
{

    gpio_set_level(GPIO_PWM0A_OUT, 0);
    gpio_set_level(GPIO_PWM0B_OUT, 1);
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void brushed_motor_backward(void)
{
    gpio_set_level(GPIO_PWM0A_OUT, 1);
    gpio_set_level(GPIO_PWM0B_OUT, 0);  
}

/**
 * @brief motor stop
 */
static void brushed_motor_stop(void)
{
    gpio_set_level(GPIO_PWM0A_OUT, 0);
    gpio_set_level(GPIO_PWM0B_OUT, 0);
    pwm_init = 3000;
    printf("PWM: %d\n", pwm_init);
}


static void motor_start(void){
    pwm_init = 3000;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pwm_init));
    printf("PWM: %d\n", pwm_init);
}

static void motor_moveFaster(int increase){
    pwm_init += increase;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pwm_init));
    printf("PWM: %d\n", pwm_init);
}


static void motor_moveSlower(int decrease){
    pwm_init -= decrease;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pwm_init));
    printf("PWM: %d\n", pwm_init);
}


/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void motor_init(void)
{
    mcpwm_example_gpio_initialize();
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = PWM_MOTOR_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 0));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generator,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
                    MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generator,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW),
                    MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

}



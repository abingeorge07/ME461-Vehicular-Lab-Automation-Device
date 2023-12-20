#include "motor_lib.c"


#define GPIO_PWM0A_OUT 15   //Used to control direction
#define GPIO_PWM0B_OUT 33   //Used to control direction
#define PWM_MOTOR_GPIO  32  //Set PWM for speed



void app_main(void)
{

    motor_init();

    

    while(1){
        brushed_motor_forward();
        motor_start();
        motor_moveFaster(3000);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        brushed_motor_stop();
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        brushed_motor_backward();
        motor_start();
        motor_moveFaster(3000);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }


}
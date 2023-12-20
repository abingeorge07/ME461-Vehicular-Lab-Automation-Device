#include "i2c_lib.c"
#include "servo_lib.c"
#include <stdio.h>
#include "string.h"

void app_main(void){

    i2c_master_init();
    i2c_scanner();
    // initialize servo
    servo_init();
    // writeRegister(0x03, 0x0b, ACC_ADDRESS);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));


    // UART Init
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
    256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    uint8_t deviceID_acc;
    getDeviceID(&deviceID_acc);

    float angles[3];


    calibration();
    printf("Gx:\n");
    while(1){
        char setAngle_char[5];
        printf("What angle to set it to?\n");
        gets(setAngle_char);
        float setAngle = atof(setAngle_char);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(setAngle* 3.9)));
        printf("Angle is %f and pwm is %ld\n", setAngle, example_angle_to_compare(setAngle *3.9));
        int i;
        float sum = 0;
        for (i=0; i<200; i++) {
            if(i>=10){
                getAngle(angles);
                printf("%f\n", angles[0]);  
                sum = sum + angles[0];
            }
            vTaskDelay(20/ portTICK_PERIOD_MS);
        }

        printf("Average for angle %f is %f\n", setAngle, sum/190);
        // getAngle(angles);

        // printf("%f\n", angles[0]);
        vTaskDelay(50/ portTICK_PERIOD_MS);
    }


}



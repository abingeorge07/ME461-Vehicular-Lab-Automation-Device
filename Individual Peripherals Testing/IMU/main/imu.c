// #include "i2c_lib.h"
#include "i2c_lib.c"


#define ACC_ADD 0x68


void app_main(void){

    i2c_master_init();
    i2c_scanner();

    // UART Init
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
    256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    uint8_t deviceID_acc;
    getDeviceID(&deviceID_acc);


    // Disable interrupts
    writeRegister(REG_INTR_EN, 0, ACC_ADDRESS);

    // setRange
    setGyroRange(GYRO250);

    // // setRange
    setAccelRange(ACC250);

    //turn off standby mode or start measuring
    writeRegister(0x6B, 0, ACC_ADDRESS);




    float accel [3];
    float gyro [3];

    calibration(accel, gyro);

    while(1 == 1){
        getAccel(accel);
        getGyro(gyro);
        printf("Acc: \nX: %f Y: %f Z: %f \n", accel[0] , accel[1] , accel[2] );
        printf("Gyro: \nX: %f Y: %f Z: %f \n", gyro[0], gyro[1] , gyro[2]);        
        vTaskDelay(500 / portTICK_PERIOD_MS);   
    }

}



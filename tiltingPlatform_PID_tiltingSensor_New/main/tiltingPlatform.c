#include <string.h>
#include <stdio.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "i2c_lib.c"
#include "servo_lib.c"
#include <rom/ets_sys.h>

// SERVO GPIO set to 32

// for testing
#define NUM_SAMPLES_TESTING 1000
#define TESTING 0

// Controls definitions
#define ANGLE_SETPOINT 0
#define GEAR_RATIO 3.917
#define PORT CONFIG_EXAMPLE_PORT
#define ACC_ADD 0x42 // address of IMU
#define DT_IMU 20 // in milliseconds
#define DT_SERVO 10  // in milliseconds

// number of interations for the servo to move to reach the final 
// each iteration moves 20% of total distance to final position 
#define NUM_SERVO_STEPS (0.8)
#define HYST          0.1

// PID constants
float K_P = 0.1;
float K_I = 1.5;
float K_D = -5.6;



// global variables
float angles[] ={0,0,0};
float servoAngle = 0;
float error = 0;
float derror_dt = 0;
float serror_dt = 0;
static const char *TAG = "example";
float prevError = 0;


void calculate_angle(){
    getAngle(angles);    
}

void setAngle_absolute(float angle2set){

    if(angle2set > SERVO_SAFETY_MAX){
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(SERVO_SAFETY_MAX)));   
    }
    else if(angle2set < SERVO_SAFETY_MIN){
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(SERVO_SAFETY_MIN)));   
    }
    else{
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle2set)));   
    }

}

void setAngle(void){

    float initialAngle = 0;
    
    while(1){

        if((initialAngle-servoAngle)>HYST || (initialAngle-servoAngle)<-1*HYST)
        {
            if(initialAngle > SERVO_SAFETY_MAX)
            {
                initialAngle = SERVO_SAFETY_MAX;
            }
            else if(initialAngle<SERVO_SAFETY_MIN)
            {
                initialAngle = SERVO_SAFETY_MIN;
            }
            else
            {
    
                initialAngle+=((servoAngle-initialAngle)*NUM_SERVO_STEPS);
            }

            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(initialAngle)));
        }

        vTaskDelay(DT_SERVO/ portTICK_PERIOD_MS); 

    }
    
}

// calculate new angle based on PID
void PID_tilt(void){

    // calculating the error
    error =  angles[1] - ANGLE_SETPOINT;
    derror_dt = ((error-prevError)) / (DT_IMU);
    serror_dt = (serror_dt) + (((error + prevError)/2.0)*(DT_IMU/1000));
    prevError = error;

    // find the total change of PID
    float change = ((error * K_P) + (derror_dt * K_D) + (serror_dt*K_I));
    
    printf("%.2f, %.2f, %.2f, %.2f, %.2f\n", angles[1], change, error, serror_dt, derror_dt);
    if(error > HYST || error < -1*HYST){
        if((servoAngle + (change*GEAR_RATIO)) > SERVO_SAFETY_MAX){
            servoAngle = SERVO_SAFETY_MAX;
        }
        else if((servoAngle + (change*GEAR_RATIO)) < SERVO_SAFETY_MIN){
            servoAngle = SERVO_SAFETY_MIN;
        }
        else{           
            servoAngle += (change*GEAR_RATIO);
        }            
    }
}


// Main task for tilting platform
void IMU_task(void){

    while(1)
    {
        calculate_angle();
        PID_tilt();
        vTaskDelay(DT_IMU/ portTICK_PERIOD_MS);       
    }

}


void IMU_task_testing(void){

    while(1)
    {
        printf("Currently K_P = %.3f K_I = %.3f K_D = %.3f\n", K_P, K_I, K_D);

        int i;
        for(i=0; i<NUM_SAMPLES_TESTING; i++){
            calculate_angle();
            PID_tilt();
            vTaskDelay(DT_IMU/ portTICK_PERIOD_MS);  
        }

        // used only for testing, not part of the final implementation
        printf("Testing is done! New PID values? (Enter 'y' if you want to continue with the same values}\n");
        char response[3];

        gets(response);
        printf("%c\n", response[0]);
        float holderVal;
        if(response[0]!='Y' || response[0]!='y'){
            printf("Currently K_P = %.3f K_I = %.3f K_D = %.3f\n", K_P, K_I, K_D);
            char resValues[10];
            printf("Enter K_P (press enter if you want to keep the same value)\n");
            gets(resValues);
            holderVal = atof(resValues);
            
            if(holderVal != 0)
              K_P = holderVal;  

            printf("K_P = %.3f\n", K_P);

            printf("Enter K_I\n");
            gets(resValues);
            holderVal = atof(resValues);
            
            if(holderVal != 0)
              K_I = holderVal;

            printf("K_I = %.3f\n", K_I);

            printf("Enter K_D,\n");
            gets(resValues);
            holderVal = atof(resValues);
            
            if(holderVal != 0)
              K_D = holderVal; 

            printf("K_D = %.3f\n", K_D);

            printf("Do you want to calibrate?\n");
            gets(resValues);

            printf("Put on a level surface\n");
            gets(response);

            if(resValues[0] == 'y' || resValues[0] == 'Y')
            {   
                setAngle_absolute(0);
                servoAngle = 0;
                calibration();
            }
        }
    }
}
    
void app_main(void)
{
    // initialize servo
    servo_init();
    setAngle_absolute(0);

    // initialize IMU
    init_IMU();

 
    // Mains tasks that are running in parallel
    #if TESTING
        printf("AngleX, P_error, I_error, D_error\n");
        xTaskCreate(IMU_task_testing, "IMU_task_testing", 4096, NULL, 5, NULL);
    #else
        xTaskCreate(IMU_task, "IMU_task", 4096, NULL, 5, NULL);
    #endif

    xTaskCreate(setAngle, "setAngle", 4096, NULL, 5, NULL);

}


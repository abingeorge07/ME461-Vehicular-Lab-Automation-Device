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
#include <rom/ets_sys.h>
#include "IR_lib.c"
#include "motor_lib.c"
#include "encoder_lib.c"

// MOTOR WIRING
// PWM ----------> PIN 32
// DIR 1---------> PIN 25
// DIR 2---------> PIN 33

// Encoder
// Signal 1 ------> PIN 14
// Signal 2 ------> PIN 27

#define PORT CONFIG_EXAMPLE_PORT
#define MINIMUM_MOTOR_PWM 30000
#define STOP_MOTOR_PWM 3000
#define THRESHOLD_IR 4090
#define RADIUS 1.0
#define DT_ENCODER 50
#define DT_MOTOR 50
#define INIT_PWM 5000
#define DT_MOTOR 50
#define ACC_ADD 0x68 // address of IMU
#define STEADY_SPEED_INCREASE 15000
#define STEADY_SPEED_INCREASE_2 9000
#define MAX_PWM 35000
#define MIN_PWN 25000
#define IMU_TASK_DELAY 10
#define PID_CONSTANT 4000

// MOTOR PINS
#define GPIO_PWM0A_OUT 25   //Used to control direction
#define GPIO_PWM0B_OUT 33   //Used to control direction
#define PWM_MOTOR_GPIO  32  //Set PWM for speed

// global variables
struct station{
    int stationNum;
    float position;
    float absPosition; // this is what the encoder reads
};

struct station* stationArr = NULL;
int* stationOrder;
int numberStations = 0;
int currentStation = 0;
int startMotor = 0;
float error = 0;
float derror_dt = 0;
float serror_dt = 0;
float prevError = 0;
float change = 0;
float distance2move = 0;
int pwm = 0;
int calStarted = 0;
char text[100]; // used to send commands to main computer
float angles[] = {0,0,0};
uint32_t IR_reading = 0;
int moving = 0;
int nextStation;
int timeout_stage = 0; // in milliseconds
int sleep_switch = 0;
float last2first = 0;// distance from the last station to the first one
float trackLength = 0;
// PID constants
float K_P = 10;
float K_I = 0.1;
float K_D = -3000;


// // Only used for calibration routing to get the trajectory of the track
void getAngles(){

    while(1){
        if(calStarted == 0){
            vTaskDelete(NULL);
            break;
        }

        calculate_angle(angles);
        vTaskDelay(IMU_TASK_DELAY/portTICK_PERIOD_MS);
    }
} 

// Only used for the calibration routine
void startCal(){

    if(stationArr != NULL){
        free(stationArr);
        free(stationOrder);
    }
    
    stationArr = (struct station*) malloc(sizeof(struct station)*numberStations);
    stationOrder = (int*) malloc(sizeof(int)*numberStations); 

    // beginning the calibration
    brushed_motor_forward();
    motor_start();
    motor_setSpeed(MINIMUM_MOTOR_PWM);

    

    while(1){
        IR_reading= getIR();
        printf("IR = %ld and position = %f\n", IR_reading, get_absolutePosition());
        
        if(IR_reading>THRESHOLD_IR){
            // stop the truck
            motor_setSpeed(STOP_MOTOR_PWM);
            brushed_motor_stop();

            // reached the end
            if(currentStation == numberStations){
                calStarted = 0;
                last2first = get_absolutePosition() - stationArr[currentStation-1].absPosition;
                trackLength += last2first;
                currentStation = 0;
                vTaskDelete(NULL);
                break;
            }
            stationArr[currentStation].stationNum = currentStation;
            // add line to store the distance information
            stationArr[currentStation].absPosition = get_absolutePosition();

            // first station is position 0
            if(currentStation == 0)
            {
                stationArr[currentStation].position = 0;
            }
            else{
                stationArr[currentStation].position = get_absolutePosition() - stationArr[currentStation-1].absPosition;
                trackLength += stationArr[currentStation].position;
            }
            currentStation++;
            vTaskDelay(2000/portTICK_PERIOD_MS);
            brushed_motor_forward();
            motor_setSpeed(MINIMUM_MOTOR_PWM);
            vTaskDelay(500/portTICK_PERIOD_MS);
        }

        vTaskDelay(30/portTICK_PERIOD_MS);
    }
}



void motor_task(void){

    while(1){
        if(startMotor == 1){
            if(error > 0){
                brushed_motor_forward();
            }
            else{
                brushed_motor_backward();
            }
            
            if(error <= 5 || error >= -5){
                pwm = STEADY_SPEED_INCREASE_2 - INIT_PWM;
                motor_setSpeed(STEADY_SPEED_INCREASE_2);
            }
            else if(error <= 10 || error >= -10){
                pwm = STEADY_SPEED_INCREASE - INIT_PWM;
                motor_setSpeed(STEADY_SPEED_INCREASE);
            }
            else{
                if(error>0 && change >0){
                    pwm = (change/distance2move) * PID_CONSTANT;
                    
                    if((pwm+INIT_PWM) >= MAX_PWM)
                        pwm = MAX_PWM - 1 - INIT_PWM;
                    else if((pwm+INIT_PWM) <= MIN_PWN)
                        pwm = MIN_PWN - INIT_PWM;

                    motor_setSpeed(INIT_PWM + pwm);
                }
                else if (error > 0 && change< 0){
                    pwm = (change/distance2move) * PID_CONSTANT;
                    if((pwm+INIT_PWM) >= MAX_PWM)
                        pwm = MAX_PWM - 1 - INIT_PWM;
                    else if((pwm+INIT_PWM) <= MIN_PWN)
                        pwm = MIN_PWN - INIT_PWM;

                    motor_setSpeed(INIT_PWM + pwm);
                }
                else if (error < 0 && change < 0){
                    pwm = (change/distance2move) * PID_CONSTANT;

                    if((pwm+INIT_PWM) >= MAX_PWM)
                        pwm = MAX_PWM - 1 - INIT_PWM;
                    else if((pwm+INIT_PWM) <= MIN_PWN)
                        pwm = MIN_PWN - INIT_PWM;


                    motor_setSpeed(INIT_PWM + pwm);
                }
                else if (error < 0 && change > 0 ){
                    pwm = (change/distance2move) * PID_CONSTANT;

                    if((pwm+INIT_PWM) >= MAX_PWM)
                        pwm = MAX_PWM - 1 - INIT_PWM;
                    else if((pwm+INIT_PWM) <= MIN_PWN)
                        pwm = MIN_PWN - INIT_PWM;


                    motor_setSpeed(INIT_PWM + pwm);
                }

            }
        }
        else{
            brushed_motor_stop();
            vTaskDelete(NULL);
        }

        vTaskDelay(DT_MOTOR/ portTICK_PERIOD_MS);
    }
}





// use relative position for the next update
void PID_task(){
    moving = 1;
    distance2move = stationArr[nextStation].absPosition - get_absolutePosition();
    int temp = (int) (distance2move /trackLength);
    distance2move = distance2move - (temp*trackLength);
    distance2move = (distance2move > (trackLength/2))?(trackLength-distance2move): distance2move;
    float finalPosition = get_absolutePosition() + distance2move;
    startMotor = 1;

    
    while(1){
        error = finalPosition - get_absolutePosition();
        derror_dt = (error - prevError)/ (DT_ENCODER);
        serror_dt = serror_dt + (((error+prevError)/2)*DT_ENCODER/1000.0);

        printf("Error %f  Derror %f Serror %f\n", error, derror_dt, serror_dt);

        prevError = error;
        change = ((error * K_P) + (derror_dt * K_D) + (serror_dt*K_I));

        if(error<.1 && error>-.1){
            startMotor = 0;
            serror_dt = 0;
            currentStation = nextStation;
            brushed_motor_stop();
            vTaskDelay(timeout_stage/portTICK_PERIOD_MS);
            moving = 0; 
            // sleep_switch = 1;
            // vTaskDelete(NULL);
            break;
        }

        xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
        vTaskDelay(DT_ENCODER/ portTICK_PERIOD_MS);
    }
}



static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        int enable = 1;
        lwip_setsockopt(sock, IPPROTO_IP, IP_PKTINFO, &enable, sizeof(enable));
#endif

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif
        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        struct iovec iov;
        struct msghdr msg;
        struct cmsghdr *cmsgtmp;
        u8_t cmsg_buf[CMSG_SPACE(sizeof(struct in_pktinfo))];

        iov.iov_base = rx_buffer;
        iov.iov_len = sizeof(rx_buffer);
        msg.msg_control = cmsg_buf;
        msg.msg_controllen = sizeof(cmsg_buf);
        msg.msg_flags = 0;
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_name = (struct sockaddr *)&source_addr;
        msg.msg_namelen = socklen;
#endif

        while (1) {
            // ESP_LOGI(TAG, "Waiting for data");
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
            int len = recvmsg(sock, &msg, 0);
#else
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
#endif
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                    #if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
                    for ( cmsgtmp = CMSG_FIRSTHDR(&msg); cmsgtmp != NULL; cmsgtmp = CMSG_NXTHDR(&msg, cmsgtmp) ) {
                        if ( cmsgtmp->cmsg_level == IPPROTO_IP && cmsgtmp->cmsg_type == IP_PKTINFO ) {
                            struct in_pktinfo *pktinfo;
                            pktinfo = (struct in_pktinfo*)CMSG_DATA(cmsgtmp);
                            ESP_LOGI(TAG, "dest ip: %s\n", inet_ntoa(pktinfo->ipi_addr));
                        }
                    }       
                    #endif
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                // ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                // ESP_LOGI(TAG, "%s", rx_buffer);
                
                // start calibration
                if(strncmp(rx_buffer, "calXX", 2) == 0){
                    numberStations = atoi(rx_buffer+4);
                    printf("Number of Stations is %d\n", numberStations);
                    calibration(accel, gyro);
                    xTaskCreate(startCal, "startCal", 4096, NULL, 5, NULL); 
                    xTaskCreate(getAngles, "getAngles", 4096, NULL, 5, NULL); 
                    calStarted = 1;
                    sprintf(text, "OK");
                }
                // listening
                else if(strncmp(rx_buffer, "Listening", 6) == 0){

                    if(calStarted == 1){
                        // calculate_angle(angles);
                        sprintf(text, "IR: %ld, Pos: %.1f, X: %.1f, Y: %.1f, Z: %.1f\n", IR_reading, get_absolutePosition(), angles[0], angles[1], angles[2]);
                        // sprintf(text, "%ld\n", IR_reading);
                        // printf("calStarted %d\n", calStarted);
                    }
                    else if(moving == 1)
                    {
                        printf("error %f\n", error);
                        sprintf(text, "error %f\n", error);

                    }
                    // else if(sleep_switch == 1){
                    //     vTaskDelay(timeout_stage/ portTICK_PERIOD_MS);
                    //     sleep_switch = 0;
                    //     sprintf(text, "Done\n");
                    // }
                    else{
                        printf("Calibration is done\n");
                        sprintf(text, "Done\n");
                    }

                }
                else if(strncmp(rx_buffer, "move2", 5) == 0){
                    nextStation = atoi(rx_buffer + 6) - 1;
                    PID_task();
                    // moving = 1;  
                    // xTaskCreate(PID_task, "PID_task", 4096, NULL, 5, NULL); 
                    sprintf(text, "Done\n");
                }
                else if(strncmp(rx_buffer, "time:", 4) == 0){
                    sprintf(text, "Done\n");
                    timeout_stage = atoi(rx_buffer + 6) * 1000;
                }

                    // else if(strncmp(rx_buffer, "Ord: ", 4) == 0){
                //     int i;
                //     for(i=0; i<numberStations; i++){
                //         // little buggy if number of stations > 10
                //         stationOrder[i] = atoi(rx_buffer + 5 + (2*i));
                //         printf("%d\t", stationOrder[i]);
                //     }
                //     sprintf(text, "OK");
                // }


                int err = sendto(sock, text, strlen(text), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}



void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());


    ESP_ERROR_CHECK(example_connect());


    //motor init
    motor_init();
    brushed_motor_stop();
    // IR init
    IR_init();

    //encoder init
    encoderInit();
    setRadius(RADIUS);

    // IMU init
    init_IMU();


    #ifdef CONFIG_EXAMPLE_IPV4
        xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
    #endif
    #ifdef CONFIG_EXAMPLE_IPV6
        xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET6, 5, NULL);
    #endif

}



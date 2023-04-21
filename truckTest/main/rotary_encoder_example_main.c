#include "motor_lib.c"
#include <stdio.h>
#include "encoder_lib.c"
#include "i2c_lib.c"
#include <string.h>
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
#include <rom/ets_sys.h>
#include "IR_lib.c"
#include "math.h"


// MOTOR WIRING
// PWM ----------> PIN 32
// DIR 1---------> PIN 25
// DIR 2---------> PIN 33


// Encoder
// Signal 1 ------> PIN 14
// Signal 2 ------> PIN 27

#define ENCODER_PRINT 0

#define PORT CONFIG_EXAMPLE_PORT

#define ACC_ADD 0x68 // address of IMU
#define DT_IMU 10
#define DT_ENCODER 50
#define DT_MOTOR 50
#define DT_MAIN 20
#define STEADY_SPEED_INCREASE 15000
#define RADIUS 1.0
#define INIT_PWM 5000


#define IMU_ON 0

// global variables
float angles[] ={0,0,0};
int startMotor = 0;
float error = 0;
float derror_dt = 0;
float serror_dt = 0;
float prevError = 0;
float change = 0;

int pwm = 0;

char direction = 'N';
int speed = 0;
float distance = 0;
int PID_on = 0;
uint32_t IR_output;


// PID constants
float K_P = 5;
float K_I = 0.1;
float K_D = -3000;


void calculate_angle(){

    // calculating area under graph using trapezoidal
    angles[0] += 0.5*(DT_IMU*portTICK_PERIOD_MS/1000.0)*(float)((round(gyro_prev[0]*100)/100.0) + (round(gyro[0]*100)/100.0));
    angles[1] += 0.5*(DT_IMU*portTICK_PERIOD_MS/1000.0)*(float)(round(gyro_prev[1]) + round(gyro[1]));
    angles[2] += 0.5*(DT_IMU*portTICK_PERIOD_MS/1000.0)*(float)(round(gyro_prev[2]) + round(gyro[2]));

    // update the previous value
    gyro_prev[0] = gyro[0];
    gyro_prev[1] = gyro[1];
    gyro_prev[2] = gyro[2];
    
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

            if(distance <= 15 || distance>= -15){

                pwm = (STEADY_SPEED_INCREASE - INIT_PWM)*(abs(distance)/15);
                motor_setSpeed(STEADY_SPEED_INCREASE);
            }
            else{
                if(error>0 && change >0){
                    pwm = (change/distance) * 2000;
                    
                    if((pwm+INIT_PWM) >= MAX_PWM)
                        pwm = MAX_PWM - 1 - INIT_PWM;
                    else if((pwm+INIT_PWM) <= MIN_PWN)
                        pwm = 0;

                    motor_setSpeed(INIT_PWM + pwm);
                }
                else if (error > 0 && change< 0){
                    pwm = (change/distance) * 2000;
                    if((pwm+INIT_PWM) >= MAX_PWM)
                        pwm = MAX_PWM - 1 - INIT_PWM;
                    else if((pwm+INIT_PWM) <= MIN_PWN)
                        pwm = 0;

                    motor_setSpeed(INIT_PWM + pwm);
                }
                else if (error < 0 && change < 0){
                    pwm = (change/distance) * 2000;

                    if((pwm+INIT_PWM) >= MAX_PWM)
                        pwm = MAX_PWM - 1 - INIT_PWM;
                    else if((pwm+INIT_PWM) <= MIN_PWN)
                        pwm = 0;


                    motor_setSpeed(INIT_PWM + pwm);
                }
                else if (error < 0 && change > 0 ){
                    pwm = (change/distance) * 2000;

                    if((pwm+INIT_PWM) >= MAX_PWM)
                        pwm = MAX_PWM - 1 - INIT_PWM;
                    else if((pwm+INIT_PWM) <= MIN_PWN)
                        pwm = MIN_PWN - INIT_PWM;


                    motor_setSpeed(INIT_PWM + pwm);
                }

            }
            // motor_setSpeed(STEADY_SPEED_INCREASE);
        }
        else{
            brushed_motor_stop();
            vTaskDelete(NULL);
        }

        vTaskDelay(DT_MOTOR/ portTICK_PERIOD_MS);
    }
}



void PID_task(void){

    // turn on the motor task
    startMotor = 1;
    float finalPosition = get_absolutePosition() + distance;
    uint32_t out;
    while(1){
        error = finalPosition - get_absolutePosition();
        derror_dt = (error - prevError)/ (DT_ENCODER);
        serror_dt = serror_dt + (((error+prevError)/2)*DT_ENCODER/1000.0);

        // printf("Error is %f\n", error);

        prevError = error;
        IR_output = getIR();
        change = ((error * K_P) + (derror_dt * K_D) + (serror_dt*K_I));
        printf("Pwm is %d\n", pwm);
        printf("Error : %f \tD_Error: %f \tI_Error: %f \tChange: %f\n", error, derror_dt, serror_dt, change);
        if(error<.1 && error>-.1){
            startMotor = 0;
            brushed_motor_stop();
            // vTaskDelete(motor_task);
            serror_dt = 0;
            PID_on = 0;
            vTaskDelete(NULL);
            break;
        }
        
        xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
        vTaskDelay(DT_ENCODER/ portTICK_PERIOD_MS);
    }

}


void mainTask(void){
    char read[10];
    while(1){
        printf("PWM to move in cm: \n");
        gets(read);
        distance = atof(read);
        printf("Distance is %f\n", distance);
        // brushed_motor_forward();
        // motor_setSpeed(5000);
        // vTaskDelay(10000 / portTICK_PERIOD_MS);
        // brushed_motor_stop();
        // xTaskCreate(PID_task, "PID_task", 4096, &distance2move, 5, NULL);
        // vTaskDelete(PID_task);
        // PID_task(distance2move);
        PID_task();
        vTaskDelay(DT_MAIN / portTICK_PERIOD_MS);
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

        char text[100];



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
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

                if(PID_on == 1){
                    sprintf(text, "%.2f cm and position is %.2f and IR is %ld\n", error, get_absolutePosition(), IR_output);
                }

                else{


                    if(strncmp(rx_buffer, "s", 1) == 0){
                        brushed_motor_stop();
                        direction = 's';
                        // sprintf(text, "%.2f cm and position is %.2f\n", error, get_absolutePosition());
                        sprintf(text, "Done\n");
                    }
                    else if (strncmp(rx_buffer, "f", 1) == 0){
                        if(direction != 'f'){
                            brushed_motor_stop();
                            direction = 'f';
                            brushed_motor_forward();
                            #if ENCODER_PRINT
                                PID_on = 1;
                            #endif
                        }
                        
                        speed = atoi(rx_buffer+2);
                        motor_setSpeed(speed);
                        
                        #if ENCODER_PRINT
                            sprintf(text, "Waiting\n");
                        #else
                            sprintf(text, "Done\n");
                        #endif
                    }
                    else if (strncmp(rx_buffer, "b", 1) == 0){
                        if(direction != 'b'){
                            brushed_motor_stop();
                            direction = 'b';
                            brushed_motor_backward();
                        }

                        speed = atoi(rx_buffer+2);
                        motor_setSpeed(speed);
                        sprintf(text, "Done\n");
                    }
                    else if (strncmp(rx_buffer, "d", 1) == 0){
                        
                        brushed_motor_stop();
                        distance = atof(rx_buffer+2);
                        xTaskCreate(PID_task, "PID_task", 4096, NULL, 5, NULL);  
                        PID_on = 1;   
                        sprintf(text, "Wait\n");        

                    }
                    else if(strncmp(rx_buffer, "P", 1) == 0){
                        K_P = atof(rx_buffer + 2);
                    }
                    else if(strncmp(rx_buffer, "I", 1) == 0){
                        K_I = atof(rx_buffer + 2);
                    }
                    else if(strncmp(rx_buffer, "D", 1) == 0){
                        K_D = atof(rx_buffer + 2);
                    }
                    else{
                        sprintf(text, "Done\n");
                    }
                }
                

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

    // UART Init
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
    256, 0, 0, NULL, 0) );
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());


    ESP_ERROR_CHECK(example_connect());



    setbuf(stdout, NULL);

    motor_init();
    // int i = 3000;
    brushed_motor_stop();

    #if IMU_ON
        // initialize IMU
        init_IMU();
    #endif

    encoderInit();
    setRadius(RADIUS);

    startEncoder();
    IR_init();
    // float pos = 0;
    // while(1){

    //     uint32_t out = getIR();
    //     printf("IR output is %ld\n", out);

    //     // pos = absoluteRevolutions();
    //     // printf("%f\n", pos);
    //     vTaskDelay(50 / portTICK_PERIOD_MS);
    // }

    #ifdef CONFIG_EXAMPLE_IPV4
        xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
    #endif
    #ifdef CONFIG_EXAMPLE_IPV6
        xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET6, 5, NULL);
    #endif

    // xTaskCreate(mainTask, "mainTask", 4096, NULL, 5, NULL);
    // xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);

    //testing
    // float absPosition;


    

    // while(1){
        

    //     if(i >= 34500)
    //         i = 34500;
    //     else{
    //         motor_moveFaster(500);
    //         i = i+500;
    //     }

    //     getAccel(accel);
    //     getGyro(gyro);
    //     calculate_angle();

    //     absPosition = get_absolutePosition();
    //     printf("Motor PWM: %d\tAbsolute position %f\t Angle: %f\n", i,absPosition, angles[2]);

        

    //     vTaskDelay(DT_IMU / portTICK_PERIOD_MS);
    // }


}
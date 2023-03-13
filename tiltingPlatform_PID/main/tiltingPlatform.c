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
#include"servo_lib.c"
#include <rom/ets_sys.h>

// SERVO GPIO set to 32

// for testing
#define UDP_ON 0
#define NUM_SAMPLES_TESTING 1500



#define ANGLE_SETPOINT 0
#define GEAR_RATIO 3.917
#define PORT CONFIG_EXAMPLE_PORT
#define ACC_ADD 0x68 // address of IMU
#define DT_IMU 10 // in milliseconds



// PID constants
float K_P = 1.7;
float K_I = 0.0;
float K_D = -2.9;
float prevError = 0;

int time_testing = 0;


// global variables
float angles[] ={0,0,0};
float servoAngle = 0;
float error = 0;
float derror_dt = 0;
float serror_dt = 0;
static const char *TAG = "example";

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

                char text[100];
                sprintf(text,"Tx: %f, Ty: %f, Tz:%f\n", angles[0], angles[1], angles[2]); 
                // sprintf(text,"Ax: %f, Ay: %f, Az:%f, Gx: %f, Gy:%f, Gz:%f\n", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]); 
                int err = sendto(sock, text, strlen(text), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                text[0] = '\0';
                if (err < 0) {
                    // ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            // ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void calculate_angle(){

    // calculating area under graph using trapezoidal
    angles[0] += 0.5*(DT_IMU*portTICK_PERIOD_MS/1000.0)*(float)((round(gyro_prev[0]*100)/100.0) + (round(gyro[0]*100)/100.0));
    angles[1] += round((10*0.5*(DT_IMU*portTICK_PERIOD_MS/1000.0)*(float)((round(gyro_prev[1]*100)/100.0) + (round(gyro[1]*100)/100.0))))/10;
    angles[2] += 0.5*(DT_IMU*portTICK_PERIOD_MS/1000.0)*(float)((round(gyro_prev[2]*100)/100.0) + (round(gyro[2]*100)/100.0));
    // update the previous value
    gyro_prev[0] = gyro[0];
    gyro_prev[1] = gyro[1];
    gyro_prev[2] = gyro[2];

    
}

void PID_tilt(void){

    // calculating the error
    error = ANGLE_SETPOINT - angles[1];
    derror_dt = (error-prevError) / DT_IMU;
    serror_dt = (serror_dt) + (((error + prevError)/2.0)*DT_IMU);

    prevError = error;
    // find the P component of PID
    float change = (error * K_P) + (derror_dt * K_D) + (serror_dt*K_I);
    printf("Angle_y: %f, Change: %f, P_change= %f, I_change= %f, D_change= %f\n", angles[1], change, error*K_P, serror_dt*K_I, derror_dt*K_D);
    if((change*GEAR_RATIO) > SERVO_MAX_DEGREE)
        setAngle(SERVO_MAX_DEGREE);
    else if((change*GEAR_RATIO) < SERVO_MIN_DEGREE)
        setAngle(SERVO_MIN_DEGREE);
    else
        setAngle(change*GEAR_RATIO);

    // printf("Angle_y: %f, Error: %f, Change: %f Servo Angle: %f\n", angles[1], error, change, change*GEAR_RATIO);
}

void IMU_task(void){

    while(1)
    {
        printf("Currently K_P = %.3f K_I = %.3f K_D = %.3f\n", K_P, K_I, K_D);

        int i;
        for(i=0; i<NUM_SAMPLES_TESTING; i++){
            getAccel(accel);
            getGyro(gyro);
            calculate_angle();
            PID_tilt();
            // temporary angle for the servo replace with PID
            // setAngle(GEAR_RATIO*angles[0]);
            vTaskDelay(DT_IMU);  
            time_testing += DT_IMU;
        }

        printf("Testing is done! New PID values? (Enter 'y' if you want to continue with the same values}\n");
        char response[3];

        gets(response);
        printf("%c\n", response[0]);
        if(response[0]!='Y' || response[0]!='y'){
            printf("Currently K_P = %.3f K_I = %.3f K_D = %.3f\n", K_P, K_I, K_D);
            char resValues[10];
            printf("Enter K_P\n");
            gets(resValues);
            K_P = atof(resValues);
            printf("K_P = %.3f\n", K_P);

            printf("Enter K_I\n");
            gets(resValues);
            K_I = atof(resValues);
            printf("K_I = %.3f\n", K_I);

            printf("Enter K_D\n");
            gets(resValues);
            K_D = atof(resValues);
            printf("K_D = %.3f\n", K_D);

            printf("Put on a level surface\n");
            gets(response);

            setAngle(0);
            // calibration of IMU
            calibration(accel, gyro, 50, 0);
            gyro_prev[1] = 0;
            gyro[1] = 0; 

        }
    }
}
    
void app_main(void)
{
    if(UDP_ON){
        // UDP initializations
        ESP_ERROR_CHECK(nvs_flash_init());
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        ESP_ERROR_CHECK(example_connect());
    }

    // initialize servo
    servo_init();
    setAngle(servoAngle);

    // initialize IMU
    init_IMU();
    
    

    if(UDP_ON){
        #ifdef CONFIG_EXAMPLE_IPV4
            xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
        #endif
        #ifdef CONFIG_EXAMPLE_IPV6
            xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET6, 5, NULL);
        #endif
    }

    xTaskCreate(IMU_task, "IMU_task", 4096, NULL, 5, NULL);

}
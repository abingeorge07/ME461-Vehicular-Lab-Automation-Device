/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <arpa/inet.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_err.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "addr_from_stdin.h"
#include <math.h>
#include <stdlib.h>
#include "string.h"
//GPIO task
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
/// ADC
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"


#define I2C_FLAG 1


static const char *TAG = "example";

#define EXAMPLE_PCNT_HIGH_LIMIT 1000
#define EXAMPLE_PCNT_LOW_LIMIT  -1000

#define EXAMPLE_EC11_GPIO_A 34
#define EXAMPLE_EC11_GPIO_B -1

//////////// Def for Accelerometer
#define ADXL343_REG_DATAX0              (0x32)    /**< X-axis data 0 */
#define ADXL343_REG_DATAX1              (0x33)    /**< X-axis data 1 */
#define ADXL343_REG_DATAY0              (0x34)    /**< Y-axis data 0 */
#define ADXL343_REG_DATAY1              (0x35)    /**< Y-axis data 1 */
#define ADXL343_REG_DATAZ0              (0x36)    /**< Z-axis data 0 */
#define ADXL343_REG_DATAZ1              (0x37)    /**< Z-axis data 1 */
#define ADXL343_REG_BW_RATE             (0x2C)    /**< Data rate and power mode control */
#define ADXL343_REG_DATA_FORMAT         (0x31) 
 #define ADXL343_REG_INT_ENABLE          (0x2E)

#define ADXL343_MG2G_MULTIPLIER (0.004F) 

#define SENSORS_GRAVITY_EARTH     (9.80665F) // < Earth's gravity in m/s^2
#define SENSORS_GRAVITY_STANDARD  (SENSORS_GRAVITY_EARTH)
#define ADXL343_REG_POWER_CTL           (0x2D) 
#define ACK_CHECK_EN                       0x1
#define ACK_CHECK_DIS                       0x0
#define ADXL343_REG_DEVID               (0x00) 

typedef enum
{
  ADXL343_RANGE_16_G          = 0b11,   /**< +/- 16g */
  ADXL343_RANGE_8_G           = 0b10,   /**< +/- 8g */
  ADXL343_RANGE_4_G           = 0b01,   /**< +/- 4g */
  ADXL343_RANGE_2_G           = 0b00    /**< +/- 2g (default value) */
} range_t;

typedef enum
{
  ADXL343_DATARATE_3200_HZ    = 0b1111, /**< 3200Hz Bandwidth */
  ADXL343_DATARATE_1600_HZ    = 0b1110, /**< 1600Hz Bandwidth */
  ADXL343_DATARATE_800_HZ     = 0b1101, /**<  800Hz Bandwidth */
  ADXL343_DATARATE_400_HZ     = 0b1100, /**<  400Hz Bandwidth */
  ADXL343_DATARATE_200_HZ     = 0b1011, /**<  200Hz Bandwidth */
  ADXL343_DATARATE_100_HZ     = 0b1010, /**<  100Hz Bandwidth */
  ADXL343_DATARATE_50_HZ      = 0b1001, /**<   50Hz Bandwidth */
  ADXL343_DATARATE_25_HZ      = 0b1000, /**<   25Hz Bandwidth */
  ADXL343_DATARATE_12_5_HZ    = 0b0111, /**< 12.5Hz Bandwidth */
  ADXL343_DATARATE_6_25HZ     = 0b0110, /**< 6.25Hz Bandwidth */
  ADXL343_DATARATE_3_13_HZ    = 0b0101, /**< 3.13Hz Bandwidth */
  ADXL343_DATARATE_1_56_HZ    = 0b0100, /**< 1.56Hz Bandwidth */
  ADXL343_DATARATE_0_78_HZ    = 0b0011, /**< 0.78Hz Bandwidth */
  ADXL343_DATARATE_0_39_HZ    = 0b0010, /**< 0.39Hz Bandwidth */
  ADXL343_DATARATE_0_20_HZ    = 0b0001, /**< 0.20Hz Bandwidth */
  ADXL343_DATARATE_0_10_HZ    = 0b0000  /**< 0.10Hz Bandwidth (default value) */
} dataRate_t;


//////////// Def of I2C
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         400000     // i2c master clock freq
#define ACK_CHECK_EN                       0x1


#define    ACC_ADDRESS         0x53




float changed = 0;
char jsonString[100];
char accelText[100];
float xVal, yVal, zVal;


typedef struct {
    uint64_t event_count;
} example_queue_element_t;





pcnt_unit_handle_t pcnt_unit = NULL;




    
////////////////////////////////////////////////////////////////////////////////// I2C Utility Functions

int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}




// Utility function to scan for i2c device
// Returns the count of I2C devices
static uint8_t i2c_scanner() {
  int32_t scanTimeout = 1000;
  printf("\n>> I2C scanning ..."  "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {printf("- No I2C devices found!" "\n");}

  return count;
}
//i2c init from Master init
static void i2c_master_init(){
  // Debug
  printf("\n>> i2c Config\n");
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  conf.clk_flags = 0;
  err = i2c_param_config(i2c_master_port, &conf);           // Configure
  if (err == ESP_OK) {printf("- parameters: ok\n");}

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK) {printf("- initialized: yes\n");}

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}






///////////////////////////////// i2c

////////////////////////////////////////////////////////////////////////////////
// Read and write to register Functions ///////////////////////////////////////////////////////////

// Write one byte to register (single byte write)
void writeRegister(uint8_t reg, uint8_t data, uint8_t address) {
  // create i2c communication init
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);	// 1. Start (Master write start)
  i2c_master_write_byte(cmd, ( address << 1 ) | WRITE_BIT, I2C_MASTER_ACK); // (Master write slave add + write bit)
  // wait for salve to ack
  i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK); // (Master write register address)
  // wait for slave to ack
  i2c_master_write_byte(cmd, data, I2C_MASTER_ACK);// master write data 
  // wait for slave to ack
  i2c_master_stop(cmd); // 11. Stop
  // i2c communication done and delete
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  // no return here since data is written by master onto slave

}



uint8_t readRegister2(uint8_t reg , uint8_t address) {


  uint8_t data1; //first byte MSB

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_cmd_handle_t cmd1 = i2c_cmd_link_create();

  // Start
  i2c_master_start(cmd);
  // Master write slave address + write bit
  i2c_master_write_byte(cmd, ( address << 1 ) | WRITE_BIT, I2C_MASTER_ACK); 
  // Master write register address + send ack
  i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK); 
  //master stops
  i2c_master_stop(cmd);
  // This starts the I2C communication
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS); 
  i2c_cmd_link_delete(cmd);

  //master starts
  i2c_master_start(cmd1);
  // Master write slave address + read bit
  i2c_master_write_byte(cmd1, ( address  << 1 ) | READ_BIT, I2C_MASTER_ACK);  
  // Master reads in slave ack and data
  i2c_master_read_byte(cmd1, &data1 , I2C_MASTER_NACK);
  // Master nacks and stops.
  i2c_master_stop(cmd1);
  // This starts the I2C communication
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd1, 1000 / portTICK_PERIOD_MS); 
  i2c_cmd_link_delete(cmd1);

  return data1;


}
uint8_t readRegisterSingleByte(uint8_t reg , uint8_t address) {
    // YOUR CODE HERE
  uint8_t data;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // Create a command link
  i2c_master_start(cmd); //  start bit
  i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, I2C_MASTER_ACK); // specify the address
  i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK); // specify the start address
  i2c_master_start(cmd); //  start bit again
  i2c_master_write_byte(cmd, (address << 1) | READ_BIT, I2C_MASTER_ACK); // specify the address
  i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK); // read byte
  i2c_master_stop(cmd); // Send STOP bit
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS); // send all queed commands
  i2c_cmd_link_delete(cmd);
  return data;
}


// read 16 bits (2 bytes)
int16_t read16(uint8_t reg, uint8_t address) {
  // read data 1 then data 2
  // then join them together
  uint8_t data1 = readRegister2(reg, address);
  uint8_t data2 = readRegister2(reg+1, address);  
  //uint8_t data1 = readRegisterSingleByte(reg, address);
  //uint8_t data2 = readRegisterSingleByte(reg+1, address); 
  int16_t dataJ = (((uint16_t)data1) << 8) | data2;
  return dataJ;
}

// read 16 bits (2 bytes)
int16_t read16_accel(uint8_t reg, uint8_t address) {
  // read data 1 then data 2
  // then join them together
  uint8_t data1 = readRegister2(reg, address);
  uint8_t data2 = readRegister2(reg+1, address);  
  //uint8_t data1 = readRegisterSingleByte(reg, address);
  //uint8_t data2 = readRegisterSingleByte(reg+1, address); 
  int16_t dataJ = (((uint16_t)data2) << 8) | data1;
  return dataJ;
}


// Read register (single byte read)
uint16_t readRegister(uint8_t reg, uint8_t address) {
  uint8_t data1; //first byte MSB
  uint8_t data2; //second byte LSB

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_cmd_handle_t cmd1 = i2c_cmd_link_create();

    // printf("reg value is %X\n", address);

  // Start
  i2c_master_start(cmd);
  // Master write slave address + write bit
  i2c_master_write_byte(cmd, ( address << 1 ) | WRITE_BIT, I2C_MASTER_ACK); 
  // Master write register address + send ack
  i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK); 
  //master stops
  i2c_master_stop(cmd);
  // This starts the I2C communication
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS); 
  i2c_cmd_link_delete(cmd);

  //master starts
  i2c_master_start(cmd1);
  // Master write slave address + read bit
  i2c_master_write_byte(cmd1, (address << 1 ) | READ_BIT, I2C_MASTER_ACK);  
  // Master reads in slave ack and data
  i2c_master_read_byte(cmd1, &data1 , I2C_MASTER_ACK);
  i2c_master_read_byte(cmd1, &data2 , I2C_MASTER_NACK);
  // Master nacks and stops.
  i2c_master_stop(cmd1);
  // This starts the I2C communication
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd1, 1000 / portTICK_PERIOD_MS); 
  i2c_cmd_link_delete(cmd1);
  

  uint16_t two_byte_data = (data1 << 8 | data2);
  return two_byte_data;
}





// Setting new I2C address for LIDAR sensor
// void setI2C(uint8_t newAddress){
//     uint8_t dataBytes[2];
//     uint16_t temp;
//     //read reg
//     temp = read16(0x16, LIDAR_DEFAULT_ADDR); // read 2 byte serial number from 0x16 (high byte) and 0x17 (low byte)
//     printf("16 bit serial number 0x%x\n", temp);
//     dataBytes[0] = (uint8_t)(temp >> 8);
//     printf("MSB: %x\n", dataBytes[0]);
//     writeRegister(0x18, dataBytes[0], LIDAR_DEFAULT_ADDR); // write the serial number high byte to 0x18
//     dataBytes[1] =  (uint8_t) (temp&0xFF);
//     printf("LSB 0x%x\n", dataBytes[1]);
//     writeRegister(0x19, dataBytes[1], LIDAR_DEFAULT_ADDR); // write the serial number low byte to 0x19
//     dataBytes[0] = (newAddress << 1); // write the new I2C address to 0x1s
//     printf("0x18 and 0x19: 0x%x\n", readRegister(0x18, LIDAR_DEFAULT_ADDR));
//     writeRegister(0x1a, dataBytes[0], LIDAR_DEFAULT_ADDR);   
//     printf("Value at 0x1a: %x\n", readRegister(0x1a, LIDAR_DEFAULT_ADDR) >> 8);
    
//     writeRegister(0x1e, 0x08, LIDAR_DEFAULT_ADDR);
//     printf("I2C config new: 0x%x\n", readRegister(0x1e, newAddress) >> 8);
//     printf("I2C config old: 0x%x\n", readRegister(0x1e, LIDAR_DEFAULT_ADDR) >> 8);



    
//     printf("done");

// }




////////////////////////////////////// Acceleration

// Get Device ID
int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( ACC_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( ACC_ADDRESS << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}




void setRange(range_t range) {
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister2(ADXL343_REG_DATA_FORMAT, ACC_ADDRESS);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL343_REG_DATA_FORMAT, format, ACC_ADDRESS);

}

range_t getRange(void) {
  /* Red the data format register to preserve bits */
  return (range_t)(readRegister2(ADXL343_REG_DATA_FORMAT, ACC_ADDRESS) & 0x03);
}

dataRate_t getDataRate(void) {
  return (dataRate_t)(readRegister2(ADXL343_REG_BW_RATE, ACC_ADDRESS) & 0x0F);
}



int stringify(char* string, float number){
    int i = 1;
    float temp = number/10;

    int offset = 48;

    while (floor(temp)>0 || floor(temp)<0){
        temp = temp/10;
        i++;
    }

    int j;
    int temp1 = number;
    int temp2;
    int temp3;
    int m = 0;

    if(temp1<0){
        string[m++] = '-';
    }


    for (j = m; j < i; j++){
        temp2 = (int)floor(temp1/(pow(10,i-j-1)));
        string[j] = (char) (temp2 + offset);
        temp3 = temp2* (pow(10,i-j-1));
        temp1 = temp1 - temp3;
    }

    string[i++] = '.';

    temp2 = (int)floor((number - floor(number)) * 10);
    string[i++] = (char) (temp2 + offset);
    string[i++] = '\0';
    return i-1;
}



void convert2string(float* data, int len){
    int i = 0;
    int j = 0;
    int l = 0;
    int negative = 0;
    int temp;
    char tempString[5];

    for (i = 0; i<len ; i++){

        l = 5*i;

        if (data[i] < 0)
        {
            negative = 1;
            data[i] = -1*data[i];
        }
    

        temp = stringify(tempString, data[i]);


        if (temp != 4){
            for (j = 3; j>=(4-temp); j-- ){
                tempString[j] = tempString[j-4+temp];
            }

            for (j = 0; j<(4-temp); j++){
                tempString[j] = '0';
            }
        }

        if(negative){
            negative = 0;
            if(temp == 4){
                for(j = 3; i>0; i--){
                    tempString[i] = tempString[i-1];
                }
            }
            tempString[0] = '-';
        }
    
        for (j=0 ;j<4 ; j++){
            accelText[l++] = tempString[j];
        }

        accelText[l++] = ',';
    }

    accelText[34] = '\0';
    printf("%s\n", accelText);
}

// function to convert from text to json
void jsonify(){

    int i = 0;
    int j = 0;
    int k = 0;
    int m;
    // jsonString[j++] = '\'';
    jsonString[j++] = '{';

    for (i=0; i<7; i++){
        if(i == 0){
            jsonString[j++] = '"';
            jsonString[j++] = 'x';
            jsonString[j++] = 'a';
            jsonString[j++] = 'c';
            jsonString[j++] = 'c';
            jsonString[j++] = 'e';
            jsonString[j++] = 'l';
            jsonString[j++] = '"';
            jsonString[j++] = ':';
            jsonString[j++] = '"';
            for (m = 0; m<4; m++){
                jsonString[j++] = accelText[k++];
            }
            k++;
            jsonString[j++] = '"';
            jsonString[j++] = ',';
        }
        if(i == 1){
            jsonString[j++] = '"';
            jsonString[j++] = 'y';
            jsonString[j++] = 'a';
            jsonString[j++] = 'c';
            jsonString[j++] = 'c';
            jsonString[j++] = 'e';
            jsonString[j++] = 'l';
            jsonString[j++] = '"';
            jsonString[j++] = ':';
            jsonString[j++] = '"';
            for (m = 0; m<4; m++){
                jsonString[j++] = accelText[k++];
            }
            k++;
            jsonString[j++] = '"';
            jsonString[j++] = ',';
        }
        if(i == 2){
            jsonString[j++] = '"';
            jsonString[j++] = 'z';
            jsonString[j++] = 'a';
            jsonString[j++] = 'c';
            jsonString[j++] = 'c';
            jsonString[j++] = 'e';
            jsonString[j++] = 'l';
            jsonString[j++] = '"';
            jsonString[j++] = ':';
            jsonString[j++] = '"';
            for (m = 0; m<4; m++){
                jsonString[j++] = accelText[k++];
            }
            k++;
            jsonString[j++] = '"';
            jsonString[j++] = ',';
        }
        if(i == 3){
            jsonString[j++] = '"';
            jsonString[j++] = 's';
            jsonString[j++] = 'p';
            jsonString[j++] = 'e';
            jsonString[j++] = 'e';
            jsonString[j++] = 'd';
            jsonString[j++] = '"';
            jsonString[j++] = ':';
            jsonString[j++] = '"';
            for (m = 0; m<4; m++){
                jsonString[j++] = accelText[k++];
            }
            k++;
            jsonString[j++] = '"';
            // jsonString[j++] = ',';
        }

    }

    jsonString[j++] = '}';
    // jsonString[j++] = '\'';    
    jsonString[j] = '\0';


} 


// function to print roll and pitch
void calcRP(float x, float y, float z){
  
    float roll = atan2(y , z) * 57.3;
    float pitch = atan2((- x) , sqrt(y * y + y * y)) * 57.3;
    printf("roll: %.2f \t pitch: %.2f \n", roll, pitch);
    float data[4] = {round(10*(x))/10, round(10*(y))/10, round(10*(z))/10, round(10*x)/10};
    convert2string(data, 4);
    changed = 1;
    jsonify();
    printf("%s\n", jsonString);

}



// function to get acceleration
void getAccel(float * xp, float *yp, float *zp) {
  *xp = read16_accel(ADXL343_REG_DATAX0, ACC_ADDRESS) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *yp = read16_accel(ADXL343_REG_DATAY0, ACC_ADDRESS) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *zp = read16_accel(ADXL343_REG_DATAZ0, ACC_ADDRESS) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  calcRP(*xp, *yp, *zp);
  printf("X: %.2f \t Y: %.2f \t Z: %.2f\n", *xp, *yp, *zp);
}










/////////////////////////////////////////////// UDP
// static void udp_client_task(void *pvParameters)
// {
//     char rx_buffer[128];
//     char host_ip[] = HOST_IP_ADDR;
//     int addr_family = 0;
//     int ip_protocol = 0;

//     while (1) {

// #if defined(CONFIG_EXAMPLE_IPV4)
//         struct sockaddr_in dest_addr;
//         dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
//         dest_addr.sin_family = AF_INET;
//         dest_addr.sin_port = htons(PORT);
//         addr_family = AF_INET;
//         ip_protocol = IPPROTO_IP;
// #elif defined(CONFIG_EXAMPLE_IPV6)
//         struct sockaddr_in6 dest_addr = { 0 };
//         inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
//         dest_addr.sin6_family = AF_INET6;
//         dest_addr.sin6_port = htons(PORT);
//         dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
//         addr_family = AF_INET6;
//         ip_protocol = IPPROTO_IPV6;
// #elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
//         struct sockaddr_storage dest_addr = { 0 };
//         ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));
// #endif

//         int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
//         if (sock < 0) {
//             ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
//             break;
//         }

//         // Set timeout
//         struct timeval timeout;
//         timeout.tv_sec = 5;
//         timeout.tv_usec = 0;
//         setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

//         ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

//         while (1) {

//             // sending data
//             if (changed){ // sends the acceleration data

//                 // int err = sendto(sock, accelText, strlen(accelText), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
//                 int err = sendto(sock, jsonString, strlen(jsonString), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

//                 printf("strlen = %d\n", strlen(jsonString));

//                 if (err < 0) {
//                     ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
//                     break;
//                 }
//                 else
//                     changed = 0; 
//             }

    

//             struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
//             socklen_t socklen = sizeof(source_addr);
//             int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

//             // Error occurred during receiving
//             if (len < 0) {
//                 ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
//                 break;
//             }
//             // Data received
//             else {
                
//                 rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
//                 ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
//                 ESP_LOGI(TAG, "%s", rx_buffer);
//                 if (rx_buffer[0] == '0'){
//                     remote ^= 1;
//                     gpio_set_level(ONBOARD_LED, remote);                   
//                     remote_angle = 28;
//                     remote_speed_parameter = 1400;
//                     remote_stop = 0;
//                 }else if (rx_buffer[0] == '1'){
//                     remote_angle = 28;
//                     remote_speed_parameter = 1520;
//                     remote_stop = 0;
//                 }else if (rx_buffer[0] == '2'){
//                     remote_angle = 28;
//                     remote_speed_parameter = 1400;
//                     remote_stop = 1;
//                 }else if (rx_buffer[0] == '3'){
//                     if(remote_angle != 0)remote_angle -= 5;
//                 }else if (rx_buffer[0] == '4'){
//                     if(remote_angle != 40)remote_angle += 5;
//                 }else if (rx_buffer[0] == '5'){
//                     if(remote_speed_parameter != 2100) remote_speed_parameter += 25;
//                 }else if (rx_buffer[0] == '6'){
//                     if(remote_speed_parameter != 1400)remote_speed_parameter -= 25;
//                 }
        
//             }

//             vTaskDelay(100 / portTICK_PERIOD_MS);
//         }

//         if (sock != -1) {
//             ESP_LOGE(TAG, "Shutting down socket and restarting...");
//             shutdown(sock, 0);
//             close(sock);
//         }
//     }
//     vTaskDelete(NULL);
// }









void i2c_functions(){

    while(1) {
        getAccel(&xVal, &yVal, &zVal);       
        vTaskDelay(3000 / portTICK_PERIOD_MS);   
    }
}


void app_main(void)
{
    

    i2c_master_init();
    i2c_scanner();

    

    // initialize i2c function
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
      256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);


    ///// Check for ADXL343
    uint8_t deviceID;
    getDeviceID(&deviceID);
    if (deviceID == 0xE5) {
        printf("\n>> Found ADAXL343\n");
    }
    // Disable interrupts
    writeRegister(ADXL343_REG_INT_ENABLE, 0, ACC_ADDRESS);
    
    // Set range
    setRange(ADXL343_RANGE_16_G);

    // Enable measurements
    writeRegister(ADXL343_REG_POWER_CTL, 0x08, ACC_ADDRESS);
    

    // WIFI 
    // // init for socket
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    // Buggy
    

    if (I2C_FLAG) {
        xTaskCreate(i2c_functions, "i2c_functions", 4096, NULL, 5, NULL);
    }

}

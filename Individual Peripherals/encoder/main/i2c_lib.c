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
#include "accel_reg.h"


//////////// Def of I2C
#define WRITE_BIT                          0x0 // i2c master write
#define READ_BIT                           0x1  // i2c master read
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         400000     // i2c master clock freq
#define ACK_CHECK_EN                       0x1
#define ACC_ADDRESS                        0x68

#define MPU6050_REG_DEVID               (0x00) 
#define ACK_CHECK_DIS                       0x0


//////////// OFFSETS FOR ACCEL VALUES
float accel_offset[3] = {0,0,0};
float gyro_offset[3] = {0,0,0};

// checks if address x is an i2c device
int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}



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



// Get Device ID
int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (ACC_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, MPU6050_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (ACC_ADDRESS << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
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


// writing one byte to a register
void writeRegister(uint8_t reg, uint8_t data, uint8_t address) {

    // start
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    // Decvice# + read or write
    i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, I2C_MASTER_ACK);
    // Reg reading or writing from
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
    // Data to write to or store from slave
    i2c_master_write_byte(cmd, data, I2C_MASTER_ACK);

    //stoop
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

}

// reads register
uint8_t readRegister(uint8_t reg , uint8_t address) {


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

// sets range for gyro
void setGyroRange(uint8_t sel){

    /* Red the data format register to preserve bits */
    uint8_t format = readRegister(GYRO_RANGE, ACC_ADDRESS);

    // bit manipulation
    format &=  0xE7;
    format = format | (sel<<3);
    
    // write back to the register
    writeRegister(GYRO_RANGE, format, ACC_ADDRESS);
}


// sets range for gyro
void setAccelRange(uint8_t sel){

    /* Red the data format register to preserve bits */
    uint8_t format = readRegister(ACC_RANGE, ACC_ADDRESS);

    // bit manipulation
    format &=  0xE7;
    format = format | (sel<<3);
    
    // write back to the register
    writeRegister(ACC_RANGE, format, ACC_ADDRESS);
}

// read 16 bits (reg and reg+1)
int16_t read16(uint8_t reg , uint8_t address){
    
    uint8_t MSB = readRegister(reg, ACC_ADDRESS); 
    uint8_t LSB = readRegister(reg+1, ACC_ADDRESS); 

    int16_t  out = (int16_t) ((MSB << 8) | LSB);

    return out;
}

void getAccel(float* accel){

    accel[0] = read16(REG_AX_H, ACC_ADDRESS)/16384.0 - accel_offset[0];// - (2.0*9.81);
    // accel[0] = (float) val;

    accel[1] = read16(REG_AY_H, ACC_ADDRESS)/16384.0  - accel_offset[1];// - (2.0*9.81);
    // accel[1] = (float) val;

    accel[2] = read16(REG_AZ_H, ACC_ADDRESS)/16384.0  - accel_offset[2];// - (2.0*9.81);
    // accel[2] = (float) val;

}


void getGyro(float* gyro){

    float val = read16(REG_GX_H, ACC_ADDRESS) /131.0;
    gyro[0] = (float) val - gyro_offset[0];

    val = read16(REG_GY_H, ACC_ADDRESS) /131.0;
    gyro[1] = (float) val - gyro_offset[1];

    val = read16(REG_GZ_H, ACC_ADDRESS) /131.0;
    gyro[2] = (float) val - gyro_offset[2];

}


void calibration(float* accel, float* gyro){

  float accelOffset[3];
  float gyroOffset[3];
  int num = 200;
  int i;

  printf("-------------CALIBRATION--------------\n");

  for(i=0; i<num; i++){
    getAccel(accel);
    getGyro(gyro);
  
    if(i>=100){
      accelOffset[0] +=  accel[0];
      accelOffset[1] +=  accel[1];
      accelOffset[2] +=  (accel[2] + 9.8);

      gyroOffset[0] += gyro[0];
      gyroOffset[1] += gyro[1];
      gyroOffset[2] += gyro[2];
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);   

  }

  printf("-------------END CALIBRATION--------------\n");

  for (i=0; i<3; i++){
    accel_offset[i] = accelOffset[i] / (num-100);
    gyro_offset[i] =  gyroOffset[i] / (num-100);
  }

  printf("Offset Values :\nAccel: %f %f %f\n", accel_offset[0], accel_offset[1], accel_offset[2]);
  printf("Gyro: %f %f %f\n", gyro_offset[0], gyro_offset[1], gyro_offset[2]);

}



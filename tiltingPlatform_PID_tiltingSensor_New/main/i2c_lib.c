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
#include "accel_reg.h"


//////////// Def of I2C
#define WRITE_BIT                          0x0 // i2c master write
#define READ_BIT                           0x1  // i2c master read
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          21   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         400000     // i2c master clock freq
#define ACK_CHECK_EN                       0x1
#define ACC_ADDRESS                        0x50
#define I2C_MASTER_ACK                     0x0
#define I2C_MASTER_NACK                     0x1
#define REG                                0x3d
#define MPU6050_REG_DEVID                 (0x00) 
#define ACK_CHECK_DIS                       0x0



//////////// OFFSETS FOR ACCEL VALUES
float angle_offset[3] = {0,0,0};
uint8_t data[6];
int i2c_master_port;

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
  i2c_master_read_byte(cmd, data, I2C_MASTER_ACK);
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


float readRegister(uint8_t reg) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  int ret;
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (ACC_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (ACC_ADDRESS << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read(cmd, data, 5, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, data+5, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);
  i2c_master_stop(cmd);
  i2c_cmd_link_delete(cmd);
  float output = (float)((((uint16_t)data[1]<<8) | data[0]));
  return output;
}


 void getAngle(float* angle){
  float temp;
  temp = readRegister(0x3d);
  temp = (int16_t)(((data[1]<<8) | data[0]));
  angle[0] = (float)((temp/32768.0)*180) - angle_offset[0];


  temp = (int16_t)(((data[3]<<8) | data[2]));
  angle[1] = (float)((temp/32768.0)*180) - angle_offset[1];

  temp = (int16_t)(((data[5]<<8) | data[4]));
  angle[2] = (float)((temp/32768.0)*180) - angle_offset[2];

}

void calibration(void){

  float angleOffset_acc[3];
  float angles[3];
  int num = 100;
  int i;
  float temp;

  printf("-------------CALIBRATION--------------\n");

  for(i=0; i<num; i++){
    getAngle(angles);
    angleOffset_acc[0] += angles[0];
    angleOffset_acc[1] += angles[1];
    angleOffset_acc[2] += angles[2]; 

    vTaskDelay(50 / portTICK_PERIOD_MS);  

  }

  printf("-------------END CALIBRATION--------------\n");

  for (i=0; i<3; i++){
    angle_offset[i] = angleOffset_acc[i] / (float)num;
  }


  printf("Offset Values :\nAngle: %f %f %f\n", angle_offset[0], angle_offset[1], angle_offset[2]);

}

void init_IMU(void){
  i2c_master_init();
  i2c_scanner();

  // UART Init
  ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
  256, 0, 0, NULL, 0) );

  /* Tell VFS to use UART driver */
  esp_vfs_dev_uart_use_driver(UART_NUM_0);
  writeRegister(0x03, 0x0b, ACC_ADDRESS);
  writeRegister(0x01, 0x01, ACC_ADDRESS);
  writeRegister(0x01, 0x02, ACC_ADDRESS);
  writeRegister(0x24, 0x00, ACC_ADDRESS);

  uint8_t deviceID_acc;
  getDeviceID(&deviceID_acc);

  calibration();
}

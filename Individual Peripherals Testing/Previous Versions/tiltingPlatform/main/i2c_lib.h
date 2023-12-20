// initialize for i2c 
static void i2c_master_init();

// searches for i2c devices
static uint8_t i2c_scanner(); 

// start communication with the device
int getDeviceID(uint8_t *data);

// writing one byte to a register
void writeRegister(uint8_t reg, uint8_t data, uint8_t address);

//sets the range of the imu
void setGyroRange(uint8_t sel);

//sets the range of the imu
void setAccelRange(uint8_t sel);


// gets Accel Data
void getAccel(float* accel);

// gets gyro Data
void getGyro( float* gyro);

// used to calibrate the IMU
void calibration(float* accel, float* gyro);

// initializes the IMU (sets the range)
void init_IMU()
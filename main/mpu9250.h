#ifndef MPU6500_H
#define MPU6500_H

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spi.h"


// Địa chỉ thanh ghi của MPU6500
#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38

#define DMP_INT_STATUS   0x39  // Check DMP interrupt

#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C

#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 

#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6500 0x75 // Should return 0x70
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E


// // Địa chỉ của cảm biến MPU6500
// #define WHO_AM_I_RESP    0x71

// Set initial input parameters
enum Ascale {
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum Gscale {
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
};

uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
float aRes, gRes;
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
float gyroBias[3] = { 0, 0, 0 }, accelBias[3] = { 0, 0, 0 }; // Bias corrections for gyro and accelerometer
float ax, ay, az, gx, gy, gz; // variables to hold latest sensor data values 
int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];

int delt_t = 0; // used to control display output rate
int count = 0;  // used to control display output rate
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float pitch, yaw, roll;
float deltat = 0.0f;                             // integration interval for both filter schemes
int lastUpdate = 0, firstUpdate = 0, Now = 0;    // used to calculate integration interval                               // used to calculate integration interval
float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };           // vector to hold quaternion
float eInt[3] = { 0.0f, 0.0f, 0.0f };              // vector to hold integral error for Mahony method
int fdMPU6500;
float PI, GyroMeasError, beta, GyroMeasDrift, zeta;

static const char *TAG = "MPU6500";

void MPU6500() {
    // Parameters for 6 DoF sensor fusion calculations
    PI = 3.14159265358979323846f;
    GyroMeasError = PI * (60.0f / 180.0f); // Gyroscope measurement error in rads/s (start at 60 deg/s)
    beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // Compute beta
    GyroMeasDrift = PI * (1.0f / 180.0f);     // Gyroscope measurement drift in rad/s/s
    zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; // Compute zeta

    ESP_LOGI("MPU6500", "GyroMeasError: %f, GyroMeasDrift: %f", GyroMeasError, GyroMeasDrift);
    
    // Wake up MPU6500
    spi_write(spi_handle_mpu, PWR_MGMT_1, 0x00); // Set power management register to wake up the sensor

    // Verify device IDs
	uint8_t whoAmI = spi_read(spi_handle_mpu, WHO_AM_I_MPU6500); // WHO_AM_I register
    ESP_LOGI(TAG, "MPU6500 WHO_AM_I = 0x%02X", whoAmI);

    if (whoAmI != 0x70) {
        ESP_LOGE(TAG, "MPU6500 not found! (WHO_AM_I = 0x%02X)", whoAmI);
    } else {
        ESP_LOGI(TAG, "MPU6500 initialization successful.");
    }
}

void readBytes(uint8_t subAddress, uint8_t count, uint8_t * dest){
	uint8_t data = 0;
	for (uint8_t i = 0; i < count; i++){
		data = spi_read(spi_handle_mpu, subAddress + i);
		dest[i] = data;
	}
}

float getGres() {
	switch (Gscale){
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case GFS_250DPS:
			gRes = 250.0 / 32768.0;
			break;
		case GFS_500DPS:
			gRes = 500.0 / 32768.0;
			break;
		case GFS_1000DPS:
			gRes = 1000.0 / 32768.0;
			break;
		case GFS_2000DPS:
			gRes = 2000.0 / 32768.0;
			break;
	}
	return gRes;
}

float getAres() {
	switch (Ascale){
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case AFS_2G:
			aRes = 2.0 / 32768.0;
			break;
		case AFS_4G:
			aRes = 4.0 / 32768.0;
			break;
		case AFS_8G:
			aRes = 8.0 / 32768.0;
			break;
		case AFS_16G:
			aRes = 16.0 / 32768.0;
			break;
	}
	return aRes;
}

void readAccelData(int16_t * destination){
	uint8_t rawData[6];  // x/y/z accel register data stored here
	readBytes(ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
	destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
}

void readGyroData(int16_t * destination){
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
	destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
}

int16_t readTempData(){
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	readBytes(TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
	return (int16_t)(((int16_t)rawData[0]) << 8 | rawData[1]);  // Turn the MSB and LSB into a 16-bit value
}

float readTempInC(){
	return (float)readTempData()/ 333.87 + 21.0;
}

void resetMPU6500() {
	// reset device
	spi_write(spi_handle_mpu, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	vTaskDelay(100 / portTICK_PERIOD_MS);
}

void initMPU6500(){
	// Initialize MPU6500 device
	// wake up device
	spi_write(spi_handle_mpu, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
	vTaskDelay(10 / portTICK_PERIOD_MS); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

	// get stable time source
	spi_write(spi_handle_mpu, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	spi_write(spi_handle_mpu,CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	spi_write(spi_handle_mpu,SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = spi_read(spi_handle_mpu,GYRO_CONFIG);
	spi_write(spi_handle_mpu,GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
	spi_write(spi_handle_mpu,GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	spi_write(spi_handle_mpu,GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

	// Set accelerometer configuration
	c = spi_read(spi_handle_mpu,ACCEL_CONFIG);
	spi_write(spi_handle_mpu,ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
	spi_write(spi_handle_mpu,ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	spi_write(spi_handle_mpu,ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer 

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = spi_read(spi_handle_mpu,ACCEL_CONFIG2);
	spi_write(spi_handle_mpu,ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	spi_write(spi_handle_mpu,ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
	// can join the I2C bus and all can be controlled by the Arduino as master
	spi_write(spi_handle_mpu,INT_PIN_CFG, 0x22);
	spi_write(spi_handle_mpu,INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU6500(float * dest1, float * dest2){
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device, reset all registers, clear gyro and accelerometer bias registers
	spi_write(spi_handle_mpu, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	vTaskDelay(100 / portTICK_PERIOD_MS);

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	spi_write(spi_handle_mpu, PWR_MGMT_1, 0x01);
	spi_write(spi_handle_mpu, PWR_MGMT_2, 0x00);
	vTaskDelay(200 / portTICK_PERIOD_MS);

	// Configure device for bias calculation
	spi_write(spi_handle_mpu,INT_ENABLE, 0x00);   // Disable all interrupts
	spi_write(spi_handle_mpu,FIFO_EN, 0x00);      // Disable FIFO
	spi_write(spi_handle_mpu,PWR_MGMT_1, 0x00);   // Turn on internal clock source
	// spi_write(spi_handle_mpu,I2C_MST_CTRL, 0x00); // Disable I2C master
	spi_write(spi_handle_mpu,USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	spi_write(spi_handle_mpu,USER_CTRL, 0x0C);    // Reset FIFO and DMP
	vTaskDelay(15 / portTICK_PERIOD_MS);

	// Configure MPU6500 gyro and accelerometer for bias calculation
	spi_write(spi_handle_mpu,CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	spi_write(spi_handle_mpu,SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	spi_write(spi_handle_mpu,GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	spi_write(spi_handle_mpu,ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	spi_write(spi_handle_mpu,USER_CTRL, 0x40);   // Enable FIFO  
	spi_write(spi_handle_mpu,FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-6500)
	vTaskDelay(80 / portTICK_PERIOD_MS);

	// At end of sample accumulation, turn off FIFO sensor read
	spi_write(spi_handle_mpu,FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytes(FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		readBytes(FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t)accel_temp[1];
		accel_bias[2] += (int32_t)accel_temp[2];
		gyro_bias[0] += (int32_t)gyro_temp[0];
		gyro_bias[1] += (int32_t)gyro_temp[1];
		gyro_bias[2] += (int32_t)gyro_temp[2];
	}
	accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t)packet_count;
	accel_bias[2] /= (int32_t)packet_count;
	gyro_bias[0] /= (int32_t)packet_count;
	gyro_bias[1] /= (int32_t)packet_count;
	gyro_bias[2] /= (int32_t)packet_count;

	if (accel_bias[2] > 0L) { accel_bias[2] -= (int32_t)accelsensitivity; }  // Remove gravity from the z-axis accelerometer bias calculation
	else { accel_bias[2] += (int32_t)accelsensitivity; }

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;

	dest1[0] = (float)gyro_bias[0] / (float)gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
	dest1[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
	dest1[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

	int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	readBytes( XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
	readBytes( YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int16_t)((int16_t)data[0] << 8) | data[1];
	readBytes( ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int16_t)((int16_t)data[0] << 8) | data[1];

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++) {
		if (accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0]) & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1]) & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2]) & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Output scaled accelerometer biases for manual subtraction in the main program
	dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
	dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
	dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

void MPU6500SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t selfTest[6];
	int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
	float factoryTrim[6];
	uint8_t FS = 0;

	spi_write(spi_handle_mpu, SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
	spi_write(spi_handle_mpu, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	spi_write(spi_handle_mpu, GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
	spi_write(spi_handle_mpu, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	spi_write(spi_handle_mpu, ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

	for (int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

		readBytes( ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

		readBytes( GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	spi_write(spi_handle_mpu, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	spi_write(spi_handle_mpu, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	vTaskDelay(25 / portTICK_PERIOD_MS);
	for (int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

		readBytes( ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

		readBytes( GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	spi_write(spi_handle_mpu, ACCEL_CONFIG, 0x00);
	spi_write(spi_handle_mpu, GYRO_CONFIG, 0x00);
	vTaskDelay(25 / portTICK_PERIOD_MS); // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = spi_read(spi_handle_mpu, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = spi_read(spi_handle_mpu, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = spi_read(spi_handle_mpu, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = spi_read(spi_handle_mpu, SELF_TEST_X_GYRO); // X-axis gyro self-test results
	selfTest[4] = spi_read(spi_handle_mpu, SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
	selfTest[5] = spi_read(spi_handle_mpu, SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		destination[i] = 100.0*((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]; // Report percent differences
		destination[i + 3] = 100.0*((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3]; // Report percent differences
	}
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Normalize accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // Avoid division by zero
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Gradient descent algorithm corrective step
    s1 = -2.0f * q3 * (2.0f * (q2 * q4 - q1 * q3) - ax) + 2.0f * q2 * (2.0f * (q1 * q2 + q3 * q4) - ay);
    s2 = 2.0f * q4 * (2.0f * (q2 * q4 - q1 * q3) - ax) + 2.0f * q1 * (2.0f * (q1 * q2 + q3 * q4) - ay) - 4.0f * q2 * (1.0f - 2.0f * (q2 * q2 + q3 * q3) - az);
    s3 = -2.0f * q1 * (2.0f * (q2 * q4 - q1 * q3) - ax) + 2.0f * q4 * (2.0f * (q1 * q2 + q3 * q4) - ay) - 4.0f * q3 * (1.0f - 2.0f * (q2 * q2 + q3 * q3) - az);
    s4 = 2.0f * q2 * (2.0f * (q2 * q4 - q1 * q3) - ax) + 2.0f * q3 * (2.0f * (q1 * q2 + q3 * q4) - ay);

    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // Normalize step magnitude
    norm = 1.0f / norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;

    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // Normalize quaternion
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Normalize accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // Avoid division by zero
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Estimated direction of gravity
    vx = 2.0f * (q2 * q4 - q1 * q3);
    vy = 2.0f * (q1 * q2 + q3 * q4);
    vz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    if (Ki > 0.0f)
    {
        eInt[0] += ex; // Accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
    }
    else
    {
        eInt[0] = 0.0f;
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalize quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

#endif // MPU6500_H

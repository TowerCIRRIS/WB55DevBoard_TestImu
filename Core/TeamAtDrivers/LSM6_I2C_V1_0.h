/**
  ******************************************************************************
  * @file   LSM6_I2C_v1.h
  * @author Simon Latour
  * @brief  LSM 6 IMU Driver created to be as much platform independent as possible
  * @version 1.0
  * @date 2022-06-28
  *
  * @copyright Copyright (c) 2022
  *
  */


#ifndef __LSM6_I2C_H__
#define __LSM6_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
//////////////////////////////////////////////////////
/// Accelerometer configuration constants
//////////////////////////////////////////////////////
#define LSM6_ACCEL_SCALE_2G		 (0b00 << 2) // 0x00
#define LSM6_ACCEL_SCALE_4G		 (0b10 << 2) // 0x10
#define LSM6_ACCEL_SCALE_8G		 (0b11 << 2) // 0x18
#define LSM6_ACCEL_SCALE_16G	 (0b01 << 2) // 0x08

//  Output data rate for accelerometer, see table 52 of datasheet
#define LSM6_ACCEL_ODR_POWER_DOWN	(0b0000 << 4)
#define LSM6_ACCEL_ODR_1_6			(0b1011 << 4)
#define LSM6_ACCEL_ODR_12_5			(0b0001 << 4)
#define LSM6_ACCEL_ODR_26			(0b0010 << 4)
#define LSM6_ACCEL_ODR_52			(0b0011 << 4)
#define LSM6_ACCEL_ODR_104			(0b0100 << 4)
#define LSM6_ACCEL_ODR_208			(0b0101 << 4)
#define LSM6_ACCEL_ODR_416			(0b0110 << 4)
#define LSM6_ACCEL_ODR_833			(0b0111 << 4)
#define LSM6_ACCEL_ODR_1666			(0b1000 << 4)
#define LSM6_ACCEL_ODR_3332			(0b1001 << 4)
#define LSM6_ACCEL_ODR_6664			(0b1010 << 4)

#define LSM6_SENSITIVITY_ACCELEROMETER_2  0.000061
#define LSM6_SENSITIVITY_ACCELEROMETER_4  0.000122
#define LSM6_SENSITIVITY_ACCELEROMETER_8  0.000244
#define LSM6_SENSITIVITY_ACCELEROMETER_16 0.000488


//////////////////////////////////////////////////////
/// Gyroscope configuration constants
//////////////////////////////////////////////////////
#define LSM6_GYRO_SCALE_125_DPS  	(0b001)
#define LSM6_GYRO_SCALE_250_DPS  	(0b000)
#define LSM6_GYRO_SCALE_500_DPS  	(0b010)
#define LSM6_GYRO_SCALE_1000_DPS  	(0b100)
#define LSM6_GYRO_SCALE_2000_DPS 	(0b110)

//  Output data rate for accelerometer, see table 52 of datasheet

//  Output data rate for gyroscope, see table 52 of datasheet
#define LSM6_GYRO_ODR_POWER_DOWN	(0b0000 << 4)
#define LSM6_GYRO_ODR_1_6			(0b1011 << 4)
#define LSM6_GYRO_ODR_12_5			(0b0001 << 4)
#define LSM6_GYRO_ODR_26			(0b0010 << 4)
#define LSM6_GYRO_ODR_52			(0b0011 << 4)
#define LSM6_GYRO_ODR_104			(0b0100 << 4)
#define LSM6_GYRO_ODR_208			(0b0101 << 4)
#define LSM6_GYRO_ODR_416			(0b0110 << 4)
#define LSM6_GYRO_ODR_833			(0b0111 << 4)
#define LSM6_GYRO_ODR_1666			(0b1000 << 4)
#define LSM6_GYRO_ODR_3332			(0b1001 << 4)
#define LSM6_GYRO_ODR_6664			(0b1010 << 4)

#define LSM6_SENSITIVITY_GYROSCOPE_125    0.004375
#define LSM6_SENSITIVITY_GYROSCOPE_250    0.00875
#define LSM6_SENSITIVITY_GYROSCOPE_500    0.0175
#define LSM6_SENSITIVITY_GYROSCOPE_1000   0.035
#define LSM6_SENSITIVITY_GYROSCOPE_2000   0.070

//////////////////////////////////////////////////////
/// General IMU constants
//////////////////////////////////////////////////////

// For data access
#define X_AXIS	0
#define Y_AXIS  1
#define Z_AXIS  2


//Used by library for device selection
#define XLG_SELECT	0 // Select accelerometer+gyroscope
#define MAG_SELECT  1 // Select magnetometer

//////////////////////////////////////////////////////
/// IMU Possible ADDRESSES
//////////////////////////////////////////////////////

//Master address
#define LSM6_WHO_AM_I           0x0f

/// ACCEL/GYRO ADDRESS
#define IMU_I2C_ADRESS_ACC1 	0X6a	// when SA0=0
#define IMU_I2C_ADRESS_ACC2 	0X6b	// when SA0=1 (Default when pin not connected)


//////////////////////////////////////////////////////
/// Errors
//////////////////////////////////////////////////////
#define IMU_ERRORS_NONE 	0 // HAL_I2C_ERROR_NONE    	//  0 = No error

//lIBRARY RELATED ERRORS
#define IMU_ERRORS_MAG_NOT_INIT		0X500U	// Magnetometer not initialized
#define IMU_ERRORS_ACCEL_NOT_INIT	0X501U	// Accelerometer not initialized
#define IMU_ERRORS_GYRO_NOT_INIT	0X502U	// Gyroscope not initialized
#define IMU_ERRORS_ACCGYRO_NOT_INIT	0X503U	// Accelerometer and Gyroscope not initialized
#define IMU_ERRORS_MAG_POWER_DOWN	0X504U	// Magnetometer not initialized

////////////////////////////////////////////////////
/// IMU Handle type
/// ////////////////////////////////////////////////

// Callback functions model for I2C
typedef uint8_t (*sendI2cDataCallback_t)(uint8_t address, uint8_t* buffer, int length, long timeout, bool stopBit);
typedef uint8_t (*receiveI2cDataCallback_t)(uint8_t address, uint8_t* buffer, int length, long timeout);

// Callback functions model for chipSelect
typedef void (*chipSelectControlCallback_t)(uint8_t state);

typedef struct {
	uint8_t accelerationScale;
	uint8_t accelerationOutputDataRate;
	uint8_t gyroScale;
	uint8_t gyroOutputDataRate;
	uint8_t magnetoMeterScale;

	// Set this value to match you hardware configuration.
	// Default value 1: SDO is pulled up on imu boards, if you use a imu module with SDO unconnected, use: 1
	// To use "chip select" functionality or if your IMU board SDO pin is connected to ground, use 0
	uint8_t AddressSelectSdo;


	// Callback for I2C send and receive functions
	sendI2cDataCallback_t 		sendI2cCallback;
	receiveI2cDataCallback_t	receiveI2cCallback;

	// If you wish to use chip select functionality, put your pin an port here.
	// Otherwise set to NULL
	chipSelectControlCallback_t chipSelectCallback;

}imuUserConfig_t;


typedef struct{

	uint8_t ctrl2_g;
	uint8_t reg2;
	uint8_t reg3;
	uint8_t reg4;
	uint8_t reg5;
	uint8_t ctrl1_xl;
	uint8_t reg7;
	uint8_t reg8;
	uint8_t reg9;
	uint8_t reg10;

}agControlRegisters_t;

typedef struct{

	uint8_t reg1;
	uint8_t reg2;
	uint8_t reg3;
	uint8_t reg4;
	uint8_t reg5;

}magControlRegisters_t;

typedef struct {
	uint16_t gAddress;
	//uint16_t mAddress;

	// Accel/Gyro config
	uint8_t agInitDone;
	uint8_t accelInitDone;
	uint8_t gyroInitDone;
	float accelSensitivityFactor;
	float gyroSensitivityFactor;
	agControlRegisters_t agCtrlReg;

}lsm9ds1LibConfig_t;


typedef struct{

	// Only modify values in userConfig
	imuUserConfig_t userConfig;

	// Used by library, do not modify
	lsm9ds1LibConfig_t libConfig;

}lsm6Handle_t;


////////////////////////////////////////////////////
/// User Functions
///////////////////////////////////////////////////

/**
 * @fn int imuInit(imuHandle_t*)
 * @brief Initialize the complete IMU ( Accel, Gyro) and send config to device
 *
 * @param imuHandle	handle to the imu
 * @return	0 = no errors, see IMU_ERRORS_ for error codes
 */
int lsm6_imuInit(lsm6Handle_t * imuHandle);

/**
 * @fn int accelInit(lsm6ds1Handle_t*)
 * @brief Initialize only the Accelerometer and send config to device
 *
 * @param imuHandle handle to the imu
 * @return 0 = no errors, see IMU_ERRORS_ for error codes
 */
int lsm6_accelInit(lsm6Handle_t * imuHandle);

/**
 * @fn int gyroInit(lsm6ds1Handle_t*)
 * @brief  Initialize only the Gyroscope and send config to device
 *
 * @param imuHandle handle to the imu
 * @return 0 = no errors, see IMU_ERRORS_ for error codes
 */
int lsm9ds1_gyroInit(lsm6Handle_t * imuHandle);

/**
 * @fn int accelGyroInit(imuHandle_t*)
 * @brief Initialize only the Gyroscope and Accelerometer and send config to device
 *
 * @param imuHandle handle to the imu
 * @return 0 = no errors, see IMU_ERRORS_ for error codes
 */
int lsm9ds1_accelGyroInit(lsm6Handle_t * imuHandle);

/**
 * @fn int getAccelerationM(imuHandle_t*, float*)
 * @brief  Get acceleration in meters/seconds2
 *
 * @param imuHandle handle to the IMU
 * @param receivedData	Pointer to the float buffer to receive the data
 * @return 0 = no errors, see IMU_ERRORS_ for error codes
 */
int lsm6_getAccelerationM(lsm6Handle_t * imuHandle, float *receivedData);

/**
 * @fn int getAccelerationG(imuHandle_t*, float*)
 * @brief
 *
 * @param imuHandle handle to the IMU
 * @param receivedData
 * @return 0 = no errors, see IMU_ERRORS_ for error codes
 */
int lsm6_getAccelerationG(lsm6Handle_t * imuHandle, float *receivedData);

/**
 * @fn int getGyroscope(imuHandle_t*, float*)
 * @brief The the scaled gyroscope value
 *
 * @param imuHandle handle to the IMU
 * @param receivedData	Pointer to the float buffer to receive the data
 * @return				0 = no errors, see IMU_ERRORS_ for error codes
 */
int lsm6_getGyroscope(lsm6Handle_t * imuHandle, float *receivedData);

////////////////////////////////////////////////////
/// Lower level Functions
///////////////////////////////////////////////////

/**
 * @fn int getRawAcceleration(imuHandle_t*, int*)
 * @brief Raw acceleration data received from the sensor in meters per seconds
 *
 * @param imuHandle handle to the IMU
 * @param receivedData	Pointer to the float buffer to receive the data
 * @return				0 = no errors, see IMU_ERRORS_ for error codes
 */
int lsm6_getRawAcceleration(lsm6Handle_t * imuHandle, int *receivedData);

/**
 * @fn int getRawGyroscope(imuHandle_t*, int*)
 * @brief Raw Gyroscope data received from the sensor
 *
 * @param imuHandle handle to the IMU
 * @param receivedData	Pointer to the float buffer to receive the data
 * @return				0 = no errors, see IMU_ERRORS_ for error codes
 */
int lsm6_getRawGyroscope(lsm6Handle_t * imuHandle, int *receivedData);

/**
 * @fn int imuSendRegister(imuHandle_t*, uint8_t, uint8_t, uint8_t)
 * @brief Send register data to the IMU
 *
 * @param imuHandle	handle to the IMU
 * @param xlgMagSelect	select magnetometer register or accel/gy with: XLG_SELECT/ MAG_SELECT ( not used but kep to mathe the lsm9 version
 * @param regID ID 	of the register
 * @param regValue	value of the register
 * @return 0 = no errors, see IMU_ERRORS_ for error codes
 */
int lsm6_imuSendRegister(lsm6Handle_t * imuHandle, uint8_t xlgMagSelect, uint8_t regID, uint8_t regValue);

/**
 * @fn void tryChipSelect(imuHandle_t*)
 * @brief Toggle pin to select active IMU if chip select logic is enabled
 *
 * @param imuHandle
 */
void lsm6_tryChipSelect(lsm6Handle_t * imuHandle);

/**
 * @fn void tryChipDeselect(imuHandle_t*)
 * @brief Toggle pin to deselect active IMU if chip select logic is enabled
 *
 * @param imuHandle handle to the IMU
 */
void lsm6_tryChipDeSelect(lsm6Handle_t * imuHandle);






#ifdef __cplusplus
}
#endif

#endif





#include <LSM6_I2C_V1_0.h>
#include <stddef.h>


/////////////////////////////////////////
// LSM6D Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////

#define LSM6_CTRL1_XL 		0x10
#define LSM6_CTRL2_G		0x11

#define LSM6_OUTX_L_G		0x22
#define LSM6_OUTY_L_G		0x24
#define LSM6_OUTZ_L_G		0x26

#define LSM6_OUTX_L_XL		0x28
#define LSM6_OUTY_L_XL		0x2A
#define LSM6_OUTZ_L_XL		0x2C

//#define CTRL_REG2_M_i2c			0x21
//#define CTRL_REG3_M_i2c			0x22
//#define CTRL_REG4_M_i2c			0x23
//#define CTRL_REG5_M_i2c			0x24
//#define STATUS_REG_M_i2c		0x27
//#define OUT_X_L_M_i2c			0x28 //pareil
//#define OUT_X_H_M_i2c			0x29 //pareil
//#define OUT_Y_L_M_i2c			0x2A //pareil
//#define OUT_Y_H_M_i2c			0x2B //pareil
//#define OUT_Z_L_M_i2c			0x2C //pareil
//#define OUT_Z_H_M_i2c			0x2D //pareil
//#define INT_CFG_M_i2c			0x30
//#define INT_SRC_M_i2c			0x30
//#define INT_THS_L_M_i2c			0x32
//#define INT_THS_H_M_i2c			0x33

////////////////////////////////
//// LSM9DS1 WHO_AM_I Responses //
//////////////////////////////////
//#define WHO_AM_I_AG_RSP_i2c		0x68
//#define WHO_AM_I_M_RSP_i2c		0x3D



////////////////////////////////////////////////////
/// Private Functions
///////////////////////////////////////////////////

void lsm6_tryChipSelect(lsm6Handle_t * imuHandle)
{
	if(imuHandle->userConfig.chipSelectCallback != NULL)
	{
		// If chip select is used
	 	imuHandle->userConfig.chipSelectCallback(imuHandle->userConfig.AddressSelectSdo);
	}
}

void lsm6_tryChipDeSelect(lsm6Handle_t * imuHandle)
{
	if(imuHandle->userConfig.chipSelectCallback != NULL)
	{
		// If chip select is used
		imuHandle->userConfig.chipSelectCallback(!imuHandle->userConfig.AddressSelectSdo);
	}
}

////////////////////////////////////////////////////
/// Public Functions
///////////////////////////////////////////////////

int lsm6_imuInit(lsm6Handle_t * imuHandle)
{
	int readerror=0;
	int statusReturn = 0;

	readerror = lsm9ds1_accelGyroInit(imuHandle);
	if(readerror)
	{
		statusReturn = readerror;
	}

	return statusReturn;
}

int lsm6_accelInit(lsm6Handle_t * imuHandle)
{
	int readerror=0;
	int statusReturn = 0;

	// Set I2C addresses
	if(imuHandle->userConfig.AddressSelectSdo)
	{
		imuHandle->libConfig.gAddress = IMU_I2C_ADRESS_ACC2 << 1;
	}
	else
	{
		imuHandle->libConfig.gAddress = IMU_I2C_ADRESS_ACC1 << 1;
	}

	// Set conversion factors
	switch(imuHandle->userConfig.accelerationScale)
	{
		case LSM6_ACCEL_SCALE_2G:
			imuHandle->libConfig.accelSensitivityFactor = LSM6_SENSITIVITY_ACCELEROMETER_2;
			break;

		case LSM6_ACCEL_SCALE_4G:
			imuHandle->libConfig.accelSensitivityFactor = LSM6_SENSITIVITY_ACCELEROMETER_4;
			break;

		case LSM6_ACCEL_SCALE_8G:
			imuHandle->libConfig.accelSensitivityFactor = LSM6_SENSITIVITY_ACCELEROMETER_8;
			break;

		case LSM6_ACCEL_SCALE_16G:
			imuHandle->libConfig.accelSensitivityFactor = LSM6_SENSITIVITY_ACCELEROMETER_16;
			break;

		default:
			break;
	}

	lsm6_tryChipSelect(imuHandle);

	// Set register values
	imuHandle->libConfig.agCtrlReg.ctrl1_xl = imuHandle->userConfig.accelerationScale | imuHandle->userConfig.accelerationOutputDataRate;	// Set acceleration scale

	// Send register config values to IMU IC
	readerror = lsm6_imuSendRegister(imuHandle, XLG_SELECT, LSM6_CTRL1_XL, imuHandle->libConfig.agCtrlReg.ctrl1_xl);

	if(readerror)
	{
		statusReturn = readerror;
	}

	lsm6_tryChipDeSelect(imuHandle);

	if(statusReturn == 0)
	{
		imuHandle->libConfig.accelInitDone = 1;
	}

	return statusReturn;

}

int lsm9ds1_gyroInit(lsm6Handle_t * imuHandle)
{
	int readerror=0;
	int statusReturn = 0;

	// Set I2C addresses
	if(imuHandle->userConfig.AddressSelectSdo)
	{
		imuHandle->libConfig.gAddress = IMU_I2C_ADRESS_ACC2 << 1;
	}
	else
	{
		imuHandle->libConfig.gAddress = IMU_I2C_ADRESS_ACC1 << 1;
	}

	// Set conversion factors
	switch(imuHandle->userConfig.gyroScale)
	{
		case LSM6_GYRO_SCALE_125_DPS:
				imuHandle->libConfig.gyroSensitivityFactor = LSM6_SENSITIVITY_GYROSCOPE_125;
				break;

		case LSM6_GYRO_SCALE_250_DPS:
			imuHandle->libConfig.gyroSensitivityFactor = LSM6_SENSITIVITY_GYROSCOPE_250;
			break;

		case LSM6_GYRO_SCALE_500_DPS:
			imuHandle->libConfig.gyroSensitivityFactor = LSM6_SENSITIVITY_GYROSCOPE_500;
			break;

		case LSM6_GYRO_SCALE_1000_DPS:
			imuHandle->libConfig.gyroSensitivityFactor = LSM6_SENSITIVITY_GYROSCOPE_1000;
			break;

		case LSM6_GYRO_SCALE_2000_DPS:
			imuHandle->libConfig.gyroSensitivityFactor = LSM6_SENSITIVITY_GYROSCOPE_2000;
			break;

		default:
			break;
	}

	lsm6_tryChipSelect(imuHandle);

	// Set register values
	imuHandle->libConfig.agCtrlReg.ctrl2_g = imuHandle->userConfig.gyroScale | imuHandle->userConfig.gyroOutputDataRate; 			// Set Gyroscope scale

	// Send register config values to IMU IC
	readerror = lsm6_imuSendRegister(imuHandle, XLG_SELECT,LSM6_CTRL2_G,imuHandle->libConfig.agCtrlReg.ctrl2_g);
	if(readerror)
	{
		statusReturn = readerror;
	}

	lsm6_tryChipDeSelect(imuHandle);

	if(statusReturn == 0)
	{
		imuHandle->libConfig.gyroInitDone = 1;
	}

	return statusReturn;


}

int lsm9ds1_accelGyroInit(lsm6Handle_t * imuHandle)
{
	int readerror=0;
	//int statusReturn = 0;

	readerror = lsm6_accelInit(imuHandle);

	if(readerror)
	{
		return readerror;
	}

	readerror = lsm9ds1_gyroInit(imuHandle);
	if(readerror)
	{
		return readerror;
	}

	return IMU_ERRORS_NONE;


}


int lsm6_imuSendRegister(lsm6Handle_t * imuHandle, uint8_t xlgMagSelect, uint8_t regID, uint8_t regValue)
{
	int readError;
	uint8_t data[1] ;
	uint8_t address;

//	if(xlgMagSelect == XLG_SELECT) // Only lsm9 has that logic
//	{
		address = imuHandle->libConfig.gAddress;
//	}
//	else
//	{
//		address = imuHandle->libConfig.mAddress;
//	}

	// Configure Accelerometer scale
	data[0] = regID;
	data[1] = regValue;

	readError = imuHandle->userConfig.sendI2cCallback(address,data,2,500,true);

	return readError;
}



int lsm6_getRawAcceleration(lsm6Handle_t * imuHandle, int *receivedData)
{
	int readError;
	uint8_t buf[10];
	uint8_t data[1] ;

	if(!imuHandle->libConfig.accelInitDone)
	{
		return IMU_ERRORS_ACCGYRO_NOT_INIT;
	}

	data[0] = LSM6_OUTX_L_XL; // Indicate that we want to read Acceleration value register

	lsm6_tryChipSelect(imuHandle);

	readError = imuHandle->userConfig.sendI2cCallback(imuHandle->libConfig.gAddress, data, 1, 500, false);


	if(readError == IMU_ERRORS_NONE)
	{
		imuHandle->userConfig.receiveI2cCallback(imuHandle->libConfig.gAddress, buf, 6, 500);

		*receivedData = (buf[0]&0xff) + ((buf[1]&0xff)<<8);
		if(*receivedData & 0x8000) *receivedData|=0xffff0000;
		receivedData++;

		*receivedData = (buf[2]&0xff) + ((buf[3]&0xff)<<8);
		if(*receivedData & 0x8000) *receivedData|=0xffff0000;
		receivedData++;

		*receivedData = (buf[4]&0xff) + ((buf[5]&0xff)<<8);
		if(*receivedData & 0x8000) *receivedData|=0xffff0000;
	}
	
	lsm6_tryChipDeSelect(imuHandle);

	return readError;
}


int lsm6_getRawGyroscope(lsm6Handle_t * imuHandle, int *receivedData)
{
	int readError;
	uint8_t buf[6];
	uint8_t data[1] ;

	if(!imuHandle->libConfig.gyroInitDone)
	{
		return IMU_ERRORS_ACCGYRO_NOT_INIT;
	}

	data[0] = LSM6_OUTX_L_G; // Indicate that we want to read Gyroscope value register

	lsm6_tryChipSelect(imuHandle);

	readError = imuHandle->userConfig.sendI2cCallback(imuHandle->libConfig.gAddress,data,1,500, false);

	if(readError == IMU_ERRORS_NONE)
	{
		imuHandle->userConfig.receiveI2cCallback(imuHandle->libConfig.gAddress, buf, 6, 500);

		// Concatenate two 8 bits to make one 16 bits buf[1]buf[0]
		*receivedData = (buf[0]&0xff) + ((buf[1]&0xff)<<8);
		if(*receivedData&0x8000) *receivedData|=0xffff0000;
		receivedData++;

		*receivedData = (buf[2]&0xff) + ((buf[3]&0xff)<<8);
		if(*receivedData&0x8000) *receivedData|=0xffff0000;
		receivedData++;

		*receivedData = (buf[4]&0xff) + ((buf[5]&0xff)<<8);
		if(*receivedData&0x8000) *receivedData|=0xffff0000;
	}
	
	lsm6_tryChipDeSelect(imuHandle);

	return readError;

}



int lsm6_getAccelerationG(lsm6Handle_t * imuHandle, float *receivedData)
{
	int dataInteger[3];
	int status;

	status = lsm6_getRawAcceleration(imuHandle, dataInteger);
	if(status == IMU_ERRORS_NONE)
	{
		receivedData[X_AXIS] = (float)dataInteger[X_AXIS] * imuHandle->libConfig.accelSensitivityFactor;
		receivedData[Y_AXIS] = (float)dataInteger[Y_AXIS] * imuHandle->libConfig.accelSensitivityFactor;
		receivedData[Z_AXIS] = (float)dataInteger[Z_AXIS] * imuHandle->libConfig.accelSensitivityFactor;
	}
	else
	{
		receivedData[X_AXIS] = 0;
		receivedData[Y_AXIS] = 0;
		receivedData[Z_AXIS] = 0;
	}

	return status;
}

int lsm6_getAccelerationM(lsm6Handle_t * imuHandle, float *receivedData)
{
	int dataInteger[3];
	int status;

	status = lsm6_getRawAcceleration(imuHandle, dataInteger);
	if(status == IMU_ERRORS_NONE)
	{
		receivedData[X_AXIS] = (float)dataInteger[X_AXIS] * imuHandle->libConfig.accelSensitivityFactor * 9.80665;
		receivedData[Y_AXIS] = (float)dataInteger[Y_AXIS] * imuHandle->libConfig.accelSensitivityFactor * 9.80665;
		receivedData[Z_AXIS] = (float)dataInteger[Z_AXIS] * imuHandle->libConfig.accelSensitivityFactor * 9.80665;
	}
	else
	{
		receivedData[X_AXIS] = 0;
		receivedData[Y_AXIS] = 0;
		receivedData[Z_AXIS] = 0;
	}

	return status;
}

int lsm6_getGyroscope(lsm6Handle_t * imuHandle, float *receivedData)
{
	int dataInteger[3];
	int status;

	status = lsm6_getRawGyroscope(imuHandle, dataInteger);
	if(status == IMU_ERRORS_NONE)
	{
		receivedData[X_AXIS] = (float)dataInteger[X_AXIS] * imuHandle->libConfig.gyroSensitivityFactor;
		receivedData[Y_AXIS] = (float)dataInteger[Y_AXIS] * imuHandle->libConfig.gyroSensitivityFactor;
		receivedData[Z_AXIS] = (float)dataInteger[Z_AXIS] * imuHandle->libConfig.gyroSensitivityFactor;
	}
	else
	{
		receivedData[X_AXIS] = 0;
		receivedData[Y_AXIS] = 0;
		receivedData[Z_AXIS] = 0;
	}

	return status;
}




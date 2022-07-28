/**
 * @file imuManager_V0_3.h
 * @author Simon Latour simon.latour@cirris.ulaval.ca
 * @brief This library manage the IMU data acquisition
 * @version 0.3
 * @date 2020-10-08
 * 
 * @copyright Copyright (c) 2020
 * 
 * Release info:
 * V0.1 : Initial proto release
 * V0.2 : Update Chip select pin mechanism to support "No chip select pin"
 * V0.3 : Modification to be more generic and support more platforms. Better abstraction from the hardware
 *
 * -----------How to use this library-----------
 * Setup Steps
 * 1. Declare an instance of the imu manager
 * 2. For each IMU, create a global handle variable of type imuManagerHandle_t
 * 3. For each IMU, Request function that sends a data request to the IMU IC. Make sur the function maches the imuDataRequestFunc_t callback format: int yourFunctionName()
 *    Do this for type of data used ( if used) accel, gyro and mag.
 * 4. Link you Request functions to the Imu manager by assigned the correct function pointer to the correct function
 * 5. Add you IMU to the the imu manager.
 *
 * Use information
 * 1. Received data from the IMU must be passed to the IMU manager.  This is done using the set functions.
 * 
 * 	Option 1: Polling
 * 		When using polling method, you should put this inside and at the end of the request function you linked in step 3,4 of the setup.
 * 		is shoudl look somethign like that:
 * 		int imu1requestAccelFunction()
 * 		{
 * 			//Code requesting and waiting to received data for the IMU IC here (i2c, spi, ect)
 *
 * 			imuManager.setAccel(imu1Index, receivedData); // Received data is passed to the IMU Manager
 * 		}
 */
#ifndef IMUMANAGER_V0_3_H

  #define IMUMANAGER_V0_3_H

#include "stdint.h"
#include "imuDataTypes.h"

#define IMUMANAGER_ERROR_INDEX_ERROR		-1
#define IMUMANAGER_ERROR_NONE  				0
// 0x001 to 0x999 reserved for IMU drivers errors and I2C errors
#define IMUMANAGER_ERROR_MODULEINIT 		(0x00001001)
#define IMUMANAGER_ERROR_FUNCTION_NOT_SET   (0x00001002)

#define IMUMANAGER_NB_IMU_MAX 16
    

/// Function callback types


typedef int (*imuDataRequestFunc_t)(int);  // Used to link the functions thats sends thje data request to the IMU IC

// Structure fill all needed function callback
typedef struct
{
	imuDataRequestFunc_t requestAccel;
	imuDataRequestFunc_t requestGyro;
	imuDataRequestFunc_t requestMag;

}imuManagerCallbackHandle_t;

// Create a structure type to make access to imu data easier and tider 
typedef struct imuDataStruct{

    // imu Data variables
    floatXYZ_t accel;		// Data value
    uint8_t newAccelFlag;	// Used to communicate if new data has been entered

    floatXYZ_t gyro;		// Data value
    uint8_t newGyroFlag;	// Used to communicate if new data has been entered

    floatXYZ_t mag;			// Data value
    uint8_t newMagFlag;		// Used to communicate if new data has been entered

 
}imuDataType;

// Create a structure type to make access to imu data easier and tider 
typedef struct imuStruct{

	// imu function callbacks
	//imuManagerCallbackHandle_t functions;
    
	// imu Data variables
    imuDataType data;

    //imu Config

}imuManagerHandle_t;


class imuManagerClass
{
    private:

        uint8_t m_imuCount = 0;			// Number of IMU currently used
        imuManagerHandle_t *imuList[IMUMANAGER_NB_IMU_MAX];   //List of added IMUs

        imuManagerCallbackHandle_t cbFunc;
    public:

        int init(imuDataRequestFunc_t requestAccel, imuDataRequestFunc_t requestGyro,
        		imuDataRequestFunc_t requestMag);

  /// Configuration functions
        /**
         * @fn int addImu(imuType*)
         * @brief Add an imu to the imu manager
         *
         * @param imuPtr pointer to the imu handle for the imu manager
         * @return return the index number of the added imu. You will need this number for future access to this imu
         */
        int addImu(imuManagerHandle_t *imuPtr);




   /// User interface functions

        /**
            * @fn uint8_t getImuCount()
            * @brief  Get the current number of imu managed by the imu manager
            *
            * @return
            */
           uint8_t getImuCount();

        ////////////// General procedure for getting data //////////////
        /// 1. Request Data   				[What is happening: UC --> Send data request --> IMU CHIP --> Send data to UC --> Data stored in buffer]
        /// 2. Check if new data available	[Check in new data is in the buffer]
        /// 3. Get data						[Returns the data tha is present in the buffer]
        ///////////////////////////////////////////////////////////////

        /// Data request functions
			/**
			 * @fn void requestAccel(int)
			 * @brief Initiate data request to the IMU IC
			 *
			 * @param imuSelect index of the imu to read the data from
			 *
			 * @return status
			 */
           int requestAccel(int imuSelect);

			/**
			 * @fn void requestGyro(int)
			 * @brief Initiate data request to the IMU IC
			 *
			 * @param imuSelect index of the imu to read the data from
			 *
			 * @return status
			 */
			int requestGyro(int imuSelect);

			/**
			 * @fn void requestMag(int)
			 * @brief Initiate data request to the IMU IC
			 *
			 * @param imuSelect index of the imu to read the data from
			 *
			 * @return status
			 */
			int requestMag(int imuSelect);

		/// New data check functions
			/**
			 * @fn bool newAccelData(int)
			 * @brief 	This function is used to know if new, unread data is present. If the result is false,
			 * 			it means that the data was already read before.
			 *
			 * @param imuSelect index of the imu to read the data from
			 * @return	true if new data is available, false if data was already read
			 */
			bool newAccelData(int imuSelect);

			/**
			 * @fn bool newGyroData(int)
			 * @brief 	This function is used to know if new, unread data is present. If the result is false,
			 * 			it means that the data was already read before.
			 *
			 * @param  imuSelect index of the imu to read the data from
			 * @return true if new data is available, false if data was already read
			 */
			bool newGyroData(int imuSelect);

			/**
			 * @fn bool newMagData(int)
			 * @brief 	This function is used to know if new, unread data is present. If the result is false,
			 * 			it means that the data was already read before.
			 *
			 * @param  imuSelect index of the imu to read the data from
			 * @return true if new data is available, false if data was already read
			 */
			bool newMagData(int imuSelect);

		/// Get data function
			/**
			 * @fn floatXYZ_t getAccel(int)
			 * @brief Get last received accelerometer value
			 *
			 * @note Use the associated request function before getting data.
			 *
			 * @param imuSelect index of the imu to read the data from
			 * @return acceleration value in meters/seconds
			 */
			floatXYZ_t getAccel(int imuSelect);

			/**
			 * @fn floatXYZ_t getGyro(int)
			 * @brief Get last received gyroscope value
			 *
			 * @param imuSelect index of the imu to read the data from
			 * @return gyroscope value
			 */
			floatXYZ_t getGyro(int imuSelect);

			/**
			 * @fn floatXYZ_t getMag(int)
			 * @brief Get last received Magnometer value
			 *
			 * @param imuSelect index of the imu to read the data from
			 * @return Magnometer value
			 */
			floatXYZ_t getMag(int imuSelect);

			/**
			 * @fn imuDataType getImuData(int)
			 * @brief Get all last receive values for Accel, gyro and mag and put everyting in a structure
			 *
			 * @param imuSelect index of the imu to read the data from
			 * @return imuDataType structure fill with the las received data
			 */
			imuDataType getImuData(int imuSelect);



        /// Setting Data in the imu manager

		/**
	     * @fn void setAccel(int, floatXYZ_t)
		 * @brief This function is used to pass de received data to the IMU manager
		 *
		 * @param imuSelect index of the imu related to the received data
		 * @param data the data
		 */
        void setAccel(int imuSelect, floatXYZ_t data);
        /**
		 * @fn void setGyro(int, floatXYZ_t)
		 * @brief This function is used to pass de received data to the IMU manager
		 *
		 * @param imuSelect index of the imu related to the received data
		 * @param data the data
		 */
        void setGyro(int imuSelect, floatXYZ_t data);

        /**
		 * @fn void setMag(int, floatXYZ_t)
		 * @brief This function is used to pass de received data to the IMU manager
		 *
		 * @param imuSelect index of the imu related to the received data
		 * @param data the data
		 */
        void setMag(int imuSelect, floatXYZ_t data);




};

#endif

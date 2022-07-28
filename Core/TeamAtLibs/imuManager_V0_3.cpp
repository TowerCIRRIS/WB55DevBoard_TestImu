
#include "imuManager_V0_3.h"


int imuManagerClass::init(imuDataRequestFunc_t requestAccel, imuDataRequestFunc_t requestGyro,
        		imuDataRequestFunc_t requestMag)
{
	cbFunc.requestAccel = requestAccel;
	cbFunc.requestGyro = requestGyro;
	cbFunc.requestMag = requestMag;

	return IMUMANAGER_ERROR_NONE;
}


int imuManagerClass::addImu(imuManagerHandle_t *imuPtr)
{

	if(m_imuCount < IMUMANAGER_NB_IMU_MAX)
	{
		imuList[m_imuCount] = imuPtr;
		
		imuList[m_imuCount]->data.newAccelFlag = 0;
		imuList[m_imuCount]->data.newGyroFlag = 0;
		imuList[m_imuCount]->data.newMagFlag = 0;

		 m_imuCount++;

		return m_imuCount-1;
	}
	return IMUMANAGER_ERROR_INDEX_ERROR;	//TODO retourner status d'erreur

}

uint8_t imuManagerClass::getImuCount()
{
	return m_imuCount;
}

int imuManagerClass::requestAccel(int imuSelect){

//	if(imuList[imuSelect]->functions.requestAccel) // check if not null
//	{
//		imuList[imuSelect]->functions.requestAccel(imuSelect);
//	}

	if(cbFunc.requestAccel) // check if not null
	{
		return cbFunc.requestAccel(imuSelect);
	}
	return IMUMANAGER_ERROR_FUNCTION_NOT_SET;
}

int imuManagerClass::requestGyro(int imuSelect)
{
//	if(imuList[imuSelect]->functions.requestGyro) // check if not null
//	{
//		imuList[imuSelect]->functions.requestGyro(imuSelect);
//	}

	if(cbFunc.requestGyro) // check if not null
	{
		return cbFunc.requestGyro(imuSelect);
	}

	return IMUMANAGER_ERROR_FUNCTION_NOT_SET;
}

int imuManagerClass::requestMag(int imuSelect)
{
//	if(imuList[imuSelect]->functions.requestMag) // check if not null
//	{
//		imuList[imuSelect]->functions.requestMag(imuSelect);
//	}

	if(cbFunc.requestMag) // check if not null
	{
		return cbFunc.requestMag(imuSelect);
	}

    return IMUMANAGER_ERROR_FUNCTION_NOT_SET;
}


void imuManagerClass::setAccel(int imuSelect, floatXYZ_t data)
{
	if(imuSelect >= 0 && imuSelect < m_imuCount)
	{
		imuList[imuSelect]->data.accel.x = data.x;
		imuList[imuSelect]->data.accel.y = data.y;
		imuList[imuSelect]->data.accel.z = data.z;
		imuList[imuSelect]->data.newAccelFlag = 1;
	}
}

void imuManagerClass::setGyro(int imuSelect, floatXYZ_t data)
{
	if(imuSelect >= 0 && imuSelect < m_imuCount)
	{
		imuList[imuSelect]->data.gyro.x = data.x;
		imuList[imuSelect]->data.gyro.y = data.y;
		imuList[imuSelect]->data.gyro.z = data.z;
		imuList[imuSelect]->data.newGyroFlag = 1;
	}
}

void imuManagerClass::setMag(int imuSelect, floatXYZ_t data)
{
	if(imuSelect >= 0 && imuSelect < m_imuCount)
	{
		imuList[imuSelect]->data.mag.x = data.x;
		imuList[imuSelect]->data.mag.y = data.y;
		imuList[imuSelect]->data.mag.z = data.z;
		imuList[imuSelect]->data.newMagFlag= 1;
	}
}

bool imuManagerClass::newAccelData(int imuSelect)
{
	return imuList[imuSelect]->data.newAccelFlag;
}

bool imuManagerClass::newGyroData(int imuSelect)
{
	return imuList[imuSelect]->data.newGyroFlag;
}

bool imuManagerClass::newMagData(int imuSelect)
{
	return imuList[imuSelect]->data.newMagFlag;
}


floatXYZ_t imuManagerClass::getAccel(int imuSelect)
{
	imuList[imuSelect]->data.newAccelFlag = 0;	// indicate that data was read
	return imuList[imuSelect]->data.accel;
}

floatXYZ_t imuManagerClass::getGyro(int imuSelect)
{
	imuList[imuSelect]->data.newGyroFlag = 0;	// indicate that data was read
	return imuList[imuSelect]->data.gyro;
}

floatXYZ_t imuManagerClass::getMag(int imuSelect)
{
	imuList[imuSelect]->data.newMagFlag = 0;	// indicate that data was read
	return imuList[imuSelect]->data.mag;
}


imuDataType imuManagerClass::getImuData(int imuSelect)
{
	imuList[imuSelect]->data.newAccelFlag = 0;	// indicate that data was read
	imuList[imuSelect]->data.newGyroFlag = 0;	// indicate that data was read
	imuList[imuSelect]->data.newMagFlag = 0;	// indicate that data was read

    return imuList[imuSelect]->data ;//imuInfo[imuSelect].data;
}

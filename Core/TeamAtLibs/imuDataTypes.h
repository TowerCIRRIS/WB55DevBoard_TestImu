/*
 * imuDataTypes.h
 *
 *  Created on: 7 avr. 2021
 *      Author: teamat
 */

#ifndef SRC_IMUDATATYPES_H_
#define SRC_IMUDATATYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	float x;
	float y;
	float z;
} floatXYZ_t;

typedef struct{
	floatXYZ_t accel;
	floatXYZ_t gyro;
	floatXYZ_t mag;
} imuData_t;

#ifdef __cplusplus
}
#endif

#endif /* SRC_IMUDATATYPES_H_ */

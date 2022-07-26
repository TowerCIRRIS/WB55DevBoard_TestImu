   /**
 * @file teamATBasic_V1_1.h
 * @author Simon Latour
 * @brief  Basic tool functions
 * @version 1.1.0
 * @date 2021-08-26
 * 
 * @copyright Copyright (c) 2021
 * 
 * V1.0.1
 *    - Added output size for the millisToHumanTime(..) function to prevent memory errors
 * 
 */

#ifndef TEAMATBASIC_h

    #define TEAMATBASIC_h 

   #include "stdint.h"
 //#include "Arduino.h"

    
    /**
 * @brief  This function converts numbers in an array to an unsigned int
 * 
 * @param array         The source aray that contains the number in char
 * @param nbDigit       The number of digit
 * @return unsigned int The number in unsigned int
 */
unsigned int arrayToUint(const char* array, uint8_t nbDigit );

/**
 * @brief This functions converts an unsigned in into a character array
 * 
 * @param number  The number to convert  
 * @param array   The array where to put the result
 * @param nbDigit The number of digits.
 * 
 * @return error Status
 */
uint8_t uintToArray(uint16_t number, char* array, uint8_t nbDigit );

/**
 * @brief This function checks the difference between two times and protects against the corner case where the time loops 
 *        when the unsigne long variable reaches  4,294,967,295
 * 
 * @param newTime The most recent time read
 * @param lastTime The porevious time red
 * @return uint32_t The difference between both
 */
uint32_t timeElapsed(uint32_t newTime, uint32_t lastTime);

/**
 * @brief This functions takes milliseconds and converts in into human readable time 
 *      example: 5400000 ms returns the string "1:30"
 * 
 * @param millisTime    The input time in ms
 * @param timeString    The buffer that will recieve the time string
 * @param size          The size of the output buffer
 */
void millisToHumanTime(unsigned long millisTime, char * timeString, uint32_t size);


#endif
/**
 * @file teamATBasic.cpp
 * @author Simon Latour
 * @brief  Basic tool functions
 * @version 1.1.0
 * @date 2021-08-26
 * 
 * @copyright Copyright (c) 2020
 * 
 * V1.0.1
 *    - Added output size for the millisToHumanTime(..) function to prevent memory errors
 */

#include "teamATbasic_V1_1.h"

#include "stdio.h"

/**
 * @brief  This function converts numbers in an array to an unsigned int
 * 
 * @param array         The source aray that contains the number in char
 * @param nbDigit       The number of digit
 * @return unsigned int The number in unsigned int
 */
unsigned int arrayToUint(const char* array, uint8_t nbDigit )
{
    uint16_t filenumberInt = 0;;


    for(int i = 0 ; i < nbDigit; i++)
        {
            filenumberInt = (filenumberInt * 10);

            if(array[i] != '0')
            {
                filenumberInt = filenumberInt + (array[i] - '0');
            }
        }
    return filenumberInt;
}

/**
 * @brief This functions converts an unsigned in into a character array
 * 
 * @param number  The number to convert  
 * @param array   The array where to put the result
 * @param nbDigit The number of digits.
 * 
 * @return error Status
 */
uint8_t uintToArray(uint16_t number, char* array, uint8_t nbDigit )
{

    uint16_t originalNumber = number;
    uint16_t divider = 1;
    uint16_t testnumber = 0;

    for(int i = 1 ; i < nbDigit; i++)
    {
        divider = divider * 10;
    }

    if(number >= (divider * 10))
    {
       return 0;
    }
    else
    {
        for(int i = 0 ; i < nbDigit; i++)
        {
            testnumber = originalNumber / divider;
            array[i] = testnumber + '0';
            originalNumber -= (testnumber * divider);
            divider = divider / 10;
        }
    }

    return 1;
}

/**
 * @brief This function checks the difference between two times and protects against the corner case where the time loops 
 *        when the unsigne long variable reaches  4,294,967,295
 * 
 * @param newTime The most recent time read
 * @param lastTime The porevious time red
 * @return uint32_t The difference between both
 */
uint32_t timeElapsed(uint32_t newTime, uint32_t lastTime)
{
    if(newTime < lastTime ) // Variable overflow, 
    {
        //0xFFFFFFFF
        return (0xFFFFFFFF-lastTime) + newTime;
    }
    else
    {
        return newTime - lastTime;
    }
    
}


void millisToHumanTime(unsigned long millisTime, char * timeString, uint32_t size)
{
    unsigned long timeSeconds,timeminutes,timehours;

        timeSeconds = millisTime / 1000;
        timeminutes = timeSeconds / 60;
        timehours = timeminutes /60;
        timeminutes = timeminutes % 60;        

        snprintf(timeString ,size, "%lu:%02lu",timehours,timeminutes);
}

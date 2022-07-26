/**
 * @def AT_PLATFORMABSTRACTION_H
 * @brief 	This librarie is used to do an abstraction of the different platforms
 * 			so that the code build using this library may be reused on ST,
 * 			Arduino, Atmel, etc.
 *
 */

#ifndef AT_PLATFORMABSTRACTION_H
#define AT_PLATFORMABSTRACTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"


// For STM32
#if defined(STM32L496xx) || defined(STM32WB5Mxx)

	#include "main.h"	// Main with standard includes for the platform
	//#include "stm32l4xx.h"

	#define digitalPinRead(pin,port)  ((unsigned char)HAL_GPIO_ReadPin((GPIO_TypeDef*)port, (uint16_t)pin))
	#define digitalPinWrite(pin,port,state) (HAL_GPIO_WritePin((GPIO_TypeDef*)port, (uint16_t)pin, (GPIO_PinState)state))

	typedef struct{

			uint16_t pin;
			GPIO_TypeDef* port;

		}atPin_t;

		#define atPinRead(atPin) 		 (digitalPinRead(atPin.pin,atPin.port))
		#define atPinWrite(atPin, state) (digitalPinWrite(atPin.pin,atPin.port,state))

		#define AT_PIN_HIGH	1
		#define AT_PIN_LOW	0

		#define delay(timeMS) 	HAL_Delay(timeMS)

#endif



#ifdef ARDUINO
		// Arduino Example (Not tested yet)
		#define digitalPinRead(pin,port)  ((unsigned char)digitalRead(pin)
		#define digitalPinWrite(pin,port,state) (digitalWrite((uint32_t)pin, (uint32_t)state))

		typedef struct{
					uint32_t pin;
					//uint8_t port; // Not used in arduino

				}atPin_t;

				#define atPinRead(atPin) 		 ((unsigned char)digitalRead(atPin.pin))
				#define atPinWrite(atPin, state) (digitalWrite((uint32_t)atPin.pin, (uint32_t)state))

				#define AT_PIN_HIGH	HIGH
				#define AT_PIN_LOW	LOW

#endif

		#define atPinRead(atPin) 		 (digitalPinRead(atPin.pin,atPin.port))
		#define atPinWrite(atPin, state) (digitalPinWrite(atPin.pin,atPin.port,state))


	uint32_t atGetSysTick_ms();

#ifdef __cplusplus
}
#endif


#endif

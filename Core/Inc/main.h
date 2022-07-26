/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN_B2_Pin GPIO_PIN_2
#define IN_B2_GPIO_Port GPIOA
#define TFT_SCK_Pin GPIO_PIN_1
#define TFT_SCK_GPIO_Port GPIOA
#define IN_B3_Pin GPIO_PIN_0
#define IN_B3_GPIO_Port GPIOA
#define EXT4_Pin GPIO_PIN_3
#define EXT4_GPIO_Port GPIOC
#define EXT3_Pin GPIO_PIN_2
#define EXT3_GPIO_Port GPIOC
#define EXT2_UART_TX_Pin GPIO_PIN_1
#define EXT2_UART_TX_GPIO_Port GPIOC
#define SD_DET_Pin GPIO_PIN_9
#define SD_DET_GPIO_Port GPIOB
#define EXT1_UART_RX_Pin GPIO_PIN_0
#define EXT1_UART_RX_GPIO_Port GPIOC
#define I2C1_SCL_Pin GPIO_PIN_8
#define I2C1_SCL_GPIO_Port GPIOB
#define VCP_RX_Pin GPIO_PIN_7
#define VCP_RX_GPIO_Port GPIOB
#define OUT_TFT_RST_Pin GPIO_PIN_5
#define OUT_TFT_RST_GPIO_Port GPIOB
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define IN_B1_Pin GPIO_PIN_12
#define IN_B1_GPIO_Port GPIOC
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define I2C1_SDA_Pin GPIO_PIN_10
#define I2C1_SDA_GPIO_Port GPIOA
#define USB_P_Pin GPIO_PIN_12
#define USB_P_GPIO_Port GPIOA
#define USB_N_Pin GPIO_PIN_11
#define USB_N_GPIO_Port GPIOA
#define SD_SCLK_Pin GPIO_PIN_13
#define SD_SCLK_GPIO_Port GPIOB
#define TSC_G4_IO1_Pin GPIO_PIN_6
#define TSC_G4_IO1_GPIO_Port GPIOC
#define SD_MISO_Pin GPIO_PIN_14
#define SD_MISO_GPIO_Port GPIOB
#define SD_MOSI_Pin GPIO_PIN_15
#define SD_MOSI_GPIO_Port GPIOB
#define IN_STAT_PWR_Pin GPIO_PIN_6
#define IN_STAT_PWR_GPIO_Port GPIOB
#define IN_B4_Pin GPIO_PIN_13
#define IN_B4_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_12
#define SD_CS_GPIO_Port GPIOB
#define I2C3_SDA_Pin GPIO_PIN_11
#define I2C3_SDA_GPIO_Port GPIOB
#define OUT_TFT_CS_Pin GPIO_PIN_2
#define OUT_TFT_CS_GPIO_Port GPIOB
#define OUT_TFT_DC_Pin GPIO_PIN_4
#define OUT_TFT_DC_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOA
#define SAI1_D2_Pin GPIO_PIN_9
#define SAI1_D2_GPIO_Port GPIOA
#define TFT_MOSI_Pin GPIO_PIN_7
#define TFT_MOSI_GPIO_Port GPIOA
#define TFT_BL_Pin GPIO_PIN_5
#define TFT_BL_GPIO_Port GPIOA
#define VBUS_MON_Pin GPIO_PIN_4
#define VBUS_MON_GPIO_Port GPIOA
#define VBAT_MON_Pin GPIO_PIN_3
#define VBAT_MON_GPIO_Port GPIOA
#define DRDY_Pin GPIO_PIN_1
#define DRDY_GPIO_Port GPIOE
#define INT1_Pin GPIO_PIN_2
#define INT1_GPIO_Port GPIOD
#define OUT_DEV_PWR_EN_Pin GPIO_PIN_3
#define OUT_DEV_PWR_EN_GPIO_Port GPIOD
#define IN_CHRG_STAT_Pin GPIO_PIN_9
#define IN_CHRG_STAT_GPIO_Port GPIOD
#define OUT_LED_Pin GPIO_PIN_15
#define OUT_LED_GPIO_Port GPIOD
#define TSC_G6_IO1_Pin GPIO_PIN_10
#define TSC_G6_IO1_GPIO_Port GPIOD
#define GPIO_SELECT1_Pin GPIO_PIN_2
#define GPIO_SELECT1_GPIO_Port GPIOE
#define QSPI_BK_IO1_Pin GPIO_PIN_5
#define QSPI_BK_IO1_GPIO_Port GPIOD
#define QSPI_BK_IO2_Pin GPIO_PIN_6
#define QSPI_BK_IO2_GPIO_Port GPIOD
#define TSC_G6_IO2_Pin GPIO_PIN_11
#define TSC_G6_IO2_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

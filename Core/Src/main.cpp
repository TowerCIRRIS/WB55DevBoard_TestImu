/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include <ButtonPollingManager_V2.3.h>
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "lptim.h"
#include "usart.h"
#include "rf.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "wwdg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dmaspi_comm_stm32_V1.0.0.h" // For SPI display
#include "oledDisplayManager_V1_1.h"
#include "teamAt_GC9A01_dma_V1_0.h"
#include "imu.h"

#include <String>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SUBMENU_WIDTH	100
#define SUBMENU_HEIGHT	60
#define START_X	(120-SUBMENU_WIDTH)
#define START_Y 20
#define TITLE_HEIGHT 30

#define BUTTONS_POS_Y	125
#define BUTTONS_TITLE_HEIGHT 12
#define BUTTONS_SPACING	54
#define BUTTONS_WIDTH	50

#define BUTTON_1_POS_X	120-(BUTTONS_SPACING * 2)
#define BUTTON_2_POS_X	120-(BUTTONS_SPACING )
#define BUTTON_3_POS_X	120+(BUTTONS_SPACING )
#define BUTTON_4_POS_X	120+(BUTTONS_SPACING * 2)

#define BUTTON_POS_X(x) (BUTTON_1_POS_X + (BUTTONS_SPACING - BUTTONS_WIDTH)/2  + x*BUTTONS_SPACING)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define LPTIMER_PERIOD	(16000-412) // avleur ajouter poru avoir le plus près possible de 10 ms
#define TASK10MS_COUNTER 20
volatile int task10msTimer = TASK10MS_COUNTER;
volatile uint8_t flagTask10ms;

char serialOutLine[200];	// Buffer for string to send serial buffer

///étape 1: Instance du manager de communication
dmaSpiManagerClass spiManager(&hspi1); // For SPI display

///étape 2: Définir les I/O requis pour le SPI. On utilise le type atPin_t pour rester compatible avec les différentes plateformes
atPin_t oled_dc = {OUT_TFT_DC_Pin, OUT_TFT_DC_GPIO_Port};
atPin_t oled_reset = {OUT_TFT_RST_Pin, OUT_TFT_RST_GPIO_Port};
atPin_t oled_cs = {OUT_TFT_CS_Pin, OUT_TFT_CS_GPIO_Port};


#define SCREEN_WIDTH GC9A01_TFTWIDTH // OLED display width, in pixels
#define SCREEN_HEIGHT GC9A01_TFTHEIGHT// OLED display height, in pixels

///étape 3: instance de notre afficheur SSD1306
// For SPI display
teamAt_GC9A01 display1(
&spiManager,					// Relier le gestionnaire de communication avec l'afficheur
oled_dc,oled_reset, oled_cs,	// Donner les pins utilisés
SCREEN_WIDTH ,SCREEN_HEIGHT 	// Définir la grosseur de l'écran utilisé
);

///étape 4: Instance du gestionnaire d'affichage
#define DISPLAY_REFRESH_PERIOD  50   // Screen will be refreshed every 50 ms
oledDisplayManager dispManager1(
		&display1,					// Relier l'afficheur avec le gestionnaire d'affichage
		DISPLAY_REFRESH_PERIOD); 	// Définir la période d'affichage désirée ( refresh rate)



ButtonPollingManager buttonManager;

#define NB_BUTTONS 4

managedButton buttonList[NB_BUTTONS] ={
		{IN_B1_Pin, (uint32_t *)IN_B1_GPIO_Port, ACTIVE_LOW, CLICK_ENABLE | /*DOUBLECLICK_ENABLE | LONGCLICK_ENABLE |*/ HOLD_ENABLE},
		{IN_B2_Pin, (uint32_t *)IN_B2_GPIO_Port, ACTIVE_LOW, CLICK_ENABLE | /*DOUBLECLICK_ENABLE | LONGCLICK_ENABLE |*/ HOLD_ENABLE},
		{IN_B3_Pin, (uint32_t *)IN_B3_GPIO_Port, ACTIVE_LOW, CLICK_ENABLE | /*DOUBLECLICK_ENABLE | LONGCLICK_ENABLE |*/ HOLD_ENABLE},
		{IN_B4_Pin, (uint32_t *)IN_B4_GPIO_Port, ACTIVE_LOW, CLICK_ENABLE | /*DOUBLECLICK_ENABLE | LONGCLICK_ENABLE |*/ HOLD_ENABLE}

};

char buttonText[NB_BUTTONS][20];
	 char buttonTitle[NB_BUTTONS][20];


void buttonEventManagement();

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  //MX_CRC_Init();
  MX_I2C1_Init();
  //MX_IWDG_Init();
  MX_LPTIM1_Init();
  //MX_LPTIM2_Init();
  MX_LPUART1_UART_Init();
  //MX_RF_Init();
  //MX_RTC_Init();
  MX_SPI1_Init();
  //MX_SPI2_Init();
  //MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  //MX_WWDG_Init();
  //MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	// Enable Timer 17 interrupts
  	HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
  	HAL_LPTIM_Counter_Start_IT(&hlptim1, LPTIMER_PERIOD);



  	buttonManager.init(NB_BUTTONS,buttonList);


	// Initialisation Display
	HAL_GPIO_WritePin(OUT_TFT_CS_GPIO_Port, OUT_TFT_CS_Pin, GPIO_PIN_SET);
	TIM2->CCR1 = 1000; // 100% pendant la config
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	///Démarrer le gestionnaire d'affichage
	dispManager1.start();

	//Make sur display is cleared;
	TIM2->CCR1 = 750; // 75%
	dispManager1.display->clearDisplay(BLACK);
	dispManager1.display->display();
	display1.waitForTxComplete();
	dispManager1.display->displayOn();


  	// Initialisation of the IMUs. Modify this function to change the humber of IMUs of parameter settings
	int imuInitStatus = imuInit();

	if (imuInitStatus) // Error
	{
		char errorCode[40];
		sprintf(errorCode, "Code:%d", imuInitStatus);
		dispManager1.widget_2Row(40, 40, 160, 160,
		  "IMU ERROR", TEXTSTYLE_SIZE_3|RED,
		  errorCode, TEXTSTYLE_SIZE_2|WHITE,
		  W_CENTER_ALIGN, 0, BORDERSTYLE_CIRCLE|BLUE);

		dispManager1.display->display();
		display1.waitForTxComplete();

		delay(5000);
	}


	/// Splash screen
	TIM2->CCR1 = 750; // 75%
	dispManager1.display->clearDisplay(BLACK);
	dispManager1.display->displayOn();
	dispManager1.display->display();
	display1.waitForTxComplete();
	dispManager1.display->setTextSize(3);             // Normal 1:1 pixel scale
	dispManager1.display->setTextColor(BLUE);        // Draw white text
	dispManager1.widget_2Row(40, 40, 160, 160,
			  "Cortex Mini", TEXTSTYLE_SIZE_3|BLUE,
			  "Version 0.1", TEXTSTYLE_SIZE_2|GREEN,
			  W_CENTER_ALIGN, 0, BORDERSTYLE_CIRCLE|BLUE);
	dispManager1.display->display();
	display1.waitForTxComplete();
//	dispManager1.display->display();
//	display1.waitForTxComplete();

    // test de luminosité
	delay(1500);
	TIM2->CCR1 = 600;
	delay(100);
	TIM2->CCR1 = 500;
	delay(100);
	TIM2->CCR1 = 400;
	delay(100);
	TIM2->CCR1 = 500;
	delay(100);
	TIM2->CCR1 = 400;
	delay(100);
	TIM2->CCR1 = 300;
	delay(100);
	TIM2->CCR1 = 200;
	dispManager1.display->clearDisplay(BLACK);
	dispManager1.display->display();
	display1.waitForTxComplete();
	delay(1000);
	TIM2->CCR1 = 1000;


 uint32_t counterAcquisition = 0;
  while (1)
  {
    /* USER CODE END WHILE */

	  buttonManager.handle(atGetSysTick_ms());	// Run the button manager
	  buttonEventManagement();		// Attache tasks or code to the button events
	  dispManager1.run(atGetSysTick_ms());
	  spiManager.handle();

	  if(flagTask10ms)
	  {
		  flagTask10ms = 0;
		  HAL_GPIO_TogglePin(OUT_LED_GPIO_Port, OUT_LED_Pin);


		  counterAcquisition++;

		  if(counterAcquisition >= 10  )
		  {
			  counterAcquisition = 0;

			  imuDataType imuData;
			  for(int i= 0; i < imuManager.getImuCount(); i++)
			  {
				  /// Refresh IMU DATA
				  imuManager.requestAccel(i);
				  imuManager.requestGyro(i);
				  //imuManager.requestMag(i); //mag not supported with LSM6

				  //imuData = imuManager.getImuData(i); // Alternatively we could have use this to get all data one shot

			  /// Display received data



				  sprintf(serialOutLine, "\n\n\rIMU %d -----------------------\n\r",i);
				  HAL_UART_Transmit(&hlpuart1, (uint8_t*)serialOutLine, strlen(serialOutLine), HAL_MAX_DELAY);

				  if(imuManager.newAccelData(i))
				  {
					  imuData.accel = imuManager.getAccel(i);
				  }
				  else
				  {
					  imuData.accel.x = 0.0;
					  imuData.accel.y = 0.0;
					  imuData.accel.z = 0.0;
				  }

				  if(imuManager.newGyroData(i))
				  {
					  imuData.gyro = imuManager.getGyro(i);
				  }
				  else
				  {
					  imuData.gyro.x = 0.0;
					  imuData.gyro.y = 0.0;
					  imuData.gyro.z = 0.0;
				  }

			  }

			  char line1[40];
			  char line2[40];
			  char line3[40];

			  dispManager1.display->clearDisplay(BLACK);
			  dispManager1.widget_1row(START_X+20, START_Y, SUBMENU_WIDTH-20, TITLE_HEIGHT,
					  "ACCEL", TEXTSTYLE_SIZE_2|GREEN,
					  W_CENTER_ALIGN, 0, 0, BORDERSTYLE_NONE);

			  sprintf(line1, "x:%.2f", imuData.accel.x);
			  sprintf(line2, "y:%.2f", imuData.accel.y);
			  sprintf(line3, "z:%.2f", imuData.accel.z);
			  dispManager1.widget_3Row(START_X, START_Y+TITLE_HEIGHT, SUBMENU_WIDTH, SUBMENU_HEIGHT,
					  line1, TEXTSTYLE_SIZE_2|WHITE,
					  line2, TEXTSTYLE_SIZE_2|WHITE,
					  line3, TEXTSTYLE_SIZE_2|WHITE,
			 			W_CENTER_ALIGN, 0, BORDERSTYLE_NONE);

			  dispManager1.widget_1row(120, START_Y, SUBMENU_WIDTH-20, TITLE_HEIGHT,
								  "GYRO", TEXTSTYLE_SIZE_2|GREEN,
								  W_CENTER_ALIGN, 0, 0, BORDERSTYLE_NONE);

			  sprintf(line1, "x:%.2f", imuData.gyro.x);
			  sprintf(line2, "y:%.2f", imuData.gyro.y);
			  sprintf(line3, "z:%.2f", imuData.gyro.z);
			  dispManager1.widget_3Row(120, START_Y+TITLE_HEIGHT, SUBMENU_WIDTH, SUBMENU_HEIGHT,
							  line1, TEXTSTYLE_SIZE_2|WHITE,
							  line2, TEXTSTYLE_SIZE_2|WHITE,
							  line3, TEXTSTYLE_SIZE_2|WHITE,
					 			W_CENTER_ALIGN, 0, BORDERSTYLE_NONE);
			  dispManager1.display->drawLine(0, 120, 240, 120, BLUE);
			  dispManager1.display->drawLine(120, 0, 120, 120, BLUE);

			  for(int i = 0 ; i < NB_BUTTONS; i++)
			  {

				  dispManager1.widget_1row(BUTTON_POS_X(i), BUTTONS_POS_Y, BUTTONS_WIDTH, BUTTONS_WIDTH,
				  			 	 			 				  	  	  	  buttonTitle[i], TEXTSTYLE_SIZE_1|YELLOW,
				  			 	 			 							  W_CENTER_ALIGN, 0, 0, BORDERSTYLE_NONE);

				  dispManager1.widget_1row(BUTTON_POS_X(i), BUTTONS_POS_Y+BUTTONS_TITLE_HEIGHT, BUTTONS_WIDTH, BUTTONS_WIDTH,
				  								  buttonText[i], TEXTSTYLE_SIZE_1|GREEN,
				  								  W_CENTER_ALIGN, 0, 0, BORDERSTYLE_CIRCLE_CONTAINED|WHITE);

			  }


			  dispManager1.display->display();

		  }

	  }


    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI1
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLLSAI1.PLLN = 24;
  PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_USBCLK|RCC_PLLSAI1_ADCCLK;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_HSE_DIV1024;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/* USER CODE BEGIN 4 */

///étape 5: Relier l'interruption de fin de transmission DMA au spi manager.
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	// if from spi 1
	if(hspi->Instance == hspi1.Instance )
	{
		spiManager.txDoneCallback();
	}
}



 void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{

	if(hlptim->Instance == hlptim1.Instance)
	{
		task10msTimer--;
			if (task10msTimer <= 0)
			{
				task10msTimer = TASK10MS_COUNTER;
				flagTask10ms = true;
			}


			HAL_LPTIM_Counter_Start_IT(&hlptim1, LPTIMER_PERIOD);
	}


}





 /**
  * @fn void buttonEventManagement()
  * @brief Used to map code / tasks to button events
  *
  */
 void buttonEventManagement()
 {




	for(int i = 0 ; i< NB_BUTTONS ; i++)
	{
		bool eventDetect;

		sprintf(buttonTitle[i],"Btn %d",i);
		eventDetect = false;
	 	if(buttonManager.getClickEvent(i)) //Button 1 click
	 	{
	 	 // digitalPinWrite(OUT_LED_Pin,OUT_LED_GPIO_Port,GPIO_PIN_SET);

	 	  sprintf(buttonText[i],"click");
	 	 eventDetect = true;
	 	}

	 	if(buttonManager.getHoldEvent(i)) //Button 1 hold
	 	{

	 		sprintf(buttonText[i],"hold");
	 		eventDetect = true;
	 	}

	 	if(buttonManager.getLongClickEvent(i)) //Button 1 long click
	 	{
	 		sprintf(buttonText[i],"L click");
	 		eventDetect = true;

	 	}

	 	if(buttonManager.getDoubleClickEvent(i)) // Button 1 double click
	 	{
	 		sprintf(buttonText[i],"D click");
	 		eventDetect = true;
	 	}

	 	//if(eventDetect)
	 	//{

	 	//}
	}




 }

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

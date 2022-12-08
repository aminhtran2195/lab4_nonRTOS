/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_gyroscope.h"
#include "MyLib.h"
#include "math.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "cmsis_os2.h"
SPI_HandleTypeDef hspi5;

char bufposx[5];
char bufposy[5];
char bufposz[5];

/* Definitions for MyTask02 */
osThreadId_t MyTask02Handle;
const osThreadAttr_t MyTask02_attributes = {
  .name = "MyTask02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask01 */
osThreadId_t myTask01Handle;
const osThreadAttr_t myTask01_attributes = {
  .name = "myTask01",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI5_Init(void);
void StartTask02(void *argument);
void StartTask01(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * @brief 	Kiem tra gia tri moi lon hon gia tri cu 1 don vi hay khong
 * @return 	Neu co it nhat 1 gia tri moi lon hon gia tri cu 1 don vi thi tra ve 1
 * 			Nguoc lai, tra ve 0
 */
uint8_t NoiseFilter(float* oldvalue,float* newvalue,int offset){
	uint8_t res = 0;
	for(uint8_t i=0;i<3;i++){
		if(abs(newvalue[i] - oldvalue[i]) > offset) res = 1;
	}
	return res;
}

void ConcatenateStr(const char* firstStr, char* secondStr, char* res){
	uint8_t frlen = strlen(firstStr);
	uint8_t total_len = frlen + strlen(secondStr);
	for(uint8_t i=0;i<total_len;i++){
		if(i<frlen) res[i] = firstStr[i];
		else res[i] = secondStr[i-frlen];
	}
	res[total_len] = '\0';
}

void ResetStr(char* str){
	uint8_t len = strlen(str);
	for(uint8_t i=0;i<len;i++){
		str[i] = '\0';
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();


  SystemClock_Config();


  MX_GPIO_Init();
  MX_SPI5_Init();

  osKernelInitialize();

  MyTask02Handle = osThreadNew(StartTask02, NULL, &MyTask02_attributes);

  /* creation of myTask01 */
  myTask01Handle = osThreadNew(StartTask01, NULL, &myTask01_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask02 */
/**
  * @brief  Function implementing the MyTask02 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	  CDC_Transmit_HS(bufposx, strlen(bufposx));
	  CDC_Transmit_HS(bufposy, strlen(bufposy));
	  CDC_Transmit_HS(bufposz, strlen(bufposz));
    osDelay(500);
  }
  /* USER CODE END 5 */
}



void StartTask01(void *argument)
{
	uint8_t cnt = 0,a_cnt = 0;
	float pos[3],oldpos[3] = {0};
	float angle[3];
	uint8_t x_direction,y_direction,idle;
  /* USER CODE BEGIN StartTask01 */
	if (BSP_GYRO_Init() != GYRO_OK){
		  BSP_LCD_DisplayStringAtLine(2,(uint8_t*)"CAN'T FIND GYRO");
		  while(1);
	osDelay(2000); // delay 2s để task1 thực hiện xong init
	}
	pPoint Points[]= {{80, 100}, {105, 50}, {130, 100}};
	BSP_LCD_Init();//init LCD
	//set the layer buffer address into SDRAM
	BSP_LCD_LayerDefaultInit(1, SDRAM_DEVICE_ADDR);
	BSP_LCD_SelectLayer(1);//select on which layer we write
	BSP_LCD_DisplayOn();//turn on LCD
	BSP_LCD_Clear(LCD_COLOR_WHITE);//clear the LCD on blue color
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);//set text background color
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);//set text color
	//write text
	//BSP_LCD_FillPolygon(Points, 3);
	BSP_LCD_DisplayStringAtLine(2,"x= 0");
	BSP_LCD_DisplayStringAtLine(3,"y= 0");
	BSP_LCD_DisplayStringAtLine(4,"z= 0");
  /* Infinite loop */
  for(;;)
  {
	BSP_GYRO_GetXYZ(pos);
	for(uint8_t i = 0;i<3;i++){
		// tich phan van toc tren mien t: S = v.dt (S la goc nghieng, v la  van toc goc)
		// lenh dieu kien de chan cac gia tri gay nhieu, co the dan den go'c nga`y cang lon'
		if(abs(pos[i])>500) angle[i]+= pos[i]*0.005/1000;
		pos[i] /= 1000; 	// chuyen tu mdps qua dps
	}

	/* 	Cu moi 5*100 ms thuc hien kiem tra van toc goc 1 lan,
	/*	neu su thay doi > 1 dps thi cap nhat vao trong chuoi~ bufpos */
	if(cnt > 100){
		cnt = 0;
		if(NoiseFilter(oldpos, pos, 1)){
			ftoa(pos[0], bufposx, 1);
			BSP_LCD_DisplayStringAtLine(2, bufposx);
			ftoa(pos[1], bufposy, 1);
			BSP_LCD_DisplayStringAtLine(3, bufposy);
			ftoa(pos[2], bufposz, 1);
			BSP_LCD_DisplayStringAtLine(4, bufposz);
			oldpos[0] = pos[0];
			oldpos[1] = pos[1];
			oldpos[2] = pos[2];
		}
	}

	/* moi~ 20*5 ms kiem tra su thay doi goc xoay 1 lan*/
	/* Neu sau 20*5, su thay doi goc > 3 do => gyro dang xoay, kiem tra trang thai xoay => dua ra ket qua */
	if(a_cnt > 20){
		a_cnt = 0;
			/* Su dung nhi phan {x_direction,y_direction} de to hop 4 trang thai*/
			if(angle[0] > 0) x_direction = 1;
			else x_direction = 0;
			if(angle[1] > 0) y_direction = 1;
			else y_direction = 0;
			switch(x_direction*2+y_direction){
			case 0:
				BSP_LCD_DisplayStringAtLine(6,(uint8_t*)"huong 0");
				break;
			case 1:
				BSP_LCD_DisplayStringAtLine(6,(uint8_t*)"huong 1");
				break;
			case 2:
				BSP_LCD_DisplayStringAtLine(6,(uint8_t*)"huong 2");
				break;
			case 3:
				BSP_LCD_DisplayStringAtLine(6,(uint8_t*)"huong 3");
				break;
			default:
				break;
			}
	}

	cnt++;
	a_cnt++;
  }
  /* USER CODE END StartTask01 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

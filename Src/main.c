/**
  ******************************************************************************
  interrupts is handled in stm32f4xx_it.c, serial port interrupt is also defined here.
	
 12/11/15 - Jess Notes
 1d_D1 is a revision to merge 1d and D1; 1d was i2c tested, while D1 id Delorean tested code.
 - code/driver will base to 1d (for some reason D1 does not support I2C readily, though it has a newer driver.
 - so back tracking to 1d.
 
 1/22/16 -
 - D0,D15 are input external interrupts
 - Handlers are located in gecho_gpio_app.c - line 585
 - command <a r> clears the LED1
 
 4/22/16
 - control of config pins of ILD added with D15 interrupt
 - command <a V> turns on/off vcore2 dynamic 
 
 CFG gap delay
 command <a txxxx> - 16bit number 
 
 08/05/16 (VCORE2) 
 - remove interrupt modes
 - add DVS1, DVS2 as dynamic modes;
 
 09/20/16 (VCORE2)
 - add D4, D5 to use as CFG pin drive for DVS 
 
 09-27-16 (VCORE2)
 - added 1 byte bit-bang RFFE write  (see gecho_gpio_app.c)
 
 Version D5, 10-20-16
 - All MIPI writes are now SPMI mode (added 2 clock cycles)
 
 Version D5.3, 1-12-17
 - Added control to ild sync pins direction. D2 pin
  
-*/
#include "stm32f4xx_hal.h"
#include "gecho_spi_app.h"
#include "gecho_gpio_app.h"
#include "decode.h"
#include "gecho_i2c_app.h"

#define DVS1 7
#define DVS2 8
#define DVS3 9

struct _settings {
	uint32_t spmi;

	uint32_t what_chip;
	uint32_t dynamic;
	uint32_t num_dyn_set;
	uint32_t bitbang_dynamic;
	uint32_t _slot_data_[40][10];
  uint32_t _slot_trigger[40];
	
	uint32_t one_byte[40];
	uint32_t one_byte_addr;
  uint32_t dyn_byte_1[40];
	uint32_t dyn_byte_2[40];
	uint32_t two_byte_addr; 
	
	uint32_t _num_slot;
	uint32_t _slot_delay;
	uint32_t exit_delay;
	int32_t _slot_delay_correction;
	int32_t _slot_delay_correction_2;
	int32_t _tr_gap_delay;
	uint32_t gsid;
	uint32_t pa_usid;
	
	uint32_t gto_dac_now;
	uint32_t gto_dac_prev;
	uint32_t gto_cuda_prior[40];
	
	uint32_t vcore2_dynamic;
	uint32_t vcore2_dynamic_mode;
	uint32_t vcore2_dyn_delay;
	uint32_t vcore2_dyn_delay2;
	uint32_t vcore2_dyn_slot;
	
	uint32_t vcore2_bh_thread;
	
	//dvs
	uint32_t dvs_vout[4];
	uint32_t dvs_iout[4];
	
} _config;


 
/* Private variables ---------------------------------------------------------*/
//TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef dma_spi1_tx;
DMA_HandleTypeDef dma_spi1_rx;
/* USER CODE BEGIN 0 */
/*  Delay Functions */
void TM_Delay_Init(void);
void TM_DelayMicros(uint32_t micros);

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  TM_Delay_Init();
	
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_I2C1_Init(); removed 10/19/16 to use pins B5,B6,B7 for cfg IO (VCORE2)
	
  MX_SPI1_Init(2);
	//MX_DMA_Init(); 
	//JV_SPI1_Init_DMA(SPI1);
	
  MX_USART3_UART_Init();

   /* INITIAL SETTINGS */
  _config._slot_delay = 500;
	_config._slot_delay_correction = 10;
	_config._tr_gap_delay = 500;
	_config.bitbang_dynamic = 0;
	_config.vcore2_dynamic = 0;
	_config.vcore2_bh_thread = 0;
	
	/* INITILIAZE IO'S */
	PortB_init ();
  PortD_init ();
	PortE_init ();	 
	Pin_B_On (1); //OK/READY LED
	ild_sync_out (0); //set ild buffers to input, hiz
	
	//Pin_B_On (5);
	//Pin_B_On (6);
	//Pin_B_On (7);
	//set_ILD_cfg(1, 1, 1);
	//set_ILD_cfg(0, 0, 1);
	//set_ILD_cfg(1, 1, 0);
	//set_ILD_cfg(0, 0, 0);
	
	/* HARDCODED version */
  HAL_UART_Transmit (&huart3, "\n>>>>>R2 USB-RFFE-SPMI-I2C-SPI D5 \n\r", 32, 1000);
	
	
  /* Infinite loop */
  while (1)
  {
	 if (_config.dynamic) {					 
	            if (_config.vcore2_dynamic_mode==DVS1)
							{ 
									transmit_vcore2_dvs1();
							}				
              if (_config.vcore2_dynamic_mode==DVS2)
							{ 
                  transmit_vcore2_dvs2();
							}			
							if (_config.vcore2_dynamic_mode==DVS3)
							{ 
                  transmit_vcore2_dvs3();
							}										
	   			 }			 
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
static void SystemClock_Config(void)
{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
  
	HAL_SYSTICK_Config(HSE_VALUE/ 10000); //100us
}

/* I2C1 init function */
//void MX_I2C1_Init(void)
//{

//  hi2c1.Instance = I2C1;
//  hi2c1.Init.ClockSpeed = 100000;
//  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//  hi2c1.Init.OwnAddress1 = 0;
//  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
//  hi2c1.Init.OwnAddress2 = 0;
//  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
//  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
//  HAL_I2C_Init(&hi2c1);

//}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE); //Enable interrupt
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE10 PE11 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

 /*Configure GPIO pins : PC0 PC1 PC2 
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);*/
	
  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           PD13 PD14 PD15 PD0 
                           PD2 PD3 PD4 PD5 
                           PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_7
                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 as interrupt */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(EXTI0_IRQn, 0x0F, 0x00);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		
	/*Configure GPIO pins : PD15 as interrupt */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x0F, 0x00);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	
	
  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

uint32_t multiplier;
 
/* TIM2 init function */
//void MX_TIM2_Init(void)
//{

//  TIM_MasterConfigTypeDef sMasterConfig;
//  TIM_OC_InitTypeDef sConfigOC;

//  htim2.Instance = TIM2;
//  htim2.Init.Prescaler = 255;
//  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 65535;
//  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
//  HAL_TIM_PWM_Init(&htim2);

//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 32768;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

//}

/* TIM3 init function */
//void MX_TIM3_Init(void)
//{

//  TIM_MasterConfigTypeDef sMasterConfig;
//  TIM_OC_InitTypeDef sConfigOC;

//  htim3.Instance = TIM3;
//  htim3.Init.Prescaler = 255;
//  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim3.Init.Period = 65535;
//  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
//  HAL_TIM_PWM_Init(&htim3);

//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 35768;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

//}
void TM_Delay_Init(void) {
	uint32_t freq;
    //RCC_ClocksTypeDef RCC_Clocks;
    
    /* Get system clocks */
	 //freq = HAL_RCC_GetSysClockFreq();
   // RCC_GetClocksFreq(&RCC_Clocks);
	  freq = HAL_RCC_GetHCLKFreq();
    
    /* While loop takes 3 cycles */
    /* For 1 us delay, we need to divide with 3M */
    //multiplier = RCC_Clocks.HCLK_Frequency / 3000000;
	  multiplier = freq / 3000000;
}

void TM_DelayMicros(uint32_t micros) {
    /* Multiply micros with multipler */
    /* Substract 10 */
    micros = micros * multiplier - 10;
    /* 3 cycles for one loop */
    while (micros--);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

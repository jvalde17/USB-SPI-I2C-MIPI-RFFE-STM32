#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"


#define I2C_100KHz_SPEED                        100000
#define I2C_400KHz_SPEED                        400000

#define I2C_TIMEOUT         ((uint32_t)0x1000)
#define I2C_LONG_TIMEOUT    ((uint32_t)(10 * I2C_TIMEOUT))

#define TIMEOUT 500
    __IO uint32_t  sTimeout = I2C_LONG_TIMEOUT;
		
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}
void R2_I2C1_Transmit_test(uint32_t address)
{
	uint8_t dat[4];
	 
	dat[0] = 0x15;
	dat[1] = 0x11;
	dat[2] = 0x55;
	dat[3] = 0;
	
  hi2c1.Init.OwnAddress1 = address ;
	hi2c1.pBuffPtr = dat;
	hi2c1.XferCount = 3;
	hi2c1.XferSize = 3;
	
//	I2C_MasterTransmit_TXE(&hi2c1);
	HAL_I2C_Master_Transmit(&hi2c1, address, dat, 3, 1000);
 
}

void R2_I2C1_write_1byte_cmd(uint32_t address, uint32_t dat0, uint32_t dat1, uint16_t w_r)
{
	uint8_t dat[2];
	 
	dat[0] = dat0;
	dat[1] = dat1;

	//I2C_RequestMemoryWrite
		HAL_I2C_Master_Transmit(&hi2c1, address, dat, w_r, 1000);

}

//Check if device is OK
void R2_I2C1_check_device_cmd(uint32_t address)
{
   uint32_t error;
	
	 error = HAL_I2C_IsDeviceReady(&hi2c1, address, 2, 10000);
	if (error == HAL_OK)
	{
		HAL_UART_Transmit (&huart3, "\n>>>>>Device is OK!   \n\r", 27, 1000);
	}
	if (error == HAL_BUSY)
	{
		HAL_UART_Transmit (&huart3, "\n>>>>>Device is busy! \n\r", 27, 1000);
	}
	if (error == HAL_ERROR)
	{
		HAL_UART_Transmit (&huart3, "\n>>>>>Device is in err!\n\r", 27, 1000);
	}

}

uint8_t R2_I2C1_read_1byte_cmd(uint32_t dev_address, uint32_t reg_address)
{
  uint8_t dat[2];
	
  //HAL_I2C_Master_Receive(&hi2c1, address, dat, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, dev_address, reg_address, 1, dat, 1, 1000);
	
	return dat[0]; 
}
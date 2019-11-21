/**
  ******************************************************************************
  * File Name          : stm32f4xx_hal_msp.c
  * Date               : 20/10/2014 09:22:51
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
extern DMA_HandleTypeDef dma_spi1_tx;
extern DMA_HandleTypeDef dma_spi1_rx;

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==I2C1)
  {
    /* Peripheral clock enable */
    __I2C1_CLK_ENABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  }

}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

  if(hi2c->Instance==I2C1)
  {
    /* Peripheral clock disable */
    __I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hspi->Instance==SPI1)
  {
    /* Peripheral clock enable */
    __SPI1_CLK_ENABLE();
  
    /**SPI1 GPIO Configuration    
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
		/* Enable DMA2 clock TX Stream*/	
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
		/* Enable DMA2 clock RX Stream*/	
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
		
	HAL_DMA_DeInit(&dma_spi1_rx);	
	HAL_DMA_DeInit(&dma_spi1_tx);		
		
	/* Set DMA options for RX stream */
	dma_spi1_rx.Instance = DMA2_Stream2;
	dma_spi1_rx.Init.Channel = DMA_CHANNEL_3;
	dma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	dma_spi1_rx.Init.Mode = DMA_NORMAL;
	dma_spi1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
	dma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	dma_spi1_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
	dma_spi1_rx.Init.MemBurst = DMA_MBURST_SINGLE;
	dma_spi1_rx.Init.MemDataAlignment =DMA_MDATAALIGN_BYTE;
	dma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
	dma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	dma_spi1_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
	HAL_DMA_Init(&dma_spi1_rx);
	
	/* Set DMA options for TX stream */
	dma_spi1_tx.Instance = DMA2_Stream3;
	dma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
	dma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	dma_spi1_tx.Init.Mode = DMA_NORMAL;
	dma_spi1_tx.Init.Priority = DMA_PRIORITY_HIGH;
	dma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	dma_spi1_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
	dma_spi1_tx.Init.MemBurst = DMA_MBURST_SINGLE;
	dma_spi1_tx.Init.MemDataAlignment =DMA_MDATAALIGN_BYTE;
	dma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
	dma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	dma_spi1_tx.Init.PeriphBurst = DMA_PBURST_INC4;
	HAL_DMA_Init(&dma_spi1_tx);
	
	  __HAL_LINKDMA(hspi, hdmarx, dma_spi1_rx);
		__HAL_LINKDMA(hspi, hdmatx, dma_spi1_tx);
  }

}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{

  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __SPI1_CLK_DISABLE();
  
    /**SPI1 GPIO Configuration    
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(hspi->hdmatx);
  }
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */

}
/* this was replaced by code above due to the addition of DMA
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{

  if(hspi->Instance==SPI1)
  {
    / * Peripheral clock disable * /
    __SPI1_CLK_DISABLE();
  
    / **SPI1 GPIO Configuration    
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    * /
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  }
} */

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==USART3)
  {
    /* Peripheral clock enable */
    __USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    /* Sets the priority grouping field */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==USART3)
  {
    /* Peripheral clock disable */
    __USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

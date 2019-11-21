
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "gecho_gpio_app.h"
#include "gecho_structures.h"


//Below is my macro for quick HIZ; 1UL << 10
#define HIZ()(GPIOE->BSRRH = (1UL << 10);)  

extern void TM_DelayMicros(uint32_t micros);

#ifdef HAL_SPI_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SPI_TIMEOUT_VALUE  10
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
//static void SPI_TxCloseIRQHandler(SPI_HandleTypeDef *hspi);
//static void SPI_TxISR(SPI_HandleTypeDef *hspi);
//static void SPI_RxCloseIRQHandler(SPI_HandleTypeDef *hspi);
//static void SPI_2LinesRxISR(SPI_HandleTypeDef *hspi);
//static void SPI_RxISR(SPI_HandleTypeDef *hspi);
//static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma);
//static void SPI_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
//static void SPI_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma);
//static void SPI_DMAError(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef SPI_WaitOnFlagUntilTimeout_R2(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status, uint32_t Timeout);
HAL_StatusTypeDef SPI_WaitOnFlagUntilTimeout_R2_simple(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status, uint32_t Timeout); 
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_1B_EXT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_1B_EXT_LONG(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_1B_STD(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_2B_EXT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_3B_EXT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_4B_EXT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size, uint32_t Timeout);
HAL_StatusTypeDef Setup_HAL_SPI_Transmit_RFFE_1B_EXT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size);

HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_1B_STD_hardcode_ready(SPI_HandleTypeDef *hspi, uint32_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_1B_STD_hardcode_ethb(SPI_HandleTypeDef *hspi, uint32_t Size, uint32_t Timeout);

HAL_StatusTypeDef HAL_SPI_Transmit_DMA_JV(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef SPI_WaitOnFlagUntilTimeout_JV(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status, uint32_t Timeout);
void SPI_DMATransmitCplt_JV(DMA_HandleTypeDef *hdma);
void SPI_DMAError_JV(DMA_HandleTypeDef *hdma);
void DMA_SetConfig_JV(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);

SPI_HandleTypeDef hspi1;
//DMA_HandleTypeDef dma_spi1;
extern DMA_HandleTypeDef dma_spi1_tx;
DMA_Stream_TypeDef dma_tx_stream;
DMA_Stream_TypeDef dma_rx_stream;

int calc_parity(unsigned int i_bin, int nbits);

void MX_SPI1_Init(uint32_t speedscale)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
	
	//SPI_BaudRatePrescaler_8 = 6.7MHz
  //SPI_BaudRatePrescaler_4 = 13.xMHz
  //SPI_BaudRatePrescaler_2 = 26.8MHz
  switch(speedscale) {
          case 1: hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;  break;          
          case 2: hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;  break;          
          case 3: hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;  break;          
          default: hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; break;
  }
	
  HAL_SPI_Init(&hspi1);
} 

/** 
  * Enable DMA controller clock
//Do this last
  / *##-4- Configure the NVIC for DMA #########################################* / 
  / * NVIC configuration for DMA transfer complete interrupt (SPI3_TX) * /
  HAL_NVIC_SetPriority(SPIx_DMA_TX_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(SPIx_DMA_TX_IRQn);
    
  / * NVIC configuration for DMA transfer complete interrupt (SPI3_RX) * /
  HAL_NVIC_SetPriority(SPIx_DMA_RX_IRQn, 0, 0);   
  HAL_NVIC_EnableIRQ(SPIx_DMA_RX_IRQn);

  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	//HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

}

//void DMA2_Stream3_IRQn(void){
void DMA2_Stream3_IRQHandler (void)
{
	HAL_DMA_IRQHandler(&dma_spi1_tx);
}	
void DMA2_Stream5_IRQHandler (void)
{
	HAL_DMA_IRQHandler(&dma_spi1_tx);
}	

//==================================================
// 1-Byte standard rffe (address < 0x1f 
//==================================================
void write_rffe_1byte_std(uint32_t usid, uint32_t reg_add, uint32_t d)
{
  uint8_t dat[3];
	 
	dat[0] = usid;
	dat[1] = reg_add;
	dat[2] = d;

	
   Setup_HAL_SPI_Transmit_RFFE_1B_EXT(&hspi1, dat, 0x4); 	

	 //Pin_E_On(5); //Data
	 Pin_E_Off(7); //Clock
   Pin_E_Off(15);	
	  
   Pin_E_On(5); //unleash SPI, remove if not DMA transfer	
   HAL_SPI_Transmit_RFFE_1B_STD(&hspi1, dat, 0x4, 50);
	  //HAL_SPI_Transmit_DMA(&hspi1, dat, 4);
			//		HAL_SPI_Transmit_DMA_JV (&hspi1, dat, 4);
	//Pin_E_Off(7); 
	// Pin_E_Off(5);  
}

void set_spi_speed(char pcmd[100])
{
    switch(pcmd[1]) {
        case '1': //6MHz
          MX_SPI1_Init(1);
          break;
          
        case '2': //13MHz
          MX_SPI1_Init(2);
          break;   

        case '3': //26MHz
          MX_SPI1_Init(3);
          break; 
    }
}

//===========================================================
// 1-Byte extended long (rffe write)   4/17/15 vcore support
// Address is 16 bits
//===========================================================
void write_rffe_1byte_ext_long(uint32_t usid, uint32_t reg_add, uint32_t d)
{
  uint8_t dat[5];
	 
	dat[0] = usid;
	dat[1] = (reg_add >> 8) & 0xff; //msb
	dat[2] = reg_add & 0xff; //lsb
	dat[3] = d;
	dat[4] = 0;
	
   Setup_HAL_SPI_Transmit_RFFE_1B_EXT(&hspi1, dat, 0x6); 
	 //Pin_E_On(5); //Data
	 Pin_E_Off(7); //Clock
   Pin_E_Off(15);	
	  	
   HAL_SPI_Transmit_RFFE_1B_EXT_LONG(&hspi1, dat, 0x6, 50);	
   Pin_E_Off(7);
   Pin_E_Off(5);	
}
//==================================================
// 1-Byte extended rffe 
//==================================================
void write_rffe_1byte_ext(uint32_t usid, uint32_t reg_add, uint32_t d)
{
  uint8_t dat[4];
	 
	dat[0] = usid;
	dat[1] = reg_add;
	dat[2] = d;
	dat[3] = 0;
	
   Setup_HAL_SPI_Transmit_RFFE_1B_EXT(&hspi1, dat, 0x5); 
	 //Pin_E_On(5); //Data
	 Pin_E_Off(7); //Clock
   Pin_E_Off(15);	
	  	
   HAL_SPI_Transmit_RFFE_1B_EXT(&hspi1, dat, 0x5, 150);	
   Pin_E_Off(7);
   Pin_E_Off(5);	
}

void one_time_setup_4(uint32_t usid, uint32_t reg_add, uint32_t d)
{
	 uint8_t dat[3];
	 
	dat[0] = usid;
	dat[1] = reg_add;
	dat[2] = d;
	
  Setup_HAL_SPI_Transmit_RFFE_1B_EXT(&hspi1, dat, 4); 	
}

//////////////////////////////////////////////////////
//   Hard Coded READY
///////////////////////////////////////////////////////
void write_rffe_1byte_std_hardcode_ready(void)
{
  uint8_t dat[3];
	 
	dat[0] = 5;
	dat[1] = 0x19;
	dat[2] = 4;
	
   Setup_HAL_SPI_Transmit_RFFE_1B_EXT(&hspi1, dat, 0x4); 	
	 Pin_E_Off(7); //Clock
	  	
   HAL_SPI_Transmit_RFFE_1B_STD_hardcode_ready(&hspi1, 0x4, 50);
	 Pin_E_Off(5);  
}
//////////////////////////////////////////////////////
//   Hard Coded ETHB
///////////////////////////////////////////////////////
void write_rffe_1byte_std_hardcode_ethb(void)
{
  uint8_t dat[3];
	 
	dat[0] = 5;
	dat[1] = 0x19;
	dat[2] = 0x25;
	
   Setup_HAL_SPI_Transmit_RFFE_1B_EXT(&hspi1, dat, 0x4); 	
	 Pin_E_Off(7); //Clock
	  	
   HAL_SPI_Transmit_RFFE_1B_STD_hardcode_ethb(&hspi1, 0x4, 50);
	 Pin_E_Off(5);  
}
//==================================================
// Send (2)-1 byte commands - consecutive, used in dynamic
// to minimize gap time
// 1-Byte standard rffe (address < 0x1f 
//==================================================
void write_rffe_1byte_std_x2(uint32_t usid1, uint32_t reg_add1, uint32_t d1, uint32_t usid2, uint32_t reg_add2, uint32_t d2, uint32_t gap)
{
   uint8_t dat1[3], dat2[3];
	 
	 dat1[0] = usid1;
	 dat1[1] = reg_add1;
	 dat1[2] = d1;
	
	 dat2[0] = usid2;
	 dat2[1] = reg_add2;
	 dat2[2] = d2;
	
   Setup_HAL_SPI_Transmit_RFFE_1B_EXT(&hspi1, dat1, 0x8); 	
	 //Pin_E_On(5); //Data
	 Pin_E_Off(7); //Clock
   Pin_E_Off(15);	
	  	
   HAL_SPI_Transmit_RFFE_1B_STD(&hspi1, dat1, 0x3, 50);
	 Pin_E_Off(7); //Clock
	 TM_DelayMicros(gap);
	 HAL_SPI_Transmit_RFFE_1B_STD(&hspi1, dat2, 0x3, 50);
   Pin_E_Off(7); 
	 Pin_E_Off(5);  
}
	
void BIT_BANG_write_rffe_1byte_std(uint32_t usid, uint32_t reg_add, uint32_t d)
{
	unsigned char dat[3];
	 
		dat[0] = usid;
		dat[1] = reg_add;
		dat[2] = d;

	  Pin_E_On(15); //Clock
	  Pin_E_On(10); //Data  
	  Pin_E_On(10); //Data
	
	  bit_bang_rffe_write_1byte_std(dat);
	
	  Pin_E_Off(15); //Clock
	  Pin_E_Off(10); //Data
}

//bit bang command that writes to vcore2 and then to ILD
void BIT_BANG_write_rffe_1byte_std_2(uint32_t usid, uint32_t reg_add, uint32_t d, uint32_t usid2, uint32_t reg_add2, uint32_t d2)
{
	unsigned char dat[3];
	unsigned char dat2[3]; 
	
		dat[0] = usid;
		dat[1] = reg_add;
		dat[2] = d;
	
	  dat2[0] = usid2;
		dat2[1] = reg_add2;
		dat2[2] = d2;

	  Pin_E_On(15); //Clock
	  Pin_E_On(10); //Data  
	  Pin_E_On(10); //Data
	
	  bit_bang_rffe_write_1byte_std_vcore2_ild(dat, dat2);
	
	  Pin_E_Off(15); //Clock
	  Pin_E_Off(10); //Data
}	

//bit bang command that writes to ILD then to vcore2
void BIT_BANG_write_rffe_1byte_std_3(uint32_t usid, uint32_t reg_add, uint32_t d, uint32_t usid2, uint32_t reg_add2, uint32_t d2)
{
	unsigned char dat[3];
	unsigned char dat2[3]; 
	
		dat[0] = usid;
		dat[1] = reg_add;
		dat[2] = d;
	
	  dat2[0] = usid2;
		dat2[1] = reg_add2;
		dat2[2] = d2;

	  Pin_E_On(15); //Clock
	  Pin_E_On(10); //Data  
	  Pin_E_On(10); //Data
	
	  bit_bang_rffe_write_1byte_std_ild_vcore2(dat, dat2);
	
	  Pin_E_Off(15); //Clock
	  Pin_E_Off(10); //Data
}	

	
//==================================================
// BIT-BANG 1-Byte extended rffe 
//==================================================
void BIT_BANG_write_rffe_1byte_xtd(uint32_t usid, uint32_t reg_add, uint32_t d)
{
    unsigned char dat[3];
	 
		dat[0] = usid;
		dat[1] = reg_add;
		dat[2] = d;

	  Pin_E_On(15); //Clock
	  Pin_E_On(10); //Data  
	  Pin_E_On(10); //Data
	
	  bit_bang_rffe_write_1byte_ext(dat);
	
	  Pin_E_Off(15); //Clock
	  Pin_E_Off(10); //Data
}
//==================================================
// BIT-BANG 2-Byte extended rffe 
//==================================================
void BIT_BANG_write_rffe_2byte_xtd(uint32_t usid, uint32_t reg_add, uint32_t *d)
{
    unsigned char dat[4];
	 
		dat[0] = usid;
		dat[1] = reg_add;
		dat[2] = d[0];
	  dat[3] = d[1];

	  Pin_E_On(15); //Clock
	  Pin_E_On(10); //Data  
	  Pin_E_On(10); //Data
	
	  bit_bang_rffe_write_2byte_ext(dat);
	
	  Pin_E_Off(15); //Clock
	  Pin_E_Off(10); //Data
}
//==================================================
// 2-Byte extended rffe (using the SPI bus)
//==================================================
void write_rffe_2byte_ext(uint32_t usid, uint32_t reg_add, uint32_t *d)
{
  uint8_t dat[4];
	 
	dat[0] = usid;
	dat[1] = reg_add;
	dat[2] = d[0];
	dat[3] = d[1];

   Setup_HAL_SPI_Transmit_RFFE_1B_EXT(&hspi1, dat, 0x6); 
	 //Pin_E_On(5); //Data, I moved this inside transmit so we open switch right before spi xfer
	 Pin_E_Off(7); //Clock	
   Pin_E_Off(15);	
	 HAL_SPI_Transmit_RFFE_2B_EXT(&hspi1, dat, 0x6, 50);
   Pin_E_Off(7);
   Pin_E_Off(5);
   Pin_E_Off(10);	
}	
	
//==================================================
// 3-Byte extended rffe 
//==================================================
void write_rffe_3byte_ext(uint32_t usid, uint32_t reg_add, uint32_t *d)
{
  uint8_t dat[5];
	 
	dat[0] = usid;
	dat[1] = reg_add;
	dat[2] = d[0];
	dat[3] = d[1];
  dat[4] = d[2];

    
	 //Pin_E_On(5); //Data, I moved this inside transmit so we open switch right before spi xfer
	 Pin_E_Off(7); //Clock	
   Pin_E_Off(15);	
	 Pin_E_Off(10);	
	
	 Setup_HAL_SPI_Transmit_RFFE_1B_EXT(&hspi1, dat, 0x7);
	 HAL_SPI_Transmit_RFFE_3B_EXT(&hspi1, dat, 0x7, 50);
   Pin_E_Off(7);
   Pin_E_Off(5);	
}	

//==================================================
// 4-Byte extended rffe 
//==================================================
void write_rffe_4byte_ext(uint32_t usid, uint32_t reg_add, uint32_t *d)
{
  uint8_t dat[6];
	 
	dat[0] = usid;
	dat[1] = reg_add;
	dat[2] = d[0];
	dat[3] = d[1];
	dat[4] = d[2];
	dat[5] = d[3];
	
   Setup_HAL_SPI_Transmit_RFFE_1B_EXT(&hspi1, dat, 0x9); 
	 //Pin_E_On(5); //Data
	 Pin_E_Off(7); //Clock
	 Pin_E_Off(15);	
   HAL_SPI_Transmit_RFFE_4B_EXT(&hspi1, dat, 0x9, 50);
   Pin_E_Off(7);
   Pin_E_Off(5);	
}
/*
    Setup of the command that follows
*/
HAL_StatusTypeDef Setup_HAL_SPI_Transmit_RFFE_1B_EXT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size)
{
 if(hspi->State == HAL_SPI_STATE_READY)
  {
    if((pData == NULL ) || (Size == 0)) 
    {
      return  HAL_ERROR;
    }

    /* Check the parameters */
    assert_param(IS_SPI_DIRECTION_2LINES_OR_1LINE(hspi->Init.Direction));

    /* Process Locked */
    __HAL_LOCK(hspi);

    /* Configure communication */
    hspi->State = HAL_SPI_STATE_BUSY_TX;
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
		
		hspi->pTxBuffPtr = pData;
    hspi->TxXferSize = Size;
    hspi->TxXferCount = Size;

    /*Init field not used in handle to zero */
    hspi->TxISR = 0;
    hspi->RxISR = 0;
    hspi->RxXferSize   = 0;
    hspi->RxXferCount  = 0;

    /* Check if the SPI is already enabled */ 
    if((hspi->Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
    {
      /* Enable SPI peripheral */
      __HAL_SPI_ENABLE(hspi);
    }
	}
	
return HAL_OK;
}

/**
  * @brief  Transmit an amount of data in no-blocking mode with DMA
  * @param  hspi: SPI handle
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be sent
  * @retval HAL status
  *
  * Jess' version with for PC-106
****/
HAL_StatusTypeDef HAL_SPI_Transmit_DMA_JV(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
	// if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, 100) != HAL_OK) return HAL_TIMEOUT;
	
  //if(hspi->State == HAL_SPI_STATE_READY)
  //{
    if((pData == NULL) || (Size == 0))
    {
      return  HAL_ERROR;
    }

    /* Check the parameters */
    assert_param(IS_SPI_DIRECTION_2LINES_OR_1LINE(hspi->Init.Direction));

    /* Process Locked */
    __HAL_LOCK(hspi);

    /* Configure communication */
    hspi->State       = HAL_SPI_STATE_BUSY_TX;
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;

    hspi->pTxBuffPtr  = pData;
    hspi->TxXferSize  = Size;
    hspi->TxXferCount = Size;

    /*Init field not used in handle to zero */
    hspi->TxISR = 0;
    hspi->RxISR = 0;
    hspi->RxXferSize   = 0;
    hspi->RxXferCount  = 0;

    /* Configure communication direction : 1Line */
    if(hspi->Init.Direction == SPI_DIRECTION_1LINE)
    {
      __HAL_SPI_1LINE_TX(hspi);
    }

    /* Reset CRC Calculation */
    if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLED)
    {
      __HAL_SPI_RESET_CRC(hspi);
    }

    /* Set the SPI TxDMA transfer complete callback */
    hspi->hdmatx->XferCpltCallback = SPI_DMATransmitCplt_JV;

    /* Set the DMA error callback */
    hspi->hdmatx->XferErrorCallback = SPI_DMAError_JV;

    /* Enable the Tx DMA Stream */
    HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DR, hspi->TxXferCount);

    /* Enable Tx DMA Request */
    hspi->Instance->CR2 |= SPI_CR2_TXDMAEN;

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    /* Check if the SPI is already enabled */ 
    if((hspi->Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
    {
      /* Enable SPI peripheral */
      __HAL_SPI_ENABLE(hspi);
    }

    return HAL_OK;
  //}
  //else
  //{
  //  return HAL_BUSY;
  //}
}

/**
  * @brief  Transmit an amount of data in blocking mode
  * @param  hspi: SPI handle
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be sent
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_1B_EXT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size, uint32_t Timeout)
{
  uint32_t  cmd_frame, cmd_par, addr_par, d_par;
	uint32_t byte0=0, byte1=0, byte2=0, byte3=0;  
	
	 cmd_frame = (pData[0] << 8);       
   cmd_par = calc_parity(cmd_frame, 12); //x500
   addr_par = calc_parity(pData[1], 8);    
   d_par = calc_parity(pData[2], 8);

   byte0 = cmd_frame >> 4;
   byte1 =  (cmd_par<<3) +  ((pData[1] & 0xe0) >> 5); //BC is removed bec 0x0 on 1 byte
   byte2 = ((pData[1] & 0x1F) <<3) + (addr_par <<2) + ((pData[2] & 0xc0)>> 6);
   byte3 = ((pData[2] & 0x3f) <<2) + (d_par << 1); 
	
    /* Transmit data in 8 Bit mode */
 //   if(hspi->Init.DataSize == SPI_DATASIZE_8BIT)
 //   {
	    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			Pin_E_On(5); //Unleash SPI Data on the last instant
			
			//Clock is blocked
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = 0x2; //SSC
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			Pin_E_On(7); //Unblock the clock		
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = byte0;			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte1;					
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = byte2;			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte3;		

		/* Wait until TXE flag is set to send data */
    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
    {
      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
      return HAL_TIMEOUT;
    }
		//ADD SPMI cycles
		switch_port_to_bit_bang_hiz();
				  bang_clock_data(0);		
				  bang_clock_data(0);		
			    bus_park_cycle();
		

 
    /* Clear OVERUN flag in 2 Lines communication mode because received is not read */
    if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
    {
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }

    hspi->State = HAL_SPI_STATE_READY; 

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}

/**
  * 1 Byte extended write long (16-Bit Address)
  * @brief  Transmit an amount of data in blocking mode
  * @param  hspi: SPI handle
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be sent
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_1B_EXT_LONG(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size, uint32_t Timeout)
{
	uint32_t  cmd_frame, cmd_par;
  uint32_t	msb_addr_par, lsb_addr_par, msb_addr, lsb_addr, dat, d_par;
	uint32_t byte0=0, byte1=0, byte2=0, byte3=0, byte4=0;  
	
	//dat[0] = usid;
	//dat[1] = (reg_add >> 8) & 0xff; //msb
	//dat[2] = reg_add & 0xff; //lsb
	//dat[3] = d;
	
	 cmd_frame = (pData[0] << 8) + 0x30;  //9 bits -> pData[0] is the usid, 0x30 is the cmd (6), and BC =0 for 1byte of data      
   cmd_par = calc_parity(cmd_frame, 12); //
	 msb_addr = pData[1]; 
	 lsb_addr = pData[2]; 	
   msb_addr_par = calc_parity(pData[1], 8);    
	 lsb_addr_par = calc_parity(pData[2], 8);    
	 dat = pData[3];
   d_par = calc_parity(pData[3], 8);
 
   
//SPI byte = MIPI stream
   byte0 = cmd_frame >> 4;
   byte1 = ((cmd_frame & 0xf) << 4) + (cmd_par << 3) + ((msb_addr & 0xe0) >> 5);
	 byte2 = ((msb_addr & 0x1f) << 3) + (msb_addr_par << 2) + ((lsb_addr & 0xc0) >> 6); 
	 byte3 = ((lsb_addr & 0x3f) << 2) + (lsb_addr_par << 1) + ((dat & 0x80) >> 7);
	 byte4 = ((dat & 0x7f) << 1) + d_par;
	
//test pattern w2 5 aa 11aa

    /* Transmit data in 8 Bit mode */
    //if(hspi->Init.DataSize == SPI_DATASIZE_8BIT)
    //{
	    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			Pin_E_On(5); //Unleash SPI Data on the last instant
	
	   //Clock is blocked
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = 0x2; //SSC
				
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			Pin_E_On(7); //Unblock the clock			
	
	    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = byte0;			
      
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte1;				
			
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = byte2;			
			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte3;					
			
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte4;					
	
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			switch_port_bus_park_cycle();

      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = 0x0;	
			
			/* Wait until TXE flag is set to send data */
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
			{
				hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
				return HAL_TIMEOUT;
			}

		//ADD SPMI cycles
				  switch_port_to_bit_bang_hiz();
				  bang_clock_data(0);		
				  bang_clock_data(0);		
			    bus_park_cycle();
		
 
    /* Clear OVERUN flag in 2 Lines communication mode because received is not read */
    if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
    {
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }

    hspi->State = HAL_SPI_STATE_READY; 

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}
	
/**
  * @brief  Transmit an amount of data in blocking mode
  * @param  hspi: SPI handle
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be sent
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_2B_EXT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size, uint32_t Timeout)
{
  uint32_t  cmd_frame, cmd_par, addr_par, d_par, d2_par;
	uint32_t byte0=0, byte1=0, byte2=0, byte3=0, byte4=0;  
	
	 cmd_frame = (pData[0] << 8) + 1;     //added the byte count for correct parity calculation  
   cmd_par = calc_parity(cmd_frame, 12); //x500
   addr_par = calc_parity(pData[1], 8);    
   d_par = calc_parity(pData[2], 8);
	 d2_par = calc_parity(pData[3], 8); 
   
//SPI byte = MIPI stream
   byte0 = cmd_frame >> 4;
   byte1 = (0x01<<4)  + (cmd_par<<3) +  ((pData[1] & 0xe0) >> 5); //BC = 0x1 for 2 bytes
   byte2 = ((pData[1] & 0x1F) <<3) + (addr_par <<2) + ((pData[2] & 0xc0)>> 6);
   byte3 = ((pData[2] & 0x3f) <<2) + (d_par << 1) + ((pData[3] & 0x80) >> 7);
	 byte4 = ((pData[3] & 0x7f) <<1) + (d2_par); 
	
//test pattern w2 5 aa 11aa

    /* Transmit data in 8 Bit mode */
    //if(hspi->Init.DataSize == SPI_DATASIZE_8BIT)
    //{
	    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			Pin_E_On(5); //Unleash SPI Data on the last instant
			
			//Clock is blocked
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = 0x2; //SSC
				
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			Pin_E_On(7); //Unblock the clock			
			
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = byte0;			
      
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte1;				
			
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = byte2;			
			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte3;					
			
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte4;					
			
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			switch_port_bus_park_cycle();

      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = 0x0;				
    /* Wait until TXE flag is set to send data */
    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
    {
      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
      return HAL_TIMEOUT;
    }

    /* Wait until Busy flag is reset before disabling SPI */
//    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_BSY, SET, Timeout) != HAL_OK)
//    { 
//      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
//      return HAL_TIMEOUT;
//    }
 
    /* Clear OVERUN flag in 2 Lines communication mode because received is not read */
    if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
    {
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }

    hspi->State = HAL_SPI_STATE_READY; 

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}

///HARDCODE ethb
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_1B_STD_hardcode_ethb(SPI_HandleTypeDef *hspi, uint32_t Size, uint32_t Timeout)
{
   uint32_t byte0=0x55, byte1=0x99, byte2=0x28;  //// Byte0=0x55, Byte1=0x99, Byte2=0x28 et-hb

    /* Transmit data in 8 Bit mode */
			
			//Clock is blocked
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = 0x2; //SSC				

      Pin_E_On(5); //5 on for data
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			Pin_E_On(7); //Unblock the clock			
			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = byte0;			
			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte1;			
			
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = byte2;
				
      //Pin_E_Off(7); //block clk for next cycle
    /* Wait until TXE flag is set to send data */
    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
    {
      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
      return HAL_TIMEOUT;
    }

    /* Wait until Busy flag is reset before disabling SPI */
    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_BSY, SET, Timeout) != HAL_OK)
    { 
      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
      return HAL_TIMEOUT;
    }
 
    /* Clear OVERUN flag in 2 Lines communication mode because received is not read */
    //if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
    //{
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    //}

    hspi->State = HAL_SPI_STATE_READY; 

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}

///HARDCODE ready
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_1B_STD_hardcode_ready(SPI_HandleTypeDef *hspi, uint32_t Size, uint32_t Timeout)
{
   uint32_t byte0=0x55, byte1=0x98, byte2=0x20;  //// Byte0=0x55, Byte1=0x98, Byte2=0x20 ready

    /* Transmit data in 8 Bit mode */
			
			//Clock is blocked
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = 0x2; //SSC				

      Pin_E_On(5); //5 on for data
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			Pin_E_On(7); //Unblock the clock			
			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = byte0;			
			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte1;			
			
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = byte2;
				
      //Pin_E_Off(7); //block clk for next cycle
    /* Wait until TXE flag is set to send data */
    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
    {
      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
      return HAL_TIMEOUT;
    }

    /* Wait until Busy flag is reset before disabling SPI */
    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_BSY, SET, Timeout) != HAL_OK)
    { 
      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
      return HAL_TIMEOUT;
    }
 
    /* Clear OVERUN flag in 2 Lines communication mode because received is not read */
    //if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
    //{
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    //}

    hspi->State = HAL_SPI_STATE_READY; 

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}
/**
  * Similar to function above just for 3 bytes
  * Issue - 1 extra clock cyle after bus park
  * this is used for addess < 0x20 - PA's or Session registers
*/
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_1B_STD(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size, uint32_t Timeout)
{
  uint32_t  cmd_frame, cmd_par,  d_par;
	uint32_t byte0=0, byte1=0, byte2=0;  
	
	 cmd_frame = (pData[0] << 8) + 0x40 + (pData[1] & 0x1f);  //12 bits + parity       
   cmd_par = calc_parity(cmd_frame, 12); 
   d_par = calc_parity(pData[2], 8);

   byte0 = (cmd_frame >> 4) & 0xff; //USID + CMD + first MSB of Addr
   byte1 = ((pData[1] & 0xf) << 4) +  (cmd_par << 3) + ((pData[2] & 0xe0) >> 5); //4Bit addr + addr par + first 3 bits of data
   byte2 = ((pData[2] & 0x1f) << 3) + (d_par << 2); 
	
    /* Transmit data in 8 Bit mode */
	    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			Pin_E_On(5); //Unleash SPI Data on the last instant
			
			//Clock is blocked
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
	    //if (SPI_WaitOnFlagUntilTimeout_R2_simple(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = 0x2; //SSC				
			
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			//if (SPI_WaitOnFlagUntilTimeout_R2_simple(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			Pin_E_On(7); //Unblock the clock			
			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			//if (SPI_WaitOnFlagUntilTimeout_R2_simple(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = byte0;			
			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			//if (SPI_WaitOnFlagUntilTimeout_R2_simple(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte1;			
			
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			//if (SPI_WaitOnFlagUntilTimeout_R2_simple(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = byte2;
				
    /* Wait until TXE flag is set to send data */
    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
    {
      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
      return HAL_TIMEOUT;
    }

		 //ADD SPMI cycles
				switch_port_to_bit_bang_hiz();
        bang_clock_data(0);
				bang_clock_data(0);		
			  bus_park_cycle();
	

    /* Clear OVERUN flag in 2 Lines communication mode because received is not read */
    //if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
    //{
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    //}

    hspi->State = HAL_SPI_STATE_READY; 

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   HAL_SPI_Transmit_RFFE_3B_EXT
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_3B_EXT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size, uint32_t Timeout)
{
  uint32_t  cmd_frame, cmd_par, addr_par, d_par, d2_par, d3_par;
	uint32_t byte0=0, byte1=0, byte2=0, byte3=0, byte4=0, byte5=0;  
	
	 cmd_frame = (pData[0] << 8) + 2;     //added the byte count for correct parity calculation  
   cmd_par = calc_parity(cmd_frame, 12); //x500
   addr_par = calc_parity(pData[1], 8);    
   d_par = calc_parity(pData[2], 8);
	 d2_par = calc_parity(pData[3], 8);
   d3_par = calc_parity(pData[4], 8);	

   byte0 = cmd_frame >> 4;
   byte1 = (0x02<<4)  + (cmd_par<<3) +  ((pData[1] & 0xe0) >> 5); //BC = 0x2 for 3 bytes
   byte2 = ((pData[1] & 0x1F) <<3) + (addr_par <<2) + ((pData[2] & 0xc0)>> 6);
   byte3 = ((pData[2] & 0x3f) <<2) + (d_par << 1) + ((pData[3] & 0x80) >> 7);
	 byte4 = ((pData[3] & 0x7f) <<1) + (d2_par); 
	 byte5 = (pData[4] & 0xff) ; //parity and bus park will be bit banged.
	
//test pattern w2 5 aa 11aa

    /* Transmit data in 8 Bit mode */
    //if(hspi->Init.DataSize == SPI_DATASIZE_8BIT)
    //{
		  if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			Pin_E_On(5); //Unleash SPI Data on the last instant
			
			//Clock is blocked
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = 0x2; //SSC				
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			Pin_E_On(7); //Unblock the clock			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = byte0;			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte1;					
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = byte2;			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte3;					
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte4;				
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = byte5;					
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;			
			switch_port_to_bit_bang();
			bang_clock_data(d3_par);
			
			bang_clock_down();
			bus_park_cycle();
   // }


    /* Wait until TXE flag is set to send data */
    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
    {
      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
      return HAL_TIMEOUT;
    }

    /* Wait until Busy flag is reset before disabling SPI */
//    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_BSY, SET, Timeout) != HAL_OK)
//    { 
//      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
//      return HAL_TIMEOUT;
//    }
 
    /* Clear OVERUN flag in 2 Lines communication mode because received is not read */
    if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
    {
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }

    hspi->State = HAL_SPI_STATE_READY; 

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}


/* 4-Byte extended command */
HAL_StatusTypeDef HAL_SPI_Transmit_RFFE_4B_EXT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size, uint32_t Timeout)
{
  uint32_t  cmd_frame, cmd_par, addr_par; 
	uint32_t b[10], dp[6];
	
	 cmd_frame = (pData[0] << 8) + 0x03;       
   cmd_par = calc_parity(cmd_frame, 12); //x500
   addr_par = calc_parity(pData[1], 8);    
   dp[0] = calc_parity(pData[2], 8);
   dp[1] = calc_parity(pData[3], 8);
   dp[2] = calc_parity(pData[4], 8);
   dp[3] = calc_parity(pData[5], 8);
   
   b[0] = (cmd_frame >> 4) & 0xFF;
   b[1] = (0x03<<4)  + (cmd_par<<3) +  ((pData[1] & 0xe0) >> 5); //BC=0x3 for 4 bytes
   b[2] = ((pData[1] & 0x1F) <<3) + (addr_par <<2) + ((pData[2] & 0xc0)>> 6);
   b[3] = ((pData[2] & 0x3f) <<2) + (dp[0] << 1) + ((pData[3] & 0x80) >>7); //last bit = first (MSB) bit data1
   b[4] = (pData[3] << 1) + dp[1]; //data1 <<1 + parity
   b[5] = pData[4]; //bit to bit equal
   b[6] = (dp[2] <<7) + (pData[5] >>1); //data2 parity + 7 bits of data3
   b[7] = ((pData[5] & 0x01) << 7) + (dp[3] <<6) ; //da[3] parity <<6, first 6 bits of data4 
	 
    /* Transmit data in 8 Bit mode */
    //if(hspi->Init.DataSize == SPI_DATASIZE_8BIT)
    //{
		  if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			Pin_E_On(5); //Unleash SPI Data on the last instant
			
			//Clock is blocked
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = 0x2; //SSC				
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			Pin_E_On(7); //Unblock the clock			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = b[0];			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = b[1];					
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = b[2];			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = b[3];					
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = b[4];			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = b[5];				
			if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
      SPI1->DR = b[6];			
      if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK) return HAL_TIMEOUT;
			SPI1->DR = b[7];		


    /* Wait until TXE flag is set to send data */
    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_TXE, RESET, Timeout) != HAL_OK)
    {
      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
      return HAL_TIMEOUT;
    }

    /* Wait until Busy flag is reset before disabling SPI */
//    if(SPI_WaitOnFlagUntilTimeout_R2(hspi, SPI_FLAG_BSY, SET, Timeout) != HAL_OK)
//    { 
//      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
//      return HAL_TIMEOUT;
//    }
 
    /* Clear OVERUN flag in 2 Lines communication mode because received is not read */
    if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
    {
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }

    hspi->State = HAL_SPI_STATE_READY; 

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}

/**
  * @brief This function handles SPI Communication Timeout.
  * @param hspi: SPI handle
  * @retval HAL status
  */
HAL_StatusTypeDef SPI_WaitOnFlagUntilTimeout_R2(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status, uint32_t Timeout)  
{
  uint32_t timeout = 0;

  timeout = HAL_GetTick() + Timeout;

  /* Wait until flag is set */
  if(Status == RESET)
  {
    while(__HAL_SPI_GET_FLAG(hspi, Flag) == RESET)
    {
      if(Timeout != HAL_MAX_DELAY)
      {
        if(HAL_GetTick() >= timeout)
        {
          /* Disable the SPI and reset the CRC: the CRC value should be cleared
             on both master and slave sides in order to resynchronize the master
             and slave for their respective CRC calculation */

          /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
          __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

          /* Reset CRC Calculation */
          if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLED)
          {
            __HAL_SPI_RESET_CRC(hspi);
          }

          hspi->State= HAL_SPI_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspi);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  else
  {
    while(__HAL_SPI_GET_FLAG(hspi, Flag) != RESET)
    {
      if(Timeout != HAL_MAX_DELAY)
      {
        if(HAL_GetTick() >= timeout)
        {
          /* Disable the SPI and reset the CRC: the CRC value should be cleared
             on both master and slave sides in order to resynchronize the master
             and slave for their respective CRC calculation */

          /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
          __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

          /* Reset CRC Calculation */
          if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLED)
          {
            __HAL_SPI_RESET_CRC(hspi);
          }

          hspi->State= HAL_SPI_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspi);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}


//Modified SPI unlock process, no timeout
HAL_StatusTypeDef SPI_WaitOnFlagUntilTimeout_R2_simple(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status, uint32_t Timeout)   
{
   uint32_t timeout = 0;

  timeout = HAL_GetTick() + Timeout;

    while(__HAL_SPI_GET_FLAG(hspi, Flag) == RESET)
    {
      if(Timeout != HAL_MAX_DELAY)
      {
        if(HAL_GetTick() >= timeout)
        {
          /* Disable the SPI and reset the CRC: the CRC value should be cleared
             on both master and slave sides in order to resynchronize the master
             and slave for their respective CRC calculation */

          /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
          __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

          /* Reset CRC Calculation */
          if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLED)
          {
            __HAL_SPI_RESET_CRC(hspi);
          }

          hspi->State= HAL_SPI_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspi);

          return HAL_TIMEOUT;
        }
	 }
 }
		
			 return HAL_OK;
}

/**
  * @brief DMA SPI transmit process complete callback 
  * @param hdma : DMA handle
  * @retval None
  
  * copied from stm32f4xx_hal_spi.c
*/
void SPI_DMATransmitCplt_JV(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Wait until TXE flag is set to send data */
  if(SPI_WaitOnFlagUntilTimeout_JV(hspi, SPI_FLAG_TXE, RESET, SPI_TIMEOUT_VALUE) != HAL_OK)
  {
    hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
  }

  /* Disable Tx DMA Request */
  hspi->Instance->CR2 &= (uint32_t)(~SPI_CR2_TXDMAEN);

  /* Wait until Busy flag is reset before disabling SPI */
  if(SPI_WaitOnFlagUntilTimeout_JV(hspi, SPI_FLAG_BSY, SET, SPI_TIMEOUT_VALUE) != HAL_OK)
  {
    hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
  }

  hspi->TxXferCount = 0;

  hspi->State = HAL_SPI_STATE_READY;

  /* Clear OVERUN flag in 2 Lines communication mode because received is not read */
  if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
  {
    __HAL_SPI_CLEAR_OVRFLAG(hspi);
  }

  /* Check if Errors has been detected during transfer */
  if(hspi->ErrorCode != HAL_SPI_ERROR_NONE)
  {
    HAL_SPI_ErrorCallback(hspi);
  }
  else
  {
    HAL_SPI_TxCpltCallback(hspi);
  }
}

/**
  * @brief DMA SPI communication error callback 
  * @param hdma : DMA handle
  * @retval None
   Error callback - copied from sm32f4xx_hal_spi.c
  */
void SPI_DMAError_JV(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = (SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  hspi->TxXferCount = 0;
  hspi->RxXferCount = 0;
  hspi->State= HAL_SPI_STATE_READY;
  hspi->ErrorCode |= HAL_SPI_ERROR_DMA;
  HAL_SPI_ErrorCallback(hspi);
}

/**
  * @brief  Sets the DMA Transfer parameter.
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.
  * @param  SrcAddress: The source memory Buffer address
  * @param  DstAddress: The destination memory Buffer address
  * @param  DataLength: The length of data to be transferred from source to destination
  * @retval HAL status
  */
void DMA_SetConfig_JV(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  /* Configure DMA Stream data length */
  hdma->Instance->NDTR = DataLength;

  /* Peripheral to Memory */
  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {
    /* Configure DMA Stream destination address */
    hdma->Instance->PAR = DstAddress;

    /* Configure DMA Stream source address */
    hdma->Instance->M0AR = SrcAddress;
  }
  /* Memory to Peripheral */
  else
  {
    /* Configure DMA Stream source address */
    hdma->Instance->PAR = SrcAddress;
    
    /* Configure DMA Stream destination address */
    hdma->Instance->M0AR = DstAddress;
  }
}	
/**
  * @brief This function handles SPI Communication Timeout.
  * @param hspi: SPI handle
  * @retval HAL status
  */
HAL_StatusTypeDef SPI_WaitOnFlagUntilTimeout_JV(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status, uint32_t Timeout)  
{
  uint32_t timeout = 0;

  timeout = HAL_GetTick() + Timeout;

  /* Wait until flag is set */
  if(Status == RESET)
  {
    while(__HAL_SPI_GET_FLAG(hspi, Flag) == RESET)
    {
      if(Timeout != HAL_MAX_DELAY)
      {
        if(HAL_GetTick() >= timeout)
        {
          /* Disable the SPI and reset the CRC: the CRC value should be cleared
             on both master and slave sides in order to resynchronize the master
             and slave for their respective CRC calculation */

          /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
          __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

          /* Reset CRC Calculation */
          if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLED)
          {
            __HAL_SPI_RESET_CRC(hspi);
          }

          hspi->State= HAL_SPI_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspi);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  else
  {
    while(__HAL_SPI_GET_FLAG(hspi, Flag) != RESET)
    {
      if(Timeout != HAL_MAX_DELAY)
      {
        if(HAL_GetTick() >= timeout)
        {
          /* Disable the SPI and reset the CRC: the CRC value should be cleared
             on both master and slave sides in order to resynchronize the master
             and slave for their respective CRC calculation */

          /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
          __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

          /* Reset CRC Calculation */
          if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLED)
          {
            __HAL_SPI_RESET_CRC(hspi);
          }

          hspi->State= HAL_SPI_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspi);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}
/* mipi_rffe odd parity calculator */
int calc_parity(unsigned int i_bin, int nbits)
{
	unsigned int iparity, mask=0x1;
	int x, dat;
	
	mask<<=(nbits-1);
	
	dat = (i_bin & mask) >> (nbits-1); //initial
	mask>>=1;
	
	for (x=(nbits-2);x>-1;x--)
	 {
	 	dat = dat ^ ((i_bin & mask) >> x);
		mask>>=1;
	 }
	 
	iparity = (~dat & 0x1) ; //odd parity
	return iparity;
}

/**
  * @}
  */

#endif /* HAL_SPI_MODULE_ENABLED */


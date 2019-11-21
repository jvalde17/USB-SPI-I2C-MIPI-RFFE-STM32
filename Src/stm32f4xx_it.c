/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @date    20/10/2014 09:22:51
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "decode.h"

/* External variables --------------------------------------------------------*/
#define MAX_STRLEN 100
char tx_buf[100];
extern UART_HandleTypeDef huart3;
void get_serial_cmd(void);

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
  HAL_UART_IRQHandler(&huart3);
	get_serial_cmd();
}

void get_serial_cmd(void)
{
// check if the USART3 receive interrupt flag was set

	if(__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_RXNE) ) {
		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART3->DR; // the character from the USART3 data register is saved in t

		/* check if the received character is not the LF character (used to determine end of string) 
		 * or the if the maximum string length has been been reached 
		 */
		if( (t != '\n') && (t != '\r') && (cnt < MAX_STRLEN) ){ 
			//received_string[cnt] = t;
			tx_buf[cnt] = t;
      cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			cnt = 0;
                        //USART_puts(USART3, "\nYou entered: ");
			//USART_puts(USART3, tx_buf);
                        //Decode(tx_buf); 
			decode_command(tx_buf);
                        //tx_buf[0] = '\0'; //clean buffer
		}
	}

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

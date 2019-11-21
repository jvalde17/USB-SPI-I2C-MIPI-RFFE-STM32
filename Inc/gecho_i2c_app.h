//gecho_i2c_app.h

#ifndef __GECHO_I2C_APP_H
#define __GECHO_I2C_APP_H

//#include "stdint.h"

//extern void R2_I2C1_Init(void);
void MX_I2C1_Init(void);
void MX_DMA_Init(void) ;
void R2_I2C1_Transmit_test(uint32_t address);
void R2_I2C1_write_1byte_cmd(uint32_t address, uint32_t dat0, uint32_t dat1, uint16_t w_r);
uint8_t R2_I2C1_read_1byte_cmd(uint32_t dev_address, uint32_t reg_address);

#endif

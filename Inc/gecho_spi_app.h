
#ifndef __GECHO_SPI_APP_H
#define __GECHO_SPI_APP_H

extern void MX_SPI1_Init(uint32_t);
extern void JV_SPI1_Init_DMA(SPI_TypeDef* SPIx);

extern void set_spi_speed(char pcmd[100]);
extern void write_rffe_1byte_ext(uint32_t usid, uint32_t reg_add, uint32_t d);
extern void write_rffe_1byte_std_x2(uint32_t usid1, uint32_t reg_add1, uint32_t d1, uint32_t usid2, uint32_t reg_add2, uint32_t d2, uint32_t gap);
extern void write_rffe_1byte_std(uint32_t usid, uint32_t reg_add, uint32_t d);
extern void write_rffe_1byte_std_hardcode_ready(void);
extern void write_rffe_1byte_std_hardcode_ethb(void);

extern void write_rffe_1byte_ext_long(uint32_t usid, uint32_t reg_add, uint32_t d);
extern void write_rffe_2byte_ext(uint32_t usid, uint32_t reg_add, uint32_t *d);
extern void write_rffe_3byte_ext(uint32_t usid, uint32_t reg_add, uint32_t *d);
extern void write_rffe_4byte_ext(uint32_t usid, uint32_t reg_add, uint32_t *d);

extern void BIT_BANG_write_rffe_1byte_std(uint32_t usid, uint32_t reg_add, uint32_t d);
extern void BIT_BANG_write_rffe_1byte_std_2(uint32_t usid, uint32_t reg_add, uint32_t d, uint32_t usid2, uint32_t reg_add2, uint32_t d2);
extern void BIT_BANG_write_rffe_1byte_std_3(uint32_t usid, uint32_t reg_add, uint32_t d, uint32_t usid2, uint32_t reg_add2, uint32_t d2);

extern void BIT_BANG_write_rffe_1byte_xtd(uint32_t usid, uint32_t reg_add, uint32_t d);
extern void BIT_BANG_write_rffe_2byte_xtd(uint32_t usid, uint32_t reg_add, uint32_t *d);

extern void one_time_setup_4(uint32_t usid, uint32_t reg_add, uint32_t d);


#endif

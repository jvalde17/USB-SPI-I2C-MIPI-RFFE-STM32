/*
   header file for gpio app
*/

#ifndef __GECHO_GPIO_APP_H
#define __GECHO_GPIO_APP_H

void EXTI0_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

void send_RFFE_SSC(void);
	
extern void Pin_B_On  (unsigned int num);
extern void Pin_B_Off  (unsigned int num);
extern void Pin_C_On  (unsigned int num);
extern void Pin_C_Off  (unsigned int num);
extern void Pin_D_On  (unsigned int num);
extern void Pin_D_Off  (unsigned int num);
extern void Pin_E_On  (unsigned int num);
extern void Pin_E_Off  (unsigned int num);

void set_ILD_cfg(int cfg2, int cfg1, int cfg0);

extern void LED_Off (unsigned int num);
extern void LED_On (unsigned int num);
extern void sync_loop (uint32_t delay_gap);

extern void PortE_init (void);
extern void PortD_init (void);
extern void PortB_init (void);
extern void PortC_init (void); 	

extern void set_gpio_port_d(uint16_t val);
extern void set_gpio_port_e(uint16_t val);

extern void bit_bang_rffe_write_1byte_ext(uint8_t *pData);
extern void bit_bang_rffe_write_2byte_ext(uint8_t *pData);
extern void bit_bang_rffe_write_1byte_std(uint8_t *pData);
extern void bit_bang_rffe_write_1byte_std_vcore2_ild(uint8_t *pData_1, uint8_t *pData_2);
extern void bit_bang_rffe_write_1byte_std_ild_vcore2(uint8_t *pData_1, uint8_t *pData_2);

extern uint32_t bit_bang_rffe_read_1byte_ext(uint16_t usid, uint16_t addr);
extern uint32_t bit_bang_rffe_read_1byte_ext_long(uint16_t usid, uint16_t addr);
extern uint32_t bit_bang_rffe_read_1byte_std(uint16_t usid, uint16_t addr);

extern void bang_clock_data(uint16_t d);
extern void bang_clock_up(void);
extern void bang_clock_down(void);
extern void bus_park_cycle(void);
extern void switch_port_bus_park_cycle(void);

extern void switch_port_to_bit_bang(void);
void switch_port_to_bit_bang_hiz(void);
extern void switch_port_to_spi(void);

extern void bottom_half_process(void);
extern void synch_A (void);
extern void synch_B (void);
extern void ild_sync_out (int set);

#endif


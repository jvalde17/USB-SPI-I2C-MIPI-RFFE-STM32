/*
  gpio app controls for the gecho board
  gecho_gpio_app.c
*/

#include "STM32F4xx.h"
#include "gecho_gpio_app.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal.h"
#include "gecho_structures.h"
#include "gecho_spi_app.h"

int calc_parity(unsigned int i_bin, int nbits);
void bang_clock_data(uint16_t d);
void bang_SSC_up(void);
void bang_SSC_down(void);
void bang_clock_up(void);
void bang_clock_down(void);
uint16_t bang_clock_down_read_bit(void);
void bus_park_cycle(void);

extern void TM_DelayMicros(uint32_t micros);

void vcore_dynamic_by_interrupt(void);


#define VCORE_DYN_DVS	7
#define VCORE_DYN_SYNC_LOOP 8
#define VCORE_DYN_DVS_SUDHA_SPECIAL	0xf

#define NUM_PD     16                        /* Number of user IOs in port D  */
const unsigned long led_mask[] = {1UL << 0,1UL << 1,1UL << 2,1UL << 3,1UL << 4,1UL << 5,1UL << 6,1UL << 7,1UL << 8,1UL << 9,1UL << 10,1UL << 11,1UL << 12,1UL << 13,1UL << 14,1UL << 15};

	
extern UART_HandleTypeDef huart3;
	
/* =====================================
    MIPI SSC Function
	  - Bit bang - in one function for speed
 =======================================*/
void send_RFFE_SSC(void)
{
    GPIOE->BSRRL = led_mask[3]; 
    GPIOE->BSRRL = led_mask[3]; 
	  GPIOE->BSRRL = led_mask[3]; 
	
	  GPIOE->BSRRH = led_mask[3]; 
	  GPIOE->BSRRH = led_mask[10]; //data sel_1 down
	
	  GPIOE->BSRRL = led_mask[5]; //data sel_0 up 
}	
	
/*----------------------------------------------------------------------------
  Function that turns on requested LED
 *----------------------------------------------------------------------------*/
void Pin_B_On (unsigned int num) {

   if (num < NUM_PD) {
    GPIOB->BSRRL = led_mask[num];
  }
	
}	

void Pin_B_Off (unsigned int num) {

  if (num < NUM_PD) {
    GPIOB->BSRRH = led_mask[num];
  }
}
void Pin_C_On (unsigned int num) {

   if (num < NUM_PD) {
    GPIOC->BSRRL = led_mask[num];
  }
	
}	

void Pin_C_Off (unsigned int num) {

  if (num < NUM_PD) {
    GPIOC->BSRRH = led_mask[num];
  }
}

/*
   controls for port B 5,6,7 Pins
   used as CFG(ILD) pin for VCORE2 transient test (demo)
   added 10-19-2016
*/
void set_ILD_cfg(int cfg2, int cfg1, int cfg0)
{
	 //Pins 5 -- cfg0
	 //Pins 6 -- cfg1
	 //Pins 7 -- cfg2
	 if (cfg0) 
		 GPIOB->BSRRL = led_mask[5];
	 else	GPIOB->BSRRH = led_mask[5];
	
	 if (cfg1) 
		 GPIOB->BSRRL = led_mask[6];
	 else	GPIOB->BSRRH = led_mask[6];
	
	 if (cfg2) 
		 GPIOB->BSRRL = led_mask[7];
	 else	GPIOB->BSRRH = led_mask[7];	 
}

/*----------------------------------------------------------------------------
  Function that turns on requested Pin
 *----------------------------------------------------------------------------*/
void Pin_D_On (unsigned int num) {

  if (num < NUM_PD) {
    GPIOD->BSRRL = led_mask[num];
  }
}
void Pin_D_Off (unsigned int num) {

  if (num < NUM_PD) {
    GPIOD->BSRRH = led_mask[num];
  }
}
//void CFG_D_On (unsigned int num1, unsigned int num2)
//{
//	  GPIOD->BSRRL = led_mask[num1] + led_mask[num2];
//}

void Pin_E_On (unsigned int num) {

  if (num < NUM_PD) {
    GPIOE->BSRRL = led_mask[num];
  }
}
void Pin_E_Off (unsigned int num) {

  if (num < NUM_PD) {
    GPIOE->BSRRH = led_mask[num];
  }
}

/*----------------------------------------------------------------------------
  Function that turns off requested LED
 *----------------------------------------------------------------------------*/
void LED_Off (unsigned int num) {

  if (num < NUM_PD) {
    GPIOD->BSRRH = led_mask[num];
  }
}
void LED_On (unsigned int num) {

  if (num < NUM_PD) {
    GPIOD->BSRRL = led_mask[num];
  }
}

/* --------------------------------
   sync loop (VCORE2 sync pins)
    - vcore2 sync loop is used to drive ILD slots in the absence of Delorean (vcore1) as master.
		D8 - bit 0
    D9 - bit 1
    D10 - bit 2
 ----------------------------------*/
void sync_loop (uint32_t delay_gap)
{
    GPIOD->BSRRL = led_mask[8];
	  TM_DelayMicros(delay_gap);
	  GPIOD->BSRRL = led_mask[9];
	  TM_DelayMicros(delay_gap);
	  GPIOD->BSRRL = led_mask[10];
	  TM_DelayMicros(delay_gap);
	   //ALL OFF
	  GPIOD->BSRRH = led_mask[10] | led_mask[9] | led_mask[8];
}

/* -------------------------------------
   BIT BANG SSC
- Setup as individual routine (up/down) for timing reasons
 --------------------------------------- */
void bang_SSC_up(void)
{    	
   	GPIOE->BSRRL = led_mask[3];
	  GPIOE->BSRRL = led_mask[3];
	  GPIOE->BSRRH = led_mask[2];	 
	  GPIOE->BSRRH = led_mask[2];	 
	  GPIOE->BSRRH = led_mask[2]; //clock down	
}
void bang_SSC_down(void)
{
    GPIOE->BSRRH = led_mask[3];
	  GPIOE->BSRRH = led_mask[2];	 
	  GPIOE->BSRRH = led_mask[2];	 
	  GPIOE->BSRRH = led_mask[2];	 
	  GPIOE->BSRRH = led_mask[2];	 
}

void bus_park_cycle(void)
{
	    GPIOE->BSRRH = led_mask[3]; //DATA down before HIZ so it won't float.
     
	    
	    GPIOE->BSRRL = led_mask[2];	 //bang_clock_up();  
      GPIOE->BSRRH = led_mask[10]; //HIZ DATA  
	    
	bang_clock_down();
}

/* 
    switch to bit-bang port
*/
void switch_port_to_bit_bang(void)
{
	    GPIOE->BSRRH = led_mask[7]; //CLK A down
	    GPIOE->BSRRH = led_mask[5];  //DATA A down
	
			GPIOE->BSRRL = led_mask[15]; //CLK B up	    
	    GPIOE->BSRRL = led_mask[10]; //DATA B up
	    
}

void switch_port_to_bit_bang_hiz(void)
{
	    GPIOE->BSRRH = led_mask[7]; //CLK A down
	    GPIOE->BSRRH = led_mask[5];  //DATA A down
	
			GPIOE->BSRRL = led_mask[15]; //CLK B up	    
	    GPIOE->BSRRL = led_mask[10]; //DATA B up
	    GPIOE->BSRRH = led_mask[10]; //HIZ
	    
}
void switch_port_to_spi(void)
{
			GPIOE->BSRRH = led_mask[15]; //CLK B down
	    GPIOE->BSRRL = led_mask[7]; //CLK A up
	    GPIOE->BSRRH = led_mask[10]; //DATA B Low
	    GPIOE->BSRRL = led_mask[5];  //DATA A Hi
}

/* +++++++++++++++++++++++++++++++++++++++++
   This is the bit bang mode bus park
   -used when coming from the SPI mode
++++++++++++++++++++++++++++++++++++++++++++*/
void switch_port_bus_park_cycle(void)
{
	    GPIOE->BSRRL = led_mask[15]; //CLK B up
	    GPIOE->BSRRH = led_mask[7]; //CLK A down
	    GPIOE->BSRRL = led_mask[10]; //DATA B HI
	    GPIOE->BSRRH = led_mask[5];  //DATA A Low
	
	      
	    GPIOE->BSRRL = led_mask[2];	 //bang_clock_up();  
	      
	    GPIOE->BSRRH = led_mask[3]; //DATA down before HIZ so it won't float.	  
	    GPIOE->BSRRL = led_mask[2];	 //bang_clock_up(), practically just a delay.
      GPIOE->BSRRH = led_mask[10]; //HIZ DATA  
	    
	bang_clock_down();
}
/* -------------------------------------
   BIT BANG clk and data (write cycles)
   DATA = Pin E3
   CLK  = Pin E2
 --------------------------------------- */
void bang_clock_data(uint16_t d)
{
	uint16_t bit;
	
	 //clk down
    	GPIOE->BSRRH = led_mask[2];	 
	
	 Pin_B_Off (0);
	 bit = GPIOE->IDR & led_mask[1]; //read
	
     if (d)
		 {  
			 GPIOE->BSRRL = led_mask[3];
			 GPIOE->BSRRL = led_mask[2];			 
		 }
		 else
		 {
			 GPIOE->BSRRH = led_mask[3]; 
			 GPIOE->BSRRL = led_mask[2];
		 } 
}

/* -------------------------------------
   BIT BANG clk read data bit (read cyles)
 --------------------------------------- */
uint16_t bang_clock_read_data(void)
{
   uint16_t bit;
  
	 GPIOE->BSRRH = led_mask[2]; //bang_clock_down();
	 GPIOE->BSRRL = led_mask[2]; //bang_clock_up();
   //GPIOE->BSRRL = led_mask[2];
	 //GPIOE->BSRRL = led_mask[2];
	 //GPIOE->BSRRL = led_mask[2];
	 Pin_B_Off (1);
	 
	 bit = GPIOE->IDR & led_mask[1]; //read
	 GPIOE->BSRRH = led_mask[2]; //clock down 
	
	 //bit = bang_clock_down_read_bit(); 
	
	 return bit;
}
	
void bang_clock_up(void)
{GPIOE->BSRRL = led_mask[2];}
void bang_clock_down(void)
{GPIOE->BSRRH = led_mask[2];}

uint16_t bang_clock_down_read_bit(void)
{
	uint16_t rbit;
	
     GPIOE->BSRRH = led_mask[2];
	   rbit = GPIOE->IDR & led_mask[1];
	return rbit;
}

/* =============================================
   Bit-Bang serial command
   1-byte RFFE standard (shortest version) - added 9/27/2016
   DATA = Pin E3
   CLK  = Pin E2
 ===============================================*/
void bit_bang_rffe_write_1byte_std(uint8_t *pData)
{
	uint16_t  bit;
  int x;
	
	uint32_t  cmd_frame, cmd_par,  d_par;
	uint32_t byte0=0, byte1=0, byte2=0;  
	
	cmd_frame = (pData[0] << 8) + 0x40 + (pData[1] & 0x1f);  //12 bits + parity       
  cmd_par = calc_parity(cmd_frame, 12); 
  d_par = calc_parity(pData[2], 8);

  byte0 = (cmd_frame >> 4) & 0xff; //USID + CMD + first MSB of Addr
  byte1 = ((pData[1] & 0xf) << 4) +  (cmd_par << 3) + ((pData[2] & 0xe0) >> 5); //4Bit addr + addr par + first 3 bits of data
  byte2 = ((pData[2] & 0x1f) << 3) + (d_par << 2); 
	
 //SSC
	 bang_SSC_up();
	 bang_SSC_up();
   bang_SSC_down();
	 
	//now bit banging
		for (x=7; x>-1; x--)
			{
				 bit = (byte0 >> x) & 1;
			   bang_clock_data(bit);
			}
			
		for (x=7; x>-1; x--)
			{
				 bit = (byte1 >> x) & 1;
			   bang_clock_data(bit);
			}
			
		for (x=7; x>-1; x--)
			{
				 bit = (byte2 >> x) & 1;
			   bang_clock_data(bit);
			}

     //Add 2 extra cycles for SPMI			
      bang_clock_data(0);
      bang_clock_data(0);
			
			GPIOE->BSRRH = led_mask[3]; //Data down 
			GPIOE->BSRRH = led_mask[2]; //clock down
}
	
/* =============================================
   - A special vcore2 + ild consecutive command 
   - I needed a fast turn around command in bit bang mode.
   - 1/13/2017
   Bit-Bang serial command 
   1-byte RFFE standard (shortest version) - added 9/27/2016
   DATA = Pin E3
   CLK  = Pin E2
 ===============================================*/
void bit_bang_rffe_write_1byte_std_vcore2_ild(uint8_t *pData_1, uint8_t *pData_2)
{
  uint16_t  bit;
  int x;
	
	uint32_t  cmd_frame, cmd_par,  d_par;
	uint32_t byte0=0, byte1=0, byte2=0;  
	
	cmd_frame = (pData_1[0] << 8) + 0x40 + (pData_1[1] & 0x1f);  //12 bits + parity       
  cmd_par = calc_parity(cmd_frame, 12); 
  d_par = calc_parity(pData_1[2], 8);

  byte0 = (cmd_frame >> 4) & 0xff; //USID + CMD + first MSB of Addr
  byte1 = ((pData_1[1] & 0xf) << 4) +  (cmd_par << 3) + ((pData_1[2] & 0xe0) >> 5); //4Bit addr + addr par + first 3 bits of data
  byte2 = ((pData_1[2] & 0x1f) << 3) + (d_par << 2); 
	
 //SSC
	 bang_SSC_up();
	 bang_SSC_up();
   bang_SSC_down();
	 
	//now bit banging
		for (x=7; x>-1; x--)
			{
				 bit = (byte0 >> x) & 1;
			   bang_clock_data(bit);
			}
			
		for (x=7; x>-1; x--)
			{
				 bit = (byte1 >> x) & 1;
			   bang_clock_data(bit);
			}
			
		for (x=7; x>-1; x--)
			{
				 bit = (byte2 >> x) & 1;
			   bang_clock_data(bit);
			}

     //Add 2 extra cycles for SPMI			
      bang_clock_data(0);
      bang_clock_data(0);
			
			GPIOE->BSRRH = led_mask[3]; //Data down 
			GPIOE->BSRRH = led_mask[2]; //clock down

			TM_DelayMicros(_config.vcore2_dyn_delay+3);//wait 
			
			//now add the 2nd command.
			cmd_frame = (pData_2[0] << 8) + 0x40 + (pData_2[1] & 0x1f);  //12 bits + parity       
			cmd_par = calc_parity(cmd_frame, 12); 
			d_par = calc_parity(pData_2[2], 8);

			byte0 = (cmd_frame >> 4) & 0xff; //USID + CMD + first MSB of Addr
			byte1 = ((pData_2[1] & 0xf) << 4) +  (cmd_par << 3) + ((pData_2[2] & 0xe0) >> 5); //4Bit addr + addr par + first 3 bits of data
			byte2 = ((pData_2[2] & 0x1f) << 3) + (d_par << 2); 
			
		 //SSC
			 bang_SSC_up();
			 bang_SSC_up();
			 bang_SSC_down();
			 
			//now bit banging
				for (x=7; x>-1; x--)
					{
						 bit = (byte0 >> x) & 1;
						 bang_clock_data(bit);
					}
					
				for (x=7; x>-1; x--)
					{
						 bit = (byte1 >> x) & 1;
						 bang_clock_data(bit);
					}
					
				for (x=7; x>-1; x--)
					{
						 bit = (byte2 >> x) & 1;
						 bang_clock_data(bit);
					}
					
			GPIOE->BSRRH = led_mask[3]; //Data down 
			GPIOE->BSRRH = led_mask[2]; //clock down
}

/* =============================================
   - A special ild + vcore2 consecutive command 
   - This is the reverse of code above
   - I needed a fast turn around command in bit bang mode.
   - 3/15/2017
   Bit-Bang serial command 
   1-byte RFFE standard (shortest version)
   DATA = Pin E3
   CLK  = Pin E2
 ===============================================*/
void bit_bang_rffe_write_1byte_std_ild_vcore2(uint8_t *pData_1, uint8_t *pData_2)
{
	uint16_t  bit;
  int x;
	
	uint32_t  cmd_frame, cmd_par,  d_par;
	uint32_t byte0=0, byte1=0, byte2=0;  
	
	cmd_frame = (pData_1[0] << 8) + 0x40 + (pData_1[1] & 0x1f);  //12 bits + parity       
  cmd_par = calc_parity(cmd_frame, 12); 
  d_par = calc_parity(pData_1[2], 8);

  byte0 = (cmd_frame >> 4) & 0xff; //USID + CMD + first MSB of Addr
  byte1 = ((pData_1[1] & 0xf) << 4) +  (cmd_par << 3) + ((pData_1[2] & 0xe0) >> 5); //4Bit addr + addr par + first 3 bits of data
  byte2 = ((pData_1[2] & 0x1f) << 3) + (d_par << 2); 
	
 //SSC
	 bang_SSC_up();
	 bang_SSC_up();
   bang_SSC_down();
	 
	//now bit banging
		for (x=7; x>-1; x--)
			{
				 bit = (byte0 >> x) & 1;
			   bang_clock_data(bit);
			}
			
		for (x=7; x>-1; x--)
			{
				 bit = (byte1 >> x) & 1;
			   bang_clock_data(bit);
			}
			
		for (x=7; x>-1; x--)
			{
				 bit = (byte2 >> x) & 1;
			   bang_clock_data(bit);
			}

			GPIOE->BSRRH = led_mask[3]; //Data down 
			GPIOE->BSRRH = led_mask[2]; //clock down

			TM_DelayMicros(_config.vcore2_dyn_delay+3);//wait 
			//now add the 2nd command.
			cmd_frame = (pData_2[0] << 8) + 0x40 + (pData_2[1] & 0x1f);  //12 bits + parity       
			cmd_par = calc_parity(cmd_frame, 12); 
			d_par = calc_parity(pData_2[2], 8);

			byte0 = (cmd_frame >> 4) & 0xff; //USID + CMD + first MSB of Addr
			byte1 = ((pData_2[1] & 0xf) << 4) +  (cmd_par << 3) + ((pData_2[2] & 0xe0) >> 5); //4Bit addr + addr par + first 3 bits of data
			byte2 = ((pData_2[2] & 0x1f) << 3) + (d_par << 2); 
			
		 //SSC
			 bang_SSC_up();
			 bang_SSC_up();
			 bang_SSC_down();
			 
			//now bit banging
				for (x=7; x>-1; x--)
					{
						 bit = (byte0 >> x) & 1;
						 bang_clock_data(bit);
					}
					
				for (x=7; x>-1; x--)
					{
						 bit = (byte1 >> x) & 1;
						 bang_clock_data(bit);
					}
					
				for (x=7; x>-1; x--)
					{
						 bit = (byte2 >> x) & 1;
						 bang_clock_data(bit);
					}
					
			//Add 2 extra cycles for SPMI			
      bang_clock_data(0);
      bang_clock_data(0);
					
			GPIOE->BSRRH = led_mask[3]; //Data down 
			GPIOE->BSRRH = led_mask[2]; //clock down
}


/* =============================================
   Bit-Bang serial command
   1-byte RFFE extended
   DATA = Pin E3
   CLK  = Pin E2
 ===============================================*/
void bit_bang_rffe_write_1byte_ext(uint8_t *pData)
{
  uint16_t  bit;
  int x;
	
	uint16_t  cmd_frame, cmd_par, addr_par, d_par;
	uint16_t byte0=0, byte1=0, byte2=0, byte3=0;  
	
	 cmd_frame = (pData[0] << 8);       
   cmd_par = calc_parity(cmd_frame, 12); //x500
   addr_par = calc_parity(pData[1], 8);    
   d_par = calc_parity(pData[2], 8);

   byte0 = cmd_frame >> 4;
   byte1 =  (cmd_par<<3) +  ((pData[1] & 0xe0) >> 5); //BC is removed bec 0x0 on 1 byte
   byte2 = ((pData[1] & 0x1F) <<3) + (addr_par <<2) + ((pData[2] & 0xc0)>> 6);
   byte3 = ((pData[2] & 0x3f) <<2) + (d_par << 1); 
	
	 //SSC
	 bang_SSC_up();
	 bang_SSC_up();
   bang_SSC_down();
	 
	//now bit banging
		for (x=7; x>-1; x--)
			{
				 bit = (byte0 >> x) & 1;
			   bang_clock_data(bit);
			}
			
		for (x=7; x>-1; x--)
			{
				 bit = (byte1 >> x) & 1;
			   bang_clock_data(bit);
			}
			
		for (x=7; x>-1; x--)
			{
				 bit = (byte2 >> x) & 1;
			   bang_clock_data(bit);
			}
			
		for (x=7; x>-1; x--)
			{
				 bit = (byte3 >> x) & 1;
			   bang_clock_data(bit);
			}
			
	   //Add 2 extra cycles for SPMI			
      bang_clock_data(0);
      bang_clock_data(0);

			GPIOE->BSRRH = led_mask[3]; //Data down 
			GPIOE->BSRRH = led_mask[2]; //clock down
}
/* =============================================
   Bit-Bang serial command
   2-byte RFFE extended
   DATA = Pin E3
   CLK  = Pin E2
 ===============================================*/
void bit_bang_rffe_write_2byte_ext(uint8_t *pData)
{
  uint16_t  bit;
  int x;
	
	uint16_t  cmd_frame, cmd_par, addr_par, d_par, d2_par;
	uint16_t byte0=0, byte1=0, byte2=0, byte3=0, byte4=0;  
	
	 cmd_frame = (pData[0] << 8) + 1;       //Add 1 BC
   cmd_par = calc_parity(cmd_frame, 12); //x500
   addr_par = calc_parity(pData[1], 8);    
   d_par = calc_parity(pData[2], 8);
   d2_par = calc_parity(pData[3], 8); 
	
	 byte0 = cmd_frame >> 4;
   byte1 = (0x01<<4)  + (cmd_par<<3) +  ((pData[1] & 0xe0) >> 5); //BC = 0x1 for 2 bytes
   byte2 = ((pData[1] & 0x1F) <<3) + (addr_par <<2) + ((pData[2] & 0xc0)>> 6);
   byte3 = ((pData[2] & 0x3f) <<2) + (d_par << 1) + ((pData[3] & 0x80) >> 7);
	 byte4 = ((pData[3] & 0x7f) <<1) + (d2_par); 
	 
	 //SSC
	 bang_SSC_up();
	 bang_SSC_up();
   bang_SSC_down();
	 
	//now bit banging
		for (x=7; x>-1; x--)
			{
				 bit = (byte0 >> x) & 1;
			   bang_clock_data(bit);
			}
			
		for (x=7; x>-1; x--)
			{
				 bit = (byte1 >> x) & 1;
			   bang_clock_data(bit);
			}
			
		for (x=7; x>-1; x--)
			{
				 bit = (byte2 >> x) & 1;
			   bang_clock_data(bit);
			}
			
		for (x=7; x>-1; x--)
			{
				 bit = (byte3 >> x) & 1;
			   bang_clock_data(bit);
			}
			
		for (x=7; x>-1; x--)
			{
				 bit = (byte4 >> x) & 1;
			   bang_clock_data(bit);
			}	
			
		//Add 2 extra cycles for SPMI			
      bang_clock_data(0);
      bang_clock_data(0);
			
			GPIOE->BSRRH = led_mask[3]; //Data down 
			GPIOE->BSRRH = led_mask[2]; //clock down
}

/* =============================================
   Bit-Bang I2C
   1-byte 
   DATA = Pin E3
   CLK  = Pin E2
   -> <Start><CMD><STOP>
 ===============================================*/
uint32_t bit_bang_i2c_write(uint16_t dev, uint16_t addr, uint16_t dat)
{
	int x;	
  uint32_t bit;  
	
   Pin_E_On(15); //Clock
	 Pin_E_On(10); //Data  
	 Pin_E_Off(4);
	
	//now bit bang address
	for (x=7; x>-1; x--)
			{
				 bit = (dev >> x) & 1;
			   bang_clock_data(bit);
				 //HAL_Delay();
			}
   
  //			
		return 0;	
}

/* =============================================
   Bit-Bang serial command - extended long (16 bit address)
   1-byte(data), 2-byte(address) RFFE extended read long
   DATA = Pin E3
   CLK  = Pin E2
 ===============================================*/
uint32_t bit_bang_rffe_read_1byte_ext_long(uint16_t usid, uint16_t addr)
{
	uint32_t rdat=0, bit;
  int x;	  
  uint16_t  cmd_frame, cmd_par, msb_addr_par, lsb_addr_par, msb_addr, lsb_addr; 
	uint32_t w;
	
	Pin_E_On(15); //Clock
	Pin_E_On(10); //Data  
	Pin_E_Off(4);
	
	msb_addr = (addr >>8) & 0xff;
	lsb_addr = addr & 0xff; 
	  //bc =0x0
	  cmd_frame = (usid << 8) + (0x7 << 3); 
	  cmd_par = calc_parity(cmd_frame, 12); 
	  msb_addr_par = calc_parity(msb_addr, 8);
    lsb_addr_par = calc_parity(lsb_addr, 8);	
	
	 w = (cmd_frame <<20) + (cmd_par <<19) + (msb_addr <<11) + (msb_addr_par<<10) + (lsb_addr <<2) + (lsb_addr_par<<1);
	
	 //SSC
	 bang_SSC_up();
	 bang_SSC_up();
   bang_SSC_down();
	
	  //now bit banging
	 for (x=31; x>-1; x--)
			{
				 bit = (w >> x) & 1;
			   bang_clock_data(bit);
			}
   	bus_park_cycle();	
	

			//Now the read cycles
    for (x=10; x>0; x--)
			{
			   bit = bang_clock_read_data();
				 rdat += bit << (x-1);
			} 

			GPIOE->BSRRH = led_mask[10]; //HIZ DATA  
			
    Pin_E_Off(15); //Clock
	  Pin_E_Off(10); //Data  
		rdat = rdat >> 2; //Making it compatible with old B style Firmware.	
			
		Pin_B_On (1); //OK LED - used for delay in bang_clock_data(bit)
    return rdat;			
}

/* =============================================
   Bit-Bang serial command
   1-byte RFFE read extended
   DATA = Pin E3
   CLK  = Pin E2
 ===============================================*/
uint32_t bit_bang_rffe_read_1byte_ext(uint16_t usid, uint16_t addr)
{
  uint32_t rdat=0, bit;
  int x;	  
  uint16_t  cmd_frame, cmd_par, addr_par; 
	uint32_t w;
	
	
	Pin_E_On(15); //Clock
	Pin_E_On(10); //Data  
	Pin_E_Off(4);
	
	 cmd_frame = (usid << 8) + (0x2 <<4);  //0x2 is read command     
   cmd_par = calc_parity(cmd_frame, 12); //x500
   addr_par = calc_parity(addr, 8);    
  
	 w = (cmd_frame <<11) + (cmd_par <<10) + (addr <<2) + (addr_par<<1);
	
	 //SSC
	 bang_SSC_up();
	 bang_SSC_up();
   bang_SSC_down();
	 
	//now bit banging
	for (x=22; x>-1; x--)
			{
				 bit = (w >> x) & 1;
			   bang_clock_data(bit);
			}

		bus_park_cycle();	
			
			rdat = 0;
		//Now the read cycles
   for (x=10; x>0; x--)
			{
			   bit = bang_clock_read_data();
				 rdat += bit << (x-1);
			} 

			GPIOE->BSRRH = led_mask[10]; //HIZ DATA  
			//bus_park_cycle();	
			//bang_clock_read_data(); //dummy bus park
			
	   Pin_E_Off(15); //Clock
	   Pin_E_Off(10); //Data  
		 rdat = rdat >> 2; //Making it compatible with old B style Firmware.
     Pin_B_On (1);  //OK LED - used for delay in bang_clock_data(bit)			
    return rdat;			
}
	
/* =============================================
   Bit-Bang serial command
   1-byte RFFE read standard
   DATA = Pin E3
   CLK  = Pin E2
 ===============================================*/
uint32_t bit_bang_rffe_read_1byte_std(uint16_t usid, uint16_t addr)
{
  uint32_t rdat, bit;
  int x;	  
  uint16_t  cmd_frame, cmd_par; 
	uint32_t w;
	
	Pin_E_On(15); //Clock
	Pin_E_On(10); //Data  
	Pin_E_Off(4);
	
	 cmd_frame = (usid << 8) + (0x3<<5) + addr;  //0x3 is read command     
   cmd_par = calc_parity(cmd_frame, 12); //x500
      
	 w = (cmd_frame <<1) + cmd_par;
	
	 //SSC
	 bang_SSC_up();
   bang_SSC_down();
	 
	//now bit banging
	for (x=13; x>-1; x--)
			{
				 bit = (w >> x) & 1;
			   bang_clock_data(bit);
			}

		bus_park_cycle();	
			
		//Now the read cycles
   for (x=10; x>1; x--)
			{
			   bit = bang_clock_read_data();
				 rdat += bit << x;
			}
			
			bus_park_cycle();	
			//bang_clock_read_data(); //dummy bus park
			
	   Pin_E_Off(15); //Clock
	   Pin_E_Off(10); //Data  

		rdat = rdat >> 3; //Making it compatible with old B style Firmware.	
    return rdat;			
}

void synch_A (void)
{
	Pin_E_On(11); 
	Pin_E_Off(11);
}
void synch_B (void)
{
	Pin_D_On(12); 
	Pin_D_Off(12);
}

//This sets the ILD buffers to output direction
//using D2 
//1-12-17 (vcore2 ild sync pins control
void ild_sync_out (int set)
{
   if (set)
	 {
			Pin_D_On(2); 
	 }
	 else
	 {
			Pin_D_Off(2); 
	 }
}

/***** GPIO COMMAND for VIO etc****/
void set_gpio_port_d(uint16_t val)
{
    GPIOD->ODR = val;
}
void set_gpio_port_e(uint16_t val)
{
    GPIOE->ODR = val;
}


/* interrupt handler D0 */
void EXTI0_IRQHandler(void)
{
	//interrupt handlers 
  //HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
	//HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	//Pin_B_On (11);
	//HAL_UART_Transmit (&huart3, "\n>>>Interrupt..0 \n\r", 22, 1000);
}

/* interrupt handler D15 */
void EXTI15_10_IRQHandler(void)
{
  //HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	//HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
	
	//HAL_UART_Transmit (&huart3, "\n>>>Interrupt..15 \n\r", 22, 1000);
	//bottom_half_process();
}	

//this is the extended process after interrupt occurred
//to prevent over loading the interrupt controller
void bottom_half_process(void)
{

		switch (_config.vcore2_dynamic_mode)
		{
			case VCORE_DYN_SYNC_LOOP:
		    		sync_loop (_config._tr_gap_delay);
					  
			      break;
			
			case VCORE_DYN_DVS:
						vcore_dynamic_by_interrupt();
			      //HAL_UART_Transmit (&huart3, "\n>>>Debug DYN_DVS \n\r", 22, 1000);
					break;
			
			default:
				  //HAL_UART_Transmit (&huart3, "\n>>>No dynamic mode set! error.. \n\r", 22, 1000);
			    break;
		}
	
}



void vcore_dynamic_by_interrupt(void)
{
	//I made a top switch just in case new mode comes up
	//Simply add more modes when required.
	switch(_config.vcore2_dynamic_mode)
	{
		case VCORE_DYN_DVS:
					//  ---0--|---1--|---2--|---3--|---4--|---5--|---6--|---7--|
					//  ------|------|------|------|------|------|------|------|
 		      //        T      w      T      w      T      w      T      w          
		      //T = Triggered
		      //w = wait
			  		
				switch (_config.vcore2_dyn_slot) {
					case 0:				
									_config.vcore2_dyn_slot = 1;					  
									write_rffe_1byte_std(4, 0xa, 0x13);   //mipi write command, a 13
																
									TM_DelayMicros(_config.vcore2_dyn_delay+3); //wait 
																	
								 _config.vcore2_dyn_slot = 2;//Go to slot 2, a 14
									write_rffe_1byte_std(4, 0xa, 0x23);   //mipi write command	
//HAL_UART_Transmit (&huart3, "\n>>>Slot 2 \n\r", 22, 1000);				  
									break;
				  case 2:				
									_config.vcore2_dyn_slot = 3;					  
									write_rffe_1byte_std(4, 0xa, 0x33);   //mipi write command, a 33
																	
									TM_DelayMicros(_config.vcore2_dyn_delay+3);//wait 
																	
								 _config.vcore2_dyn_slot = 4;//Go to slot 2, a 14
									write_rffe_1byte_std(4, 0xa, 0x43);   //mipi write command	
//HAL_UART_Transmit (&huart3, "\n>>>Slot 4 \n\r", 22, 1000);					  
									break;
					case 4:				
									_config.vcore2_dyn_slot = 5;					  
									write_rffe_1byte_std(4, 0xa, 0x53);   //mipi write command, a 53
																
									TM_DelayMicros(_config.vcore2_dyn_delay+3); //wait 
																	
								 _config.vcore2_dyn_slot = 6;//Go to slot 6, a 64
									write_rffe_1byte_std(4, 0xa, 0x63);   //mipi write command	
//HAL_UART_Transmit (&huart3, "\n>>>Slot 6 \n\r", 22, 1000);					  
									break;
				  case 6:				
									_config.vcore2_dyn_slot = 7;					  
									write_rffe_1byte_std(4, 0xa, 0x73);   //mipi write command, a 73
																	
									TM_DelayMicros(_config.vcore2_dyn_delay+3);//wait 
																	
								 _config.vcore2_dyn_slot = 0;//Go back to slot 0
									write_rffe_1byte_std(4, 0xa, 0x03);   //mipi write command
//HAL_UART_Transmit (&huart3, "\n>>>Slot 0 \n\r", 22, 1000);						
									break;
					default: //this should not happen.
				          _config.vcore2_dyn_slot = 0;//Go back to slot 0
									write_rffe_1byte_std(4, 0xa, 0x03);   //mipi write command		
									break;				}
				
				    
		break;
		
		default: 
		//HAL_GPIO_TogglePin( GPIOD, led_mask[4]);	
		break;
	}
}


/*----------------------------------------------------------------------------
  initialize Port E Pins
 *----------------------------------------------------------------------------*/
void PortE_init (void) 
{
	GPIOE->MODER    &= ~(
											(3UL << 2*2) |
											(3UL << 2*3) |
											(3UL << 2*4) |
											(3UL << 2*5) |
											(3UL << 2*7) |
											(3UL << 2*10) |
											(3UL << 2*11) |                     
                      (3UL << 2*15)  );   /* PD.12..15 is output               */
  GPIOE->MODER    |=  (
		
											 (1UL << 2*2) |
											 (1UL << 2*3) |
											 (1UL << 2*4) |
											 (1UL << 2*5) |
											 (1UL << 2*7) |
											 (1UL << 2*10) |
											 (1UL << 2*11) |
                       (1UL << 2*15)  ); 
  GPIOE->OTYPER   &= ~(
											(1UL <<   2) |
											(1UL <<   3) |
											(1UL <<   4) |
											(1UL <<   5) |
											(1UL <<   7) |
											(1UL <<   10) |
											(1UL <<   11) |
                      (1UL <<   15)  );   /* PD.12..15 is output Push-Pull     */
  GPIOE->OSPEEDR  &= ~(

											(3UL << 2*2) |
											(3UL << 2*3) |
											(3UL << 2*4) |
											(3UL << 2*5) |
											(3UL << 2*7) |
											(3UL << 2*10) |
											(3UL << 2*11) |
                       (3UL << 2*15)  );   /* PD.12..15 is 50MHz Fast Speed     */
  GPIOE->OSPEEDR  |=  (
											(2UL << 2*2) |
											(2UL << 2*3) |
											(2UL << 2*4) |
											(2UL << 2*5) |
											(2UL << 2*7) |	
											(2UL << 2*10) |
											(2UL << 2*11) | 
                       (2UL << 2*15)  ); 
  GPIOE->PUPDR    &= ~(

											(3UL << 2*2) |
											(3UL << 2*3) |
											(3UL << 2*4) |
											(3UL << 2*5) |
											(3UL << 2*7) |											
											(3UL << 2*10) |
											(3UL << 2*11) |
                       (3UL << 2*15)  );   /* PD.12..15 is Pull up              */
  GPIOE->PUPDR    |=  (

											(1UL << 2*2) |
											(1UL << 2*3) |
											(1UL << 2*4) |
											(1UL << 2*5) |
											(1UL << 2*6) |
											(1UL << 2*7) |
											(1UL << 2*10) |
											(1UL << 2*11) | 
                       (1UL << 2*15)  ); 
}
/*----------------------------------------------------------------------------
  initialize Port D Pins
 *----------------------------------------------------------------------------*/
void PortD_init (void) 
{

  //RCC->AHB1ENR  |= ((1UL <<  3) );         /* Enable GPIOD clock                */

  GPIOD->MODER    &= ~(
											/*(3UL << 2*0) |*/
											(3UL << 2*1) |
											(3UL << 2*2) |
											(3UL << 2*3) |
											(3UL << 2*4) |
											(3UL << 2*5) |
											(3UL << 2*6) |
											(3UL << 2*7) |
											(3UL << 2*8) |
											(3UL << 2*9) |
											(3UL << 2*10) |
											(3UL << 2*11) |
                      (3UL << 2*13) |
                      (3UL << 2*14) 
                      /*(3UL << 2*15)  */
	                     );   /* PD.12..15 is output               */
  GPIOD->MODER    |=  (
										   /*(1UL << 2*0) |*/
										 	 (1UL << 2*1) |
											 (1UL << 2*2) |
											 (1UL << 2*3) |
											 (1UL << 2*4) |
											 (1UL << 2*5) |
											 (1UL << 2*6) |
											 (1UL << 2*7) |
											 (1UL << 2*8) |
											 (1UL << 2*9) |
											 (1UL << 2*10) |
											 (1UL << 2*11) |
											 (1UL << 2*12) |
                       (1UL << 2*13) | 
                       (1UL << 2*14) 
                      /* (1UL << 2*15)*/ 
											 ); 
  GPIOD->OTYPER   &= ~(
											/*(1UL <<   0) |*/
											(1UL <<   1) |
											(1UL <<   2) |
											(1UL <<   3) |
											(1UL <<   4) |
											(1UL <<   5) |
											(1UL <<   6) |
											(1UL <<   7) |
											(1UL <<   8) |
											(1UL <<   9) |
											(1UL <<   10) |
											(1UL <<   11) |
											(1UL <<   12) |
                      (1UL <<   13) |
                      (1UL <<   14) 
                      /*(1UL <<   15)*/  
											);   /* PD.12..15 is output Push-Pull     */
  GPIOD->OSPEEDR  &= ~(
											(3UL << 2*0) |
											(3UL << 2*1) |
											(3UL << 2*2) |
											(3UL << 2*3) |
											(3UL << 2*4) |
											(3UL << 2*5) |
											(3UL << 2*6) |
											(3UL << 2*7) |
											(3UL << 2*8) |
											(3UL << 2*9) |
											(3UL << 2*10) |
											(3UL << 2*11) |
											(3UL << 2*12) |
                       (3UL << 2*13) |
                       (3UL << 2*14) |
                       (3UL << 2*15)  );   /* PD.12..15 is 50MHz Fast Speed     */
  GPIOD->OSPEEDR  |=  (
											(2UL << 2*0) |
											(2UL << 2*1) |
											(2UL << 2*2) |
											(2UL << 2*3) |
											(2UL << 2*4) |
											(2UL << 2*5) |
											(2UL << 2*6) |
											(2UL << 2*7) |
											(2UL << 2*8) |
											(2UL << 2*9) |
											(2UL << 2*10) |
											(2UL << 2*11) |
											(2UL << 2*12) |
                       (2UL << 2*13) | 
                       (2UL << 2*14) | 
                       (2UL << 2*15)  ); 
  GPIOD->PUPDR    &= ~(
											/*(3UL << 2*0) |*/
											(3UL << 2*1) |
											(3UL << 2*2) |
											(3UL << 2*3) |
											(3UL << 2*4) |
											(3UL << 2*5) |
											(3UL << 2*6) |
											(3UL << 2*7) |
											(3UL << 2*8) |
											(3UL << 2*9) |
											(3UL << 2*10) |
											(3UL << 2*11) |
											(3UL << 2*12) |
                       (3UL << 2*13) |
                       (3UL << 2*14) 
                       /*(3UL << 2*15)*/  
											 );   /* PD.12..15 is Pull up              */
  GPIOD->PUPDR    |=  (
											/*(1UL << 2*0) |*/
											(1UL << 2*1) |
											(1UL << 2*2) |
											(1UL << 2*3) |
											(1UL << 2*4) |
											(1UL << 2*5) |
											(1UL << 2*6) |
											(1UL << 2*7) |
											(1UL << 2*8) |
											(1UL << 2*9) |
											(1UL << 2*10) |
											(1UL << 2*11) |
											(1UL << 2*12) |
                       (1UL << 2*13) | 
                       (1UL << 2*14) 
                       /*(1UL << 2*15)*/  
											 ); 
}

//Pins B1, B11 are LED Pins
void PortB_init (void) 
{

  //RCC->AHB1ENR  |= ((1UL <<  3) );         /* Enable GPIOD clock                */

  GPIOB->MODER    &= ~(
											(3UL << 2*1) |
	                    (3UL << 2*5) |
											(3UL << 2*6) |
											(3UL << 2*7) |
											(3UL << 2*11) );
	GPIOB->MODER    |= (
											(1UL << 2*1) |
	(1UL << 2*5) |
	(1UL << 2*6) |
	(1UL << 2*7) |
											(1UL << 2*11) );
	GPIOB->OTYPER   &= ~(
											(1UL <<   1) |
	(1UL <<   5) |
	(1UL <<   6) |
	(1UL <<   7) |
											(1UL <<   11) );
	
  GPIOB->OSPEEDR  &= ~(
											(3UL << 2*1) |
											(3UL << 2*5) |
											(3UL << 2*6) |
											(3UL << 2*7) |
											(3UL << 2*11) );
	GPIOB->OSPEEDR  |=  (
											(2UL << 2*1) |
											(2UL << 2*5) |
											(2UL << 2*6) |
											(2UL << 2*7) |
											(2UL << 2*11) );
	
	GPIOB->PUPDR    &= ~(
											(3UL << 2*1) |
											(3UL << 2*5) |
											(3UL << 2*6) |
											(3UL << 2*7) |
											(3UL << 2*11) );
											
	GPIOB->PUPDR    |=  (
											(1UL << 2*1) |
											(1UL << 2*5) |
											(1UL << 2*6) |
											(1UL << 2*7) |
											(1UL << 2*11) );										
}

//Pins C0, C1 will be used as CFG PINS on VCORE2
void PortC_init (void) 
{

  //RCC->AHB1ENR  |= ((1UL <<  3) );         /* Enable GPIOD clock                */

  GPIOC->MODER    &= ~(
											(3UL << 2*0) |
											(3UL << 2*1) |
	                    (3UL << 2*2) );
	GPIOC->MODER    |= (
											(1UL << 2*0) |
											(1UL << 2*1) |
                      (1UL << 2*2)	);
	GPIOC->OTYPER   &= ~(
											(1UL <<   0) |
											(1UL <<   1) |
	                    (1UL <<   2));
	
  GPIOC->OSPEEDR  &= ~(
											(3UL << 2*0) |
											(3UL << 2*1) |
	                    (3UL << 2*2) );
	GPIOC->OSPEEDR  |=  (
											(2UL << 2*0) |
											(2UL << 2*1) |
											(2UL << 2*2));
	
	GPIOC->PUPDR    &= ~(
											(3UL << 2*0) |
											(3UL << 2*1) |
											(3UL << 2*2));
											
	GPIOC->PUPDR    |=  (
											(1UL << 2*0) |
											(1UL << 2*1) |
                      (1UL << 2*2));										
}
//End

//decode_command()
#include <stdio.h>
#include <string.h>
#include "STM32F4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "gecho_spi_app.h"
#include "gecho_gpio_app.h"
#include "gecho_i2c_app.h"
#include "gecho_structures.h"

#define MUSTANG 5
#define SUPERBIRD 6
#define ONEBYTE 7
#define SUPERBIRD3 8

////////////////////////
#define BOSS 21
#define BOSS_PA 93
/////////////////////////

////////////////////////
#define DELOREAN1 31
/////////////////////////
#define VCORE2	33
#define SPMI	34
/////////////////////////


/////////////////////////
#define CUDA_20SLOT_DEMO 220
#define CUDA_40SLOT_DEMO 240
/////////////////////////
/////////////////////////
#define CUDA_20SLOT_GRP1 220
#define CUDA_20SLOT_GRP2 240
/////////////////////////

#define SB4_GTO 88
#define CUDA_GTO 108
#define CUDA_TR_GTO_TR 208
#define GTO_TR_CUDA_TR 209


#define M5_PA 89
#define SB4_PA 90
#define CUDA_PA 91
#define GTO_CUDA_PA 92
#define BOSS_PA 93
#define ONEBYTE_M 9
#define ONEBYTE_S 10
#define ONEBYTE_C 11
#define ONEBYTE_C_TRIGGER 12

#define DVS1 7
#define DVS2 8
#define DVS3 9

extern UART_HandleTypeDef huart3;
uint32_t AtoH(char cmd[40], uint32_t n, uint32_t);

//main decode options
void gpio_write_port(char pcmd[100]);
void sub_Decode(char acmd[100]);
void update_dynamic_array(char dcmd[100]);
void update_dynamic_R2_plus_PA(char dcmd[100]);
void update_dynamic_array_superbird(char dcmd[100]);
void update_dynamic_array_mustang(char dcmd[100]);
void update_dynamic_array_mustang(char dcmd[100]);
void update_dynamic_C0(char dcmd[40]); /////NEW UNIVERSAL VERSION - USE THIS FROM NOW ON 6/26/2015

void set_dynamic(char dcmd[100]);
void echo_slot_array(void);

//RFFE Commands
void transfer_write_cmd(char icmd[100]);
void transfer_read_cmd(char icmd[100]);

//I2C commands
void i2c_write_cmd(char icmd[100]); //initial debug
void i2c_write_read(char icmd[100]);
void R2_I2C1_check_device_cmd(uint32_t address);

uint32_t bit_bang_i2c_write(uint16_t dev, uint16_t addr, uint16_t dat);

//Dynamic modes
void transmit_slot_config(void); 
//void transmit_2x_frame(void);
void transmit_frame1(void);
void transmit_frame2(void);

extern void TM_DelayMicros(uint32_t micros);
extern void _delay(uint32_t );

void decode_command(char cmd[100])
{
		switch(cmd[0]) {
		case 'a':			   
         sub_Decode(cmd);//use this option to expand the command set
		     break;

		case 'i':
			   //i2c_write_cmd(cmd);
		     i2c_write_read(cmd); //Both read and write
		     break;
		
		case 'W':
		case 'w':
			       transfer_write_cmd(cmd); 
		         break;
		case 'r':
    case 'R':
             transfer_read_cmd(cmd);
             break;
	
		case 'p':
    case 'P':  //PD FFFF
             gpio_write_port(cmd);
             break;
	  case 'd':
    case 'D': 
          set_dynamic(cmd);
          break; 
		
		case 'o':      //Used to update slot config array ->onebyte 
          //what_chip is determined inside update..(), os=SB, om=M5
          update_dynamic_array(cmd); 
          break; 
          
    case 'x':    //not used
          break;
    
		case 'v':
		case 'V':
    case 'u':     
    case 'U':      //Used in the old SB1
			             //6/26/15 U becomes the new Update command
		               //1. Cuda 40slot
		               //2. DeLorean MIPI dynamic eg..UDII DD TTT
		      update_dynamic_C0(cmd);
          break;
		 
		 case 's':
     case 'S':  
          set_spi_speed(cmd);
          break;
		 case 'e':
			   echo_slot_array();
		     break;
		 case '?':
		      HAL_UART_Transmit (&huart3, "\n>>>>>R2 USB-RFFE-SPMI-I2C-SPI D5 \n\r", 32, 1000);
		      break;
		default:
			HAL_UART_Transmit (&huart3, ">\n\r", 3, 1000);
		break;
		}
		cmd[0] = '\0';		
}

/********** I2C Transfer ************/
void i2c_write_cmd(char icmd[100])
{
     R2_I2C1_Transmit_test(0x3e);
}

void i2c_write_read(char icmd[100])
{
   uint32_t add, dat=0, dat2=0;  
   uint8_t rdat=0;
	 char tstr[100];
	
	 add =  AtoH(icmd, 3, 2) << 1;
	 
	 switch(icmd[1]) 
		{
		 case 'w':   //write to device>>>>>> iw 28aadd
			    dat = AtoH(icmd, 5, 2); //internal r2 address
		      dat2 = AtoH(icmd, 7, 2); //actual r2 data
		      R2_I2C1_write_1byte_cmd(add, dat, dat2, 2);
		      //R2_I2C1_Transmit_test(add);
		 
		      /////bit_bang_i2c_write(add, 45,  dat);
					break;
		 
		 case 'r':  //read from device>>>>>> ir 28aa
			    dat = AtoH(icmd, 5, 2); //internal r2 register address
		      //R2_I2C1_write_1byte_cmd(add, dat, dat2, 1);
		      rdat = R2_I2C1_read_1byte_cmd(add, dat);
		 
		       sprintf(tstr, ">>>%.2x = %.3x \n\r", (add >>1), rdat);
           HAL_UART_Transmit (&huart3, (uint8_t *) tstr, strlen(tstr), 1000);
					break;
		 
		 case 'c':  //check if device is ready
			     R2_I2C1_check_device_cmd(add);
		      break;
		 
		 default: break;
		}
		
	 //add = 
}


/********** RFFE TRANSFER ************/
void transfer_write_cmd(char icmd[100])
{
  uint32_t _usid=0x0, _reg_add=0x0;
  uint32_t dat, dat2, gap_delay, _dat[10];
 
  switch(icmd[1]) {
    case '1':  //1-byte write command w1 5 aa dd
           _usid = AtoH(icmd, 3, 1);
           _reg_add = AtoH(icmd, 5, 2);
           dat = AtoH(icmd, 8, 2);           
           	
		       //write_rffe_1byte_ext(_usid, _reg_add, dat); //Now a SPMI only (2 extra clocks)
					 	
           if (_reg_add < 0x20)   write_rffe_1byte_std(_usid, _reg_add, dat);   
           else  write_rffe_1byte_ext(_usid, _reg_add, dat);
    break;
		
		
				//VCORE DMC feature (D0.6) - 2 (1byte consecutive with controlled delay (gap time)
		//wz 5 aa dd dd del (first dd is byte1, 2nd dd is byte 2, del is delay)
		case 'z':
			     _usid = AtoH(icmd, 3, 1);
           _reg_add = AtoH(icmd, 5, 2);
		       dat = AtoH(icmd, 8, 2);  
		       dat2 = AtoH(icmd, 11, 2); 
           gap_delay = AtoH(icmd, 14, 3); 
		
           if (_reg_add < 0x20)   
						{
							write_rffe_1byte_std(_usid, _reg_add, dat);  
              TM_DelayMicros( gap_delay * 1600) ;							
							write_rffe_1byte_std(_usid, _reg_add, dat2); 
							
							TM_DelayMicros(50) ;							
							write_rffe_1byte_std(_usid, _reg_add, 0x22); //Go to HFS 3rd slot
						}
           else  
					 {
						 write_rffe_1byte_ext(_usid, _reg_add, dat);
             TM_DelayMicros( gap_delay * 1600) ;
             write_rffe_1byte_ext(_usid, _reg_add, dat2);		

             TM_DelayMicros(50) ;		
             write_rffe_1byte_ext(_usid, _reg_add, 0x22);		//Go to HFS						 
					 }		     
		break;
					 
		case 'L':
		case 'l':  //1-byte write command (extended long) wL 5 aaaa dd	 
		       _usid = AtoH(icmd, 3, 1);
           _reg_add = AtoH(icmd, 5, 4);
           dat = AtoH(icmd, 10, 2);           
           	
           write_rffe_1byte_ext_long(_usid, _reg_add, dat);
    break;
		
	 case 'b': //bit bang write
			     _usid = AtoH(icmd, 3, 1);
           _reg_add = AtoH(icmd, 5, 2);
           dat = AtoH(icmd, 8, 2);   
			     BIT_BANG_write_rffe_1byte_xtd(_usid, _reg_add, dat);
		break;
   
   case '2':  //2-byte write command w2 5 aa dddd
           _usid = AtoH(icmd, 3, 1);
           _reg_add = AtoH(icmd, 5, 2);
           _dat[0] = AtoH(icmd, 8, 2);
           _dat[1] = AtoH(icmd, 10, 2);           
           write_rffe_2byte_ext(_usid, _reg_add, _dat);	        
   break;
   
	 case '3':
		       _usid = AtoH(icmd, 3, 1);
           _reg_add = AtoH(icmd, 5, 2);
           _dat[0] = AtoH(icmd, 8, 2);
           _dat[1] = AtoH(icmd, 10, 2);           
           _dat[2] = AtoH(icmd, 12, 2);            
           write_rffe_3byte_ext(_usid, _reg_add, _dat);	        
   break;
	 
   case '4':  //4-byte write command 
           _usid = AtoH(icmd, 3, 1);
           _reg_add = AtoH(icmd, 5, 2);
           _dat[0] = AtoH(icmd, 8, 2);
           _dat[1] = AtoH(icmd, 10, 2);           
           _dat[2] = AtoH(icmd, 12, 2);           
           _dat[3] = AtoH(icmd, 14, 2);    
           write_rffe_4byte_ext(_usid, _reg_add, _dat);	        
   break;
  
   case '6':  //6-byte write command
           _usid = AtoH(icmd, 3, 1);
           _reg_add = AtoH(icmd, 5, 2);
           _dat[0] = AtoH(icmd, 8, 2);
           _dat[1] = AtoH(icmd, 10, 2);           
           _dat[2] = AtoH(icmd, 12, 2);           
           _dat[3] = AtoH(icmd, 14, 2);           
           _dat[4] = AtoH(icmd, 16, 2);           
           _dat[5] = AtoH(icmd, 18, 2);           
           // write_rffe_6_byte(_usid, _reg_add, _dat[0], _dat[1], _dat[2], _dat[3], _dat[4], _dat[5]);   
   break;
   
   case 'q':  //q cmds, see mipi_rffe_qual
            //decode_q_write_cmd(icmd);   
   break;
   
   case 'f': //16-byte write command
           _usid = AtoH(icmd, 3, 1);
           _reg_add = AtoH(icmd, 5, 2);
           _dat[0] = AtoH(icmd, 8, 4);
           _dat[1] = AtoH(icmd, 12, 4);           
           _dat[2] = AtoH(icmd, 16, 4);           
           _dat[3] = AtoH(icmd, 20, 4);           
           _dat[4] = AtoH(icmd, 24, 4);           
           _dat[5] = AtoH(icmd, 28, 4); 
           _dat[6] = AtoH(icmd, 32, 4);
           _dat[7] = AtoH(icmd, 36, 2) << 8;
           //write_rffe_16_byte(_usid, _reg_add, _dat[0], _dat[2], _dat[2], _dat[3], _dat[4], _dat[5], _dat[6], _dat[7]);   
   break;        
        
  default: 
    break; 
  }
}

/********* READ COMMAND ********/
void transfer_read_cmd(char icmd[100])
{
  uint32_t _usid=0x0, _reg_add=0x0;
  uint32_t dat;
  char tstr[100];
  
  switch(icmd[1]) {
    case '1':  //1-byte read command r1 5 aa
           _usid = AtoH(icmd, 3, 1);
           _reg_add = AtoH(icmd, 5, 2);
           
           dat = bit_bang_rffe_read_1byte_ext(_usid, _reg_add);		
           sprintf(tstr, ">>>%.2x = %.3x \n\r", _reg_add, dat);
           HAL_UART_Transmit (&huart3, (uint8_t *) tstr, strlen(tstr), 1000);
    break;
  
		case 'e':  //extended long read command -- re 5 aaaa 
			     _usid = AtoH(icmd, 3, 1);
           _reg_add = AtoH(icmd, 5, 4);
		       dat = 0; //debug only
		
		       dat = bit_bang_rffe_read_1byte_ext_long(_usid, _reg_add);
			     sprintf(tstr, ">>>%.4x = %.3x \n\r", _reg_add, dat);
           HAL_UART_Transmit (&huart3, (uint8_t *) tstr, strlen(tstr), 1000);
		break;
		
  default: break;
  }
}

	
//+++++++++++++++++++++++++++++++++++++++
//  Turn dynamic on / off
//+++++++++++++++++++++++++++++++++++++++
void set_dynamic(char dcmd[100])
{
   switch(dcmd[1]) {
    case '0': 
      _config.dynamic=0; 
		  _config.vcore2_dynamic = 0;
		   Pin_B_Off (11);
		   ild_sync_out (0);
      break;
    case '1': //d1 7   ---> 7 = DVS1, 8 = DVS2
      _config.dynamic=1;
		  _config.vcore2_dynamic = 1;
		  _config.vcore2_dynamic_mode = AtoH(dcmd, 3, 1);
		  Pin_B_On (11);
		  HAL_UART_Transmit (&huart3, "\n>>>dynamic mode.. \n\r", 22, 1000);
		  _config.vcore2_dyn_slot = 0;
		
		  ild_sync_out (1); //buffers for ild sync to output direction.
       break;
    default: 
			_config.dynamic=0; 
	    _config.vcore2_dynamic = 0;	
		break;
 }
}
//+++++++++++++++++++++++++++++++++++++++
//single byte dynamic slot
//not used when PA is included (Customer GUI)
//12/18/14 - this used to be update_dynamic_array_onebyte(cmd)
//+++++++++++++++++++++++++++++++++++++++
void update_dynamic_array(char dcmd[40])
{
  uint32_t  ndx;
  uint32_t b0, b1, p0, p1, d0;

  //if m address = 0x15
  //if s address = 0x17
	 
   switch(dcmd[1]) {
    case 'm': //M5
			    //om01 b0 ddd<<<adjust to this
					ndx = AtoH(dcmd, 2, 2); //ndx
					b0 = AtoH(dcmd, 5, 2); //data
					d0 = AtoH(dcmd, 8, 3); //delay
					_config.one_byte[ndx] = b0; 
					_config._slot_data_[ndx][6] = d0; //delay 
          _config.one_byte_addr = 0x15;
          _config.what_chip = ONEBYTE_M;
          break;
		
    case 's': //SB4
			    //om01 b0 ddd<<<adjust to this
					ndx = AtoH(dcmd, 2, 2); //ndx
					b0 = AtoH(dcmd, 5, 2); //data
					d0 = AtoH(dcmd, 8, 3); //delay
					_config.one_byte[ndx] = b0; 
					_config._slot_data_[ndx][6] = d0; //delay 
          _config.one_byte_addr = 0x17;
          _config.what_chip = ONEBYTE_S;
          break;      
		
    case 'c': //Cuda
			    //om01 b0 ddd<<<adjust to this
					ndx = AtoH(dcmd, 2, 2); //ndx
					b0 = AtoH(dcmd, 5, 2); //data
					d0 = AtoH(dcmd, 8, 3); //delay
					_config.one_byte[ndx] = b0; 
					_config._slot_data_[ndx][6] = d0; //delay 
          _config.one_byte_addr = 0x19;
          _config.what_chip = ONEBYTE_C;
          break;      
		
		case 'b': //Boss
			    //ob01 b0 b1 ddd<<<adjust to this
					ndx = AtoH(dcmd, 2, 2); //ndx
					b0 = AtoH(dcmd, 5, 2); //data1
		      b1 = AtoH(dcmd, 8, 2); //data2
					d0 = AtoH(dcmd, 11, 3); //delay
					_config.dyn_byte_1[ndx] = b0; 
		      _config.dyn_byte_2[ndx] = b1; 
		      _config._slot_data_[ndx][0]= b0; //_slot_data_0[ndx] = b0; 
          _config._slot_data_[ndx][1]= b1; //_slot_data_1[ndx] = b1;
					_config._slot_data_[ndx][6] = d0; //delay 
          _config.one_byte_addr = 0x11;
          _config.what_chip = BOSS;
          break;   

     case 'B': //Boss with PA
			    //ob01 b0 b1 p0 p1 ddd<<<adjust to this
					ndx = AtoH(dcmd, 2, 2); //ndx
					b0 = AtoH(dcmd, 5, 2); //data1
		      b1 = AtoH(dcmd, 8, 2); //data2
		      p0 = AtoH(dcmd, 11, 2); //pa data1
		      p1 = AtoH(dcmd, 14, 2); //pa data2
					d0 = AtoH(dcmd, 17, 3); //delay
					_config.dyn_byte_1[ndx] = b0; 
		      _config.dyn_byte_2[ndx] = b1; 
		      _config._slot_data_[ndx][0]= b0; //_slot_data_0[ndx] = b0; 
          _config._slot_data_[ndx][1]= b1; //_slot_data_1[ndx] = b1;
		      _config._slot_data_[ndx][4] = p0;
		      _config._slot_data_[ndx][5] = p1;
					_config._slot_data_[ndx][6] = d0; //delay 
          _config.one_byte_addr = 0x11;
          _config.what_chip = BOSS_PA;
          break;   		
   } 
  //sprintf(tstr, ">>>%.2x = %.2x %.3x\n\r", ndx, b0,  d0);
  //SerialPutString(tstr); 
}

//++++++++++++++++++++++++++++++++++++++++++++++++
//   6/26/2015
// New Universal version of update dynamic Version C0 (TIMER Version)
// U1 
//++++++++++++++++++++++++++++++++++++++++++++++++
void update_dynamic_C0(char dcmd[40])
{
	uint32_t b0,   pa0, pa1, d0, ndx;    
	
	 _config.what_chip = CUDA_40SLOT_DEMO;
    switch(dcmd[1]) {
    case '1': 
			      _config.what_chip = CUDA_20SLOT_DEMO;
		          ndx = AtoH(dcmd, 2, 2);
              b0 = AtoH(dcmd, 5, 2); //Single byte
              pa0 = AtoH(dcmd, 8, 2);
              pa1 = AtoH(dcmd, 11, 2);
              d0 = AtoH(dcmd, 14, 3);
              
              _config._slot_data_[ndx][0]= b0; //_slot_data_0[ndx] = b0; 
              _config._slot_data_[ndx][4]= pa0;//_slot_data_4[ndx] = pa0;  //pa slot
              _config._slot_data_[ndx][5]= pa1;//_slot_data_5[ndx] = pa1;  //pa slot
              _config._slot_data_[ndx][6]= d0; //_slot_data_6[ndx] = d0; //delay  
		        break;
		
		case '2':////U2ss dd pp pp ddd
			      _config.what_chip = CUDA_40SLOT_DEMO;
		         ndx = AtoH(dcmd, 2, 2);
              b0 = AtoH(dcmd, 5, 2); //Single byte
              pa0 = AtoH(dcmd, 8, 2);
              pa1 = AtoH(dcmd, 11, 2);
              d0 = AtoH(dcmd, 14, 3);
              
              _config._slot_data_[ndx][0]= b0; //_slot_data_0[ndx] = b0; 
              _config._slot_data_[ndx][4]= pa0;//_slot_data_4[ndx] = pa0;  //pa slot
              _config._slot_data_[ndx][5]= pa1;//_slot_data_5[ndx] = pa1;  //pa slot
              _config._slot_data_[ndx][6]= d0; //_slot_data_6[ndx] = d0; //delay   
		        break;
		
		case 'd':
		case 'D':  //DeLorean >UDII DD TTT   (TTT is the delay value)
			      _config.what_chip = DELOREAN1;
		         ndx = AtoH(dcmd, 2, 2);
              b0 = AtoH(dcmd, 5, 2); //Single byte
              d0 = AtoH(dcmd, 8, 3);
              
              _config._slot_data_[ndx][0]= b0; //_slot_data_0[ndx] = b0; 
              _config._slot_data_[ndx][6]= d0; //_slot_data_6[ndx] = d0; //delay   
		        break;
		
		case 'v':
		case 'V':  //VCORE2 (dac value for 0-3 slots) cmd==>UV dd dd dd dd DDD DDD
               //need to add delay
		        _config.dvs_vout[0] = AtoH(dcmd, 3, 2);
		        _config.dvs_vout[1] = AtoH(dcmd, 6, 2);
		        _config.dvs_vout[2] = AtoH(dcmd, 9, 2);
		        _config.dvs_vout[3] = AtoH(dcmd, 12, 2);
		
            _config.vcore2_dyn_delay = AtoH(dcmd, 15, 3);
						_config.vcore2_dyn_delay2 = AtoH(dcmd, 19, 3);
		        break;		
		
		
			
		default: break;
	}
}	

////HAL_GPIO_TogglePin( GPIOD, led_mask[4]);
//HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
//VCORE2 DYNAMIC FEATURES
void transmit_vcore2_dvs1(void)
{
   //1. V1, I1
	 //2. V2, I2
   TM_DelayMicros(_config.vcore2_dyn_delay2+3);//wait
	 BIT_BANG_write_rffe_1byte_std_2(0x8, 0, _config.dvs_vout[0], 4, 0xa, 0x01);
	 TM_DelayMicros(_config.vcore2_dyn_delay2+3);//wait 
	 BIT_BANG_write_rffe_1byte_std_3(0x4, 0xa, 0x11, 0x8, 0, _config.dvs_vout[1]);
   //TM_DelayMicros(_config.vcore2_dyn_delay2+3);//wait 
	 //sync pin
	 synch_A ();
}

void transmit_vcore2_dvs2(void)
{
   //1. V1, I1
	 //2. V2, I2
	 //3. I3, V3
	 //4. V4, I4
	 //back to ----> 1. V1, I1
	 //Need to add programmable delays in the gaps
	
	 BIT_BANG_write_rffe_1byte_std(0x8, 0, _config.dvs_vout[0]);
	 set_ILD_cfg(0, 0, 0);
	 TM_DelayMicros(_config.vcore2_dyn_delay+3);//wait 
	
	 BIT_BANG_write_rffe_1byte_std(0x8, 0, _config.dvs_vout[1]);
	 set_ILD_cfg(0, 0, 1);
	 TM_DelayMicros(_config.vcore2_dyn_delay+3);//wait 

	 
	 //sync pin
	 synch_A ();
}

void transmit_vcore2_dvs3(void)
{
   //1. V1, I1
	 //2. V2, I2
	 //3. I3, V3
	 //4. V4, I4
	 //back to ----> 1. V1, I1
	 //Need to add programmable delays in the gaps
	
	 BIT_BANG_write_rffe_1byte_std(0x8, 0, _config.dvs_vout[0]);
	 set_ILD_cfg(0, 0, 0);
	 TM_DelayMicros(_config.vcore2_dyn_delay+3);//wait 
	
	 BIT_BANG_write_rffe_1byte_std(0x8, 0, _config.dvs_vout[1]);
	 set_ILD_cfg(0, 0, 1);
	 TM_DelayMicros(_config.vcore2_dyn_delay+3);//wait 
	
	 BIT_BANG_write_rffe_1byte_std(0x8, 0, _config.dvs_vout[2]);
	 set_ILD_cfg(0, 1, 0);
	 TM_DelayMicros(_config.vcore2_dyn_delay+3);//wait 
	
	 BIT_BANG_write_rffe_1byte_std(0x8, 0, _config.dvs_vout[2]);
	 set_ILD_cfg(0, 1, 1);
	 TM_DelayMicros(_config.vcore2_dyn_delay+3);//wait 
	 
	 BIT_BANG_write_rffe_1byte_std(0x8, 0, _config.dvs_vout[2]);
	 set_ILD_cfg(1, 0, 0);
	 TM_DelayMicros(_config.vcore2_dyn_delay+3);//wait 
	 
	 //sync pin
	 synch_A ();
}

//++++++++++++++++++++++++++++++++++++++++++++++++
//   Customer GUI Update, include PA control
//++++++++++++++++++++++++++++++++++++++++++++++++
void update_dynamic_R2_plus_PA(char dcmd[40])
{
    uint32_t b0, b1, b2, b3, pa0, pa1, d0, ndx;    
  
    switch(_config.what_chip) {
    case M5_PA:
    case SB4_PA:  //ii b0 b1 b2 b3 P0 P1 ff1              
             ndx = AtoH(dcmd, 1, 2);
              b0 = AtoH(dcmd, 4, 2);
              b1 = AtoH(dcmd, 7, 2);
              b2 = AtoH(dcmd, 10, 2);
              b3 = AtoH(dcmd, 13, 2);
              pa0 = AtoH(dcmd, 16, 2); //pa first byte
              pa1 = AtoH(dcmd, 19, 2); //pa 2nd byte
              d0 = AtoH(dcmd, 22, 3);
              
		          _config._slot_data_[ndx][0]= b0; //_slot_data_0[ndx] = b0; 
              _config._slot_data_[ndx][1]= b1; //_slot_data_1[ndx] = b1;
              _config._slot_data_[ndx][2]= b2; //_slot_data_2[ndx] = b2;
              _config._slot_data_[ndx][3]= b3; //_slot_data_3[ndx] = b3;
              _config._slot_data_[ndx][4]= pa0; //_slot_data_4[ndx] = pa0;  //pa slot
              _config._slot_data_[ndx][5]= pa1; //_slot_data_5[ndx] = pa1;  //pa slot
              _config._slot_data_[ndx][6]= d0;	//_slot_data_6[ndx] = d0; //delay  	

              _config._slot_trigger[ndx] = 1;
             // sprintf(tstr, ">>>%.2x = %.2x %.2x %.2x %.2x %.2x %.2x %.3x \n\r", ndx, b0, b1, b2, b3, pa0, pa1, d0);
             //SerialPutString(tstr); 
    break;
    
    case CUDA_PA:
             ndx = AtoH(dcmd, 1, 2);
              b0 = AtoH(dcmd, 4, 2); //Single byte
              pa0 = AtoH(dcmd, 7, 2);
              pa1 = AtoH(dcmd, 10, 2);
              d0 = AtoH(dcmd, 13, 3);
              
              _config._slot_data_[ndx][0]= b0; //_slot_data_0[ndx] = b0; 
              _config._slot_data_[ndx][4]= pa0;//_slot_data_4[ndx] = pa0;  //pa slot
              _config._slot_data_[ndx][5]= pa1;//_slot_data_5[ndx] = pa1;  //pa slot
              _config._slot_data_[ndx][6]= d0; //_slot_data_6[ndx] = d0; //delay  
              
              //sprintf(tstr, ">>>%.2x = %.2x %.2x %.2x %.3x \n\r", ndx, b0,  pa0, pa1, d0);
              //SerialPutString(tstr); 
         break;
    
    case GTO_CUDA_PA:     
             ndx = AtoH(dcmd, 1, 2);
              b0 = AtoH(dcmd, 4, 2); //Single byte
              pa0 = AtoH(dcmd, 7, 2); //gto
              pa1 = AtoH(dcmd, 10, 2);
              b1 = AtoH(dcmd, 13, 2);
              d0 = AtoH(dcmd, 16, 3);
              
              _config._slot_data_[ndx][0]= b0;//_slot_data_0[ndx] = b0; 
              _config._slot_data_[ndx][1]= b1;//_slot_data_1[ndx] = b1; 
              _config._slot_data_[ndx][4]= pa0;//_slot_data_4[ndx] = pa0;  //pa slot
              _config._slot_data_[ndx][5]= pa1;//_slot_data_5[ndx] = pa1;  //pa slot
              _config._slot_data_[ndx][6]= d0;//_slot_data_6[ndx] = d0; //delay  
              
              //sprintf(tstr, ">>>%.2x = %.2x %.2x %.2x %.2x %.3x \n\r", ndx, b0, b1, pa0, pa1, d0);
              //SerialPutString(tstr); 
            
            //gto_dac_now, gto_dac_prev            
            _config.gto_dac_now = (b1 & 0xff) >> 2;
            
            if ( _config.gto_dac_now > _config.gto_dac_prev) _config.gto_cuda_prior[ndx] = 1;
            else _config.gto_cuda_prior[ndx] = 0; 
            
            if (ndx == 0) _config.gto_cuda_prior[ndx] = 1;  //if 1, trigger gto first            
            _config.gto_dac_prev = _config.gto_dac_now;
           break;
           
    default: break;
    }
}
// ++++++++++++++++++++++++++++++++++++++++++++++++
// SUPERBIRD  
// ++++++++++++++++++++++++++++++++++++++++++++++++
void update_dynamic_array_superbird(char dcmd[40])
{
  uint32_t b0, b1, b2, b3, b4, b5, g0, d0, ndx;

  if (_config.what_chip == SUPERBIRD)
  {
    //u01 b0 b1 b2 b3 b4 b5 <<<adjust to this
    //u19 b0 b1 b2 b3 b4 b5 ff
    //012 45 78 01 34 67 90 
    //_slot_data must be a 2d array (_slot_data[20][6]), to contain 20 slots of 6 bytes
    ndx = AtoH(dcmd, 1, 2);
    b0 = AtoH(dcmd, 4, 2);
    b1 = AtoH(dcmd, 7, 2);
    b2 = AtoH(dcmd, 10, 2);
    b3 = AtoH(dcmd, 13, 2);
    b4 = AtoH(dcmd, 16, 2);
    b5 = AtoH(dcmd, 19, 2);
    d0 = AtoH(dcmd, 22, 3);

    _config._slot_data_[ndx][0]= b0; //_slot_data_0[ndx] = b0; 
    _config._slot_data_[ndx][1]= b1; //_slot_data_1[ndx] = b1;
    _config._slot_data_[ndx][2]= b2; //_slot_data_2[ndx] = b2;
    _config._slot_data_[ndx][3]= b3; //_slot_data_3[ndx] = b3;
    _config._slot_data_[ndx][4]= b4; //_slot_data_4[ndx] = b4;
    _config._slot_data_[ndx][5]= b5; //_slot_data_5[ndx] = b5;
    _config._slot_data_[ndx][6]= d0; //_slot_data_6[ndx] = d0; //delay 
    _config._slot_trigger[ndx] = 1; //work around, just fixed to one, it should be adjustable from GUI. fix me later
    
    //sprintf(tstr, ">>>%.2x = %.2x %.2x %.2x %.2x %.2x %.2x %.3x \n\r", ndx, b0, b1, b2, b3, b4, b5, d0);
    //SerialPutString(tstr); 
  }
  
  if (_config.what_chip == SUPERBIRD3)
  {
     //319 b0 b1 b2 b3 ff1 >slot 19, sb3
     ndx = AtoH(dcmd, 1, 2);
      b0 = AtoH(dcmd, 4, 2);
      b1 = AtoH(dcmd, 7, 2);
      b2 = AtoH(dcmd, 10, 2);
      b3 = AtoH(dcmd, 13, 2);
      d0 = AtoH(dcmd, 16, 3);
      
    _config._slot_data_[ndx][0]= b0; //_slot_data_0[ndx] = b0; 
    _config._slot_data_[ndx][1]= b1; //_slot_data_1[ndx] = b1;
    _config._slot_data_[ndx][2]= b2; //_slot_data_2[ndx] = b2;
    _config._slot_data_[ndx][3]= b3; //_slot_data_3[ndx] = b3;
    _config._slot_data_[ndx][6]= d0; //_slot_data_6[ndx] = d0; //delay  
    _config._slot_trigger[ndx] = 1;
     //sprintf(tstr, ">>>%.2x = %.2x %.2x %.2x %.2x %.3x \n\r", ndx, b0, b1, b2, b3, d0);
    //SerialPutString(tstr);
  }
  
  if ((_config.what_chip == SB4_GTO) || (_config.what_chip == CUDA_GTO))
  {
     //319 b0 b1 b2 b3 ff1 >slot 19, sb3
     ndx = AtoH(dcmd, 1, 2);
      b0 = AtoH(dcmd, 4, 2);
      b1 = AtoH(dcmd, 7, 2);
      b2 = AtoH(dcmd, 10, 2);
      b3 = AtoH(dcmd, 13, 2);
      g0 = AtoH(dcmd, 16, 2); //gto slot
      d0 = AtoH(dcmd, 19, 3);
      
     _config._slot_data_[ndx][0]= b0; //_slot_data_0[ndx] = b0; 
     _config._slot_data_[ndx][1]= b1; //_slot_data_1[ndx] = b1;
     _config._slot_data_[ndx][2]= b2; //_slot_data_2[ndx] = b2;
     _config._slot_data_[ndx][3]= b3; //_slot_data_3[ndx] = b3;
     _config._slot_data_[ndx][4]= g0; //_slot_data_4[ndx] = g0;  //gto slot
     _config._slot_data_[ndx][6]= d0; // _slot_data_6[ndx] = d0; //delay  
     _config._slot_trigger[ndx] = 1;
     //sprintf(tstr, ">>>%.2x = %.2x %.2x %.2x %.2x %.2x %.3x \n\r", ndx, b0, b1, b2, b3, g0, d0);
    //SerialPutString(tstr);
  }
}

void update_dynamic_array_mustang(char dcmd[40])
{
  uint32_t  ndx;
  uint32_t b0, b1, b2, b3, tr, d0;

  //u01 b0 b1 b2 b3 b4 b5 <<<adjust to this
  //u19 b0 b1 b2 b3 b4 b5 ff
  //012 45 78 01 34 67 90 
  //_slot_data must be a 2d array (_slot_data[20][6]), to contain 20 slots of 6 bytes 
		ndx = AtoH(dcmd, 1, 2);
		b0 = AtoH(dcmd, 4, 2);
		b1 = AtoH(dcmd, 7, 2);
		b2 = AtoH(dcmd, 10, 2);
		b3 = AtoH(dcmd, 13, 2);
		tr = AtoH(dcmd, 16, 2);
		d0 = AtoH(dcmd, 19, 3);

    _config._slot_data_[ndx][0]= b0; //_slot_data_0[ndx] = b0; 
    _config._slot_data_[ndx][1]= b1; //_slot_data_1[ndx] = b1;
    _config._slot_data_[ndx][2]= b2; //_slot_data_2[ndx] = b2;
    _config._slot_data_[ndx][3]= b3; //_slot_data_3[ndx] = b3;
    _config._slot_trigger[ndx] = tr;
    _config._slot_data_[ndx][6]= d0; //_slot_data_6[ndx] = d0; //delay 
  
  //sprintf(tstr, ">>>%.2x = %.2x %.2x %.2x %.2x %.2x %.3x, tr= \n\r", ndx, b0, b1, b2, b3, tr, d0);
  //SerialPutString(tstr); 
}

//Rev Cx
//fast transmit experiment for cuda
void transmit_really_fast(void)
{
	   write_rffe_1byte_std_hardcode_ready();
	   TM_DelayMicros(2 + _config._slot_delay_correction);
	  
	   write_rffe_1byte_std(0x5, 0x19, _config.one_byte[0]); 	   
	   TM_DelayMicros(2 + _config._slot_delay_correction_2);
}

//New Master dynamic loop
void dynamic_loop(void)
{
   switch(_config.what_chip){
		
		case CUDA_20SLOT_GRP1: transmit_frame1(); break; 
		case CUDA_20SLOT_GRP2: transmit_frame2(); break; 
		 
	  default: transmit_slot_config(); break;
	}

}

//Rev D0 (TIMER VERSION) BLINKER
//20 slot apt + 20 slot et (DEMO Dynamic - Cuda (Shanghai DEMO 6/25/2015)
void transmit_frame1(void)
{
	uint32_t s;
	uint32_t cdelay=0;
	
	//for (f = 0; f< _config.num_dyn_set; f++)
	//{
		  Pin_E_On(11);   //ARB TRIG 1
			Pin_E_Off(11);
		for (s=0; s<20; s++)
		{	
						write_rffe_1byte_std(0x5, 0x19, _config._slot_data_[s][0]); //Cuda single byte 
						write_rffe_1byte_std(_config.pa_usid, 0, _config._slot_data_[s][4]); //write to address 0 of PA
						write_rffe_1byte_std(_config.pa_usid, 1, _config._slot_data_[s][5]); //write to address 1 of PA 
						write_rffe_1byte_std(0x0, 0x1c, 1);
			
			      cdelay = (_config._slot_data_[s][6] ) * 6;
						TM_DelayMicros( cdelay) ; //0xbb8);
			
			      if (!_config.dynamic) break;
		}
	//}
	
	//ARB TRIG 2
  // Pin_D_On(12);
	// Pin_D_Off(12);
}

void transmit_frame2(void)
{
		
	uint32_t s;
	uint32_t cdelay=0;
	//for (f = 0; f< _config.num_dyn_set; f++)
	//{
		  Pin_E_On(11);  //ARB TRIG 1
			Pin_E_Off(11);
		
			for (s=20; s<40; s++)
			{	
							write_rffe_1byte_std(0x5, 0x19, _config._slot_data_[s][0]); //Cuda single byte 
							write_rffe_1byte_std(_config.pa_usid, 0, _config._slot_data_[s][4]); //write to address 0 of PA
							write_rffe_1byte_std(_config.pa_usid, 1, _config._slot_data_[s][5]); //write to address 1 of PA 
							write_rffe_1byte_std(0x0, 0x1c, 1);
				
				      cdelay = (_config._slot_data_[s][6] ) * 6;
						  TM_DelayMicros( cdelay) ; //0xbb8);
				
				      if (!_config.dynamic) break;
			}
  //}
	
	  //ARB TRIG 2
   //Pin_D_On(12);
	 //Pin_D_Off(12);
}



//>>>>>>>>>>>>>>>>>+++++++++++++++++++++++++++++++++++++++<<<<<<<<<<<<<<<<<<<<
//                 DYNAMIC SLOT - Call this in main loop
//>>>>>>>>>>>>>>>>>+++++++++++++++++++++++++++++++++++++++<<<<<<<<<<<<<<<<<<<<	
void transmit_slot_config(void)
{
  uint32_t slot;
  
   //ARB TRIG 1
   Pin_E_On(11);
	 Pin_E_Off(11);
	
  for (slot=0; slot<_config._num_slot; slot++)
  {   
    switch(_config.what_chip){
         
    case DELOREAN1:  //Added in C0.4 6/26/2015
          write_rffe_1byte_std(0x0, 0xA, _config._slot_data_[slot][0]); //BROADCAST COMMAND OF SLOT CONFIG
           break;
		
		case BOSS:
			   if (_config.bitbang_dynamic) 
				 {
				 //BIT_BANG_write_rffe_2byte_xtd(0x5, 0x11,  _config._slot_data_[slot]);
				 BIT_BANG_write_rffe_1byte_xtd(0x5, 0x11, _config._slot_data_[slot][0]);
				 BIT_BANG_write_rffe_1byte_xtd(0x5, 0x12, _config._slot_data_[slot][1]);	 
				 BIT_BANG_write_rffe_1byte_xtd(0x5, 0x1c, 1);	 
				 }
         else {				 
          write_rffe_2byte_ext(0x5, 0x11,  _config._slot_data_[slot]);
          write_rffe_1byte_std(0x5, 0x1c, 1);}
         break;
		
		case BOSS_PA:
			   write_rffe_2byte_ext(0x5, 0x11,  _config._slot_data_[slot]);
         write_rffe_1byte_std(_config.pa_usid, 0, _config._slot_data_[slot][4]); //write to address 0 of PA
         write_rffe_1byte_std(_config.pa_usid, 1, _config._slot_data_[slot][5]); //write to address 1 of PA  
		     write_rffe_1byte_std(0x0, 0x1c, 1); //broadcast trigger
		     break;
		
    case MUSTANG:
          write_rffe_4byte_ext(0x5, 0x11,  _config._slot_data_[slot]);
          write_rffe_1byte_std(0x5, 0x1c, _config._slot_trigger[slot]); 
          break;
             
    case SB4_GTO:
          write_rffe_4byte_ext(0x5, 0x14,  _config._slot_data_[slot]); 
          write_rffe_1byte_std(0x4, 0x18, _config._slot_data_[slot][4]); //gto 
          //Delay(_tr_gap_delay);
          write_rffe_1byte_std(0x0, 0x1c, _config._slot_trigger[slot]); //broadcast
          break;   
          
    case CUDA_GTO:
          write_rffe_4byte_ext(0x5, 0x16,  _config._slot_data_[slot]); 
          write_rffe_1byte_std(0x4, 0x18, _config._slot_data_[slot][4]); //gto 
          //Delay(_tr_gap_delay);
          write_rffe_1byte_std(_config.gsid, 0x1c, _config._slot_trigger[slot]); //broadcast
          break;
          
     //CUDA_TR_GTO_TR 208
     //GTO_TR_CUDA_TR 209
     case CUDA_TR_GTO_TR:          
          write_rffe_1byte_std(0x4, 0x18, _config._slot_data_[slot][4]);  //gto slot on (4)
          write_rffe_3byte_ext(0x5, 0x16, _config._slot_data_[slot]);
          write_rffe_1byte_std(_config.gsid, 0x19, _config._slot_data_[slot][3]); //cuda 19 on (9) (groupid)
		      write_rffe_1byte_std_x2(0x5, 0x1c, _config._slot_trigger[slot], 0x4, 0x1c, _config._slot_trigger[slot], _config._tr_gap_delay); //Trigger
          break;
          
    case GTO_TR_CUDA_TR:
          write_rffe_1byte_std(0x4, 0x18, _config._slot_data_[slot][4]); //gto 
          write_rffe_4byte_ext(0x5, 0x16,  _config._slot_data_[slot]); 		      
          write_rffe_1byte_std_x2(0x4, 0x1c, _config._slot_trigger[slot], 0x5, 0x1c, _config._slot_trigger[slot], _config._tr_gap_delay); //Trigger
          break;      
          
    case M5_PA:      
          write_rffe_4byte_ext(0x5, 0x11,  _config._slot_data_[slot]); 
          write_rffe_1byte_std(_config.pa_usid, 0, _config._slot_data_[slot][4]); //write to address 0 of PA
          write_rffe_1byte_std(_config.pa_usid, 1, _config._slot_data_[slot][5]); //write to address 1 of PA 
          //Delay(5);
          write_rffe_1byte_std(0x0, 0x1c, 1);  //broadcast trigger
          break;
          
    case SB4_PA:
          write_rffe_4byte_ext(0x5, 0x14,  _config._slot_data_[slot]); 
          write_rffe_1byte_std(_config.pa_usid, 0, _config._slot_data_[slot][4]); //write to address 0 of PA
          write_rffe_1byte_std(_config.pa_usid, 1, _config._slot_data_[slot][5]); //write to address 1 of PA 
          //Delay(5);
          write_rffe_1byte_std(0x0, 0x1c, 1);  //broadcast trigger
          break;
   
		case CUDA_PA:  //Added in B9
          write_rffe_1byte_std(0x5, 0x19, _config._slot_data_[slot][0]); //Cuda single byte 
          write_rffe_1byte_std(_config.pa_usid, 0, _config._slot_data_[slot][4]); //write to address 0 of PA
          write_rffe_1byte_std(_config.pa_usid, 1, _config._slot_data_[slot][5]); //write to address 1 of PA 
          write_rffe_1byte_std(0x0, 0x1c, 1);  //broadcast rigger
           break;
		
    case ONEBYTE_M:
          //write_rffe_1byte_std(0x5, _config.exit_byte_addr, exit_byte_data); //psm exit
          write_rffe_1byte_std(0x5, _config.one_byte_addr, _config.one_byte[slot]);  //slot
          break;
          
    case ONEBYTE_S:
          write_rffe_1byte_std(0x5, _config.one_byte_addr, _config.one_byte[slot]);  //slot
          break;         
          
    case ONEBYTE_C:
			    //Setup_HAL_SPI_Transmit_RFFE_1B_EXT(&hspi1, dat, 0x4); 	
          write_rffe_1byte_std(0x5, _config.one_byte_addr, _config.one_byte[slot]);  //slot
		      //BIT_BANG_write_rffe_1byte_xtd(0x5, _config.one_byte_addr, _config.one_byte[slot]);
          break; 
	  case ONEBYTE_C_TRIGGER:   //Added in B9 for One-byte cuda w trigger
          write_rffe_1byte_std(0x5, _config.one_byte_addr, _config.one_byte[slot]);  //slot -> customer GUI, triggered onebyte.
          write_rffe_1byte_std(0x5, 0x1c, 1);
          break;  
		
		      // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
          // CUDA+GTO+PA
          // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    case GTO_CUDA_PA:
          if (_config.gto_cuda_prior[slot] == 1)  //boost++ => trigger gto first
            {
               write_rffe_1byte_std(0x4, 0x18, _config._slot_data_[slot][1]); //gto 
               write_rffe_1byte_std(0x5, 0x19, _config._slot_data_[slot][0]); //cuda slot
        	     write_rffe_1byte_std_x2(0x4, 0x1c, 1, 0x5, 0x1c, 1, _config._tr_gap_delay); //GtoTrigger then Cuda Trigger
            }
          else
            {
              write_rffe_1byte_std(0x4, 0x18, _config._slot_data_[slot][1]); //gto 
              write_rffe_1byte_std(0x5, 0x19, _config._slot_data_[slot][0]); //cuda slot
              write_rffe_1byte_std_x2(0x5, 0x1c, 1, 0x4, 0x1c, 1, _config._tr_gap_delay); //Cuda Trigger then GTO Trigger
            }
          
              write_rffe_1byte_std(_config.pa_usid, 0, _config._slot_data_[slot][4]); //write to address 0 of PA
              write_rffe_1byte_std(_config.pa_usid, 1, _config._slot_data_[slot][5]); //write to address 1 of PA 
              write_rffe_1byte_std(_config.pa_usid, 0x1c, 1);  //pa trigger
         break;
    default: break;		
    }
		
		_config._slot_delay = (_config._slot_data_[slot][6] - _config._slot_delay_correction); ///// / 8.6; 
		//HAL_Delay(_config._slot_delay ); //About 4.18uS per 1
    TM_DelayMicros(_config._slot_delay * 6);
		
	}
   //ARB TRIG 2
   Pin_D_On(12);
	 Pin_D_Off(12);
}

//------------------------------------------------------------
//This is a generic decode used to expand the command sets
//------------------------------------------------------------
void sub_Decode(char acmd[100])
{

    switch (acmd[2]) {
//    case 'e': //exit delay >a e45
//         val  = AtoH(acmd, 3, 2);
//         _config.exit_delay = val;
//         _config._slot_delay_correction = 41 - _config.exit_delay;
//         break;
    
    case 'f': //num frames cuda dynamic - demo (40 slot) 	
			    //>a f064
			   _config.num_dyn_set = AtoH(acmd, 4, 3);		
		      break;
		
    case 'p': //update PA usid; >a pf
          _config.pa_usid = AtoH(acmd, 3, 1);
          break;
          
		case 'b': //a b1 or a b0
			    _config.bitbang_dynamic = _config.gsid = AtoH(acmd, 3, 1);
		      break;
		
    case 'c':  //a cd0; //0xd0=208
           //cuda + gto dynamic transmit mode
           //# define CUDA_TR_GTO_TR 208
           //# define GTO_TR_CUDA_TR 209
           //other wise just 108
		      //_config.what_chip = 0;
          _config.what_chip = AtoH(acmd, 3, 2);
         break;
    
    case 'i':  //a id0; //0xd0=208
          _config.gsid = AtoH(acmd, 3, 1);
         break;
         
    case 't':
          //char _tr_gap_delay;
          _config._tr_gap_delay = AtoH(acmd, 3, 4);
          break;
          
		case 'T':
			    //dynamic gap time correction  ---- a T233
		      _config._slot_delay_correction = AtoH(acmd, 3, 3);
		      break;
		
		case 'S':
			    //dynamic gap time correction  ---- a S233
		      _config._slot_delay_correction_2 = AtoH(acmd, 3, 3);
		      break;
		
		case 's': //a spmi=1
			    if  (acmd[7] == '1') _config.spmi = 1;
		      else _config.spmi = 0;
		      break;
		
		case 'r': //reset or clear interrupt
			    	Pin_B_Off (11);
		      break;
		
		case 'V': //vcore2 to dynamic (used for DVS demo)
			    _config.vcore2_dynamic = !_config.vcore2_dynamic;	
		      break;	    
    default:
          	
		      _config._slot_delay_correction = AtoH(acmd, 3, 3);
          //HAL_UART_Transmit (&huart3, acmd, strlen(acmd), 1000);		
			    break;
    }
		acmd[0]='\0';
}

void echo_slot_array(void)
{
  // uint32_t s;
   char tstr[150];
   
//   for(s=0; s<20; s++)
//   {
//      sprintf(tstr, "%d> b0=%.2x, b1=%.2x, b2=%.2x, b3=%.2x, b4=%.2x, b5=%.2x, delay=%.3x, tr_prior=%d\n\r", 
//		           s, _config._slot_data_[s][0], 
//		              _config._slot_data_[s][1],
//		              _config._slot_data_[s][2],
//		              _config._slot_data_[s][3],
//		              _config._slot_data_[s][4],
//		              _config._slot_data_[s][5],
//		              _config._slot_data_[s][6], _config.gto_cuda_prior[s]);  
//      //SerialPutString(tstr);  

//		  HAL_UART_Transmit (&huart3, (uint8_t *)  tstr, strlen(tstr), 1000);
//   }
	sprintf(tstr, "> v0=%.2x, v1=%.2x, v2=%.2x, v3=%.2x, Delay=%.3x\n\r",
	 		      _config.dvs_vout[0],
		        _config.dvs_vout[1],
		        _config.dvs_vout[2],
		        _config.dvs_vout[3],
            _config.vcore2_dyn_delay);
						HAL_UART_Transmit (&huart3, (uint8_t *)  tstr, strlen(tstr), 500);
}

/********** GPIO COMMAND *************/
void gpio_write_port(char pcmd[100])
{
   uint32_t pdat;

  switch(pcmd[1]) {
    case 'd': 
    case 'D': //pd 0000
              pdat = AtoH(pcmd, 3, 4);
              set_gpio_port_d(pdat); 
    break;
    
    case 'e': 
    case 'E': 
              pdat = AtoH(pcmd, 3, 4);
              set_gpio_port_e(pdat); 
     break; 

     default: 
     break;  
  }
}

//*******************************************************
// Convert ASCII to HEX
// Start at location n for m characters
// return unsigned int
//*******************************************************
uint32_t AtoH(char cmd[100], uint32_t n, uint32_t m)	
{
	uint32_t  i,j;
	char k; 
	
	j = 0;
	for (i = n ; i < n + m ; i++)
	{
		j = j << 4;
		k = cmd[i];
		if((k > 0x2F) && (k < 0x3A))
			k = k - 0x30;
		else if((k > 0x40) && (k < 0x47))  // caps
			k = k -  0x37;
		else if((k > 0x60) && (k < 0x67))  // lower case
			k = k - 0x57;
		else
		{
			j =  0xFF;  // command error
			break;
		}
		j = j + (uint32_t)k;
	}	
	return j;
} 


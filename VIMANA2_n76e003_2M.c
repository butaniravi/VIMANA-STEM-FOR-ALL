//***********************************************************************************************************
//  VIMANA2 N76e003 firmware
//  Designed and developed by Ravi Butani 
//  Contact : ravi.butani03@gmail.com
//  Version 2 - April 2023
//  Project Documentation : https://hackaday.io/project/190462-vimana-stem-for-all
//  P11-->PWM1 	P10-->PWM2
//  P06-->TX   	P07-->RX
//  P12-->LED_ARM
//  P13-->LED_CON
//***********************************************************************************************************
#include "N76E003.h"
#include "Common.h"
#include "Delay.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "uart0_function.h"
#define PKT_LENGTH   6
#define LOOP_DELAY   1
#define MAX_LOOP_CNT 1000
UINT8 pkt_rx[PKT_LENGTH];
UINT16 mot_l,mot_r;
UINT32 cnt_loop = 0,cnt_pkt = 0, lst_cnt_pkt = 0;

void set_PWM_period(unsigned int value);
void set_PWM1(unsigned int value);
void set_PWM2(unsigned int value);
void init_PWM(void);


/************************************************************************************************************
*    Main function 
************************************************************************************************************/

void main (void)
{
	MODIFY_HIRC_166();  	// Change FOSC to 16.6MHz for 115200 accurate baudrate
	Timer0_Delay1ms(200); // Keep TX RX floating until ESP8266 complete throw junk data on UART 
	init_uart0(); 				// Initlize UART
	init_PWM();  					// Initlize PWM
	P12_PushPull_Mode;       // P12-->LED_ARM
  P13_PushPull_Mode;		   // P13-->LED_CON
	P12 = 0;  // Turn OFF LED_ARM
	P13 = 1;  // Turn ON LED_CON
	set_PWM1(0);					// STOP both motors
	set_PWM2(0);
  while(1)
	{
		Timer0_Delay1ms(LOOP_DELAY);
		cnt_loop++;
		if(uart0_rx_available()>= PKT_LENGTH)
		{
			pkt_rx[0] = uart0_rx_read(); // mcmd
			pkt_rx[1] = uart0_rx_read(); // high byte
			pkt_rx[2] = uart0_rx_read(); // low byte
			pkt_rx[3] = uart0_rx_read(); // high byte
			pkt_rx[4] = uart0_rx_read(); // low byte
			pkt_rx[5] = uart0_rx_read(); // checksum
			if(pkt_rx[5] == (UINT8)(pkt_rx[0]+pkt_rx[1]+pkt_rx[2]+pkt_rx[3]+pkt_rx[4]))
			{
				if((pkt_rx[0]&0x80) == 0x80)	// if armed update PWM based on RX data
				{
					uart0_rx_flush();
					mot_l = pkt_rx[1];
					mot_l <<= 8;
					mot_l |= pkt_rx[2];
					set_PWM1(mot_l);
					mot_r = pkt_rx[3];
					mot_r <<= 8;
					mot_r |= pkt_rx[4];
					set_PWM2(mot_r);	
					cnt_pkt++;
				}	
				else
				{
					uart0_rx_flush();
					set_PWM1(0); // STOP both motors
				  set_PWM2(0);
				}	
				P13 = pkt_rx[0]&0x01;  // Turn ON LED_CON if con_flag is 1								
			}
			uart0_rx_flush();
		}
		// Every 1000x1.5 1.5 sec check UART pkt update only important in case UART hanges
		if(cnt_loop >= MAX_LOOP_CNT) 
		{
			cnt_loop = 0;
			if(lst_cnt_pkt == cnt_pkt) // if no new pkt detected than turnoff motors
			{
				set_PWM1(0); // STOP both motors
				set_PWM2(0);
			}
			lst_cnt_pkt = cnt_pkt;
		}
		// ARM LED Flash Control depends upon MAX_LOOP_CNT
		if(cnt_loop == 1)P12 = 1; 																				// LED_ARM ON
		else if ((cnt_loop == 50)  && ((pkt_rx[0]&0x80)==0x80))	P12 = 0;	// LED_ARM OFF
		else if ((cnt_loop == 200) && ((pkt_rx[0]&0xA0)==0xA0)) P12 = 1;	// LED_ARM ON
		else if ((cnt_loop == 250) && ((pkt_rx[0]&0xA0)==0xA0)) P12 = 0;	// LED_ARM OFF
	}	
}

//************** Initlizing PWM1 and PWM2 ***************
void init_PWM(void)
{
  P11_PushPull_Mode; // PWM1 on P11
	P10_PushPull_Mode; // PWM2 on P10					             
	PWM1_P11_OUTPUT_ENABLE;
	PWM2_P10_OUTPUT_ENABLE;
	PWM_IMDEPENDENT_MODE;
	PWM_EDGE_TYPE;
	set_CLRPWM;
	PWM_CLOCK_FSYS;
	PWM_CLOCK_DIV_2;
	PWM_OUTPUT_ALL_NORMAL;
	set_PWM_period(512); // 16kHz
	set_PWMRUN;
}

//************** Set PWM1 and PWM2 period ***************
void set_PWM_period(unsigned int value){
  PWMPL = (value & 0x00FF);
  PWMPH = ((value & 0xFF00) >> 8);
}

//************** Set PWM1 dutycycle ***************
void set_PWM1(unsigned int value){
  PWM1L = (value & 0x00FF);
  PWM1H = ((value & 0xFF00) >> 8);
  set_LOAD;
}

//************** Set PWM2 dutycycle ***************
void set_PWM2(unsigned int value){
  PWM2L = (value & 0x00FF);
  PWM2H = ((value & 0xFF00) >> 8);
  set_LOAD;
}
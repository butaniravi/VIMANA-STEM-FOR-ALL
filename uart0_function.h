//***********************************************************************************************************
//  UART 0 Function Headerfile
//  VIMANA2 N76e003 firmware library
//  Designed and developed by Ravi Butani 
//  Contact : ravi.butani03@gmail.com
//  Version 2 - April 2023
//  Project Documentation : https://hackaday.io/project/190462-vimana-stem-for-all
//  Should be used with N76e003 to make communication possible with ESP8266 over UART
//***********************************************************************************************************
#define RXBUFSIZE 32

UINT8 head = 1;
UINT8 tail = 1;
UINT8 rxbuf[RXBUFSIZE];

//Function Prototypes
void init_uart0(void);
unsigned int uart0_rx_available(void);
char uart0_rx_read(void);
void uart0_rx_flush(void);
void MODIFY_HIRC_166(void);

//************** Initlizing uart ***************//
void init_uart0(void)
{
	P06_Quasi_Mode;	  // TX pin P06
	P07_Input_Mode;		// RX pin P07
	
	SCON = 0x50;			// Special setting the mode 3 and 
	TMOD |= 0x20;    	// Timer1 Mode1
    
  set_SMOD;        	// UART0 Double Rate Enable
  set_T1M;
  clr_BRCK;        	// Serial port 0 baud rate clock source = Timer1
	TH1 = 256 - (1037500/115200);               // 115200 Baudrate at 16.6MHz RC Osc
	set_TR1;	
	set_RB8;					//This bit is for setting the stop bit 2 high/low status, 	
	//TI=1; 
  set_ES;           //enable UART interrupt
  set_EA;           //enable global interrupt
}


//************** UART bytes available  0 means nothing available ***************
unsigned int uart0_rx_available(void)
{
	return (RXBUFSIZE+head-tail)%RXBUFSIZE;
}

//************** UART read one byte from rx buffer ***************
char uart0_rx_read(void)
{
	if(head == tail)
    {
        return 0;
    }
    else
    {
        UINT8 c = rxbuf[tail];
        tail = (tail+1)%RXBUFSIZE;
        return c;
    }	
}

//************** UART flush rx buffer ***************
void uart0_rx_flush(void)
{
	head=tail;
}

//************** Modify RC OSC Freq to 16.6MHz from 16MHz for 115200 baud ***************
void MODIFY_HIRC_166(void) 
{ 
	unsigned char hircmap0,hircmap1; 
	unsigned int trimvalue16bit; 
	/* Since only power on will  reload RCTRIM0 and RCTRIM1 value, check power on flag*/  
	if ((PCON&SET_BIT4)==SET_BIT4)      
	{   
		hircmap0 = RCTRIM0;   
		hircmap1 = RCTRIM1;   
		trimvalue16bit = ((hircmap0<<1)+(hircmap1&0x01)); 
		trimvalue16bit = trimvalue16bit - 15;   
		hircmap1 = trimvalue16bit&0x01;   
		hircmap0 = trimvalue16bit>>1;   
		TA=0XAA;   TA=0X55;   
		RCTRIM0 = hircmap0;   
		TA=0XAA;   TA=0X55;   
		RCTRIM1 = hircmap1; 
		/* After modify HIRC value, clear power on flag */   
		PCON &= CLR_BIT4;  
	} 
}

//************* UART0 ISR for RXC  ********************* 
void SerialPort0_ISR(void) interrupt 4 
{
    if (RI==1) 
    {                                       /* if reception occur */
        clr_RI;                             /* clear reception flag for next reception */
			  rxbuf[head]= SBUF;
        head = (head+1)%RXBUFSIZE;
    }
}
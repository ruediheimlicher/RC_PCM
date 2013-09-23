#pragma config FOSC = HSHP,  PLLCFG = OFF //FOSC = 20MHz

#include "p18f45k22.h"
#include "delays.h"
#include "stdlib.h"

#define LCD_RS	 LATDbits.LATD2
#define LCD_EN	 LATDbits.LATD3
#define LCD_D4	 LATDbits.LATD4
#define LCD_D5	 LATDbits.LATD5
#define LCD_D6	 LATDbits.LATD6
#define LCD_D7	 LATDbits.LATD7

#define LCD_RS_Dir	TRISDbits.TRISD2
#define LCD_EN_Dir	TRISDbits.TRISD3
#define LCD_D4_Dir	TRISDbits.TRISD4
#define LCD_D5_Dir	TRISDbits.TRISD5
#define LCD_D6_Dir	TRISDbits.TRISD6
#define LCD_D7_Dir	TRISDbits.TRISD7

#define Enable Delay10TCYx(1); LCD_EN=1; Delay10TCYx(1); LCD_EN=0; Delay10TCYx(1);

#define EEPROM_CS		LATCbits.LATC2
#define EEPROM_CS_Dir	TRISCbits.TRISC2

void LCD_Init(void);
void LCD_Send(unsigned char chr);
void LCD_DDRAM_Addr(unsigned char addr);
void LCD_CGRAM_Addr(unsigned char addr);
void LCD_Char(unsigned char row, unsigned char column, unsigned char text);
void LCD_Text(unsigned char row, unsigned char column, unsigned char *text);
void LCD_Gen_Char(unsigned char number, unsigned char *code);

void SPI_Init(void);
unsigned char SPI_Transfer(unsigned char byte);
void EEPROM_Write_Enable(unsigned char w_enable);
unsigned char EEPROM_Read_Status(void);
void EEPROM_Write(unsigned int adress, char data);
unsigned char EEPROM_Read(unsigned int adress);

void main(void)
{
	unsigned char txt[]="25LC128 Test";
	unsigned char stat, stat_txt[10];
	unsigned char data, data_txt[10];
   
	LATB=0x00;
	TRISB=0x00;
	ANSELB=0x00;
	
	LATC=0x00;
	TRISC=0x00;
	ANSELC=0x00;
   
	LATD=0x00;
	TRISD=0x00;
	ANSELD=0x00;
   
   
	
	LCD_Init();
	SPI_Init();
	
	LCD_Text(1,1,txt);
	
	EEPROM_CS_Dir=0;
	
	
	EEPROM_Write_Enable(1);
	
	EEPROM_Write(59,64);
   
	data=EEPROM_Read(59);
	itoa(data,data_txt);
	LCD_Text(3,1,data_txt);
	
	EEPROM_Write_Enable(1);
	
	EEPROM_Write(512,55);
   
   
	data=EEPROM_Read(512);
	itoa(data,data_txt);
	LCD_Text(4,1,data_txt);
	
   
	while(1);
}

void SPI_Init(void)
{
   //Init SPI:
   SSP1STAT=0b00000000;
   SSP1CON1=0b00100001;
   SSP1CON1bits.CKP=0;
   SSP1STATbits.CKE=1;
   
   TRISCbits.TRISC3=0;		// SCK: output
   TRISCbits.TRISC4=1;		// SDI: input
   TRISCbits.TRISC5=0;		// SDO: output
}

unsigned char SPI_Transfer(unsigned char byte)
{
   PIR1bits.SSPIF = 0;			// Just in case, clear Interrupt Flag
   
   SSP1BUF = byte;
   while(SSP1STATbits.BF == 0);
   
   return SSP1BUF;
}

void EEPROM_Write_Enable(unsigned char w_enable)
{
	EEPROM_CS=0;
	if(w_enable==0)
	{
		SPI_Transfer(0b00000100);	//Write Disable 0x04
	}
	else
	{
		SPI_Transfer(0b00000110);	//Write Enable 0x06
	}
	EEPROM_CS=1;
}

unsigned char EEPROM_Read_Status(void)
{
	unsigned char eeprom_stat;
	EEPROM_CS=0;
	SPI_Transfer(0b00000101); // 0x05
	eeprom_stat=SPI_Transfer(0x00);
	EEPROM_CS=1;
	return eeprom_stat;
}
void EEPROM_Write(unsigned int adress, char data)
{
	EEPROM_CS=0;
	SPI_Transfer(0b00000010);			//Write Instruction 0x02
	SPI_Transfer((adress>>8)&0xFF);		//Adress 15:8
	SPI_Transfer(adress&0xFF);			//Adress  7:0
	SPI_Transfer(data);					//Send Data
	EEPROM_CS=1;
	while(EEPROM_Read_Status()&0x01);
}

unsigned char EEPROM_Read(unsigned int adress)
{
	unsigned char data;
	EEPROM_CS=0;
	SPI_Transfer(0b00000011);			//Read Instruction 0x03
	SPI_Transfer((adress>>8)&0xFF);		//Adress 15:8
	SPI_Transfer(adress&0xFF);			//Adress  7:0
	data=SPI_Transfer(0x00);			//Send Data
	EEPROM_CS=1;
	return data;
}


void LCD_Init(void)
{
	LCD_RS_Dir=0;		//Set Pins to Outpur
	LCD_EN_Dir=0;
	LCD_D4_Dir=0;
	LCD_D5_Dir=0;
	LCD_D6_Dir=0;
	LCD_D7_Dir=0;
   
	Delay10KTCYx(20);	//Delay 40ms
   
	LCD_RS=0;
   
	LCD_D7=0;			//Function Set 8-bit
	LCD_D6=0;
	LCD_D5=1;
	LCD_D4=1;
	Enable
	Delay10KTCYx(21);	//Delay ca. 4.2ms
   
	LCD_D7=0;			//Function Set 8-bit
	LCD_D6=0;
	LCD_D5=1;
	LCD_D4=1;
	Enable
	Delay10TCYx(50);	//Delay ca. 100us
   
	LCD_D7=0; 			//Function Set 8-bit
	LCD_D6=0;
	LCD_D5=1;
	LCD_D4=1;
	Enable
	Delay10TCYx(19);	//Delay ca. 38us
   
	LCD_D7=0; 			//Function Set 4-bit
	LCD_D6=0;
	LCD_D5=1;
	LCD_D4=0;
	Enable
	Delay10TCYx(19);	//Delay ca. 38us
   
	LCD_D7=0; 			//Function Set
	LCD_D6=0;
	LCD_D5=1;
	LCD_D4=0;			//DL=0
	Enable
	LCD_D7=1; 			//N = 1
	LCD_D6=0;			//F = 0
	LCD_D5=0;			//x
	LCD_D4=0;			//x
	Enable
	Delay10TCYx(19);	//Delay ca. 38us
   
	LCD_D7=0; 			//Display ON/OFF
	LCD_D6=0;
	LCD_D5=0;
	LCD_D4=0;
	Enable
	LCD_D7=1;
	LCD_D6=1;			//D=1	(D=1:Display ON)
	LCD_D5=0;			//C=0	(C=1:Under-line cursor ON)
	LCD_D4=0;			//B=0	(B=1:Block cursor ON)
	Enable
	Delay10TCYx(19);	//Delay ca. 38us
   
	LCD_D7=0; 			//Clear Display
	LCD_D6=0;
	LCD_D5=0;
	LCD_D4=0;
	Enable
	LCD_D7=0;
	LCD_D6=0;
	LCD_D5=0;
	LCD_D4=1;
	Enable
	Delay100TCYx(80);	//Delay ca. 1.6ms
   
	LCD_D7=0; 			//Entry Mode
	LCD_D6=0;
	LCD_D5=0;
	LCD_D4=0;
	Enable
	LCD_D7=0;
	LCD_D6=1;
	LCD_D5=1;			//ID = 1
	LCD_D4=0;			//S = 0
	Enable
	Delay100TCYx(19);	//Delay ca. 38us
	
	LCD_DDRAM_Addr(0x00);
   
}

void LCD_Send(unsigned char chr)
{
	LCD_D7=(chr&0b10000000) ? 1:0;
	LCD_D6=(chr&0b01000000) ? 1:0;
	LCD_D5=(chr&0b00100000) ? 1:0;
	LCD_D4=(chr&0b00010000) ? 1:0;
	Enable
	LCD_D7=(chr&0b00001000) ? 1:0;
	LCD_D6=(chr&0b00000100) ? 1:0;
	LCD_D5=(chr&0b00000010) ? 1:0;
	LCD_D4=chr&0x01;
	Enable
	Delay10TCYx(20);  //Delay ca. 38us
}

void LCD_DDRAM_Addr(unsigned char addr)
{
	LCD_RS=0;
	LCD_Send(addr|0x80);
}

void LCD_CGRAM_Addr(unsigned char addr)
{
	LCD_RS=0;
	LCD_Send((addr&0x3F)|0x40);
}

void LCD_Char(unsigned char row, unsigned char column, unsigned char text)
{
	column--;
	if(row==2)
	{
		column=column+0x40;
	}
	if(row==3)
	{
		column=column+0x14;
	}
	if(row==4)
	{
		column=column+0x54;
	}
   
	LCD_DDRAM_Addr(column);
	LCD_RS=1;
   LCD_Send(text);
}

void LCD_Text(unsigned char row, unsigned char column, unsigned char * text)
{
	unsigned char counter;
	LCD_Char(row, column, text[0]);
	
   for(counter=1; text[counter]; counter++)
	{
    	LCD_Send(text[counter]);
   }
}

void LCD_Gen_Char(unsigned char number, unsigned char *code)
{
	unsigned char i;
	LCD_RS=0;
	number=number<<3;
	LCD_CGRAM_Addr(number);
	LCD_RS=1;
	for(i=0;i<8;i++)
	{
		LCD_Send(code[i]);
	}
}	



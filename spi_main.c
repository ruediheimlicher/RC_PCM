// http://read.pudn.com/downloads163/sourcecode/embed/741340/Mega16_MCP3208/main.c__.htm
//  spi_main.c
//  RC_PPM
//
//  Created by Ruedi Heimlicher on 26.07.2013.
//
//

#include <stdio.h>
// AVR mega16 + MCP3208 (12bit-A/D,SPI port) + UART
// by   >> wlasoi@hotmail.com
// date >> 23 September 2007
// Dept. of Physics, Fact. of Science, Ubonrajathane University

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include "delay.h"
#include "myDef.h"
#include "uart.h"
#include "mcp3208.h"

//**************************** Global Constant + Variable ************************//

const char usartPGM0[] PROGMEM = " >> AVR Project :: AVR + 8Ch-12Bit A/D\n\r";
const char usartPGM1[] PROGMEM = " >> MCU : ATMega16 , A/D Device : MCP3208\n\r";
const char usartPGM2[] PROGMEM = " >> by wlasoi@hotmail.com\n\r";
const char usartPGM3[] PROGMEM = " Command 0,1,2,3: \"0\" --> start A/D \n\r";
const char usartPGM4[] PROGMEM = " AtoD 12Bit process \n\r";
const char usartPGM5[] PROGMEM = " Revieve \"1\" test \n\r";
const char usartPGM6[] PROGMEM = " Revieve \"2\" test \n\r";
const char usartPGM7[] PROGMEM = " Revieve \"3\" test \n\r";

PGM_P usartPointer[8] PROGMEM = {usartPGM0,usartPGM1,usartPGM2,usartPGM3,usartPGM4,usartPGM5
   ,usartPGM6,usartPGM7};

//************************ Sub Function Prototype ********************************//

void MCP3208_ShowData(unsigned char ADcontrol,unsigned char ADchanel);

//*************************** main Program ***************************************//

int main (void)
{
   
   MCP3208_spiInit();  // init SPI mode3 , fclk/64
   
   USART_Init(USART_BAUD_SELECT(USART_BAUD_RATE,F_CPU));   // set uart baudrate = 19200
   sei();
   
   usart_puts_p(usartPointer[0]);
   usart_puts_p(usartPointer[1]);
   usart_puts_p(usartPointer[2]);
   usart_puts_p(usartPointer[3]);
   
   while(1)
   {
      while(DataInReceiveBuffer()) /* True if "Not" empty */
      {
         if (DataInReceiveBuffer())
            
            c = usart_getc();
         
         if(c =='0')   // Recieve "0"  for start A/D
         {
            usart_puts_p(usartPointer[4]);
            do{
               
               for(unsigned char i=0;i<=7;i++)
               {
                  MCP3208_ShowData(SingleEnd,i);
                  delay_us(100);
               }
               usart_puts("\n");
               delay_ms(499);
               
            }while(!DataInReceiveBuffer());
            
         }
         
         if (c =='1')  // Recieve "1"  test
         {
            usart_puts_p(usartPointer[5]);
         }
         
         if(c =='2')   // Recieve "2"  test
         {
            usart_puts_p(usartPointer[6]);
         }
         
         if (c =='3')  // Recieve "3"  test
         {
            usart_puts_p(usartPointer[7]);
         }
         
      }/* End of check Command data buffer*/
      
   }/* End of While() loop  */
   
   return 0;
}/* Eend of main() loop  */

// ********************  Sub Function  ***************************  //

void MCP3208_ShowData(unsigned char ADcontrol,unsigned char ADchanel)
{
   unsigned int temp16Bit;
   
   temp16Bit = MCP3208_spiRead(ADcontrol,ADchanel);
   
   usart_puts("CH-");
   usart_SendInt(ADchanel);
   usart_puts(" = ");
   usart_SendInt(temp16Bit);
   usart_puts(" , ");
   
   gReciveHighByte=0;
   gReciveLowByte=0;
   temp16Bit=0;
}

// ********************  END  ***************************  //

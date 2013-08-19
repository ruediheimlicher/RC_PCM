//
//  RC_PPM.c
//  
//
//  Created by Sysadmin on 20.07.13
//  Copyright Ruedi Heimlicher 2013. All rights reserved.
//


#include <string.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>

#include "lcd.c"
#include "adc.c"
#include "version.c"
#include "usb_rawhid.c"
#include "def.h"

#include "spi.c"
#include "spi_adc.c"


// USB
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))


volatile uint8_t do_output=0;
static volatile uint8_t buffer[32]={};
static volatile uint8_t sendbuffer[32]={};


#define TIMER0_STARTWERT	0x40

volatile uint8_t timer0startwert=TIMER0_STARTWERT;
#define USB_DATENBREITE 32
//volatile uint8_t rxbuffer[USB_DATENBREITE];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[USB_DATENBREITE];

//uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit

void delay_ms(unsigned int ms);

volatile uint8_t           adcstatus=0x00;

volatile uint8_t           usbstatus=0x00;
static volatile uint8_t    potstatus=0x80; // Bit 7 gesetzt, Mittelwerte setzen
static volatile uint8_t    impulscounter=0x00;

#define USB_SEND  0 

volatile uint8_t status=0;

volatile uint8_t           PWM=0;
static volatile uint8_t    pwmposition=0;
static volatile uint8_t    pwmdivider=0;


volatile char SPI_data='0';
volatile char SPI_dataArray[SPI_BUFSIZE];
volatile uint16_t POT_Array[SPI_BUFSIZE];
volatile uint16_t Mitte_Array[SPI_BUFSIZE];

volatile uint16_t Batteriespannung =0;
volatile short int received=0;

volatile uint16_t abschnittnummer=0;
volatile uint16_t usbcount=0;



void startTimer2(void)
{
   //timer2
   TCNT2   = 0; 
   //	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode 
   TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8 
   //	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt 
   TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt 
   //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64 
   TCCR2A = 0x00;
   
   sei();
}

void stopTimer2(void)
{
   TCCR2B = 0;
}

void slaveinit(void)
{
	//OSZIPORTDDR |= (1<<PULS);	// Output
	//OSZIPORT |= (1<<PULS);		// HI
	
	LOOPLEDDDR |=(1<<LOOPLED);
	LOOPLEDPORT |= (1<<LOOPLED);	// HI
	
	//Pin 0 von   als Ausgang fuer OSZI
	OSZIPORTDDR |= (1<<OSZI_PULS_A);	//Pin 0 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_A);		// HI
	
   OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
	
   
	TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang f�r Taste 0
	TASTENPORT |= (1<<TASTE0);	//Pull-up
   
   //	DDRB &= ~(1<<PORTB1);	//Bit 1 von PORT B als Eingang f�r Taste 1
   //	PORTB |= (1<<PORTB1);	//Pull-up
	
   
	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);		//Pin 4 von PORT D als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT D als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT D als Ausgang fuer LCD
   
	
   DDRD |= (1<<PORTD6);
   PORTD |= (1<<PORTD6);
   
   ADC_DDR &= ~(1<<PORTF0);
   
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

// http://www.co-pylit.org/courses/COSC2425/lectures/AVRNetworks/index.html

// send a SPI message to the other device - 3 bytes then go back into
// slave mode

void send_message(void)
{
   spi_init(SPI_MODE_1, SPI_MSB, SPI_NO_INTERRUPT, SPI_MSTR_CLK8);
   if (SPCR & (1<<MSTR)) { // if we are still in master mode
      send_spi(READ_ADC_COMMAND);
      send_spi(0x02);
      send_spi(0x00);
   }
   //spi_init(SPI_MODE_1, SPI_MSB, SPI_INTERRUPT, SPI_SLAVE);
   //flash_led(5);
}

void parse_message(void)
{
   switch(SPI_dataArray[0])
   {
      case 0:
      {
         
      }break;
      default:
      {
         
      }}
}


ISR(SPI_STC_vect)
{
   
//   SPI_dataArray[received++] = received_from_spi(0x00);
   
//   if (received >= SPI_BUFSIZE || SPI_dataArray[received-1] == 0x00)
   {
//      parse_message();
//      received = 0;
   }
}

void timer1_init(void)
{
   // Quelle http://www.mikrocontroller.net/topic/103629
   
   //#define FRAME_TIME 20 // msec
   DDRB |= (1<<PORTB5); // OC1A Ausgang
   PORTB |= (1<<PORTB5); // OC1A Ausgang
   
   DDRD |= (1<<PORTD5); //  Ausgang
   PORTD |= (1<<PORTD5); //  Ausgang

   //ICR1   = FRAME_TIME * 1200;								// PWM cycle time in usec, 50 ms
   TCCR1A = (1<<COM1A0) | (1<<COM1A1);// | (1<<WGM11);	// OC1B set on match, set on TOP
   TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11);		// TOP = ICR1, clk = sysclk/8 (->1us)
   TCNT1  = 0;														// reset Timer
   
                              // Impulsdauer
   OCR1B  = 0x1FF;				// Impulsdauer des Kanalimpulses
   
   TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt:
   TIMSK1 |= (1 << OCIE1B);  // enable timer compare interrupt:
   OSZI_A_LO ;
   OSZI_B_LO ;

   impulscounter = 0;
   OCR1A  = 2.141*POT_Array[impulscounter];

} // end timer1

void timer1_stop(void)
{
   TCCR1A = 0;
   
}




ISR(TIMER1_COMPA_vect)	 //Ende der Pulslaenge fuer einen Kanal
{
   
   impulscounter++;
   if (impulscounter < ANZ_POT)
   {
      OSZI_A_LO ;
      // Laenge des naechsten Impuls setzen
      OCR1A  = 2.141*POT_Array[impulscounter];
      TCNT1  = 0;
   }
   else
   {
      OSZI_A_HI ;
      
      // Alle Impulse gesendet, Timer1 stop. Timer1 wird bei Beginn des naechsten Paketes wieder gestartet 
      timer1_stop();
      OSZI_B_HI ;
   }
   
   //OSZI_B_LO ;
   _delay_us(10);
   
   
}


ISR(TIMER1_COMPB_vect)	 //Ende des Kanalimpuls. ca 0.3 us
{
   //OSZI_A_LO ;
   PORTB |= (1<<PORTB5); // OC1A Ausgang
   
   OSZI_A_HI ;
}


void timer0 (void) // nicht verwendet
{
// Timer fuer Exp
	//TCCR0 |= (1<<CS01);						// clock	/8
	//TCCR0 |= (1<<CS01)|(1<<CS02);			// clock	/64
	TCCR0B |= (1<<CS02)| (1<<CS02);			// clock	/256
	//TCCR0 |= (1<<CS00)|(1<<CS02);			// clock /1024
	TCCR0B |= (1 << CS10); // Set up timer 
	OCR0A = 0x02;
	
	//TIFR |= (1<<TOV0);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<TOIE0);							//Overflow Interrupt aktivieren
	TCNT0 = TIMER0_STARTWERT;					//R�cksetzen des Timers

}

/*
void timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);							//8-Bit Timer, Timer clock = system clock/256

//Takt fuer Servo
	TCCR2 |= (1<<CS20)|(1<<CS21);				//Takt /64	Intervall 64 us

	TCCR2 |= (1<<WGM21);							//	ClearTimerOnCompareMatch CTC

	//OC2 akt
//	TCCR2 |= (1<<COM20);						//	OC2 Pin zuruecksetzen bei CTC


	TIFR |= (1<<TOV2);							//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<OCIE2);							//CTC Interrupt aktivieren

	TCNT2 = 0x00;									//Zaehler zuruecksetzen
	
	OCR2 = wert;									//Setzen des Compare Registers auf Servoimpulsdauer
} 
*/

volatile uint16_t timer2Counter=0;
volatile uint16_t timer2BatterieCounter=0;

ISR (TIMER2_OVF_vect) 
{ 
	timer2Counter ++;
   
	if (timer2Counter >= 0x474) // Laenge des Impulspakets 20ms
	{
      
      potstatus |= (1<<POT_START); // Potentiometer messen
      
      
      timer2BatterieCounter++; // Intervall fuer Messung der Batteriespannung
      if (timer2BatterieCounter >= 0xF)
      {
         adcstatus |= (1<<ADC_START); // Batteriespannung messen
         timer2BatterieCounter = 0;
         
      }

		timer2Counter = 0;
       ;
	} 
	TCNT2 = 10;							// ergibt 2 kHz fuer Timertakt
}

/*
ISR(TIMER2_COMP_vect) // Schaltet Impuls an SERVOPIN0 aus
{
//		lcd_clr_line(1);
//		lcd_puts("Timer2 Comp\0");
		TCCR20=0;
}
*/

void setMitte(void)
{
   for (uint8_t i=0;i< SPI_BUFSIZE;i++)
   {
      Mitte_Array[i] = POT_Array[i];
   }
}


#pragma mark - main
int main (void) 
{
    int8_t r;

   uint16_t count=0;
    
	// set for 16 MHz clock
	CPU_PRESCALE(0);
    
	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;
    
	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	sei();
	
	
	slaveinit();
	
	spi_init(SPI_MODE_1, SPI_MSB, SPI_INTERRUPT, 0);
   
   MCP3208_spi_Init();
   
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	lcd_puts("Guten Tag\0");
	delay_ms(100);
	lcd_cls();
	//lcd_puts("READY\0");
	lcd_puts("V: \0");
	lcd_puts(VERSION);
   lcd_clr_line(1);

	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;
	//timer0();
	
	initADC(0);
	//wdt_enable(WDTO_2S);
	
	uint16_t loopcount0=0;
	uint8_t loopcount1=0;

	
	
	
	/*
	Bit 0: 1 wenn wdt ausgel�st wurde
	 
	  */ 
	uint8_t i=0;
   
	//timer2
	TCNT2   = 0; 
//	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode 
	TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8 
//	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt 
	TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt 
	//OCR2A   = 5;             // Set CTC compare value with a prescaler of 64 
    TCCR2A = 0x00;
   
   sei();
   
   PWM = 0;
   
   char* versionstring = (char*) malloc(4);
   strncpy(versionstring, VERSION+9, 3);
   versionstring[3]='\0';
   volatile uint16_t versionint = atoi(versionstring);
   volatile uint8_t versionintl = versionint & 0x00FF;
   //versionint >>=8;
   volatile uint8_t versioninth = (versionint & 0xFF00)>>8;
   
   uint8_t anzeigecounter=0;
   
#pragma mark while	  
	while (1)
	{
      //OSZI_B_LO;
		//Blinkanzeige
		loopcount0+=1;
		if (loopcount0==0x4FFF)
		{
			loopcount0=0;
			loopcount1+=1;
			LOOPLEDPORT ^=(1<<LOOPLED);
         PORTD ^= (1<<PORTD6);
			//
			//timer0();
         
         if (loopcount1%0x0F == 0)
         {
            //lcd_gotoxy(18,1);
            //lcd_puthex(hidstatus);
         }
         else if(loopcount1%8 == 4)
         {
            //lcd_gotoxy(18,1);
            //lcd_putc('*');
            //lcd_putc('*');
            
         }
         
         sendbuffer[0]=0x33;
         sendbuffer[1]= abschnittnummer;
         sendbuffer[2]= abschnittnummer>>8;
         abschnittnummer++;
         sendbuffer[3]= usbcount & 0xFF;
         
         
         sendbuffer[4] = loopcount1&0xFF;
         uint16_t adc0wert = adc_read(0);
         sendbuffer[5] = adc0wert & 0xFF;
         sendbuffer[6] = (adc0wert>>8) & 0xFF;

         
         if (loopcount1%0x1F == 0)
         {
            /*
            lcd_gotoxy(0,1);
            lcd_putint16(POT_Array[0]);
            lcd_putc('*');
            lcd_putint16(POT_Array[1]);
            lcd_putc('*');
             */
         }
         
         for (int i=0;i<8;i++)
         {
            
            sendbuffer[8+2*i]=(POT_Array[i] & 0xFF);    // 8 10
            sendbuffer[8+2*i+1]= (POT_Array[i]>>8) & 0xFF;  // 9  11
            
         }
         //OSZI_B_LO;
         usb_rawhid_send((void*)sendbuffer, 50); // 20 us
		//OSZI_B_HI;
      }
      
      /**	ADC	***********************/

      if (adcstatus & (1<< ADC_START)) // ADC starten
      {
         Batteriespannung = adc_read(0);
         
         adcstatus &=  ~(1<< ADC_START);
         
         /*
         lcd_gotoxy(0,0);
         
         lcd_putint12Bit(Batteriespannung);
         
         lcd_putc('*');
         uint8_t high = (Batteriespannung & 0xFF00)>>8; // *0xFF rechnen und zu low dazuzaehlen
         lcd_putint(high);
         uint8_t low = (Batteriespannung & 0xFF);
         lcd_putc('*');
         lcd_putint(low);
         //lcd_puthex(loopcount1);
          */
         ;

      }
      
      if (potstatus & (1<< POT_START)) // POT starten
      {
         //OSZI_A_LO ;
         uint8_t i=0;
         for(i=0;i< ANZ_POT;i++)
         {
            //lcd_putint(i);
            if (i<2)
            {
               //POT_Array[i] = MCP3208_spiRead(SingleEnd,i);
               // Filter
               POT_Array[i] = 3*POT_Array[i]/4 + (MCP3208_spiRead(SingleEnd,i)/4);
               _delay_us(100);
            }
            else
            {
               POT_Array[i] = 0x800;
            }
         }
         anzeigecounter++;
         
         // Mittelwert speichern
         if (potstatus & (1<< POT_MITTE))
         {
            setMitte();
            potstatus &= ~(1<< POT_MITTE);
         }
         
         if (anzeigecounter %0xF ==0)
         {
            //lcd_gotoxy(0,1);
             if (usbstatus & (1<<USB_SEND))
             {
                //lcd_putc('+');
                usbstatus &= ~(1<<USB_SEND);
             }
            /*
            lcd_putint12Bit(POT_Array[0]);
            lcd_putc(' ');
            lcd_putint12Bit(POT_Array[1]);
             */
         }
         
         //usb_rawhid_send((void*)sendbuffer, 50);
         potstatus &= ~(1<< POT_START);
         
         
         
         
         timer1_init();
        //PORTD &= ~(1<<PORTD5); //  LO
         
         
         //timer1_stop();
         
         
         //OSZI_A_HI ;
      } // end Pot messen
      
      /**	END ADC	***********************/
      
       /**	Begin USB-routinen	***********************/
      
        // Start USB
      //lcd_putc('u');
      //OSZI_B_LO;
      r = usb_rawhid_recv((void*)buffer, 0);
      //OSZI_B_HI;
      if (r > 0) 
      {
         usbstatus |= (1<<USB_SEND);
        
         cli(); 
         
         uint8_t code = 0x00;
         code = buffer[2];
         lcd_gotoxy(0,1);
         lcd_puthex(code);
         lcd_putc('0');
         lcd_puthex(buffer[4]);
         switch (code)
         {   
               
            case 0xE0: // Man: Alles stoppen
            {
               
            }break;
               
               
             
#pragma mark default
            default:
            {
               // Abschnittnummer bestimmen
               uint8_t indexh=buffer[18];
               uint8_t indexl=buffer[19];
               
               for (int i=8;i<16;i++)
               {
                  //sendbuffer[i]=POT_Array[i];

               }
               //sendbuffer[6]=buffer[16];
               
               //sendbuffer[8]= versionintl;
               //sendbuffer[9]= versioninth;
               usb_rawhid_send((void*)sendbuffer, 50); // nicht jedes Paket melden
               
             
               
                
            } // default
               
         } // switch code
         code=0;
         sei();
         
         
		} // r>0, neue Daten
      else
      {
         //OSZI_B_LO;
      }
      
      /**	End USB-routinen	***********************/
 		
		/* **** rx_buffer abfragen **************** */
		//rxdata=0;
		
#pragma mark Tasten		
		//	Daten von USB vorhanden
		 // rxdata
		
		//lcd_gotoxy(16,0);
        //lcd_putint(StepCounterA & 0x00FF);
		
		if (!(TASTENPIN & (1<<TASTE0))) // Taste 0
		{
			//lcd_gotoxy(8,1);
			//lcd_puts("T0 Down\0");
			
			if (!(TastenStatus & (1<<TASTE0))) //Taste 0 war noch nicht gedrueckt
			{
				//RingD2(5);
				TastenStatus |= (1<<TASTE0);
				
				Tastencount=0;
				//lcd_gotoxy(0,1);
				//lcd_puts("P0 \0");
				//lcd_putint(TastenStatus);
				//delay_ms(800);
			}
			else
			{
				
				
				Tastencount +=1;
				//lcd_gotoxy(7,1);
				//lcd_puts("TC \0");
				//lcd_putint(Tastencount);
				
				if (Tastencount >= Tastenprellen)
				{
               
					Tastencount=0;
               if (TastenStatus & (1<<TASTE0))
               {
                  //sendbuffer[0]=loopcount1;
                  //sendbuffer[1]=0xAB;
                  //usbstatus |= (1<<USB_SEND);
                  //lcd_gotoxy(2,1);
                  //lcd_putc('1');

                  //usb_rawhid_send((void*)sendbuffer, 50);
               }
					TastenStatus &= ~(1<<TASTE0);
               //lcd_gotoxy(3,1);
               //lcd_puts("ON \0");
               //delay_ms(400);
               //lcd_gotoxy(3,1);
              // lcd_puts("  \0");
               //lcd_putint(TastenStatus);
               
               
				}
			}//else
			
		}	// Taste 0
		
         
		
		if (!(TASTENPIN & (1<<TASTE1))) // Taste 1
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("T1 Down\0");
			
			if (! (TastenStatus & (1<<TASTE1))) //Taste 1 war nicht nicht gedrueckt
			{
				TastenStatus |= (1<<TASTE1);
				Tastencount=0;
				//lcd_gotoxy(3,1);
				//lcd_puts("P1 \0");
				//lcd_putint(Servoimpulsdauer);
				//delay_ms(800);
				
			}
			else
			{
				//lcd_gotoxy(3,1);
				//lcd_puts("       \0");
				
				Tastencount +=1;
				if (Tastencount >= Tastenprellen)
				{
					
					
					Tastencount=0;
					TastenStatus &= ~(1<<TASTE1);
					
				}
			}//	else
			
		} // Taste 1
		
		/* ******************** */
		//		initADC(TASTATURPIN);
		//		Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
		
		Tastenwert=0;
		
		//lcd_gotoxy(3,1);
		//lcd_putint(Tastenwert);
   
		//OSZI_B_HI;
      if (usbstatus & (1<< USB_SEND))
      {
         //lcd_gotoxy(10,1);
         //lcd_puthex(AbschnittCounter);
         //sendbuffer[3]= AbschnittCounter;
         //usb_rawhid_send((void*)sendbuffer, 50);
         //sendbuffer[0]=0;
         //sendbuffer[5]=0;
         //sendbuffer[6]=0;
         //usbstatus &= ~(1<< USB_SEND);
         
      }

	}//while
   //free (sendbuffer);

// return 0;
}

//
//  spi_adc.h
//  RC_PPM
//
//  Created by Ruedi Heimlicher on 26.07.2013.
//
//

#ifndef RC_PPM_spi_adc_h
#define RC_PPM_spi_adc_h



#endif
#ifndef __MCP3208_H_
#define __MCP3208_H_

#define SingleEnd    0x01        // set to Single-End A/D input
#define Differential 0x00        // set to Differential A/D input

#define PORT_SPI     PORTB       // SPI PORT
#define PIN_CS       PINB4       // SPI PIN for CS
#define PIN_MOSI     PINB5       // SPI PIN for MOSI
#define PIN_CLK      PINB7       // SPI PIN for CLK

#define DDR_SPI      DDRB        // DDR control SPI PORT
#define DD_SS        DDB4        // DDR control SS  (CS)  pin(PB4)
#define DD_MOSI      DDB5        // DDR control MOSI(DO)  pin(PB5)
#define DD_MISO      DDB6        // DDR control MISO(DI)  pin(PB6)
#define DD_SCK       DDB7        // DDR control SCK (CLK) pin(PB7)

#define delayCount   ((F_CPU)/1000000UL)

volatile unsigned char gReciveHighByte, gReciveLowByte; // global Variables

void MCP3208_spiDelay(unsigned int NOPcount);
void MCP3208_spiInit(void);
unsigned char MCP3208_spiWrite(char cData);
unsigned int MCP3208_spiRead(unsigned char AD_type,unsigned char ADchanel);

#endif

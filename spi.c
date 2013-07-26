/* 
 * Copyright (c) 2009 Andrew Smallbone <andrew@rocketnumbernine.com>
 *
 */

#include <spi.h>

#ifdef __cplusplus
extern "C"{
#endif


// von http://sites.google.com/site/qeewiki/books/avr-guide/spi

void spi_init(uint8_t mode, int dord, int interrupt, uint8_t clock)
{
  // specify pin directions for SPI pins on spi port
  if (clock == SPI_SLAVE) 
  { // if slave SS and SCK is input
    SPI_DDR &= ~(1<<SPI_MOSI_PIN); // input
    SPI_DDR |= (1<<SPI_MISO_PIN); // output
    SPI_DDR &= ~(1<<SPI_SS_PIN); // input
    SPI_DDR &= ~(1<<SPI_SCK_PIN);// input
  } 
  else 
  {
    SPI_DDR |= (1<<SPI_MOSI_PIN); // output
    SPI_DDR &= ~(1<<SPI_MISO_PIN); // input
    SPI_DDR |= (1<<SPI_SCK_PIN);// output
    SPI_DDR |= (1<<SPI_SS_PIN);// output
  }
  SPCR = ((interrupt ? 1 : 0)<<SPIE) // interrupt enabled
    | (1<<SPE) // enable SPI
    | (dord<<DORD) // LSB or MSB
    | (((clock != SPI_SLAVE) ? 1 : 0) <<MSTR) // Slave or Master
    | (((mode & 0x02) == 2) << CPOL) // clock timing mode CPOL
    | (((mode & 0x01)) << CPHA) // clock timing mode CPHA
    | (((clock & 0x02) == 2) << SPR1) // cpu clock divisor SPR1
    | ((clock & 0x01) << SPR0); // cpu clock divisor SPR0
  SPSR = (((clock & 0x04) == 4) << SPI2X); // clock divisor SPI2X
}

void disable_spi()
{
  SPCR = 0;
}

uint8_t send_spi(uint8_t out)
{
  SPDR = out;
  while (!(SPSR & (1<<SPIF)));
  return SPDR;
}

uint8_t received_from_spi(uint8_t data)
{
  SPDR = data;
  return SPDR;
}

//Oszi
#define OSZIPORT           PORTD
#define OSZIPORTDDR        DDRD
#define OSZIPORTPIN        PIND
#define OSZI_PULS_A        0
#define OSZI_PULS_B        1

#define OSZI_A_LO OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZI_A_HI OSZIPORT |= (1<<OSZI_PULS_A)


#define OSZI_B_LO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZI_B_HI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZI_B_TOGG OSZIPORT ^= (1<<OSZI_PULS_B)


#define LOOPLEDDDR          DDRF    //DDRD
#define LOOPLEDPORT         PORTF   //PORTD
#define LOOPLED             4       //6

#define TASTENDDR           DDRF
#define TASTENPORT          PORTF
#define TASTENPIN          PINF

#define TASTE0				0   //
#define TASTE1				1

// SPI

#define SPI_BUFSIZE 8

#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define WDTBIT              7

// CNC12
#define CMD_PORT            PORTD   //    PORTB
#define CMD_DDR             DDRD    //    DDRB
#define CMD_PIN             PIND    //    PINB

// ADC
// CNC12
#define ADC_PORT            PORTF   //    PORTF
#define ADC_DDR             DDRF    //    DDRF
#define ADC_PIN             PINF    //    PINF

#define ADC_START 0  // Start Messung Batteriespannung mit internem ADC

#define POT_START 0  //    Start Messung Potentiometer
#define ANZ_POT   6

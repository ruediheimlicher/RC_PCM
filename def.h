//Oszi
#define OSZIPORT           PORTA
#define OSZIPORTDDR        DDRA
#define OSZIPORTPIN        PINA
#define OSZI_PULS_A        0
#define OSZI_PULS_B        1
#define OSZI_PULS_C        2
#define OSZI_PULS_D        3


#define OSZI_A_LO OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZI_A_HI OSZIPORT |= (1<<OSZI_PULS_A)
#define OSZI_A_TOGG OSZIPORT ^= (1<<OSZI_PULS_A)


#define OSZI_B_LO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZI_B_HI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZI_B_TOGG OSZIPORT ^= (1<<OSZI_PULS_B)

#define OSZI_C_LO OSZIPORT &= ~(1<<OSZI_PULS_C)
#define OSZI_C_HI OSZIPORT |= (1<<OSZI_PULS_C)


#define OSZI_D_LO OSZIPORT &= ~(1<<OSZI_PULS_D)
#define OSZI_D_HI OSZIPORT |= (1<<OSZI_PULS_D)



#define LOOPLEDDDR          DDRD    
#define LOOPLEDPORT         PORTD   
#define LOOPLED             6       // fix verdrahtet

#define TASTENDDR           DDRD
#define TASTENPORT          PORTD
#define TASTENPIN           PIND

#define TASTE0				   0
#define TASTE1             1

// SPI

#define SPI_BUFSIZE 8

#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define WDTBIT              7

// CNC12
#define CMD_PORT            PORTD   
#define CMD_DDR             DDRD    
#define CMD_PIN             PIND


#define KANAL_PORT            PORTB   //    PORTB
#define KANAL_DDR             DDRB    //    DDRB


#define KANAL_PIN          4                             // Ausgang fuer Summensignal
#define KANAL_LO           KANAL_PORT &= ~(1<<KANAL_PIN)
#define KANAL_HI           KANAL_PORT |= (1<<KANAL_PIN)


// ADC
// CNC12
#define ADC_PORT            PORTF   //    PORTF
#define ADC_DDR             DDRF    //    DDRF
#define ADC_PIN             PINF    //    PINF


// Bit

#define ADC_START 0  //    Start Messung Batteriespannung mit internem ADC

#define POT_START 0  //    Start Messung Potentiometer

#define SPI_START 2  //    Start SPI auf diesem device

#define SPI_END   3  //    End SPI auf diesem device

#define POT_MITTE 7  //    Mittelwerte der Potentiometer speichern

#define ANZ_POT   6

#define POT_FAKTOR 1.20


#define EE_WREN   0
#define EE_WRITE  1
#define EE_READ   2

#define MASTER_EN_PORT            PORTD   //    PORTB
#define MASTER_EN_DDR             DDRD    //    DDRB

#define MASTER_EN_PIN            7 // Mit PinChange-Interrupt beim Sub
#define SUB_BUSY_PIN             6 // Eingang fuer busy-Meldung des Sub

#define MASTER_EN_BIT            0 // Master erlaubt SPI

#define MEM_EN_PORT              PORTE // CS fuer Memory-Zugriffe des Masters
#define MEM_EN_DDR               DDRE
#define MEM_EN_PIN               0


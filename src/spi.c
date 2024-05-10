#include <avr/io.h>
#include <util/delay.h>
#include "spi.h"

void SPI_init(void)
{
    /* Set MOSI and SCK output, all others input */
    SPI_DDR = (1 << SPI_SCK) | (1 << SPI_SS);
    SPI_DDR &= ~(1 << SPI_MISO);

    /* Set SS PIN on HIGH */
    SPI_PORT |= (1 << SPI_SS);

    /* Enable SPI, Master, set clock rate fck/16 */
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

uint8_t SPI_read(uint8_t data)
{
    SPDR = data;

    while(!(SPSR & (1 << SPIF)));

    return SPDR;
}

#ifndef SPI_H
#define SPI_H

#include <stdint.h>

/* SPI config */
#define SPI_PORT	PORTB
#define SPI_DDR 	DDRB
#define SPI_SS      PB2
#define SPI_MISO	PB4
#define SPI_SCK 	PB5

void SPI_init(void);
uint8_t SPI_read(uint8_t data);

#endif // SPI_H

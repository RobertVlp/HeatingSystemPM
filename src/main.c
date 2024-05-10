#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "spi.h"

#define PM_BAUD 9600

uint16_t read_temperature(void)
{
    uint16_t v = 0;

    SPI_PORT &= ~(1 << SPI_SS);
    _delay_us(10);

    v = (uint16_t) SPI_read(0x00) << 8;
    v |= SPI_read(0x00);

    SPI_PORT |= (1 << SPI_SS);

    /* Error, no thermocouple attached! */
    if (v & 0x4)
    {
        USART0_print("Thermocouple not attached ...\n");
        return 0;
    }

    v >>= 3;

    return v >> 2;
}

void ENC_init(void)
{
    DDRB &= ~((1 << PB0) | (1 << PB1) | (1 << PB3));
    PORTB |= (1 << PB0) | (1 << PB1) | (1 << PB3);

    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT3);
}

ISR(PCINT0_vect)
{
    USART0_print("Pin change ... \n");
}

int main(void)
{
    char output[30] = {0};
    uint16_t temp = 0;

    USART0_init(CALC_USART_UBRR(PM_BAUD));
    SPI_init();
    ENC_init();
    USART0_print("Starting ...\n");

    sei();

    while (1) {
        temp = read_temperature();

        sprintf(output, "Temp is: %u.\n", temp);
        USART0_print(output);

        _delay_ms(1000);
    }

    return 0;
}

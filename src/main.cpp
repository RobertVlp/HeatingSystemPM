#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <LiquidCrystal_I2C.h>

extern "C" {
    #include "usart.h"
    #include "spi.h"
} 

#define PM_BAUD 9600
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

volatile enum {
    SET_TARGET_TEMP,
    SET_P_K,
    SET_I_K,
    SET_D_K,
    PID_CONTROL
} system_state;

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
char output[30];
volatile uint8_t last_state, clk_state, dt_state, button_pressed;
uint16_t temp;
volatile uint16_t set_temp;

uint16_t read_temperature(void)
{
    uint16_t v = 0;

    /* Read the value from the sensor */
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

    /* Ignore the lower three bits (status bits) */
    v >>= 3;

    return v >> 2;
}

void ENC_init(void)
{
    /* Configure encoder data and clk pins as input pins, with pull up enabled */
    DDRB &= ~((1 << PB0) | (1 << PB1));
    PORTB |= (1 << PB0) | (1 << PB1);

    /* Enable interrupts for encoder data and clk pins */
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);

    /* Configure button pin, with pull up enabled */
    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);

    /* Enable interrupt for the button pin */
    EICRA |= (1 << ISC01);
    EIMSK |= (1 << INT0);
}

void TIM_init(void)
{
    /* Configure timer 2 in Fast PWM mode*/
    DDRD |= (1 << PD3);
    TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B |= (1 << CS21) | (1 << CS20);
    OCR2B = 127;
}

ISR(PCINT0_vect)
{
    clk_state = PINB & (1 << PB0);

    if (clk_state != last_state)
    {
        dt_state = PINB & (1 << PB1);

        if ((!dt_state) != clk_state)
            set_temp += 1;
        else
            set_temp -= 1;

        last_state = clk_state;
    }
}

ISR(INT0_vect)
{
    USART0_print("Button pressed, going to the next state ...\n");
    button_pressed = 1;

    switch (system_state)
    {
        case SET_TARGET_TEMP:
            /* Start PID Control and disable the encoder */
            system_state = PID_CONTROL;
            PCMSK0 &= ~((1 << PCINT0) | (1 << PCINT1));
            break;

        case PID_CONTROL:
            /* Stop PID Control, enable the encoder and go to the initial state */
            system_state = SET_TARGET_TEMP;
            PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);
            break;
        
        default:
            break;
    }
}

void setup()
{
    USART0_init(CALC_USART_UBRR(PM_BAUD));
    SPI_init();
    ENC_init();
    TIM_init();

    lcd.init();
    lcd.backlight();

    USART0_print("Starting ...\n");

    /* Save the initial state of the encoder */
    last_state = PINB & (1 << PB0);

    sei();
}

void loop()
{
    if (button_pressed)
    {
        lcd.clear();
        button_pressed = 0;
    }

    switch (system_state)
    {
        case SET_TARGET_TEMP:
            lcd.setCursor(3, 0);
            lcd.print("Set Target");

            lcd.setCursor(4, 1);
            lcd.print("Temp:");
            lcd.setCursor(9, 1);
            lcd.print(set_temp);

            break;
        
        case PID_CONTROL:
            temp = read_temperature();

            lcd.setCursor(0, 0);
            lcd.print("PID TEMP control");

            lcd.setCursor(0, 1);
            lcd.print("S:");

            lcd.setCursor(2, 1);
            lcd.print(set_temp);

            lcd.setCursor(9, 1);
            lcd.print("R:");

            lcd.setCursor(11, 1);
            lcd.print(temp);
            break;

        default:
            break;
    }
}

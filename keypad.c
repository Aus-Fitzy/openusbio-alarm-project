#include "keypad.h"

#include <avr/io.h>
#include <avr/interrupt.h>

void initKeypad(void)
{
    DDRB  = 0xF0;   //lower nibble is keypad in
    PORTB = 0x0F;   //pullup for keypad

    DDRD  = 0xF7; //NOTE: configure only PD3 (INT1) as input and activate only its pull up resistor
    PORTD = 0x08;

    int1_init(); //initialise registers to enable INT1
}

//user-defined function for the initialisation of the external interrupt INT1
void int1_init(void)
{
    cli(); //SREG = 0
    MCUCR = MCUCR | 0x0C; //ISC10 =1, ISC11 = 1 --> rising edge
    GIFR |= (1 << INTF1);
    GICR |= (1 << INT1);
    sei(); //SREG = 1
}

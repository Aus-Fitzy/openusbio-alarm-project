#include "LCD4.h"

#include <string.h>
#include <avr/io.h>
#include <util/delay.h>

void send4String(char * str)
{
    unsigned char i;
    unsigned char length = strlen(str);

    for(i=0; i< length; i++ ) //max of 256 chars
    {
        if(str[i] == '\n')
            goto4LCD(1,0);
        else
            send4Char(str[i]);
    }
}

void send4Cmd(unsigned char data)
{
    _delay_ms(1);

    //E,RS,RW low/ clear data lines
    //PORTC = 0xX0;

    //high order bits
    PORTC = (data & 0xF0);
    pulseE();

    //low order bits
    data <<= 4;
    PORTC = (data & 0xF0);
    pulseE();
}

void send4Char(unsigned char data)
{
    _delay_ms(1);

    //E,RW low, RS high / clear data lines
    //PORTC = 0xX4;

    PORTC = (data & 0xF0);
    PORTC |= 0x04;
    pulseE();

    data <<= 4;
    PORTC = (data & 0xF0);
    PORTC |= 0x04;
    pulseE();
}

void init4LCD()
{

    _delay_ms(100);

    //config IO
    DDRC = 0xFF;

    //INST: Function set [ 4bit ]
    PORTC = 0x20;
    //Once because it is still in 8 bit mode
    pulseE();

    //INST: Function set [ 4bit, 2 line, 5x7 char]
    send4Cmd(0b00101000);
    _delay_ms(100);

    //INST: Display on/off [  On, cursor, blink ]
    send4Cmd(0b00001111);
    _delay_ms(100);

    //INST : Clear display
    clearLCD();
}

void clearLCD(void)
{
    send4Cmd(0b00000001);
    _delay_ms(1);
}

void goto4LCD(unsigned char line, unsigned char pos)
{
    if(!line)
        send4Cmd(0x80 + pos);
    else
        send4Cmd(0xC0 + pos);
}

void pulseE()
{
    //E on pin PC0
    PORTC = PORTC | 1<<PC0; //on
    _delay_ms(1);
    PORTC = PORTC & ~(1<<PC0); //off
    _delay_ms(1);
}

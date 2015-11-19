#ifndef LCD4_H_INCLUDED
#define LCD4_H_INCLUDED

#define F_CPU 12000000UL

void send4Cmd(unsigned char);
void send4Char(unsigned char);
void send4String(char *);
void init4LCD(void);
void goto4LCD(unsigned char, unsigned char);
void pulseE(void);
void clearLCD(void);

#endif // LCD4_H_INCLUDED

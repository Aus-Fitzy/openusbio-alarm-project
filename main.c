/*
 */
#include <avr/io.h>
#include <avr/interrupt.h>

#include "LCD4.h"
#include "keypad.h"

#include <util/delay.h>
#include <stdlib.h>

#define ALERT_ON    1
#define ALERT_OFF   0

#define ARM_DISARM  0
#define ARM_AWAY    1
#define ARM_STAY    2
#define ARM_INSTANT 3

#define SENSOR_ARMED        1
#define SENSOR_DISARMED     0
#define SENSOR_ACTIVATED    1
#define SENSOR_DISACTIVATED 0

#define SENSOR_COUNT 3

//TODO calcualte threasholds for voltages
#define SENSOR_LOW  300
#define SENSOR_HIGH 700

#define TIMER_1MS_COUNT  188//12000000 / (64 * 1000)


#define GUI_LOGIN           1
#define GUI_INCORRECT       2
#define GUI_MENU            3
#define GUI_MODE_SELECTED   4
#define GUI_CODE            "1111"
#define GUI_HIDDEN          'X'

struct GUI
{
    char code[4];
    char code_input_count;
    volatile char last;
};

struct SystemState
{
    volatile char alert;
    volatile char arm;
    volatile char gui;
};

struct Sensor
{
    char mux[3];
    char active;
    char armed;
    unsigned int time_on;
    unsigned int level;
};

struct SystemState state;
struct Sensor sensors[SENSOR_COUNT];
struct GUI gui;

void init();
void init_timer();
void configure_state();
void poll_sensors();
void process_input();
void update_display();

void init_sensors();
unsigned int sensor_read(struct Sensor sensor);


int main(void)
{
    init();

    while(1)
    {
        configure_state();
        poll_sensors();
        process_input();
        update_display();
    }

    return 0;
}

void init()
{
    //system wide inturpts enabled
    sei();

    init4LCD();
    initKeypad();
    init_timer();
    init_sensors();

    state.gui = GUI_LOGIN;
}

void init_timer()
{
    //TODO test
    OCR0 = TIMER_1MS_COUNT;
    TIMSK |= (1 << OCIE0);  //Enable compare inturput
    TCCR0 |= (1 << WGM01) | //Clear Timer on Compare Match (CTC) mode
             (1 << CS01)  | //Clock Select 64 prescaler
             (1 << CS00);

}

void init_sensors()
{
    //TODO adjust pins

    //Sensor 0
    sensors[0].active = SENSOR_DISACTIVATED;
    sensors[0].armed = SENSOR_ARMED;
    sensors[0].mux[0] = 0;//PA6
    sensors[0].mux[1] = 1;
    sensors[0].mux[2] = 1;
    sensors[0].time_on = 0;

    //Sensor 1
    sensors[1].active = SENSOR_DISACTIVATED;
    sensors[1].armed = SENSOR_DISARMED;
    sensors[0].mux[0] = 0;//PA6
    sensors[0].mux[1] = 1;
    sensors[0].mux[2] = 1;
    sensors[1].time_on = 0;

    //Sensor 2
    sensors[2].active = SENSOR_DISACTIVATED;
    sensors[2].armed = SENSOR_DISARMED;
    sensors[0].mux[0] = 0;//PA6
    sensors[0].mux[1] = 1;
    sensors[0].mux[2] = 1;
    sensors[2].time_on = 0;

    //setup the ADC
    ADMUX   |=  (1 << REFS0);   //AVCC with external cap

    ADCSRA  |=  (1 << ADEN)  |   //ADC Enable
                (1 << ADPS2) | (1 << ADPS1); //64x prescaler

}

void configure_state()
{
    if(state.alert == ALERT_ON)
    {
        //TODO activate siren
        goto4LCD(1, 0);
        send4Char('A');
    }
    else
    {
        //TODO disactivate siren
        goto4LCD(1, 0);
        send4Char('D');
    }

    switch(state.arm)
    {
    case ARM_AWAY:
        //TODO
        break;

    case ARM_STAY:
        //TODO
        break;

    case ARM_INSTANT:
        //TODO
        break;

    case ARM_DISARM:
        //TODO
        break;

    }



}

void poll_sensors()
{
    short i;

    for(i = 0; i < SENSOR_COUNT; i++)
    {
        //TODO
        sensors[i].level = sensor_read(sensors[i]);
        if(sensors[i].level <= SENSOR_LOW || sensors[i].level >= SENSOR_HIGH )
            sensors[i].active = SENSOR_ACTIVATED;
        else
            sensors[i].active = SENSOR_DISACTIVATED;
    }
}

unsigned int sensor_read(struct Sensor sensor)
{
    unsigned int level;
    //TODO ADC from sensor pin to level;

    //select the channel based on the sensor setup
    ADMUX |= (sensor.mux[2] << MUX2 |
              sensor.mux[1] << MUX1 |
              sensor.mux[0] << MUX0);
    //start the conversion
    ADCSRA |= (1 << ADSC);
    while((ADCSRA & (1 << ADIF)) == 0)
        ;
    // wait for conversion to be done
    // 12Mhz / 64 prescale / 25 clock for conversion = 7.5Khz sample rate
    // or 133us per sample

    level  = ADCL; //read ADCL first
    level |= ADCH << 8;

    return level;
}

void process_input()
{
    switch(state.gui)
    {
    case GUI_LOGIN:
        if(gui.code_input_count >= 4)
        {
            //reset code and count
            gui.code_input_count = 0;
            if(gui.code[0] == GUI_CODE[0] &&
                    gui.code[1] == GUI_CODE[1] &&
                    gui.code[2] == GUI_CODE[2] &&
                    gui.code[3] == GUI_CODE[3])
            {
                //check for correct code, could you strcmp, but this is one less libary to include
                state.gui = GUI_MENU;
                clearLCD();
            }
            else
            {
                state.gui = GUI_INCORRECT;
                clearLCD();
            }
        }
        else
        {
            if(gui.last != 0)
            {
                gui.code[gui.code_input_count] = gui.last;
                gui.last = 0;
                gui.code_input_count++;
            }
        }
        break;
    case GUI_MENU:
        switch(gui.last)
        {
        case 'A':
            state.arm = ARM_DISARM;
            state.gui = GUI_MODE_SELECTED;
            break;
        case 'B':
            state.arm = ARM_STAY;
            state.gui = GUI_MODE_SELECTED;
            break;
        case 'C':
            state.arm = ARM_AWAY;
            state.gui = GUI_MODE_SELECTED;
            break;
        case 'D':
            state.arm = ARM_INSTANT;
            state.gui = GUI_MODE_SELECTED;
            break;
        }
        gui.last=0;
        break;

    case GUI_MODE_SELECTED:
        //break; fall through
    case GUI_INCORRECT:
        if(gui.last != 0)
        {
            gui.last = 0;
            state.gui = GUI_LOGIN;
            clearLCD();
        }
        break;
    }

}

void update_display()
{
    char buffer[8];
    short i = 0;

    //Over wrighting is better then clearing and then writing
    //printf is too slow to use
    switch(state.gui)
    {
    case GUI_LOGIN:

        goto4LCD(0, 0);
        send4String("Code: ");
        for(i = 0; i < gui.code_input_count; i++)
        {
            send4Char(GUI_HIDDEN);
        }
        break;
    case GUI_MENU:
        goto4LCD(0, 0);
        send4String("A: Disa  B: Stay\n"
                    "C: Away  D: Inst");
        break;
    case GUI_MODE_SELECTED:
        goto4LCD(0, 0);
        switch(state.arm)
        {
        case ARM_AWAY:
            send4String("  System Armed  \n"
                        "   AWAY MODE    ");
            break;
        case ARM_DISARM:
            send4String("System Disarmed \n"
                        "                ");
            break;
        case ARM_STAY:
            send4String("  System Armed  \n"
                        "   STAY MODE    ");
            break;
        case ARM_INSTANT:
            send4String("  System Armed  \n"
                        "  INSTANT MODE  ");
            break;
        }
        break;
    case GUI_INCORRECT:
        goto4LCD(0, 0);
        send4String(" Incorrect Code \n"
                    "   Try Again    ");
        break;
    }

    itoa(sensors[0].level, buffer, 10);
    goto4LCD(1, 10);
    send4String(buffer);
}

ISR(TIMER0_COMP_vect)
{
    short i;
    //Timer/Counter0 Compare Match
    //called once every 1ms
    for(i = 0; i < SENSOR_COUNT; i++)
    {
        if(sensors[i].armed == SENSOR_ARMED &&
                sensors[i].active == SENSOR_ACTIVATED)
        {
            if(sensors[i].time_on > 1000)
            {
                state.alert = ALERT_ON;
            }
            else
            {
                sensors[i].time_on++;
            }
        }
        else
        {
            sensors[i].time_on = 0;
        }
    }

}
ISR(INT1_vect)
{
    //static keyword: keep scoped to the function, but not re-allocated every call
    static unsigned char keymap[] = {'A', '3', '2', '1', 'B', '6', '5', '4', 'C', '9', '8', '7', 'D', '#', '0', '*'};

    unsigned char keypad;

    keypad = PINB & KEYPAD_DATA; //lower nibble
    gui.last = keymap[keypad];
}

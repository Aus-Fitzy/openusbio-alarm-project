/*
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include "LCD4.h"
#include "keypad.h"

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

struct SystemState
{
    char alert;
    char arm;
};

struct Sensor{
    char pin;
    char active;
    char armed;
};

struct SystemState state;
struct Sensor sensors[SENSOR_COUNT];

void init();
void init_timer();
void configure_state();
void poll_sensors();

void init_sensors();
unsigned int sensor_read(struct Sensor sensor);


int main(void)
{
    init();

    while(1)
    {
        configure_state();
        poll_sensors();
    }

    return 0;
}

void init()
{
    init4LCD();
    init_timer();
    init_sensors();
}

void init_timer()
{
    //TODO
}

void init_sensors()
{
    //TODO
}

void configure_state()
{
    if(state.alert == ALERT_ON)
    {
        //TODO activate siren
    }
    else
    {
        //TODO disactivate siren
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
    unsigned int sensor_level;

    for(i=0;i<SENSOR_COUNT;i++){
        //TODO
        sensor_level = sensor_read(sensors[i]);
        if(sensor_level <= SENSOR_LOW || sensor_level >= SENSOR_HIGH )
            sensors[i].active = SENSOR_ACTIVATED;
        else
            sensors[i].active = SENSOR_DISACTIVATED;
    }
}

unsigned int sensor_read(struct Sensor sensor){
    unsigned int level;
    //TODO ADC from sensor pin to level;
    level = 0;

    return level;
}


//Interupts
ISR(TIMER0_OVF_vect){
    //TODO
    //Timer/Counter0 Overflow
}
ISR(TIMER0_COMP_vect){
    //TODO
    //Timer/Counter0 Compare Match
}
ISR(INT1_vect){
    //TODO recode to make it useful for the alarm program

    //static keyword: keep scoped to the function, but not re-allocated every call
    static unsigned char counter = 0;
    static unsigned char keymap[] = {'A','3','2','1','B','6','5','4','C','9','8','7','D','#','0','*'};

    unsigned char keypad;

    if(counter == 16)
    {
        goto4LCD(1,0);
    }
    else if(counter == 32)
    {
        clearLCD();
        counter = 0;
    }

    keypad = PINB & KEYPAD_DATA; //lower nibble
    send4Char(keymap[keypad]);
    counter++;
}

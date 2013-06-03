/*
Author : Jonathan Whitten
Date : 24 AUG 2012
Title : Motor Driver
Description : The motor driver program executes a simple up-down linear actuator, which should never be simultaneously driven in the up and down position. It is by this necessity that I am writing a program that supports a short delay during the interrupts preventing the movement of any motors.

*/
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define F_CPU 1000000

#define TIMER_MAX_VALUE 65535
#define BS(bitnum) (1<<bitnum)
#define NOT(val) (val^0xFF)

#define data 0
#define shift 1
#define latch 2
#define enable1 3
#define enable2 4
#define reset 5

uint16_t matrix[8][8];
uint8_t inttimer;
uint16_t delay;

uint8_t posx;
uint8_t posy;
uint8_t dirx;
uint8_t diry;
uint16_t wallbrightness;

/* Smileyface map
uint8_t matrix[8][8] = {
  {0, 0,  255,  255,  255,  255,  0,  0},
  {0, 255,  0,  0,  0,  0,  255,  0},
  {255, 0,  255,  0,  0,  255,  0,  255},
  {255, 0,  0,  0,  0,  0,  0,  255},
  {255, 0,  255,  0,  0,  255,  0,  255},
  {255, 0,  0,  255,  255,  0,  0,  255},
  {0, 255,  0,  0,  0,  0,  255,  0},
  {0, 0,  255,  255,  255,  255,  0,  0}};
*/



void erase_screen(uint16_t val)
{
  int i=0,j =0;
  for(i=0;i<8;i++)
  {
    for(j=0;j<8;j++)
    {
      matrix[i][j] = val;
    }
  }
}//end erase screen

void draw_outline()
{
    int i,j;
    for(i=0;i<8;i++)
    {
      for(j=0;j<8;j++)
      {
        if(i==7 || i==0 || j==7 || j==0)
          matrix[j][i] = wallbrightness;
      }
    }
}

void configure_timer0()
{
  cli();
  TCCR0A = 0;
  TCCR0B = 0;
  TIMSK0 = 0; //Clear registers
  TCNT0 = 0; //reset timer

  OCR0A = 0xff;
  TIMSK0 |= BS(OCIE1A); //enable COMPA1 ISR
  //TCCR0A |= BS(WGM11);//not needed since not toggling anything
  //TCCR0B |= BS(CS12)|BS(CS10); //1024 prescaler, must be configured
  TCCR0B |= BS(CS02)|BS(CS00)|BS(WGM02);
  sei();
}

void configure_timer1()
{
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0; //Clear registers
  TCNT1 = 0; //reset timer

  OCR1A = 0xffff;
  TIMSK1 |= BS(OCIE1A); //enable COMPA1 ISR
  //TCCR0A |= BS(WGM11);//not needed since not toggling anything
  //TCCR0B |= BS(CS12)|BS(CS10); //1024 prescaler, must be configured
  TCCR1B |= BS(CS11)|BS(WGM12);
  sei();
}

void setup()
{
  erase_screen(0x0000);

  posx = 1;
  posy = 0;
  dirx = 0;
  diry = 0;
  wallbrightness = 0x0000;
  delay = 0;

  //configure timer configuration
  configure_timer0();
  configure_timer1();
}


//Compare on match interrupt
ISR(TIMER0_COMPA_vect)
{
    if(delay == 0x02)
    {
        wallbrightness += 0x07;
        wallbrightness += 0x0500;
        delay = 0;
    }else
        delay++;
}

ISR(TIMER1_COMPA_vect)
{

  erase_screen(0x0000);
  //update x

  if(dirx == 1)
  {
    if(posx == 7)
    {
      dirx = 0;
      posx -= 2;
    }else posx += 1;
  }
  else{
    if(posx == 0)
    {
      dirx = 1;
      posx += 2;
    }
    else posx -= 1;
  }

  if(diry == 1)
  {
    if(posy == 7)
    {
      diry = 0;
      posy -= 1;
    }else posy += 1;
  }
  else{
    if(posy == 0)
    {
      diry = 1;
      posy += 1;
    }
    else posy -= 1;
  }

  draw_outline();
  matrix[posy][posx] = 0x00ff;
}

/* Main */

int main(void)
{

    setup();

    //Configure image
    //topline (y,x)

    /*
    //write green elsewhere
    for(i=0;i<8;i++)
      for(j=0;j<8;j++)
      {
        if(matrix[i][j] != 255)
          matrix[i][j] = (0xff<<8);
      }
      */

    //Configure timers first
    //timer = TIMER_MAX_VALUE;

    //Configure all outputs
    DDRB = 0xff; //SPI out
    DDRD = 0xff; //rows
    DDRC = 0xff; //shifter

    //Clear ports
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;

    //Initailize variables
    uint8_t y=0;
    PORTD = 0x00; //clear rows
    uint8_t x=0;
    inttimer=0x00;

    while(1)
    {
        PORTC &= NOT(BS(reset));//clear everything
        PORTC = 0x00|BS(reset);

        //Green pixels
        for(x=0;x<8;x++)
        {
            if( (((matrix[y][7-x]))&0x00ff) >= inttimer)
            {
                PORTC |= BS(data);
            }else{
                PORTC &= NOT(BS(data));
            }

            PORTC |= BS(shift);
            PORTC &= NOT(BS(shift));
        }

          //Red pixels
        for(x=0;x<8;x++)
        {
            if( (((matrix[y][7-x])>>8)) >= inttimer)
            {
                PORTC |= BS(data);
            }else{
                PORTC &= NOT(BS(data));
            }

            PORTC |= BS(shift);
            PORTC &= NOT(BS(shift));
        }//end for

        PORTD = 0x00; //clear display
        PORTC |= BS(latch);
        PORTC &= NOT(BS(latch));

        //Output row data
        PORTD = BS(y);
        y++;
        y = y%8;


        //Increment intensity timer
        
        if(inttimer==255)
            inttimer=1;
        else
            inttimer++;
         

        //delay
        //_delay_ms(100); //for diangostic purposes
             
    }//END MAIN WHILE

}//EOF


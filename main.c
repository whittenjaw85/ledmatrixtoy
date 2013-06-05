/*
Author : Jonathan Whitten
Date : 24 AUG 2012
Title : Motor Driver
Description : The motor driver program executes a simple up-down linear actuator, which should never be simultaneously driven in the up and down position. It is by this necessity that I am writing a program that supports a short delay during the interrupts preventing the movement of any motors.

*/
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <avr/power.h>
#include <avr/interrupt.h>

#define F_CPU 1000000
#include <util/delay.h>

#define TIMER_MAX_VALUE 65535
#define BS(bitnum) (1<<bitnum)
#define NOT(val) (val^0xFF)

#define data 0
#define shift 1
#define latch 2
#define enable1 3
#define enable2 4
#define reset 5

enum{
    BOUNCER = 0,
    SMILEY,
    SINE,
    TREE,
    SINECIRCLE
};

//Universal variables
uint8_t state;
uint16_t statetimer;
uint16_t matrix[8][8];
uint8_t inttimer;
uint16_t maxtimer;
uint16_t delay;

//Bouncer variables
uint8_t posx;
uint8_t posy;
uint8_t dirx;
uint8_t diry;
uint16_t wallbrightness;

//sinecircle variables
uint8_t counter;

//tree variables
uint8_t apples[5];

//Smileyface map
uint8_t smileymap[8] = {
    0b00111100,
    0b01000010,
    0b10100101,
    0b10000001,
    0b10100101,
    0b10011001,
    0b01000010,
    0b00111100
};

//Sinemap
uint32_t sinemap[8] = {
    0b1111000000000000000000000000,
    0b0000110000000000000000000011,
    0b0000001100000000000000001100,
    0b0000000010000000000000010000,
    0b0000000001000000000000100000,
    0b0000000000110000000011000000,
    0b0000000000001100001100000000,
    0b0000000000000011110000000000
};


uint8_t sinecirclemap[11][8] = {
{
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000
},
{
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000
},
{
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000
},
{
    0b00000000,
    0b00000000,
    0b00000000,
    0b00010000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000

},
{
    0b00000000,
    0b00000000,
    0b00000000,
    0b00011000,
    0b00011000,
    0b00000000,
    0b00000000,
    0b00000000
},
{
    0b00000000,
    0b00000000,
    0b00011000,
    0b00100100,
    0b00100100,
    0b00011000,
    0b00000000,
    0b00000000
},
{
    0b00000000,
    0b00100100,
    0b01100110,
    0b00000000,
    0b00000000,
    0b01100110,
    0b00100100,
    0b00000000
},
{
    0b00000000,
    0b00011000,
    0b00000000,
    0b01000010,
    0b01000010,
    0b00000000,
    0b00011000,
    0b00000000
},
{
    0b00100100,
    0b01000010,
    0b10000001,
    0b00000000,
    0b00000000,
    0b10000001,
    0b01000010,
    0b00100100
},
{
    0b11000011,
    0b10000001,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b10000001,
    0b11000011
},
{
    0b00011000,
    0b00000000,
    0b00000000,
    0b10000001,
    0b10000001,
    0b00000000,
    0b00000000,
    0b00011000
}};

uint8_t treemap[3][8] = {
//0 greenmap
{
    0b00111000,
    0b00111110,
    0b01111111,
    0b11111111,
    0b01111110,
    0b00000000,
    0b00000000,
    0b00000000
},
//1 brownmap
{
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00011000,
    0b00011000,
    0b00111100
},
//2 applemap (static)
{
    0b00000000,
    0b00001000,
    0b00100010,
    0b01000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000
}
};

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

void copy_image(uint8_t img[8], uint16_t color)
{
    uint8_t i,j;
    for(i=0;i<8;i++)
        for(j=0;j<8;j++)
        {
            if( ((img[j]>>i)&0x1) == 0x1)
                matrix[j][i] = color;
        }
}

void draw_circle(uint8_t radius, uint16_t color)
{
    uint8_t i;
    for(i=0;i<4;i++)
    {
        //copy_image(sinecirclemap[ (i+radius)%11 ], colors[i]); 
        copy_image(sinecirclemap[ (i+radius)%11 ], color-i*0x3333); 
    }
}

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

void update_sinecircle(){
    if(counter==10) counter = 0;
    else counter += 1; 
    draw_circle(counter, wallbrightness);
}//end update sinecircle

void update_sine(){    
    uint8_t i,j,mask;
    for(i=0;i<8;i++)
        for(j=0;j<8;j++)
        {
            mask = (counter+i)%28;
            if( ((sinemap[j]>>(mask))&0x01) == 0x01)
                matrix[j][i] = wallbrightness;
            //if(sinemap[j][(i+sinecounter)%28] == 1)
            //    matrix[j][i] = wallbrightness;
        }
    counter = (counter+1)%28;  
}


void update_smiley(){
    copy_image(smileymap, wallbrightness);
}

void update_tree(){
    copy_image(treemap, 0x00ff); //leaves
    copy_image(treemap[1], 0x1010); //trunk
    //copy_image(treemap[2], 0xaa10); //bland apples
    copy_image(treemap[2], wallbrightness);
}

void update_bouncer(){
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

void configure_sine(){
    cli();
    OCR1A = 0x02ff;
    TCNT1 = 0;
    sei();
    maxtimer = 0x0200;
}

void configure_sinecircle(){
    cli();
    OCR1A = 0x1222;
    TCNT1 = 0;
    sei();
    maxtimer = 0x00ff;
}

void configure_timer0()
{
  cli();
  TCCR0A = 0;
  TCCR0B = 0;
  TIMSK0 = 0; //Clear registers
  TCNT0 = 0; //reset timer

  OCR0A = 0xff; // timer interrupt counter
  TIMSK0 |= BS(OCIE1A); //enable COMPA1 ISR
  TCCR0B |= BS(CS02)|BS(CS00)|BS(WGM02); //1024 clk divider CTC 
  sei();
}

void configure_timer1()
{
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0; //Clear registers
  TCNT1 = 0; //reset timer

  OCR1A = 0x0fff;
  TIMSK1 |= BS(OCIE1A); //enable COMPA1 ISR
  TCCR1B |= BS(CS12)|BS(WGM12); //256 presclaer, CTC configured
  sei();
}

void setup()
{
    //initialize important variables
    erase_screen(0x0000);
    maxtimer = 0x00ff;
    state = TREE;
    statetimer = 0;
    counter = 0;

    //bouncer variables
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
    }else delay++;
}

ISR(TIMER1_COMPA_vect)
{
    erase_screen(0x0000);
    //update x
    switch(state){
        case BOUNCER:
            update_bouncer();
            break;
        case SMILEY:
            update_smiley();
            break;
        case SINE:
            update_sine();
            break;
        case SINECIRCLE:
            update_sinecircle();
            break;
        case TREE:
            update_tree();
            break;
    }//end switch

    statetimer += 1;
    if(statetimer==maxtimer)
    {
        statetimer=0;
        counter=0;
        state = (state+1)%(SINECIRCLE+1);
        switch(state){
            case(SINE):
                configure_sine();
                break;
            case(SINECIRCLE):
                configure_sinecircle();
                break;
            default:
                OCR1A = 0x0fff;
                maxtimer = 0x00ff;
        }
    }
}//end ISR COMPA TIMER1

/* Main */

int main(void)
{

    setup();

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
            if( (((matrix[y][x]))&0x00ff) >= inttimer)
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
            if( (((matrix[y][x])>>8)) >= inttimer)
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


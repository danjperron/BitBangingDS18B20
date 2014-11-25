// modified version to read DS18B20 in bit banging
//
//  24 May 2014
//  Daniel Perron
//
// Use At your own risk

// 7 August 2014
// Add arg parameter to select the GPIO pin



// Add the priority function from Adafruit DHT22 c code
//  August 3 , 2014
// Priority added 
// code base on Adafruit DHT22  source code  for  
// set_max_priority and set_default_priority
// Copyright (c) 2014 Adafruit Industries
// Author: Tony DiCola
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALI



//
//  How to access GPIO registers from C-code on the Raspberry-Pi
//  Example program
//  15-January-2012
//  Dom and Gert
//  Revised: 15-Feb-2013


// Access from ARM Running Linux

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */


#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <sched.h>

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
void *gpio_map;

// I/O access
volatile unsigned *gpio;


// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

#define GPIO_READ(g)  (*(gpio + 13) &= (1<<(g)))


#define DS18B20_SKIP_ROM       0xCC
#define DS18B20_CONVERT_T       0x44
#define DS18B20_READ_SCRATCHPAD         0xBE
#define DS18B20_WRITE_SCRATCHPAD        0x4E
#define DS18B20_COPY_SCRATCHPAD         0x48

unsigned char ScratchPad[9];
double  temperature;
int   resolution;

void setup_io();


unsigned int DS_PIN=4;

void set_max_priority(void) {
  struct sched_param sched;
  memset(&sched, 0, sizeof(sched));
  // Use FIFO scheduler with highest priority for the lowest chance of the kernel context switching.
  sched.sched_priority = sched_get_priority_max(SCHED_FIFO);
  sched_setscheduler(0, SCHED_FIFO, &sched);
}

void set_default_priority(void) {
  struct sched_param sched;
  memset(&sched, 0, sizeof(sched));
  // Go back to default scheduler with default 0 priority.
  sched.sched_priority = 0;
  sched_setscheduler(0, SCHED_OTHER, &sched);
}


int  DoReset(void)
{
 int loop;

   INP_GPIO(DS_PIN);


   usleep(1000);

   INP_GPIO(DS_PIN);
   OUT_GPIO(DS_PIN);
   
   // pin low for 480 us
   GPIO_CLR=1<<DS_PIN;
   usleep(480);
   INP_GPIO(DS_PIN);
   usleep(60);
   if(GPIO_READ(DS_PIN)==0)
   {
     usleep(380);
     return 1;
   }
 
  return 0;
}

#define DELAY1US  smalldelay();

void  smalldelay(void)
{
  int loop2;
   for(loop2=0;loop2<100;loop2++);
}



void WriteByte(unsigned char value)
{
  unsigned char Mask=1;
  int loop;

   for(loop=0;loop<8;loop++)
     {
       INP_GPIO(DS_PIN);
       OUT_GPIO(DS_PIN);
       GPIO_CLR= 1 <<DS_PIN;

       if((value & Mask)!=0)
        {
           DELAY1US
            INP_GPIO(DS_PIN);
           usleep(60);

        }
        else
        {
           usleep(60);
           INP_GPIO(DS_PIN);
           usleep(1);
        }
      Mask*=2;
      usleep(60);
    }


   usleep(100);
}


unsigned char ReadBit(void)
{
   INP_GPIO(DS_PIN);
   OUT_GPIO(DS_PIN);
   // PIN LOW
   GPIO_CLR= 1 << DS_PIN;
   DELAY1US
   // set INPUT
   INP_GPIO(DS_PIN);
   DELAY1US
   DELAY1US
   DELAY1US
   if(GPIO_READ(DS_PIN)!=0)
     return 1;
   return 0;
}

unsigned char ReadByte(void)
{

   unsigned char Mask=1;
   int loop;
   unsigned  char data=0;

  int loop2;


   for(loop=0;loop<8;loop++)
     {
       //  set output
       INP_GPIO(DS_PIN);
       OUT_GPIO(DS_PIN);
       //  PIN LOW
       GPIO_CLR= 1<<DS_PIN;
       DELAY1US
       //  set input
       INP_GPIO(DS_PIN);
       // Wait  2 us
       DELAY1US
       DELAY1US
       DELAY1US
       if(GPIO_READ(DS_PIN)!=0)
       data |= Mask;
       Mask*=2;
       usleep(60);
      }

    return data;
}



int ReadScratchPad(void)
{
   int loop;

    if(DoReset())
     {
       WriteByte(DS18B20_SKIP_ROM);
       WriteByte(DS18B20_READ_SCRATCHPAD);
       for(loop=0;loop<9;loop++)
         {
          ScratchPad[loop]=ReadByte();
        }
    return 1;
  }
  return 0;
}

unsigned char  CalcCRC(unsigned char * data, unsigned char  byteSize)
{
   unsigned char  shift_register = 0;
   unsigned char  loop,loop2;
   char  DataByte;

   for(loop = 0; loop < byteSize; loop++)
   {
      DataByte = *(data + loop);
      for(loop2 = 0; loop2 < 8; loop2++)
      {
         if((shift_register ^ DataByte)& 1)
         {
            shift_register = shift_register >> 1;
            shift_register ^=  0x8C;
         }
         else
            shift_register = shift_register >> 1;
         DataByte = DataByte >> 1;
      }
   }
   return shift_register;
}

int ReadSensor(void)
{
  int maxloop;
  int RetryCount;
  int loop;
  unsigned char  CRCByte;
  union {
   short SHORT;
   unsigned char CHAR[2];
  }IntTemp;


  time_t t = time(NULL);
  struct tm tm = *localtime(&t);

  temperature=-9999.9;


  for(RetryCount=0;RetryCount<5;RetryCount++)
  {



  if(!DoReset()) continue;

  // start a conversion
  WriteByte(DS18B20_SKIP_ROM);
  WriteByte(DS18B20_CONVERT_T);


  maxloop=0;
  // wait until ready
   while(!ReadBit())
   {
    putchar('.');
     maxloop++;
    if(maxloop>100000) break;
   }

  if(maxloop>100000) continue;


  if(!ReadScratchPad()) continue;

     for(loop=0;loop<9;loop++)
       printf("%02X ",ScratchPad[loop]);
     printf("\n");fflush(stdout);

  // OK Check sum Check;
  CRCByte= CalcCRC(ScratchPad,8);

  if(CRCByte!=ScratchPad[8]) continue;

  //Check Resolution
   resolution=0;
   switch(ScratchPad[4])
   {

     case  0x1f: resolution=9;break;
     case  0x3f: resolution=10;break;
     case  0x5f: resolution=11;break;
     case  0x7f: resolution=12;break;
   }

   if(resolution==0) continue;
    // Read Temperature

    IntTemp.CHAR[0]=ScratchPad[0];
    IntTemp.CHAR[1]=ScratchPad[1];


    temperature =  0.0625 * (double) IntTemp.SHORT;

      printf("%02d bits  Temperature: %6.2f +/- %f Celsius\n", resolution ,temperature, 0.0625 * (double)  (1<<(12 - resolution)));

    return 1;
   }
  return 0;
}




void WriteScratchPad(unsigned char TH, unsigned char TL, unsigned char config)
{
int loop;

    // First reset device

    DoReset();

    usleep(1000);
    // Skip ROM command
     WriteByte(DS18B20_SKIP_ROM);


     // Write Scratch pad

    WriteByte(DS18B20_WRITE_SCRATCHPAD);

    // Write TH

    WriteByte(TH);

    // Write TL

    WriteByte(TL);

    // Write config

    WriteByte(config);
}

void  CopyScratchPad(void)
{

   // Reset device
    DoReset();
    usleep(1000);

   // Skip ROM Command

    WriteByte(DS18B20_SKIP_ROM);

   //  copy scratch pad

    WriteByte(DS18B20_COPY_SCRATCHPAD);
    usleep(100000);
}


int main(int argc, char **argv)
{
  int loop;
  int config;
  // Set up gpi pointer for direct register access
  
  if(argc==2)
{
 DS_PIN = atoi(argv[1]);
}

 printf("GPIO %d\n",DS_PIN);

 if((DS_PIN < 1) || (DS_PIN>32))
  {
  printf("Invalid GPIO PIN\n");
  return -1;
  }

  setup_io();
  set_max_priority();

  if(ReadSensor())
    {
     printf("DS18B20 Resolution (9,10,11 or 12) ?");fflush(stdout);

    config=0;
  set_default_priority();

    if(scanf("%d",&resolution)==1)
      {
        switch(resolution)
         {
           case 9:  config=0x1f;break;
           case 10: config=0x3f;break;
           case 11: config=0x5f;break;
           case 12: config=0x7f;break;
         }
      }

    if(config==0)
         printf("Invalid Value! Nothing done.\n");
    else
    {
      printf("Try to set %d bits  config=%2X\n",resolution,config);
      usleep(1000);
        set_max_priority();
 
     WriteScratchPad(ScratchPad[2],ScratchPad[3],config);
      usleep(1000);
      CopyScratchPad();

    }

  }
  set_default_priority();

  return 0;

} // main


//
// Set up a memory regions to access GPIO
//
void setup_io()
{
   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      exit(-1);
   }

   /* mmap GPIO */
   gpio_map = mmap(
      NULL,             //Any adddress in our space will do
      BLOCK_SIZE,       //Map length
      PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
      MAP_SHARED,       //Shared with other processes
      mem_fd,           //File to map
      GPIO_BASE         //Offset to GPIO peripheral
   );

   close(mem_fd); //No need to keep mem_fd open after mmap

   if (gpio_map == MAP_FAILED) {
      printf("mmap error %d\n", (int)gpio_map);//errno also set!
      exit(-1);
   }

   // Always use volatile pointer!
   gpio = (volatile unsigned *)gpio_map;


} // setup_io

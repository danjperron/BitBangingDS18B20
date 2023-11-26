// nov 26, 2023
// usage of gpiod library to deal with Raspberry  Pi5


// April 2018
//  changed /dev/mem to /dev/gpiomem
// now sudo  is not needed anymore

// modified version to check BCM physical address
// February 1, 2016
// check  "/proc/device-tree/soc/ranges" for BCM address
//
// add timer delay for us resolution from Gladkikh Artem
// DelayMicrosecondsNoSleep


// update to  run on arm64 bits
//
// 31 march 2021
// Daniel Perron


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



#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <sched.h>
#include <string.h>
#include <gpiod.h>

struct gpiod_chip  *gpiochip;
struct gpiod_line  *gpioline;

int intvalue;

bool  init_gpiod(int Pin)
{
  gpiochip = gpiod_chip_open_by_name("gpiochip4");

  if(gpiochip == NULL)
      gpiochip = gpiod_chip_open_by_name("gpiochip0");

  if(gpiochip == NULL)
      {
           printf("unable to open GPIO\n");
           return false;
      }


  gpioline = gpiod_chip_get_line(gpiochip,Pin);

  if(gpioline == NULL)
      {
          printf("unable to open specific GPIO20\n");
          return false;
      }

 gpiod_line_request_output_flags(gpioline,"DS18B20",GPIOD_LINE_REQUEST_FLAG_OPEN_DRAIN|GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP,1);
 return true;
}


#define GPIO_READ gpiod_line_get_value(gpioline)
#define GPIO_SET  gpiod_line_set_value(gpioline,1);
#define GPIO_CLR  gpiod_line_set_value(gpioline,0);
#define DELAY1US  DelayMicrosecondsNoSleep(1);

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



void DelayMicrosecondsNoSleep (int delay_us)
{
   long int start_time;
   long int time_difference;
   struct timespec gettime_now;

   clock_gettime(CLOCK_REALTIME, &gettime_now);
   start_time = gettime_now.tv_nsec;      //Get nS value
   while (1)
   {
      clock_gettime(CLOCK_REALTIME, &gettime_now);
      time_difference = gettime_now.tv_nsec - start_time;
      if (time_difference < 0)
         time_difference += 1000000000;            //(Rolls over every 1 second)
      if (time_difference > (delay_us * 1000))      //Delay for # nS
         break;
   }
}




int  DoReset(void)
{
 int loop;

   GPIO_SET
   DelayMicrosecondsNoSleep(10);
   GPIO_CLR
   DelayMicrosecondsNoSleep(480);
   GPIO_SET
   DelayMicrosecondsNoSleep(60);
   if(GPIO_READ==0)
   {
     DelayMicrosecondsNoSleep(420);
     return 1;
   }
  return 0;
}





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
       GPIO_CLR

       if((value & Mask)!=0)
        {
           DELAY1US
           GPIO_SET
           DelayMicrosecondsNoSleep(60);

        }
        else
        {
           DelayMicrosecondsNoSleep(60);
           GPIO_SET;
           DelayMicrosecondsNoSleep(1);
        }
      Mask*=2;
      DelayMicrosecondsNoSleep(60);
    }
   usleep(100);
}

void WriteBit(unsigned char value)
{
   GPIO_CLR
   if(value)
    {
      DELAY1US
      GPIO_SET
      DelayMicrosecondsNoSleep(60);
    }
   else
    {
      DelayMicrosecondsNoSleep(60);
      GPIO_SET
      DelayMicrosecondsNoSleep(1);
     }
   DelayMicrosecondsNoSleep(60);
}





unsigned char ReadBit(void)
{
   unsigned char rvalue=0;
   // PIN LOW
   GPIO_CLR
   DELAY1US
   // set INPUT
   GPIO_SET
   DelayMicrosecondsNoSleep(2);
   if(GPIO_READ!=0)
    rvalue=1;
   DelayMicrosecondsNoSleep(60);
   return rvalue;
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
       GPIO_CLR
       //  PIN LOW
       DELAY1US
       //  set input
       GPIO_SET
       // Wait  2 us
       DelayMicrosecondsNoSleep(2);
       if(GPIO_READ!=0)
       data |= Mask;
       Mask*=2;
       DelayMicrosecondsNoSleep(60);
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

    DelayMicrosecondsNoSleep(1000);
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
    DelayMicrosecondsNoSleep(1000);

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

  init_gpiod(DS_PIN);
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

  gpiod_line_release(gpioline);
  gpiod_chip_close(gpiochip);
  return 0;

} // main

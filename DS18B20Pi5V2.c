// Read DS18B20 sensor using one DS18B20 sensor per GPIO PIN using
// parallel method to speed up process
// Copyright (c) 29 June 2014  Daniel Perron
//
// Need to install pull-up resistor

// It's need to be compiled like
// gcc -l gpiod -o DS18B20V2 DS18B20V2.c

// 3-  Reset DS18B20
// 4-  send SKIP_ROM command
// 5-  send START_ACQUISITION
// 6-  wait until acquisition is done
// 7-  Reset DS18B20
// 8-  Send SKIP_ROM command
// 9-  Send read register command
// 10-  Collect GPIO word into table
// 11-  Decode individual bit to get sensor temperature
// 12- End

// November 2023
// use lib gpiod

// ---- obsolete ----
// modified version to check BCM physical address
// February 1, 2016
// check  "/proc/device-tree/soc/ranges" for BCM address
//
// add timer delay for us resolution from Gladkikh Artem
// DelayMicrosecondsNoSleep


// --- obsolete ---
//  August 3 , 2014
// Priority added
// code base on Adafruit DHT22  source code  for
// set_max_priority and set_default_priority
// Copyright (c) 2014 Adafruit Industries
// Author: Tony DiCola

// August 5, 2014
// - Add retried on fail
// - Add bit resolution checking at the start to set the correct acquisition waiting time
// - Add loop scanning.


// copyrigth  declaration

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
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <gpiod.h>


// define gpiod structure
struct gpiod_line_bulk gpiolines;
struct gpiod_chip  *gpiochip;
struct gpiod_line  *gpioline;

// define gpio access array
int intset[32]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
int intclr[32]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int intvalue[32];
int  DS18B20_Pins[32]= {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
int BadSensors[32];
int NumberOfPin=0;

int intvalues[72*32];
int Idx=0;

unsigned int bitdatatable[72];
int bitdataCounter;

#define GPIO_READ(A) gpiod_line_get_value_bulk(&gpiolines,A);
#define GPIO_SET  gpiod_line_set_value_bulk(&gpiolines,intset);
#define GPIO_CLR  gpiod_line_set_value_bulk(&gpiolines,intclr);



#define DS18B20_SKIP_ROM            0xCC
#define DS18B20_CONVERT_T           0x44
#define DS18B20_MATCH_ROM           0x55
#define DS18B20_SEARCH_ROM          0XF0
#define DS18B20_READ_SCRATCHPAD     0xBE
#define DS18B20_WRITE_SCRATCHPAD    0x4E
#define DS18B20_COPY_SCRATCHPAD     0x48

unsigned char ScratchPad[9];
double  temperature;
int   resolution;

bool  init_gpiod(void)
{
  gpiochip = gpiod_chip_open_by_name("gpiochip4");

  if(gpiochip == NULL)
      gpiochip = gpiod_chip_open_by_name("gpiochip0");

  if(gpiochip == NULL)
      {
           printf("unable to open GPIO\n");
           return false;
      }
   gpiod_line_bulk_init(&gpiolines);
}

bool add_pin(int pin)
{

  printf("add Pin %d =>",pin);
  gpioline = gpiod_chip_get_line(gpiochip,pin);

  if(gpioline == NULL)
      {
         printf("  NULL\n");
          return false;
      }
   printf("Done\n");
//   gpiod_line_request_output_flags(gpioline,"DS18B20",
//                 GPIOD_LINE_REQUEST_FLAG_OPEN_DRAIN|GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP,1);
   gpiod_line_bulk_add (&gpiolines,gpioline);


  return true;
}



typedef struct {
unsigned char valid;
unsigned char resolution;
double temperature;
}SensorInfoStruct;

SensorInfoStruct DS18B20_Data[32];


struct timespec  mystart,myacqstart,myend;


double clock_diff(struct timespec start,struct  timespec end)
{
  double dtime;;

  dtime = (double) end.tv_sec- start.tv_sec;
  dtime += (double) ((end.tv_nsec - start.tv_nsec)/ 1.0e9);
  return dtime;
}


#define DELAY1US  DelayMicrosecondsNoSleep(1);

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




// If everything  is ok it will return 0
// otherwise  BadSensor will have the  bit corresponding to the bad sensor set
int   DoReset(void)
{
 unsigned int gpio_pin;


  GPIO_SET
  DelayMicrosecondsNoSleep(10);

  GPIO_CLR
  usleep(480);

  GPIO_SET
  DelayMicrosecondsNoSleep(60);

  GPIO_READ(intvalue)

  DelayMicrosecondsNoSleep(420);

  int Flag=1;
 for(int loop=0;loop<NumberOfPin;loop++)
   {
     BadSensors[loop]=intvalue[loop];
     if(BadSensors[loop]==1)
        Flag=0;
   }
   return Flag;
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
           usleep(60);

        }
        else
        {
           DelayMicrosecondsNoSleep(60);
           GPIO_SET
           usleep(1);
        }
      Mask*=2;
      DelayMicrosecondsNoSleep(60);
    }


   usleep(100);
}


void  ReadByte(unsigned int *datatable)
{
   int loop,loop2;
   unsigned int  _temp;
   unsigned int mask;

   for(loop=0;loop<8;loop++)
     {
       GPIO_CLR
       DELAY1US
       //  set input
       GPIO_SET
       DelayMicrosecondsNoSleep(2);
       GPIO_READ(datatable)
       datatable+=32;
       DelayMicrosecondsNoSleep(60);
      }
}


// extract information by bit position from  table of 72  unsigned long 
void ExtractScratchPad( unsigned int bitmask, unsigned char *ScratchPad)
{
    int loop,loopr,Idx;
    unsigned char Mask=1;

    unsigned char databyte=0;
    unsigned int *pointer= &bitdatatable[0];
    for(loopr=0;loopr<9;loopr++)
     {
       Mask=1;
       databyte=0;
       for(loop=0;loop<8;loop++)
       {
         if((*(pointer++) & bitmask)!=0)
           databyte|=Mask;
         Mask*=2;
       }
      *(ScratchPad++)=databyte;
     }
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






// Adafruit   set_max_priority and set_default priority add-on

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




int ReadSensors(void)
{
  int temp;
  int loop;
  int GotOneResult;
  int GotAllResults;
  unsigned char  CRCByte;

  union {
   short SHORT;
   unsigned char CHAR[2];
  }IntTemp;


   int retryloop;
  // ok now read until we got a least one valid crc up to n times

  #define RETRY_MAX 5

  for(retryloop=0;retryloop<RETRY_MAX;retryloop++)
  {
  GotOneResult=0;  // this will indicate if we have one reading with a good crc 
  GotAllResults=1; // this will indicate if we have all readins from all sensors

  set_max_priority();
  DoReset();

  // Read scratch pad

  WriteByte(DS18B20_SKIP_ROM);
  WriteByte(DS18B20_READ_SCRATCHPAD);

//  for(loop=0;loop<72;loop+=8)
//   ReadByte(&bitdatatable[loop]);

  for(loop=0;loop<72;loop+=8)
    ReadByte(&intvalues[loop*32]);

// convert to bitdatatable
  int mask=1;
  for(loop=0;loop<72;loop++)
   {
     int _temp=0;
     for(int loop2=0;loop2<32;loop2++)
        if(intvalues[loop*32+loop2])
          _temp|= 1 << loop2;
     bitdatatable[loop]=_temp;
   }


  set_default_priority();

  // extract bit info fro valid gpio pin
   for(loop=0;loop<NumberOfPin;loop++)
      {
//       temp = DS18B20_Pins[loop];
       temp = loop;
//      if(temp<0) break;

       // by default put data invalid
         DS18B20_Data[loop].valid=0;

       ExtractScratchPad(1UL<<temp,ScratchPad);
       CRCByte= CalcCRC(ScratchPad,8);

       if(CRCByte!=ScratchPad[8])
        {
         GotAllResults=0;
        }
        else
         {
          //Check Resolution
          resolution=0;

          if((ScratchPad[4] & 0x9F)== 0x1f)
           {
            GotOneResult=1;

            DS18B20_Data[loop].valid=1;
          switch(ScratchPad[4])
           {
            case  0x1f: resolution=9;break;
            case  0x3f: resolution=10;break;
            case  0x5f: resolution=11;break;
            default: resolution=12;break;
           }

          DS18B20_Data[loop].resolution=resolution;
          // Read Temperature

          IntTemp.CHAR[0]=ScratchPad[0];
          IntTemp.CHAR[1]=ScratchPad[1];

          temperature =  0.0625 * (double) IntTemp.SHORT;
          DS18B20_Data[loop].temperature= temperature;
          }
         else
            GotAllResults=0;
         }
       }
   // if(GotOneResult) return(1);
   if(GotAllResults) return(1);
     usleep(10000);
}
return 0;
}


int main(int argc, char **argv)
{
  int temp;
  int loop;
  int Flag=0;
  unsigned long AcqDelay; // Acquisition time delay needed
  unsigned char Hresolution;




  if(argc==1)
  {
    printf("Usage:  DS18B20  GPIOFirst, GPIOSecond, ...,  GPIOLast\n");
    return -1;
  }

   init_gpiod();


  for(loop=1;loop<argc;loop++)
    {
     DS18B20_Pins[loop-1]=atoi(argv[loop]);
     add_pin(DS18B20_Pins[loop-1]);
     NumberOfPin++;
    }
   gpiod_line_request_bulk_output_flags(&gpiolines,"DS18B20",
                 GPIOD_LINE_REQUEST_FLAG_OPEN_DRAIN|GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP,intset);


// first thing to do is to check all sensor to determine which is the highest resolution
//


   Hresolution=9;

   ReadSensors();

   for(loop=0;;loop++)
    {
      if(DS18B20_Pins[loop]<0) break;
      if(DS18B20_Data[loop].valid)
       if(DS18B20_Data[loop].resolution > Hresolution)
          Hresolution=DS18B20_Data[loop].resolution;
    }

// now set timing according to the highesh resolution sensor.

   switch(Hresolution)
   {

     case 9: AcqDelay= 94000;break;
     case 10: AcqDelay= 188000;break;
     case 11: AcqDelay= 375000;break;
     default: AcqDelay= 750000;
   }


   printf("Highest resolution is %d bits. Acquisition delay will be %ldms.\n",Hresolution,AcqDelay/1000);fflush(stdout);
   printf("Hit enter to continue. Ctrl-c to break.\n");
   fflush(stdout);
   getchar();
   clock_gettime(CLOCK_MONOTONIC,&mystart);


// and now the real stuff

    do
   {
   // Do Reset

   clock_gettime(CLOCK_MONOTONIC,&myacqstart);


  set_max_priority();


  DoReset();



  // Start Acquisition

  WriteByte(DS18B20_SKIP_ROM);
  WriteByte(DS18B20_CONVERT_T);

  set_default_priority();

  //  wait  for the highest resolution probe
  usleep(AcqDelay);


  ReadSensors();

   // now let's print result

   clock_gettime(CLOCK_MONOTONIC,&myend);

   printf("====\n%.3f sec  acquisition time = %.3f sec\n",clock_diff(mystart, myacqstart),clock_diff(myacqstart,myend));

   for(loop=0;;loop++)
   {
    if(DS18B20_Pins[loop]<0) break;

    printf("GPIO %d : ",DS18B20_Pins[loop]);

    if(DS18B20_Data[loop].valid)
        printf("%02d bits  Temperature: %6.2f +/- %4.2f Celsius\n", DS18B20_Data[loop].resolution ,DS18B20_Data[loop].temperature, 0.0625 * (double)  (1<<(12 - DS18B20_Data[loop].resolution)));
    else
         printf("Bad CRC!\n");
   }
    fflush(stdout);
  }while(1);

 gpiod_line_release_bulk(&gpiolines);
 gpiod_chip_close(gpiochip);
  return 0;
} // main



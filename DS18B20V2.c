// Read DS18B20 sensor using one DS18B20 sensor per GPIO PIN using
// parallel method to speed up process
// Copyright (c) 29 June 2014  Daniel Perron
//
// Need to install pull-up resistor

// It's need to be compiled like
// gcc -lrt -o DS18B20V2 DS18B20V2.c

// 1 - Create mask for  Set/clear Bit
// 2 - Create mask for  PIO MODE
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
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// August 5, 2014
// - Add retried on fail
// - Add bit resolution checking at the start to set the correct acquisition waiting time
// - Add loop scanning.




// Access from ARM Running Linux
// remark the next define for the original PI, B,B+,A,A+
#define PI2


#ifdef PI2
 #define BCM2708_PERI_BASE        0x3F000000
#else
 #define BCM2708_PERI_BASE        0x20000000
#endif
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */




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




#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
void *gpio_map;

// I/O access
volatile unsigned long *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))


#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
#define GPIO_READ  *(gpio + 13)


#define DS18B20_SKIP_ROM       0xCC
#define DS18B20_CONVERT_T       0x44
#define DS18B20_MATCH_ROM               0x55
#define DS18B20_SEARCH_ROM      0XF0
#define DS18B20_READ_SCRATCHPAD         0xBE
#define DS18B20_WRITE_SCRATCHPAD        0x4E
#define DS18B20_COPY_SCRATCHPAD         0x48



unsigned char ScratchPad[9];
double  temperature;
int   resolution;

void setup_io();

// 32 bits bitdatatable[72];  // 9 register of  8 bits
unsigned long bitdatatable[72];





// pin definition use for sensor

int  DS18B20_Pins[32]= {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};


// Data Sensor result info

typedef struct {
unsigned char valid;
unsigned char resolution;
double temperature;
}SensorInfoStruct;

SensorInfoStruct DS18B20_Data[32];

//  mask bit definition


unsigned long PinMask;

unsigned long  ModeMaskInput[3];
unsigned long  ModeMaskOutput[4];

unsigned long  BadSensor=0;


// time interval calculation

struct timespec  mystart,myacqstart,myend;


double clock_diff(struct timespec start,struct  timespec end)
{
  double dtime;;

  dtime = (double) end.tv_sec- start.tv_sec;
  dtime += (double) ((end.tv_nsec - start.tv_nsec)/ 1.0e9);
  return dtime;
}


void SetInputMode(void)
{
  int loop;

  *(gpio) &= ModeMaskInput[0];
  *(gpio+1) &= ModeMaskInput[1];
  *(gpio+2) &= ModeMaskInput[2];
  *(gpio+3) &= ModeMaskInput[3];
};

void SetOutputMode(void)
{
  *gpio &= ModeMaskInput[0];
  *gpio |= ModeMaskOutput[0];
  *(gpio+1) &= ModeMaskInput[1];
  *(gpio+1) |= ModeMaskOutput[1];
  *(gpio+2) &= ModeMaskInput[2];
  *(gpio+2) |= ModeMaskOutput[2];
  *(gpio+3) &= ModeMaskInput[3];
  *(gpio+3) |= ModeMaskOutput[3];
};



// If everything  is ok it will return 0
// otherwise  BadSensor will have the  bit corresponding to the bad sensor set
int   DoReset(void)
{
 unsigned long gpio_pin;


  SetInputMode();
  usleep(10);

  SetOutputMode();
   // pin low for 480 us

   GPIO_CLR = PinMask;

   usleep(480);

   SetInputMode();

   usleep(60);

   gpio_pin = GPIO_READ;

   usleep(420);

   gpio_pin &= PinMask;

   if(gpio_pin ==0) return 1;

   BadSensor|= gpio_pin;
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

      SetOutputMode();
      GPIO_CLR= PinMask;

       if((value & Mask)!=0)
        {
           DELAY1US
           SetInputMode();
           usleep(60);

        }
        else
        {
           usleep(60);
           SetInputMode();
           usleep(1);
        }
      Mask*=2;
      usleep(60);
    }


   usleep(100);
}


void  ReadByte(unsigned long *datatable)
{
   int loop;

   for(loop=0;loop<8;loop++)
     {
       //  set output
       SetOutputMode();
       //  PIN LOW
       GPIO_CLR= PinMask;
       DELAY1US
       //  set input
       SetInputMode();
       // Wait  2 us
       DELAY1US
       DELAY1US
//       DELAY1US
       *(datatable++)= GPIO_READ;
       usleep(60);
      }
}


// extract information by bit position from  table of 72  unsigned long 
void ExtractScratchPad( unsigned long bitmask, unsigned char *ScratchPad)
{
    int loop,loopr,Idx;
    unsigned char Mask=1;

    unsigned char databyte=0;
    unsigned long *pointer= &bitdatatable[0];
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

  for(loop=0;loop<72;loop+=8)
   ReadByte(&bitdatatable[loop]);

  set_default_priority();

  // extract bit info fro valid gpio pin
   for(loop=0;;loop++)
      {
       temp = DS18B20_Pins[loop];
       if(temp<0) break;

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

  for(loop=1;loop<argc;loop++)
     DS18B20_Pins[loop-1]=atoi(argv[loop]);



  // Set up gpi pointer for direct register access

  //  set default mask


  PinMask = 0;

  for(loop=0;loop<4;loop++)
   {
     ModeMaskInput[loop]=0xffffffffL;
     ModeMaskOutput[loop]=0;
   }



  // set mask for every pin

  for(loop=0;;loop++)
   {
     temp = DS18B20_Pins[loop];
      if(temp<0) break;

      PinMask|= 1<<temp;

     ModeMaskInput[temp/10] &= ~(7<<((temp % 10) * 3));
     ModeMaskOutput[temp/10] |= (1<<((temp % 10) * 3)); 
   }

   setup_io();



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


   printf("Highest resolution is %d bits. Acquisition delay will be %dms.\n",Hresolution,AcqDelay/1000);fflush(stdout);
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
   gpio = (volatile unsigned long *)gpio_map;

} // setup_io


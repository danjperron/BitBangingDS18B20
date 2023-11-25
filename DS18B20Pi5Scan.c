// (c) Copywright  Nov 24, 2023  Daniel Perron
//  version using gpiod library
//  DS18B20Pi5Scan.c
//  read from Raspberry Pi 5  GPIO  A DS18B20 sensor by bitbanging 1 wire protocol 
//  from user space

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <sched.h>
#include <gpiod.h>


struct gpiod_chip  *gpiochip;
struct gpiod_line  *gpioline;


bool  init_gpiod(int Pin)
{
  gpiochip = gpiod_chip_open_by_name("gpiochip4");

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


#define GPIO_READ gpiod_line_get_value (gpioline)
#define GPIO_SET  gpiod_line_set_value(gpioline,1);
#define GPIO_CLR  gpiod_line_set_value(gpioline,0);

#define DS18B20_SKIP_ROM 		0xCC
#define DS18B20_CONVERT_T 		0x44
#define DS18B20_MATCH_ROM               0x55
#define DS18B20_SEARCH_ROM		0XF0
#define DS18B20_READ_SCRATCHPAD         0xBE
#define DS18B20_WRITE_SCRATCHPAD        0x4E
#define DS18B20_COPY_SCRATCHPAD         0x48


unsigned char ScratchPad[9];
double  temperature;
int   resolution;



unsigned short DS_PIN=10;
unsigned short ArgResolution=0;
unsigned short ArgScan=0;
unsigned short ArgFile=0;
unsigned short ArgWaitTime=750;
char FileName[256];


//#define DELAY1US  DelayMicrosecondsNoSleep(1);
#define DELAY1US  DelayNanosecondsNoSleep(1000);

void DelayNanosecondsNoSleep (int delay_ns)
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
      if (time_difference > (delay_ns ))      //Delay for # nS
         break;
   }
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
   GPIO_SET;
   DelayMicrosecondsNoSleep(60);
   if(GPIO_READ==0)
   {
     DelayMicrosecondsNoSleep(420);
     return 1;
   }
  return 0;
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

       WriteByte(DS18B20_READ_SCRATCHPAD);
       for(loop=0;loop<9;loop++)
         {
          ScratchPad[loop]=ReadByte();
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

char  IDGetBit(unsigned long long *llvalue, char bit)
{
  unsigned long long Mask = 1ULL << bit;

  return ((*llvalue & Mask) ? 1 : 0);
}


unsigned long long   IDSetBit(unsigned long long *llvalue, char bit, unsigned char newValue)
{
  unsigned long long Mask = 1ULL << bit;

  if((bit >= 0) && (bit < 64))
  {
  if(newValue==0)
   *llvalue &= ~Mask;
  else
   *llvalue |= Mask;
   }
  return *llvalue;
}


void SelectSensor(unsigned  long long ID)
{
int BitIndex;
char Bit;


WriteByte(DS18B20_MATCH_ROM);

for(BitIndex=0;BitIndex<64;BitIndex++)
   WriteBit(IDGetBit(&ID,BitIndex));

}

int  SearchSensor(unsigned long long * ID, int * LastBitChange)
{
 int BitIndex;
  char Bit , NoBit;


if(*LastBitChange <0) return 0;

// Set bit at LastBitChange Position to 1
// Every bit after LastbitChange will be 0

if(*LastBitChange <64)
{

   IDSetBit(ID,*LastBitChange,1);
   for(BitIndex=*LastBitChange+1;BitIndex<64;BitIndex++)
    IDSetBit(ID,BitIndex,0);
}

*LastBitChange=-1;

if(!DoReset()) return -1;


WriteByte(DS18B20_SEARCH_ROM);

  for(BitIndex=0;BitIndex<64;BitIndex++)
    {

      NoBit = ReadBit();
      Bit = ReadBit();

     if(Bit && NoBit)
        return -2;

     if(!Bit && !NoBit)
        {
          // ok 2 possibilities
//          printf("B");
          if(IDGetBit(ID,BitIndex))
            {
               // Bit High already set 
                WriteBit(1);
             }
          else
             {
               // ok let's try LOW value first
               *LastBitChange=BitIndex;
                WriteBit(0);
             }
         }
      else if(!Bit)
        { 
//	printf("1");
         WriteBit(1);
         IDSetBit(ID,BitIndex,1);
        }
      else
        {
        //printf("0");
        WriteBit(0);
        IDSetBit(ID,BitIndex,0);
        }
//   if((BitIndex % 4)==3)printf(" ");
    }
//
// printf("\n");
  return 1;



}







int ReadSensor(unsigned long long ID)
{
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

  for(RetryCount=0;RetryCount<10;RetryCount++)
  {

   if(!DoReset()) continue;

   // start a conversion
   SelectSensor(ID);

  if(!ReadScratchPad()) continue;

//     for(loop=0;loop<9;loop++)
//       printf("%02X ",ScratchPad[loop]);
//     printf("\n");fflush(stdout);

  // OK Check sum Check;
  CRCByte= CalcCRC(ScratchPad,8);

  if(CRCByte!=ScratchPad[8]) continue;;

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

    ID &= 0x00FFFFFFFFFFFFFFULL;
    printf("%02llX-%012llX : ",ID & 0xFFULL, ID >>8);

    printf("%02d bits  Temperature: %6.2f +/- %4.2f Celsius\n", resolution ,temperature, 0.0625 * (double)  (1<<(12 - resolution)));

    return 1;
    }

  return 0;

}



int GlobalStartConversion(void)
{
   int retry=0;
   int maxloop;

   while(retry<10)
   {
     if(!DoReset())
      usleep(10000);
     else
      {
       WriteByte(DS18B20_SKIP_ROM);
       WriteByte(DS18B20_CONVERT_T);
       maxloop=0;

#define USE_CONSTANT_DELAY
#ifdef USE_CONSTANT_DELAY
       usleep(ArgWaitTime * 1000);
       return 1;
#else
      // wait until ready
      while(!ReadBit())
      {
       maxloop++;
       if(maxloop>100000) break;
      }

      if(maxloop<=100000)  return 1;
#endif
     }
    retry++;
   }
   return 0;


}


void WriteScratchPad(unsigned char TH, unsigned char TL, unsigned char config)
{

    // First reset device

    DoReset();

    usleep(10);
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

void ChangeSensorsResolution(int resolution)
{
   int config=0;

        switch(resolution)
         {
           case 9:  config=0x1f;break;
           case 10: config=0x3f;break;
           case 11: config=0x5f;break;
           default: config=0x7f;break;
         }
      WriteScratchPad(0xff,0xff,config);
      usleep(1000);
      CopyScratchPad();
}



void ScanForSensor(void)
{
  unsigned long long  ID=0ULL;
  int  NextBit=64;
  int  _NextBit;
  int  rcode;
  int retry=0;
  unsigned long long  _ID;
  unsigned char  _ID_CRC;
  unsigned char _ID_Calc_CRC;
  unsigned char  _ID_Family;

  while(retry<10){
   _ID=ID;
   _NextBit=NextBit;
   rcode=SearchSensor(&_ID,&_NextBit);
    if(rcode==1)
     {
        _ID_CRC =  (unsigned char)  (_ID>>56);
        _ID_Calc_CRC =  CalcCRC((unsigned char *) &_ID,7);
        if(_ID_CRC == _ID_Calc_CRC)
        {
         if(ArgScan==0)
          {
           if(ReadSensor(_ID))
            {
              ID=_ID;
              NextBit=_NextBit;
              retry=0;
            }
           else
             retry=0;
          }
          else
           {
            ID=_ID;
            NextBit=_NextBit;
            printf("%016llX\n",ID);
           }
        }
        else retry++;
     }
    else if(rcode==0 )
     break;
    else
     retry++;
}
}


void PrintUsage(char * app)
{
  printf("usage :\n\n\t");
  printf("%s -gpio n [-xbits] [-s] [-t delay] [-f filename]\n\n",app);
  printf(" -gpio n     ->  n specify the BCM GPIO number to check\n");
  printf(" -xbits      ->  x set the number of bits -9bits,-10bits,-11bits and -12bits\n");
  printf(" -t delay    ->  delay is the time in ms to wait after conversion\n");
  printf(" -s          ->  Scan for sensor\n");
  printf(" -f filename ->  filename to read sensor id and return information\n");


}


int DecodeArg(int argc, char ** argv)
{

   int idx=1;

   if(argc==1)
    {
      PrintUsage(argv[0]);
      return 0;
    }


   while(idx<argc)
    {
       if(strstr(argv[idx],"help")!=NULL)
        {
          PrintUsage(argv[0]);
          return 0;
        }
       if(strcmp(argv[idx],"-gpio")==0)
            DS_PIN = atoi(argv[++idx]);
       else if(strcmp(argv[idx],"-9bits")==0)
            ArgResolution=9;
       else if(strcmp(argv[idx],"-10bits")==0)
            ArgResolution=10;
       else if(strcmp(argv[idx],"-11bits")==0)
            ArgResolution=11;
       else if(strcmp(argv[idx],"-12bits")==0)
            ArgResolution=12;
       else if(strcmp(argv[idx],"-s")==0)
         ArgScan=1;
       else if(strcmp(argv[idx],"-t")==0)
           ArgWaitTime = atoi(argv[++idx]);
       else if(strcmp(argv[idx],"-f")==0)
        {
          ArgFile=1;
          strcpy(FileName,argv[++idx]);
        }
       else
        {
         printf("Unknown argument %s! ",argv[idx]);
         exit(0);
        }
       idx++;
     }
  return 1;
}


void ReadSensorsFromFile(char * Filename)
{
   FILE * src;
   char LineBuffer[256];
   unsigned long long SensorID;

   src = fopen(Filename,"rt");
   if(src==NULL) return;
   while(1)
   {
    if(fgets(LineBuffer,256,src)==NULL) break;
    if(sscanf(LineBuffer,"%llx",&SensorID)==1)
        ReadSensor(SensorID);
   }
  fclose(src);
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



int main(int argc, char **argv)
{
  int loop;
  int Flag=0;
  // Set up gpi pointer for direct register access

  if(DecodeArg(argc,argv)==0)
     return 0;

  //  set gpiod
  init_gpiod(DS_PIN);

  // Check for pull up resistor
  // Signal  input should be high

  GPIO_SET
  Flag=0;
  for(loop=0;loop<100;loop++)
   {
     usleep(1000);
     if(GPIO_READ!=0)
        {
          Flag=1;
          break;
        }
   }

   if(Flag==0)
    {
      printf("*** Error Unable to detect Logic level 1. No pull-up ?\n");
      gpiod_line_release(gpioline);
      gpiod_chip_close(gpiochip);
      exit(-1);
    }

    if(ArgResolution>0)
      {
        // need to change resolution
        ChangeSensorsResolution(ArgResolution);
        // do it twice just in case
        ChangeSensorsResolution(ArgResolution);
     }


    if(GlobalStartConversion()==0)
    {
      printf("*** Error Unable to detect any DS18B20 sensor\n");
      gpiod_line_release(gpioline);
      gpiod_chip_close(gpiochip);
      exit(-2);
    }

  set_max_priority();


   if(ArgFile)
   {
     ReadSensorsFromFile(FileName);
   }
   else
     ScanForSensor();

  set_default_priority(); 

  gpiod_line_release(gpioline);
  gpiod_chip_close(gpiochip);

  return 0;

} // main



// April 2018
//  change /dev/mem to /dev/gpiomem
// now sudo  is not needed anymore
#define USE_GPIOMEM

// modified version to check BCM physical address
// February 1, 2016
// check  "/proc/device-tree/soc/ranges" for BCM address
//
// add timer delay for us resolution from Gladkikh Artem
// DelayMicrosecondsNoSleep

// modified version to read DS18B20 in bit banging
//
// 26 June 2014
//
// Add fix for no input pull-up or sensor
// Set Start Acquistion waiting delay  using a constant of 750ms
// Add set aquisition resolution into the eerom config (-9bits,-10bits,-11bits,-12bits(default)
// Add parameter to change the Wait time after start of aquisition (check pdf info for correct timing).
// Add possibility to use 64bits sensor ID from a file.

//  25 June 2014
// 
// Updated version
// Search rom  code command added
// More than one sensor  on the same pin
//
// Argument:
//  -s        =>  just scan and display the 64 bits rom code of the probe
//  -gpio n   =>  Specify pin other than GPIO10
//
//  Step process
//  1- Start Conversion using SKIP ROM  (all devices)
//  2- Search for sensor. When one is found, Display the temperature


//  24 May 2014
//  Daniel Perron
//
// Use At your own risk




//
//  How to access GPIO registers from C-code on the Raspberry-Pi
//  Example program
//  15-January-2012
//  Dom and Gert
//  Revised: 15-Feb-2013


// Access from ARM Running Linux



#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <sched.h>

//PI4
unsigned long BCM_PERI_BASE=0xFE000000;

//unsigned long BCM_PERI_BASE=0x20000000;


#define GPIO_BASE                (BCM_PERI_BASE + 0x200000) /* GPIO controller */


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

void setup_io();



unsigned short DS_PIN=10;
unsigned short ArgResolution=0;
unsigned short ArgScan=0;
unsigned short ArgFile=0;
unsigned short ArgWaitTime=750;
char FileName[256];


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



int  DoReset(void)
{
 int loop;

   INP_GPIO(DS_PIN);


   DelayMicrosecondsNoSleep(10);

   INP_GPIO(DS_PIN);
   OUT_GPIO(DS_PIN);

   // pin low for 480 us
   GPIO_CLR=1<<DS_PIN;
   usleep(480);
   INP_GPIO(DS_PIN);
   DelayMicrosecondsNoSleep(60);
   if(GPIO_READ(DS_PIN)==0)
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
       INP_GPIO(DS_PIN);
       OUT_GPIO(DS_PIN);
       GPIO_CLR= 1 <<DS_PIN;

       if((value & Mask)!=0)
        {
           DELAY1US
            INP_GPIO(DS_PIN);
           DelayMicrosecondsNoSleep(60);

        }
        else
        {
           DelayMicrosecondsNoSleep(60);
           INP_GPIO(DS_PIN);
           DelayMicrosecondsNoSleep(1);
        }
      Mask*=2;
      DelayMicrosecondsNoSleep(60);
    }


   usleep(100);
}

void WriteBit(unsigned char value)
{
   INP_GPIO(DS_PIN);
   OUT_GPIO(DS_PIN);
   GPIO_CLR=1 <<DS_PIN;
   if(value)
    {
      DELAY1US
      INP_GPIO(DS_PIN);
      DelayMicrosecondsNoSleep(60);
    }
   else
    {
      DelayMicrosecondsNoSleep(60);
      INP_GPIO(DS_PIN);
      DelayMicrosecondsNoSleep(1);
     }
   DelayMicrosecondsNoSleep(60);
}





unsigned char ReadBit(void)
{
   unsigned char rvalue=0;
   INP_GPIO(DS_PIN);
   OUT_GPIO(DS_PIN);
   // PIN LOW
   GPIO_CLR= 1 << DS_PIN;
   DELAY1US
   // set INPUT
   INP_GPIO(DS_PIN);
   DelayMicrosecondsNoSleep(2);
   if(GPIO_READ(DS_PIN)!=0)
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
       INP_GPIO(DS_PIN);
       OUT_GPIO(DS_PIN);
       //  PIN LOW
       GPIO_CLR= 1<<DS_PIN;
       DELAY1US
       //  set input
       INP_GPIO(DS_PIN);
       // Wait  2 us
       DelayMicrosecondsNoSleep(2);
       if(GPIO_READ(DS_PIN)!=0)
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

  setup_io();

  // Check for pull up resistor
  // Signal  input should be high

  // Set PIN to INPUT MODe
  INP_GPIO(DS_PIN);

  Flag=0;
  for(loop=0;loop<100;loop++)
   {
     usleep(1000);
     if(GPIO_READ(DS_PIN)!=0)
        {
          Flag=1;
          break;
        }
   }

   if(Flag==0)
    {
      printf("*** Error Unable to detect Logic level 1. No pull-up ?\n");
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

  return 0;

} // main


//
// Set up a memory regions to access GPIO
//
void setup_io()
{
 int handle;
  int count;

  #ifdef __arch64__
      struct{
       unsigned  int  V1,X1,V2,X2,V3,X3;
      }ranges;
  #else
      struct{
      unsigned  int  V1,V2,V3;
      unsigned  int  X1,X2,X3;
      }ranges;
  #endif


#ifdef USE_GPIOMEM
   /* open /dev/mem */
   if ((mem_fd = open("/dev/gpiomem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/gpiomem \n");
      exit(-1);
   }
#else
   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      exit(-1);
   }
#endif

  // read /proc/device-tree/soc/ranges
  // to check if we have the GPIO at 0x20000000 or 0x3F000000

#define Swap4Bytes(val) \
 ( (((val) >> 24) & 0x000000FF) | (((val) >>  8) & 0x0000FF00) | \
   (((val) <<  8) & 0x00FF0000) | (((val) << 24) & 0xFF000000) )


  handle =  open("/proc/device-tree/soc/ranges" ,  O_RDONLY);

  if(handle >=0)
   {
     count = read(handle,&ranges,24);
     if(count == 24)
       BCM_PERI_BASE=Swap4Bytes(ranges.V2);
     close(handle);
   }

//   printf("BCM GPIO BASE= %lx\n",BCM2708_PERI_BASE);


#ifdef USE_GPIOMEM
  /* mmap GPIO */
   gpio_map = mmap(
      NULL,             //Any adddress in our space will do
      0xB4,             //Map length
      PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
      MAP_SHARED,       //Shared with other processes
      mem_fd,           //File to map
      0                 //Offset to GPIO peripheral
   );
#else
   /* mmap GPIO */
   gpio_map = mmap(
      NULL,             //Any adddress in our space will do
      BLOCK_SIZE,       //Map length
      PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
      MAP_SHARED,       //Shared with other processes
      mem_fd,           //File to map
      GPIO_BASE         //Offset to GPIO peripheral
   );
#endif


   close(mem_fd); //No need to keep mem_fd open after mmap

   if (gpio_map == MAP_FAILED) {
      printf("mmap error gpio_map=%p\n", gpio_map);//errno also set!
      exit(-1);
   }

   // Always use volatile pointer!
   gpio = (volatile unsigned *)gpio_map;


} // setup_io


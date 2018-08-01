#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sched.h>
#include <string.h>
#include <Python.h>
#include <sys/time.h>
#include <time.h>

#define DEBUG
#undef DEBUG


#define TYPE_DS18B20  0x28
#define TYPE_MAX31850 0x3B





/*
Copyright (c) <2017> <Daniel Perron>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


/*
   Programmer : Daniel Perron
   Date       : 15 December 2017
   software   : DS18B20 user space reader
   License     : MIT license
   version 2.01
               - python module implementation
*/

/*
   Programmer : Daniel Perron
   Date       : 4 April 2018
   software   : DS18B20 user space reader
   License     : MIT license
   version 2.02
               - python module implementation using /dev/gpiomem
                 no need of sudo anymore
*/


// Usage of /dev/gpiomem instead of /dev/mem
// if you want /dev/mem undefine USE_GPIOMEM
#define USE_GPIOMEM




unsigned long AcquisitionDelay=750000;

unsigned long BCM2708_PERI_BASE=0x20000000;
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */




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

#define GPIO_READ_BIT(g)  (*(gpio + 13) &= (1<<(g)))

#define DS18B20_SKIP_ROM       0xCC
#define DS18B20_CONVERT_T       0x44
#define DS18B20_MATCH_ROM               0x55
#define DS18B20_SEARCH_ROM      0XF0
#define DS18B20_READ_SCRATCHPAD         0xBE
#define DS18B20_WRITE_SCRATCHPAD        0x4E
#define DS18B20_COPY_SCRATCHPAD         0x48



unsigned short DS_PIN;
unsigned char ScratchPad[9];
double  temperature;
int   resolution;


// 32 bits bitdatatable[72];  // 9 register of  8 bits
unsigned long bitdatatable[72];





// pin definition use for sensor

int  DS18B20_Pins[32];


// Data Sensor result info

typedef struct {
unsigned char valid;
unsigned char resolution;
double temperature;
}SensorInfoStruct;

SensorInfoStruct DS18B20_Data[32];

//  mask bit definition


unsigned long PinMask;

unsigned long  ModeMaskInput[4];
unsigned long  ModeMaskOutput[4];

unsigned long  BadSensor=0;



//////////////////////////
// prototype
int   DoReset(void);
static PyObject* DS18B20_pinsReadTemperature(PyObject* self, PyObject* args);
static PyObject* DS18B20_setResolution(PyObject* self, PyObject* args);
static PyObject* DS18B20_getResolution(PyObject* self, PyObject* args);
static PyObject* DS18B20_scan(PyObject* self, PyObject* args);
static PyObject* DS18B20_readTemperature(PyObject* self, PyObject* args);
static PyObject* MAX31850_readTemperature(PyObject* self, PyObject* args);
static PyObject* DS18B20_pinsStartConversion(PyObject* self, PyObject* args);
static PyObject* DS18B20_setAcquisitionDelay(PyObject* self, PyObject* args);
static PyObject* DS18B20_getAcquisitionDelay(PyObject* self, PyObject* args);
static PyObject* DS18B20_readScratchPad(PyObject* self, PyObject* args);
static PyObject * DS18B20_version(PyObject* self, PyObject* args);


// time interval calculation

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




void SetInputMode(void)
{
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


 GPIO_SET= PinMask;
 SetOutputMode();
 GPIO_SET= PinMask;

// SetInputMode();
  DelayMicrosecondsNoSleep(10);

//  SetOutputMode();
   // pin low for 480 us

   GPIO_CLR = PinMask;

   usleep(480);


 GPIO_SET= PinMask;
 SetOutputMode();
 GPIO_SET= PinMask;
 DelayMicrosecondsNoSleep(10);

  SetInputMode();

   DelayMicrosecondsNoSleep(110);

   gpio_pin = GPIO_READ;

   DelayMicrosecondsNoSleep(360);

   gpio_pin &= PinMask;

   if(gpio_pin ==0) return 1;

   BadSensor|= gpio_pin;
   return 0;
}


void WriteByte(unsigned char value,unsigned char ParasiteMode)
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
         if((loop == 7) && ParasiteMode)
          {
           GPIO_SET = PinMask;
          }
          else
          {
           SetInputMode();
          }
           usleep(60);
        }
        else
        {
           DelayMicrosecondsNoSleep(60);
         if((loop == 7) && ParasiteMode)
          {
           GPIO_SET = PinMask;
          }
          else
          {
           SetInputMode();
          }
           usleep(1);
        }
      Mask*=2;
      DelayMicrosecondsNoSleep(60);
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
       // Wait  less than 15 us
       DelayMicrosecondsNoSleep(13);
       *(datatable++)= GPIO_READ;
       DelayMicrosecondsNoSleep(47);
      }
}


// extract information by bit position from  table of 72  unsigned long 
void ExtractScratchPad( unsigned long bitmask, unsigned char *ScratchPad)
{
    int loop,loopr;
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
//  int GotOneResult;
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
 // GotOneResult=0;  // this will indicate if we have one reading with a good crc 
  GotAllResults=1; // this will indicate if we have all readins from all sensors

  set_max_priority();
  DoReset();

  // Read scratch pad

  WriteByte(DS18B20_SKIP_ROM,0);
  WriteByte(DS18B20_READ_SCRATCHPAD,0);

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
//            GotOneResult=1;

            DS18B20_Data[loop].valid=1;

          if((ScratchPad[4] & 0xF0) == 0xF0)
          {
            // we have a Max31850
             resolution=10;
             IntTemp.CHAR[0]=ScratchPad[0] & 0xFE;
          }
          else
          {
          switch(ScratchPad[4])
           {
            case  0x1f: resolution=9;break;
            case  0x3f: resolution=10;break; 
            case  0x5f: resolution=11;break;
            default: resolution=12;break;
           }
          IntTemp.CHAR[0]=ScratchPad[0];
          }
          DS18B20_Data[loop].resolution=resolution;
          // Read Temperature

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










typedef union
{
  unsigned long word32[2];
  unsigned long long word64;
}union_word64_32;



static PyObject *DS18B20Error;


static PyObject * DS18B20_readTemperature(PyObject *self, PyObject * args);



static char  Help_pinsRead[]="pinsRead(StartConversion, pins list [])\n"\
                           " Get All sensors Temperature from BCM selected pins from list\n"\
                           "ex:\n\t import DS18B20\n"\
                                "\t DS18B20.pinsRead(True, [ 20,21,19,16])\n";


static char  Help_pinsStartConversion[] = "pinsStartConversion( pins list [])\n"\
                                        "Start conversion  off al sensors  from BCM selected pins from the list\n"\
                                        "ex:\n\t import DS18B20\n"\
                                             "\t DS18B20.pinsStartConversion([20,21,19,16])\n"\
                                             "\t time.sleep(0.75) #750ms acquisition delay(12bits res)\n"\
                                             "\t sensorsA= DS18B20.pinsRead(False, [20,21,19])\n"\
                                             "\t sensorB=  DS18B20.read(False,16,'28-000006EF85D6')\n";

static char  Help_read[]=    "read(StartConversion, BCM pin, sensor ID)\n"\
                           "read  the temperature from the specific sensor ID string and the BCM Pin Number\n"\
                           "StartConversion False! No Conversion,no delay, read scratch pad\n"\
                           "StartConversion True!  Start Conversion ,delay of 0.75 sec, read scratch pad\n"
                           "ex:\n\t import DS18B20\n"\
                                "\t DS18B20.read(True,16,'28-000006EF85D6')\n"\
                                "\t 19.875\n";

static char  Help_read31850[]=    "readMax31850(StartConversion, BCM pin, sensor ID)\n"\
                           "read  both termocouple and the internal temperature from the specific sensor ID string and the BCM Pin Number\n"\
                                "\t return [ Thermocouple , internal , Errorcode]\n"\
                                "\t Where the Error code bit is,\n"\
                                "\t Bit0:Fault\n\t Bit1:Open circuit\n\t Bit2:Short to GND\n\t Bit3:Short to VDD\n"\
                                "\t An error code of 0  is no error.\n"\
                           "ex:\n\t import DS18B20\n"\
                                "\t DS18B20.readMax31850(True,16,'3B-000006EF85D6')\n"\
                                "\t [19.875, 18.000, 0] \n";


static char  Help_scan[]=    "scan(BCM pin)\n"\
                          "Find all sensors connected to the specific BCM pins\n"\
                           "ex:\n\t import DS18B20\n"\
                                "\t sensors=DS18B20.scan(20)\n"\
                                "\t print(sensors)\n"\
                                "\t ['28-000006EF85D6', '28-000006F058C9', '28-0215C26F41FF']\n";

static char  Help_getRes[]=  "getResolution(BCM pin, sensor ID)\n"\
                           "read resolution from a specific sensor connected to a specific BCM pin\n"\
                           "ex:\n\t import DS18B20\n"\
                                "\t DS18B20.getResolution(20,'28-000006EF85D6')\n"\
                                "\t 12\n";

static char  Help_setRes[]=  "setResolution(BCM pin, sensor ID, resolution)\n"\
                           "set bit resolution, (9 , 10 ,11 or 12) to a specific sensor connected to a specific BCM pin\n"\
                           "ex:\n\t import DS18B20\n"\
                                "\t DS18B20.setResolution(20,'28-000006EF85D6',12)\n";


static char Help_getAcq[]=  "getAcquisitionDelay()\n"\
                           "return the Acquisition delay time in second when the read function \"StartConversion\" flag is true\n"\
                           "ex:\n\t import DS18B20\n"\
                                "\t DS18B20.getAcquisitionDelay()\n";

static char Help_setAcq[]=  "setAcquisitionDelay( aquisition delay in second)\n"\
                           "set the Acquisition delay time in second when the read function \"StartConversion\" flag is true\n"\
                           "ex:\n\t import DS18B20\n"\
                                "\t DS18B20.setAcquisitionDelay(0.75)\n";

static char Help_ScratchPad[]="readScratchPad(BCM pin, sensor ID)\n"\
                             "read Scratch Pad from the specific sensor  ID string and the BCM Pin Number\n"\
                             "ex:\n\t import DS18B20\n"\
                                  "\t DS18B20.readScratchPad(16,'28-000006EF85D6')\n"\
                                  "\t [62, 1, 255, 255, 127, 255, 2, 16, 11]\n";

static char Help_version[]="version()\n"\
                           "return software version number\n";




//  python function table

static PyMethodDef DS18B20Methods[] = {
  { "pinsRead", DS18B20_pinsReadTemperature,METH_VARARGS, Help_pinsRead},
  { "pinsStartConversion", DS18B20_pinsStartConversion,METH_VARARGS, Help_pinsStartConversion},
  { "read", DS18B20_readTemperature,METH_VARARGS, Help_read},
  { "readMax31850", MAX31850_readTemperature,METH_VARARGS, Help_read31850},
  { "scan", DS18B20_scan,METH_VARARGS,Help_scan},
  { "getResolution", DS18B20_getResolution, METH_VARARGS,Help_getRes},
  { "setResolution", DS18B20_setResolution, METH_VARARGS,Help_setRes},
  { "setAcquisitionDelay", DS18B20_setAcquisitionDelay,METH_O,Help_setAcq},
  { "getAcquisitionDelay", DS18B20_getAcquisitionDelay,METH_NOARGS,Help_getAcq},
  { "readScratchPad",      DS18B20_readScratchPad,METH_VARARGS,Help_ScratchPad},
  { "version",             DS18B20_version,METH_NOARGS,Help_version},
  { NULL,NULL,0,NULL}
};



static char MainDoc[] = "DS18B20 Version 2.02  April 4, 2018  (sudo not needed)\n"\
			"(c) Daniel Perron 15 December 2017\n"\
                        "Rapsberry Pi user space DS18B20 utility via GPIO\n"\
                        "Bitbanging manipulation to read DS18B20 sensor from one or multiple GPIO.\n"\
                        "pinsStartConversion() and pinsRead() functions allow to read and start conversion\n"\
                        "on multiple GPIOs but with only one sensor per GPIO. In that mode all sensors are read in parallel.\n"\
                        "the scan() function will return the sensor ID from a specific pin.\n"\
                        "All the other functions  need to be set with the current pin and Sensor ID string\n";



#if  PY_MAJOR_VERSION == 3



// python module definition
static struct PyModuleDef DS18B20Def=
{
   PyModuleDef_HEAD_INIT,
   "DS18B20",
   MainDoc,-1,
   DS18B20Methods
};

#endif


// find which  raspberry Pi for the correct IOMAP

unsigned int ReturnIOMapAddress(void)
{
  // let's read /proc/iomem and check for soc/gpio
  // if found extract the first address of the line

  unsigned int Start,End;
  char line[1024];
  FILE * src;

  src = fopen("/proc/iomem","rt");
  if(src == NULL)
    {
      return 0 ;
    }
  while(!feof(src))
  {
    if(fgets(line,1024,src)==NULL) break;
     if(strstr(line,"/soc/gpio")!=NULL)
     {
        // found /soc/gpio
        // extract address
        if(sscanf(line,"%x-%x", &Start,&End) == 2)
        {
          fclose(src);
          return Start;
        }
     }

  }

fclose(src);

return 0; // not found


}



// define correct function for python2 or 3

#if  PY_MAJOR_VERSION == 3
PyMODINIT_FUNC
PyInit_DS18B20(void)
#else
PyMODINIT_FUNC
initDS18B20(void)
#endif
{
   PyObject *m;
   int handle;
   int count;
   struct { unsigned long V1,V2,V3;} ranges;




#if  PY_MAJOR_VERSION == 3
   m = PyModule_Create(&DS18B20Def);
   #define RETURN(A) return(A)

#else
   m = Py_InitModule3("DS18B20", DS18B20Methods, MainDoc);
   #define RETURN(A) return;

#endif

   if ( m == NULL)
      RETURN(NULL);

   DS18B20Error = PyErr_NewException("DS18B20.error",NULL,NULL);
   Py_INCREF(DS18B20Error);
   PyModule_AddObject(m, "error", DS18B20Error);


#ifdef USE_GPIOMEM
    /* open /dev/mem */
   if ((mem_fd = open("/dev/gpiomem", O_RDWR|O_SYNC) ) < 0) {
      PyErr_SetString(DS18B20Error,"Unable to open /dev/gpiomem. Failed");
      RETURN(NULL);
#else
    /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      PyErr_SetString(DS18B20Error,"Unable to open /dev/mem. Failed");
      RETURN(NULL);
#endif
   }


 // read /proc/device-tree/soc/ranges
  // to check if we have the GPIO at 0x20000000 or 0x3F000000

  #define Swap4Bytes(val) \
  ((((val) >> 24) & 0x000000FF) | (((val) >>  8) & 0x0000FF00) | \
   (((val) <<  8) & 0x00FF0000) | (((val) << 24) & 0xFF000000) )


  handle =  open("/proc/device-tree/soc/ranges" ,  O_RDONLY);

  if(handle >=0)
   {
     count = read(handle,&ranges,12);
     if(count == 12)
       BCM2708_PERI_BASE=Swap4Bytes(ranges.V2);
     close(handle);
   }
    else
   {
      PyErr_SetString(DS18B20Error,"Unable to open /proc/device-tree/soc/ranges. Failed");
      RETURN(NULL);
   }

#ifdef DEBUG
    printf("PeriBase is %lX\n",BCM2708_PERI_BASE);
    fflush(stdout);
#endif

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
   close(mem_fd);


   if(gpio_map == MAP_FAILED)
   {
      PyErr_SetString(DS18B20Error,"Unable to map GPIO. Failed");
      RETURN(NULL);
   }

   gpio = (volatile unsigned long *) gpio_map;


   RETURN(m);

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
   // wait less than 15 us
   DelayMicrosecondsNoSleep(13);
   if(GPIO_READ_BIT(DS_PIN)!=0)
    rvalue=1;
   DelayMicrosecondsNoSleep(47);
   return rvalue;
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

 DoReset();

  WriteByte(DS18B20_SEARCH_ROM,0);
  for(BitIndex=0;BitIndex<64;BitIndex++)
    {

      NoBit =ReadBit();
      Bit = ReadBit();

     if(Bit && NoBit)
      {
        return -2;
      }
     if(!Bit && !NoBit)
        {
          // ok 2 possibilities
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
         WriteBit(1);
         IDSetBit(ID,BitIndex,1);
        }
      else
        {
        WriteBit(0);
        IDSetBit(ID,BitIndex,0);
        }
    }
//
  return 1;



}


unsigned long long SensorIdToLLong(char * ID)
{

  unsigned int Family;
  unsigned long long SerialNo;
  unsigned long long ltemp;

 int rcode = sscanf(ID,"%02X-%llX",&Family,&SerialNo);

#ifdef DEBUG
   printf("rcode = %d Family=%02X SerialNo=%llX\n", rcode,Family, SerialNo);fflush(stdout);
#endif
  if(rcode !=2)
    return 0;
 SerialNo <<= 8;
 SerialNo &= ~(0xFFULL);
 SerialNo |= (unsigned long long) Family;
 ltemp  = ((unsigned long long) CalcCRC((unsigned char *) &SerialNo,7)) << 56;
 SerialNo |= ltemp;
#ifdef DEBUG
   printf("Final SerialNo=%llX\n",SerialNo);fflush(stdout);
#endif
 return SerialNo;

}




void ScanForSensor(PyObject *sensorList)
{
  unsigned long long  ID=0ULL;
  int  NextBit=64;
  int  _NextBit=0;
  int  rcode=0;
  int retry=0;
  unsigned long long  _ID=0;
  unsigned char  _ID_CRC=0;
  unsigned char _ID_Calc_CRC=0;
  char IDString[64];


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
            ID=_ID;
            NextBit=_NextBit;
#ifdef DEBUG
            printf("%016llX\n",ID);fflush(stdout);
#endif
            ID &= 0x00FFFFFFFFFFFFFFULL;
            if(ID != 0ULL)
            {
            sprintf(IDString, "%02llX-%012llX",ID & 0xFFULL, ID >>8);
            int len = strlen(IDString);
//            PyList_Append(sensorList,Py_Decode(IDString,len,"ascii","ignore"));
            PyList_Append(sensorList,Py_BuildValue("s#",IDString,len));
            }
#ifdef DEBUG
           printf("ID To Hex=%llX\n",SensorIdToLLong(IDString));fflush(stdout);
#endif
        }
        else retry++;
     }
    else if(rcode==0 )
     break;
    else
     retry++;
}
}


unsigned char pinReadByte(void)
{

   unsigned char Mask=1;
   int loop;
   unsigned  char data=0;


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
      if((GPIO_READ_BIT(DS_PIN))!=0)
       {
        data |= Mask;
       }
       Mask*=2;
       DelayMicrosecondsNoSleep(60);
      }

    return data;
}



int ReadScratchPad(void)
{
   int loop;

       WriteByte(DS18B20_READ_SCRATCHPAD,0);
       for(loop=0;loop<9;loop++)
         {
          ScratchPad[loop]=pinReadByte();
        }
   return 1;
}



void WriteScratchPad(unsigned char TH, unsigned char TL, unsigned char config)
{


    // Write Scratch pad

    WriteByte(DS18B20_WRITE_SCRATCHPAD,0);

    // Write TH

    WriteByte(TH,0);

    // Write TL

    WriteByte(TL,0);

    // Write config

    WriteByte(config,0);
}






void SelectSensor(unsigned  long long ID)
{
int BitIndex;

WriteByte(DS18B20_MATCH_ROM,0);

for(BitIndex=0;BitIndex<64;BitIndex++)
   WriteBit(IDGetBit(&ID,BitIndex));

}


int ReadSensor( unsigned long long ID,unsigned char SensorType, double * value, double * value2,int * ErrorFlag)
{
  int RetryCount;
  unsigned char  CRCByte;
  union {
   short SHORT;
   unsigned char CHAR[2];
  }IntTemp;

#ifdef DEBUG
  printf("In readSensor(%llX)\n",ID);fflush(stdout);
#endif


  *value=-9999.9;

  for(RetryCount=0;RetryCount<10;RetryCount++)
  {

   DoReset();

   SelectSensor(ID);



  if(!ReadScratchPad())
   {
#ifdef DEBUG
      printf("Unable to read Scratchpad\n");fflush(stdout);
#endif
      continue;
   }
//     for(loop=0;loop<9;loop++)
//       printf("%02X ",ScratchPad[loop]);
//     printf("\n");fflush(stdout);

  // OK Check sum Check;
  CRCByte= CalcCRC(ScratchPad,8);

  if(CRCByte!=ScratchPad[8])
   {
#ifdef DEBUG
      printf("CRC Invalid\n");fflush(stdout);
#endif
      continue;;
   }

 resolution=0;

if(SensorType == TYPE_DS18B20)
{
  //Check Resolution
   switch(ScratchPad[4])
   {

     case  0x1f: resolution=9;break;
     case  0x3f: resolution=10;break;
     case  0x5f: resolution=11;break;
     case  0x7f: resolution=12;break;
   }

   if(resolution==0)
   {
#ifdef DEBUG
      printf("resolution Invalid\n");fflush(stdout);
#endif
      continue;
    }
}

    // Read Temperature

  *ErrorFlag=0;
  if( SensorType == TYPE_MAX31850)
   {
    IntTemp.CHAR[0]=ScratchPad[0] & 0xFC;
    if((ScratchPad[0] & 1)==1)
      *ErrorFlag=1;
   }
  else
   {
    IntTemp.CHAR[0]=ScratchPad[0];
   }
    IntTemp.CHAR[1]=ScratchPad[1];
    *value =  0.0625 * (double) IntTemp.SHORT;
    *value2= 0.0;
  if( SensorType == TYPE_MAX31850)
   {
    IntTemp.CHAR[0]=ScratchPad[2] & 0xF0;
    IntTemp.CHAR[1]=ScratchPad[3];
    if((ScratchPad[2] & 1) ==1) {*ErrorFlag |=2;}
    if((ScratchPad[2] & 2) ==2) {*ErrorFlag |=4;}
    if((ScratchPad[2] & 4) ==4) {*ErrorFlag |=8;}
    *value2 =  0.0625 * (double)  (IntTemp.SHORT >> 4);
   }

#ifdef DEBUG
    ID &= 0x00FFFFFFFFFFFFFFULL;
    printf("%02llX-%012llX : ",ID & 0xFFULL, ID >>8);

    printf("Thermocouple: %6.2f Celsius, Internal %6.2f Celsius\n",*value,*value2);fflush(stdout); 
#endif
    return 1;
    }

  return 0;

}




//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//   pinsStartConversion(pins [])
static PyObject* DS18B20_pinsStartConversion(PyObject* self, PyObject* args)
{
   int loop;

#ifdef DEBUG
  printf("In DS18B20_pinsStartConversion\n");fflush(stdout);
#endif

  // clean Previous  Pins usage
 for(loop=0;loop<32;loop++)
   DS18B20_Pins[loop]=-1;


  // clean mask
  for(loop=0;loop<4;loop++)
   {
     ModeMaskInput[loop]=0xffffffffL;
     ModeMaskOutput[loop]=0;
   }


  //  get number of pins and set mask

  long length = PyTuple_Size(args);


#ifdef DEBUG
  printf("Tuple size is %ld\n",length);fflush(stdout);
#endif


  if(length !=1)
  {
    Py_INCREF(Py_None);
    return(Py_None);
  }


 PyObject * Plist = PyTuple_GetItem(args,0);

  length = PyList_Size(Plist);

#ifdef DEBUG
  printf("Plist size is %ld\n",length);fflush(stdout);
#endif


  if(length <1)
  {

#ifdef DEBUG
   printf("# of args!=1 (%ld)\n",length);fflush(stdout);
#endif
    Py_INCREF(Py_None);
    return(Py_None);
  }


#ifdef DEBUG
   printf("%ld Arguments\n",length);fflush(stdout);
#endif

  int i;
  int temp;
  int PinCount =0;
  PinMask = 0;

  for(i=0;i<length;i++)
  {
    PyObject* Pytemp = PyList_GetItem(Plist,i);
    long elem = PyLong_AsLong(Pytemp);
#ifdef DEBUG
    printf("Element %d = %ld\n",i,elem);fflush(stdout);
#endif
    if(elem < 0) continue;
    if(elem > 31) continue;

#ifdef DEBUG
    printf("Pin #%ld\n",elem);fflush(stdout);
#endif

    DS18B20_Pins[PinCount++]=elem;
    temp = elem;
    PinMask |= 1 << temp;
    ModeMaskInput[temp/10] &= ~(7<<((temp % 10) * 3));
    ModeMaskOutput[temp/10] |= (1<<((temp % 10) * 3));

  }

   // ok now it's time to read the sensor in bulk
      clock_gettime(CLOCK_MONOTONIC,&myacqstart);

      set_max_priority();
      DoReset();
      // start Acquisition
      WriteByte(DS18B20_SKIP_ROM,0);
      WriteByte(DS18B20_CONVERT_T,1);

      set_default_priority();

      Py_INCREF(Py_None);
      return(Py_None);
}

//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//   readMax31850
static PyObject* MAX31850_readTemperature(PyObject* self, PyObject* args)
{
 unsigned long long ID;
 int loop,temp;
 int conversion;

 long length = PyTuple_Size(args);

  if(length != 3)
  {
#ifdef DEBUG
   printf("(Length=%ld) != 3\n",length);
#endif
    Py_INCREF(Py_None);
    return(Py_None);
  }
    PyObject* Pytemp = PyTuple_GetItem(args,0);
    conversion = (int) PyLong_AsLong(Pytemp);

    Pytemp = PyTuple_GetItem(args,1);
    DS_PIN = (unsigned short) PyLong_AsLong(Pytemp);

    Pytemp = PyTuple_GetItem(args,2);
#if PY_MAJOR_VERSION == 3
     PyObject * Idstr = PyUnicode_AsEncodedString(Pytemp, "utf-8","Error ~");
     char * p = PyBytes_AS_STRING(Idstr);
     ID    = SensorIdToLLong(p);
     Py_XDECREF(Idstr);
#else
    ID     = SensorIdToLLong(PyString_AsString(Pytemp));
#endif

#ifdef DEBUG
    printf("cv=%d DS_PIN=%d ID=%llX\n",conversion,DS_PIN,ID);fflush(stdout);
#endif

unsigned char SensorType= (unsigned char) (ID & 0xFF);
  if(SensorType != TYPE_MAX31850)
     {
        Py_INCREF(Py_None);
        return(Py_None);
     }

    if ((DS_PIN < 0) || (DS_PIN > 31))
      {
#ifdef DEBUG
    printf("DS_PIN=%d not in range\n",DS_PIN);fflush(stdout);
#endif
        Py_INCREF(Py_None);
        return(Py_None);
      }


  // clean Previous  Pins usage
 for(loop=0;loop<32;loop++)
   DS18B20_Pins[loop]=-1;


  // clean mask
  for(loop=0;loop<4;loop++)
   {
     ModeMaskInput[loop]=0xffffffffL;
     ModeMaskOutput[loop]=0;
   }


#ifdef DEBUG
    printf("Pin #%d\n",DS_PIN);fflush(stdout);
#endif

    DS18B20_Pins[0]=DS_PIN;
    temp = DS_PIN;
    PinMask |= 1 << temp;
    ModeMaskInput[temp/10] &= ~(7<<((temp % 10) * 3));
    ModeMaskOutput[temp/10] |= (1<<((temp % 10) * 3));

    if(conversion)
    {
      set_max_priority();
      DoReset();
      // start Acquisition
       WriteByte(DS18B20_SKIP_ROM,0);
       WriteByte(DS18B20_CONVERT_T,1);
       GPIO_SET= PinMask;
       SetOutputMode(); 
       GPIO_SET= PinMask;


       set_default_priority();

       //  wait  for the highest resolution probe
       usleep(AcquisitionDelay);
    }

    double value,value2;
    int  ErrorFlag;
    if(ReadSensor(ID,SensorType,&value,&value2,&ErrorFlag))
    {

         PyObject *tempList = PyList_New(3);
         PyList_SetItem(tempList,0,Py_BuildValue("f",value));
         PyList_SetItem(tempList,1,Py_BuildValue("f",value2));
         PyList_SetItem(tempList,2,Py_BuildValue("i",ErrorFlag));
//         Py_DECREF(tempList);
        return tempList;

    }

  Py_INCREF(Py_None);
  return(Py_None);
}
//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//   read
static PyObject* DS18B20_readTemperature(PyObject* self, PyObject* args)
{
 unsigned long long ID;
 int loop,temp;
 int conversion;

 long length = PyTuple_Size(args);

  if(length != 3)
  {
#ifdef DEBUG
   printf("(Length=%ld) != 3\n",length);
#endif
    Py_INCREF(Py_None);
    return(Py_None);
  }
    PyObject* Pytemp = PyTuple_GetItem(args,0);
    conversion = (int) PyLong_AsLong(Pytemp);

    Pytemp = PyTuple_GetItem(args,1);
    DS_PIN = (unsigned short) PyLong_AsLong(Pytemp);

    Pytemp = PyTuple_GetItem(args,2);
#if PY_MAJOR_VERSION == 3
     PyObject * Idstr = PyUnicode_AsEncodedString(Pytemp, "utf-8","Error ~");
     char * p = PyBytes_AS_STRING(Idstr);
     ID    = SensorIdToLLong(p);
     Py_XDECREF(Idstr);
#else
    ID     = SensorIdToLLong(PyString_AsString(Pytemp));
#endif

#ifdef DEBUG
    printf("cv=%d DS_PIN=%d ID=%llX\n",conversion,DS_PIN,ID);fflush(stdout);
#endif

unsigned char SensorType= (unsigned char) (ID & 0xFF);
    if ((DS_PIN < 0) || (DS_PIN > 31))
      {
#ifdef DEBUG
    printf("DS_PIN=%d not in range\n",DS_PIN);fflush(stdout);
#endif
        Py_INCREF(Py_None);
        return(Py_None);
      }


  // clean Previous  Pins usage
 for(loop=0;loop<32;loop++)
   DS18B20_Pins[loop]=-1;


  // clean mask
  for(loop=0;loop<4;loop++)
   {
     ModeMaskInput[loop]=0xffffffffL;
     ModeMaskOutput[loop]=0;
   }


#ifdef DEBUG
    printf("Pin #%d\n",DS_PIN);fflush(stdout);
#endif

    DS18B20_Pins[0]=DS_PIN;
    temp = DS_PIN;
    PinMask |= 1 << temp;
    ModeMaskInput[temp/10] &= ~(7<<((temp % 10) * 3));
    ModeMaskOutput[temp/10] |= (1<<((temp % 10) * 3));

    if(conversion)
    {
      set_max_priority();
      DoReset();
      // start Acquisition
       WriteByte(DS18B20_SKIP_ROM,0);
       GPIO_SET= PinMask;
       SetOutputMode(); 
       GPIO_SET= PinMask;
       usleep(10);
       WriteByte(DS18B20_CONVERT_T,1);
       // Force Output High
       // in case of parasitic mode
       GPIO_SET= PinMask;
       SetOutputMode(); 
       GPIO_SET= PinMask;

       set_default_priority();

       //  wait  for the highest resolution probe



       usleep(AcquisitionDelay);
    }

    double value,value2;
    int ErrorFlag;
    if(ReadSensor(ID,SensorType,&value,&value2,&ErrorFlag))
    {

        return (Py_BuildValue("f",value));

    }

  Py_INCREF(Py_None);
  return(Py_None);
}

//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//   scan
static PyObject* DS18B20_scan(PyObject* self, PyObject* args)
{
  int loop,temp;
#ifdef DEBUG
  printf("In DS18B20_scan\n");fflush(stdout);
#endif


  long length = PyTuple_Size(args);

  if(length != 1)
  {
    Py_INCREF(Py_None);
    return(Py_None);
  }

    PyObject* Pytemp = PyTuple_GetItem(args,0);
    long elem = PyLong_AsLong(Pytemp);
    if ((elem < 0) || (elem > 31))
      {
        Py_INCREF(Py_None);
        return(Py_None);
      }

  // clean Previous  Pins usage
 for(loop=0;loop<32;loop++)
   DS18B20_Pins[loop]=-1;


  // clean mask
  for(loop=0;loop<4;loop++)
   {
     ModeMaskInput[loop]=0xffffffffL;
     ModeMaskOutput[loop]=0;
   }


#ifdef DEBUG
    printf("Pin #%ld\n",elem);fflush(stdout);
#endif

    DS18B20_Pins[0]=elem;
    temp = elem;
    PinMask |= 1 << temp;
    ModeMaskInput[temp/10] &= ~(7<<((temp % 10) * 3));
    ModeMaskOutput[temp/10] |= (1<<((temp % 10) * 3));


     DS_PIN= elem;

     PyObject * sensorList = PyList_New(0);
     ScanForSensor(sensorList);

     return(sensorList);
}

//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//   [] readScratchPad(pin,ID)

static PyObject* DS18B20_readScratchPad(PyObject* self, PyObject* args)
{
 unsigned long long ID;
 int loop,temp;

 long length = PyTuple_Size(args);

  if(length != 2)
  {
#ifdef DEBUG
    printf("(Length=%ld) != 3\n",length);
#endif
    Py_INCREF(Py_None);
    return(Py_None);
  }

    PyObject* Pytemp = PyTuple_GetItem(args,0);
    DS_PIN = (unsigned short) PyLong_AsLong(Pytemp);

    Pytemp = PyTuple_GetItem(args,1);


#if PY_MAJOR_VERSION == 3
     PyObject * Idstr = PyUnicode_AsEncodedString(Pytemp, "utf-8","Error ~");
     char * p = PyBytes_AS_STRING(Idstr);
     ID    = SensorIdToLLong(p);
     Py_XDECREF(Idstr);
#else
    ID     = SensorIdToLLong(PyString_AsString(Pytemp));
#endif





#ifdef DEBUG
    printf("DS_PIN=%d ID=%llX ",DS_PIN,ID);fflush(stdout);
#endif


    if ((DS_PIN < 0) || (DS_PIN > 31))
      {
#ifdef DEBUG
    printf("DS_PIN=%d not in range\n",DS_PIN);fflush(stdout);
#endif
        Py_INCREF(Py_None);
        return(Py_None);
      }


  // clean Previous  Pins usage
 for(loop=0;loop<32;loop++)
   DS18B20_Pins[loop]=-1;

  // clean mask
  for(loop=0;loop<4;loop++)
   {
     ModeMaskInput[loop]=0xffffffffL;
     ModeMaskOutput[loop]=0;
   }


#ifdef DEBUG
    printf("Pin #%d\n",DS_PIN);fflush(stdout);
#endif

    DS18B20_Pins[0]=DS_PIN;
    temp = DS_PIN;
    PinMask |= 1 << temp;
    ModeMaskInput[temp/10] &= ~(7<<((temp % 10) * 3));
    ModeMaskOutput[temp/10] |= (1<<((temp % 10) * 3));


  DoReset();
  SelectSensor(ID);
  ReadScratchPad();

  PyObject *reg = PyList_New(0);
  for(loop=0;loop<9;loop++)
  {
   PyList_Append(reg, Py_BuildValue("b",(unsigned char) ScratchPad[loop]));
  }

  return reg;


}

//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//   getResolution(pin,ID)
static PyObject* DS18B20_getResolution(PyObject* self, PyObject* args)
{

 unsigned long long ID;
 int resolution,loop,temp;

 long length = PyTuple_Size(args);

  if(length != 2)
  {
#ifdef DEBUG
    printf("(Length=%ld) != 3\n",length);
#endif
    Py_INCREF(Py_None);
    return(Py_None);
  }

    PyObject* Pytemp = PyTuple_GetItem(args,0);
    DS_PIN = (unsigned short) PyLong_AsLong(Pytemp);

    Pytemp = PyTuple_GetItem(args,1);
#if PY_MAJOR_VERSION == 3
     PyObject * Idstr = PyUnicode_AsEncodedString(Pytemp, "utf-8","Error ~");
     char * p = PyBytes_AS_STRING(Idstr);
     ID    = SensorIdToLLong(p);
     Py_XDECREF(Idstr);
#else
    ID     = SensorIdToLLong(PyString_AsString(Pytemp));
#endif



unsigned char SensorType = (unsigned char) (ID & 0xff);


if(SensorType != TYPE_DS18B20)
 {
   Py_INCREF(Py_None);
   return Py_None;
 }



#ifdef DEBUG
    printf("DS_PIN=%d ID=%llX ",DS_PIN,ID);fflush(stdout);
#endif


    if ((DS_PIN < 0) || (DS_PIN > 31))
      {
#ifdef DEBUG
    printf("DS_PIN=%d not in range\n",DS_PIN);fflush(stdout);
#endif
        Py_INCREF(Py_None);
        return(Py_None);
      }


  // clean Previous  Pins usage
 for(loop=0;loop<32;loop++)
   DS18B20_Pins[loop]=-1;

  // clean mask
  for(loop=0;loop<4;loop++)
   {
     ModeMaskInput[loop]=0xffffffffL;
     ModeMaskOutput[loop]=0;
   }


#ifdef DEBUG
    printf("Pin #%d\n",DS_PIN);fflush(stdout);
#endif

    DS18B20_Pins[0]=DS_PIN;
    temp = DS_PIN;
    PinMask |= 1 << temp;
    ModeMaskInput[temp/10] &= ~(7<<((temp % 10) * 3));
    ModeMaskOutput[temp/10] |= (1<<((temp % 10) * 3));


  DoReset();
  SelectSensor(ID);
  ReadScratchPad();

  switch(ScratchPad[4])
  {
    case 0x1f:  resolution=9;break;
    case 0x3f:  resolution=10;break;
    case 0x5f:  resolution=11;break;
    default:  resolution=12;break;
  }

  return Py_BuildValue("i",resolution);

}






//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//   setResolution(pins,ID, new Resolution)
static PyObject* DS18B20_setResolution(PyObject* self, PyObject* args)
{
 unsigned long long ID;
 int resolution,loop,temp;

 long length = PyTuple_Size(args);

  if(length != 3)
  {
#ifdef DEBUG
    printf("(Length=%ld) != 3\n",length);
#endif
    Py_INCREF(Py_None);
    return(Py_None);
  }
    PyObject* Pytemp = PyTuple_GetItem(args,0);
    DS_PIN = (unsigned short) PyLong_AsLong(Pytemp);

    Pytemp = PyTuple_GetItem(args,1);
#if PY_MAJOR_VERSION == 3
     PyObject * Idstr = PyUnicode_AsEncodedString(Pytemp, "utf-8","Error ~");
     char * p = PyBytes_AS_STRING(Idstr);
     ID    = SensorIdToLLong(p);
     Py_XDECREF(Idstr);
#else
    ID     = SensorIdToLLong(PyString_AsString(Pytemp));
#endif

    Pytemp = PyTuple_GetItem(args,2);
    resolution = (int) PyLong_AsLong(Pytemp);

unsigned char SensorType = (unsigned char) (ID & 0xff);


if(SensorType != TYPE_DS18B20)
 {
   Py_INCREF(Py_None);
   return Py_None;
 }

#ifdef DEBUG
    printf("DS_PIN=%d ID=%llX resolution=%d\n",DS_PIN,ID,resolution);fflush(stdout);
#endif


    if ((DS_PIN < 0) || (DS_PIN > 31))
      {
#ifdef DEBUG
    printf("DS_PIN=%d not in range\n",DS_PIN);fflush(stdout);
#endif
        Py_INCREF(Py_None);
        return(Py_None);
      }


  // clean Previous  Pins usage
 for(loop=0;loop<32;loop++)
   DS18B20_Pins[loop]=-1;

  // clean mask
  for(loop=0;loop<4;loop++)
   {
     ModeMaskInput[loop]=0xffffffffL;
     ModeMaskOutput[loop]=0;
   }


#ifdef DEBUG
    printf("Pin #%d\n",DS_PIN);fflush(stdout);
#endif

    DS18B20_Pins[0]=DS_PIN;
    temp = DS_PIN;
    PinMask |= 1 << temp;
    ModeMaskInput[temp/10] &= ~(7<<((temp % 10) * 3));
    ModeMaskOutput[temp/10] |= (1<<((temp % 10) * 3));

    unsigned short config;
   switch(resolution)
  {
    case 9:  config=0x1f;break;
    case 10: config=0x3f;break;
    case 11: config=0x5f;break;
    default:  // 12 bits
             config=0x7f;
  }

  DoReset();
  SelectSensor(ID);
  ReadScratchPad();
  DoReset();
  SelectSensor(ID);
  WriteScratchPad(ScratchPad[2],ScratchPad[3],config);
  DoReset();
  WriteByte(DS18B20_COPY_SCRATCHPAD,1);
  usleep(10000);
  SetInputMode();

  Py_INCREF(Py_None);
  return(Py_None);
}

//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//   setAquisitionDelay
static PyObject* DS18B20_setAcquisitionDelay(PyObject* self, PyObject* args)
{
//     long length = PyTuple_Size(args);

//  if(length ==1)
//  {

//  PyObject *  conv = PyTuple_GetItem(args,0);
  AcquisitionDelay = PyLong_AsLong(args);
//  }

#ifdef DEBUG
  printf("Acquisition Delay set to %lu\n",AcquisitionDelay);
#endif


  Py_INCREF(Py_None);
  return(Py_None);
}

//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//   getAquisitionDelay

static PyObject* DS18B20_getAcquisitionDelay(PyObject* self, PyObject* args)
{
  return PyLong_FromUnsignedLong(AcquisitionDelay);  
}

//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//   version()

static PyObject * DS18B20_version(PyObject* self, PyObject* args)
{
   return Py_BuildValue("f",2.02);
}




//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//   pinsRead(StartConversion, [pins list])
static PyObject* DS18B20_pinsReadTemperature(PyObject* self, PyObject* args)
{
   int loop;

#ifdef DEBUG
  printf("In DS18B20_readTemperature\n");fflush(stdout);
#endif

  // clean Previous  Pins usage
 for(loop=0;loop<32;loop++)
   DS18B20_Pins[loop]=-1;


  // clean mask
  for(loop=0;loop<4;loop++)
   {
     ModeMaskInput[loop]=0xffffffffL;
     ModeMaskOutput[loop]=0;
   }



  //  get number of pins and set mask

  long length = PyTuple_Size(args);

  if(length != 2)
  {
    Py_INCREF(Py_None);
    return(Py_None);
  }



#ifdef DEBUG
   printf("%ld Arguments\n",length);fflush(stdout);
#endif

  int i;
  int temp;
  int PinCount =0;
  PinMask = 0;

  PyObject *  conv = PyTuple_GetItem(args,0);
  long conversion = PyLong_AsLong(conv); 
#ifdef DEBUG
 printf("Conversion = %ld\n",conversion);fflush(stdout);
#endif

  PyObject * Plist = PyTuple_GetItem(args,1);

  length = PyList_Size(Plist);

  if(length < 1)
  {
    Py_INCREF(Py_None);
    return(Py_None);
  }



  for(i=0;i<length;i++)
  {
    PyObject* Pytemp = PyList_GetItem(Plist,i);
    long elem = PyLong_AsLong(Pytemp);
    if(elem < 0) continue;
    if(elem > 31) continue;

#ifdef DEBUG
    printf("Pin #%ld\n",elem);fflush(stdout);
#endif

    DS18B20_Pins[PinCount++]=elem;
    temp = elem;
    PinMask |= 1 << temp;
    ModeMaskInput[temp/10] &= ~(7<<((temp % 10) * 3));
    ModeMaskOutput[temp/10] |= (1<<((temp % 10) * 3));

  }


   // ok now it's time to read the sensor in bulk
      clock_gettime(CLOCK_MONOTONIC,&myacqstart);

     if(conversion)
     {
      set_max_priority();
      DoReset();
      // start Acquisition
       WriteByte(DS18B20_SKIP_ROM,0);
       WriteByte(DS18B20_CONVERT_T,1);

       set_default_priority();

       //  wait  for the highest resolution probe
       usleep(AcquisitionDelay);
      }

       ReadSensors();


       // now let's print result

       clock_gettime(CLOCK_MONOTONIC,&myend);

#ifdef DEBUG
       printf("====\n%.3f sec  acquisition time = %.3f sec\n",clock_diff(mystart, myacqstart),clock_diff(myacqstart,myend));
       fflush(stdout);
#endif


       PyObject *lst = PyList_New(PinCount);

       for(loop=0;loop<PinCount;loop++)
       {

#ifdef DEBUG
         printf("GPIO %d : ",DS18B20_Pins[loop]);
         fflush(stdout);
#endif
         PyObject *sensorList = PyList_New(2);

         PyList_SetItem(sensorList,0,Py_BuildValue("i",DS18B20_Pins[loop]));

         if(DS18B20_Data[loop].valid)
          {
#ifdef DEBUG
           printf("%02d bits  Temperature: %6.2f +/- %4.2f Celsius\n", DS18B20_Data[loop].resolution ,DS18B20_Data[loop].temperature, 0.0625 * (double)  (1<<(12 - DS18B20_Data[loop].resolution)));
#endif
           PyList_SetItem(sensorList,1,Py_BuildValue("f",DS18B20_Data[loop].temperature));
          }
         else
          {
           Py_INCREF(Py_None);
           PyList_SetItem(sensorList,1,Py_None);
           Py_DECREF(Py_None);
#ifdef DEBUG
          printf("Bad CRC!\n");
          fflush(stdout);
#endif
          }
        PyList_SetItem(lst,loop,Py_BuildValue("O",sensorList));
        Py_DECREF(sensorList);
       }
         return(lst);
}


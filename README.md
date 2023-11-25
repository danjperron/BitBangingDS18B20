BitBangingDS18B20
=================

N.B.  this works on 32bit OS.

<b> Raspberry Pi 5 *** update *** works on 64bits using gpiod</b><br>
You will need to install gpiod<br>
sudo apt-get install gpiod libgpiod-dev libgpiod-doc<br>

 - DS18B20Pi5Scan.c &nbsp;&nbsp;&nbsp;&nbsp;Application to connect multiple DS18B20 on one GPIO.
   (I need to check if it works on others Pi and on 32 bits)
 - Python module, configDS18B20Pi5.c and DS18B20Pi5V2.c to be done later

<b> OLD 32bits method</b><br>
Method to access the DS18B20 sensor using Rapsberry Pi GPIO

 - configDS18B20.c &nbsp;&nbsp;Application to set bit resolution  of the DS18B20.
 - DS18B20Scan.c &nbsp;&nbsp;&nbsp;&nbsp;Application to connect multiple DS18B20 on one GPIO.
 - DS18B20V2.c &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Application to Connect one DS18B20 per GPIO but read them together.

    to compile
    
        gcc -lrt -o configDS18B20 configDS18B20.c
        gcc -lrt -o DS18B20Scan   DS18B20Scan.c
        gcc -lrt -o DS18B20V2  DS18B20V2.c

Python Add-on

    To install
        sudo apt-get install python-dev
        sudo apt-get install python3-dev
        cd python
        sudo python setup.py install
        sudo python3 setup.py install

    Example
	DS_Array.py     Read 30 sensors in less than one second using Tkinter

    Help inside module
	python
	help('DS18B20')


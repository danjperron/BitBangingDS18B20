BitBangingDS18B20
=================

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
	cd python
        sudo python setup.py install
        sudo python3 setup.py install


    Don't forget to run the module using sudo!
    Help inside module
	sudo python
	help('DS18B20')


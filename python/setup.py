from distutils.core import setup, Extension

module1 = Extension('DS18B20',
                     sources = ['DS18B20.c'])

setup (name = 'DS18B20',
       version = '1.0',
       description= 'Read the DS18B20 using any GPIO in user space',
       ext_modules= [module1])


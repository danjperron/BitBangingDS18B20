#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    Daniel Perron  June 2019.
    demonstration of 30 DS18B20 sensor  read  in user mode
    in less than 1 second when resolution is set to 9 bits.
    DS18B20 Module is available in github
    https://github.com/danjperron/BitBangingDS18B20
"""


try:
    # for Python2
    import Tkinter  as tk
except ImportError:
    # for Python3
    import tkinter as tk

try:
  import tkFont
except ImportError:
  import tkinter.font as tkFont

import signal
import time
import DS18B20 as ds



"""
Copyright (c) <2019> <Daniel Perron>
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
"""




DS_PIN=21

#offset for minimum temp
minTemp = 20.0
#gain factor  color[temperature * gainFactor]  100 levels
gainFactor = 2;


#ds18b20 sensor
sensorTable = [ '28-0217C1146FFF', '28-0117C17B2DFF', '28-0217C13A79FF', '28-0117C19740FF', '28-0117C19E86FF',
           '28-0217C10009FF', '28-0117C17A75FF', '28-0217C0FB8BFF', '28-0217C1129CFF', '28-0217C1B8CDFF',
           '28-0117C19757FF', '28-0217C0FD79FF', '28-0217C1283EFF', '28-0117C1747EFF', '28-0117C16783FF',
           '28-0117C140A6FF', '28-0217C11EE0FF', '28-0117C16F35FF', '28-0217C14203FF', '28-0217C1B35BFF',
           '28-0117C190E7FF', '28-0117C17799FF', '28-0117C14282FF', '28-0117C17354FF', '28-0117C1757EFF',
           '28-0117C1AFC0FF', '28-0217C0FCEBFF', '28-0217C1426EFF', '28-0217C12228FF', '28-0117C16769FF']

nbSensorRow = 6
nbSensorCol = 5

sensordelay = { 9 : 100 , 10 : 200 , 11 : 375 , 12 : 750}


class sensorRectangle:
   def __init__(self, canvas, x, y , rectSize):
       self.canvas = canvas
       self.x = x
       self.y = y
       self.rectSize = rectSize
       posX = x * rectSize
       posY = y * rectSize
       self.rect= canvas.create_rectangle(posX,posY,posX+rectSize,posY+rectSize,fill = '#ffffff',width=0)
       self.temp = canvas.create_text(posX + rectSize//2, posY + rectSize//2,text="  0.00",fill='white',font=("TkDefaultFont",14))

   def changeColor(self,color):
       self.canvas.itemconfig(self.rect,fill=color)

   def changeTemp(self, temp):
       if temp is None:
         str_temp = "N.C."
       else:
         str_temp = "{:05.2f}\u00b0C".format(temp)
       self.canvas.itemconfig(self.temp, text = str_temp)


class App():


   def __init__(self):
       self.sensorTimer=0
       self.root = tk.Tk()
       self.root.wm_title('Temperature array')
       #self.root.protocol('WM_DELETE_WINDOW', self.closeEverything)
       self.canvas = tk.Canvas(self.root, width=600, height = 600)
       self.canvas.pack()

       #create rectangle
       self.myrect=[]
       for i in  range(len(sensorTable)):
          self.myrect.append( sensorRectangle(self.canvas, i % 5 , i // 5, 100))

       self.setColorTable()

       #bit resolution
       self.resolution = 10
       self.setSensorResolution(self.resolution)
       self.startConversion()


   def setColorTable(self):
       #create color table
       self.colorTable=[]
       target= [ [0x4b,0x0,0x82],[0,0,0xff],[0,0x80,0],[0xff,0xff,0],[0xff,0xA5,0],[0xff,0,0],[0xff,0,0]]
       for i in range(6):
           scaleR = (target[i+1][0] - target[i][0]) / 20.0
           scaleG = (target[i+1][1] - target[i][1]) / 20.0
           scaleB = (target[i+1][2] - target[i][2]) / 20.0
           for j in range(20):
               k = i*20+j
               R= int(target[i][0] +   scaleR * j)
               G= int(target[i][1] +   scaleG * j)
               B= int(target[i][2] +   scaleB * j)
               color= self.from_rgb(R,G,B)
               self.colorTable.append(color)
               self.canvas.create_rectangle(570,50+(100-k)*5,598,55+(100-k)*5,width=0,fill=color)
               #add scale
               if (k % 10)== 0:
                   scale = "{:5.1f}\u00b0C -".format(minTemp + (k / gainFactor))
                   self.canvas.create_text(543,52+(100-k)*5,text=scale,fill='black')
               if i == 5 :
                  break

   def setSensorResolution(self,resolution):
       for i in sensorTable:
          #check the resolution first
          res = ds.getResolution(DS_PIN,i)
          if res != resolution:
             ds.setResolution(DS_PIN,i,resolution)



   def from_rgb_int(self,rgb):
       return "#%02x%02x%02x" % rgb

   def from_rgb(self, r, g ,b):
       return "#%02x%02x%02x" % (r, g, b)


   def updateSensors(self):
       for i in range(len(sensorTable)):
          x = i % nbSensorCol
          y = i // nbSensorCol
          Temp = ds.read(False,DS_PIN,sensorTable[i])
          if Temp is None:
             self.myrect[i].changeColor('black')
          else:
             T = Temp - minTemp;
             T = T * gainFactor;
             T = int(T)
             if T < 0 :
                T = 0
             if T >= len(self.colorTable):
                T = len(self.colorTable)-1
             self.myrect[i].changeColor(self.colorTable[T])
          self.myrect[i].changeTemp(Temp)
       self.root.after(1,self.startConversion)

   def startConversion(self):
       #start sensor conversion
       ds.pinsStartConversion([DS_PIN])
       self.root.after(sensordelay[self.resolution],self.updateSensors)


   def SensorColor(self, x, y , color):
       x1 = x * 100
       y1 = y * 100
       self.canvas.create_rectangle(x1,y1,x1+100,y1+100,fill = color,width=0)



   def mainloop(self):
       self.root.mainloop()

app=App()


def signal_handler(signal, frame):
    app.root.quit()

signal.signal(signal.SIGINT,signal_handler)


app.mainloop()

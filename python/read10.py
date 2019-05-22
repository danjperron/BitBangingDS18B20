#!/usr/bin/env python3
import DS18B20 as ds
import datetime as dt
import time

sensorPin = 20


print("Scan sensors on pin {} ".format(sensorPin),end="")
time1= dt.datetime.now()
sensorsId = ds.scan(sensorPin)
delta_t= dt.datetime.now()-time1
print(" ... {} second.  Found {} sensors.".format(delta_t.total_seconds(),len(sensorsId)))


print("Send start of conversion commands for all sensors on pin {}.".format(sensorPin),end="")
time2 = dt.datetime.now()
ds.pinsStartConversion([sensorPin])
delta_t= dt.datetime.now()-time2
print(" ... {} second".format(delta_t.total_seconds()))

print("wait 0.75 seconds")
time.sleep(1.0)

print("Read all sensors", end="")
time2 = dt.datetime.now()
temperature =[]
for s in sensorsId:
  value = ds.read(False, sensorPin, s)
  temperature.append(value)
delta_t= dt.datetime.now() - time2
print(" ... {} second".format(delta_t.total_seconds()))

for i in range(len(sensorsId)):
    ends = "\t"
    if (i % 4) == 3 :
       ends= "\n"
    print("[{}] : {:.1f}'C".format(sensorsId[i],temperature[i]),end=ends)
print("")


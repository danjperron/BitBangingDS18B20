import time
import DS18B20 as DS

sensors = DS.scan(20)

try:
  while(True):
    DS.pinsStartConversion([20])
    time.sleep(0.75)
    for i in sensors:
      print("{:.3f}".format(DS.read(False,20,i)),end=" ")
    print(" ")
except KeyboardInterrupt:
   quit()


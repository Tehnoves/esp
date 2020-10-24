#-----------------------------------------
#  c.py обработка энкодера начало 22.10.20
#  24.10.20 V1.2 все работает
# 
#
#
#-----------------------------------------

from machine import Pin
import time

i = 0
new =1
new2 = 1
fist = 0
fist2 = 0
start = time.ticks_ms()
start2 = time.ticks_ms()
p5 = Pin(5, Pin.IN, Pin.PULL_UP)
p4 = Pin(4, Pin.IN, Pin.PULL_UP)
while True:
    #p5 = Pin(5, Pin.IN, Pin.PULL_UP)
    #print("new: ",new)
    #print(p5.value())
    if p4.value() != new:
        
        print("old: ",p4.value())
        if  p4.value() == 0:
            time.sleep_ms(2)
            print("---------------------")
            
            start = time.ticks_ms()
            fist = 1
        if p4.value() == 1 and fist == 1 :   
            fist = 0
            delta = time.ticks_diff(time.ticks_ms(), start)
            print("delta = ",delta)
        #start = time.ticks_ms()
        new = p4.value()
        print("new: ",new)
    if p5.value() != new2:
        
        print("old2: ",p5.value())
        if  p5.value() == 0:
            time.sleep_ms(2)
            start2 = time.ticks_ms()
            fist2 = 1
        if p5.value() == 1 and fist2 == 1 :   
            fist2 = 0
            delta2 = time.ticks_diff(time.ticks_ms(), start2)
            period = time.ticks_diff(time.ticks_ms(), start)
            if (period - delta2) > 0:
                print("вправо ==>")
                i += 1
            else:
                print("влево <==")    
                i -= 1
            print("обороты : ", i )   
            print("delta2 = ",delta2, " period: ", period)
            print("++++++++++++++++++++")
        #start = time.ticks_ms()
        new2 = p5.value()
        print("new2: ",new2)


    
        


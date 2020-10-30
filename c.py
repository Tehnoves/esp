#-----------------------------------------
#  c.py обработка энкодера начало 22.10.20
#  24.10.20 V1.2 все работает
# 			V1.3 есть пркрывания
#  30.10.20 перенос Pin  
#
#-----------------------------------------

from machine import Pin
import time

def encoder () :
	global new, end, start,fist,delta, new2,start2, fist2, delta2, i
	while end == 0 :
		if p0.value() != new:
			
			print("old: ",p0.value())
			if  p0.value() == 0:
				time.sleep_ms(2)
				print("---------------------")
				
				start = time.ticks_ms()
				fist = 1
			if p0.value() == 1 and fist == 1 :   
				fist = 0
				delta = time.ticks_diff(time.ticks_ms(), start)
				print("delta = ",delta)
			#start = time.ticks_ms()
			new = p0.value()
			print("new: ",new)
		if p2.value() != new2:
			
			print("old2: ",p2.value())
			if  p2.value() == 0:
				time.sleep_ms(2)
				start2 = time.ticks_ms()
				fist2 = 1
			if p2.value() == 1 and fist2 == 1 :   
				fist2 = 0
				delta2 = time.ticks_diff(time.ticks_ms(), start2)
				period = time.ticks_diff(time.ticks_ms(), start)
				if (period - delta2) > 0:
					print("вправо ==>")
					i += 1
				else:
					print(" <==  влево")    
					i -= 1
				print("обороты : ", i )   
				print("delta2 = ",delta2, " period: ", period)
				print("++++++++++++++++++++")
				end = 1
			#start = time.ticks_ms()
			new2 = p2.value()
			print("new2: ",new2)



end = 0
i = 0
new =1
new2 = 1
fist = 0
fist2 = 0
start = time.ticks_ms()
start2 = time.ticks_ms()
p2 = Pin(2, Pin.IN, Pin.PULL_UP)
p0 = Pin(0, Pin.IN, Pin.PULL_UP)

while True:
    if end == 1:
        end = 0
    p0.irq(handler= encoder(), trigger = Pin.IRQ_FALLING )
    p2.irq(handler= encoder(), trigger = Pin.IRQ_FALLING )
	

    
        


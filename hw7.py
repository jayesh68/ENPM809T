import RPi.GPIO as gpio
import numpy as np
import time
import matplotlib.pyplot as plt


gpio.setmode(gpio.BOARD)
gpio.setup(31,gpio.OUT)
gpio.setup(33,gpio.OUT)
gpio.setup(35,gpio.OUT)
gpio.setup(37,gpio.OUT)

gpio.setup(7,gpio.IN,pull_up_down=gpio.PUD_UP)
gpio.setup(12,gpio.IN,pull_up_down=gpio.PUD_UP)



def gameover():
	#Set all pins low
	gpio.output(31,False)
	gpio.output(33,False)
	gpio.output(35,False)
	gpio.output(37,False)



def forward():
	counterBR=np.uint64(0)
	counterFL=np.uint64(0)

	buttonBR=int(0)
	buttonFL=int(0)

	pwm1=gpio.PWM(31,50) #left
	pwm2=gpio.PWM(37,50) #right
	val=32
	pwm1.start(val)
	pwm2.start(val)
	time.sleep(0.1)

	while(True):

		print("Forward","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))

		if int(gpio.input(12))!=int(buttonBR):
			buttonBR=int(gpio.input(12))
			counterBR+=1
			#print(counterBR)

		if int(gpio.input(7))!=int(buttonFL):
			buttonFL=int(gpio.input(7))
			counterFL+=1
			#print(counterFL)

		if counterBR>=214:
			pwm2.stop()

		if counterFL>=214:
			pwm1.stop()

		if counterBR>=214 and counterFL>=214:
			#gameover()
			#print("Thanks for playing")
			break

def reverse():
	counterBR=np.uint64(0)
	counterFL=np.uint64(0)

	buttonBR=int(0)
	buttonFL=int(0)

	pwm1=gpio.PWM(33,50) #left
	pwm2=gpio.PWM(35,50) #right
	val=22
	pwm1.start(40)
	pwm2.start(40)
	time.sleep(0.1)

	while(True):

		print("Reverse","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))

		if int(gpio.input(12))!=int(buttonBR):
			buttonBR=int(gpio.input(12))
			counterBR+=1
			#print(counterBR)

		if int(gpio.input(7))!=int(buttonFL):
			buttonFL=int(gpio.input(7))
			counterFL+=1
			#print(counterFL)

		if counterBR>=98:
			pwm2.stop()

		if counterFL>=98:
			pwm1.stop()

		if counterBR>=98 and counterFL>=98:
			#gameover()
			#print("Thanks for playing")
			break

def pivot_left():
	counterBR=np.uint64(0)
	counterFL=np.uint64(0)

	buttonBR=int(0)
	buttonFL=int(0)

	pwm1=gpio.PWM(33,50) #left
	pwm2=gpio.PWM(37,50) #right
	val=22
	pwm1.start(100)
	pwm2.start(100)
	time.sleep(0.1)

	while(True):

		print("Pivot Left","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))

		if int(gpio.input(12))!=int(buttonBR):
			buttonBR=int(gpio.input(12))
			counterBR+=1
			#print(counterBR)

		if int(gpio.input(7))!=int(buttonFL):
			buttonFL=int(gpio.input(7))
			counterFL+=1
			#print(counterFL)

		if counterBR>=20:
			pwm2.stop()

		if counterFL>=20:
			pwm1.stop()

		if counterBR>=20 and counterFL>=20:
			#gameover()
			#print("Thanks for playing")
			break
		
def pivot_right():
	counterBR=np.uint64(0)
	counterFL=np.uint64(0)

	buttonBR=int(0)
	buttonFL=int(0)

	pwm1=gpio.PWM(31,50) #left
	pwm2=gpio.PWM(35,50) #right
	val=22
	pwm1.start(100)
	pwm2.start(100)
	time.sleep(0.1)

	while(True):

		print("Pivot Right","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))

		if int(gpio.input(12))!=int(buttonBR):
			buttonBR=int(gpio.input(12))
			counterBR+=1
			#print(counterBR)

		if int(gpio.input(7))!=int(buttonFL):
			buttonFL=int(gpio.input(7))
			counterFL+=1
			#print(counterFL)

		if counterBR>=40:
			pwm2.stop()

		if counterFL>=40:
			pwm1.stop()

		if counterBR>=40 and counterFL>=40:
			#gameover()
			#print("Thanks for playing")
			break

forward()

#everse()

#pivot_right()

#pivot_left()
gameover()
gpio.cleanup()

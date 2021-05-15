import RPi.GPIO as gpio
import time

def init():
	gpio.setmode(gpio.BOARD)
	gpio.setup(31,gpio.OUT)
	gpio.setup(33,gpio.OUT)
	gpio.setup(35,gpio.OUT)
	gpio.setup(37,gpio.OUT)

def gameover():
	#Set all pins low
	gpio.output(31,False)
	gpio.output(33,False)
	gpio.output(35,False)
	gpio.output(37,False)

def forward(tf):
	init()
	#Left Wheels
	gpio.output(31,True)
	gpio.output(33,False)
	#Right wheels
	gpio.output(35,False)
	gpio.output(37,True)
	#Wait
	time.sleep(tf)
	#Send all pins low and cleanup
	gameover()
	gpio.cleanup()

def reverse(tf):
	init()
	#Left Wheels
	gpio.output(31,False)
	gpio.output(33,True)
	#Right wheels
	gpio.output(35,True)
	gpio.output(37,False)
	#Wait
	time.sleep(tf)
	#Send all pins low and cleanup
	gameover()
	gpio.cleanup()

def right(tf):
	init()
	#Left Wheels
	gpio.output(31,True)
	gpio.output(33,False)
	#Right wheels
	gpio.output(35,True)
	gpio.output(37,True)
	#Wait
	time.sleep(tf)
	#Send all pins low and cleanup
	gameover()
	gpio.cleanup()

def left(tf):
	init()
	#Left Wheels
	gpio.output(31,False)
	gpio.output(33,False)
	#Right wheels
	gpio.output(35,False)
	gpio.output(37,True)
	#Wait
	time.sleep(tf)
	#Send all pins low and cleanup
	gameover()
	gpio.cleanup()

def pivotleft(tf):
	init()
	#Left Wheels
	gpio.output(31,False)
	gpio.output(33,True)
	#Right wheels
	gpio.output(35,False)
	gpio.output(37,True)
	#Wait
	time.sleep(tf)
	#Send all pins low and cleanup
	gameover()
	gpio.cleanup()

def pivotright(tf):
	init()
	#Left Wheels
	gpio.output(31,True)
	gpio.output(33,False)
	#Right wheels
	gpio.output(35,True)
	gpio.output(37,False)
	#Wait
	time.sleep(tf)
	#Send all pins low and cleanupn
	gameover()
	gpio.cleanup()

def key_input(event):
	init()
	print("Key:",event)
	key_press=event
	tf=2
	if key_press.lower() == 'f':
		forward(tf)
	elif key_press.lower() == 'r':
		reverse(tf)
	elif key_press.lower() == 's':
		right(tf)
	elif key_press.lower() == 'l':
		left(tf)
	elif key_press.lower() == 'p':
		pivotright(tf)
	elif key_press.lower() == 'q':
		pivotleft(tf)
	else:
		print("Invalid key pressed")

while True:
	key_press=input("Select driving mode:")
	if key_press=='a':
		break
	key_input(key_press)

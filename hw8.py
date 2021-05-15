import RPi.GPIO as gpio
import numpy as np
import time
import matplotlib.pyplot as plt
import serial

gpio.setmode(gpio.BOARD)
gpio.setup(31,gpio.OUT)
gpio.setup(33,gpio.OUT)
gpio.setup(35,gpio.OUT)
gpio.setup(37,gpio.OUT)
start=[35,40]
traj=[]
traj1=[]
traj.append(start[0])
traj1.append(start[1])
gpio.setup(7,gpio.IN,pull_up_down=gpio.PUD_UP)
gpio.setup(12,gpio.IN,pull_up_down=gpio.PUD_UP)
curr_ang=0
def gameover():
	#Set all pins low
	gpio.output(31,False)
	gpio.output(33,False)
	gpio.output(35,False)
	gpio.output(37,False)
	
def forward(lim,ser):
	global curr_ang
	counterBR=np.uint64(0)
	counterFL=np.uint64(0)

	buttonBR=int(0)
	buttonFL=int(0)
	t=time.time()
	pwm1=gpio.PWM(31,50) #left
	pwm2=gpio.PWM(37,50) #right
	val=40
	pwm1.start(val)
	pwm2.start(val)
	time.sleep(0.1)
	
	while(True):
		print("Forward","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))
		
		#curr_ang=float(line)	
		line=ser.readline()
		line=line.rstrip().lstrip()
		line=str(line)
		line=line.strip("'")
		line=line.strip("b'")
		curr_ang=float(line)
		print('curr_ang',curr_ang)
		if int(gpio.input(12))!=int(buttonBR):
			buttonBR=int(gpio.input(12))
			counterBR+=1
			print(counterBR)

		if int(gpio.input(7))!=int(buttonFL):
			buttonFL=int(gpio.input(7))
			counterFL+=1
			print(counterFL)

		if counterBR>=lim:
			pwm2.stop()

		if counterFL>=lim:
			pwm1.stop()

		if counterBR>=lim or counterFL>=lim:

			#traj.append(
			#gameover()
			#gpio.cleanup()
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
	pwm1.start(85)
	pwm2.start(85)
	time.sleep(0.08)

	#print("Pivot Left","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))

	if int(gpio.input(12))!=int(buttonBR):
		buttonBR=int(gpio.input(12))
		counterBR+=1
		#print(counterBR)

	if int(gpio.input(7))!=int(buttonFL):
		buttonFL=int(gpio.input(7))
		counterFL+=1
		#print(counterFL)
		
	#gameover()

#identify serial connection
ser=serial.Serial('/dev/ttyUSB0',9600)
count=0
new_ang=0
l=0
while True:
	if(ser.in_waiting > 0):
		count+=1
		#read serial stream
		line=ser.readline()
		
		if count > 10:
			#traj1.append(traj)
			print('forward first')
			forward(5,ser)
			start=[35,75]
			traj.append(start[0])
			traj1.append(start[1])
			time.sleep(0.9)
			
			line=ser.readline()
			line=line.rstrip().lstrip()
			line=str(line)
			line=line.strip("'")
			line=line.strip("b'")
			new_ang=float(line)
			diff=curr_ang-new_ang
			count1=0
			while 0<=(abs(curr_ang-new_ang))<=90 or 270<=(abs(curr_ang-new_ang))<=360:
				line=ser.readline()
				line=line.rstrip().lstrip()
				line=str(line)
				line=line.strip("'")
				line=line.strip("b'")
				new_ang=float(line)
				diff1=curr_ang-new_ang
				'''
				if diff1<=1:
					print('count1',count)
					count1+=1
				else:
				'''
				pivot_left()
				print(count1)
				print('angle diff',abs(curr_ang-new_ang))
				print('left1')
		
			print('current angle',curr_ang)
			print('new angle',new_ang)
			
			time.sleep(0.9)
			break
		
print('done')
gameover()
gpio.cleanup()
plt.grid()
plt.plot(traj,traj1)
plt.show()

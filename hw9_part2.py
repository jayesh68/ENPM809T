# import the necessary packages

from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as gpio
import time as t
import cv2
import numpy as np
import datetime
import serial
import time
import os
import io
import math
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from datetime import datetime
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

# initialize the Raspberry Pi camera
#camera = cv2.VideoCapture(0)
camera = PiCamera()
camera.rotation=180
camera.resolution = (640, 480)
camera.framerate = 2

rawCapture = PiRGBArray(camera, size=(640,480))

# allow the camera to warmup
t.sleep(0.1)
ser=serial.Serial('/dev/ttyUSB0',9600)
gpio.setmode(gpio.BOARD)
gpio.setup(31,gpio.OUT)
gpio.setup(33,gpio.OUT)
gpio.setup(35,gpio.OUT)
gpio.setup(37,gpio.OUT)
gpio.setup(36,gpio.OUT)
gpio.setup(16, gpio.OUT)
gpio.setup(18, gpio.IN)
#pwm1=gpio.PWM(31,50) #left
#pwm2=gpio.PWM(37,50) #right
pwm=gpio.PWM(36,50)
pwm.start(7)
gpio.setup(7,gpio.IN,pull_up_down=gpio.PUD_UP)
gpio.setup(12,gpio.IN,pull_up_down=gpio.PUD_UP)

curr_ang=0
curr_ang1=0
# define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('hw9final.avi', fourcc, 2, (640, 480))
count=0

def distance_range():	
	#Ensure output has no value
	gpio.output(16, False)
	time.sleep(0.01)

	#Generate trigger pulse
	gpio.output(16, True)
	time.sleep(0.00001)
	gpio.output(16, False)

	#Generate echo time signal
	while gpio.input(18) == 0:
		pulse_start = time.time()

	while gpio.input(18) == 1:
		pulse_end = time.time()

	pulse_duration = pulse_end - pulse_start

	#Convert time to distance
	distance = pulse_duration*17150
	distance = round(distance,2)

	return distance
	
def gameover():
	#Set all pins low
	gpio.output(31,False)
	gpio.output(33,False)
	gpio.output(35,False)
	gpio.output(37,False)

def pic_email():
	#Define time stamp and record an image
	print('about to send')
	pic_time=datetime.now().strftime('%Y%m%d%H%M%S')
	image=np.empty((480,640,3),dtype=np.uint8)
	camera.capture(image,'bgr')
	cv2.imwrite('objgrab.jpg',image)
	#Email information
	smtpUser = 'enpm809tjay@gmail.com'	
	smtpPass = 'enpm809tjay'

	#Destination email info
	toAdd = ['ENPM809TS19@gmail.com','skotasai@umd.edu']
	fromAdd = smtpUser
	subject = 'Name: Jayesh Object grabbed by robot at ' + pic_time
	msg=MIMEMultipart()
	msg['Subject']=subject
	msg['From']=fromAdd
	#msg['To']=toAdd
	msg['To']=",".join(toAdd)
	msg.preamble='Image recorded at' + pic_time

	#Email text
	body=MIMEText('avoided obstacles in cluttered environment, and grabbed object' + pic_time)
	msg.attach(body)
	print('adding body')
	#Attach image
	print('opening')
	fp=open('objgrab.jpg','rb')
	img=MIMEImage(fp.read())
	fp.close()
	msg.attach(img)
	print('sending mail')
	#Send email	
	s=smtplib.SMTP('smtp.gmail.com',587)

	s.ehlo()
	s.starttls()
	s.ehlo()

	s.login(smtpUser,smtpPass)
	s.sendmail(fromAdd,toAdd,msg.as_string())
	s.quit()
	key = cv2.waitKey(1) & 0xFF
	rawCapture.truncate()

	print("Email delivered")
	
def take_pic():
        #pwm1.stop()
	#pwm2.stop()
	stream=io.BytesIO()
	print('taking_pic')
	t.sleep(2)
	pic_name='pos1.jpg'
	#command='raspistill -w 640 -h 480 -vf -hf -o ' + pic_name
	#os.system(command)
	image=np.empty((480,640,3),dtype=np.uint8)
	camera.capture(image,'bgr')
	
	#camera.close()	
	#image=cv2.imread(output)
	rows,cols,chn=image.shape
	print(rows,cols,chn)
	blank_img=np.zeros(shape=[rows,cols,chn],dtype=np.uint8)

	hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        #cv2.imshow('hsv',hsv)

	lower_green = np.array([35,110,57])
	upper_green = np.array([48,200,180])
	
	blank_img = cv2.inRange(hsv,lower_green,upper_green)
	blank_img=cv2.erode(blank_img,None,iterations=3)
	blank_img=cv2.dilate(blank_img,None,iterations=3) 
	#cv2.imshow('mask',blank_img)

	contours,hierarchy = cv2.findContours(blank_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cx1=[]
	cy1=[]
	rad=[]	    

	if len(contours)>0:
                c=max(contours,key=cv2.contourArea)
                ((x,y),radius)=cv2.minEnclosingCircle(c)
                M=cv2.moments(c)
                if M["m00"]!=0:
                   #print('moments')
                   cx=int(M["m10"]/M["m00"])
                   cy=int(M["m01"]/M["m00"])
                #cx1.append(cx)
                #cy1.append(cy)
                else:
                   cx,cy=0,0
                center=(cx,cy)
		
                if radius > 20:
                   cv2.circle(image,center,int(radius),((0,255,255)),2)
                   cv2.imshow('Closer',image)
                else:
                   radius=0
                return radius
	else:
                return 0
	out.write(image)
	key = cv2.waitKey(1) & 0xFF
	rawCapture.truncate()
	
	
def front_distance(width,focal):
	print('calculating front distance')	
	pix=take_pic()
	if pix == 0:
		return 0
	else:
		return int(focal*width/pix)

def forward_obs(ser):
	counterBR=np.uint64(0)
	counterFL=np.uint64(0)
	print('moving forward')
	buttonBR=int(0)
	buttonFL=int(0)
	pwm1=gpio.PWM(31,50) #left
	pwm2=gpio.PWM(37,50) #right
	val=40
	#pwm1.start(val)
	#pwm2.start(val)
	time.sleep(0.1)

	#print("Forward","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))
	dist=distance_range()
	pwm1.start(val)
	pwm2.start(val)
	print('distance from obj',dist)
	time.sleep(0.1)
	while(dist >22):
		#print("Forward","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))
		dist=distance_range()
		time.sleep(0.5)
		pwm1.start(val)
		pwm2.start(val)
		print('distance from obj1',dist)
		while(True):
			#print('inside while')
			#print("Forward","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))				
			line=ser.readline()
			line=line.rstrip().lstrip()
			line=str(line)
			line=line.strip("'")
			line=line.strip("b'")
			curr_ang1=float(line)
			print('curr_ang',curr_ang)			
			if int(gpio.input(12))!=int(buttonBR):
				buttonBR=int(gpio.input(12))
				counterBR+=1
				#print(counterBR)

			if int(gpio.input(7))!=int(buttonFL):
				buttonFL=int(gpio.input(7))
				counterFL+=1
				#print(counterFL)
			
			if counterBR>3 or counterFL>=3:
				print('in stopping')
				counterBR=0
				counterFL=0
				pwm1.stop()
				pwm2.stop()	

				break	
	pwm1.stop()
	pwm2.stop()
	print('broken out of forward obstacle')

def pivot_left1():
	counterBR=np.uint64(0)
	counterFL=np.uint64(0)
	print('in left 1')
	buttonBR=int(0)
	buttonFL=int(0)

	pwm1=gpio.PWM(33,50) #left
	pwm2=gpio.PWM(37,50) #right
	val=22
	pwm1.start(85)
	pwm2.start(85)
	time.sleep(0.07)

	#print("Pivot Left","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))

	if int(gpio.input(12))!=int(buttonBR):
		buttonBR=int(gpio.input(12))
		counterBR+=1
		#print(counterBR)

	if int(gpio.input(7))!=int(buttonFL):
		buttonFL=int(gpio.input(7))
		counterFL+=1
		

def pivot_right1():
	counterBR=np.uint64(0)
	counterFL=np.uint64(0)

	buttonBR=int(0)
	buttonFL=int(0)

	pwm1=gpio.PWM(31,50) #left
	pwm2=gpio.PWM(35,50) #right
	val=22
	pwm1.start(85)
	pwm2.start(85)
	time.sleep(0.07)

	#print("Pivot Left","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))

	if int(gpio.input(12))!=int(buttonBR):
		buttonBR=int(gpio.input(12))
		counterBR+=1
		#print(counterBR)

	if int(gpio.input(7))!=int(buttonFL):
		buttonFL=int(gpio.input(7))
		counterFL+=1

		
def forward1(lim,ser):
	global curr_ang1
	counterBR=np.uint64(0)
	counterFL=np.uint64(0)
	print('forward after left')
	buttonBR=int(0)
	buttonFL=int(0)
	t=time.time()
	pwm1=gpio.PWM(31,50) #left
	pwm2=gpio.PWM(37,50) #right
	val=50
	pwm1.start(val)
	pwm2.start(val)
	time.sleep(0.1)
	
	while(True):
		#print("Forward","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))
		
		#curr_ang=float(line)	
		line=ser.readline()
		line=line.rstrip().lstrip()
		line=str(line)
		line=line.strip("'")
		line=line.strip("b'")
		curr_ang1=float(line)
		print('curr_ang',curr_ang1)
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
			
def pivot_left90(ser):
	count=0
	new_ang=0
	l=0
	while True:
						
		line=ser.readline()
		line=line.rstrip().lstrip()
		line=str(line)
		line=line.strip("'")
		line=line.strip("b'")
		new_ang=float(line)
		diff=curr_ang1-new_ang
		count1=0
		while 0<=(abs(curr_ang1-new_ang))<=90 or 270<=(abs(curr_ang1-new_ang))<=360:
			line=ser.readline()
			line=line.rstrip().lstrip()
			line=str(line)
			line=line.strip("'")
			line=line.strip("b'")
			new_ang=float(line)
			diff1=curr_ang1-new_ang
			pivot_left1()
			print(count1)
			print('angle diff',abs(curr_ang1-new_ang))
			print('left1')
	
		print('current angle',curr_ang)
		print('new angle',new_ang)
		
		time.sleep(0.9)
	
		print('forward 4')
		forward1(6,ser)
		break	

def pivot_right90(ser):
	count=0
	new_ang=0
	l=0
	while True:
						
		line=ser.readline()
		line=line.rstrip().lstrip()
		line=str(line)
		line=line.strip("'")
		line=line.strip("b'")
		new_ang=float(line)
		diff=curr_ang1-new_ang
		count1=0
		while 0<=(abs(curr_ang1-new_ang))<=90 or 270<=(abs(curr_ang1-new_ang))<=360:
			line=ser.readline()
			line=line.rstrip().lstrip()
			line=str(line)
			line=line.strip("'")
			line=line.strip("b'")
			new_ang=float(line)
			diff1=curr_ang1-new_ang
			pivot_right1()
			print(count1)
			print('angle diff',abs(curr_ang1-new_ang))
			print('left1')
	
		print('current angle',curr_ang)
		print('new angle',new_ang)
		
		time.sleep(0.9)
		break				

			

				
def forward():
	counterBR=np.uint64(0)
	counterFL=np.uint64(0)
	print('moving forward')
	buttonBR=int(0)
	buttonFL=int(0)
	pwm1=gpio.PWM(31,50) #left
	pwm2=gpio.PWM(37,50) #right
	val=70
	#pwm1.start(val)
	#pwm2.start(val)
	time.sleep(0.1)
	dist=30

	while(dist >13):

		#print("Forward","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))
		dist=front_distance(15,100)
		pwm1.start(val)
		pwm2.start(val)
		print('distance from obj',dist)
		while(True):
			#print('inside while')
			#print("Forward","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))				
			if int(gpio.input(12))!=int(buttonBR):
				buttonBR=int(gpio.input(12))
				counterBR+=1
				#print(counterBR)

			if int(gpio.input(7))!=int(buttonFL):
				buttonFL=int(gpio.input(7))
				counterFL+=1
				#print(counterFL)
			
			if counterBR>7 or counterFL>=7:
				print('in stopping')
				counterBR=0
				counterFL=0
				pwm1.stop()
				pwm2.stop()
				break	
	print('broken out of forward')
	pwm2.stop()
	pwm1.stop()
	pwm.ChangeDutyCycle(4.8)
	pic_email()

def pivot_left():
	print('left pivoting')
	counterBR=np.uint64(0)   
	counterFL=np.uint64(0)
	buttonBR=int(0)
	buttonFL=int(0)

	pwm1=gpio.PWM(33,50) #left
	pwm2=gpio.PWM(37,50) #right
	val=22
	pwm1.start(70)
	pwm2.start(70)
	time.sleep(0.02)

	#print("Pivot Left","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))

	if int(gpio.input(12))!=int(buttonBR):
		buttonBR=int(gpio.input(12))
		counterBR+=1
		#print(counterBR)
        #pwm2.stop()

	if int(gpio.input(7))!=int(buttonFL):
		buttonFL=int(gpio.input(7))
		counterFL+=1
		
    
def pivot_right():
	print('pivoting right')
	counterBR=np.uint64(0)
	counterFL=np.uint64(0)

	buttonBR=int(0)
	buttonFL=int(0)

	pwm1=gpio.PWM(31,50) #left
	pwm2=gpio.PWM(35,50) #right

	pwm1.start(85)
	pwm2.start(85)
	time.sleep(0.05)

	#print("Pivot Left","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))

	if int(gpio.input(12))!=int(buttonBR):
		buttonBR=int(gpio.input(12))
		counterBR+=1
		#print(counterBR)
        #pwm2.stop()
	if int(gpio.input(7))!=int(buttonFL):
		buttonFL=int(gpio.input(7))
		counterFL+=1
		
    
def imu_orient(angle):
    count=0
    count1=0
    new_ang=0
    count2=0
    #curr_ang1=359
    #curr_ang2=359
    print('IMU Orient')
    while True:
        if(ser.in_waiting > 0):
            line=ser.readline()
            count+=1
	    
            if count > 10:
            #read serial stream
                line=ser.readline()
                line=line.rstrip().lstrip()
                line=str(line)
                line=line.strip("'")
                line=line.strip("b'")
                curr_ang=float(line)
                print('init angle',curr_ang)
                new_ang=curr_ang
                print('angle track',new_ang)

                if -2.5<=angle<=2.5:
                    break
		    
                if angle>0:
                    print('angle>0')
                    while ((curr_ang<=new_ang<=(curr_ang+angle+4))):
                        print(curr_ang)
                        print(angle)			 
                        #print(curr_ang+((curr_ang-angle)%360)) 						    
                        print('in first while')   
                        '''
                        if math.isclose(new_ang,360,abs_tol=2) or  math.isclose(new_ang,0,abs_tol=2):
                                z=abs(360-curr_ang)
                                diff=angle-z
                                print(diff)
                                break
                        '''	
                        pivot_right()
                        line=ser.readline()
                        line=line.rstrip().lstrip()
                        line=str(line)
                        line=line.strip("'")
                        line=line.strip("b'")
                        new_ang=float(line)
                        print('new_ang',new_ang)
                        #print('close',math.isclose(new_ang,0,abs_tol=2))
                    ''' 	
                    line=ser.readline()
                    line=line.rstrip().lstrip()
                    line=str(line)
                    line=line.strip("'")
                    line=line.strip("b'")
                    new_ang=float(line)
		    '''
                    if curr_ang + angle>= 360:
                        print('360 here')
                        new_ang=0
                        angle_covered=360-curr_ang	
                        angle_left=angle-angle_covered		    
                        while 0<=new_ang<=angle_left:
                             print('in second while')				
                             pivot_right()
                             line=ser.readline()
                             line=line.rstrip().lstrip()
                             line=str(line)
                             line=line.strip("'")
                             line=line.strip("b'")
                             new_ang=float(line)
                             print('new_ang',new_ang)
			    
                        #print('close',math.isclose(new_ang,0,abs_tol=2))
                        '''
                        if math.isclose(new_ang,00,abs_tol=2):
                              curr_ang1=new_ang
                        else:
                              curr_ang1=0
                              print('curr_ang1',curr_ang)
                        '''
                        
                    forward()
                if angle<0:
                    print('angle<0')
                    print('diff',abs(curr_ang-new_ang),abs(curr_ang+angle))

                    while (curr_ang+angle)<=new_ang<=curr_ang+3:
                        '''			    
                        if math.isclose(new_ang,00,abs_tol=2) or math.isclose(new_ang,359,abs_tol=2):
                             z=abs(0-curr_ang)
                             diff1=abs(-(angle)-z)
                             print('diff1',diff1)
                             print('break')
                             break
                        '''			     
                        pivot_left()
                        line=ser.readline()
                        line=line.rstrip().lstrip()
                        line=str(line)
                        line=line.strip("'")
                        line=line.strip("b'")
                        new_ang=float(line)
                        print('new_ang',new_ang)
                        #print('cur_ang',curr_ang1)
                        #print('close',math.isclose(new_ang,360,abs_tol=2))

				     												
                    line=ser.readline()
                    line=line.rstrip().lstrip()
                    line=str(line)
                    line=line.strip("'")
                    line=line.strip("b'")
                    new_ang=float(line)
		    
                    if curr_ang + angle<= 0: 
                        new_ang=360
                        angle_covered=curr_ang	
                        angle_left=-angle-angle_covered			
                        print('angle<360')		    
                        while 360-angle_left<=new_ang<=360:
                             #time.sleep(1)
                             pivot_left()
                             line=ser.readline()
                             line=line.rstrip().lstrip()
                             line=str(line)
                             line=line.strip("'")
                             line=line.strip("b'")
                             new_ang=float(line)
                             print('new_ang',new_ang)
                    forward()			     

                break
		
#count2=0
def obj_tracker():
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
		print('in object tracking')
		#count2+=1
		# grab the current frame
		#frame=np.uint8(frame)
		#frame=cv2.flip(frame,-1)
		#start=datetime.datetime.now()
		#print('start',start)
		image = frame.array
		# show the frame to our screen
		#cv2.imshow("Frame", image)
	    
		rows,cols,chn=image.shape
	    
		blank_img=np.zeros(shape=[rows,cols,chn],dtype=np.uint8)
	
		#cv2.imshow('Image',image)
		#cv2.imshow('blank image',blank_img)
	
		hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		#cv2.imshow('hsv',hsv)
		'''
		lower_green = np.array([31,122,15])
		upper_green = np.array([59,255,120])
		
		lower_green = np.array([36,111,11])
		upper_green = np.array([55,255,217])
		'''
		lower_green = np.array([35,110,57])
		upper_green = np.array([48,200,180])
		
		blank_img = cv2.inRange(hsv,lower_green,upper_green)
		blank_img=cv2.erode(blank_img,None,iterations=3)
		blank_img=cv2.dilate(blank_img,None,iterations=3) 
		cv2.imshow('mask',blank_img)
	
		contours,hierarchy = cv2.findContours(blank_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		cx1=[]
		cy1=[]
		rad=[]
		#print('contours',len(contours))
		'''  
		for i in range(0,len(contours)):
		    ((x,y),radius)=cv2.minEnclosingCircle(contours[i])
		    M=cv2.moments(contours[i])
		    if M["m00"]!=0:
			#print('moments')
			cx=int(M["m10"]/M["m00"])
			cy=int(M["m01"]/M["m00"])
			cx1.append(cx)
			cy1.append(cy)
		    else:
			cx,cy=0,0
			cx1.append(cx)
			cy1.append(cy)
		    rad.append(radius)
		    center=(cx,cy)
		    #cv2.circle(image,center,int(radius),((0,255,255)),2)
		'''	    
	
		if len(contours)>0:
			c=max(contours,key=cv2.contourArea)
			((x,y),radius)=cv2.minEnclosingCircle(c)
			M=cv2.moments(c)
			if M["m00"]!=0:
			   #print('moments')
			   cx=int(M["m10"]/M["m00"])
			   cy=int(M["m01"]/M["m00"])
			#cx1.append(cx)
			#cy1.append(cy)
			else:
			   cx,cy=0,0
			center=(cx,cy)
			
			if radius > 20:
			   cv2.circle(image,center,int(radius),((0,255,255)),2)
			else:
			   radius=0
		else:
			continue
		#print(cx1,cy1,rad)
		#radius_fin=max(rad)
		#centre_x=int((min(cx1)+max(cx1))/2)
		#centre_y=int((min(cy1)+max(cy1))/2)
		#centre_x=max(cx1)
		#centre_y=max(cy1)
		#cv2.circle(image,(int(centre_x),int(centre_y)),int(radius),(255,25,255),1)
		
		cv2.imshow('Image1',image)
		if radius>20:
		     print('center',center[0],640-center[0])
		     angle_cen=(((center[0])-320))*0.061
		     print('angle',angle_cen)
		     imu_orient(angle_cen)
		else:
		     continue
		     
		key = cv2.waitKey(1) & 0xFF
	    
		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)
		# press the 'q' key to stop the video stream
		#count+=1
		if key == ord("q"):
		    pwm.ChangeDutyCycle(3)
		    time.sleep(2)
		    gameover()
		    gpio.cleanup()
		    pwm.stop
		    break
		#end=datetime.datetime.now()
		#print('end',end)
	    
		#now=end-start
		#print(now)
	
		#print(now.total_seconds())
		# write frame to video file
		out.write(image)

count3=0
while True:
	if(ser.in_waiting > 0):
		count3+=1
		#read serial stream
		line=ser.readline()
		
		if count3 > 10:	
			forward_obs(ser)
			time.sleep(0.9)
			pivot_left90(ser)
			time.sleep(0.9)
			pivot_right90(ser)
			#time.sleep(0.9)
			break			
obj_tracker()
gameover()
gpio.cleanup()			



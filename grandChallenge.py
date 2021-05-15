from picamera.array import PiRGBArray
from picamera import PiCamera
from datetime import datetime
from imutils.video import VideoStream
import RPi.GPIO as gpio
import numpy as np
import time
import math
import argparse
import imutils
import cv2
import serial
import os
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
import imaplib
import threading
camera=0
qrcount=0
pwm=0

def gameover():
	#Set all pins low
	gpio.output(31,False)
	gpio.output(33,False)
	gpio.output(35,False)
	gpio.output(36,False)
	gpio.output(37,False)
	gpio.cleanup()

ser=serial.Serial('/dev/ttyUSB0',9600)

def init():
	gpio.setmode(gpio.BOARD)
	gpio.setup(36,gpio.OUT)
	gpio.setup(31,gpio.OUT)
	gpio.setup(33,gpio.OUT)
	gpio.setup(35,gpio.OUT)
	gpio.setup(37,gpio.OUT)
	gpio.setup(7,gpio.IN,pull_up_down=gpio.PUD_UP)
	gpio.setup(12,gpio.IN,pull_up_down=gpio.PUD_UP)
	gpio.setup(16, gpio.OUT)
	gpio.setup(18, gpio.IN)
	
init()
gameover()


def distance_range():
	pulse_start=time.time()
	pulse_end=time.time()
		
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
	
def take_pic(data1):
	print('taking_pic')
	global camera
	camera.rotation=180
	camera.resolution = (640, 480)
	camera.framerate = 2
	
	time.sleep(2)
	pic_name='obj.jpg'
	image=np.empty((480,640,3),dtype=np.uint8)
	camera.capture(image,'bgr')
	
	rows,cols,chn=image.shape
	print(rows,cols,chn)
	blank_img=np.zeros(shape=[rows,cols,chn],dtype=np.uint8)

	hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

	#Green
	if data1=='MODERNA':
		low= np.array([63,76,43])
		up = np.array([88,216,121])
	
	#Red
	if data1=='PFIZER':
		low = np.array([139,65,51])
		up = np.array([255,255,255])

	#Blue	
	if data1=='J&J':
		#print('JJ')
		low = np.array([83,95,93])
		up= np.array([150,255,255])
	
	blank_img = cv2.inRange(hsv,low,up)
	blank_img=cv2.erode(blank_img,None,iterations=3)
	blank_img=cv2.dilate(blank_img,None,iterations=3) 

	contours,hierarchy = cv2.findContours(blank_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cx1=[]
	cy1=[]
	rad=[]	    

	if len(contours)>0:
                c=max(contours,key=cv2.contourArea)
                ((x,y),radius)=cv2.minEnclosingCircle(c)
                M=cv2.moments(c)
                if M["m00"]!=0:
                   cx=int(M["m10"]/M["m00"])
                   cy=int(M["m01"]/M["m00"])
                else:
                   cx,cy=0,0
                center=(cx,cy)
		
                if radius > 15:
                   cv2.circle(image,center,int(radius),((0,255,255)),2)
                   cv2.imshow('Closer',image)
                   angle_cen=(((center[0])-320))*0.061
                   imu_orient1(angle_cen,data1)
                print('radius',radius)
                return radius
	else:
                return 0
                
	out.write(image)
	key = cv2.waitKey(1) & 0xFF
	rawCapture.truncate()
	if key == ord("q"):
		pwm.ChangeDutyCycle(3)
		time.sleep(2)
		gameover()
		pwm.stop
		        
def front_distance(width,focal,data):
	print('calculating front distance')	
	pix=take_pic(data)
	if pix == 0:
		return 0
	else:
		return (focal*width/pix)

def pivot_left1():
	print('left pivoting')
	#init()
	counterBR=np.uint64(0)   
	counterFL=np.uint64(0)
	buttonBR=int(0)
	buttonFL=int(0)

	pwm1=gpio.PWM(33,50) #left
	pwm2=gpio.PWM(37,50) #right
	val=22
	pwm1.start(85)
	pwm2.start(85)
	time.sleep(0.05)

	if int(gpio.input(12))!=int(buttonBR):
		buttonBR=int(gpio.input(12))
		counterBR+=1
		#print(counterBR)

	if int(gpio.input(7))!=int(buttonFL):
		buttonFL=int(gpio.input(7))
		counterFL+=1
		#print(counterFL)
	
	pwm1.stop()
	pwm2.stop()
	
def pivot_right1():
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

	if int(gpio.input(12))!=int(buttonBR):
		buttonBR=int(gpio.input(12))
		counterBR+=1

	if int(gpio.input(7))!=int(buttonFL):
		buttonFL=int(gpio.input(7))
		counterFL+=1
	
	pwm1.stop()
	
def pivot_left():
	print('left pivoting')
	#init()
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

	if int(gpio.input(12))!=int(buttonBR):
		buttonBR=int(gpio.input(12))
		counterBR+=1
		#print(counterBR)

	if int(gpio.input(7))!=int(buttonFL):
		buttonFL=int(gpio.input(7))
		counterFL+=1
		#print(counterFL)
	
	pwm1.stop()
	pwm2.stop()
	
def pivot_right():
	print('pivoting right')

	counterBR=np.uint64(0)
	counterFL=np.uint64(0)

	buttonBR=int(0)
	buttonFL=int(0)

	pwm1=gpio.PWM(31,50) #left
	pwm2=gpio.PWM(35,50) #right

	pwm1.start(100)
	pwm2.start(100)
	time.sleep(0.1)

	if int(gpio.input(12))!=int(buttonBR):
		buttonBR=int(gpio.input(12))
		counterBR+=1

	if int(gpio.input(7))!=int(buttonFL):
		buttonFL=int(gpio.input(7))
		counterFL+=1
	
	pwm1.stop()

def forward(data):
	counterBR=np.uint64(0)
	counterFL=np.uint64(0)
	print('moving forward')
	buttonBR=int(0)
	buttonFL=int(0)

	val=42
	time.sleep(0.1)
	dist=1000

	
	pwm1=gpio.PWM(31,50) #left
	pwm2=gpio.PWM(37,50) #right
	while dist>=7.8:

		pwm1.start(val)
		pwm2.start(val)

		while(True):				
			if int(gpio.input(12))!=int(buttonBR):
				buttonBR=int(gpio.input(12))
				counterBR+=1

			if int(gpio.input(7))!=int(buttonFL):
				buttonFL=int(gpio.input(7))
				counterFL+=1
			
			if counterBR>1 and counterFL>=1:
				print('in stopping')
				counterBR=0
				counterFL=0
				pwm1.stop()
				pwm2.stop()			
				break
		dist=front_distance(3,340,data)
		print('distance from obj',dist)
		
	print('broken out of forward')
	pwm.ChangeDutyCycle(2.5)
	pwm2.stop()
	pwm1.stop()
	time.sleep(2)

def imu_orient1(angle,data):
	global ser
	global imuang
	count=0
	count1=0
	new_ang=0
	count2=0
	print('IMU Orient')

	curr_ang=imuang
	print('init angle',curr_ang,angle)
	new_ang=curr_ang
	print('angle updated',new_ang)

	if -2.5<=angle<=2:
	    print('middle')

	    
	if angle>2:
	    print('angle>0')
	    while ((curr_ang<=new_ang<=(curr_ang+angle))):
	    	print(curr_ang)
	    	print(angle)			 						    
	    	print('in first while')   
	    	try:
		        new_ang=imuang
	    	except ValueError:
		        new_ang=imuang
	    	print('new_ang',new_ang)
	    	pivot_right1()			

	    if curr_ang + angle>= 360:
	    	print('360 here')
	    	new_ang=0
	    	angle_covered=360-curr_ang	
	    	angle_left=angle-angle_covered		    
	    	while 0<=new_ang<=angle_left:
		     print('in second while')				
		     try:
		        new_ang=imuang
		     except ValueError:
		        new_ang=imuang

		     pivot_right1()	
					    
	    time.sleep(0.1) 
	    
	if angle<-2.5:
	    print('angle<0')
	    print('diff',abs(curr_ang-new_ang),abs(curr_ang+angle))

	    while (curr_ang+angle)<=new_ang<=curr_ang:		     
	    	try:
		        new_ang=imuang
	    	except ValueError:
		        new_ang=imuang
						
	    	pivot_left1()

	    	print('new_ang first while',new_ang)
															
	    new_ang=imuang
	    
	    if curr_ang + angle<= 0: 
	    	new_ang=360
	    	angle_covered=curr_ang	
	    	angle_left=-angle-angle_covered			
	    	print('angle<360')		    
	    	while 360-angle_left<=new_ang<=360:
		     try:
		     	new_ang=imuang
		     except ValueError:
		     	new_ang=imuang
   
		     pivot_left1()

	    	print('new_ang first while',new_ang)

	    time.sleep(0.1)
                
def imu_orient(angle,data):
	global ser
	global imuang
	count=0
	count1=0
	new_ang=0
	count2=0
	print('IMU Orient')
	curr_ang=imuang
	print('init angle',curr_ang)
	new_ang=curr_ang
	print('angle updated',new_ang)

	if -2.5<=angle<=2:
	    print('middle')
	    forward(data)

	    
	if angle>2:
	    print('angle>0')
	    while ((curr_ang<=new_ang<=(curr_ang+angle))):			 						    
	    	print('in first while')   
	    	try:
		        new_ang=imuang
	    	except ValueError:
		        new_ang=imuang
	    	print('new_ang',new_ang)
	    	pivot_right()			

	    if curr_ang + angle>= 360:
	    	print('360 here')
	    	new_ang=0
	    	angle_covered=360-curr_ang	
	    	angle_left=angle-angle_covered		    
	    	while 0<=new_ang<=angle_left:
		     print('in second while')				
		     try:
		       	new_ang=imuang
		     except ValueError:
		       	new_ang=imuang
		       	continue

		     pivot_right1()	
					    
	    time.sleep(0.1) 
	    forward(data)
	    
	if angle<-2.5:
	    print('angle<0')
	    print('diff',abs(curr_ang-new_ang),abs(curr_ang+angle))

	    while (curr_ang+angle)<=new_ang<=curr_ang:		     
	    	try:
		        new_ang=imuang
	    	except ValueError:
		        new_ang=imuang
						
	    	pivot_left1()

	    	print('new_ang first while',new_ang)

	    new_ang=curr_ang
	    
	    if curr_ang + angle<= 0: 
	    	new_ang=360
	    	angle_covered=curr_ang	
	    	angle_left=-angle-angle_covered			
	    	print('angle<360')		    
	    	while 360-angle_left<=new_ang<=360:
		     try:
		       	new_ang=imuang
		     except ValueError:
		       	new_ang=imuang

		     pivot_left1()

	    time.sleep(0.1)
	    forward(data)

def checkEmail():

	mail=imaplib.IMAP4_SSL('imap.gmail.com')
	mail.login('enpm809tjay@gmail.com','enpm809tjay')
	mail.list()

	count=0
	
	while count<300:
		try:
			#Connect to inbox
			mail.select("inbox")

			#search for an unread email from user's email address
			result,data=mail.search(None,'(UNSEEN FROM "jayeshjayashankar95@gmail.com")')
			
			print(result)
			print(len(data))

			ids=data[0]
			id_list=ids.split()

			latest_email_id=id_list[-1]
			result,data=mail.fetch(latest_email_id,"(RFC822)")

			if data is None:
				print('Waiting...')
			
			if data is not None:
				print('Process Initiated!')
				return 'y'
				
		except IndexError:
			time.sleep(2)
			if count < 299:
				count=count+1
				continue
			else:
				print('gameover')
				count=60

def pic_email(x):
	#Email information
	smtpUser = 'enpm809tjay@gmail.com'	
	smtpPass = 'enpm809tjay'
	pic_time=datetime.now().strftime('%Y%m%d%H%M%S')
	#Destination email info
	toAdd = 'jayeshjayashankar95@gmail.com'
	fromAdd = smtpUser
	subject = 'Grand Challenge Image' + pic_time
	msg=MIMEMultipart()
	msg['Subject']=subject
	msg['From']=fromAdd
	msg['To']=toAdd
	#msg['To']=",".join(toAdd)
	msg.preamble='Image recorded at' + pic_time

	#Email text
	body=MIMEText('Object Detected' + pic_time)
	msg.attach(body)
	print('adding body')
	#Attach image
	print('opening')
	fp=open(x,'rb')
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
	#rawCapture.truncate()

	print("Email delivered")

def QRdetect(file):
	global ser
	global qrcount
	count=0
	global camera
	time.sleep(3)
	if qrcount==0:
		camera = PiCamera()
		qrcount+=1
	else:
		camera.close()
		camera = PiCamera()
		
	camera.rotation=180
	camera.resolution = (640, 480)
	camera.framerate = 2
	rawCapture = PiRGBArray(camera, size=(640,480))

	'''
	command = 'sudo modprobe bcm2835-v4l2'
	os.system(command)
	global qrcount
	#open video capture(0)
	cap=cv2.VideoCapture(0)
	cap.set(cv2.CAP_PROP_BUFFERSIZE,3)
	'''
	#define detector
	detector = cv2.QRCodeDetector()
	
	while True:
		'''
		check, img = cap.read()
		img=cv2.flip(img,-1)
		'''
		img=np.empty((480,640,3),dtype=np.uint8)
		camera.capture(img,'bgr')
		
		data, bbox, _ = detector.detectAndDecode(img)

		if(bbox is not None):
			for i in range(len(bbox)):
				cv2.line(img, tuple(bbox[i][0]), tuple(bbox[(i+1) % len(bbox)][0]),color = (0,0,255), thickness = 4)
				cv2.putText(img,data,(int(bbox[0][0][0]), int(bbox[0][0][1])-10), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)
				cv2.imwrite(file,img)
		if data:
			print("Data:",data)
			break
			
		#Show result to the screen
		cv2.imshow("QR code detector",img)

		#Break out of loop by pressing the q key
		if(cv2.waitKey(1) == ord("q")):
			break
			
	return data
	rawCapture.truncate(0)
	camera.close()
	cv2.destroyAllWindows()
	time.sleep(2)
	
def obj_tracker(data1):

	global camera
	camera.close()
	time.sleep(3)
	camera = PiCamera()
	
	#global camera
	camera.rotation=180
	camera.resolution = (640, 480)
	camera.framerate = 2

	rawCapture = PiRGBArray(camera, size=(640,480))
	# clear the stream in preparation for the next frame
	#rawCapture.truncate(0)
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
		print('in object tracking')
		
		image = frame.array

		rows,cols,chn=image.shape
	    
		blank_img=np.zeros(shape=[rows,cols,chn],dtype=np.uint8)
	
		hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		
		#Green
		if data1=='MODERNA':
			low= np.array([63,76,43])
			up = np.array([88,216,121])
		
		#Red
		if data1=='PFIZER':
			low = np.array([139,65,51])
			up = np.array([255,255,255])

		#Blue	
		if data1=='J&J':
			#print('JJ')
			low = np.array([83,95,93])
			up= np.array([150,255,255])
			
		blank_img = cv2.inRange(hsv,low,up)
		blank_img=cv2.erode(blank_img,None,iterations=3)
		blank_img=cv2.dilate(blank_img,None,iterations=3) 
		cv2.imshow('mask',blank_img)
	
		contours,hierarchy = cv2.findContours(blank_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		cx1=[]
		cy1=[]
		rad=[]	    
		print('len',len(contours))
		key = cv2.waitKey(1) & 0xFF
		if len(contours)>0:
			c=max(contours,key=cv2.contourArea)
			((x,y),radius)=cv2.minEnclosingCircle(c)
			M=cv2.moments(c)
			if M["m00"]!=0:
			   cx=int(M["m10"]/M["m00"])
			   cy=int(M["m01"]/M["m00"])
			else:
			   cx,cy=0,0
			center=(cx,cy)
			
			if radius > 20:
			   cv2.circle(image,center,int(radius),((0,255,255)),2)
			else:
			   radius=0
		else:
			rawCapture.truncate(0)
			print('continuing')
			if key == ord("q"):
				pwm.ChangeDutyCycle(3)
				time.sleep(2)
				gameover()
				pwm.stop()
				break
			continue
		
		cv2.imshow('Image1',image)
		if key == ord("q"):
			#pwm.ChangeDutyCycle(3)
			time.sleep(2)
			gameover()
			#gpio.cleanup()
			pwm.stop()
			break
		if radius>20:
		     print('center',center[0],640-center[0])
		     angle_cen=(((center[0])-320))*0.061
		     print('angle',angle_cen)
		     imu_orient(angle_cen,data1)
		     break
		else:
		     rawCapture.truncate(0)
		     if key == ord("q"):
		        pwm.ChangeDutyCycle(3)
		        time.sleep(2)
		        gameover()
		        pwm.stop()
		        break
		     

		print('truncating')
		rawCapture.truncate(0)
		camera.close()		

def IMUang():
	global ser
	global imuang
	count=0
	while True:
		if(ser.in_waiting > 0):
			line=ser.readline()
			count+=1
	    
			if count > 10:
				line=ser.readline()
				line=line.rstrip().lstrip()
				line=str(line)
				line=line.strip("'")
				line=line.strip("b'")
				imuang=float(line)
				
def reverse(lim):
	count=0
	global ser
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
	print('reverse')
	while(True):

		#print("Reverse","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))

		if int(gpio.input(12))!=int(buttonBR):
			buttonBR=int(gpio.input(12))
			counterBR+=1
			#print(counterBR)

		if int(gpio.input(7))!=int(buttonFL):
			buttonFL=int(gpio.input(7))
			counterFL+=1
			#print(counterFL)
		

		if counterBR>=lim and counterFL>=lim:
			#gameover()
			pwm1.stop()
			pwm2.stop()
			#print("Thanks for playing")
			break

def detectFace():
	global camera
	camera.close()
	time.sleep(3)
	camera = PiCamera()
	camera.rotation=180
	camera.resolution = (640, 480)
	camera.framerate = 2
	rawCapture = PiRGBArray(camera, size=(640,480))
	
	# construct the argument parse and parse the arguments
	ap = argparse.ArgumentParser()
	ap.add_argument("-p", "--prototxt", required=True,
		help="path to Caffe 'deploy' prototxt file")
	ap.add_argument("-m", "--model", required=True,
		help="path to Caffe pre-trained model")
	ap.add_argument("-c", "--confidence", type=float, default=0.5,
		help="minimum probability to filter weak detections")
	args = vars(ap.parse_args())

	# load our serialized model from disk
	print("[INFO] loading model...")
	net = cv2.dnn.readNetFromCaffe(args["prototxt"], args["model"])
	'''
	# initialize the video stream and allow the cammera sensor to warmup
	print("[INFO] starting video stream...")
	vs=VideoStream(src=0).start()
	time.sleep(2.0)
	'''
	pic_name='face1.jpg'
	while True:
		img=np.empty((480,640,3),dtype=np.uint8)
		camera.capture(img,'bgr')
		# loop over the frames from the video stream

		# grab the frame from the threaded video stream and resize it
		# to have a maximum width of 400 pixels
		#ret,frame = vs.read()
		#dx,dy,chn=frame.shape
		frame = imutils.resize(img, width=400)
		cv2.imwrite('face1.jpg',frame)
		#cv2.imshow('frame',frame)
		# grab the frame dimensions and convert it to a blob
		(h, w) = frame.shape[:2]
		blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0,
			(300, 300), (104.0, 177.0, 123.0))
	 
		# pass the blob through the network and obtain the detections and
		# predictions
		net.setInput(blob)
		detections = net.forward()

		# loop over the detections
		for i in range(0, detections.shape[2]):
			# extract the confidence (i.e., probability) associated with the
			# prediction
			confidence = detections[0, 0, i, 2]

			# filter out weak detections by ensuring the `confidence` is
			# greater than the minimum confidence
			if confidence < args["confidence"]:
				continue

			# compute the (x, y)-coordinates of the bounding box for the
			# object
			box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
			(startX, startY, endX, endY) = box.astype("int")
			
			if startX >0 and startY>0 and endX>0 and endY>0:
				flag='y'				
				# draw the bounding box of the face along with the associated
				# probability
				text = "{:.2f}%".format(confidence * 100)
				y = startY - 10 if startY - 10 > 10 else startY + 10
				cv2.rectangle(frame, (startX, startY), (endX, endY),
					(0, 255, 0), 2)
				cv2.putText(frame, text, (startX, y),
					cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
				img=cv2.flip(frame,-1)
				if flag=='y':
					cv2.imwrite('face.jpg',img)
					return flag
				else:
					continue
				
		# show the output frame
		cv2.imshow("Frame", cv2.flip(frame,-1))
		rawCapture.truncate(0)
		cv2.destroyAllWindows()
	
def arrowDetect():
	global camera
	#time.sleep(3)
	camera.close()
	time.sleep(3)
	camera = PiCamera()
	camera.rotation=180
	camera.resolution = (640, 480)
	camera.framerate = 2
	rawCapture = PiRGBArray(camera, size=(640,480))
	time.sleep(2)
	pic_name='arr1.jpg'
	#command='raspistill -w 640 -h 480 -vf -hf -o ' + pic_name
	#os.system(command)
	img=np.empty((480,640,3),dtype=np.uint8)
	camera.capture(img,'bgr')
	
	#camera.close()	
	#image=cv2.imread(output)
	rows,cols,chn=img.shape
	print(rows,cols,chn)
	blank_img=np.zeros(shape=[rows,cols,chn],dtype=np.uint8)

	hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        #cv2.imshow('hsv',hsv)
	
	low = np.array([0,95,126])
	up= np.array([78,255,255])
	
	mask = cv2.inRange(hsv,low,up) 	
    #cv2.imshow('mask',mask)

	mask_stack=np.stack((mask,)*3,axis=-1)
	gblur=cv2.GaussianBlur(mask_stack,(5,5),0)
	gblur1=cv2.cvtColor(gblur,cv2.COLOR_BGR2GRAY)
    #cv2.imshow('gblur',gblur1))

	contours,hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
   

	cv2.drawContours(img,contours,-1,(36,255,12),2)
	corners=cv2.goodFeaturesToTrack(gblur1,7,0.04,10)
	if corners is None:
		rawCapture.truncate(0)
		print('nothing')
    		#continue
    
	else:
		corners=np.int0(corners)
		for i in corners:
			x,y=i.ravel()

			cv2.circle(img,(x,y),3,255,-1)

		xmax,ymax=(np.max(corners,axis=0)).ravel()
		xmin,ymin=(np.min(corners,axis=0)).ravel()
    	#print('xmax',xmax,'ymax',ymax,'xmin',xmin,'ymin',ymin)
    	#print('corners',corners[:,0,0])
    	#print('corners1',corners[:,0,1])
		font=cv2.FONT_HERSHEY_SIMPLEX
		if(abs(xmax-xmin)>abs(ymax-ymin)):
			a=corners[:,0,0]
			count=0
			for i in range(0,len(a)):
				if math.isclose(a[i],xmax,abs_tol=5):
					count+=1 
			if count>=2:
				cv2.putText(img,'left',(350,300),font,1,(255,155,55),2,cv2.LINE_AA)
				#cv2.putText(img,str(count),(450,300),font,1,(255,155,55),2,cv2.LINE_AA)
				print('left')
				dir='left'
			else:
				cv2.putText(img,'right',(350,300),font,1,(255,155,55),2,cv2.LINE_AA)
				#cv2.putText(img,str(count),(450,300),font,1,(255,155,55),2,cv2.LINE_AA)
				cv2.imwrite('arright.jpg',img)
				print('right')
				
				dir='right'
					
		else:
			b=corners[:,0,1]
			count1=0
			for i in range(0,len(b)):
				if math.isclose(b[i],ymax,abs_tol=5):
					count1+=1
			if count1>=2:
				cv2.putText(img,'up',(350,300),font,1,(255,155,55),2,cv2.LINE_AA)
				#cv2.putText(img,str(count1),(450,300),font,1,(255,155,55),2,cv2.LINE_AA)    			    			
				print('up')
				dir='up'
			else:
				cv2.putText(img,'down',(350,300),font,1,(255,155,55),2,cv2.LINE_AA)
    			#cv2.putText(img,str(count1),(450,300),font,1,(255,155,55),2,cv2.LINE_AA)	
				print('down') 
				dir='down' 
	
	cv2.imshow('Image1',img)
	rawCapture.truncate(0)
	camera.close()
	cv2.destroyAllWindows()		
	return dir
	

def pivot90():
	global imuang
	curr_ang=imuang
	new_ang=curr_ang
	print('curr_ang',curr_ang)
	while 0<=abs(curr_ang-new_ang)<=80 or 280<=abs(curr_ang-new_ang)<=360:
		pivot_right()
		#time.sleep(0.1)
		print(new_ang,'90')
		new_ang=imuang

def pivot90l():
	global imuang
	curr_ang=imuang
	new_ang=curr_ang
	print('curr_ang',curr_ang)
	while 0<=abs(curr_ang-new_ang)<=90 or 270<=abs(curr_ang-new_ang)<=360:
		pivot_left()
		print(new_ang,'90')
		new_ang=imuang
		
def pivot120():
	global imuang
	curr_ang=imuang
	new_ang=curr_ang
	print('curr_ang',curr_ang)
	while 0<=abs(curr_ang-new_ang)<=120 or 240<=abs(curr_ang-new_ang)<=360:
		pivot_right()
		print(new_ang,'90')
		new_ang=imuang

def pivot60():
	global imuang
	curr_ang=imuang
	new_ang=curr_ang
	print('curr_ang',curr_ang)
	while 0<=abs(curr_ang-new_ang)<=57 or 303<=abs(curr_ang-new_ang)<=360:
		pivot_right()
		print(new_ang,'90')
		new_ang=imuang

def pivotx():
	global imuang
	curr_ang=imuang
	new_ang=curr_ang
	print('curr_ang',curr_ang)
	while 0<=abs(curr_ang-new_ang)<=curr_ang:
		pivot_left()
		#time.sleep(0.1)
		print(new_ang,'x')
		new_ang=imuang
					
def reverseOrient():
	global ser
	global imuang
	curr_ang=imuang
	print('current angle',curr_ang)
	new_ang=curr_ang
	count1=0
	if curr_ang>=0 and curr_ang<=90:
		while curr_ang<=new_ang<=77:
			new_ang=imuang
			pivot_right()
			#stime.sleep(0.04)
			print('diff',curr_ang-new_ang)
			print('new angle',new_ang)
	if curr_ang>=270 and curr_ang<=360:
		while curr_ang<=new_ang<=360:
			pivot_right()
			print('angle>270')
			new_ang=imuang
			print('new ang 270',new_ang)
			left_over=abs(360-new_ang)
			if math.isclose(new_ang,00,abs_tol=2) or math.isclose(new_ang,360,abs_tol=2):
			     print('breaking')
			     new_ang=0
			     break
                             
		while 0<=new_ang<=85:
			pivot_right()
			new_ang=imuang
			if math.isclose(new_ang,00,abs_tol=2) or math.isclose(new_ang,360,abs_tol=2):
			     print('breaking')
			     new_ang=0
			print(new_ang)

def forward5(lim,kp):

	counterBR=np.uint64(0)
	counterFL=np.uint64(0)
	print('moving forward')
	buttonBR=int(0)
	buttonFL=int(0)
	pwm1=gpio.PWM(31,50) #left
	pwm2=gpio.PWM(37,50) #right
	val=40
	stop=0
	pwm1.start(val)
	pwm2.start(val)
	time.sleep(0.1)
	curr_ang=imuang
	while(True):
		counterBR1=counterBR
		counterFL1=counterFL

		print("Forward","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))
		
		dist=distance_range()
		
		while dist<=20:
			pwm1.stop()
			pwm2.stop()
			dist=distance_range()
			print('stopping')
			stop=1
		
		#dist=distance_range()
		if dist>20:
			if stop==1:
				stop=0
				pwm1.start(val)
				pwm2.start(val)
				print('restarting')
				
			if int(gpio.input(12))!=int(buttonBR):
				buttonBR=int(gpio.input(12))
				counterBR+=1
				print('counter BR',counterBR)

			if int(gpio.input(7))!=int(buttonFL):
				buttonFL=int(gpio.input(7))
				counterFL+=1
				print('counter FL',counterFL)
					
			if (counterBR>=lim or counterFL>=lim):
				#gameover()
				print("Thanks for playing break 20")
				break
				
			new_ang=imuang
			if curr_ang>=350:
				curr_ang=0
				
			if new_ang>=350:
				new_ang-=360
			
			if (new_ang-curr_ang)>0:
				pwm1.stop()
				pwm2.ChangeDutyCycle(val+((new_ang-curr_ang)*kp))
				time.sleep(0.02)
				pwm1.start(val)
				pwm2.ChangeDutyCycle(val)
				time.sleep(0.02)
				
			if (new_ang-curr_ang)<0:
				pwm2.stop()
				if 0<=(val-((new_ang-curr_ang)*kp))<=100:
					pwm1.ChangeDutyCycle(val-((new_ang-curr_ang)*kp))
				time.sleep(0.02)
				pwm2.start(val)
				pwm1.ChangeDutyCycle(val)
				time.sleep(0.02)
								

t=threading.Thread(target=IMUang)
t.start()
start=checkEmail()
if start=='y':		
	for i in range(0,3):
		init()
		pwm=gpio.PWM(36,50)
		pwm.start(7)
		
		data=QRdetect('vacname.jpg')
		pic_email('vacname.jpg')
		#data='J&J'
		obj_tracker(data)
		reverse(12)
		time.sleep(1)
		reverseOrient()
		time.sleep(1)
		
		dir=QRdetect('qrright.jpg')
		pic_email('qrright.jpg')
		if dir=='right':
			pivot90()
			time.sleep(2)
			
		forward5(190,4)
		time.sleep(1)
			
		print('imuang',imuang)
		#pivotx()
		#time.sleep(1)
		dir=arrowDetect()
		if dir=="right":
			pic_email('arright.jpg')
			pivot90()
			time.sleep(2)
			forward5(214,8)
		
		time.sleep(2)
		x=detectFace()
		if x=='y':
			#pic_email('face.jpg')
			pwm.ChangeDutyCycle(7)
			reverse(8)
		time.sleep(2)
		pivot120()
		time.sleep(2)
		forward5(80,4)
		time.sleep(2)
		pivot60()
		time.sleep(2)
		forward5(186,8)
		time.sleep(2)
		pivot90l()
		time.sleep(2)
		forward5(150,8)
		time.sleep(2)

		gameover()
		pwm=0
